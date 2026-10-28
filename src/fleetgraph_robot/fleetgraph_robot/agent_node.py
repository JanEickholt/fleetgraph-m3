"""FleetGraph robot agent (Module 3): receives TaskAssignment / TaskCommand,
runs the TaskMachine, executes legs through Nav2 NavigateToPose, publishes
TaskUpdate, RobotState (heartbeat) and RobotConnection.

  ros2 run fleetgraph_robot agent --ros-args -p robot_id:=robot_1 -p use_sim_time:=true

Pose comes from TF map -> base_footprint (AMCL), never from Gazebo ground
truth. Battery is a simple model (distance + idle drain) until a real
battery source exists.
"""
import math

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from fleetgraph_interfaces.msg import (
    ErrorInfo, RobotConnection, RobotState, TaskAssignment, TaskCommand, TaskUpdate)
from fleetgraph_robot import contract as c
from fleetgraph_robot.task_machine import (
    CancelNavigation, E, Navigate, TaskMachine, Update, Wait)


class Agent(Node):
    def __init__(self):
        super().__init__('fleetgraph_agent')
        default_locations = get_package_share_directory('fleetgraph_sim') + \
            '/config/locations.yaml'
        p = self.declare_parameters('', [
            ('robot_id', 'robot_1'),
            ('capabilities', ['navigate', 'deliver', 'patrol', 'lidar', 'camera']),
            ('locations_file', default_locations),
            ('state_rate_hz', 2.0),
            ('feedback_period_sec', 1.0),     # throttle of progress TaskUpdates
            ('min_battery', 0.15),
            ('dwell_sec', 2.0),
            ('max_leg_retries', 1),
            ('battery_initial', 1.0),
            ('battery_drain_per_m', 0.004),
            ('battery_drain_per_s', 0.0001),
            ('maintenance', False),
            ('map_frame', 'map'),
            ('base_frame', 'base_footprint'),
            ('nav_action', 'navigate_to_pose'),
            ('nav_server_timeout_sec', 30.0),
        ])
        prm = {x.name: x.value for x in p}
        self.robot_id = prm['robot_id']
        self.prm = prm
        self.locations = c.load_locations(prm['locations_file'])
        self.machine = TaskMachine(
            capabilities=prm['capabilities'], resolve=self.locations.get,
            min_battery=prm['min_battery'], dwell_sec=prm['dwell_sec'],
            max_leg_retries=prm['max_leg_retries'])
        self.meta = c.MetaStamper(self.robot_id, self.get_clock())

        self.pub_update = self.create_publisher(TaskUpdate, c.TOPIC_TASK_UPDATE, c.QOS_EVENTS)
        self.pub_state = self.create_publisher(RobotState, c.TOPIC_ROBOT_STATE, c.QOS_TELEMETRY)
        self.pub_conn = self.create_publisher(
            RobotConnection, c.TOPIC_ROBOT_CONNECTION, c.QOS_LATCHED)
        self.create_subscription(TaskAssignment, c.TOPIC_TASK_ASSIGNMENT,
                                 self._on_assignment, c.QOS_COMMANDS)
        self.create_subscription(TaskCommand, c.TOPIC_TASK_COMMAND,
                                 self._on_command, c.QOS_COMMANDS)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 self._on_amcl, 10)

        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.nav = ActionClient(self, NavigateToPose, prm['nav_action'])
        self._goal_handle = None
        self._dwell_timer = None
        self._last_feedback_t = None

        self.battery = float(prm['battery_initial'])
        self._last_odom_xy = None
        self._last_drain_t = None
        self._velocity = None
        self._loc_score = -1.0
        self._pose = None

        self.create_timer(1.0 / prm['state_rate_hz'], self._publish_state)
        self._publish_connection(RobotConnection.STATE_ONLINE)
        self.get_logger().info(
            f'{self.robot_id} ready: {len(self.locations)} locations, '
            f'capabilities {prm["capabilities"]}')

    # ------------------------------------------------------------- inputs
    def _on_assignment(self, msg):
        if msg.robot_id != self.robot_id:
            return
        spec = c.spec_from_assignment(msg)
        available = self._status() not in (RobotState.STATUS_MAINTENANCE,
                                           RobotState.STATUS_ERROR)
        self.get_logger().info(
            f'assignment {spec.task_id} rev {spec.assignment_rev} type {spec.type}')
        self._dispatch(self.machine.assign(spec, self.battery, available))

    def _on_command(self, msg):
        if msg.robot_id != self.robot_id:
            return
        if msg.command == TaskCommand.COMMAND_CANCEL:
            self.get_logger().info(f'cancel {msg.task_id}: {msg.reason}')
            self._dispatch(self.machine.cancel(msg.task_id, msg.reason or 'cancelled'))

    def _on_odom(self, msg):
        self._velocity = msg.twist.twist
        p = msg.pose.pose.position
        if self._last_odom_xy is not None:
            moved = math.hypot(p.x - self._last_odom_xy[0], p.y - self._last_odom_xy[1])
            self.battery -= moved * self.prm['battery_drain_per_m']
        self._last_odom_xy = (p.x, p.y)

    def _on_amcl(self, msg):
        cov = msg.pose.covariance
        std_xy = math.sqrt(max(cov[0], 0.0) + max(cov[7], 0.0))
        # heuristic: 1.0 at 0 m std, 0.5 at 0.25 m std
        self._loc_score = 1.0 / (1.0 + std_xy / 0.25)

    # ----------------------------------------------------------- commands
    def _dispatch(self, outputs):
        for o in outputs:
            if isinstance(o, Update):
                self._publish_update(o)
            elif isinstance(o, Navigate):
                self._navigate(o)
            elif isinstance(o, CancelNavigation):
                self._cancel_navigation()
            elif isinstance(o, Wait):
                self._start_dwell(o.seconds)

    def _publish_update(self, u):
        msg = c.update_to_msg(u, self.robot_id)
        self.pub_update.publish(self.meta.stamp(msg, c.TOPIC_TASK_UPDATE))
        if u.state != TaskUpdate.STATE_EXECUTING or u.reason:
            self.get_logger().info(
                f'task {u.task_id}: state {u.state} code {u.error_code} {u.reason}')

    def _navigate(self, cmd, waited=0.0):
        gen = cmd.generation
        if gen != self.machine.nav_generation:
            return                                  # superseded while waiting
        if not self.nav.server_is_ready():
            limit = self.prm['nav_server_timeout_sec']
            if waited >= limit:
                self.get_logger().warn(f'Nav2 not available after {limit:.0f}s')
                self._dispatch(self.machine.nav_result(gen, False, 'nav2 not available'))
                return
            # Nav2 activates its lifecycle nodes after start: poll (sim time)
            timer = None

            def retry():
                timer.cancel()
                self._navigate(cmd, waited + 0.5)
            timer = self.create_timer(0.5, retry)
            return
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.prm['map_frame']
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = cmd.goal.x
        goal.pose.pose.position.y = cmd.goal.y
        qx, qy, qz, qw = c.quaternion_from_yaw(cmd.goal.yaw)
        goal.pose.pose.orientation.x, goal.pose.pose.orientation.y = qx, qy
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = qz, qw
        self.get_logger().info(
            f'leg {cmd.leg_index} -> {cmd.goal.location_id} '
            f'({cmd.goal.x:.2f}, {cmd.goal.y:.2f})')
        future = self.nav.send_goal_async(
            goal, feedback_callback=lambda fb, g=gen: self._on_feedback(g, fb))
        future.add_done_callback(lambda f, g=gen: self._on_goal_response(g, f))

    def _on_goal_response(self, gen, future):
        handle = future.result()
        if not handle.accepted:
            self._dispatch(self.machine.nav_result(gen, False, 'goal rejected by Nav2'))
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(
            lambda f, g=gen: self._on_result(g, f))

    def _on_feedback(self, gen, fb):
        if gen != self.machine.nav_generation:
            return
        now = self.get_clock().now()
        period = Duration(seconds=self.prm['feedback_period_sec'])
        if self._last_feedback_t is not None and now - self._last_feedback_t < period:
            return
        self._last_feedback_t = now
        self._dispatch(self.machine.nav_feedback(fb.feedback.distance_remaining))

    def _on_result(self, gen, future):
        res = future.result()
        ok = res.status == GoalStatus.STATUS_SUCCEEDED
        reason = '' if ok else (getattr(res.result, 'error_msg', '') or
                                f'nav2 status {res.status}')
        self._dispatch(self.machine.nav_result(gen, ok, reason))

    def _cancel_navigation(self):
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None
        if self._dwell_timer is not None:
            self._dwell_timer.cancel()
            self._dwell_timer = None

    def _start_dwell(self, seconds):
        def done():
            self._dwell_timer.cancel()
            self._dwell_timer = None
            self._dispatch(self.machine.dwell_done())
        self._dwell_timer = self.create_timer(seconds, done)

    # ---------------------------------------------------------- telemetry
    def _status(self):
        # live parameter: `ros2 param set /fleetgraph_agent maintenance true`
        if self.get_parameter('maintenance').value:
            return RobotState.STATUS_MAINTENANCE
        if self.machine.active_task_id:
            return RobotState.STATUS_BUSY
        return RobotState.STATUS_AVAILABLE

    def _errors(self):
        errs = []
        if self.battery < self.prm['min_battery']:
            errs.append(ErrorInfo(code=E.LOW_BATTERY, level=ErrorInfo.LEVEL_WARNING,
                                  description=f'battery {self.battery:.2f}'))
        if self._pose is None:
            errs.append(ErrorInfo(code=0, level=ErrorInfo.LEVEL_WARNING,
                                  description='not localized (no map->base TF)'))
        return errs

    def _publish_state(self):
        now = self.get_clock().now()
        if self._last_drain_t is not None:
            dt = (now - self._last_drain_t).nanoseconds * 1e-9
            self.battery -= dt * self.prm['battery_drain_per_s']
        self._last_drain_t = now
        self.battery = max(0.0, self.battery)
        try:
            tf = self.tf.lookup_transform(self.prm['map_frame'], self.prm['base_frame'],
                                          Time())
            pose = PoseStamped()
            pose.header = tf.header
            pose.pose.position.x = tf.transform.translation.x
            pose.pose.position.y = tf.transform.translation.y
            pose.pose.position.z = tf.transform.translation.z
            pose.pose.orientation = tf.transform.rotation
            self._pose = pose
        except TransformException:
            self._pose = None
        msg = RobotState(robot_id=self.robot_id, battery=float(self.battery),
                         charging=False, status=self._status(),
                         capabilities=list(self.prm['capabilities']),
                         current_task_id=self.machine.active_task_id,
                         connection_quality=-1.0,
                         localization_score=float(self._loc_score),
                         errors=self._errors())
        if self._pose is not None:
            msg.pose = self._pose
        if self._velocity is not None:
            msg.velocity = self._velocity
        self.pub_state.publish(self.meta.stamp(msg, c.TOPIC_ROBOT_STATE))

    def _publish_connection(self, state):
        msg = RobotConnection(robot_id=self.robot_id, state=state)
        self.pub_conn.publish(self.meta.stamp(msg, c.TOPIC_ROBOT_CONNECTION))


def main():
    rclpy.init()
    node = Agent()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node._publish_connection(RobotConnection.STATE_OFFLINE)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
