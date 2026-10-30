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
from fleetgraph_robot import contract
from fleetgraph_robot.task_machine import (
    CancelNavigation, ErrorCode, Navigate, TaskMachine, Update, Wait)

NAV_SERVER_POLL_SEC = 0.5


class Agent(Node):
    def __init__(self):
        super().__init__('fleetgraph_agent')
        default_locations = get_package_share_directory('fleetgraph_sim') + \
            '/config/locations.yaml'
        declared = self.declare_parameters('', [
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
        self.params = {parameter.name: parameter.value for parameter in declared}
        self.robot_id = self.params['robot_id']
        self.locations = contract.load_locations(self.params['locations_file'])
        self.machine = TaskMachine(
            capabilities=self.params['capabilities'], resolve=self.locations.get,
            min_battery=self.params['min_battery'], dwell_sec=self.params['dwell_sec'],
            max_leg_retries=self.params['max_leg_retries'])
        self.meta = contract.MetaStamper(self.robot_id, self.get_clock())

        self.pub_update = self.create_publisher(
            TaskUpdate, contract.TOPIC_TASK_UPDATE, contract.QOS_EVENTS)
        self.pub_state = self.create_publisher(
            RobotState, contract.TOPIC_ROBOT_STATE, contract.QOS_TELEMETRY)
        self.pub_connection = self.create_publisher(
            RobotConnection, contract.TOPIC_ROBOT_CONNECTION, contract.QOS_LATCHED)
        self.create_subscription(TaskAssignment, contract.TOPIC_TASK_ASSIGNMENT,
                                 self._on_assignment, contract.QOS_COMMANDS)
        self.create_subscription(TaskCommand, contract.TOPIC_TASK_COMMAND,
                                 self._on_command, contract.QOS_COMMANDS)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 self._on_amcl_pose, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, self.params['nav_action'])
        self._goal_handle = None
        self._dwell_timer = None
        self._last_feedback_time = None

        self.battery = float(self.params['battery_initial'])
        self._last_odom_xy = None
        self._last_drain_time = None
        self._velocity = None
        self._localization_score = -1.0
        self._pose = None

        self.create_timer(1.0 / self.params['state_rate_hz'], self._publish_state)
        self._publish_connection(RobotConnection.STATE_ONLINE)
        self.get_logger().info(
            f'{self.robot_id} ready: {len(self.locations)} locations, '
            f'capabilities {self.params["capabilities"]}')

    # ------------------------------------------------------------- inputs
    def _on_assignment(self, msg):
        if msg.robot_id != self.robot_id:
            return
        spec = contract.spec_from_assignment(msg)
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
        position = msg.pose.pose.position
        if self._last_odom_xy is not None:
            last_x, last_y = self._last_odom_xy
            moved = math.hypot(position.x - last_x, position.y - last_y)
            self.battery -= moved * self.params['battery_drain_per_m']
        self._last_odom_xy = (position.x, position.y)

    def _on_amcl_pose(self, msg):
        covariance = msg.pose.covariance
        variance_x, variance_y = covariance[0], covariance[7]
        std_xy = math.sqrt(max(variance_x, 0.0) + max(variance_y, 0.0))
        # heuristic: 1.0 at 0 m std, 0.5 at 0.25 m std
        self._localization_score = 1.0 / (1.0 + std_xy / 0.25)

    # ----------------------------------------------------------- commands
    def _dispatch(self, outputs):
        for output in outputs:
            if isinstance(output, Update):
                self._publish_update(output)
            elif isinstance(output, Navigate):
                self._navigate(output)
            elif isinstance(output, CancelNavigation):
                self._cancel_navigation()
            elif isinstance(output, Wait):
                self._start_dwell(output.seconds)

    def _publish_update(self, update):
        msg = contract.update_to_msg(update, self.robot_id)
        self.pub_update.publish(self.meta.stamp(msg, contract.TOPIC_TASK_UPDATE))
        if update.state != TaskUpdate.STATE_EXECUTING or update.reason:
            self.get_logger().info(
                f'task {update.task_id}: state {update.state} '
                f'code {update.error_code} {update.reason}')

    def _navigate(self, navigate, waited_sec=0.0):
        generation = navigate.generation
        if generation != self.machine.nav_generation:
            return                                  # superseded while waiting
        if not self.nav_client.server_is_ready():
            timeout_sec = self.params['nav_server_timeout_sec']
            if waited_sec >= timeout_sec:
                self.get_logger().warn(f'Nav2 not available after {timeout_sec:.0f}s')
                self._dispatch(self.machine.nav_result(
                    generation, False, 'nav2 not available'))
                return
            # Nav2 activates its lifecycle nodes after start: poll (sim time)
            retry_timer = None

            def retry():
                retry_timer.cancel()
                self._navigate(navigate, waited_sec + NAV_SERVER_POLL_SEC)
            retry_timer = self.create_timer(NAV_SERVER_POLL_SEC, retry)
            return
        goal = navigate.goal
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = PoseStamped()
        nav_goal.pose.header.frame_id = self.params['map_frame']
        nav_goal.pose.header.stamp = self.get_clock().now().to_msg()
        nav_goal.pose.pose.position.x = goal.x
        nav_goal.pose.pose.position.y = goal.y
        orientation = nav_goal.pose.pose.orientation
        orientation.x, orientation.y, orientation.z, orientation.w = \
            contract.quaternion_from_yaw(goal.yaw)
        self.get_logger().info(
            f'leg {navigate.leg_index} -> {goal.location_id} '
            f'({goal.x:.2f}, {goal.y:.2f})')
        send_future = self.nav_client.send_goal_async(
            nav_goal,
            feedback_callback=lambda feedback: self._on_feedback(generation, feedback))
        send_future.add_done_callback(
            lambda future: self._on_goal_response(generation, future))

    def _on_goal_response(self, generation, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._dispatch(self.machine.nav_result(
                generation, False, 'goal rejected by Nav2'))
            return
        self._goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(
            lambda result_future: self._on_result(generation, result_future))

    def _on_feedback(self, generation, feedback):
        if generation != self.machine.nav_generation:
            return
        now = self.get_clock().now()
        period = Duration(seconds=self.params['feedback_period_sec'])
        if self._last_feedback_time is not None and now - self._last_feedback_time < period:
            return
        self._last_feedback_time = now
        self._dispatch(self.machine.nav_feedback(feedback.feedback.distance_remaining))

    def _on_result(self, generation, future):
        response = future.result()
        succeeded = response.status == GoalStatus.STATUS_SUCCEEDED
        reason = '' if succeeded else (getattr(response.result, 'error_msg', '') or
                                       f'nav2 status {response.status}')
        self._dispatch(self.machine.nav_result(generation, succeeded, reason))

    def _cancel_navigation(self):
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None
        if self._dwell_timer is not None:
            self._dwell_timer.cancel()
            self._dwell_timer = None

    def _start_dwell(self, seconds):
        def dwell_finished():
            self._dwell_timer.cancel()
            self._dwell_timer = None
            self._dispatch(self.machine.dwell_done())
        self._dwell_timer = self.create_timer(seconds, dwell_finished)

    # ---------------------------------------------------------- telemetry
    def _status(self):
        # live parameter: `ros2 param set /fleetgraph_agent maintenance true`
        if self.get_parameter('maintenance').value:
            return RobotState.STATUS_MAINTENANCE
        if self.machine.active_task_id:
            return RobotState.STATUS_BUSY
        return RobotState.STATUS_AVAILABLE

    def _errors(self):
        errors = []
        if self.battery < self.params['min_battery']:
            errors.append(ErrorInfo(code=ErrorCode.LOW_BATTERY,
                                    level=ErrorInfo.LEVEL_WARNING,
                                    description=f'battery {self.battery:.2f}'))
        if self._pose is None:
            errors.append(ErrorInfo(code=ErrorCode.NONE, level=ErrorInfo.LEVEL_WARNING,
                                    description='not localized (no map->base TF)'))
        return errors

    def _drain_battery_idle(self):
        now = self.get_clock().now()
        if self._last_drain_time is not None:
            elapsed_sec = (now - self._last_drain_time).nanoseconds * 1e-9
            self.battery -= elapsed_sec * self.params['battery_drain_per_s']
        self._last_drain_time = now
        self.battery = max(0.0, self.battery)

    def _lookup_pose(self):
        """Robot pose in the map frame from TF, None when not localized."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.params['map_frame'], self.params['base_frame'], Time())
        except TransformException:
            return None
        pose = PoseStamped()
        pose.header = transform.header
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        return pose

    def _publish_state(self):
        self._drain_battery_idle()
        self._pose = self._lookup_pose()
        msg = RobotState(robot_id=self.robot_id, battery=float(self.battery),
                         charging=False, status=self._status(),
                         capabilities=list(self.params['capabilities']),
                         current_task_id=self.machine.active_task_id,
                         connection_quality=-1.0,
                         localization_score=float(self._localization_score),
                         errors=self._errors())
        if self._pose is not None:
            msg.pose = self._pose
        if self._velocity is not None:
            msg.velocity = self._velocity
        self.pub_state.publish(self.meta.stamp(msg, contract.TOPIC_ROBOT_STATE))

    def _publish_connection(self, state):
        msg = RobotConnection(robot_id=self.robot_id, state=state)
        self.pub_connection.publish(self.meta.stamp(msg, contract.TOPIC_ROBOT_CONNECTION))


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
