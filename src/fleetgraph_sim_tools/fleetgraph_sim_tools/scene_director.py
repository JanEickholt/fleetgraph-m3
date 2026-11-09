"""Scene director (simulation only): plays a scene script in Gazebo and
publishes its ground truth.

  ros2 run fleetgraph_sim_tools scene_director --ros-args -p scene:=box_front \
      -p use_sim_time:=true

scene is a name in share/fleetgraph_sim_tools/scenes or a path to a .yaml
file (format: scene_script.py). Scene time 0 lies at the clock time the test
harness passes in FLEETGRAPH_SCRIPT_ORIGIN_SEC (its window start plus the
action's t, the origin the mock M1 counts task times from), else at the first
/clock tick.
Each tick (rate_hz, sim time) spawns the models that became active, moves
those whose pose changed and removes expired ones through the world's
UserCommands services, then publishes /sim/truth: every active entity, also
preexisting world objects. The /blocking service variants return only once
Gazebo has applied the command, and say whether it worked. /sim/truth is for
sim tools and tests: robot software never subscribes.
"""
import math
import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from fleetgraph_interfaces.msg import SceneEntity, SceneObservation
from fleetgraph_robot import contract
from fleetgraph_robot.mock_orchestrator import script_origin
from fleetgraph_sim_tools import scene_script

GZ_TIMEOUT_MS = 2000


def scene_path(scene):
    if scene.endswith(('.yaml', '.yml')) or os.sep in scene:
        return scene
    return os.path.join(get_package_share_directory('fleetgraph_sim_tools'), 'scenes',
                        f'{scene}.yaml')


def entity_msg(truth):
    msg = SceneEntity(id=truth.id, category=truth.category, label=truth.label, confidence=1.0)
    msg.position.x, msg.position.y, msg.position.z = truth.x, truth.y, truth.z
    msg.size.x, msg.size.y, msg.size.z = truth.size
    msg.velocity.x, msg.velocity.y = truth.vx, truth.vy
    return msg


def fill_pose(msg, entity, pose):
    """gz.msgs.Pose of the model: standing on the floor, turned by yaw."""
    x, y, yaw = pose
    msg.position.x, msg.position.y, msg.position.z = x, y, entity.shape.size[2] / 2
    msg.orientation.z, msg.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)


class SceneDirector(Node):
    def __init__(self):
        super().__init__('scene_director')
        declared = self.declare_parameters('', [
            ('scene', ''),
            ('world', 'lab_floor'),
            ('rate_hz', 10.0),
        ])
        self.params = {parameter.name: parameter.value for parameter in declared}
        self.scene = scene_script.load(scene_path(self.params['scene']))
        self.playback = scene_script.Playback(self.scene)
        self.gz = GzNode()
        self.meta = contract.MetaStamper('scene_director', self.get_clock())
        self.pub_truth = self.create_publisher(SceneObservation, scene_script.TOPIC_TRUTH,
                                               contract.QOS_TELEMETRY)
        self._origin_sec = None             # clock time of scene time 0
        self.create_timer(1.0 / self.params['rate_hz'], self._tick)
        self.get_logger().info(f'scene {self.scene.name}: {len(self.scene.entities)} entities')

    def _tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now == 0:
            return                          # no /clock yet
        if self._origin_sec is None:
            self._origin_sec, problem = script_origin(now)
            if problem:
                self.get_logger().warn(f'{problem}; scene time starts now')
            self.get_logger().info(f'scene time 0 at clock {self._origin_sec:.3f} '
                                   f'(now {now - self._origin_sec:.3f})')
        step = self.playback.step(now - self._origin_sec)
        for entity, pose in step.spawn:
            self._spawn(entity, pose)
        if step.move:
            self._move(step.move)
        for entity in step.remove:
            self._remove(entity)
        msg = SceneObservation(robot_id='sim', frame_id='map',
                               entities=[entity_msg(truth) for truth in step.truth],
                               removed_entity_ids=step.removed_ids)
        self.pub_truth.publish(self.meta.stamp(msg, scene_script.TOPIC_TRUTH))

    def _call(self, service, request):
        ok, reply = self.gz.request(f"/world/{self.params['world']}/{service}/blocking",
                                    request, type(request), Boolean, GZ_TIMEOUT_MS)
        return ok and reply.data

    def _spawn(self, entity, pose):
        request = EntityFactory(sdf=scene_script.model_sdf(entity))
        fill_pose(request.pose, entity, pose)
        if not self._call('create', request):
            # most likely left over from an earlier director run: replace it
            self._call('remove', Entity(name=entity.model_name, type=Entity.MODEL))
            if not self._call('create', request):
                self.get_logger().error(f'could not spawn {entity.model_name}')
                return
        self.get_logger().info(f'spawned {entity.model_name} at ({pose[0]:.2f}, {pose[1]:.2f})')

    def _move(self, moves):
        request = Pose_V()
        for entity, pose in moves:
            item = request.pose.add()
            item.name = entity.model_name
            fill_pose(item, entity, pose)
        if not self._call('set_pose_vector', request):
            self.get_logger().warn(f'could not move {[entity.model_name for entity, _ in moves]}')

    def _remove(self, entity):
        if self._call('remove', Entity(name=entity.model_name, type=Entity.MODEL)):
            self.get_logger().info(f'removed {entity.model_name}')
        else:
            self.get_logger().error(f'could not remove {entity.model_name}')


def main():
    rclpy.init()
    try:
        node = SceneDirector()
    except (OSError, ValueError) as error:  # missing or invalid scene file
        rclpy.logging.get_logger('scene_director').error(str(error))
        rclpy.shutdown()
        raise SystemExit(1)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
