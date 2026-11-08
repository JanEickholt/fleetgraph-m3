"""Social keepout mask (Module 3): the robot's social zones as a Nav2
KeepoutFilter mask, so plans avoid every person's and group's zone and the
controller stops short of one (social_mask.py).

  ros2 run fleetgraph_robot social_mask --ros-args -r __ns:=/robot_1 \
      -p robot_id:=robot_1 -p use_sim_time:=true

Inputs, relative to the robot namespace: map (map_server, latched; the mask
takes its grid) and, per entry in `sources`: local = this robot's own
SceneObservation deltas on /fleet/scene_observation (its local scene graph),
central = the central replica's snapshots on central/scene (scene_replica,
for the shared-graph baseline). Outputs: keepout_mask (nav_msgs/
OccupancyGrid) and keepout_filter_info (nav2_msgs/CostmapFilterInfo: keepout,
base 0, multiplier 1, naming the mask topic), reliable and transient local
as the KeepoutFilter subscribes them. The mask is rebuilt at most
max_rate_hz and published only when it changed. Robot software: never reads
ground truth (/ground_truth/*, /sim/*).
"""
import math

import rclpy
from nav2_msgs.msg import CostmapFilterInfo
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from fleetgraph_interfaces.msg import SceneGraphUpdate, SceneObservation
from fleetgraph_robot import contract
from fleetgraph_robot.scene_msgs import MAP_FRAME, entity_from_msg
from fleetgraph_robot.social import SocialParams
from fleetgraph_robot.social_mask import Grid, SocialMask

SOURCES = ('local', 'central')
CENTRAL_SCENE_TOPIC = 'central/scene'       # scene_replica_node.SCENE_TOPIC
FILTER_TYPE_KEEPOUT = 0                     # CostmapFilterInfo.type: keepout / lanes
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


def grid_from_info(info):
    """nav_msgs/MapMetaData -> Grid; ValueError for a rotated origin."""
    orientation = info.origin.orientation
    yaw = contract.yaw_from_quaternion(orientation)
    if abs(yaw) > 1e-6:
        raise ValueError(f'map origin turned by {math.degrees(yaw):.2f} deg: '
                         f'only an axis-aligned grid is supported')
    return Grid(info.origin.position.x, info.origin.position.y, info.resolution,
                info.width, info.height)


class SocialMaskNode(Node):
    def __init__(self):
        super().__init__('fleetgraph_social_mask')
        declared = self.declare_parameters('', [
            ('robot_id', 'robot_1'),
            ('sources', ['local']),           # local and/or central
            ('robot_radius', 0.25),           # Nav2 footprint radius (nav2_params.yaml)
            # AMCL is up to 0.14 m off while driving (README, Perception), the
            # zones sit where the scan-matched pose put the detections
            ('margin', 0.15),
            ('min_distance', 0.5),            # social.SocialParams
            ('person_radius', 0.25),
            ('group_margin', 0.2),
            ('max_rate_hz', 2.0),             # the perception node's publish rate
            ('stale_after_sec', 10.0),        # > the local graph's person time-to-live
        ])
        self.params = {parameter.name: parameter.value for parameter in declared}
        self.robot_id = self.params['robot_id']
        sources = list(self.params['sources'])
        unknown = sorted(set(sources) - set(SOURCES))
        if unknown or not sources:
            raise ValueError(f'sources must name some of {list(SOURCES)}, got {sources}')
        social = SocialParams(min_distance=self.params['min_distance'],
                              person_radius=self.params['person_radius'],
                              group_margin=self.params['group_margin'])
        self.mask = SocialMask(social, self.params['robot_radius'], self.params['margin'],
                               self.params['stale_after_sec'])
        self._map_header = None
        self._map_info = None

        self.pub_mask = self.create_publisher(OccupancyGrid, 'keepout_mask', LATCHED)
        self.pub_info = self.create_publisher(CostmapFilterInfo, 'keepout_filter_info', LATCHED)
        self.create_subscription(OccupancyGrid, 'map', self._on_map, LATCHED)
        if 'local' in sources:
            self.create_subscription(SceneObservation, contract.TOPIC_SCENE_OBSERVATION,
                                     self._on_observation, contract.QOS_COMMANDS)
        if 'central' in sources:
            self.create_subscription(SceneGraphUpdate, CENTRAL_SCENE_TOPIC,
                                     self._on_central_scene, LATCHED)
        self._publish_info()
        self.create_timer(1.0 / self.params['max_rate_hz'], self._publish_mask)
        self.get_logger().info(
            f'{self.robot_id} social mask from {sources} on {self.pub_mask.topic_name}: '
            f'robot radius {self.params["robot_radius"]} m + margin {self.params["margin"]} m')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------- inputs
    def _on_map(self, msg):
        try:
            grid = grid_from_info(msg.info)
        except ValueError as error:
            self.get_logger().error(f'map ignored: {error}')
            return
        self._map_header, self._map_info = msg.header, msg.info
        self.mask.set_grid(grid)
        self.get_logger().info(f'map grid {grid.width}x{grid.height} cells of '
                               f'{grid.resolution:.2f} m')

    def _on_observation(self, msg):
        if msg.robot_id != self.robot_id:
            return
        if msg.frame_id != MAP_FRAME:
            self.get_logger().warn(f'observation in frame {msg.frame_id!r} ignored: '
                                   f'zones need {MAP_FRAME!r}', throttle_duration_sec=10.0)
            return
        self.mask.apply_delta('local', [entity_from_msg(entity) for entity in msg.entities],
                              list(msg.removed_entity_ids), self._now())

    def _on_central_scene(self, msg):
        if not msg.is_snapshot or msg.frame_id != MAP_FRAME:
            self.get_logger().warn('central scene ignored: not a map-frame snapshot',
                                   throttle_duration_sec=10.0)
            return
        self.mask.replace('central', [entity_from_msg(entity) for entity in msg.added_nodes])

    # ------------------------------------------------------------ outputs
    def _publish_info(self):
        msg = CostmapFilterInfo(type=FILTER_TYPE_KEEPOUT, base=0.0, multiplier=1.0,
                                filter_mask_topic=self.pub_mask.topic_name)
        msg.header.frame_id = MAP_FRAME
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_info.publish(msg)

    def _publish_mask(self):
        zones_before = self.mask.zone_count
        data = self.mask.update(self._now())
        if data is None:
            return
        msg = OccupancyGrid(info=self._map_info, data=data)
        msg.header.frame_id = self._map_header.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_mask.publish(msg)
        if self.mask.zone_count != zones_before:
            self.get_logger().info(f'{self.mask.zone_count} social zones in the mask')


def main():
    rclpy.init()
    node = SocialMaskNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
