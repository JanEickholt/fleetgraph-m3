"""FleetGraph local perception (Module 3): builds the robot's local scene
graph from its lidar (returns where the static map says free) and from
semantic detections, and publishes it to Module 1 as SceneObservation deltas.

  ros2 run fleetgraph_robot perception --ros-args -r __ns:=/robot_1 \
      -p robot_id:=robot_1 -p use_sim_time:=true

Inputs, relative to the robot namespace: scan, map (map_server, latched),
amcl_pose, perception/detections (SceneObservation in a robot frame such as
base_footprint, stamped with the sensing time) and TF. Output:
/fleet/scene_observation. Poses come from TF (AMCL), never from simulator
ground truth.

Per scan: the lidar's map pose at the scan stamp (TF, corrected against the
static map by MapMatcher), lidar obstacles against the static map plus the
semantic detections sensed up to that stamp (each placed with TF at its own
stamp), and one LocalSceneGraph.integrate() with per-sensor coverage: the
scan's, and the detector's from each detection frame folded in (at that
frame's stamp), so a track decays only on frames of sensors that saw it
before. At publish_rate_hz the delta goes out when it holds anything
(entities, removals, a changed relation set), otherwise at least every
heartbeat_sec. Nothing is published before the first integrated scan.

Latency = meta.stamp - sensor_pose.header.stamp: how old the newest scan in
a published observation is (sim seconds).
"""
import math
from collections import deque

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from fleetgraph_interfaces.msg import SceneObservation
from fleetgraph_robot import contract, scene_msgs
from fleetgraph_robot.lidar_obstacles import obstacle_detections
from fleetgraph_robot.perception import (
    SEMANTIC_SOURCE, MapMatcher, any_view, compose, detector_coverage, lidar_coverage,
    localization_score, map_correction, occupancy_from_grid, semantic_detections)
from fleetgraph_robot.regions import load_regions
from fleetgraph_robot.scene_graph import LIDAR_SOURCE, LocalSceneGraph

# The TF of a scan (odom -> base_footprint at 30 Hz) arrives up to ~40 ms of
# sim time after the scan (measured 2026-11-08): pending scans are retried at
# this period and dropped once their TF is tf_timeout_sec overdue.
RETRY_PERIOD_SEC = 0.02
STATS_PERIOD_SEC = 10.0
MAX_PENDING = 20                    # scans, and detection messages, awaiting integration
MAP_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


def pose_stamped(transform):
    """TransformStamped -> PoseStamped of its child frame in its frame."""
    translation = transform.transform.translation
    pose = PoseStamped(header=transform.header)
    pose.pose.position = Point(x=translation.x, y=translation.y, z=translation.z)
    pose.pose.orientation = transform.transform.rotation
    return pose


def _pose2d(transform):
    translation = transform.transform.translation
    return (translation.x, translation.y,
            contract.yaw_from_quaternion(transform.transform.rotation))


def _seconds(time):
    return time.nanoseconds * 1e-9


class Perception(Node):
    def __init__(self):
        # tf2_ros listens on absolute /tf and /tf_static: remap them into the
        # robot namespace, as nav2_bringup does for Nav2
        super().__init__('fleetgraph_perception', cli_args=[
            '--ros-args', '-r', '/tf:=tf', '-r', '/tf_static:=tf_static'])
        default_regions = get_package_share_directory('fleetgraph_sim') + \
            '/config/regions.yaml'
        declared = self.declare_parameters('', [
            ('robot_id', 'robot_1'),
            ('regions_file', default_regions),
            ('publish_rate_hz', 2.0),
            ('heartbeat_sec', 5.0),
            ('map_frame', 'map'),
            ('base_frame', 'base_footprint'),
            ('tf_timeout_sec', 0.5),
            # The corrected lidar pose is still ~0.05 m and 1 deg off, which
            # shifts wall returns by up to ~0.12 m at 4 m: lidar evidence counts
            # within obstacle_range (m), and returns within map_tolerance (m)
            # of a mapped cell count as the map.
            ('obstacle_range', 4.0),
            ('map_tolerance', 0.2),
            # Field of view (deg) and range (m) of the semantic detector,
            # looking along the x axis of the detections' frame_id; defaults:
            # the simulated detector (fleetgraph_sim_tools DetectorParams).
            ('detector_fov_deg', 120.0),
            ('detector_range', 6.0),
        ])
        self.params = {parameter.name: parameter.value for parameter in declared}
        self.robot_id = self.params['robot_id']
        with open(self.params['regions_file']) as file:
            regions = load_regions(yaml.safe_load(file))
        self.graph = LocalSceneGraph(self.robot_id, regions)
        self.meta = contract.MetaStamper(self.robot_id, self.get_clock())

        self.pub_observation = self.create_publisher(
            SceneObservation, contract.TOPIC_SCENE_OBSERVATION, contract.QOS_COMMANDS)
        self.create_subscription(OccupancyGrid, 'map', self._on_map, MAP_QOS)
        self.create_subscription(LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(SceneObservation, 'perception/detections',
                                 self._on_detections, 10)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose',
                                 self._on_amcl_pose, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._occupancy = None
        self._matcher = None
        self._scans = deque()
        self._detections = deque()          # (sensing time, frame_id, entities)
        self._sensor_pose = None            # base pose at the newest integrated scan
        # AMCL confidence; 1.0 until the first amcl_pose (AMCL starts at the
        # known dock pose)
        self._localization_score = 1.0
        self._published_relations = None
        self._last_publish = None
        self._counts = dict.fromkeys(
            ('scans', 'scans_dropped', 'detections', 'detections_dropped', 'published'), 0)
        self._latency_sum = 0.0
        self._latency_max = 0.0
        self._correction_max = (0.0, 0.0)  # m, rad: largest MapMatcher correction
        self._region_count = len(regions)

        self.create_timer(RETRY_PERIOD_SEC, self._integrate_ready)
        self.create_timer(1.0 / self.params['publish_rate_hz'], self._publish)
        self.create_timer(STATS_PERIOD_SEC, self._log_stats)
        self.get_logger().info(f'{self.robot_id} perception ready: {len(regions)} regions')

    # ------------------------------------------------------------- inputs
    def _on_map(self, msg):
        try:
            self._occupancy = occupancy_from_grid(msg)
        except ValueError as error:
            self.get_logger().error(f'map ignored: {error}')
            return
        self._matcher = MapMatcher(self._occupancy)
        self.get_logger().info(
            f'map {msg.info.width}x{msg.info.height} cells of {msg.info.resolution:.2f} m')

    def _on_scan(self, msg):
        if len(self._scans) == MAX_PENDING:
            self._scans.popleft()
            self._counts['scans_dropped'] += 1
        self._scans.append(msg)
        self._integrate_ready()

    def _on_detections(self, msg):
        if len(self._detections) == MAX_PENDING:
            _, _, dropped = self._detections.popleft()
            self._counts['detections_dropped'] += len(dropped)
        self._detections.append(
            (Time.from_msg(msg.sensor_pose.header.stamp), msg.frame_id, msg.entities))

    def _on_amcl_pose(self, msg):
        self._localization_score = localization_score(msg.pose.covariance)

    # ---------------------------------------------------------- integration
    def _integrate_ready(self):
        """Integrate pending scans in order, each once TF covers its stamp."""
        now = self.get_clock().now()
        timeout = Duration(seconds=self.params['tf_timeout_sec'])
        while self._scans:
            scan = self._scans[0]
            stamp = Time.from_msg(scan.header.stamp)
            if self._occupancy is not None and self.tf_buffer.can_transform(
                    self.params['map_frame'], scan.header.frame_id, stamp):
                self._scans.popleft()
                self._integrate(scan, stamp)
            elif now - stamp > timeout:
                self._scans.popleft()
                self._counts['scans_dropped'] += 1
            else:
                return

    def _integrate(self, scan, stamp):
        map_frame = self.params['map_frame']
        try:
            amcl = _pose2d(self.tf_buffer.lookup_transform(map_frame, scan.header.frame_id,
                                                           stamp))
            base = self.tf_buffer.lookup_transform(map_frame, self.params['base_frame'], stamp)
        except TransformException as error:
            self.get_logger().warn(f'scan dropped: {error}', throttle_duration_sec=10.0)
            self._counts['scans_dropped'] += 1
            return
        lidar = self._matcher.refine(scan.ranges, scan.angle_min, scan.angle_increment,
                                     scan.range_min, scan.range_max, amcl)
        shift = math.hypot(lidar[0] - amcl[0], lidar[1] - amcl[1])
        self._correction_max = (max(self._correction_max[0], shift),
                                max(self._correction_max[1], abs(lidar[2] - amcl[2])))
        now = _seconds(stamp)
        range_max = min(scan.range_max, self.params['obstacle_range'])
        detections = obstacle_detections(
            scan.ranges, scan.angle_min, scan.angle_increment, scan.range_min, range_max,
            lidar, self._occupancy, stamp=now, tolerance=self.params['map_tolerance'])
        coverage = {LIDAR_SOURCE: lidar_coverage(self._occupancy, lidar[0], lidar[1], range_max)}
        semantic, views = self._semantic_until(stamp, map_correction(amcl, lidar))
        if views:
            coverage[SEMANTIC_SOURCE] = any_view(views)
        self.graph.integrate(detections + semantic, now, coverage)
        self._sensor_pose = pose_stamped(base)
        self._counts['scans'] += 1

    def _semantic_until(self, stamp, correction):
        """Map-frame detections of the messages sensed up to stamp, and the
        coverage of each of those frames (also of empty ones)."""
        detections, views = [], []
        while self._detections and self._detections[0][0] <= stamp:
            sensed, frame_id, entities = self._detections.popleft()
            try:
                # The correction of the scan they join, the first at or after their
                # stamp (<= 0.1 s later), not one interpolated to it: AMCL's map ->
                # odom changes in steps at its filter updates, rarely in between.
                pose = compose(correction, _pose2d(self.tf_buffer.lookup_transform(
                    self.params['map_frame'], frame_id, sensed)))
            except TransformException as error:
                self.get_logger().warn(f'detections dropped: {error}',
                                       throttle_duration_sec=10.0)
                self._counts['detections_dropped'] += len(entities)
                continue
            placed = semantic_detections(entities, pose, _seconds(sensed))
            self._counts['detections'] += len(placed)
            self._counts['detections_dropped'] += len(entities) - len(placed)
            detections += placed
            views.append(detector_coverage(self._occupancy, pose, self.params['detector_fov_deg'],
                                           self.params['detector_range']))
        return detections, views

    # ------------------------------------------------------------- output
    def _publish(self):
        if self._sensor_pose is None:
            return
        now = self.get_clock().now()
        delta = self.graph.take_delta()
        relations = {(relation.subject_id, relation.predicate, relation.object_id)
                     for relation in delta.relations}
        changed = (delta.entities or delta.removed_ids
                   or relations != self._published_relations)
        heartbeat_due = (self._last_publish is None or now - self._last_publish
                         >= Duration(seconds=self.params['heartbeat_sec']))
        if not (changed or heartbeat_due):
            return
        msg = scene_msgs.observation_to_msg(delta, self.robot_id, self.params['map_frame'],
                                            self._sensor_pose, self._localization_score)
        self.pub_observation.publish(self.meta.stamp(msg, contract.TOPIC_SCENE_OBSERVATION))
        self._published_relations = relations
        self._last_publish = now
        latency = _seconds(Time.from_msg(msg.meta.stamp)) - \
            _seconds(Time.from_msg(msg.sensor_pose.header.stamp))
        self._counts['published'] += 1
        self._latency_sum += latency
        self._latency_max = max(self._latency_max, latency)

    def _log_stats(self):
        counts = self._counts
        latency = (f'latency mean {self._latency_sum / counts["published"]:.3f} s, '
                   f'max {self._latency_max:.3f} s' if counts['published'] else 'no latency yet')
        tracks = len(self.graph.entities()) - self._region_count
        shift, turn = self._correction_max
        self.get_logger().info(
            f'scans {counts["scans"]} integrated, {counts["scans_dropped"]} dropped, pose '
            f'correction max {shift:.2f} m {math.degrees(turn):.1f} deg; detections '
            f'{counts["detections"]} used, {counts["detections_dropped"]} dropped; '
            f'{tracks} entities reported; observations {counts["published"]}, {latency}')


def main():
    rclpy.init()
    node = Perception()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node._log_stats()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
