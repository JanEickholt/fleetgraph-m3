"""Simulated object detector of one robot (simulation only): ground truth
in, robot-relative detections out, as a camera detector would report them.

  ros2 run fleetgraph_sim_tools sim_detector --ros-args -r __ns:=/robot_1 \
      -p robot_id:=robot_1 -p robots:=3 -p use_sim_time:=true

Truth: the scene entities on /sim/truth (scene_director, preexisting world
objects included) and the robots' /ground_truth/robot_M/odom. Each tick
(rate_hz, sim time) runs detector_model.simulate from this robot's latest
ground-truth pose, with the static map of fleetgraph_sim for line of sight,
and publishes perception/detections in base_footprint, also when empty.
Only unmapped things are fed in: the model looks at each entity's centre,
so mapped furniture would hide itself. The scene sample may be up to one
truth period (0.1 s) older than the pose sample, within the position noise
at walking speed. One Random per robot, seeded '<seed>:<robot_id>': a seed
reproduces a run, and robots draw independent noise. Detections carry
neither truth ids nor the sensor pose (identity): robot software has to
associate and localize on its own.
"""
import os
import random

import rclpy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from fleetgraph_interfaces.msg import SceneEntity, SceneObservation
from fleetgraph_robot import contract
from fleetgraph_robot.scene_graph import Category
from fleetgraph_sim_tools import map_loader, scene_script
from fleetgraph_sim_tools.detector_model import DetectorParams, TruthEntity, simulate

TOPIC_DETECTIONS = 'perception/detections'
BASE_FRAME = 'base_footprint'
ROBOT_SIZE = (0.4, 0.3, 0.16)               # body and wheels, fleetgraph_sim robot.urdf.xacro
MODEL_PARAMS = ('fov_deg', 'min_range', 'max_range', 'position_sigma', 'dropout',
                'confidence_mean', 'confidence_sigma')


def truth_entity(msg):
    """SceneEntity of /sim/truth -> TruthEntity."""
    return TruthEntity(msg.id, msg.category, msg.label, msg.position.x, msg.position.y,
                       msg.position.z, (msg.size.x, msg.size.y, msg.size.z),
                       msg.velocity.x, msg.velocity.y)


def detection_msg(detection):
    msg = SceneEntity(category=detection.category, label=detection.label,
                      confidence=float(detection.confidence))
    msg.position.x, msg.position.y, msg.position.z = \
        detection.rel_x, detection.rel_y, detection.z
    msg.size.x, msg.size.y, msg.size.z = detection.size
    return msg


class SimDetector(Node):
    def __init__(self):
        super().__init__('sim_detector')
        defaults = DetectorParams()
        declared = self.declare_parameters('', [
            ('robot_id', 'robot_1'),
            ('robots', 1),                  # robot_1..robot_N are detectable
            ('rate_hz', 5.0),
            ('seed', 1),
            ('map_yaml', os.path.join(get_package_share_directory('fleetgraph_sim'),
                                      'maps', 'lab_floor.yaml')),
            *((name, getattr(defaults, name)) for name in MODEL_PARAMS),
        ])
        self.params = {parameter.name: parameter.value for parameter in declared}
        self.robot_id = self.params['robot_id']
        self.model = DetectorParams(**{name: self.params[name] for name in MODEL_PARAMS})
        self.occupancy = map_loader.load(self.params['map_yaml'])
        self.rng = random.Random(f"{self.params['seed']}:{self.robot_id}")
        self.meta = contract.MetaStamper(f'{self.robot_id}/detector', self.get_clock())
        self.robot_ids = [f'robot_{index}' for index in range(1, self.params['robots'] + 1)]
        if self.robot_id not in self.robot_ids:
            self.robot_ids.append(self.robot_id)
        self.scene = []                     # TruthEntity, latest /sim/truth
        self.poses = {}                     # robot id -> (x, y, yaw, stamp)
        self.pub = self.create_publisher(SceneObservation, TOPIC_DETECTIONS,
                                         contract.QOS_TELEMETRY)
        self.create_subscription(SceneObservation, scene_script.TOPIC_TRUTH, self._on_truth,
                                 contract.QOS_TELEMETRY)
        for robot_id in self.robot_ids:
            self.create_subscription(
                Odometry, f'/ground_truth/{robot_id}/odom',
                lambda msg, robot_id=robot_id: self._on_odom(robot_id, msg),
                qos_profile_sensor_data)
        self.create_timer(1.0 / self.params['rate_hz'], self._tick)
        self.get_logger().info(f'{self.robot_id} detector: robots {self.robot_ids}, '
                               f'seed {self.params["seed"]}')

    def _on_truth(self, msg):
        self.scene = [truth_entity(entity) for entity in msg.entities]

    def _on_odom(self, robot_id, msg):
        pose = msg.pose.pose
        self.poses[robot_id] = (pose.position.x, pose.position.y,
                                contract.yaw_from_quaternion(pose.orientation), msg.header.stamp)

    def _tick(self):
        if self.robot_id not in self.poses:
            return                          # no pose sample yet: nothing to see from
        x, y, yaw, stamp = self.poses[self.robot_id]
        robots = [TruthEntity(robot_id, Category.ROBOT, 'robot', pose[0], pose[1],
                              z=ROBOT_SIZE[2] / 2, size=ROBOT_SIZE)
                  for robot_id in self.robot_ids if (pose := self.poses.get(robot_id))]
        detections = simulate(self.scene + robots, (x, y, yaw), self.occupancy, self.model,
                              self.rng, exclude_id=self.robot_id)
        msg = SceneObservation(robot_id=self.robot_id, frame_id=BASE_FRAME,
                               entities=[detection_msg(detection) for detection in detections])
        msg.sensor_pose.header.stamp = stamp
        msg.sensor_pose.header.frame_id = BASE_FRAME
        msg.sensor_pose.pose.orientation.w = 1.0
        self.pub.publish(self.meta.stamp(msg, TOPIC_DETECTIONS))


def main():
    rclpy.init()
    node = SimDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
