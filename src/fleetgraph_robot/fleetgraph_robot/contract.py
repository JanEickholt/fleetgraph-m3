"""FleetGraph contract constants shared by all M3 nodes: topic names, QoS,
schema version, Meta stamping and msg <-> task_machine conversions.
docs/interfaces.md is the human-readable version of this file.

QoS rule: every contract PUBLISHER is reliable (a reliable publisher
serves both reliable and best-effort subscribers; a best-effort publisher
silently serves no reliable subscriber). Subscribers pick what they need.
"""
import math

from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from fleetgraph_interfaces.msg import Meta, TaskUpdate
from fleetgraph_robot.task_machine import Goal, TaskSpec

SCHEMA_VERSION = '0.1.0'

TOPIC_ROBOT_STATE = '/fleet/robot_state'
TOPIC_ROBOT_CONNECTION = '/fleet/robot_connection'
TOPIC_TASK_ASSIGNMENT = '/fleet/task_assignment'
TOPIC_TASK_COMMAND = '/fleet/task_command'
TOPIC_TASK_UPDATE = '/fleet/task_update'
TOPIC_SCENE_OBSERVATION = '/fleet/scene_observation'
TOPIC_SCENE_GRAPH_UPDATE = '/fleet/scene_graph_update'
SERVICE_GET_SCENE_GRAPH = '/fleet/get_scene_graph'


def _reliable_qos(depth, durability=DurabilityPolicy.VOLATILE):
    return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=depth,
                      reliability=ReliabilityPolicy.RELIABLE,
                      durability=durability)


QOS_TELEMETRY = _reliable_qos(10)                                  # robot_state
QOS_COMMANDS = _reliable_qos(50)                                   # assignment, command, observation
QOS_EVENTS = _reliable_qos(100, DurabilityPolicy.TRANSIENT_LOCAL)  # task_update: late joiners get history
QOS_LATCHED = _reliable_qos(10, DurabilityPolicy.TRANSIENT_LOCAL)  # robot_connection


class MetaStamper:
    """Fills msg.meta; seq counts per (source, topic) from 1."""

    def __init__(self, source_id, clock):
        self.source_id = source_id
        self.clock = clock
        self._seq_by_topic = {}

    def stamp(self, msg, topic):
        self._seq_by_topic[topic] = self._seq_by_topic.get(topic, 0) + 1
        msg.meta = Meta(schema_version=SCHEMA_VERSION, source_id=self.source_id,
                        seq=self._seq_by_topic[topic], stamp=self.clock.now().to_msg())
        return msg


def yaw_from_quaternion(quaternion):
    x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quaternion_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))   # x, y, z, w


def spec_from_assignment(msg):
    """TaskAssignment -> TaskSpec. A Location pose is used when its frame is
    'map'; otherwise the robot resolves Location.id from its registry."""
    task = msg.task
    locations = []
    for location in task.locations:
        goal = None
        if location.pose.header.frame_id == 'map':
            pose = location.pose.pose
            goal = Goal(pose.position.x, pose.position.y,
                        yaw_from_quaternion(pose.orientation), location.id)
        locations.append((location.id, goal))
    return TaskSpec(task_id=task.task_id, type=task.type, locations=locations,
                    required_capabilities=list(task.required_capabilities),
                    patrol_loops=task.patrol_loops,
                    assignment_rev=msg.assignment_rev)


def update_to_msg(update, robot_id):
    """task_machine.Update -> TaskUpdate (meta filled by MetaStamper)."""
    return TaskUpdate(task_id=update.task_id, robot_id=robot_id,
                      assignment_rev=update.assignment_rev, state=update.state,
                      progress=float(update.progress), leg_index=update.leg_index,
                      leg_count=update.leg_count,
                      distance_remaining_m=float(update.distance_remaining),
                      error_code=update.error_code, reason=update.reason,
                      reassignment_requested=update.reassignment_requested)


def load_locations(path):
    """locations.yaml (tools/gen_world.py) -> {id: Goal}."""
    import yaml
    with open(path) as file:
        registry = yaml.safe_load(file)
    return {location_id: Goal(float(entry['x']), float(entry['y']),
                              float(entry.get('yaw', 0.0)), location_id)
            for location_id, entry in registry['locations'].items()}
