"""Topic recorder: subscribes to declared topics, stores (t, msg) streams.

Timebase: sim seconds from /clock when Gazebo publishes it, wall seconds
otherwise (no-sim scenarios). All deadlines and rates are computed in the
active timebase, never wall time when a sim is present.
"""
import time

import rclpy.time
from builtin_interfaces.msg import Time
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from rosidl_parser.definition import NamespacedType
from rosidl_runtime_py.convert import get_message_slot_types
from rosidl_runtime_py.import_message import import_message_from_namespaced_type
from rosidl_runtime_py.utilities import get_message
from rosgraph_msgs.msg import Clock

CONTRACT_QOS = QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE)
# Mirror of the contract QoS table (docs/interfaces.md §2): these topics have
# transient_local publishers, so a late-joining recorder must ask for
# transient_local to receive events published before discovery completed
# (a volatile subscriber measurably missed the mock's first events). On
# volatile-publisher topics a transient_local subscriber would be
# INCOMPATIBLE (no data at all), hence per topic.
LATCHED_TOPICS = {'/fleet/task_update', '/fleet/robot_connection',
                  '/fleet/scene_graph_update'}
LATCHED_SUFFIXES = ('/central/scene',)      # scene_replica, one per robot namespace
CONTRACT_QOS_LATCHED = QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE,
                                  durability=DurabilityPolicy.TRANSIENT_LOCAL)


def qos_for(topic):
    """Recording QoS of a topic. Contract topics (/fleet/*) have reliable
    publishers by contract and come in bursts (mock M1 emits RECEIVED,
    PARSED, QUEUED for several tasks in one tick): a best-effort depth-5
    subscriber measurably lost events (2026-10-29), so they are recorded
    reliable with a deep queue, and so are their Module 2 echoes
    (/mock_m2/*, which ws_probe publishes through rosbridge, reliable and
    volatile). Everything else (sensors, the gz bridge) best effort."""
    if topic in LATCHED_TOPICS or topic.endswith(LATCHED_SUFFIXES):
        return CONTRACT_QOS_LATCHED
    if topic.startswith(('/fleet/', '/mock_m2/')):
        return CONTRACT_QOS
    return qos_profile_sensor_data


def hydrate(msg_class, data):
    """Build an rclpy message from a (possibly nested) dict: dicts become
    nested messages, lists of dicts message sequences, and a number of
    seconds a builtin_interfaces/Time."""
    msg = msg_class()
    slot_types = get_message_slot_types(msg)
    for key, value in data.items():
        field = getattr(msg, key)
        if isinstance(value, dict):
            value = hydrate(type(field), value)
        elif isinstance(field, Time) and isinstance(value, (int, float)):
            value = rclpy.time.Time(nanoseconds=round(value * 1e9)).to_msg()
        elif isinstance(value, list):
            element_type = getattr(slot_types[key], 'value_type', None)
            if isinstance(element_type, NamespacedType):
                element_class = import_message_from_namespaced_type(element_type)
                value = [hydrate(element_class, item) for item in value]
        setattr(msg, key, value)
    return msg


class _ScenarioPublisher:
    """A scenario `publish` action: msg at hz for duration seconds from
    start, the action's t."""

    def __init__(self, node, spec, start):
        self.spec = spec
        self.start = float(start)
        self.msg_class = get_message(spec['type'])
        self.publisher = node.create_publisher(self.msg_class, spec['topic'], 10)
        self._next_publish_time = None

    def tick(self, now):
        duration = float(self.spec.get('duration', 1))
        if now < self.start or now > self.start + duration:
            return
        if self._next_publish_time is None or now >= self._next_publish_time:
            self.publisher.publish(hydrate(self.msg_class, self.spec.get('msg', {})))
            self._next_publish_time = now + 1.0 / float(self.spec.get('hz', 10))


class Recorder(Node):
    def __init__(self):
        super().__init__('harness_recorder')
        self.streams = {}              # topic -> list[(t, msg)]
        self.clock_sec = None          # latest /clock value (sim seconds)
        self._window_start_clock = None
        self._window_start_wall = None
        # sensor-data QoS (best effort) receives from both best-effort
        # (gz bridge) and reliable (ros2 run) publishers; the reverse
        # would silently miss bridge topics.
        self.create_subscription(Clock, '/clock', self._on_clock,
                                 qos_profile_sensor_data)

    def _on_clock(self, msg):
        self.clock_sec = msg.clock.sec + msg.clock.nanosec * 1e-9

    def watch(self, topic, type_name):
        self.streams.setdefault(topic, [])
        msg_class = get_message(type_name)

        def store(msg):
            self.streams[topic].append((self.elapsed(), msg))

        self.create_subscription(msg_class, topic, store, qos_for(topic))

    def make_publisher(self, spec, start):
        return _ScenarioPublisher(self, spec, start)

    def start_window(self):
        self._window_start_wall = time.time()
        self._window_start_clock = self.clock_sec

    def window_start(self):
        """Clock time the window opened at, in the active timebase: sim
        seconds, else wall seconds since the epoch (the clock of nodes
        without use_sim_time)."""
        if self._window_start_clock is not None:
            return self._window_start_clock
        return self._window_start_wall

    def wall_elapsed(self):
        """Wall seconds since start_window (elapsed / wall = real-time factor)."""
        return time.time() - self._window_start_wall

    def elapsed(self):
        if self.clock_sec is not None:
            start = (self._window_start_clock if self._window_start_clock is not None
                     else self.clock_sec)
            return self.clock_sec - start
        start = self._window_start_wall if self._window_start_wall is not None else time.time()
        return time.time() - start
