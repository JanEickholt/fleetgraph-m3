"""Topic recorder: subscribes to declared topics, stores (t, msg) streams.

Timebase: sim seconds from /clock when Gazebo publishes it, wall seconds
otherwise (no-sim scenarios). All deadlines and rates are computed in the
active timebase, never wall time when a sim is present.
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from rosidl_runtime_py.utilities import get_message
from rosgraph_msgs.msg import Clock

CONTRACT_QOS = QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE)
# Mirror of the contract QoS table (docs/interfaces.md §2): these topics have
# transient_local publishers, so a late-joining recorder must ask for
# transient_local to receive events published before discovery completed
# (a volatile subscriber measurably missed the mock's first events). On
# volatile-publisher topics a transient_local subscriber would be
# INCOMPATIBLE (no data at all), hence per topic.
LATCHED_TOPICS = {'/fleet/task_update', '/fleet/robot_connection'}
CONTRACT_QOS_LATCHED = QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE,
                                  durability=DurabilityPolicy.TRANSIENT_LOCAL)


def hydrate(cls, data):
    """Build an rclpy message from a (possibly nested) dict."""
    msg = cls()
    for key, value in data.items():
        if isinstance(value, dict):
            setattr(msg, key, hydrate(type(getattr(msg, key)), value))
        else:
            setattr(msg, key, value)
    return msg


class _Publisher:
    def __init__(self, node, spec):
        self.spec = spec
        self.cls = get_message(spec['type'])
        self.pub = node.create_publisher(self.cls, spec['topic'], 10)
        self._next_t = None

    def tick(self, now):
        start = float(self.spec.get('t', 0))
        duration = float(self.spec.get('duration', 1))
        if now < start or now > start + duration:
            return
        if self._next_t is None or now >= self._next_t:
            self.pub.publish(hydrate(self.cls, self.spec.get('msg', {})))
            self._next_t = now + 1.0 / float(self.spec.get('hz', 10))


class Recorder(Node):
    def __init__(self):
        super().__init__('harness_recorder')
        self.streams = {}          # topic -> list[(t, msg)]
        self.clock_t = None        # latest /clock value (sim seconds)
        self._clock_t0 = None
        self._wall_t0 = None
        # sensor-data QoS (best effort) receives from both best-effort
        # (gz bridge) and reliable (ros2 run) publishers; the reverse
        # would silently miss bridge topics.
        self.create_subscription(Clock, '/clock', self._on_clock,
                                 qos_profile_sensor_data)

    def _on_clock(self, msg):
        self.clock_t = msg.clock.sec + msg.clock.nanosec * 1e-9

    def watch(self, topic, type_name):
        self.streams.setdefault(topic, [])
        cls = get_message(type_name)

        def store(m, _topic=topic):
            self.streams[_topic].append((self.elapsed(), m))

        # Contract topics (/fleet/*) have reliable publishers by contract and
        # come in bursts (mock M1 emits RECEIVED/PARSED/QUEUED for several
        # tasks in one tick): a best-effort depth-5 subscriber measurably
        # lost events (2026-10-29). Record them reliable with a deep queue.
        if topic in LATCHED_TOPICS:
            qos = CONTRACT_QOS_LATCHED
        elif topic.startswith('/fleet/'):
            qos = CONTRACT_QOS
        else:
            qos = qos_profile_sensor_data
        self.create_subscription(cls, topic, store, qos)

    def make_publisher(self, spec):
        return _Publisher(self, spec)

    def start_window(self):
        self._wall_t0 = time.time()
        self._clock_t0 = self.clock_t

    def elapsed(self):
        if self.clock_t is not None:
            base = self._clock_t0 if self._clock_t0 is not None else self.clock_t
            return self.clock_t - base
        base = self._wall_t0 if self._wall_t0 is not None else time.time()
        return time.time() - base
