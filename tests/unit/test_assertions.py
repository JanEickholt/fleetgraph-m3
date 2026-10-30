"""Harness self-tests, tier 1: assertion registry over canned recordings.
Proves every assertion type both passes and fails correctly, with honest
failure messages. Pure — no rclpy, no sim.
"""
import math
from types import SimpleNamespace

import assertions


class FakeRecording:
    def __init__(self, streams, duration):
        self.streams = streams
        self._duration = duration

    def stream(self, topic):
        return self.streams.get(topic, [])

    def duration(self):
        return self._duration


def pose_msg(x, y):
    # mirrors nav_msgs/msg/Odometry depth: msg.pose.pose.position
    return SimpleNamespace(pose=SimpleNamespace(
        pose=SimpleNamespace(position=SimpleNamespace(x=x, y=y))))


def scan_msg(ranges):
    return SimpleNamespace(ranges=list(ranges), range_min=0.12)


def twist_msg(x):
    return SimpleNamespace(linear=SimpleNamespace(x=x))


EMPTY = FakeRecording({}, 10.0)


def test_publishes_pass_and_fail():
    recording = FakeRecording({'/chatter': [(0.0, 'm')]}, 10.0)
    assert assertions.publishes(recording, topic='/chatter') is None
    failure = assertions.publishes(EMPTY, topic='/chatter')
    assert failure and 'no message' in failure


def test_min_rate_pass_and_fail():
    # 20 messages, 0.5s apart -> true rate 2.0 Hz (inter-arrival estimate)
    recording = FakeRecording({'/odom': [(index * 0.5, index) for index in range(20)]}, 10.0)
    assert assertions.min_rate(recording, topic='/odom', hz=1.5) is None
    failure = assertions.min_rate(recording, topic='/odom', hz=3.0)
    assert failure and '2.00 Hz' in failure and 'nominal 3.0' in failure
    # margin override: 0 disables harness fudge
    failure = assertions.min_rate(recording, topic='/odom', hz=2.5, margin=0.0)
    assert failure and '2.00 Hz' in failure
    assert assertions.min_rate(recording, topic='/odom', hz=1.9, margin=0.0) is None
    failure = assertions.min_rate(EMPTY, topic='/odom', hz=1.0)
    assert failure and 'no messages' in failure
    single = FakeRecording({'/odom': [(1.0, 'm')]}, 10.0)
    failure = assertions.min_rate(single, topic='/odom', hz=1.0)
    assert failure and 'cannot estimate rate' in failure


def test_arrives_at_pass_and_fail():
    inside = FakeRecording({'/odom': [(1.0, pose_msg(1.95, 1.05))]}, 10.0)
    assert assertions.arrives_at(inside, topic='/odom', x=2.0, y=1.0, tol=0.2) is None
    outside = FakeRecording({'/odom': [(1.0, pose_msg(1.5, 1.5))]}, 10.0)
    failure = assertions.arrives_at(outside, topic='/odom', x=2.0, y=1.0, tol=0.2)
    assert failure and 'never came within' in failure and 'distance' in failure
    failure = assertions.arrives_at(EMPTY, topic='/odom', x=2.0, y=1.0, tol=0.2)
    assert failure and 'no messages' in failure


def test_never_below_scalar_and_array():
    safe = FakeRecording({'/scan': [(1.0, scan_msg([1.0, 0.3]))]}, 10.0)
    assert assertions.never_below(safe, topic='/scan', field='ranges', threshold=0.25) is None
    hit = FakeRecording({'/scan': [(1.0, scan_msg([1.0, 0.2]))]}, 10.0)
    failure = assertions.never_below(hit, topic='/scan', field='ranges', threshold=0.25)
    assert failure and '0.200' in failure
    # scalar field path
    assert assertions.never_below(safe, topic='/scan', field='range_min', threshold=0.1) is None
    failure = assertions.never_below(safe, topic='/scan', field='range_min', threshold=0.15)
    assert failure and '0.120' in failure


def test_never_below_nan_fails_closed():
    # NaN compares False against everything — a bare `value < threshold`
    # would silently pass a NaN sample. Must be treated as violation.
    nan_scan = FakeRecording({'/scan': [(1.0, scan_msg([1.0, float('nan')]))]}, 10.0)
    failure = assertions.never_below(nan_scan, topic='/scan', field='ranges', threshold=0.25)
    assert failure and 'NaN' in failure


def test_final_value_pass_and_fail():
    recording = FakeRecording({'/cmd_vel': [(1.0, twist_msg(0.3)), (2.0, twist_msg(0.0))]}, 10.0)
    assert assertions.final_value(recording, topic='/cmd_vel', field='linear.x', equals=0.0) is None
    nonzero = FakeRecording({'/cmd_vel': [(2.0, twist_msg(0.4))]}, 10.0)
    failure = assertions.final_value(nonzero, topic='/cmd_vel', field='linear.x', equals=0.0, tol=0.01)
    assert failure and '0.4' in failure
    failure = assertions.final_value(EMPTY, topic='/cmd_vel', field='linear.x', equals=0.0)
    assert failure and 'no messages' in failure


def quat_msg(deg):
    # yaw-only quaternion for heading `deg` (x=y=0)
    half_yaw = math.radians(deg) / 2.0
    return SimpleNamespace(pose=SimpleNamespace(pose=SimpleNamespace(
        position=SimpleNamespace(x=0.0, y=0.0),
        orientation=SimpleNamespace(x=0.0, y=0.0,
                                    z=math.sin(half_yaw), w=math.cos(half_yaw)))))


def test_yaw_final_pass_and_fail():
    recording = FakeRecording({'/odom': [(0.0, quat_msg(90.0)), (1.0, quat_msg(90.0))]}, 2.0)
    assert assertions.yaw_final(recording, topic='/odom', deg=90.0, tol=2.0) is None
    # target missed by 10 deg with tol 2 -> fail, message carries numbers
    failure = assertions.yaw_final(recording, topic='/odom', deg=100.0, tol=2.0)
    assert failure and 'final yaw 90.00' in failure and '10.00 deg > 2.0' in failure
    # wrap-around: 350 deg final vs -10 deg target is a 0-deg error
    recording_350 = FakeRecording({'/odom': [(0.0, quat_msg(350.0))]}, 1.0)
    assert assertions.yaw_final(recording_350, topic='/odom', deg=-10.0, tol=1.0) is None
    # empty stream -> honest failure
    failure = assertions.yaw_final(EMPTY, topic='/odom', deg=0.0, tol=1.0)
    assert failure and 'no messages' in failure


def test_registry_covers_contract():
    assert set(assertions.REGISTRY) == {
        'publishes', 'min_rate', 'arrives_at', 'never_below', 'final_value',
        'yaw_final', 'task_state', 'task_sequence', 'seq_contiguous'}
    for name, function in assertions.REGISTRY.items():
        assert function.__name__ == name
