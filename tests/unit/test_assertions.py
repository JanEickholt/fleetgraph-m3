"""Harness self-tests, tier 1: assertion registry over canned recordings.
Proves every assertion type both passes and fails correctly, with honest
failure messages. Pure — no rclpy, no sim.
"""
import math
from types import SimpleNamespace

import pytest

import assertions
from runner import View  # duck-typed: any object with stream()/duration()


class FakeRec:
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


EMPTY = FakeRec({}, 10.0)


def test_publishes_pass_and_fail():
    rec = FakeRec({'/chatter': [(0.0, 'm')]}, 10.0)
    assert assertions.publishes(rec, topic='/chatter') is None
    err = assertions.publishes(EMPTY, topic='/chatter')
    assert err and 'no message' in err


def test_min_rate_pass_and_fail():
    # 20 messages, 0.5s apart -> true rate 2.0 Hz (inter-arrival estimate)
    rec = FakeRec({'/odom': [(i * 0.5, i) for i in range(20)]}, 10.0)
    assert assertions.min_rate(rec, topic='/odom', hz=1.5) is None
    err = assertions.min_rate(rec, topic='/odom', hz=3.0)
    assert err and '2.00 Hz' in err and 'nominal 3.0' in err
    # margin override: 0 disables harness fudge
    err = assertions.min_rate(rec, topic='/odom', hz=2.5, margin=0.0)
    assert err and '2.00 Hz' in err
    assert assertions.min_rate(rec, topic='/odom', hz=1.9, margin=0.0) is None
    err = assertions.min_rate(EMPTY, topic='/odom', hz=1.0)
    assert err and 'no messages' in err
    one = FakeRec({'/odom': [(1.0, 'm')]}, 10.0)
    err = assertions.min_rate(one, topic='/odom', hz=1.0)
    assert err and 'cannot estimate rate' in err


def test_arrives_at_pass_and_fail():
    inside = FakeRec({'/odom': [(1.0, pose_msg(1.95, 1.05))]}, 10.0)
    assert assertions.arrives_at(inside, topic='/odom', x=2.0, y=1.0, tol=0.2) is None
    outside = FakeRec({'/odom': [(1.0, pose_msg(1.5, 1.5))]}, 10.0)
    err = assertions.arrives_at(outside, topic='/odom', x=2.0, y=1.0, tol=0.2)
    assert err and 'never came within' in err and 'distance' in err
    err = assertions.arrives_at(EMPTY, topic='/odom', x=2.0, y=1.0, tol=0.2)
    assert err and 'no messages' in err


def test_never_below_scalar_and_array():
    safe = FakeRec({'/scan': [(1.0, scan_msg([1.0, 0.3]))]}, 10.0)
    assert assertions.never_below(safe, topic='/scan', field='ranges', threshold=0.25) is None
    hit = FakeRec({'/scan': [(1.0, scan_msg([1.0, 0.2]))]}, 10.0)
    err = assertions.never_below(hit, topic='/scan', field='ranges', threshold=0.25)
    assert err and '0.200' in err
    # scalar field path
    assert assertions.never_below(safe, topic='/scan', field='range_min', threshold=0.1) is None
    err = assertions.never_below(safe, topic='/scan', field='range_min', threshold=0.15)
    assert err and '0.120' in err


def test_never_below_nan_fails_closed():
    # NaN compares False against everything — a bare `value < threshold`
    # would silently pass a NaN sample. Must be treated as violation.
    nan_scan = FakeRec({'/scan': [(1.0, scan_msg([1.0, float('nan')]))]}, 10.0)
    err = assertions.never_below(nan_scan, topic='/scan', field='ranges', threshold=0.25)
    assert err and 'NaN' in err


def test_final_value_pass_and_fail():
    rec = FakeRec({'/cmd_vel': [(1.0, twist_msg(0.3)), (2.0, twist_msg(0.0))]}, 10.0)
    assert assertions.final_value(rec, topic='/cmd_vel', field='linear.x', equals=0.0) is None
    nonzero = FakeRec({'/cmd_vel': [(2.0, twist_msg(0.4))]}, 10.0)
    err = assertions.final_value(nonzero, topic='/cmd_vel', field='linear.x', equals=0.0, tol=0.01)
    assert err and '0.4' in err
    err = assertions.final_value(EMPTY, topic='/cmd_vel', field='linear.x', equals=0.0)
    assert err and 'no messages' in err




def quat_msg(deg):
    # yaw-only quaternion for heading `deg` (x=y=0)
    r = math.radians(deg) / 2.0
    return SimpleNamespace(pose=SimpleNamespace(pose=SimpleNamespace(
        position=SimpleNamespace(x=0.0, y=0.0),
        orientation=SimpleNamespace(x=0.0, y=0.0,
                                    z=math.sin(r), w=math.cos(r)))))


def test_yaw_final_pass_and_fail():
    rec = FakeRec({'/odom': [(0.0, quat_msg(90.0)), (1.0, quat_msg(90.0))]}, 2.0)
    assert assertions.yaw_final(rec, topic='/odom', deg=90.0, tol=2.0) is None
    # target missed by 10 deg with tol 2 -> fail, message carries numbers
    err = assertions.yaw_final(rec, topic='/odom', deg=100.0, tol=2.0)
    assert err and 'final yaw 90.00' in err and '10.00 deg > 2.0' in err
    # wrap-around: 350 deg final vs -10 deg target is a 0-deg error
    rec350 = FakeRec({'/odom': [(0.0, quat_msg(350.0))]}, 1.0)
    assert assertions.yaw_final(rec350, topic='/odom', deg=-10.0, tol=1.0) is None
    # empty stream -> honest failure
    err = assertions.yaw_final(EMPTY, topic='/odom', deg=0.0, tol=1.0)
    assert err and 'no messages' in err

def test_registry_covers_contract():
    assert set(assertions.REGISTRY) == {
        'publishes', 'min_rate', 'arrives_at', 'never_below', 'final_value',
        'yaw_final', 'task_state', 'task_sequence', 'seq_contiguous'}
    for name, fn in assertions.REGISTRY.items():
        assert fn.__name__ == name
