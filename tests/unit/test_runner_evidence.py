"""Harness self-tests: run evidence (tests/artifacts/<name>.json) computed
from a canned recording, no sim.
"""
from types import SimpleNamespace

from runner import _evidence


def odom_msg(x, y):
    return SimpleNamespace(pose=SimpleNamespace(
        pose=SimpleNamespace(position=SimpleNamespace(x=x, y=y))))


def recording(clock_sec, elapsed, streams=None):
    return SimpleNamespace(clock_sec=clock_sec, elapsed=lambda: elapsed,
                           streams=streams or {})


def test_real_time_factor():
    # 60 sim seconds took 100 wall seconds
    evidence = _evidence(recording(160.0, 60.0), [], wall_sec=100.0)
    assert evidence['real_time_factor'] == 0.6
    assert evidence['window_wall_sec'] == 100.0
    # no /clock (no-sim scenario): the timebase is wall time, nothing to report
    assert 'real_time_factor' not in _evidence(recording(None, 6.0), [], wall_sec=6.0)


def test_last_position_per_odometry_topic():
    streams = {'/ground_truth/robot_2/odom': [(1.0, odom_msg(2.8, -2.3)),
                                              (9.0, odom_msg(6.5, -1.5))],
               '/robot_1/odom': []}
    evidence = _evidence(recording(10.0, 10.0, streams), [], wall_sec=10.0)
    assert evidence['last_ground_truth_robot_2_odom'] == [6.5, -1.5]
    assert 'last_robot_1_odom' not in evidence
