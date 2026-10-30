"""Simulated battery (fleetgraph_robot.battery). Pure — no rclpy, no sim.
Covers distance drain, idle drain over time, and the floor at 0.
"""
import pytest

from fleetgraph_robot.battery import BatteryModel


def make_battery(initial=1.0, drain_per_m=0.01, drain_per_s=0.001):
    return BatteryModel(initial, drain_per_m=drain_per_m, drain_per_s=drain_per_s)


def test_starts_at_initial_level():
    assert make_battery(initial=0.8).level == 0.8


def test_first_position_only_sets_reference():
    battery = make_battery()
    battery.moved_to(5.0, 5.0)
    assert battery.level == 1.0


def test_drains_per_metre_driven():
    battery = make_battery(drain_per_m=0.01)
    battery.moved_to(0.0, 0.0)
    battery.moved_to(3.0, 4.0)                  # 5 m
    battery.moved_to(3.0, 4.0)                  # standing still
    assert battery.level == pytest.approx(0.95)


def test_first_tick_only_sets_reference():
    battery = make_battery()
    battery.tick(100.0)
    assert battery.level == 1.0


def test_drains_per_second_elapsed():
    battery = make_battery(drain_per_s=0.001)
    battery.tick(10.0)
    battery.tick(30.0)                          # 20 s
    assert battery.level == pytest.approx(0.98)


def test_never_drops_below_zero():
    battery = make_battery(initial=0.01, drain_per_m=1.0, drain_per_s=1.0)
    battery.moved_to(0.0, 0.0)
    battery.moved_to(10.0, 0.0)
    assert battery.level == 0.0
    battery.tick(0.0)
    battery.tick(10.0)
    assert battery.level == 0.0
