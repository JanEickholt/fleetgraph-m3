"""Simulated battery for the robot agent until a real battery source exists.
Pure Python (no rclpy), unit-tested in tests/unit/test_battery.py.

The level (1.0 = full) drains by distance driven (moved_to(), fed from
/odom) and by time (tick(), fed with sim seconds). It never drops below 0.
"""
import math


class BatteryModel:
    def __init__(self, initial, drain_per_m, drain_per_s):
        self.level = float(initial)
        self.drain_per_m = drain_per_m
        self.drain_per_s = drain_per_s
        self._last_xy = None
        self._last_tick_sec = None

    def moved_to(self, x, y):
        """Drain for the distance since the previous position."""
        if self._last_xy is not None:
            last_x, last_y = self._last_xy
            self._drain(math.hypot(x - last_x, y - last_y) * self.drain_per_m)
        self._last_xy = (x, y)

    def tick(self, now_sec):
        """Drain for the time since the previous tick."""
        if self._last_tick_sec is not None:
            self._drain((now_sec - self._last_tick_sec) * self.drain_per_s)
        self._last_tick_sec = now_sec

    def _drain(self, amount):
        self.level = max(0.0, self.level - amount)
