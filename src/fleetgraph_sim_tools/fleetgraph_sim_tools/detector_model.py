"""Simulated object detector: ground truth in, detections relative to the
sensor out, as a camera-based detector would report them. Pure Python (no
rclpy; tests/unit/test_detector_model.py). truth_id only lets tests match
detections to truth; it must never reach robot software, which has to
associate detections on its own.

An entity is visible when it is within range and inside the field of view,
no occupied or unknown map cell lies on the line of sight to its centre,
and no nearer truth entity's footprint disc crosses that line. Visible
entities are then dropped with probability dropout; the rest get Gaussian
position noise and a clipped Gaussian confidence. All randomness comes from
the injected rng, drawn in truth order, so a seed reproduces a run.
"""
import math
import random
from collections.abc import Iterable
from dataclasses import dataclass

from fleetgraph_robot.lidar_obstacles import OccupancyMap
from fleetgraph_robot.scene_graph import Category

Size = tuple[float, float, float]
Pose = tuple[float, float, float]           # x, y, yaw in the map frame


@dataclass
class TruthEntity:
    id: str
    category: int
    label: str
    x: float                                # map frame
    y: float
    z: float = 0.0
    size: Size = (0.0, 0.0, 0.0)            # footprint disc radius max(x, y) / 2
    vx: float = 0.0
    vy: float = 0.0


@dataclass
class DetectorParams:
    fov_deg: float = 120.0
    min_range: float = 0.2
    max_range: float = 6.0
    position_sigma: float = 0.05            # m, per sensor-frame axis
    dropout: float = 0.05                   # probability a visible entity is missed
    confidence_mean: float = 0.85
    confidence_sigma: float = 0.05
    detect_categories: frozenset[int] = frozenset(
        {Category.PERSON, Category.OBJECT, Category.OBSTACLE, Category.ROBOT})

    def __post_init__(self):
        checks = [
            ('fov_deg', 0 < self.fov_deg <= 360, 'in (0, 360]'),
            ('min_range', 0 <= self.min_range < self.max_range, 'in [0, max_range)'),
            ('position_sigma', self.position_sigma >= 0, '>= 0'),
            ('dropout', 0 <= self.dropout <= 1, 'in [0, 1]'),
            ('confidence_mean', 0 <= self.confidence_mean <= 1, 'in [0, 1]'),
            ('confidence_sigma', self.confidence_sigma >= 0, '>= 0'),
        ]
        for name, ok, expected in checks:
            if not ok:
                raise ValueError(f'{name} must be {expected}, got {getattr(self, name)!r}')


@dataclass
class SimDetection:
    truth_id: str                           # tests only, never sent to the robot
    category: int
    label: str
    rel_x: float                            # sensor frame: x forward, y left
    rel_y: float
    z: float
    size: Size
    confidence: float


def simulate(truth: Iterable[TruthEntity], sensor_pose: Pose, occupancy: OccupancyMap | None,
             params: DetectorParams, rng: random.Random,
             exclude_id: str = '') -> list[SimDetection]:
    """Detections of the truth entities seen from sensor_pose, in truth
    order. exclude_id is the observing robot: never detected, never an
    occluder. occupancy None: no walls."""
    sensor_x, sensor_y, yaw = sensor_pose
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    half_fov = math.radians(params.fov_deg) / 2
    others = [entity for entity in truth if entity.id != exclude_id]
    detections = []
    for entity in others:
        if entity.category not in params.detect_categories:
            continue
        dx, dy = entity.x - sensor_x, entity.y - sensor_y
        rel_x, rel_y = cos_yaw * dx + sin_yaw * dy, cos_yaw * dy - sin_yaw * dx
        distance = math.hypot(rel_x, rel_y)
        if not params.min_range <= distance <= params.max_range:
            continue
        if abs(math.atan2(rel_y, rel_x)) > half_fov:
            continue
        if _occluded(entity, distance, others, sensor_x, sensor_y):
            continue
        if occupancy is not None and not occupancy.line_of_sight(sensor_x, sensor_y,
                                                                 entity.x, entity.y):
            continue
        if rng.random() < params.dropout:
            continue
        noisy_x = rel_x + rng.gauss(0.0, params.position_sigma)
        noisy_y = rel_y + rng.gauss(0.0, params.position_sigma)
        confidence = rng.gauss(params.confidence_mean, params.confidence_sigma)
        detections.append(SimDetection(entity.id, entity.category, entity.label, noisy_x,
                                       noisy_y, entity.z, tuple(entity.size),
                                       min(1.0, max(0.0, confidence))))
    return detections


def _occluded(target, distance, others, sensor_x, sensor_y):
    """A nearer entity's footprint disc crosses the line of sight to the
    target's centre."""
    for other in others:
        radius = max(other.size[0], other.size[1]) / 2
        if (other is target or radius <= 0
                or math.hypot(other.x - sensor_x, other.y - sensor_y) >= distance):
            continue
        if _distance_to_segment(other.x, other.y, sensor_x, sensor_y,
                                target.x, target.y) < radius:
            return True
    return False


def _distance_to_segment(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    length_squared = dx * dx + dy * dy
    t = 0.0 if length_squared == 0 else ((px - ax) * dx + (py - ay) * dy) / length_squared
    t = min(1.0, max(0.0, t))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))
