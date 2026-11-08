"""Pure glue between ROS data and the local perception core (scene_graph,
lidar_obstacles): planar frame transforms, the lidar pose correction against
the static map, the sensor coverage that decides where unmatched tracks
decay, semantic detections in the map frame and the localization score. No
rclpy and no message imports: messages are read by attribute
(tests/unit/test_perception.py).
"""
import math

import numpy as np

from fleetgraph_robot.lidar_obstacles import OccupancyMap
from fleetgraph_robot.scene_graph import PHYSICAL, Detection

Pose = tuple[float, float, float]           # x, y, yaw of a frame in the map frame
SEMANTIC_SOURCE = 'detector'                # Detection.source of semantic detections


def to_map(pose: Pose, x: float, y: float) -> tuple[float, float]:
    """A point given in the frame at pose (x forward, y left) -> map frame."""
    origin_x, origin_y, yaw = pose
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    return (origin_x + cos_yaw * x - sin_yaw * y, origin_y + sin_yaw * x + cos_yaw * y)


class MapMatcher:
    """Corrects a lidar pose by a small correlative scan match against the
    static map (after Olson 2009): of the poses within reach of the given
    one, the one whose returns lie closest to mapped cells, ties going to the
    smallest correction; a coarse grid first, then a fine one around its best.
    AMCL runs up to 0.14 m and 3 deg off while driving (measured 2026-11-08),
    which puts wall returns into free cells, where they would read as
    obstacles; the correction leaves about 0.05 m and 1 deg."""

    CLOSENESS_RADII = (0.05, 0.1, 0.15, 0.2)    # m: a return scores per band it lies in
    # per pass: turn reach, turn step (rad), shift reach, shift step (m)
    PASSES = ((math.radians(5.0), math.radians(1.0), 0.25, 0.05),
              (math.radians(1.0), math.radians(0.25), 0.05, 0.0125))
    TURN_COST = 3.0     # m/rad: a turn costs what it moves a return 3 m away

    def __init__(self, occupancy: OccupancyMap):
        self._occupancy = occupancy
        grid = np.asarray(occupancy.data, dtype=np.int16).reshape(occupancy.height,
                                                                   occupancy.width)
        occupied = grid >= occupancy.occupied_threshold
        self._closeness = sum(_dilated(occupied, radius / occupancy.resolution).astype(np.int16)
                              for radius in self.CLOSENESS_RADII)

    def refine(self, ranges, angle_min: float, angle_increment: float, range_min: float,
               range_max: float, pose: Pose) -> Pose:
        """The corrected map pose of the lidar that took this scan from pose."""
        ranges = np.asarray(ranges, dtype=float)
        angles = angle_min + angle_increment * np.arange(len(ranges))
        valid = np.isfinite(ranges) & (ranges >= range_min) & (ranges < range_max)
        if not valid.any():
            return pose
        xs, ys = ranges[valid] * np.cos(angles[valid]), ranges[valid] * np.sin(angles[valid])
        x, y, yaw = pose
        for turn_reach, turn_step, shift_reach, shift_step in self.PASSES:
            turns = _steps(turn_reach, turn_step)
            shifts = np.array([(dx, dy) for dx in _steps(shift_reach, shift_step)
                               for dy in _steps(shift_reach, shift_step)])
            scores = np.array([self._scores(xs, ys, x, y, yaw + turn, shifts) for turn in turns])
            best = zip(*np.nonzero(scores == scores.max()))
            turn_index, shift_index = min(best, key=lambda indices: (
                self.TURN_COST * abs(turns[indices[0]]) + math.hypot(*shifts[indices[1]])))
            x, y = x + shifts[shift_index][0], y + shifts[shift_index][1]
            yaw += turns[turn_index]
        return (float(x), float(y), float(yaw))

    def _scores(self, xs, ys, x, y, yaw, shifts):
        """Closeness sum of the returns for each shift of the pose (x, y, yaw)."""
        occupancy = self._occupancy
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        map_x = x + cos_yaw * xs - sin_yaw * ys + shifts[:, :1]        # shifts x returns
        map_y = y + sin_yaw * xs + cos_yaw * ys + shifts[:, 1:]
        cols = np.floor((map_x - occupancy.origin_x) / occupancy.resolution).astype(np.int64)
        rows = np.floor((map_y - occupancy.origin_y) / occupancy.resolution).astype(np.int64)
        inside = (cols >= 0) & (cols < occupancy.width) & (rows >= 0) & (rows < occupancy.height)
        closeness = np.zeros(cols.shape, dtype=np.int16)
        closeness[inside] = self._closeness[rows[inside], cols[inside]]
        return closeness.sum(axis=1)


def _steps(reach, step):
    count = round(reach / step)
    return np.arange(-count, count + 1) * step


def _dilated(mask, reach_cells):
    """Cells within reach_cells (cell centre to cell centre) of a set cell."""
    dilated = np.zeros_like(mask)
    height, width = mask.shape
    span = math.floor(reach_cells + 1e-9)
    for row_offset in range(-span, span + 1):
        for col_offset in range(-span, span + 1):
            if row_offset ** 2 + col_offset ** 2 > reach_cells ** 2 + 1e-9:
                continue
            source = mask[max(0, -row_offset):height - max(0, row_offset),
                          max(0, -col_offset):width - max(0, col_offset)]
            dilated[max(0, row_offset):height - max(0, -row_offset),
                    max(0, col_offset):width - max(0, -col_offset)] |= source
    return dilated


def coverage(occupancy: OccupancyMap, sensor_x: float, sensor_y: float, max_range: float):
    """in_view(x, y) of one lidar frame for LocalSceneGraph.integrate: within
    max_range of the sensor and a line of sight through the STATIC map.
    Unmapped obstacles do not shadow: a track hidden behind one decays too."""
    def in_view(x, y):
        return (math.hypot(x - sensor_x, y - sensor_y) <= max_range
                and occupancy.line_of_sight(sensor_x, sensor_y, x, y))
    return in_view


def occupancy_from_grid(grid) -> OccupancyMap:
    """nav_msgs/OccupancyGrid -> OccupancyMap. ValueError for a rotated map
    origin, which OccupancyMap cannot represent."""
    info = grid.info
    orientation = info.origin.orientation
    if max(abs(orientation.x), abs(orientation.y), abs(orientation.z)) > 1e-6:
        raise ValueError(f'map origin is rotated ({orientation.x}, {orientation.y}, '
                         f'{orientation.z}, {orientation.w}); only unrotated maps are supported')
    return OccupancyMap(float(info.resolution), float(info.origin.position.x),
                        float(info.origin.position.y), int(info.width), int(info.height),
                        list(grid.data))


def semantic_detections(entities, pose: Pose, stamp: float) -> list[Detection]:
    """SceneEntity detections given in the frame at pose -> map-frame
    Detections. Drops what the graph cannot track (regions, groups,
    non-finite or negative values) and clamps confidence into [0, 1]."""
    detections = []
    for entity in entities:
        position, size = entity.position, entity.size
        values = (position.x, position.y, position.z, size.x, size.y, size.z, entity.confidence)
        if (entity.category not in PHYSICAL or not all(map(math.isfinite, values))
                or min(size.x, size.y, size.z) < 0):
            continue
        x, y = to_map(pose, position.x, position.y)
        detections.append(Detection(
            int(entity.category), x, y, z=float(position.z),
            size=(float(size.x), float(size.y), float(size.z)),
            confidence=min(1.0, max(0.0, float(entity.confidence))), label=entity.label,
            stamp=stamp, source=SEMANTIC_SOURCE))
    return detections


def localization_score(covariance) -> float:
    """AMCL pose covariance (6x6, row-major) -> 0..1, the agent_node
    heuristic: 1.0 at 0 m position std, 0.5 at 0.25 m."""
    std_xy = math.sqrt(max(covariance[0], 0.0) + max(covariance[7], 0.0))
    return 1.0 / (1.0 + std_xy / 0.25)
