"""Obstacles the static map does not explain, from one lidar scan. Pure
Python (no rclpy): the ROS node passes each LaserScan with the sensor pose in
the map frame and feeds the returned detections to the local scene graph
(tests/unit/test_lidar_obstacles.py).

A return is evidence of something new only where the map says free: in a
known free cell, farther than a tolerance from every static occupied cell.
Returns in unknown cells or off the map prove nothing either way. The new
returns are clustered by single linkage; each cluster becomes one OBSTACLE
detection with label 'unknown' and source 'lidar', which the scene graph
attaches to whatever track is there, or tracks as a new obstacle.
"""
import math
from collections.abc import Iterable, Sequence
from dataclasses import dataclass, field

from fleetgraph_robot.scene_graph import LIDAR_SOURCE, Category, Detection

Point = tuple[float, float]
Pose = tuple[float, float, float]           # x, y, yaw in the map frame
MIN_SIZE = 0.1                              # m, smallest obstacle extent reported
# Slack on distance and ray-parameter comparisons, so float noise (0.15 / 0.05
# = 2.9999999999999996) cannot drop a cell or miss an exact corner crossing.
_EPSILON = 1e-9


@dataclass
class OccupancyMap:
    """nav_msgs/OccupancyGrid semantics: data is row-major starting at the
    cell whose corner is (origin_x, origin_y); values 0..100, -1 unknown.
    The map origin must not be rotated (true for map_server maps of this
    repo). data must not change afterwards: dilations are cached."""
    resolution: float
    origin_x: float
    origin_y: float
    width: int
    height: int
    data: Sequence[int]
    occupied_threshold: int = 65
    _dilated: dict[float, bytearray] = field(default_factory=dict, init=False, repr=False,
                                             compare=False)

    def __post_init__(self):
        if not self.resolution > 0:
            raise ValueError(f'resolution must be > 0, got {self.resolution!r}')
        if self.width <= 0 or self.height <= 0:
            raise ValueError(f'map needs width, height > 0, got {self.width} x {self.height}')
        if len(self.data) != self.width * self.height:
            raise ValueError(f'data has {len(self.data)} cells, '
                             f'expected {self.width} x {self.height}')
        if not 0 < self.occupied_threshold <= 100:
            raise ValueError(f'occupied_threshold must be in 1..100, '
                             f'got {self.occupied_threshold!r}')

    def world_to_cell(self, x: float, y: float) -> tuple[int, int] | None:
        """(col, row) of the cell containing the point, None off the map."""
        if not (math.isfinite(x) and math.isfinite(y)):
            return None
        col = math.floor((x - self.origin_x) / self.resolution)
        row = math.floor((y - self.origin_y) / self.resolution)
        if 0 <= col < self.width and 0 <= row < self.height:
            return col, row
        return None

    def is_occupied(self, x: float, y: float) -> bool:
        cell = self.world_to_cell(x, y)
        return cell is not None and self._value(*cell) >= self.occupied_threshold

    def near_occupied(self, x: float, y: float, radius: float) -> bool:
        """An occupied cell within radius, measured between cell centres, so
        the answer is exact to one cell. Answered from a dilated copy of the
        map, computed once per radius."""
        if not radius >= 0:
            raise ValueError(f'radius must be >= 0, got {radius!r}')
        cell = self.world_to_cell(x, y)
        if cell is None:
            return False
        dilated = self._dilated.get(radius)
        if dilated is None:
            dilated = self._dilated[radius] = self._dilate(radius)
        col, row = cell
        return bool(dilated[row * self.width + col])

    def line_of_sight(self, x0: float, y0: float, x1: float, y1: float) -> bool:
        """No occupied or unknown cell, and no map border, strictly between
        the endpoints: the endpoint cells themselves are not tested. Exact
        grid traversal (Amanatides & Woo). Where the segment passes exactly
        through a cell corner, both cells beside it count, so the answer is
        the same in both directions."""
        start, end = self.world_to_cell(x0, y0), self.world_to_cell(x1, y1)
        if start is None or end is None:
            return False
        (col, row), (end_col, end_row) = start, end
        gx, gy = (x0 - self.origin_x) / self.resolution, (y0 - self.origin_y) / self.resolution
        dx, dy = (x1 - x0) / self.resolution, (y1 - y0) / self.resolution
        step_col, step_row = (dx > 0) - (dx < 0), (dy > 0) - (dy < 0)
        # segment parameter t (0..1) of the next column / row boundary, and per cell
        next_col_t, col_t = _first_crossing(gx, col, dx), (abs(1 / dx) if dx else math.inf)
        next_row_t, row_t = _first_crossing(gy, row, dy), (abs(1 / dy) if dy else math.inf)
        # Each step moves toward the end cell, so this ends after at most
        # |d col| + |d row| steps even when float error skews the t values.
        while (col, row) != (end_col, end_row):
            if col != end_col and row != end_row and abs(next_col_t - next_row_t) <= _EPSILON:
                if self._opaque(col + step_col, row) or self._opaque(col, row + step_row):
                    return False
                col, row = col + step_col, row + step_row
                next_col_t, next_row_t = next_col_t + col_t, next_row_t + row_t
            elif row == end_row or (col != end_col and next_col_t < next_row_t):
                col, next_col_t = col + step_col, next_col_t + col_t
            else:
                row, next_row_t = row + step_row, next_row_t + row_t
            if (col, row) != (end_col, end_row) and self._opaque(col, row):
                return False
        return True

    def _value(self, col, row):
        return self.data[row * self.width + col]

    def _opaque(self, col, row):
        value = self._value(col, row)
        return value < 0 or value >= self.occupied_threshold

    def _dilate(self, radius):
        width, height = self.width, self.height
        reach = radius / self.resolution + _EPSILON          # in cells
        # per row offset, the half-width of the disc of cell centres in reach
        spans = [(row_offset, math.floor(math.sqrt(reach * reach - row_offset * row_offset)))
                 for row_offset in range(-math.floor(reach), math.floor(reach) + 1)]
        dilated = bytearray(width * height)
        ones = b'\x01' * width
        for index, value in enumerate(self.data):
            if value < self.occupied_threshold:
                continue
            row, col = divmod(index, width)
            for row_offset, half_width in spans:
                target = row + row_offset
                if 0 <= target < height:
                    first, last = max(col - half_width, 0), min(col + half_width, width - 1)
                    dilated[target * width + first:target * width + last + 1] = \
                        ones[:last - first + 1]
        return dilated


def _first_crossing(start, cell, delta):
    if delta > 0:
        return (cell + 1 - start) / delta
    if delta < 0:
        return (cell - start) / delta
    return math.inf


def scan_to_points(ranges: Iterable[float], angle_min: float, angle_increment: float,
                   range_min: float, range_max: float, sensor_pose: Pose) -> list[Point]:
    """LaserScan returns as map-frame points. A reading at range_max or
    beyond is a no-return, as in laser_geometry."""
    sensor_x, sensor_y, yaw = sensor_pose
    points = []
    for index, distance in enumerate(ranges):
        if math.isfinite(distance) and range_min <= distance < range_max:
            angle = yaw + angle_min + index * angle_increment
            points.append((sensor_x + distance * math.cos(angle),
                           sensor_y + distance * math.sin(angle)))
    return points


def unexplained_points(points: Iterable[Point], occupancy: OccupancyMap,
                       tolerance: float = 0.15) -> list[Point]:
    """Returns in known free cells farther than tolerance from every static
    occupied cell: free -> occupied evidence."""
    unexplained = []
    for x, y in points:
        cell = occupancy.world_to_cell(x, y)
        if (cell is not None and occupancy._value(*cell) >= 0
                and not occupancy.near_occupied(x, y, tolerance)):
            unexplained.append((x, y))
    return unexplained


@dataclass
class Cluster:
    x: float                # centroid
    y: float
    radius: float           # farthest member from the centroid
    count: int


def cluster_points(points: Iterable[Point], eps: float = 0.2,
                   min_points: int = 3) -> list[Cluster]:
    """Single linkage: points chained by distances <= eps form one cluster;
    clusters of fewer than min_points are noise. Ordered by first point."""
    clusters = []
    for members in _linked(points, eps, min_points):
        count = len(members)
        x, y = sum(px for px, _ in members) / count, sum(py for _, py in members) / count
        radius = max(math.hypot(px - x, py - y) for px, py in members)
        clusters.append(Cluster(x, y, radius, count))
    return clusters


def obstacle_detections(ranges: Iterable[float], angle_min: float, angle_increment: float,
                        range_min: float, range_max: float, sensor_pose: Pose,
                        occupancy: OccupancyMap, stamp: float, tolerance: float = 0.15,
                        eps: float = 0.2, min_points: int = 3) -> list[Detection]:
    """One OBSTACLE detection per cluster of unexplained returns: a disc as
    wide as the cluster across the line of sight, its near edge at the
    closest return, at least MIN_SIZE across. Confidence n / (n + min_points)
    for n returns: 0.5 for the smallest cluster kept, approaching 1 as
    evidence accumulates."""
    points = scan_to_points(ranges, angle_min, angle_increment, range_min, range_max,
                            sensor_pose)
    detections = []
    for members in _linked(unexplained_points(points, occupancy, tolerance), eps, min_points):
        x, y, radius = _facing_disc(members, sensor_pose)
        count = len(members)
        detections.append(Detection(
            Category.OBSTACLE, x, y, size=(2 * radius, 2 * radius, 0.0),
            confidence=count / (count + min_points), label='unknown', stamp=stamp,
            source=LIDAR_SOURCE))
    return detections


def _linked(points, eps, min_points):
    """Single-linkage groups of at least min_points points, in order of their
    first point. Points are binned in eps cells, so only neighbouring bins
    are compared."""
    if not eps > 0:
        raise ValueError(f'eps must be > 0, got {eps!r}')
    if min_points < 1:
        raise ValueError(f'min_points must be >= 1, got {min_points!r}')
    points = list(points)
    bins: dict[tuple[int, int], list[int]] = {}
    for index, (x, y) in enumerate(points):
        bins.setdefault((math.floor(x / eps), math.floor(y / eps)), []).append(index)
    parent = list(range(len(points)))

    def root(index):
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = parent[index]
        return index

    for (bin_x, bin_y), members in bins.items():
        for neighbour_x in (bin_x - 1, bin_x, bin_x + 1):
            for neighbour_y in (bin_y - 1, bin_y, bin_y + 1):
                for other in bins.get((neighbour_x, neighbour_y), ()):
                    for index in members:
                        if index < other and math.dist(points[index], points[other]) <= eps:
                            parent[root(index)] = root(other)
    groups: dict[int, list[Point]] = {}
    for index, point in enumerate(points):
        groups.setdefault(root(index), []).append(point)
    return [members for members in groups.values() if len(members) >= min_points]


def _facing_disc(points, sensor_pose):
    """(x, y, radius) of the disc as wide as the cluster across the line of
    sight, with its near edge at the closest return. The lidar sees only the
    near surface, so the centroid sits up to half the object's depth too
    close; this recovers the centre of a disc, or of a square at any angle,
    to within the beam spacing when seen from afar."""
    sensor_x, sensor_y, _ = sensor_pose
    count = len(points)
    mean_x = sum(x for x, _ in points) / count - sensor_x
    mean_y = sum(y for _, y in points) / count - sensor_y
    distance = math.hypot(mean_x, mean_y)
    if distance < _EPSILON:                 # wraps around the sensor: no direction
        radius = max(math.hypot(x - sensor_x, y - sensor_y) for x, y in points)
        return sensor_x, sensor_y, max(radius, MIN_SIZE / 2)
    along_x, along_y = mean_x / distance, mean_y / distance
    depths = [(x - sensor_x) * along_x + (y - sensor_y) * along_y for x, y in points]
    offsets = [(y - sensor_y) * along_x - (x - sensor_x) * along_y for x, y in points]
    radius = max((max(offsets) - min(offsets)) / 2, MIN_SIZE / 2)
    depth, offset = min(depths) + radius, (max(offsets) + min(offsets)) / 2
    return (sensor_x + depth * along_x - offset * along_y,
            sensor_y + depth * along_y + offset * along_x, radius)
