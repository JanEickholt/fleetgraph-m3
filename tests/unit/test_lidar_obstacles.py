"""Lidar obstacle extraction (fleetgraph_robot.lidar_obstacles). Pure — no
rclpy, no sim. Covers occupancy-grid lookups, exact line of sight, scan
projection, explaining hits by the static map, clustering, and the OBSTACLE
detections it feeds to the scene graph, on a synthetic room and on the lab
world of tools/gen_world.py.
"""
import importlib.util
import math
import os
import random
import statistics
import time

import pytest

from fleetgraph_robot.lidar_obstacles import (
    OccupancyMap, cluster_points, obstacle_detections, scan_to_points, unexplained_points)
from fleetgraph_robot.scene_graph import Category

RESOLUTION = 0.05
FREE, OCCUPIED, UNKNOWN = 0, 100, -1
# Room x 0..4, y 0..3 with 0.1 m walls and a wall stub along x = 2 from the
# south wall up to y = 1.5; unknown outside. Boxes: (xmin, ymin, xmax, ymax).
WALLS = [(-0.05, -0.05, 4.05, 0.05), (-0.05, 2.95, 4.05, 3.05),
         (-0.05, -0.05, 0.05, 3.05), (3.95, -0.05, 4.05, 3.05),
         (1.95, 0.05, 2.05, 1.5)]
BUILDING = (-0.05, -0.05, 4.05, 3.05)
SCAN_LIMITS = (0.12, 8.0)                   # range_min, range_max of the robot's lidar


def rasterize(boxes, building, origin, width, height, resolution=RESOLUTION):
    """As tools/gen_world.py: a cell is occupied when its centre lies in a box."""
    data = []
    for row in range(height):
        y = origin[1] + (row + 0.5) * resolution
        for col in range(width):
            x = origin[0] + (col + 0.5) * resolution
            if any(xmin <= x <= xmax and ymin <= y <= ymax for xmin, ymin, xmax, ymax in boxes):
                data.append(OCCUPIED)
            elif not (building[0] <= x <= building[2] and building[1] <= y <= building[3]):
                data.append(UNKNOWN)
            else:
                data.append(FREE)
    return OccupancyMap(resolution, origin[0], origin[1], width, height, data)


ROOM = rasterize(WALLS, BUILDING, origin=(-0.5, -0.5), width=100, height=80)


def square(x, y, side):
    return (x - side / 2, y - side / 2, x + side / 2, y + side / 2)


def ray_box(x, y, dx, dy, box):
    """Distance along the unit ray to its entry into the box (slab method)."""
    near, far = 0.0, math.inf
    for origin, direction, low, high in ((x, dx, box[0], box[2]), (y, dy, box[1], box[3])):
        if abs(direction) < 1e-12:
            if not low <= origin <= high:
                return None
            continue
        first, second = (low - origin) / direction, (high - origin) / direction
        near, far = max(near, min(first, second)), min(far, max(first, second))
    return near if near <= far else None


def raycast(pose, boxes, beams=360, max_range=SCAN_LIMITS[1]):
    """Full-circle scan of the boxes: (angle_min, angle_increment, ranges),
    inf where nothing is hit within max_range."""
    x, y, yaw = pose
    angle_min, increment = -math.pi, 2 * math.pi / beams
    ranges = []
    for index in range(beams):
        angle = yaw + angle_min + index * increment
        hits = [distance for box in boxes
                if (distance := ray_box(x, y, math.cos(angle), math.sin(angle), box)) is not None]
        nearest = min(hits, default=math.inf)
        ranges.append(nearest if nearest < max_range else math.inf)
    return angle_min, increment, ranges


def detect(pose, boxes, occupancy, **options):
    angle_min, increment, ranges = raycast(pose, boxes)
    return obstacle_detections(ranges, angle_min, increment, *SCAN_LIMITS, pose, occupancy,
                               stamp=options.pop('stamp', 0.0), **options)


# ------------------------------------------------------------ occupancy map

def test_world_to_cell_follows_the_occupancy_grid_layout():
    grid = OccupancyMap(0.5, -1.0, 2.0, 4, 3, [FREE] * 12)
    assert grid.world_to_cell(-1.0, 2.0) == (0, 0)         # origin = corner of cell 0
    assert grid.world_to_cell(0.74, 3.2) == (3, 2)
    assert grid.world_to_cell(1.0, 2.5) is None             # x = origin + width * resolution
    assert grid.world_to_cell(-1.01, 2.5) is None
    assert grid.world_to_cell(math.nan, 2.5) is None


def test_is_occupied_only_for_known_cells_at_or_above_the_threshold():
    grid = OccupancyMap(1.0, 0.0, 0.0, 4, 2, [100, 65, 64, UNKNOWN, FREE, FREE, FREE, FREE])
    assert [grid.is_occupied(col + 0.5, 0.5) for col in range(4)] == [True, True, False, False]
    assert grid.is_occupied(1.5, 1.5) is False              # row 1 is the second data row
    assert grid.is_occupied(-0.5, 0.5) is False             # off the map


def test_near_occupied_measures_between_cell_centres():
    grid = OccupancyMap(0.1, 0.0, 0.0, 10, 10,
                        [OCCUPIED if (col, row) == (5, 5) else FREE
                         for row in range(10) for col in range(10)])
    assert grid.near_occupied(0.55, 0.55, 0.0)             # the occupied cell itself
    assert grid.near_occupied(0.62, 0.55, 0.1)             # neighbour: centres 0.1 apart
    assert not grid.near_occupied(0.62, 0.55, 0.09)
    assert not grid.near_occupied(0.65, 0.65, 0.14)        # diagonal: 0.141 apart
    assert grid.near_occupied(0.65, 0.65, 0.15)
    assert not grid.near_occupied(0.85, 0.55, 0.25)
    assert not grid.near_occupied(-0.05, 0.55, 5.0)        # off the map
    with pytest.raises(ValueError, match='radius'):
        grid.near_occupied(0.55, 0.55, -0.1)


def test_near_occupied_reach_survives_float_noise():
    # west wall cells end at centre x 0.025; 0.15 / 0.05 is 2.9999999999999996 cells
    assert ROOM.near_occupied(0.17, 1.0, 0.15)             # centre 0.175: 3 cells away
    assert not ROOM.near_occupied(0.22, 1.0, 0.15)         # centre 0.225: 4 cells away


def test_unknown_cells_are_not_occupied_for_near_occupied():
    grid = OccupancyMap(0.1, 0.0, 0.0, 3, 1, [UNKNOWN, FREE, FREE])
    assert not grid.near_occupied(0.15, 0.05, 0.5)


def test_occupancy_map_validates_its_shape():
    with pytest.raises(ValueError, match='data'):
        OccupancyMap(0.1, 0.0, 0.0, 3, 2, [FREE] * 5)
    with pytest.raises(ValueError, match='resolution'):
        OccupancyMap(0.0, 0.0, 0.0, 1, 1, [FREE])
    with pytest.raises(ValueError, match='occupied_threshold'):
        OccupancyMap(0.1, 0.0, 0.0, 1, 1, [FREE], occupied_threshold=0)


# ----------------------------------------------------------- line of sight

@pytest.mark.parametrize('start, end, visible', [
    ((1.0, 0.75), (3.0, 0.75), False),     # through the stub
    ((1.0, 2.25), (3.0, 2.25), True),      # above its end
    ((1.0, 1.0), (3.0, 2.6), True),        # diagonally past its end
    ((1.0, 1.0), (3.0, 1.2), False),       # diagonally into it
    ((1.0, 1.0), (1.0, 2.5), True),        # along it
])
def test_line_of_sight_through_and_around_a_wall(start, end, visible):
    assert ROOM.line_of_sight(*start, *end) is visible
    assert ROOM.line_of_sight(*end, *start) is visible


def test_line_of_sight_does_not_test_the_endpoint_cells():
    # the stub is two cells thick: x 1.95..2.0 and 2.0..2.05
    assert ROOM.line_of_sight(2.04, 0.75, 3.0, 0.75)       # starts in its east cells
    assert ROOM.line_of_sight(1.0, 0.75, 1.96, 0.75)       # ends in its west cells
    assert not ROOM.line_of_sight(1.0, 0.75, 2.04, 0.75)   # its west cells lie between
    assert ROOM.line_of_sight(1.0, 0.75, 1.0, 0.75)        # same cell


def test_line_of_sight_is_blocked_by_unknown_cells_and_the_map_border():
    corridor = OccupancyMap(0.1, 0.0, 0.0, 10, 1, [FREE] * 4 + [UNKNOWN] + [FREE] * 5)
    assert not corridor.line_of_sight(0.05, 0.05, 0.95, 0.05)
    assert corridor.line_of_sight(0.05, 0.05, 0.35, 0.05)
    assert not corridor.line_of_sight(0.55, 0.05, 1.2, 0.05)       # ends off the map
    assert not corridor.line_of_sight(-0.2, 0.05, 0.35, 0.05)      # starts off the map


def test_line_of_sight_through_a_cell_corner_tests_both_cells_beside_it():
    # 1 m cells, only (col 1, row 0) occupied; the diagonal passes exactly
    # through its corner, so the answer must not depend on the direction
    corner = OccupancyMap(1.0, 0.0, 0.0, 3, 3, [FREE, OCCUPIED, FREE] + [FREE] * 6)
    assert not corner.line_of_sight(0.5, 0.5, 2.5, 2.5)
    assert not corner.line_of_sight(2.5, 2.5, 0.5, 0.5)


def segment_meets_box(x0, y0, x1, y1, box):
    """Whether the segment touches the closed box (slab clipping)."""
    low_t, high_t = 0.0, 1.0
    for origin, delta, low, high in ((x0, x1 - x0, box[0], box[2]), (y0, y1 - y0, box[1], box[3])):
        if delta == 0:
            if not low <= origin <= high:
                return False
            continue
        first, second = (low - origin) / delta, (high - origin) / delta
        low_t, high_t = max(low_t, min(first, second)), min(high_t, max(first, second))
    return low_t <= high_t


def test_line_of_sight_agrees_with_a_brute_force_test_on_random_segments():
    occupied = [(col, row) for row in range(ROOM.height) for col in range(ROOM.width)
                if ROOM.data[row * ROOM.width + col] == OCCUPIED]
    rng = random.Random(11)
    outcomes = []
    for _ in range(100):                    # inside the room: no unknown cells on the way
        x0, y0, x1, y1 = (rng.uniform(0.1, 3.9), rng.uniform(0.1, 2.9),
                          rng.uniform(0.1, 3.9), rng.uniform(0.1, 2.9))
        ends = {ROOM.world_to_cell(x0, y0), ROOM.world_to_cell(x1, y1)}
        blocked = any(segment_meets_box(x0, y0, x1, y1, (
            ROOM.origin_x + col * RESOLUTION, ROOM.origin_y + row * RESOLUTION,
            ROOM.origin_x + (col + 1) * RESOLUTION, ROOM.origin_y + (row + 1) * RESOLUTION))
            for col, row in occupied if (col, row) not in ends)
        assert ROOM.line_of_sight(x0, y0, x1, y1) is not blocked, (x0, y0, x1, y1)
        outcomes.append(blocked)
    assert 10 < sum(outcomes) < 90                          # both cases exercised


# ------------------------------------------------------------ scan to points

def test_scan_to_points_keeps_finite_returns_within_limits_in_the_map_frame():
    ranges = [1.0, math.inf, math.nan, 0.05, 8.0, 2.0, -1.0]
    points = scan_to_points(ranges, angle_min=0.0, angle_increment=math.pi / 2,
                            range_min=0.12, range_max=8.0, sensor_pose=(1.0, 2.0, math.pi / 2))
    # index 4 sits at range_max: a no-return reading, not a hit
    assert [value for point in points for value in point] == pytest.approx([1.0, 3.0, -1.0, 2.0])


def test_unexplained_points_are_hits_in_known_free_space_away_from_static_cells():
    points = [(0.0, 1.0),       # in the west wall
              (0.12, 1.0),      # 0.07 m off its face: within tolerance
              (0.36, 1.0),      # 0.31 m off: new
              (1.0, 1.0),       # open floor: new
              (4.3, 1.0),       # unknown, outside the building
              (9.0, 9.0)]       # off the map
    assert unexplained_points(points, ROOM, tolerance=0.15) == [(0.36, 1.0), (1.0, 1.0)]


# ---------------------------------------------------------------- clusters

def test_cluster_points_links_chains_and_drops_small_groups():
    chain = [(1.0 + 0.15 * step, 0.0) for step in range(5)]        # 0.15 m apart: linked
    pair = [(3.0, 3.0), (3.1, 3.0)]                                # too few points
    blob = [(-2.0, 1.0), (-2.1, 1.0), (-2.0, 1.1), (-2.1, 1.1)]
    first, second = cluster_points(chain + pair + blob, eps=0.2, min_points=3)
    assert (first.x, first.y, first.radius, first.count) == pytest.approx((1.3, 0.0, 0.3, 5))
    assert (second.x, second.y, second.count) == pytest.approx((-2.05, 1.05, 4))
    assert second.radius == pytest.approx(math.hypot(0.05, 0.05))


def test_cluster_points_does_not_depend_on_point_order():
    points = [(0.1 * step, 0.0) for step in range(6)] + [(5.0, 0.1 * step) for step in range(4)]
    shuffled = list(points)
    random.Random(5).shuffle(shuffled)
    summary = sorted((round(c.x, 6), round(c.y, 6), c.count) for c in cluster_points(points))
    assert summary == sorted((round(c.x, 6), round(c.y, 6), c.count)
                             for c in cluster_points(shuffled))
    assert len(summary) == 2


def test_cluster_points_does_not_link_beyond_eps():
    assert cluster_points([(0.25 * step, 0.0) for step in range(6)], eps=0.2, min_points=2) == []


def test_cluster_parameters_are_validated():
    with pytest.raises(ValueError, match='eps'):
        cluster_points([(0.0, 0.0)], eps=0.0)
    with pytest.raises(ValueError, match='min_points'):
        cluster_points([(0.0, 0.0)], min_points=0)


# ------------------------------------------------------ obstacle detections

@pytest.mark.parametrize('pose', [(1.0, 2.2, 0.3), (3.0, 0.8, 2.0), (2.0, 2.0, -1.0),
                                  (3.5, 2.5, 4.0)])
def test_static_walls_explain_every_hit(pose):
    angle_min, increment, ranges = raycast(pose, WALLS)
    points = scan_to_points(ranges, angle_min, increment, *SCAN_LIMITS, pose)
    assert len(points) == 360                                       # closed room
    assert unexplained_points(points, ROOM) == []
    assert detect(pose, WALLS, ROOM) == []


CRATE = (2.8, 2.0, 0.4)                     # x, y, side; not in the map


@pytest.mark.parametrize('pose', [(1.0, 2.0, 0.0),                  # west face, face-on
                                  (3.6, 0.6, 2.0),                  # south and east faces
                                  (1.5, 2.6, -0.5)])                # west and north faces
def test_unmapped_box_becomes_one_obstacle_at_its_centre(pose):
    (detection,) = detect(pose, WALLS + [square(*CRATE)], ROOM, stamp=12.5)
    assert math.hypot(detection.x - CRATE[0], detection.y - CRATE[1]) < 0.1
    assert (detection.category, detection.label, detection.source, detection.stamp) == \
        (Category.OBSTACLE, 'unknown', 'lidar', 12.5)
    assert detection.size[0] == detection.size[1]
    assert CRATE[2] - 0.1 <= detection.size[0] <= CRATE[2] * math.sqrt(2) + 0.05
    assert 0.5 < detection.confidence < 1.0


def test_confidence_rises_with_the_point_count_and_size_has_a_floor():
    open_floor = OccupancyMap(0.05, -5.0, -5.0, 200, 200, [FREE] * 40000)
    ranges = [1.0] * 3 + [math.inf] * 100 + [1.0] * 9      # 0.01 rad apart: 1 cm spacing
    small, large = obstacle_detections(ranges, 0.0, 0.01, *SCAN_LIMITS, (0.0, 0.0, 0.0),
                                       open_floor, stamp=0.0, min_points=3)
    assert small.confidence == pytest.approx(3 / (3 + 3))   # n / (n + min_points)
    assert large.confidence == pytest.approx(9 / (9 + 3))
    assert small.size == large.size == (0.1, 0.1, 0.0)


def test_small_or_scattered_returns_make_no_obstacle():
    open_floor = OccupancyMap(0.05, -5.0, -5.0, 200, 200, [FREE] * 40000)
    ranges = [1.0, 1.0] + [math.inf] * 60 + [1.0] + [math.inf] * 60 + [1.0]
    assert obstacle_detections(ranges, 0.0, 0.01, *SCAN_LIMITS, (0.0, 0.0, 0.0), open_floor,
                               stamp=0.0) == []


# --------------------------------------------------------------- lab world

def load_lab_world():
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'tools',
                        'gen_world.py')
    if not os.path.isfile(path):
        pytest.skip('tools/gen_world.py not available')
    spec = importlib.util.spec_from_file_location('gen_world', path)
    world = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(world)
    return world


def lab_map(world):
    """The rendered Nav2 map as nav_msgs/OccupancyGrid data (row 0 = min y)."""
    (_, pgm), _ = world.render_map()
    _, _, dimensions, _, pixels = pgm.split(b'\n', 4)
    width, height = (int(value) for value in dimensions.split())
    occupancy = {world.PIXEL_FREE: FREE, world.PIXEL_OCCUPIED: OCCUPIED,
                 world.PIXEL_UNKNOWN: UNKNOWN}
    data = [occupancy[pixel] for row in reversed(range(height))
            for pixel in pixels[row * width:(row + 1) * width]]
    return OccupancyMap(world.MAP_RESOLUTION, world.MAP_X_RANGE[0], world.MAP_Y_RANGE[0],
                        width, height, data)


def lab_boxes(world):
    """(mapped boxes, unmapped boxes) of walls and obstacles."""
    walls = [(x - size_x / 2, y - size_y / 2, x + size_x / 2, y + size_y / 2)
             for x, y, size_x, size_y in world.wall_boxes()]
    mapped, unmapped = list(walls), []
    for _, x, y, size_x, size_y, _, in_map in world.OBSTACLES:
        box = (x - size_x / 2, y - size_y / 2, x + size_x / 2, y + size_y / 2)
        (mapped if in_map else unmapped).append(box)
    return mapped, unmapped


def test_lab_world_walls_explain_every_hit_and_the_crate_is_found_from_the_dock():
    world = load_lab_world()
    occupancy = lab_map(world)
    mapped, unmapped = lab_boxes(world)
    poses = [(x, y, yaw) for _, _, x, y, yaw in world.LOCATIONS]
    for pose in poses:
        angle_min, increment, ranges = raycast(pose, mapped)
        points = scan_to_points(ranges, angle_min, increment, *SCAN_LIMITS, pose)
        assert points and unexplained_points(points, occupancy) == [], pose
    (crate,) = [obstacle for obstacle in world.OBSTACLES if obstacle[0] == 'crate']
    dock = next((x, y, yaw) for name, _, x, y, yaw in world.LOCATIONS if name == 'dock')
    (detection,) = detect(dock, mapped + unmapped, occupancy)
    assert math.hypot(detection.x - crate[1], detection.y - crate[2]) < 0.1


def test_lab_world_scan_pipeline_is_fast():
    world = load_lab_world()
    occupancy = lab_map(world)
    mapped, unmapped = lab_boxes(world)
    pose = (0.0, 0.0, 0.0)
    angle_min, increment, ranges = raycast(pose, mapped + unmapped)
    start = time.perf_counter()
    obstacle_detections(ranges, angle_min, increment, *SCAN_LIMITS, pose, occupancy, stamp=0.0)
    first = time.perf_counter() - start                     # includes the map dilation
    samples = []
    for _ in range(20):
        start = time.perf_counter()
        obstacle_detections(ranges, angle_min, increment, *SCAN_LIMITS, pose, occupancy,
                            stamp=0.0)
        samples.append(time.perf_counter() - start)
    median = statistics.median(samples)
    print(f'\nobstacle_detections(360 beams, {occupancy.width}x{occupancy.height} map): '
          f'first {first * 1e3:.1f} ms, then median {median * 1e3:.2f} ms')
    assert first < 0.5 and median < 0.020
