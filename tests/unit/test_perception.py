"""Perception glue (fleetgraph_robot.perception, the generated regions and the
SceneObservation conversion of perception_node). Pure except the message
tests, which skip when the workspace is not built.
"""
import importlib.util
import math
import os
import statistics
import time
from types import SimpleNamespace

import pytest
import yaml

from fleetgraph_robot.lidar_obstacles import OccupancyMap, obstacle_detections
from fleetgraph_robot.perception import (
    SEMANTIC_SOURCE, MapMatcher, coverage, localization_score, occupancy_from_grid,
    semantic_detections, to_map)
from fleetgraph_robot.regions import load_regions
from fleetgraph_robot.scene_graph import Category, Delta, Entity, Predicate, Relation

import test_lidar_obstacles as lidar_tests                 # lab world, raycast

CATEGORY = {category.name: category.value for category in Category}


def load_gen_world():
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'tools',
                        'gen_world.py')
    if not os.path.isfile(path):
        pytest.skip('tools/gen_world.py not available')
    spec = importlib.util.spec_from_file_location('gen_world', path)
    world = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(world)
    return world


# ------------------------------------------------------------ regions file

def test_generated_regions_match_the_floor_plan():
    world = load_gen_world()
    ((path, data),) = world.render_regions()
    assert path.endswith(os.path.join('fleetgraph_sim', 'config', 'regions.yaml'))
    regions = {region.id: region for region in load_regions(yaml.safe_load(data))}
    assert {region_id: region.kind for region_id, region in regions.items()} == {
        'hall': 'place', 'lab_1': 'place', 'lab_2': 'place',
        'door_lab1': 'traversable', 'door_lab2': 'traversable'}
    assert all(region.label for region in regions.values())
    places = {location_id: (x, y) for location_id, _, x, y, _ in world.LOCATIONS}
    for location_id, place in (('dock', 'hall'), ('hall', 'hall'), ('lab_1', 'lab_1'),
                               ('lab_2', 'lab_2')):
        assert regions[place].contains(*places[location_id]), location_id
    (crate,) = [obstacle for obstacle in world.OBSTACLES if obstacle[0] == 'crate']
    assert regions['hall'].contains(crate[1], crate[2])


def test_generated_doors_span_the_wall_gaps():
    world = load_gen_world()
    ((_, data),) = world.render_regions()
    walls = [(x - size_x / 2, y - size_y / 2, x + size_x / 2, y + size_y / 2)
             for x, y, size_x, size_y in world.wall_boxes()]

    def in_wall(x, y):
        return any(xmin <= x <= xmax and ymin <= y <= ymax for xmin, ymin, xmax, ymax in walls)

    doors = [region for region in load_regions(yaml.safe_load(data))
             if region.kind == 'traversable']
    for door in doors:
        center_x, center_y = door.center()
        assert door.travel_axis == 'x'
        # free along the travel axis through its middle, wall at both cross ends
        assert not any(in_wall(door.xmin + step * (door.xmax - door.xmin) / 20, center_y)
                       for step in range(21)), door.id
        assert in_wall(center_x, door.ymin) and in_wall(center_x, door.ymax), door.id


# ------------------------------------------------------------- geometry

def test_to_map_rotates_then_translates():
    assert to_map((0.0, 0.0, 0.0), 1.5, -0.5) == (1.5, -0.5)
    assert to_map((2.0, 3.0, 0.0), 1.0, 1.0) == (3.0, 4.0)
    x, y = to_map((2.0, 3.0, math.pi / 2), 1.0, 0.5)       # x forward = +y in the map
    assert (x, y) == pytest.approx((1.5, 4.0))


# Room x 0..4, y 0..2 at 0.1 m with a wall column at x = 2.0..2.1 from y = 0
# to 1.5; walls are occupied cells (100), the rest free (0).
def room():
    resolution, width, height = 0.1, 40, 20
    data = [100 if col == 20 and row < 15 else 0
            for row in range(height) for col in range(width)]
    return OccupancyMap(resolution, 0.0, 0.0, width, height, data)


def test_coverage_needs_range_and_static_line_of_sight():
    in_view = coverage(room(), 0.5, 0.5, max_range=3.0)
    assert in_view(1.5, 1.0)
    assert not in_view(3.0, 0.5)                    # behind the wall column
    assert in_view(2.5, 1.95)                       # over the top of the column
    assert not in_view(0.5, 3.51) and not in_view(3.9, 1.95)      # out of range
    assert not in_view(-0.5, 0.5)                   # off the map: no line of sight
    assert coverage(room(), 0.5, 0.5, max_range=1.0)(1.2, 1.2)
    assert not coverage(room(), 0.5, 0.5, max_range=1.0)(1.3, 1.3)


def grid(origin_yaw=0.0):
    orientation = SimpleNamespace(x=0.0, y=0.0, z=math.sin(origin_yaw / 2),
                                  w=math.cos(origin_yaw / 2))
    info = SimpleNamespace(resolution=0.5, width=3, height=2,
                           origin=SimpleNamespace(position=SimpleNamespace(x=-1.0, y=-0.5),
                                                  orientation=orientation))
    return SimpleNamespace(info=info, data=[0, 100, -1, 0, 0, 0])


def test_occupancy_from_grid_reads_an_occupancy_grid():
    occupancy = occupancy_from_grid(grid())
    assert (occupancy.resolution, occupancy.origin_x, occupancy.origin_y) == (0.5, -1.0, -0.5)
    assert (occupancy.width, occupancy.height) == (3, 2)
    assert occupancy.is_occupied(-0.25, -0.25) and not occupancy.is_occupied(-0.75, -0.25)
    assert occupancy.world_to_cell(0.4, 0.4) == (2, 1)


def test_occupancy_from_grid_rejects_a_rotated_origin():
    with pytest.raises(ValueError, match='rotated'):
        occupancy_from_grid(grid(origin_yaw=0.3))


# ------------------------------------------------------------ map matcher

# True lidar poses on the dock -> lab_1 route and in the hall, each tried with
# AMCL-like errors (dx m, dy m, dyaw deg): AMCL was measured up to 0.14 m and
# 3 deg off ground truth while driving (2026-11-08).
ROUTE = [(0.0, 0.0, 0.0), (1.5, 1.0, 0.6), (3.0, 1.5, 0.0), (3.6, 1.5, -0.05),
         (4.5, 1.45, -0.05), (5.8, 1.3, -0.2), (2.5, -0.3, 1.2)]
POSE_ERRORS = [(0.14, 0.05, 3.0), (-0.1, 0.12, -3.0), (0.2, -0.15, 4.0), (0.08, -0.08, -2.0)]


@pytest.fixture(scope='module')
def lab():
    """(occupancy, all boxes, matcher, scan limits, raycast) of the lab world."""
    world = lidar_tests.load_lab_world()
    occupancy = lidar_tests.lab_map(world)
    mapped, unmapped = lidar_tests.lab_boxes(world)
    return (occupancy, mapped + unmapped, MapMatcher(occupancy), lidar_tests.SCAN_LIMITS,
            lidar_tests.raycast)


def shifted(pose, dx, dy, dyaw_deg):
    return (pose[0] + dx, pose[1] + dy, pose[2] + math.radians(dyaw_deg))


def pose_error(pose, truth):
    return (math.hypot(pose[0] - truth[0], pose[1] - truth[1]),
            abs(math.degrees(pose[2] - truth[2])))


def test_map_matcher_corrects_amcl_like_pose_errors(lab):
    _, boxes, matcher, limits, raycast = lab
    for truth in ROUTE:
        angle_min, increment, ranges = raycast(truth, boxes)
        for error in POSE_ERRORS:
            refined = matcher.refine(ranges, angle_min, increment, *limits,
                                     shifted(truth, *error))
            distance, turn = pose_error(refined, truth)
            assert distance <= 0.06 and turn <= 1.25, (truth, error, distance, turn)


def test_map_matcher_keeps_a_correct_pose(lab):
    _, boxes, matcher, limits, raycast = lab
    for truth in ROUTE:
        angle_min, increment, ranges = raycast(truth, boxes)
        distance, turn = pose_error(
            matcher.refine(ranges, angle_min, increment, *limits, truth), truth)
        assert distance <= 0.0125 and turn <= 0.25, (truth, distance, turn)


def test_map_matcher_without_returns_keeps_the_pose(lab):
    occupancy, _, matcher, limits, _ = lab
    pose = (1.0, 1.0, 0.5)
    assert matcher.refine([math.inf] * 360, -math.pi, 2 * math.pi / 360, *limits, pose) == pose


def test_corrected_pose_leaves_only_the_crate(lab):
    occupancy, boxes, matcher, limits, raycast = lab
    truth = (3.0, 1.5, 0.0)                 # before the lab_1 door, crate behind
    angle_min, increment, ranges = raycast(truth, boxes)

    def obstacles(pose):
        return obstacle_detections(ranges, angle_min, increment, limits[0], 4.0, pose,
                                   occupancy, stamp=0.0, tolerance=0.2)

    guess = shifted(truth, -0.1, 0.12, -3.0)
    assert len(obstacles(guess)) > 1        # walls in free cells
    (crate,) = obstacles(matcher.refine(ranges, angle_min, increment, *limits, guess))
    assert math.hypot(crate.x - 2.5, crate.y - 0.8) < 0.15


def test_map_matcher_is_fast_enough_for_10_hz_scans(lab):
    _, boxes, matcher, limits, raycast = lab
    angle_min, increment, ranges = raycast(ROUTE[2], boxes)
    guess = shifted(ROUTE[2], *POSE_ERRORS[0])
    samples = []
    for _ in range(10):
        start = time.perf_counter()
        matcher.refine(ranges, angle_min, increment, *limits, guess)
        samples.append(time.perf_counter() - start)
    median = statistics.median(samples)
    print(f'\nMapMatcher.refine(360 beams): median {median * 1e3:.1f} ms')
    assert median < 0.1


# ------------------------------------------------------ semantic detections

def detected(category, x, y, size=(0.6, 0.6, 1.7), confidence=0.85, z=0.85, label='person'):
    return SimpleNamespace(id='', category=CATEGORY[category], label=label,
                           position=SimpleNamespace(x=x, y=y, z=z),
                           size=SimpleNamespace(x=size[0], y=size[1], z=size[2]),
                           confidence=confidence)


def test_semantic_detections_move_into_the_map_frame():
    pose = (1.0, 1.0, math.pi / 2)                  # robot at (1, 1) facing +y
    (person,) = semantic_detections([detected('PERSON', 2.0, 0.5)], pose, stamp=12.5)
    assert (person.x, person.y) == pytest.approx((0.5, 3.0))
    assert (person.category, person.label, person.z) == (Category.PERSON, 'person', 0.85)
    assert (person.size, person.confidence) == ((0.6, 0.6, 1.7), 0.85)
    assert (person.stamp, person.source) == (12.5, SEMANTIC_SOURCE)


def test_semantic_detections_drop_what_the_graph_cannot_track():
    entities = [detected('GROUP', 1.0, 0.0), detected('PLACE', 1.0, 0.0),
                detected('PERSON', math.nan, 0.0), detected('OBJECT', 1.0, 0.0, size=(-1, 1, 1)),
                detected('ROBOT', 1.0, 0.0, confidence=math.nan),
                detected('OBJECT', 2.0, 0.0, confidence=1.2, label='box'),
                detected('OBSTACLE', 3.0, 0.0, confidence=-0.1, label='')]
    kept = semantic_detections(entities, (0.0, 0.0, 0.0), stamp=1.0)
    assert [(item.category, item.x, item.confidence) for item in kept] == [
        (Category.OBJECT, 2.0, 1.0), (Category.OBSTACLE, 3.0, 0.0)]


def test_localization_score_matches_the_agent_heuristic():
    covariance = [0.0] * 36
    assert localization_score(covariance) == 1.0
    covariance[0] = covariance[7] = 0.25 ** 2 / 2          # 0.25 m std over x and y
    assert localization_score(covariance) == pytest.approx(0.5)
    covariance[0] = -1.0                                    # garbage variance counts as 0
    assert localization_score(covariance) == pytest.approx(1.0 / (1.0 + math.sqrt(0.03125) / 0.25))


# ----------------------------------------------------- message conversion

def test_time_msg_rounds_sim_seconds_to_nanoseconds():
    pytest.importorskip('fleetgraph_interfaces.msg')
    from fleetgraph_robot.perception_node import time_msg
    assert (time_msg(12.3456789).sec, time_msg(12.3456789).nanosec) == (12, 345678900)
    assert (time_msg(1.9999999999).sec, time_msg(1.9999999999).nanosec) == (2, 0)


def test_observation_msg_carries_the_delta():
    pytest.importorskip('fleetgraph_interfaces.msg')
    from geometry_msgs.msg import TransformStamped
    from fleetgraph_robot.perception_node import observation_msg, pose_stamped

    crate = Entity('robot_1/obstacle/4', Category.OBSTACLE, 'unknown', 2.5, 0.8, z=0.1,
                   size=(0.5, 0.5, 0.0), vx=0.01, vy=-0.02, confidence=0.875,
                   first_seen=20.25, last_seen=31.5, attributes={'source': 'lidar'})
    group = Entity('robot_1/group/9', Category.GROUP, 'group', 1.0, 1.0,
                   member_ids=['robot_1/person/7', 'robot_1/person/8'])
    delta = Delta([crate, group], ['robot_1/person/3'],
                  [Relation(crate.id, Predicate.INSIDE, 'hall', 0.875)])
    transform = TransformStamped()
    transform.header.frame_id = 'map'
    transform.header.stamp.sec, transform.header.stamp.nanosec = 31, 500000000
    transform.transform.translation.x, transform.transform.translation.y = 1.0, 0.5
    transform.transform.rotation.z, transform.transform.rotation.w = 0.6, 0.8
    sensor_pose = pose_stamped(transform)
    assert (sensor_pose.header.frame_id, sensor_pose.header.stamp.sec) == ('map', 31)
    assert (sensor_pose.pose.position.x, sensor_pose.pose.orientation.z) == (1.0, 0.6)

    msg = observation_msg(delta, 'robot_1', 'map', sensor_pose, confidence=0.9)
    assert (msg.robot_id, msg.frame_id, msg.sensor_pose) == ('robot_1', 'map', sensor_pose)
    assert msg.confidence == pytest.approx(0.9)
    assert list(msg.removed_entity_ids) == ['robot_1/person/3']
    first, second = msg.entities
    assert (first.id, first.category, first.label) == (crate.id, Category.OBSTACLE, 'unknown')
    assert (first.position.x, first.position.y, first.position.z) == (2.5, 0.8, 0.1)
    assert (first.size.x, first.size.y, first.size.z) == (0.5, 0.5, 0.0)
    assert (first.velocity.x, first.velocity.y, first.velocity.z) == (0.01, -0.02, 0.0)
    assert first.confidence == pytest.approx(0.875)
    assert (first.first_seen.sec, first.first_seen.nanosec) == (20, 250000000)
    assert (first.last_seen.sec, first.last_seen.nanosec) == (31, 500000000)
    assert [(pair.key, pair.value) for pair in first.attributes] == [('source', 'lidar')]
    assert list(first.source_ids) == [] and list(first.member_ids) == []
    assert list(second.member_ids) == group.member_ids
    (relation,) = msg.relations
    assert (relation.subject_id, relation.predicate, relation.object_id) == \
        (crate.id, Predicate.INSIDE, 'hall')
    assert relation.confidence == pytest.approx(0.875)
