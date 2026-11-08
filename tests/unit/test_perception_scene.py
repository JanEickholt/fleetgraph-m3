"""The scene of weekly_perception_scenes, group_hall_person_lab1, against the
robots' docks, goals and routes of fleet_three_robots: who sees whom with
the simulated detector, the group as the local graph forms it, and
footprints clear of the routes. Pure: no rclpy, no sim.
"""
import math
import os
import random

import pytest

from fleetgraph_robot.scene_graph import Category, Detection, LocalSceneGraph
from fleetgraph_sim_tools import map_loader, scene_script
from fleetgraph_sim_tools.detector_model import DetectorParams, simulate

PACKAGE = os.path.join(os.path.dirname(os.path.abspath(scene_script.__file__)), '..')
SCENE = scene_script.load(os.path.join(PACKAGE, 'scenes', 'group_hall_person_lab1.yaml'))
OCCUPANCY = map_loader.load(os.path.join(PACKAGE, '..', 'fleetgraph_sim', 'maps',
                                         'lab_floor.yaml'))
EXACT = DetectorParams(position_sigma=0.0, dropout=0.0, confidence_sigma=0.0)
# fleetgraph_sim config/locations.yaml: robot_N starts at its dock facing +x
DOCK, DOCK_2, DOCK_3 = (0.0, 0.0, 0.0), (2.8, -2.3, 0.0), (0.0, -1.5, 0.0)
LAB_1, LAB_2, HALL = (6.0, 1.2, 0.0), (6.5, -1.5, 0.0), (2.0, -1.5, 0.0)
# fleet_three_robots: robot_1 dock -> lab_1 round either side of the crate
# (Nav2 picks one), robot_2 dock_2 -> lab_2, robot_3 dock_3 -> hall
ROUTES = {'robot_1 north of the crate': [(0.0, 0.0), (2.5, 1.45), (4.0, 1.5), LAB_1[:2]],
          'robot_1 south of the crate': [(0.0, 0.0), (2.5, 0.3), (4.0, 1.5), LAB_1[:2]],
          'robot_2': [DOCK_2[:2], (4.0, -1.5), LAB_2[:2]],
          'robot_3': [DOCK_3[:2], HALL[:2]]}


def seen_from(pose, t):
    rng = random.Random(0)
    return sorted(detection.truth_id
                  for detection in simulate(SCENE.truth(t), pose, OCCUPANCY, EXACT, rng))


def persons():
    return [entity for entity in SCENE.entities if entity.category == Category.PERSON]


def test_robot_1_sees_the_group_from_the_dock_and_the_visitor_from_lab_1_until_it_leaves():
    assert seen_from(DOCK, 10.0) == ['crate', 'person_1', 'person_2']
    assert seen_from(LAB_1, 79.0) == ['visitor']
    assert seen_from(LAB_1, 80.0) == []
    assert seen_from((4.0, 1.5, 0.0), 30.0) == ['visitor']         # from door_lab1


def test_robot_3_sees_the_group_from_dock_3_and_from_hall():
    assert seen_from(DOCK_3, 10.0) == ['crate', 'person_1', 'person_2']
    assert seen_from(HALL, 10.0) == ['person_1', 'person_2']


def test_the_local_graph_makes_one_group_of_the_two_hall_persons():
    graph = LocalSceneGraph('robot_1', [])
    standing = [Detection(Category.PERSON, x, y, z=0.85, size=(0.5, 0.5, 1.7), confidence=0.85,
                          label='person', source='detector')
                for x, y, _ in (entity.pose(10.0) for entity in persons()
                                if entity.id != 'visitor')]
    for frame in range(3):
        graph.integrate(standing, frame * 0.2)
    (group,) = [entity for entity in graph.entities() if entity.category == Category.GROUP]
    assert (group.x, group.y) == pytest.approx((3.3, -0.5))
    assert len(group.member_ids) == 2


def distance_to_route(x, y, points):
    def to_segment(a, b):
        dx, dy = b[0] - a[0], b[1] - a[1]
        t = max(0.0, min(1.0, ((x - a[0]) * dx + (y - a[1]) * dy) / (dx * dx + dy * dy)))
        return math.hypot(x - a[0] - t * dx, y - a[1] - t * dy)
    return min(to_segment(a, b) for a, b in zip(points, points[1:]))


def test_persons_stand_clear_of_every_route_and_the_lab_1_goal():
    for person in persons():
        x, y, _ = person.pose(0.0)
        radius = person.shape.size[0] / 2
        for route, points in ROUTES.items():
            # 0.5 m: beyond Nav2's inflation (0.45 m) round the footprint
            assert distance_to_route(x, y, points) - radius >= 0.5, (person.id, route)
    visitor = next(person for person in persons() if person.id == 'visitor')
    assert math.dist(visitor.pose(0.0)[:2], LAB_1[:2]) - 0.25 >= 1.0
    assert (visitor.spawn, visitor.remove) == (0.0, 80.0)
