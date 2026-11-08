"""Scene scripts (fleetgraph_sim_tools.scene_script). Pure: no rclpy, no sim.
Covers validation, the timeline (active window, pose, velocity), the truth
records, the Gazebo model, the director's playback diff, the shipped scenes
against the lab world of tools/gen_world.py, and the reproducibility of the
detector pipeline fed with scene truth.
"""
import glob
import importlib.util
import math
import os
import random
import re
import xml.etree.ElementTree as ElementTree

import pytest

from fleetgraph_robot.scene_graph import Category
from fleetgraph_sim_tools import map_loader, scene_script
from fleetgraph_sim_tools.detector_model import DetectorParams, simulate
from fleetgraph_sim_tools.scene_script import Playback, parse

PACKAGE = os.path.join(os.path.dirname(os.path.abspath(scene_script.__file__)), '..')
SCENES = os.path.join(PACKAGE, 'scenes')
LAB_MAP = os.path.join(PACKAGE, '..', 'fleetgraph_sim', 'maps', 'lab_floor.yaml')
DOCK = (0.0, 0.0, 0.0)
PERSON = {'id': 'p1', 'category': 'person', 'label': 'person',
          'shape': {'type': 'cylinder', 'radius': 0.25, 'height': 1.7},
          'path': [{'t': 0, 'x': 1.0, 'y': 2.0}]}
BOX = {'id': 'b1', 'category': 'obstacle', 'label': 'box',
       'shape': {'type': 'box', 'size': [0.6, 0.4, 0.5]},
       'path': [{'t': 0, 'x': 3.0, 'y': -1.0}]}
CRATE = {'id': 'crate', 'category': 'obstacle', 'label': 'crate', 'preexisting': True,
         'shape': {'type': 'box', 'size': [0.4, 0.4, 0.5]},
         'path': [{'t': 0, 'x': 2.5, 'y': 0.8}]}


def entity(base=PERSON, **changes):
    data = dict(base)
    data.update(changes)
    return {key: value for key, value in data.items() if value is not None}


def scene(*entities):
    return parse({'name': 'test', 'entities': list(entities)})


def only(*entities):
    (parsed,) = scene(*entities).entities
    return parsed


def shipped(name):
    return scene_script.load(os.path.join(SCENES, f'{name}.yaml'))


def by_id(parsed_scene, entity_id):
    return next(entity for entity in parsed_scene.entities if entity.id == entity_id)


def test_parses_a_static_person_with_defaults():
    person = only(PERSON)
    assert (person.id, person.model_name, person.category, person.label) == \
        ('p1', 'scene_p1', Category.PERSON, 'person')
    assert (person.shape.type, person.shape.size) == ('cylinder', (0.5, 0.5, 1.7))
    assert (person.spawn, person.remove, person.preexisting) == (0.0, None, False)
    assert person.pose(123.0) == (1.0, 2.0, 0.0)
    assert person.velocity(123.0) == (0.0, 0.0)


def test_category_names_and_box_shape():
    box, thing = scene(BOX, entity(BOX, id='b2', category='object')).entities
    assert (box.category, thing.category) == (Category.OBSTACLE, Category.OBJECT)
    assert (box.shape.type, box.shape.size) == ('box', (0.6, 0.4, 0.5))


@pytest.mark.parametrize('data, words', [
    ({'entities': []}, ['name']),
    ({'name': 'x', 'entities': {}}, ['entities']),
    ({'name': 'x', 'entities': [], 'extra': 1}, ['extra']),
    ({'name': 'x', 'entities': [entity(id=None)]}, ['entity #0', 'id']),
    ({'name': 'x', 'entities': [entity(id='a b')]}, ["'a b'", 'id']),
    ({'name': 'x', 'entities': [PERSON, PERSON]}, ["'p1'", 'id', 'duplicate']),
    ({'name': 'x', 'entities': [entity(category='robot')]}, ["'p1'", 'category']),
    ({'name': 'x', 'entities': [entity(label='')]}, ["'p1'", 'label']),
    ({'name': 'x', 'entities': [entity(shape={'type': 'sphere', 'radius': 1})]},
     ["'p1'", 'shape.type']),
    ({'name': 'x', 'entities': [entity(shape={'type': 'cylinder', 'height': 1.7})]},
     ["'p1'", 'shape.radius']),
    ({'name': 'x', 'entities': [entity(shape={'type': 'cylinder', 'radius': -0.2,
                                               'height': 1.7})]}, ["'p1'", 'shape.radius']),
    ({'name': 'x', 'entities': [entity(BOX, shape={'type': 'box', 'size': [1, 1]})]},
     ["'b1'", 'shape.size']),
    ({'name': 'x', 'entities': [entity(shape={'type': 'cylinder', 'radius': 0.2,
                                               'height': 1.7, 'size': [1, 1, 1]})]},
     ["'p1'", 'shape.size']),
    ({'name': 'x', 'entities': [entity(path=[])]}, ["'p1'", 'path']),
    ({'name': 'x', 'entities': [entity(path=[{'x': 1, 'y': 2}])]}, ["'p1'", 'path[0].t']),
    ({'name': 'x', 'entities': [entity(path=[{'t': 0, 'x': 1, 'y': 2},
                                             {'t': 0, 'x': 2, 'y': 2}])]},
     ["'p1'", 'path[1].t', 'increase']),
    ({'name': 'x', 'entities': [entity(path=[{'t': 0, 'x': 'left', 'y': 2}])]},
     ["'p1'", 'path[0].x']),
    ({'name': 'x', 'entities': [entity(path=[{'t': 0, 'x': True, 'y': 2}])]},
     ["'p1'", 'path[0].x']),
    ({'name': 'x', 'entities': [entity(path=[{'t': 0, 'x': 1, 'y': 2, 'z': 0}])]},
     ["'p1'", 'path[0].z']),
    ({'name': 'x', 'entities': [entity(spawn=-1)]}, ["'p1'", 'spawn']),
    ({'name': 'x', 'entities': [entity(spawn=5, remove=5)]}, ["'p1'", 'remove']),
    ({'name': 'x', 'entities': [entity(colour='red')]}, ["'p1'", 'colour']),
    ({'name': 'x', 'entities': [entity(CRATE, remove=10)]}, ["'crate'", 'remove']),
    ({'name': 'x', 'entities': [entity(CRATE, spawn=3)]}, ["'crate'", 'spawn']),
    ({'name': 'x', 'entities': [entity(CRATE, path=[{'t': 0, 'x': 2.5, 'y': 0.8},
                                                    {'t': 5, 'x': 3.0, 'y': 0.8}])]},
     ["'crate'", 'path']),
    ({'name': 'x', 'entities': [entity(preexisting='yes')]}, ["'p1'", 'preexisting']),
])
def test_validation_errors_name_the_entity_and_field(data, words):
    with pytest.raises(ValueError) as error:
        parse(data)
    for word in words:
        assert word in str(error.value)


def test_active_from_spawn_until_remove():
    timed = only(entity(spawn=2, remove=5))
    assert [timed.active(t) for t in (1.99, 2.0, 4.99, 5.0)] == [False, True, True, False]
    assert only(PERSON).active(0.0) and only(PERSON).active(1e6)


def test_path_is_piecewise_linear_and_held_before_the_first_and_after_the_last_waypoint():
    walker = only(entity(path=[{'t': 1, 'x': 0, 'y': 0}, {'t': 3, 'x': 2, 'y': 0},
                               {'t': 5, 'x': 2, 'y': 1}]))
    expected = {0.0: ((0, 0), (0, 0)), 2.0: ((1, 0), (1, 0)), 3.0: ((2, 0), (0, 0.5)),
                4.0: ((2, 0.5), (0, 0.5)), 9.0: ((2, 1), (0, 0))}
    for t, (position, velocity) in expected.items():
        assert walker.pose(t)[:2] == pytest.approx(position), t
        assert walker.velocity(t) == pytest.approx(velocity), t


def test_yaw_interpolates_along_the_shorter_arc():
    turner = only(entity(path=[{'t': 0, 'x': 0, 'y': 0, 'yaw': 3.0},
                               {'t': 1, 'x': 0, 'y': 0, 'yaw': -3.0}]))
    assert abs(math.remainder(turner.pose(0.5)[2] - math.pi, 2 * math.pi)) < 1e-9
    assert turner.pose(1.0)[2] == pytest.approx(-3.0)


def test_truth_record_has_centre_height_axis_aligned_size_and_path_velocity():
    moving = only(entity(BOX, path=[{'t': 0, 'x': 0, 'y': 0, 'yaw': math.pi / 2},
                                    {'t': 2, 'x': 1, 'y': 0, 'yaw': math.pi / 2}]))
    truth = moving.truth(1.0)
    assert (truth.id, truth.category, truth.label) == ('b1', Category.OBSTACLE, 'box')
    assert (truth.x, truth.y, truth.z) == pytest.approx((0.5, 0.0, 0.25))
    assert truth.size == pytest.approx((0.4, 0.6, 0.5))              # rotated by 90 deg
    assert (truth.vx, truth.vy) == pytest.approx((0.5, 0.0))
    diagonal = only(entity(BOX, path=[{'t': 0, 'x': 0, 'y': 0, 'yaw': math.pi / 4}]))
    assert diagonal.truth(0.0).size[:2] == pytest.approx((0.5 * math.sqrt(2),) * 2)
    assert only(PERSON).truth(0.0).size == (0.5, 0.5, 1.7)


def test_scene_truth_lists_the_active_entities_in_scene_order():
    parsed = scene(entity(spawn=5), BOX, CRATE)
    assert [truth.id for truth in parsed.truth(1.0)] == ['b1', 'crate']
    assert [truth.id for truth in parsed.truth(5.0)] == ['p1', 'b1', 'crate']


def test_playback_spawns_moves_and_removes_models_and_reports_each_removal_once():
    walker = entity(id='walker', path=[{'t': 0, 'x': 0, 'y': 0}, {'t': 2, 'x': 2, 'y': 0}])
    box = entity(BOX, spawn=1, remove=3)
    playback = Playback(scene(walker, box, CRATE))

    def summary(t):
        step = playback.step(t)
        return ([(spawned.id, pose[:2]) for spawned, pose in step.spawn],
                [(moved.id, pose[:2]) for moved, pose in step.move],
                [removed.id for removed in step.remove],
                [truth.id for truth in step.truth], step.removed_ids)

    assert summary(0.0) == ([('walker', (0, 0))], [], [], ['walker', 'crate'], [])
    assert summary(1.0) == ([('b1', (3.0, -1.0))], [('walker', (1, 0))], [],
                            ['walker', 'b1', 'crate'], [])
    assert summary(2.5) == ([], [('walker', (2, 0))], [], ['walker', 'b1', 'crate'], [])
    # held at the last waypoint: Gazebo already has that pose
    assert summary(2.8) == ([], [], [], ['walker', 'b1', 'crate'], [])
    assert summary(3.0) == ([], [], ['b1'], ['walker', 'crate'], ['b1'])
    assert summary(3.5) == ([], [], [], ['walker', 'crate'], [])


def test_model_sdf_is_a_static_model_with_matching_visual_and_collision():
    for data, geometry in ((PERSON, {'cylinder/radius': 0.25, 'cylinder/length': 1.7}),
                           (BOX, {'box/size': '0.6 0.4 0.5'})):
        parsed = only(data)
        model = ElementTree.fromstring(scene_script.model_sdf(parsed)).find('model')
        assert model.get('name') == parsed.model_name
        assert model.findtext('static') == 'true'
        for part in ('collision', 'visual'):
            shape = model.find(f'link/{part}/geometry')
            for path, value in geometry.items():
                text = shape.findtext(path)
                assert (text if isinstance(value, str) else float(text)) == value, (part, path)


# ---------------------------------------------------------------- shipped scenes

def load_lab_world():
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'tools',
                        'gen_world.py')
    if not os.path.isfile(path):
        pytest.skip('tools/gen_world.py not available')
    spec = importlib.util.spec_from_file_location('gen_world', path)
    world = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(world)
    return world


def distance_to_box(x, y, center_x, center_y, size_x, size_y):
    return math.hypot(max(abs(x - center_x) - size_x / 2, 0.0),
                      max(abs(y - center_y) - size_y / 2, 0.0))


def distance_to_segment(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def samples(parsed_entity, step=0.1):
    """Poses along the whole path, every `step` sim s."""
    start, end = parsed_entity.path[0].t, parsed_entity.path[-1].t
    return [parsed_entity.pose(start + index * step)
            for index in range(int((end - start) / step) + 1)]


# derived from the files present: a new scene needs no edit here
SHIPPED = sorted(name[:-5] for name in os.listdir(SCENES) if name.endswith('.yaml'))
SCENARIOS = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scenarios')


def test_scenes_ship_under_their_own_name():
    assert SHIPPED
    for name in SHIPPED:
        assert shipped(name).name == name


def test_every_scene_a_scenario_plays_ships():
    played = set()
    for path in glob.glob(os.path.join(SCENARIOS, '*.yaml')):
        with open(path) as file:
            played |= set(re.findall(r'scene:=([A-Za-z0-9_-]+)', file.read()))
    assert played and played <= set(SHIPPED)


def test_preexisting_entities_are_the_unmapped_world_objects_as_generated():
    """Truth and occluder: every scene declares each object the world has but
    the static map lacks (the crate), where and as large as tools/gen_world.py
    builds it; a mapped one declared too would hide itself from the detector."""
    world = load_lab_world()
    obstacles = {name: ((x, y, 0.0), (size_x, size_y, height), in_map)
                 for name, x, y, size_x, size_y, height, in_map in world.OBSTACLES}
    unmapped = {name for name, (_, _, in_map) in obstacles.items() if not in_map}
    assert unmapped
    for name in SHIPPED:
        preexisting = [parsed for parsed in shipped(name).entities if parsed.preexisting]
        assert {parsed.id for parsed in preexisting} == unmapped, name
        for parsed in preexisting:
            pose, size, _ = obstacles[parsed.id]
            assert (parsed.pose(0.0), parsed.shape.type, parsed.shape.size) == \
                (pose, 'box', size), (name, parsed.id)


@pytest.mark.parametrize('name', SHIPPED)
def test_scene_models_clear_walls_furniture_crate_and_docks(name):
    world = load_lab_world()
    boxes = list(world.wall_boxes()) + [(x, y, size_x, size_y)
                                        for _, x, y, size_x, size_y, _, _ in world.OBSTACLES]
    docks = [(x, y) for location_id, _, x, y, _ in world.LOCATIONS
             if location_id.startswith('dock')]
    for parsed in shipped(name).entities:
        if parsed.preexisting:
            continue
        radius = max(parsed.shape.size[:2]) / 2       # disc around the footprint
        for x, y, _ in samples(parsed):
            assert min(distance_to_box(x, y, *box) for box in boxes) > radius, (parsed.id, x, y)
            assert min(math.dist((x, y), dock) for dock in docks) >= 1.0, (parsed.id, x, y)


def test_box_front_box_is_ahead_of_the_dock_then_beside_it_then_gone():
    box = by_id(shipped('box_front'), 'box')
    assert box.shape.size == (0.4, 0.4, 0.5)
    assert box.pose(10.0)[:2] == (1.2, 0.0) and box.pose(30.0)[:2] == (1.2, -0.8)
    assert box.active(box.spawn) and not box.active(box.remove)
    assert box.spawn < 20.0 < 24.0 < box.remove


def test_person_hall_person_and_crate_are_seen_from_the_dock():
    parsed = shipped('person_hall')
    exact = DetectorParams(position_sigma=0.0, dropout=0.0, confidence_sigma=0.0)
    seen = simulate(parsed.truth(0.0), DOCK, map_loader.load(LAB_MAP), exact, random.Random(0))
    assert sorted(detection.truth_id for detection in seen) == ['crate', 'person_1']


def test_door_lab1_box_blocks_the_door_from_10_to_50():
    box = by_id(shipped('door_lab1_box_temporary'), 'door_box')
    assert box.pose(0.0)[:2] == (4.0, 1.5) and (box.spawn, box.remove) == (10.0, 50.0)
    # door_lab1: gap y 1.05..1.95 between the wall ends; neither side gap fits the robot
    side_gaps = (1.95 - (1.5 + box.shape.size[1] / 2), (1.5 - box.shape.size[1] / 2) - 1.05)
    assert max(side_gaps) < 0.3


def test_group_occupies_the_door_lab1_approach_for_the_whole_scene():
    persons = [parsed for parsed in shipped('group_door_lab1').entities
               if parsed.category == Category.PERSON]
    assert len(persons) == 2
    first, second = (person.pose(0.0)[:2] for person in persons)
    assert 0.7 <= math.dist(first, second) <= 0.9
    for person in persons:
        x, y, _ = person.pose(0.0)
        assert x < 4.0 - 0.05 - 0.25 and math.dist((x, y), (4.0, 1.5)) < 1.0   # hall side
        assert (person.spawn, person.remove, len(person.path)) == (0.0, None, 1)


def test_walker_crosses_the_hall_at_walking_speed_clear_of_the_dock_to_lab1_route():
    walker = by_id(shipped('walker_hall'), 'walker')
    assert walker.category == Category.PERSON and walker.path[-1].t >= 600.0
    for start, end in zip(walker.path, walker.path[1:]):
        speed = math.dist((start.x, start.y), (end.x, end.y)) / (end.t - start.t)
        assert speed == pytest.approx(0.8, abs=0.05)
    for x, y, _ in samples(walker):
        # straight dock -> door_lab1 line (Nav2 bends it round the crate, away from the walker)
        assert distance_to_segment(x, y, 0.0, 0.0, 4.0, 1.5) > 0.7
        assert math.dist((x, y), (2.5, 0.8)) > 1.0


# ------------------------------------------------------- detector reproducibility

def test_scene_truth_through_the_detector_is_reproducible_per_seed_and_robot():
    """The sim_detector pipeline on pure parts: scene truth and the lab map
    into detector_model, one Random per robot seeded '<seed>:<robot_id>'."""
    parsed = shipped('walker_hall')
    occupancy = map_loader.load(LAB_MAP)

    def frames(seed, robot_id):
        rng = random.Random(f'{seed}:{robot_id}')
        return [simulate(parsed.truth(tick * 0.2), DOCK, occupancy, DetectorParams(), rng,
                         exclude_id=robot_id) for tick in range(100)]

    first = frames(1, 'robot_1')
    assert first == frames(1, 'robot_1')
    assert first != frames(2, 'robot_1') and first != frames(1, 'robot_2')
    assert {detection.truth_id for frame in first for detection in frame} == {'walker', 'crate'}
