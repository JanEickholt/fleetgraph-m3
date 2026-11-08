"""Local scene graph (fleetgraph_robot.scene_graph). Pure — no rclpy, no sim.
Covers track ids, association and duplicate handling, confidence and
removal, per-sensor coverage, lidar -> semantic re-identification, groups,
relations, delta semantics and the per-frame time budget.
"""
import copy
import math
import random
import statistics
import time

import pytest

from fleetgraph_robot.regions import Region, load_regions
from fleetgraph_robot.scene_graph import (
    Category, Detection, GraphParams, LocalSceneGraph, Predicate)

# Floor plan of tools/gen_world.py: hall x -1..4, labs x 4..9 split at y = 0,
# 1 m doors in the wall x = 4.
REGIONS = load_regions({'regions': [
    {'id': 'hall', 'kind': 'place', 'xmin': -1, 'ymin': -3, 'xmax': 4, 'ymax': 3},
    {'id': 'lab_1', 'kind': 'place', 'xmin': 4, 'ymin': 0, 'xmax': 9, 'ymax': 3},
    {'id': 'lab_2', 'kind': 'place', 'xmin': 4, 'ymin': -3, 'xmax': 9, 'ymax': 0},
    {'id': 'door_lab1', 'kind': 'traversable', 'xmin': 3.8, 'ymin': 1, 'xmax': 4.2, 'ymax': 2,
     'travel_axis': 'x'},
    {'id': 'door_lab2', 'kind': 'traversable', 'xmin': 3.8, 'ymin': -2, 'xmax': 4.2, 'ymax': -1,
     'travel_axis': 'x'},
]})
REGION_IDS = {region.id for region in REGIONS}
DT = 0.2                                    # 5 Hz detector
IN_VIEW = lambda x, y: True                 # noqa: E731
OUT_OF_VIEW = lambda x, y: False            # noqa: E731


def make_graph(regions=REGIONS, **params):
    return LocalSceneGraph('robot_1', regions, GraphParams(**params))


def person(x, y, confidence=0.9, size=(0.6, 0.6, 1.7)):
    return Detection(Category.PERSON, x, y, z=0.85, size=size, confidence=confidence,
                     label='person', source='detector')


def thing(x, y, side=0.6, confidence=0.9):
    return Detection(Category.OBJECT, x, y, z=0.25, size=(side, side, 0.5),
                     confidence=confidence, label='box', source='detector')


def lidar(x, y, side=0.4, confidence=0.7):
    return Detection(Category.OBSTACLE, x, y, size=(side, side, 0.0), confidence=confidence,
                     label='unknown', source='lidar')


def feed(graph, frames, start=0.0, in_view=None):
    """One integrate() per detection list, DT apart from `start`."""
    for index, detections in enumerate(frames):
        graph.integrate(list(detections), start + index * DT, in_view)


def tracked(graph, category=None, include_tentative=False):
    return [entity for entity in graph.entities(include_tentative)
            if entity.category not in (Category.PLACE, Category.TRAVERSABLE)
            and (category is None or entity.category == category)]


def relation_set(graph):
    return {(relation.subject_id, Predicate(relation.predicate).name, relation.object_id)
            for relation in graph.relations()}


# ------------------------------------------------------------------- ids

def test_region_nodes_are_reported_once_and_never_removed():
    graph = make_graph()
    nodes = {entity.id: entity for entity in graph.take_delta().entities}
    assert set(nodes) == REGION_IDS
    assert (nodes['lab_1'].category, nodes['lab_1'].confidence) == (Category.PLACE, 1.0)
    assert (nodes['lab_1'].x, nodes['lab_1'].y, nodes['lab_1'].size[:2]) == (6.5, 1.5, (5.0, 3.0))
    assert nodes['door_lab1'].category == Category.TRAVERSABLE
    assert nodes['door_lab1'].attributes == {'travel_axis': 'x'}
    feed(graph, [[person(2.0, 0.0)]] * 3)
    reported = [graph.take_delta()]
    feed(graph, [[]] * 10, start=1.0, in_view=IN_VIEW)
    reported.append(graph.take_delta())
    assert all(not REGION_IDS & {entity.id for entity in delta.entities} for delta in reported)
    assert reported[1].removed_ids and not REGION_IDS & set(reported[1].removed_ids)
    assert graph.get('lab_1').category == Category.PLACE


def test_track_ids_use_robot_category_and_a_counter_that_is_never_reused():
    graph = make_graph()
    graph.integrate([lidar(0.0, -2.0), person(2.0, 0.0)], 0.0)
    assert sorted(entity.id for entity in tracked(graph, include_tentative=True)) == \
        ['robot_1/obstacle/2', 'robot_1/person/1']        # semantic detections go first
    graph.integrate([], 100.0)                              # both past their TTL
    assert tracked(graph, include_tentative=True) == []
    graph.integrate([person(2.0, 0.0)], 100.2)
    assert [entity.id for entity in tracked(graph, include_tentative=True)] == \
        ['robot_1/person/3']


# ----------------------------------------------------------- association

def test_one_noisy_person_keeps_one_id_and_gains_confidence():
    rng = random.Random(1)
    graph = make_graph()
    reported_ids = set()
    for frame in range(20):
        graph.integrate([person(2.0 + rng.gauss(0, 0.05), 1.0 + rng.gauss(0, 0.05))], frame * DT)
        reported_ids |= {entity.id for entity in graph.take_delta().entities} - REGION_IDS
    (only,) = tracked(graph, include_tentative=True)
    assert reported_ids == {only.id}
    assert only.confidence > 0.8 and only.hits == 20
    assert math.hypot(only.x - 2.0, only.y - 1.0) < 0.1


def test_two_persons_apart_stay_two_entities():
    graph = make_graph()
    feed(graph, [[person(2.0, 0.0), person(2.0, 1.5)]] * 10)
    assert len(tracked(graph, Category.PERSON, include_tentative=True)) == 2


def test_detections_closer_than_merge_radius_collapse_into_one_entity():
    graph = make_graph(merge_radius=0.3)
    graph.integrate([person(2.0, 0.0), person(2.1, 0.0)], 0.0)
    assert len(tracked(graph, include_tentative=True)) == 1


def test_same_frame_duplicates_beside_a_track_do_not_spawn_a_second_track():
    graph = make_graph(merge_radius=0.3, position_smoothing=0.5)
    feed(graph, [[person(2.0, 0.0)]] * 3)
    # seen twice, 0.15 m apart: one copy alone would pull the track to 2.2 and
    # leave the other 0.35 m away, beyond merge_radius
    graph.integrate([person(2.4, 0.0), person(2.55, 0.0)], 3 * DT)
    assert len(tracked(graph, include_tentative=True)) == 1


def test_jitter_never_creates_a_duplicate():
    rng = random.Random(7)
    graph = make_graph()
    for frame in range(50):
        graph.integrate([person(1.0 + rng.gauss(0, 0.05), -1.0 + rng.gauss(0, 0.05))],
                        frame * DT)
        assert len(tracked(graph, include_tentative=True)) == 1


def test_semantic_detections_match_only_their_own_category():
    graph = make_graph()
    feed(graph, [[person(2.0, 0.0)]] * 3)
    graph.integrate([thing(2.1, 0.0)], 3 * DT)            # a box where the person stands
    assert sorted(entity.category for entity in tracked(graph, include_tentative=True)) == \
        [Category.PERSON, Category.OBJECT]


def test_integrate_accepts_any_iterable_of_detections():
    graph = make_graph()
    graph.integrate((detection for detection in [person(2.0, 0.0)]), 0.0)
    assert len(tracked(graph, include_tentative=True)) == 1


def test_rejects_detections_that_are_not_physical_or_malformed():
    graph = make_graph()
    with pytest.raises(ValueError, match='GROUP'):
        graph.integrate([Detection(Category.GROUP, 0.0, 0.0)], 0.0)
    with pytest.raises(ValueError, match='finite'):
        graph.integrate([Detection(Category.PERSON, float('nan'), 0.0)], 0.0)
    with pytest.raises(ValueError, match='confidence'):
        graph.integrate([Detection(Category.PERSON, 0.0, 0.0, confidence=1.5)], 0.0)
    with pytest.raises(ValueError, match='lidar'):          # range evidence has no class
        graph.integrate([Detection(Category.PERSON, 0.0, 0.0, source='lidar')], 0.0)


# ------------------------------------------------------------ confidence

def test_confidence_follows_the_hit_rule_and_tentative_entities_stay_silent():
    graph = make_graph(confidence_gain=0.5, report_min_confidence=0.5)
    graph.integrate([person(2.0, 0.0, confidence=0.8)], 0.0)
    (entity,) = tracked(graph, include_tentative=True)
    assert entity.confidence == pytest.approx(0.4)           # 0 + 0.5 * 0.8 * (1 - 0)
    assert tracked(graph) == [] and graph.relations() == []
    assert {reported.id for reported in graph.take_delta().entities} == REGION_IDS
    graph.integrate([person(2.0, 0.0, confidence=0.8)], DT)
    assert graph.get(entity.id).confidence == pytest.approx(0.4 + 0.5 * 0.8 * 0.6)
    assert [reported.id for reported in graph.take_delta().entities] == [entity.id]


def test_out_of_view_entity_keeps_its_confidence_until_the_ttl():
    graph = make_graph(ttl={Category.PERSON: 5.0})
    feed(graph, [[person(2.0, 0.0)]] * 3)                   # last hit at 0.4
    (entity,) = tracked(graph)
    graph.take_delta()
    graph.integrate([], 5.3, OUT_OF_VIEW)
    assert graph.get(entity.id).confidence == entity.confidence
    graph.integrate([], 5.5, OUT_OF_VIEW)
    assert graph.get(entity.id) is None
    assert graph.take_delta().removed_ids == [entity.id]
    assert graph.take_delta().removed_ids == []


def test_in_view_unmatched_entity_decays_and_is_removed_exactly_once():
    graph = make_graph(miss_decay=0.5, remove_below=0.1)
    feed(graph, [[person(2.0, 0.0)]] * 3)
    (entity,) = tracked(graph)
    removed = list(graph.take_delta().removed_ids)
    confidences, now = [], 0.4
    while graph.get(entity.id) is not None:
        now += DT
        graph.integrate([], now, IN_VIEW)
        if graph.get(entity.id) is not None:
            confidences.append(graph.get(entity.id).confidence)
        removed += graph.take_delta().removed_ids
        assert now < 3.0
    assert confidences == pytest.approx(
        [entity.confidence * 0.5 ** misses for misses in range(1, len(confidences) + 1)])
    assert removed == [entity.id]


def test_only_entities_in_view_decay():
    graph = make_graph(miss_decay=0.7)
    feed(graph, [[person(2.0, 0.0), person(6.0, -2.0)]] * 3)
    before = {round(entity.x): entity.confidence for entity in tracked(graph)}
    graph.integrate([], 1.0, lambda x, y: x < 4.0)          # only the hall is in view
    after = {round(entity.x): entity.confidence
             for entity in tracked(graph, include_tentative=True)}
    assert after[2] == pytest.approx(before[2] * 0.7)
    assert after[6] == pytest.approx(before[6])


def test_without_in_view_unmatched_entities_do_not_decay():
    graph = make_graph()
    feed(graph, [[person(2.0, 0.0)]] * 3)
    (entity,) = tracked(graph)
    graph.integrate([], 1.0)
    assert graph.get(entity.id).confidence == entity.confidence


# --------------------------------------------------- per-sensor coverage
# As in perception_node: one update per 10 Hz lidar scan, every other one
# also carrying a 5 Hz detector frame.
SCAN = {'lidar': IN_VIEW}
SCAN_AND_FRAME = {'lidar': IN_VIEW, 'detector': IN_VIEW}


def scans(graph, count, seen=(), start=0.0, frame_view=IN_VIEW):
    """count updates 0.1 s apart from start; even ones carry a detector
    frame (covering frame_view) with the `seen` detections."""
    for index in range(count):
        frame = index % 2 == 0
        coverage = {'lidar': IN_VIEW, 'detector': frame_view} if frame else SCAN
        graph.integrate(list(seen) if frame else [], start + index * 0.1, coverage)
        yield index, frame


def test_camera_only_track_between_lidar_scans_stays_reported():
    """The flicker: scans counted as misses for tracks the lidar does not
    see, which pulled a person at the detector's nominal confidence (0.85)
    below report_min_confidence after every scan between two frames."""
    graph = make_graph()
    visitor, deltas = None, []
    for index, _ in scans(graph, 40, seen=[person(2.0, 0.0, confidence=0.85)]):
        if index >= 2:                      # reported from the second frame on
            (visitor,) = tracked(graph)
            assert visitor.confidence >= 0.5
        if index % 5 == 4:                  # take_delta at the 2 Hz publish rate
            deltas.append(graph.take_delta())
    assert all(visitor.id in {entity.id for entity in delta.entities}
               and (visitor.id, Predicate.INSIDE, 'hall') in
               {(relation.subject_id, relation.predicate, relation.object_id)
                for relation in delta.relations}
               and not delta.removed_ids for delta in deltas)


def test_unseen_camera_track_decays_only_on_detector_frames():
    graph = make_graph(miss_decay=0.7)
    list(scans(graph, 6, seen=[person(2.0, 0.0)]))
    (entity,) = tracked(graph)
    confidence = entity.confidence
    for _, frame in scans(graph, 6, start=0.6):     # gone: empty frames still look there
        confidence *= 0.7 if frame else 1.0
        assert graph.get(entity.id).confidence == pytest.approx(confidence)


def test_camera_track_outside_the_detector_view_keeps_its_confidence():
    graph = make_graph(miss_decay=0.7)
    list(scans(graph, 6, seen=[person(2.0, 0.0)]))
    (entity,) = tracked(graph)
    list(scans(graph, 10, start=0.6, frame_view=lambda x, y: x > 3.0))    # turned away
    assert graph.get(entity.id).confidence == entity.confidence
    graph.integrate([], 1.6, {'detector': lambda x, y: x < 3.0})          # back in view
    assert graph.get(entity.id).confidence == pytest.approx(entity.confidence * 0.7)


def test_lidar_backed_tracks_decay_on_every_scan_as_before():
    graph = make_graph(miss_decay=0.7)
    graph.integrate([lidar(2.5, 0.8)], 0.0, SCAN)
    graph.integrate([lidar(2.5, 0.8), person(1.0, -2.0)], 0.1, SCAN_AND_FRAME)
    graph.integrate([lidar(2.5, 0.8), lidar(1.1, -2.0)], 0.2, SCAN)    # the person, too
    (obstacle,) = tracked(graph, Category.OBSTACLE, include_tentative=True)
    (walker,) = tracked(graph, Category.PERSON, include_tentative=True)
    expected = {obstacle.id: obstacle.confidence, walker.id: walker.confidence}
    for now, coverage in ((0.3, SCAN), (0.4, SCAN_AND_FRAME), (0.5, SCAN)):
        graph.integrate([], now, coverage)              # one miss per update, not per sensor
        expected = {entity_id: value * 0.7 for entity_id, value in expected.items()}
        assert {entity_id: graph.get(entity_id).confidence for entity_id in expected} == \
            pytest.approx(expected)
    graph.integrate([], 0.6, {'detector': IN_VIEW})  # the detector never saw the obstacle
    assert graph.get(obstacle.id).confidence == pytest.approx(expected[obstacle.id])
    assert graph.get(walker.id).confidence == pytest.approx(expected[walker.id] * 0.7)


def test_upgrades_merges_and_echoes_keep_the_lidar_evidence():
    upgraded = make_graph(miss_decay=0.7)
    feed(upgraded, [[lidar(2.5, 0.8)]] * 2 + [[person(2.55, 0.8)]])
    echoed = make_graph(miss_decay=0.7, merge_radius=0.3)
    feed(echoed, [[person(2.0, 0.0), lidar(2.4, 0.0)]] * 3)
    merged = make_graph(position_smoothing=0.0, merge_radius=0.3, miss_decay=0.7)
    feed(merged, [[lidar(2.0, 0.7)]] * 3)
    feed(merged, [[person(2.0, 0.0)]] * 3, start=0.6)
    for step, y in enumerate((0.45, 0.25)):
        merged.integrate([lidar(2.0, y)], 1.2 + step * DT)
    for graph in (upgraded, echoed, merged):
        (entity,) = tracked(graph, include_tentative=True)
        assert entity.category == Category.PERSON
        graph.integrate([], 2.0, SCAN)                  # a scan that misses it
        assert graph.get(entity.id).confidence == pytest.approx(entity.confidence * 0.7)


def test_a_plain_in_view_covers_for_every_sensor():
    graph = make_graph(miss_decay=0.7)
    graph.integrate([lidar(2.5, 0.8), person(1.0, -2.0)], 0.0)
    before = {entity.id: entity.confidence for entity in tracked(graph, include_tentative=True)}
    graph.integrate([], 0.1, IN_VIEW)
    assert {entity_id: graph.get(entity_id).confidence for entity_id in before} == \
        pytest.approx({entity_id: value * 0.7 for entity_id, value in before.items()})
    with pytest.raises(TypeError, match='coverage'):
        graph.integrate([], 0.2, [IN_VIEW])


# -------------------------------------------------------- lidar evidence

def test_lidar_cluster_then_semantic_person_upgrades_the_track():
    graph = make_graph(confidence_gain=0.5)
    feed(graph, [[lidar(2.5, 0.8)]] * 3)
    (obstacle,) = tracked(graph)
    assert (obstacle.id, obstacle.category, obstacle.label) == \
        ('robot_1/obstacle/1', Category.OBSTACLE, 'unknown')
    assert obstacle.attributes == {'source': 'lidar'}
    assert obstacle.id in {entity.id for entity in graph.take_delta().entities}
    graph.integrate([person(2.55, 0.8, confidence=0.9)], 3 * DT)
    (upgraded,) = tracked(graph)
    assert (upgraded.id, upgraded.category, upgraded.label) == \
        ('robot_1/person/2', Category.PERSON, 'person')
    assert upgraded.first_seen == obstacle.first_seen
    assert upgraded.confidence == pytest.approx(           # inherited, then one hit
        obstacle.confidence + 0.5 * 0.9 * (1 - obstacle.confidence))
    assert upgraded.size == (0.6, 0.6, 1.7) and upgraded.attributes == {'source': 'detector'}
    assert graph.get(obstacle.id) is None
    delta = graph.take_delta()
    assert delta.removed_ids == [obstacle.id]
    assert [entity.id for entity in delta.entities] == [upgraded.id]


def test_upgrading_a_tentative_lidar_track_reports_no_removal():
    graph = make_graph()
    graph.integrate([lidar(2.5, 0.8)], 0.0)                 # tentative
    graph.take_delta()
    graph.integrate([person(2.5, 0.8)], DT)
    delta = graph.take_delta()
    assert delta.removed_ids == []
    assert [entity.category for entity in delta.entities] == [Category.PERSON]


def test_semantic_obstacle_refines_a_lidar_track_without_a_new_id():
    graph = make_graph()
    feed(graph, [[lidar(2.5, 0.8)]] * 3)
    (obstacle,) = tracked(graph)
    labelled = Detection(Category.OBSTACLE, 2.5, 0.8, size=(0.4, 0.4, 0.5), label='crate',
                         source='detector')
    graph.integrate([labelled], 3 * DT)
    (refined,) = tracked(graph)
    assert (refined.id, refined.label, refined.attributes) == \
        (obstacle.id, 'crate', {'source': 'detector'})


def test_lidar_detections_reinforce_a_person_instead_of_creating_obstacles():
    graph = make_graph()
    feed(graph, [[person(2.0, 0.0)]] * 3)
    (before,) = tracked(graph)
    feed(graph, [[lidar(2.15, 0.05)]] * 5, start=1.0)
    (after,) = tracked(graph, include_tentative=True)
    assert (after.id, after.category, after.label) == (before.id, Category.PERSON, 'person')
    assert after.hits == before.hits + 5 and after.last_seen == pytest.approx(1.8)
    assert after.size == before.size                       # lidar never resizes semantic tracks


def test_lidar_echo_of_a_person_in_the_same_frame_creates_no_obstacle():
    graph = make_graph(merge_radius=0.3)
    feed(graph, [[person(2.0, 0.0), lidar(2.4, 0.0)]] * 5)   # 0.4 m apart, footprints overlap
    assert [entity.category for entity in tracked(graph, include_tentative=True)] == \
        [Category.PERSON]


def test_lidar_obstacle_away_from_people_is_tracked():
    graph = make_graph()
    graph.integrate([person(2.0, 0.0), lidar(2.0, 1.0)], 0.0)
    assert sorted(entity.category for entity in tracked(graph, include_tentative=True)) == \
        [Category.OBSTACLE, Category.PERSON]


# --------------------------------------------------------------- merging

def test_converging_tracks_merge_into_the_older_id():
    # objects: converging persons would also form (and dissolve) a group
    graph = make_graph(position_smoothing=0.0, merge_radius=0.3)
    feed(graph, [[thing(2.0, 0.0)]] * 3)                    # older, seen 0.0 .. 0.4
    feed(graph, [[thing(2.0, 1.0)]] * 3, start=0.6)         # younger, beyond the 0.5 m gate
    older, younger = sorted(tracked(graph), key=lambda entity: entity.first_seen)
    graph.take_delta()
    for step, y in enumerate((0.6, 0.35, 0.2)):             # always nearer the younger track
        graph.integrate([thing(2.0, y)], 1.2 + step * DT)
    (merged,) = tracked(graph)
    assert (merged.id, merged.first_seen) == (older.id, older.first_seen)
    assert (merged.x, merged.y) == pytest.approx((2.0, 0.2))   # the fresher state wins
    assert graph.take_delta().removed_ids == [younger.id]


def test_lidar_track_converging_onto_a_person_merges_into_the_person():
    graph = make_graph(position_smoothing=0.0, merge_radius=0.3)
    feed(graph, [[lidar(2.0, 0.7)]] * 3)                    # older, lidar only
    feed(graph, [[person(2.0, 0.0)]] * 3, start=0.6)        # beyond the person gate: new track
    (obstacle,) = tracked(graph, Category.OBSTACLE)
    (walker,) = tracked(graph, Category.PERSON)
    graph.take_delta()
    for step, y in enumerate((0.45, 0.25)):
        graph.integrate([lidar(2.0, y)], 1.2 + step * DT)
    (merged,) = tracked(graph)
    assert (merged.id, merged.first_seen) == (walker.id, obstacle.first_seen)
    assert graph.take_delta().removed_ids == [obstacle.id]


def test_lidar_track_converging_onto_a_labelled_obstacle_keeps_the_older_id_and_the_label():
    graph = make_graph(position_smoothing=0.0, merge_radius=0.3)
    feed(graph, [[lidar(2.0, 0.7)]] * 3)                    # older, lidar only
    crate = Detection(Category.OBSTACLE, 2.0, 0.0, z=0.25, size=(0.4, 0.4, 0.5),
                      label='crate', source='detector')
    feed(graph, [[crate]] * 3, start=0.6)                   # beyond the gate: new track
    older, younger = sorted(tracked(graph), key=lambda entity: entity.first_seen)
    graph.take_delta()
    for step, y in enumerate((0.45, 0.25)):
        graph.integrate([lidar(2.0, y)], 1.2 + step * DT)
    (merged,) = tracked(graph)
    assert (merged.id, merged.label, merged.size, merged.attributes) == \
        (older.id, 'crate', (0.4, 0.4, 0.5), {'source': 'detector'})
    assert graph.take_delta().removed_ids == [younger.id]


# ------------------------------------------------------------ kinematics

def test_velocity_from_smoothed_positions_zero_after_one_hit():
    graph = make_graph(position_smoothing=0.5)
    graph.integrate([person(0.0, 0.0)], 0.0)
    (first,) = tracked(graph, include_tentative=True)
    assert (first.vx, first.vy) == (0.0, 0.0)
    feed(graph, [[person(0.5 * step * DT, 0.0)] for step in range(1, 30)], start=DT)
    (walker,) = tracked(graph)
    assert walker.vx == pytest.approx(0.5, abs=0.02) and walker.vy == pytest.approx(0.0)
    # EMA lag at constant speed: v * DT * s / (1 - s) = 0.1 m
    assert walker.x == pytest.approx(0.5 * 29 * DT - 0.1, abs=0.01)


def test_zero_smoothing_tracks_raw_detections():
    graph = make_graph(position_smoothing=0.0)
    graph.integrate([person(1.0, 1.0)], 0.0)
    graph.integrate([person(1.2, 1.0)], DT)
    (walker,) = tracked(graph)
    assert (walker.x, walker.y, walker.vx) == pytest.approx((1.2, 1.0, 1.0))


# ---------------------------------------------------------------- groups

def test_persons_within_group_radius_form_a_group():
    graph = make_graph(group_radius=1.2)
    feed(graph, [[person(6.0, 1.0, confidence=0.9), person(6.0, 1.8, confidence=0.8)]] * 3)
    (group,) = tracked(graph, Category.GROUP)
    members = tracked(graph, Category.PERSON)
    assert group.id == 'robot_1/group/3' and group.label == 'group'
    assert group.member_ids == sorted(member.id for member in members)
    assert (group.x, group.y) == pytest.approx((6.0, 1.4))
    assert group.size[:2] == pytest.approx((0.6, 1.4))         # members +- 0.3 m radius
    assert group.confidence == pytest.approx(min(member.confidence for member in members))


def test_groups_chain_through_intermediate_members():
    graph = make_graph(group_radius=1.2)
    feed(graph, [[person(5.0, 1.0), person(6.0, 1.0), person(7.0, 1.0)]] * 3)
    (group,) = tracked(graph, Category.GROUP)
    assert len(group.member_ids) == 3                     # 5 and 7 are linked through 6


def test_persons_further_apart_form_no_group():
    graph = make_graph(group_radius=1.2)
    feed(graph, [[person(6.0, 0.5), person(6.0, 2.0)]] * 3)
    assert tracked(graph, Category.GROUP) == []


def test_group_id_survives_a_join_and_a_dissolved_group_is_removed_once():
    graph = make_graph(miss_decay=0.3)
    feed(graph, [[person(6.0, 1.0), person(6.0, 1.8)]] * 3)
    (group,) = tracked(graph, Category.GROUP)
    feed(graph, [[person(6.0, 1.0), person(6.0, 1.8), person(6.0, 2.6)]] * 3, start=0.6)
    (joined,) = tracked(graph, Category.GROUP)
    assert (joined.id, len(joined.member_ids)) == (group.id, 3)     # overlap 2/3
    removed, now = list(graph.take_delta().removed_ids), 1.0
    while tracked(graph, Category.GROUP):                 # the first two leave
        now += DT
        graph.integrate([person(6.0, 2.6)], now, IN_VIEW)
        removed += graph.take_delta().removed_ids
        assert now < 5.0
    assert removed.count(group.id) == 1


@pytest.mark.parametrize('arrivals, keeps_id', [
    ([(7.0, 1.0)], True),                     # {A, B} -> {A, C}: overlap 1/2
    ([(7.0, 1.0), (5.0, 1.0)], False),        # {A, B} -> {A, C, D}: overlap 1/3
])
def test_group_id_needs_half_the_members(arrivals, keeps_id):
    graph = make_graph(miss_decay=0.3, group_radius=1.2)
    a, b, c, d = (6.0, 1.0), (6.0, 1.8), (7.3, 1.0), (4.7, 1.0)    # C, D 1.3 m from A
    feed(graph, [[person(*a), person(*b), person(*c), person(*d)]] * 3)
    (before,) = tracked(graph, Category.GROUP)
    graph.take_delta()
    stayed = [person(*position) for position in ((7.3, 1.0), (4.7, 1.0))[len(arrivals):]]
    detections = [person(*a)] + [person(*position) for position in arrivals] + stayed
    graph.integrate(detections, 3 * DT, IN_VIEW)          # B unseen: drops out of the group
    (after,) = tracked(graph, Category.GROUP)
    assert len(after.member_ids) == 1 + len(arrivals)
    assert (after.id == before.id) is keeps_id
    assert graph.take_delta().removed_ids == ([] if keeps_id else [before.id])


# ------------------------------------------------------------- relations

def test_person_in_lab_1_is_inside_lab_1():
    graph = make_graph()
    feed(graph, [[person(6.0, 1.2)]] * 3)
    (visitor,) = tracked(graph)
    assert {object_id for subject_id, predicate, object_id in relation_set(graph)
            if subject_id == visitor.id and predicate == 'INSIDE'} == {'lab_1'}


def test_box_centred_in_a_door_blocks_it():
    graph = make_graph(min_clearance=0.6)
    feed(graph, [[thing(4.0, 1.5, side=0.6)]] * 3)        # leaves 0.2 m on each side
    (box,) = tracked(graph)
    assert (box.id, 'BLOCKS', 'door_lab1') in relation_set(graph)


def test_person_at_the_door_edge_occupies_it():
    graph = make_graph(min_clearance=0.6)
    feed(graph, [[person(4.0, 1.0)]] * 3)                 # r 0.3 covers 1.0 .. 1.3: 0.7 m left
    (visitor,) = tracked(graph)
    relations = relation_set(graph)
    assert (visitor.id, 'OCCUPIES', 'door_lab1') in relations
    assert (visitor.id, 'BLOCKS', 'door_lab1') not in relations


def test_two_persons_in_a_door_form_a_blocking_group():
    graph = make_graph(min_clearance=0.6, group_radius=1.2)
    feed(graph, [[person(4.0, 1.1), person(4.0, 1.9)]] * 3)
    (group,) = tracked(graph, Category.GROUP)
    assert (group.id, 'BLOCKS', 'door_lab1') in relation_set(graph)


def test_robot_never_passes_between_group_members():
    corridor = [Region('corridor', 'traversable', 0.0, 0.0, 5.0, 1.9, travel_axis='x')]
    slim = (0.4, 0.4, 1.7)
    # 1.1 m apart: 0.7 m free between them, only 0.2 m on either side
    detections = [person(2.0, 0.4, size=slim), person(2.0, 1.5, size=slim)]
    grouped = make_graph(corridor, min_clearance=0.6, group_radius=1.2)
    apart = make_graph(corridor, min_clearance=0.6, group_radius=1.0)
    for graph in (grouped, apart):
        feed(graph, [detections] * 3)
    assert len(tracked(grouped, Category.GROUP)) == 1
    assert {predicate for _, predicate, _ in relation_set(grouped)} == {'BLOCKS'}
    assert {predicate for _, predicate, _ in relation_set(apart)} == {'OCCUPIES'}


def test_near_relations_between_agents_things_and_doors():
    graph = make_graph(near_distance=0.5)
    feed(graph, [[thing(3.3, 1.5, side=0.4), person(3.3, 0.8), person(1.0, -2.0)]] * 3)
    (box,) = tracked(graph, Category.OBJECT)
    near, far = sorted(tracked(graph, Category.PERSON), key=lambda entity: -entity.y)
    relations = relation_set(graph)
    assert (near.id, 'NEAR', box.id) in relations         # 0.7 - 0.3 - 0.2 = 0.2 m apart
    assert (far.id, 'NEAR', box.id) not in relations
    assert (box.id, 'NEAR', 'door_lab1') in relations     # 0.5 - 0.2 = 0.3 m from the door
    assert not {(box.id, 'OCCUPIES', 'door_lab1'), (box.id, 'BLOCKS', 'door_lab1')} & relations


def test_relation_confidences():
    graph = make_graph()
    feed(graph, [[person(6.0, 1.2, confidence=0.9), thing(6.0, 2.0, side=0.4,
                                                          confidence=0.6)]] * 3)
    (visitor,) = tracked(graph, Category.PERSON)
    (box,) = tracked(graph, Category.OBJECT)
    confidence = {(relation.subject_id, relation.predicate, relation.object_id):
                  relation.confidence for relation in graph.relations()}
    assert confidence[(visitor.id, Predicate.INSIDE, 'lab_1')] == \
        pytest.approx(visitor.confidence)
    assert confidence[(visitor.id, Predicate.NEAR, box.id)] == \
        pytest.approx(min(visitor.confidence, box.confidence))


# ----------------------------------------------------------------- delta

def test_delta_reports_hits_and_changes_but_not_unchanged_entities():
    graph = make_graph(change_confidence_eps=0.05)
    feed(graph, [[person(2.0, 0.0)]] * 3)
    (visitor,) = tracked(graph)
    graph.take_delta()
    graph.integrate([person(2.0, 0.0)], 0.6)               # re-observed
    assert [entity.id for entity in graph.take_delta().entities] == [visitor.id]
    graph.integrate([], 0.8)                               # nothing changed
    quiet = graph.take_delta()
    assert (quiet.entities, quiet.removed_ids) == ([], [])
    assert quiet.relations == graph.relations() != []      # full relation set every time
    graph.integrate([], 1.0, IN_VIEW)                      # decayed: changed, still reported
    assert [entity.id for entity in graph.take_delta().entities] == [visitor.id]


def test_delta_entities_are_snapshots():
    graph = make_graph()
    feed(graph, [[person(2.0, 0.0)]] * 3)
    (reported,) = [entity for entity in graph.take_delta().entities
                   if entity.category == Category.PERSON]
    graph.integrate([person(2.4, 0.0)], 1.0)
    assert reported.x == pytest.approx(2.0)


def test_params_merge_partial_per_category_overrides_and_validate():
    params = GraphParams(gate_radius={Category.PERSON: 0.8})
    assert params.gate_radius[Category.PERSON] == 0.8
    assert params.gate_radius[Category.OBSTACLE] == GraphParams().gate_radius[Category.OBSTACLE]
    with pytest.raises(ValueError, match='miss_decay'):
        GraphParams(miss_decay=1.5)
    with pytest.raises(ValueError, match='gate_radius'):
        GraphParams(gate_radius={Category.PERSON: 0.0})


# ------------------------------------------------------------ time budget

def test_integrate_30_detections_on_10_tracks_plus_relations_is_fast():
    rng = random.Random(3)
    graph = make_graph()
    existing = [person(x, y) for x, y in ((0.0, -2.0), (1.5, -2.0), (3.0, -2.0),
                                          (6.0, 1.0), (7.5, 2.0))]     # too far apart to group
    existing += [thing(x, y, side=0.4) for x, y in ((0.0, 2.0), (2.5, 0.8), (3.3, 1.5),
                                                    (6.0, -2.0), (8.0, -1.0))]
    feed(graph, [existing] * 3)
    assert len(tracked(graph)) == 10
    detections = [Detection(d.category, d.x + rng.gauss(0, 0.05), d.y + rng.gauss(0, 0.05),
                            size=d.size, confidence=0.85, label=d.label, source=d.source)
                  for d in existing]
    detections += [person(-0.5 + 0.9 * (index % 5), -0.8 + 0.9 * (index // 5))
                   for index in range(10)]
    detections += [lidar(4.6 + 0.9 * (index % 5), 0.5 + 0.9 * (index // 5) - 2.5)
                   for index in range(10)]
    assert len(detections) == 30
    samples = []
    for _ in range(15):
        trial = copy.deepcopy(graph)
        start = time.perf_counter()
        trial.integrate(detections, 1.0)
        trial.relations()
        samples.append(time.perf_counter() - start)
    median = statistics.median(samples)
    print(f'\nintegrate(30 detections, 10 tracks) + relations(): median '
          f'{median * 1e3:.2f} ms, max {max(samples) * 1e3:.2f} ms')
    assert median < 0.020
