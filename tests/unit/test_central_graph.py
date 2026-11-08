"""Mock Module 1 central scene graph and robot-side replica
(fleetgraph_robot.central_graph). Pure — no rclpy, no sim. Covers global
ids, cross-robot association and fusion, contribution removal and expiry,
relation union, persistence, versioned deltas, snapshots and replica
consistency over a seeded random three-robot run.
"""
import math
import random
from collections import Counter
from dataclasses import replace

import pytest

from fleetgraph_robot.central_graph import (
    CentralSceneGraph, FusionParams, GraphReplica, Observation, classify_persistence)
from fleetgraph_robot.regions import load_regions
from fleetgraph_robot.scene_graph import (
    Category, Detection, Entity, GraphParams, LocalSceneGraph, PHYSICAL, Predicate, Relation)

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


def track(robot, serial, category, x, y, confidence=0.9, seen=(0.0, 0.0), label='', **fields):
    """A robot-local entity as its robot reports it; seen = (first, last)."""
    return Entity(f'{robot}/{Category(category).name.lower()}/{serial}', category, label, x, y,
                  confidence=confidence, first_seen=seen[0], last_seen=seen[1], **fields)


def observe(graph, robot, now, entities=(), relations=(), removed=()):
    return graph.ingest(Observation(robot, now, list(entities), list(relations), list(removed)),
                        now)


def region_nodes():
    return LocalSceneGraph('robot_0', REGIONS).take_delta().entities


def by_id(graph):
    return {entity.id: entity for entity in graph.entities()}


def tracked(graph, category=None):
    return [entity for entity in graph.entities()
            if entity.category not in (Category.PLACE, Category.TRAVERSABLE)
            and (category is None or entity.category == category)]


def inside(entity, place, confidence):
    return Relation(entity.id, Predicate.INSIDE, place, confidence)


# ------------------------------------------------------------------- ids

def test_regions_keep_their_ids_and_are_shared_by_all_robots():
    graph = CentralSceneGraph()
    observe(graph, 'robot_1', 0.0, region_nodes())
    observe(graph, 'robot_2', 0.0, region_nodes())
    nodes = by_id(graph)
    assert set(nodes) == REGION_IDS
    assert (nodes['lab_1'].category, nodes['lab_1'].source_ids) == \
        (Category.PLACE, ['robot_1', 'robot_2'])
    assert nodes['door_lab1'].attributes == {'travel_axis': 'x'}


def test_global_ids_name_the_category_and_are_never_reused():
    graph = CentralSceneGraph()
    observe(graph, 'robot_1', 1.0, [track('robot_1', 1, Category.PERSON, 2.0, 0.0),
                                    track('robot_1', 2, Category.OBSTACLE, 2.0, 2.0)])
    assert [entity.id for entity in graph.entities()] == ['g/person/1', 'g/obstacle/2']
    observe(graph, 'robot_1', 2.0, removed=['robot_1/person/1'])
    observe(graph, 'robot_1', 3.0, [
        track('robot_1', 3, Category.PERSON, 2.0, 0.0, seen=(3.0, 3.0))])
    assert [entity.id for entity in graph.entities()] == ['g/obstacle/2', 'g/person/3']


# ----------------------------------------------------------- association

def test_one_thing_seen_by_two_robots_is_one_entity_and_categories_never_mix():
    graph = CentralSceneGraph()
    observe(graph, 'robot_1', 1.0, [track('robot_1', 1, Category.OBSTACLE, 2.5, 0.8),
                                    track('robot_1', 2, Category.PERSON, 1.0, -1.0)])
    observe(graph, 'robot_2', 1.0, [track('robot_2', 1, Category.OBSTACLE, 2.6, 0.8),
                                    track('robot_2', 2, Category.OBSTACLE, 1.0, -1.0)])
    assert {entity.id: entity.source_ids for entity in graph.entities()} == {
        'g/obstacle/1': ['robot_1', 'robot_2'],
        'g/person/2': ['robot_1'],
        'g/obstacle/3': ['robot_2']}            # where robot_1 sees a person


def test_a_new_track_joins_the_nearest_entity_its_robot_does_not_already_feed():
    graph = CentralSceneGraph(FusionParams(gate_radius={Category.PERSON: 0.6}))
    observe(graph, 'robot_1', 1.0, [track('robot_1', 1, Category.PERSON, 2.0, 0.0),
                                    track('robot_1', 2, Category.PERSON, 2.0, 1.0)])
    observe(graph, 'robot_2', 1.0, [track('robot_2', 1, Category.PERSON, 2.0, 0.6)])
    assert by_id(graph)['g/person/2'].source_ids == ['robot_1', 'robot_2']    # 0.4 beats 0.6
    # nearest is g/person/2 (now y 0.8), but robot_2 feeds it; g/person/1 is beyond the gate
    observe(graph, 'robot_2', 1.2, [track('robot_2', 2, Category.PERSON, 2.0, 0.9)])
    assert by_id(graph)['g/person/3'].source_ids == ['robot_2']


def test_new_tracks_of_one_robot_are_matched_nearest_pair_first():
    graph = CentralSceneGraph()
    observe(graph, 'robot_1', 1.0, [track('robot_1', 1, Category.PERSON, 2.0, 0.0)])
    observe(graph, 'robot_2', 1.0, [track('robot_2', 1, Category.PERSON, 2.0, -0.3),
                                    track('robot_2', 2, Category.PERSON, 2.0, 0.1)])
    walker, newcomer = graph.entities()
    assert (walker.source_ids, walker.y) == (['robot_1', 'robot_2'], pytest.approx(0.05))
    assert (newcomer.id, newcomer.source_ids, newcomer.y) == \
        ('g/person/2', ['robot_2'], pytest.approx(-0.3))


# ---------------------------------------------------------------- fusion

def test_position_is_the_confidence_weighted_mean_and_confidence_the_noisy_or():
    graph = CentralSceneGraph()
    observe(graph, 'robot_1', 2.0, [track('robot_1', 1, Category.OBJECT, 2.0, 0.0, confidence=0.9,
                                          seen=(1.0, 2.0), label='box', size=(0.4, 0.4, 0.5))])
    observe(graph, 'robot_2', 3.0, [track('robot_2', 1, Category.OBJECT, 2.3, 0.3, confidence=0.6,
                                          seen=(0.5, 3.0), label='crate', size=(0.6, 0.6, 0.5))])
    (box,) = graph.entities()
    assert (box.x, box.y) == pytest.approx(((0.9 * 2.0 + 0.6 * 2.3) / 1.5, 0.6 * 0.3 / 1.5))
    assert box.confidence == pytest.approx(1 - 0.1 * 0.4)
    assert (box.first_seen, box.last_seen) == (0.5, 3.0)
    assert (box.label, box.size) == ('box', (0.4, 0.4, 0.5))     # from the most confident report
    assert box.hits == 0


def test_group_members_are_mapped_to_global_ids_and_merged_across_robots():
    graph = CentralSceneGraph()
    a = track('robot_1', 1, Category.PERSON, 6.0, 1.0)
    b = track('robot_1', 2, Category.PERSON, 6.0, 1.8)
    meeting = track('robot_1', 3, Category.GROUP, 6.0, 1.4,
                    member_ids=[a.id, b.id, 'robot_1/person/99'])    # 99 never reported
    observe(graph, 'robot_1', 1.0, [meeting, a, b])                 # listed before its members
    (group,) = tracked(graph, Category.GROUP)
    assert (group.id, group.member_ids) == ('g/group/1', ['g/person/2', 'g/person/3'])
    # robot_2 sees member a and a third person c, 0.7 m below a: beyond the person gate
    a2 = track('robot_2', 1, Category.PERSON, 6.05, 1.0)
    c2 = track('robot_2', 2, Category.PERSON, 6.0, 0.3)
    meeting2 = track('robot_2', 3, Category.GROUP, 6.0, 0.65, member_ids=[a2.id, c2.id])
    observe(graph, 'robot_2', 1.0, [a2, c2, meeting2])
    (group,) = tracked(graph, Category.GROUP)
    assert (group.member_ids, group.source_ids) == \
        (['g/person/2', 'g/person/3', 'g/person/4'], ['robot_1', 'robot_2'])


# ------------------------------------------------------------- removal

def test_removed_ids_drop_only_that_robots_contribution():
    graph = CentralSceneGraph()
    observe(graph, 'robot_1', 1.0, [
        track('robot_1', 1, Category.OBSTACLE, 2.5, 0.8, confidence=0.9)])
    observe(graph, 'robot_2', 1.0, [
        track('robot_2', 7, Category.OBSTACLE, 2.7, 0.8, confidence=0.6)])
    update = observe(graph, 'robot_1', 2.0, removed=['robot_1/obstacle/1'])
    (crate,) = graph.entities()
    assert (crate.id, crate.source_ids) == ('g/obstacle/1', ['robot_2'])
    assert (crate.x, crate.confidence) == pytest.approx((2.7, 0.6))
    assert ([entity.id for entity in update.updated], update.removed_ids) == (['g/obstacle/1'], [])
    update = observe(graph, 'robot_2', 3.0, removed=['robot_2/obstacle/7'])
    assert graph.entities() == [] and update.removed_ids == ['g/obstacle/1']
    assert observe(graph, 'robot_2', 4.0, removed=['robot_2/obstacle/7']) is None   # unknown id


# ---------------------------------------------------------------- expiry

def test_contributions_expire_after_the_ttl_without_reobservation():
    graph = CentralSceneGraph(FusionParams(ttl={Category.PERSON: 5.0}))
    observe(graph, 'robot_1', 1.0, [
        track('robot_1', 1, Category.PERSON, 2.0, 0.0, seen=(0.0, 1.0))])
    observe(graph, 'robot_2', 2.0, [
        track('robot_2', 1, Category.PERSON, 2.1, 0.0, seen=(2.0, 2.0))])
    assert graph.expire(6.0) is None
    update = graph.expire(6.5)                          # robot_1 silent since 1.0
    (walker,) = graph.entities()
    assert walker.source_ids == ['robot_2'] and walker.x == pytest.approx(2.1)
    assert ([entity.id for entity in update.updated], update.source_robot_id) == ([walker.id], '')
    update = graph.expire(7.5)                          # nobody re-observed it for 5 s
    assert graph.entities() == [] and update.removed_ids == [walker.id]


def test_regions_never_expire_and_obstacles_outlive_persons():
    graph = CentralSceneGraph()
    observe(graph, 'robot_1', 0.0, region_nodes() + [
        track('robot_1', 1, Category.PERSON, 2.0, 0.0),
        track('robot_1', 2, Category.OBSTACLE, 2.5, 0.8)])
    graph.expire(10.0)
    assert [entity.category for entity in tracked(graph)] == [Category.OBSTACLE]
    graph.expire(1000.0)
    assert set(by_id(graph)) == REGION_IDS


# ------------------------------------------------------------- relations

def test_relations_are_the_union_over_robots_at_the_highest_confidence():
    graph = CentralSceneGraph()
    observe(graph, 'robot_1', 0.0, region_nodes())          # robot_2 never reports the regions
    mine = track('robot_1', 1, Category.PERSON, 6.0, 1.2)
    yours = track('robot_2', 4, Category.PERSON, 6.1, 1.2)
    observe(graph, 'robot_1', 1.0, [mine], [inside(mine, 'lab_1', 0.7)])
    observe(graph, 'robot_2', 1.0, [yours], [inside(yours, 'lab_1', 0.9)])
    (walker,) = tracked(graph)
    assert graph.relations() == [Relation(walker.id, Predicate.INSIDE, 'lab_1', 0.9)]
    # every observation carries the robot's full set: robot_2 dropped it, robot_1 still has it
    update = observe(graph, 'robot_2', 2.0)
    assert update.added_relations == graph.relations() == \
        [Relation(walker.id, Predicate.INSIDE, 'lab_1', 0.7)]
    assert update.removed_relations == []
    update = observe(graph, 'robot_1', 3.0)
    assert graph.relations() == []
    assert update.removed_relations == [Relation(walker.id, Predicate.INSIDE, 'lab_1', 0.7)]


def test_relation_endpoints_map_to_global_ids_and_unknown_ones_drop_the_relation():
    graph = CentralSceneGraph()
    walker = track('robot_1', 1, Category.PERSON, 3.3, 0.8)
    box = track('robot_1', 2, Category.OBJECT, 3.3, 1.5)
    observe(graph, 'robot_1', 1.0, [walker, box], [
        Relation(walker.id, Predicate.NEAR, box.id, 0.8),
        Relation(box.id, Predicate.NEAR, 'door_lab1', 0.9),            # region never reported
        Relation('robot_1/person/99', Predicate.NEAR, box.id, 0.8)])   # track never reported
    assert graph.relations() == [Relation('g/person/1', Predicate.NEAR, 'g/object/2', 0.8)]


def test_a_robots_relations_end_with_its_contributions():
    graph = CentralSceneGraph(FusionParams(ttl={Category.PERSON: 5.0}))
    observe(graph, 'robot_1', 0.0, region_nodes())
    mine = track('robot_1', 1, Category.PERSON, 6.0, 1.2, seen=(0.0, 1.0))
    observe(graph, 'robot_1', 1.0, [mine], [inside(mine, 'lab_1', 0.8)])
    observe(graph, 'robot_2', 4.0, [
        track('robot_2', 1, Category.PERSON, 6.0, 1.2, seen=(0.0, 4.0))])
    update = graph.expire(6.5)                          # robot_1 went silent after 1.0
    (walker,) = tracked(graph)
    assert walker.source_ids == ['robot_2'] and graph.relations() == []
    assert update.removed_relations == [Relation(walker.id, Predicate.INSIDE, 'lab_1', 0.8)]


# ----------------------------------------------------------- persistence

def test_obstacles_turn_persistent_after_being_present_long_enough():
    graph = CentralSceneGraph(FusionParams(persistent_after_sec=30.0))
    box = track('robot_1', 1, Category.OBJECT, 2.0, 0.0, seen=(0.0, 0.0))
    crate = track('robot_1', 2, Category.OBSTACLE, 2.5, 0.8, seen=(10.0, 10.0))
    walker = track('robot_1', 3, Category.PERSON, 0.0, -2.0, seen=(0.0, 10.0))
    observe(graph, 'robot_1', 10.0, [box, crate, walker])
    assert [entity.attributes.get('persistence') for entity in graph.entities()] == \
        ['temporary', 'temporary', None]
    observe(graph, 'robot_1', 25.0, [replace(box, last_seen=25.0)], removed=[walker.id])
    assert graph.expire(29.9) is None
    update = graph.expire(30.0)                         # the box has been there since 0.0
    assert [(entity.id, entity.attributes['persistence']) for entity in update.updated] == \
        [('g/object/1', 'persistent')]
    update = graph.expire(40.0)
    assert [(entity.id, entity.attributes['persistence']) for entity in update.updated] == \
        [('g/obstacle/2', 'persistent')]


def test_classify_persistence_applies_to_obstacles_and_objects_only():
    params = FusionParams(persistent_after_sec=30.0)
    box = Entity('g/object/1', Category.OBJECT, 'box', 0.0, 0.0, first_seen=10.0)
    assert classify_persistence(box, 39.9, params) == 'temporary'
    assert classify_persistence(box, 40.0, params) == 'persistent'
    assert classify_persistence(replace(box, category=Category.OBSTACLE), 40.0) == 'persistent'
    for category in (Category.PERSON, Category.GROUP, Category.ROBOT, Category.PLACE):
        assert classify_persistence(replace(box, category=category), 40.0, params) is None


# ------------------------------------------------------------ versioning

def test_each_change_is_one_version_and_unchanged_input_reports_nothing():
    graph = CentralSceneGraph(FusionParams(change_position_eps=0.05))
    assert graph.version == 0
    box = track('robot_1', 1, Category.OBJECT, 2.0, 0.0)
    first = observe(graph, 'robot_1', 1.0, [box])
    assert (first.graph_version, first.base_version, first.is_snapshot, first.source_robot_id) == \
        (1, 0, False, 'robot_1')
    assert ([entity.id for entity in first.added], graph.version) == (['g/object/1'], 1)
    assert observe(graph, 'robot_1', 1.2, [box]) is None
    assert observe(graph, 'robot_1', 1.4, [replace(box, x=2.03)]) is None    # below the epsilon
    assert graph.entities()[0].x == 2.0
    moved = observe(graph, 'robot_1', 1.6, [replace(box, x=2.1)])
    assert (moved.graph_version, moved.base_version, graph.version) == (2, 1, 2)
    assert [entity.x for entity in moved.updated] == [pytest.approx(2.1)]
    moved.updated[0].x = 99.0                                   # updates and queries are copies
    graph.entities()[0].attributes['persistence'] = 'tampered'
    assert by_id(graph)['g/object/1'].x == pytest.approx(2.1)
    assert by_id(graph)['g/object/1'].attributes['persistence'] == 'temporary'


def test_snapshot_carries_the_whole_graph_at_the_current_version():
    graph = CentralSceneGraph()
    observe(graph, 'robot_1', 0.0, region_nodes())
    walker = track('robot_1', 1, Category.PERSON, 6.0, 1.2)
    observe(graph, 'robot_1', 1.0, [walker], [inside(walker, 'lab_1', 0.8)])
    snapshot = graph.snapshot()
    assert (snapshot.is_snapshot, snapshot.graph_version, snapshot.source_robot_id) == (True, 2, '')
    assert snapshot.added == graph.entities() and len(snapshot.added) == len(REGIONS) + 1
    assert snapshot.added_relations == graph.relations() != []
    assert (snapshot.updated, snapshot.removed_ids, snapshot.removed_relations) == ([], [], [])


# --------------------------------------------------------------- replica

def test_replica_applies_deltas_in_order_and_resyncs_from_a_snapshot():
    graph, replica = CentralSceneGraph(), GraphReplica()
    updates = [observe(graph, 'robot_1', float(step), [
        track('robot_1', 1, Category.PERSON, 2.0 + 0.5 * step, 0.0, seen=(0.0, float(step)))])
        for step in range(4)]
    assert replica.apply(updates[0]) == 'applied'
    assert replica.apply(updates[0]) == 'duplicate'
    assert replica.apply(updates[2]) == 'gap'                  # updates[1] was lost
    assert (replica.version, replica.entities()[0].x) == (1, pytest.approx(2.0))
    assert replica.apply(updates[3]) == 'gap'
    assert replica.apply(graph.snapshot()) == 'snapshot'
    assert (replica.version, replica.entities(), replica.relations()) == \
        (graph.version, graph.entities(), graph.relations())


def test_a_restarted_central_graph_is_adopted_through_its_snapshot():
    old, replica = CentralSceneGraph(), GraphReplica()
    for step in range(3):
        replica.apply(observe(old, 'robot_1', float(step), [
            track('robot_1', 1, Category.PERSON, 2.0 + 0.5 * step, 0.0)]))
    restarted = CentralSceneGraph()
    first = observe(restarted, 'robot_2', 5.0, [track('robot_2', 1, Category.OBJECT, 1.0, 1.0)])
    assert (replica.version, replica.apply(first)) == (3, 'duplicate')    # version 1 looks old
    assert replica.apply(restarted.snapshot()) == 'snapshot'
    assert (replica.version, replica.entities()) == (1, restarted.entities())


def test_a_replica_fed_every_update_reproduces_the_graph_of_a_random_three_robot_run():
    rng = random.Random(5)
    graph = CentralSceneGraph(FusionParams(persistent_after_sec=10.0))
    replica, lossy = GraphReplica(), GraphReplica()
    robots = {robot_id: LocalSceneGraph(robot_id, REGIONS)
              for robot_id in ('robot_1', 'robot_2', 'robot_3')}
    lookout = {'robot_1': (1.0, 0.0), 'robot_2': (3.0, 1.0), 'robot_3': (5.5, -0.5)}
    walkers = [[2.0, 0.3], [2.0, -0.5]] + [[rng.uniform(0.0, 7.0), rng.uniform(-2.5, 2.5)]
                                           for _ in range(4)]
    boxes = [(2.5, 0.8), (3.3, 1.5), (6.0, -2.0), (4.5, 2.0)]
    lost = 15                           # index of the update the lossy replica never gets
    delivered, seen = [], Counter()

    def deliver(update):
        if update is None:
            return
        assert (update.graph_version, update.base_version) == (graph.version, graph.version - 1)
        assert replica.apply(update) == 'applied'
        assert (replica.version, replica.entities(), replica.relations()) == \
            (graph.version, graph.entities(), graph.relations())
        delivered.append(update)
        if len(delivered) - 1 == lost + 1:
            assert lossy.apply(update) == 'gap' and lossy.version == lost
            assert lossy.apply(graph.snapshot()) == 'snapshot'
        elif len(delivered) - 1 != lost:
            assert lossy.apply(update) == 'applied'
        changed = update.added + update.updated
        seen['fused'] += any(len(entity.source_ids) > 1 for entity in changed
                             if entity.category not in (Category.PLACE, Category.TRAVERSABLE))
        seen['group'] += any(entity.category == Category.GROUP for entity in update.added)
        seen['persistent'] += any(entity.attributes.get('persistence') == 'persistent'
                                  for entity in update.updated)
        seen['removed'] += len(update.removed_ids)
        seen['unrelated'] += len(update.removed_relations)

    def sighting(category, x, y, size, label):
        return Detection(category, x + rng.gauss(0.0, 0.05), y + rng.gauss(0.0, 0.05), size=size,
                         confidence=rng.uniform(0.6, 1.0), label=label, source='detector')

    for step in range(150):
        now = step * 0.2
        for walker in walkers:
            walker[0] = min(8.5, max(-0.5, walker[0] + rng.gauss(0.0, 0.08)))
            walker[1] = min(2.5, max(-2.5, walker[1] + rng.gauss(0.0, 0.08)))
        for robot_id, local in robots.items():
            if robot_id == 'robot_3' and 60 <= step < 110:
                continue                    # offline: its contributions expire centrally
            cx, cy = lookout[robot_id]

            def in_view(x, y):
                return math.hypot(x - cx, y - cy) <= 3.5

            detections = [sighting(Category.PERSON, x, y, (0.5, 0.5, 1.7), 'person')
                          for x, y in walkers if in_view(x, y) and rng.random() > 0.1]
            detections += [sighting(Category.OBJECT, x, y, (0.4, 0.4, 0.5), 'box')
                           for x, y in boxes if in_view(x, y) and rng.random() > 0.1]
            local.integrate(detections, now, in_view)
            delta = local.take_delta()
            deliver(graph.ingest(Observation(robot_id, now, delta.entities, delta.relations,
                                             delta.removed_ids), now))
        if step % 5 == 4:
            deliver(graph.expire(now))
    print(f'\n{len(delivered)} updates, {dict(seen)}')
    assert len(delivered) > 100
    assert all(seen[key] > 0 for key in ('fused', 'group', 'persistent', 'removed', 'unrelated'))
    assert (lossy.version, lossy.entities(), lossy.relations()) == \
        (graph.version, graph.entities(), graph.relations())


def test_malformed_observations_are_rejected_without_changing_the_graph():
    graph = CentralSceneGraph()
    good = track('robot_1', 1, Category.PERSON, 2.0, 0.0)
    cases = [
        ('robot_id', Observation('', 0.0, [good], [], [])),
        ('category', Observation(
            'robot_1', 0.0, [good, replace(good, id='robot_1/x/2', category=42)], [], [])),
        ('confidence', Observation('robot_1', 0.0, [replace(good, confidence=1.5)], [], [])),
        ('finite', Observation('robot_1', 0.0, [replace(good, x=math.nan)], [], [])),
    ]
    for match, observation in cases:
        with pytest.raises(ValueError, match=match):
            graph.ingest(observation, 0.0)
    with pytest.raises(ValueError, match='now'):
        graph.ingest(Observation('robot_1', 0.0, [good], [], []), math.inf)
    assert (graph.version, graph.entities()) == (0, [])


def test_fusion_params_default_like_the_local_graph_merge_partial_overrides_and_validate():
    defaults, local = FusionParams(), GraphParams()
    for category in PHYSICAL:
        assert defaults.gate_radius[category] == local.gate_radius[category]
        assert defaults.ttl[category] == local.ttl[category]
    params = FusionParams(gate_radius={Category.PERSON: 0.8}, ttl={Category.GROUP: 9.0})
    assert params.gate_radius[Category.PERSON] == 0.8
    assert params.gate_radius[Category.OBSTACLE] == defaults.gate_radius[Category.OBSTACLE]
    assert params.ttl[Category.GROUP] == 9.0
    with pytest.raises(ValueError, match='gate_radius'):
        FusionParams(gate_radius={Category.PLACE: 1.0})          # regions are never associated
    with pytest.raises(ValueError, match='ttl'):
        FusionParams(ttl={Category.PERSON: 0.0})
    with pytest.raises(ValueError, match='persistent_after_sec'):
        FusionParams(persistent_after_sec=-1.0)
