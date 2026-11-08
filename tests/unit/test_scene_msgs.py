"""Conversions between the pure scene-graph types and the contract messages
(fleetgraph_robot.scene_msgs): stamps, entities, relations, local deltas as
observations, observations (map frame only, unset stamps) and versioned
graph updates. Skips when the workspace is not built (no generated messages)."""
from dataclasses import replace

import pytest

messages = pytest.importorskip('fleetgraph_interfaces.msg')

from builtin_interfaces.msg import Time  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402

from fleetgraph_robot import scene_msgs  # noqa: E402
from fleetgraph_robot.central_graph import (  # noqa: E402
    CentralSceneGraph, GraphReplica, GraphUpdate)
from fleetgraph_robot.scene_graph import (  # noqa: E402
    Category, Delta, Entity, Predicate, Relation)

# confidences and stamps exactly representable in float32 / nanoseconds
CRATE = Entity('g/obstacle/1', Category.OBSTACLE, 'crate', 2.5, 0.8, z=0.25,
               size=(0.4, 0.6, 0.5), vx=0.1, vy=-0.2, confidence=0.75,
               first_seen=3.5, last_seen=7.25, source_ids=['robot_1', 'robot_2'],
               attributes={'persistence': 'temporary', 'source': 'lidar'})
MEETING = Entity('robot_1/group/7', Category.GROUP, 'group', 6.0, 1.4, confidence=0.5,
                 member_ids=['robot_1/person/1', 'robot_1/person/2'])
NEAR = Relation('g/person/2', Predicate.NEAR, 'g/obstacle/1', 0.5)


def test_time_converts_both_ways_at_nanosecond_resolution():
    assert scene_msgs.sec_to_time(12.25) == Time(sec=12, nanosec=250_000_000)
    assert scene_msgs.sec_to_time(12.3456789) == Time(sec=12, nanosec=345_678_900)
    assert scene_msgs.sec_to_time(1.9999999999) == Time(sec=2, nanosec=0)
    assert scene_msgs.time_to_sec(Time(sec=3, nanosec=500_000_000)) == 3.5
    assert scene_msgs.is_unset(Time()) and not scene_msgs.is_unset(Time(nanosec=1))


def test_entity_fields_map_onto_scene_entity():
    msg = scene_msgs.entity_to_msg(CRATE)
    assert isinstance(msg, messages.SceneEntity)
    assert (msg.id, msg.category, msg.label) == \
        ('g/obstacle/1', messages.SceneEntity.CATEGORY_OBSTACLE, 'crate')
    assert (msg.position.x, msg.position.y, msg.position.z) == (2.5, 0.8, 0.25)
    assert (msg.size.x, msg.size.y, msg.size.z) == (0.4, 0.6, 0.5)
    assert (msg.velocity.x, msg.velocity.y, msg.velocity.z) == (0.1, -0.2, 0.0)
    assert (msg.confidence, msg.first_seen, msg.last_seen) == \
        (0.75, Time(sec=3, nanosec=500_000_000), Time(sec=7, nanosec=250_000_000))
    assert list(msg.source_ids) == ['robot_1', 'robot_2']
    assert [(pair.key, pair.value) for pair in msg.attributes] == \
        [('persistence', 'temporary'), ('source', 'lidar')]
    assert list(scene_msgs.entity_to_msg(MEETING).member_ids) == MEETING.member_ids


def test_entities_round_trip_except_the_local_hit_count():
    for entity in (CRATE, MEETING):
        assert scene_msgs.entity_from_msg(scene_msgs.entity_to_msg(entity)) == entity
    tracked = scene_msgs.entity_from_msg(scene_msgs.entity_to_msg(replace(CRATE, hits=9)))
    assert tracked.hits == 0                    # tracker bookkeeping, not in the contract
    assert isinstance(tracked.size, tuple)


def test_relations_map_onto_scene_relation_and_back():
    msg = scene_msgs.relation_to_msg(NEAR)
    assert (msg.subject_id, msg.predicate, msg.object_id, msg.confidence) == \
        ('g/person/2', messages.SceneRelation.PREDICATE_NEAR, 'g/obstacle/1', 0.5)
    assert scene_msgs.relation_from_msg(msg) == NEAR


def test_a_local_delta_becomes_a_scene_observation_and_reads_back():
    """perception_node publishes LocalSceneGraph deltas this way; the central
    graph (mock_central) reads them back with observation_from_msg."""
    crate = Entity('robot_1/obstacle/4', Category.OBSTACLE, 'unknown', 2.5, 0.8, z=0.125,
                   size=(0.5, 0.5, 0.0), vx=0.25, vy=-0.5, confidence=0.875,
                   first_seen=20.25, last_seen=31.5, attributes={'source': 'lidar'}, hits=12)
    delta = Delta([crate, MEETING], ['robot_1/person/3'],
                  [Relation(crate.id, Predicate.INSIDE, 'hall', 0.875)])
    sensor_pose = PoseStamped()
    sensor_pose.header.frame_id = 'map'
    sensor_pose.header.stamp = Time(sec=31, nanosec=500_000_000)
    sensor_pose.pose.position.x, sensor_pose.pose.orientation.w = 1.0, 1.0
    msg = scene_msgs.observation_to_msg(delta, 'robot_1', 'map', sensor_pose, confidence=0.9)
    assert isinstance(msg, messages.SceneObservation)
    assert (msg.robot_id, msg.frame_id, msg.sensor_pose) == ('robot_1', 'map', sensor_pose)
    assert msg.confidence == pytest.approx(0.9)
    assert list(msg.removed_entity_ids) == ['robot_1/person/3']
    assert [entity.id for entity in msg.entities] == [crate.id, MEETING.id]
    assert list(msg.entities[0].source_ids) == [] and list(msg.entities[0].member_ids) == []
    (relation,) = msg.relations
    assert (relation.subject_id, relation.predicate, relation.object_id) == \
        (crate.id, messages.SceneRelation.PREDICATE_INSIDE, 'hall')
    msg.meta.stamp = Time(sec=32)                           # MetaStamper's part
    observation = scene_msgs.observation_from_msg(msg, received=33.0)
    assert (observation.robot_id, observation.stamp) == ('robot_1', 32.0)
    assert observation.entities == [replace(crate, hits=0),        # unset stamps: receipt
                                    replace(MEETING, first_seen=33.0, last_seen=33.0)]
    assert (observation.relations, observation.removed_ids) == (delta.relations, delta.removed_ids)


def observation_msg(entities, frame_id='map', stamp=Time(sec=20)):
    return messages.SceneObservation(
        meta=messages.Meta(source_id='robot_1', seq=4, stamp=stamp), robot_id='robot_1',
        frame_id=frame_id, entities=[scene_msgs.entity_to_msg(entity) for entity in entities],
        relations=[scene_msgs.relation_to_msg(NEAR)], removed_entity_ids=['robot_1/person/3'])


def test_a_map_frame_observation_becomes_a_fusion_observation():
    local = replace(CRATE, id='robot_1/obstacle/4', source_ids=[])
    observation = scene_msgs.observation_from_msg(observation_msg([local]), received=21.0)
    assert (observation.robot_id, observation.stamp) == ('robot_1', 20.0)
    assert (observation.entities, observation.relations, observation.removed_ids) == \
        ([local], [NEAR], ['robot_1/person/3'])


def test_observations_outside_the_map_frame_are_rejected_with_the_reason():
    for frame in ('odom', ''):
        with pytest.raises(ValueError, match=f"frame_id {frame!r} is not 'map'"):
            scene_msgs.observation_from_msg(observation_msg([CRATE], frame_id=frame), 21.0)


def test_unset_stamps_take_the_receipt_time():
    unstamped = replace(CRATE, first_seen=0.0, last_seen=0.0)
    half = replace(CRATE, id='robot_1/obstacle/5', first_seen=2.0, last_seen=0.0)
    observation = scene_msgs.observation_from_msg(
        observation_msg([unstamped, half], stamp=Time()), received=21.0)
    assert observation.stamp == 21.0
    assert [(entity.first_seen, entity.last_seen) for entity in observation.entities] == \
        [(21.0, 21.0), (2.0, 21.0)]


def test_graph_updates_map_onto_scene_graph_update_and_back():
    update = GraphUpdate(7, 6, False, 'robot_2', [CRATE], [MEETING], ['g/person/3'],
                         [NEAR], [Relation('g/person/3', Predicate.INSIDE, 'lab_1', 0.25)])
    msg = scene_msgs.graph_update_to_msg(update)
    assert isinstance(msg, messages.SceneGraphUpdate)
    assert (msg.frame_id, msg.graph_version, msg.base_version, msg.is_snapshot,
            msg.source_robot_id) == ('map', 7, 6, False, 'robot_2')
    assert ([entity.id for entity in msg.added_nodes],
            [entity.id for entity in msg.updated_nodes], list(msg.removed_node_ids)) == \
        (['g/obstacle/1'], ['robot_1/group/7'], ['g/person/3'])
    assert [relation.object_id for relation in msg.removed_relations] == ['lab_1']
    assert scene_msgs.graph_update_from_msg(msg) == update


def test_a_replica_fed_converted_updates_holds_the_central_graph():
    """Through the messages both ways, as mock_central and scene_replica do;
    only confidence narrows to float32."""
    graph, replica = CentralSceneGraph(), GraphReplica()
    reports = [('robot_1', [replace(CRATE, id='robot_1/obstacle/1', confidence=0.9,
                                    source_ids=[])]),
               ('robot_2', [replace(CRATE, id='robot_2/obstacle/1', x=2.6, confidence=0.6,
                                    source_ids=[])])]
    for robot, entities in reports:
        msg = observation_msg(entities)
        msg.robot_id = robot
        update = graph.ingest(scene_msgs.observation_from_msg(msg, received=21.0), 21.0)
        received = scene_msgs.graph_update_from_msg(scene_msgs.graph_update_to_msg(update))
        assert replica.apply(received) == 'applied'
    (fused,) = replica.entities()
    assert fused == replace(graph.entities()[0], confidence=pytest.approx(1 - 0.1 * 0.4))
    assert fused.source_ids == ['robot_1', 'robot_2']
    snapshot = scene_msgs.graph_update_from_msg(scene_msgs.graph_update_to_msg(graph.snapshot()))
    assert (snapshot.is_snapshot, snapshot.graph_version, snapshot.added) == (True, 2, [fused])
