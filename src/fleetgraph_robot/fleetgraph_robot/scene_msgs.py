"""Conversions between the pure scene-graph types (scene_graph.Entity and
Relation, central_graph.Observation and GraphUpdate) and the contract
messages (tests/unit/test_scene_msgs.py). Times are seconds on the ROS
clock (sim time in simulation); Meta is left to the publisher's
MetaStamper.

  SceneEntity       <-> Entity        (hits is tracker bookkeeping: not sent)
  SceneRelation     <-> Relation
  SceneObservation   -> Observation   (map frame only)
  SceneGraphUpdate  <-> GraphUpdate
"""
from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import KeyValue
from geometry_msgs.msg import Point, Vector3

from fleetgraph_interfaces.msg import SceneEntity, SceneGraphUpdate, SceneRelation
from fleetgraph_robot.central_graph import GraphUpdate, Observation
from fleetgraph_robot.scene_graph import Entity, Relation

MAP_FRAME = 'map'


def sec_to_time(seconds: float) -> Time:
    sec, nanosec = divmod(round(seconds * 1e9), 1_000_000_000)
    return Time(sec=sec, nanosec=nanosec)


def time_to_sec(stamp: Time) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def is_unset(stamp: Time) -> bool:
    """Zero: the publisher did not stamp it."""
    return stamp.sec == 0 and stamp.nanosec == 0


def entity_to_msg(entity: Entity) -> SceneEntity:
    size_x, size_y, size_z = entity.size
    return SceneEntity(
        id=entity.id, category=int(entity.category), label=entity.label,
        position=Point(x=float(entity.x), y=float(entity.y), z=float(entity.z)),
        size=Vector3(x=float(size_x), y=float(size_y), z=float(size_z)),
        velocity=Vector3(x=float(entity.vx), y=float(entity.vy)),
        confidence=float(entity.confidence),
        first_seen=sec_to_time(entity.first_seen), last_seen=sec_to_time(entity.last_seen),
        member_ids=list(entity.member_ids), source_ids=list(entity.source_ids),
        attributes=[KeyValue(key=key, value=value) for key, value in entity.attributes.items()])


def entity_from_msg(msg: SceneEntity) -> Entity:
    return Entity(
        msg.id, msg.category, msg.label, msg.position.x, msg.position.y, z=msg.position.z,
        size=(msg.size.x, msg.size.y, msg.size.z), vx=msg.velocity.x, vy=msg.velocity.y,
        confidence=msg.confidence, first_seen=time_to_sec(msg.first_seen),
        last_seen=time_to_sec(msg.last_seen), member_ids=list(msg.member_ids),
        source_ids=list(msg.source_ids),
        attributes={pair.key: pair.value for pair in msg.attributes})


def relation_to_msg(relation: Relation) -> SceneRelation:
    return SceneRelation(subject_id=relation.subject_id, predicate=int(relation.predicate),
                         object_id=relation.object_id, confidence=float(relation.confidence))


def relation_from_msg(msg: SceneRelation) -> Relation:
    return Relation(msg.subject_id, msg.predicate, msg.object_id, msg.confidence)


def observation_from_msg(msg, received: float) -> Observation:
    """SceneObservation heard at `received` (receiver clock). ValueError
    unless its positions are in the map frame.

    Unset stamps take the receipt time, so unstamped publishers (synthetic
    observations) work on any timebase: the central graph keeps a global
    entity's earliest first_seen, which then is when it was first heard,
    and ages contributions from the last message."""
    if msg.frame_id != MAP_FRAME:
        raise ValueError(f'frame_id {msg.frame_id!r} is not {MAP_FRAME!r}: '
                         f'entity positions must be in the map frame')
    entities = []
    for entity_msg in msg.entities:
        entity = entity_from_msg(entity_msg)
        entity.first_seen = _seconds_or(entity_msg.first_seen, received)
        entity.last_seen = _seconds_or(entity_msg.last_seen, received)
        entities.append(entity)
    return Observation(msg.robot_id, _seconds_or(msg.meta.stamp, received), entities,
                       [relation_from_msg(relation) for relation in msg.relations],
                       list(msg.removed_entity_ids))


def graph_update_to_msg(update: GraphUpdate) -> SceneGraphUpdate:
    return SceneGraphUpdate(
        source_robot_id=update.source_robot_id, frame_id=MAP_FRAME,
        graph_version=update.graph_version, base_version=update.base_version,
        is_snapshot=update.is_snapshot,
        added_nodes=[entity_to_msg(entity) for entity in update.added],
        updated_nodes=[entity_to_msg(entity) for entity in update.updated],
        removed_node_ids=list(update.removed_ids),
        added_relations=[relation_to_msg(relation) for relation in update.added_relations],
        removed_relations=[relation_to_msg(relation) for relation in update.removed_relations])


def graph_update_from_msg(msg: SceneGraphUpdate) -> GraphUpdate:
    return GraphUpdate(
        msg.graph_version, msg.base_version, msg.is_snapshot, msg.source_robot_id,
        [entity_from_msg(entity) for entity in msg.added_nodes],
        [entity_from_msg(entity) for entity in msg.updated_nodes],
        list(msg.removed_node_ids),
        [relation_from_msg(relation) for relation in msg.added_relations],
        [relation_from_msg(relation) for relation in msg.removed_relations])


def _seconds_or(stamp, default):
    return default if is_unset(stamp) else time_to_sec(stamp)
