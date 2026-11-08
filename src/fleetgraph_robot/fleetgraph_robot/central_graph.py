"""Mock of Module 1's central scene graph, and the robot-side replica of it.
Pure Python (no rclpy): a mock-M1 node feeds each SceneObservation to
ingest(), calls expire() on a timer, publishes every returned GraphUpdate
as a SceneGraphUpdate and answers GetSceneGraph with snapshot(); robot
nodes apply the updates to a GraphReplica
(tests/unit/test_central_graph.py).

Each robot-local track id feeds one global entity for its whole life. A new
track joins the nearest global entity of its category within the gate that
its robot does not already feed, else it founds one ("g/<category>/<n>",
never reused); floor-plan regions keep their ids and are shared. A global
entity is the confidence-weighted mean of its contributors' latest reports,
with noisy-OR confidence; label, size and attributes come from the most
confident report. A contribution ends when its robot removes the track or
stops re-observing it for the category's ttl; the entity ends with its last
contribution.

entities() and relations() return the published graph. An entity is
republished when its position, size or confidence moves past the change
epsilons or its label, members, sources or attributes change; last_seen and
velocity only ride along. A replica that applied every update therefore
holds exactly the same graph.
"""
import math
from dataclasses import dataclass, field, fields, replace

from fleetgraph_robot.scene_graph import Category, Entity, Relation

REGION_CATEGORIES = (Category.PLACE, Category.TRAVERSABLE)
FUSED = (Category.OBSTACLE, Category.PERSON, Category.GROUP, Category.OBJECT, Category.ROBOT)
_PERSISTENCE = (Category.OBSTACLE, Category.OBJECT)


@dataclass
class Observation:
    """SceneObservation in the map frame. Ids are the robot's local track
    ids (region ids for regions); relations are its full current set."""
    robot_id: str
    stamp: float
    entities: list[Entity]
    relations: list[Relation]
    removed_ids: list[str]


@dataclass
class GraphUpdate:
    """SceneGraphUpdate: a delta applies on top of base_version, a snapshot
    replaces the receiver's graph. An added relation replaces any relation
    with the same (subject, predicate, object)."""
    graph_version: int
    base_version: int
    is_snapshot: bool
    source_robot_id: str            # '' when no observation caused it
    added: list[Entity]
    updated: list[Entity]
    removed_ids: list[str]
    added_relations: list[Relation]
    removed_relations: list[Relation]


def _per_category(obstacle, person, group, thing, robot):
    return field(default_factory=lambda: {
        Category.OBSTACLE: obstacle, Category.PERSON: person, Category.GROUP: group,
        Category.OBJECT: thing, Category.ROBOT: robot})


@dataclass
class FusionParams:
    """Distances in m, times in sim s. Per-category dicts may be partial:
    missing categories keep their defaults (GraphParams' plus groups)."""
    gate_radius: dict[int, float] = _per_category(0.5, 0.6, 1.0, 0.5, 0.6)
    ttl: dict[int, float] = _per_category(30.0, 5.0, 5.0, 30.0, 5.0)    # without re-observation
    persistent_after_sec: float = 30.0
    change_position_eps: float = 0.05       # position and size
    change_confidence_eps: float = 0.05

    def __post_init__(self):
        for spec in fields(self):
            if spec.name in ('gate_radius', 'ttl'):
                merged = _merged(spec.name, spec.default_factory(), getattr(self, spec.name))
                setattr(self, spec.name, merged)
        for name in ('persistent_after_sec', 'change_position_eps', 'change_confidence_eps'):
            if not getattr(self, name) >= 0:
                raise ValueError(f'{name} must be >= 0, got {getattr(self, name)!r}')


def _merged(name, defaults, overrides):
    for key, value in dict(overrides).items():
        if key not in FUSED:
            raise ValueError(f'{name}: {_category_name(key)} is not a fused category')
        if not value > 0:
            raise ValueError(f'{name}[{Category(key).name}] must be > 0, got {value!r}')
        defaults[Category(key)] = float(value)
    return defaults


def classify_persistence(entity: Entity, now: float,
                         params: FusionParams | None = None) -> str | None:
    """'persistent' once an OBSTACLE or OBJECT has been present for
    persistent_after_sec since it was first seen, 'temporary' before; None
    for other categories."""
    if entity.category not in _PERSISTENCE:
        return None
    params = params if params is not None else FusionParams()
    return 'persistent' if now - entity.first_seen >= params.persistent_after_sec else 'temporary'


@dataclass
class _Node:
    id: str
    category: int
    reports: dict[str, Entity] = field(default_factory=dict)    # robot id -> latest report
    # over the node's whole life, so persistence never resets when a robot leaves
    first_seen: float = math.inf
    last_seen: float = -math.inf


class CentralSceneGraph:
    def __init__(self, params: FusionParams | None = None):
        self._params = params if params is not None else FusionParams()
        self._nodes: dict[str, _Node] = {}              # creation order
        self._owner: dict[tuple[str, str], str] = {}    # (robot id, local id) -> global id
        self._robot_relations: dict[str, list[Relation]] = {}  # latest full set, local ids
        self._next_serial = 1
        self._version = 0
        self._published: dict[str, Entity] = {}
        self._published_relations: dict[tuple, Relation] = {}

    # ---- queries

    @property
    def version(self) -> int:
        return self._version

    def entities(self) -> list[Entity]:
        return [_copy(entity) for entity in self._published.values()]

    def relations(self) -> list[Relation]:
        return [self._published_relations[key] for key in sorted(self._published_relations)]

    def snapshot(self) -> GraphUpdate:
        return GraphUpdate(self._version, 0, True, '', self.entities(), [], [],
                           self.relations(), [])

    # ---- update

    def ingest(self, observation: Observation, now: float) -> GraphUpdate | None:
        """None when the published graph did not change."""
        _validate(observation, now)
        robot = observation.robot_id
        # removals first: a robot that re-creates a track may rejoin its old entity
        for local_id in observation.removed_ids:
            self._drop(robot, local_id)
        fresh = []
        for entity in {entity.id: entity for entity in observation.entities}.values():
            node = self._nodes.get(self._owner.get((robot, entity.id), ''))
            if node is not None and node.category != entity.category:
                self._drop(robot, entity.id)        # ids encode the category: never mix
                node = None
            if node is None and entity.category in REGION_CATEGORIES:
                node = self._nodes.get(entity.id) or self._add_node(entity.id, entity.category)
            if node is None:
                fresh.append(entity)
            else:
                self._report(node, robot, entity)
        self._associate(robot, fresh)
        self._robot_relations[robot] = list(observation.relations)
        return self._publish(robot, now)

    def expire(self, now: float) -> GraphUpdate | None:
        """Ends contributions not re-observed for their category's ttl (a
        silent robot's included) and publishes persistence changes."""
        if not math.isfinite(now):
            raise ValueError(f'now must be finite, got {now!r}')
        for node in list(self._nodes.values()):
            if node.category in REGION_CATEGORIES:
                continue
            for robot, report in list(node.reports.items()):
                if now - report.last_seen > self._params.ttl[node.category]:
                    self._drop(robot, report.id)
        return self._publish('', now)

    def _associate(self, robot, fresh):
        """Nearest pair first, so the order of new tracks never decides."""
        fed = {node.id for node in self._nodes.values() if robot in node.reports}
        candidates = [(serial, node, self._position(node))
                      for serial, node in enumerate(self._nodes.values())
                      if node.id not in fed and node.category not in REGION_CATEGORIES]
        pairs = []
        for index, entity in enumerate(fresh):
            gate = self._params.gate_radius[entity.category]
            for serial, node, (x, y) in candidates:
                if node.category == entity.category:
                    distance = math.hypot(entity.x - x, entity.y - y)
                    if distance <= gate:
                        pairs.append((distance, index, serial, node))
        pairs.sort(key=lambda pair: pair[:3])
        matched = {}
        for _, index, _, node in pairs:
            if index not in matched and node.id not in fed:
                matched[index] = node
                fed.add(node.id)
        for index, entity in enumerate(fresh):
            node = matched.get(index)
            if node is None:
                name = Category(entity.category).name.lower()
                node = self._add_node(f'g/{name}/{self._next_serial}', entity.category)
                self._next_serial += 1
            self._report(node, robot, entity)

    def _add_node(self, node_id, category):
        node = self._nodes[node_id] = _Node(node_id, category)
        return node

    def _report(self, node, robot, entity):
        node.reports[robot] = _copy(entity)
        node.first_seen = min(node.first_seen, entity.first_seen)
        node.last_seen = max(node.last_seen, entity.last_seen)
        self._owner[(robot, entity.id)] = node.id

    def _drop(self, robot, local_id):
        node_id = self._owner.pop((robot, local_id), None)
        if node_id is None:
            return
        node = self._nodes[node_id]
        del node.reports[robot]
        if not node.reports:
            del self._nodes[node_id]

    # ---- publishing

    def _publish(self, source_robot_id, now):
        fused = {node_id: self._fuse(node, now) for node_id, node in self._nodes.items()}
        added = [entity for node_id, entity in fused.items() if node_id not in self._published]
        updated = [entity for node_id, entity in fused.items() if node_id in self._published
                   and self._changed(self._published[node_id], entity)]
        removed_ids = [node_id for node_id in self._published if node_id not in fused]
        relations, published = self._relation_set(), self._published_relations
        added_relations = [relations[key] for key in sorted(relations) if key not in published
                           or abs(relations[key].confidence - published[key].confidence)
                           > self._params.change_confidence_eps]
        removed_relations = [published[key] for key in sorted(published) if key not in relations]
        if not (added or updated or removed_ids or added_relations or removed_relations):
            return None
        for entity in added + updated:
            self._published[entity.id] = entity
        for node_id in removed_ids:
            del self._published[node_id]
        for relation in added_relations:
            published[_key(relation)] = relation
        for relation in removed_relations:
            del published[_key(relation)]
        self._version += 1
        return GraphUpdate(self._version, self._version - 1, False, source_robot_id,
                           [_copy(entity) for entity in added],
                           [_copy(entity) for entity in updated], removed_ids,
                           added_relations, removed_relations)

    def _relation_set(self):
        """Union of every robot's latest set, deduplicated at the highest
        confidence. Mapped through the current contributions, so a robot's
        relations end with the tracks they refer to."""
        merged = {}
        for robot, relations in self._robot_relations.items():
            for relation in relations:
                subject = self._resolve(robot, relation.subject_id)
                target = self._resolve(robot, relation.object_id)
                if subject is None or target is None:
                    continue
                key = (subject, relation.predicate, target)
                if key not in merged or relation.confidence > merged[key].confidence:
                    merged[key] = Relation(subject, relation.predicate, target, relation.confidence)
        return merged

    def _resolve(self, robot, local_id):
        node_id = self._owner.get((robot, local_id))
        node = self._nodes.get(local_id)
        if node_id is None and node is not None and node.category in REGION_CATEGORIES:
            return local_id         # regions are shared: no need to have reported them
        return node_id

    def _fuse(self, node, now):
        robots = sorted(node.reports)
        reports = [node.reports[robot] for robot in robots]
        weights = _weights(reports)
        lead = max(reports, key=lambda report: report.confidence)
        missed = 1.0
        for report in reports:
            missed *= 1.0 - report.confidence
        members = {self._owner[(robot, member_id)]
                   for robot, report in zip(robots, reports) for member_id in report.member_ids
                   if (robot, member_id) in self._owner}
        entity = Entity(
            node.id, node.category, lead.label,
            _mean(weights, [report.x for report in reports]),
            _mean(weights, [report.y for report in reports]),
            z=_mean(weights, [report.z for report in reports]),
            size=tuple(lead.size),
            vx=_mean(weights, [report.vx for report in reports]),
            vy=_mean(weights, [report.vy for report in reports]),
            confidence=1.0 - missed, first_seen=node.first_seen, last_seen=node.last_seen,
            member_ids=sorted(members), source_ids=robots, attributes=dict(lead.attributes))
        persistence = classify_persistence(entity, now, self._params)
        if persistence is not None:
            entity.attributes['persistence'] = persistence
        return entity

    def _position(self, node):
        reports = [node.reports[robot] for robot in sorted(node.reports)]
        weights = _weights(reports)
        return (_mean(weights, [report.x for report in reports]),
                _mean(weights, [report.y for report in reports]))

    def _changed(self, last, entity):
        params = self._params
        moved = max(math.hypot(entity.x - last.x, entity.y - last.y),
                    *(abs(new - old) for new, old in zip(entity.size, last.size)))
        return (moved > params.change_position_eps
                or abs(entity.confidence - last.confidence) > params.change_confidence_eps
                or (entity.label, entity.member_ids, entity.source_ids, entity.attributes)
                != (last.label, last.member_ids, last.source_ids, last.attributes))


class GraphReplica:
    """Robot-side copy of the global graph: apply every GraphUpdate in
    order, resync with a snapshot after a gap."""

    def __init__(self):
        self._version = 0
        self._entities: dict[str, Entity] = {}
        self._relations: dict[tuple, Relation] = {}

    @property
    def version(self) -> int:
        return self._version

    def entities(self) -> list[Entity]:
        return [_copy(entity) for entity in self._entities.values()]

    def relations(self) -> list[Relation]:
        return [self._relations[key] for key in sorted(self._relations)]

    def apply(self, update: GraphUpdate) -> str:
        """'snapshot' (state replaced), 'applied', 'duplicate' (already
        seen: ignored) or 'gap' (unchanged: resync with a snapshot)."""
        if update.is_snapshot:
            # even an older one: Module 1 may have restarted
            self._entities = {entity.id: _copy(entity) for entity in update.added}
            self._relations = {_key(relation): relation for relation in update.added_relations}
            self._version = update.graph_version
            return 'snapshot'
        if update.graph_version <= self._version:
            return 'duplicate'
        if update.base_version != self._version:
            return 'gap'
        for entity in update.added + update.updated:
            self._entities[entity.id] = _copy(entity)
        for entity_id in update.removed_ids:
            self._entities.pop(entity_id, None)
        for relation in update.removed_relations:
            self._relations.pop(_key(relation), None)
        for relation in update.added_relations:
            self._relations[_key(relation)] = relation
        self._version = update.graph_version
        return 'applied'


def _key(relation):
    return (relation.subject_id, relation.predicate, relation.object_id)


def _weights(reports):
    weights = [report.confidence for report in reports]
    return weights if sum(weights) > 0 else [1.0] * len(reports)


def _mean(weights, values):
    return sum(weight * value for weight, value in zip(weights, values)) / sum(weights)


def _validate(observation, now):
    if not math.isfinite(now):
        raise ValueError(f'now must be finite, got {now!r}')
    if not isinstance(observation.robot_id, str) or not observation.robot_id:
        raise ValueError(f'robot_id must be a non-empty string, got {observation.robot_id!r}')
    for entity in observation.entities:
        where = f'entity {entity.id!r}'
        if entity.category not in REGION_CATEGORIES + FUSED:
            raise ValueError(f'{where}: unknown category {entity.category!r}')
        if entity.category in REGION_CATEGORIES and '/' in entity.id:
            raise ValueError(f"{where}: region ids must not contain '/' (reserved for track ids)")
        if not all(math.isfinite(value) for value in
                   (entity.x, entity.y, entity.first_seen, entity.last_seen)):
            raise ValueError(f'{where}: position and times must be finite')
        if not 0.0 <= entity.confidence <= 1.0:
            raise ValueError(f'{where}: confidence must be in [0, 1], got {entity.confidence!r}')


def _category_name(value):
    try:
        return Category(value).name
    except ValueError:
        return repr(value)


def _copy(entity):
    return replace(entity, size=tuple(entity.size), member_ids=list(entity.member_ids),
                   source_ids=list(entity.source_ids), attributes=dict(entity.attributes))
