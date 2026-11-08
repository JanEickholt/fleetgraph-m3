"""Local scene graph of one robot. Pure Python (no rclpy): the ROS node
transforms detections into the map frame, calls integrate() once per frame
and publishes take_delta() as a SceneObservation
(tests/unit/test_scene_graph.py).

Nodes:      floor-plan regions (PLACE / TRAVERSABLE: static, never removed),
            tracked OBSTACLE / PERSON / OBJECT / ROBOT entities with stable
            ids, and GROUP entities derived from persons standing together.
Relations:  INSIDE, NEAR, BLOCKS, OCCUPIES, derived from the current state.

Per frame, detections associate greedily, nearest first, within a
per-category gate: semantic detections with tracks of their own category,
then with range-only (lidar) tracks, which they re-identify under a new id;
lidar clusters with the nearest track of any physical category, or they
vanish as the echo of a semantic detection. Unmatched tracks decay only
where the sensors looked. Tracks that converge within merge_radius merge
into the older id; a range-only track yields to one of another category.
Confidence: hit c += gain * score * (1 - c); miss in view c *= miss_decay.
Tracks below report_min_confidence are kept but not reported.
"""
import math
from collections.abc import Callable, Iterable
from dataclasses import dataclass, field, fields, replace
from enum import IntEnum

from fleetgraph_robot.regions import Region

LIDAR_SOURCE = 'lidar'


class Category(IntEnum):
    """Mirror of SceneEntity.CATEGORY_* (checked against the .msg in tests)."""
    PLACE = 0
    TRAVERSABLE = 1
    OBSTACLE = 2
    PERSON = 3
    GROUP = 4
    OBJECT = 5
    ROBOT = 6


class Predicate(IntEnum):
    """Mirror of SceneRelation.PREDICATE_*."""
    INSIDE = 0
    NEAR = 1
    BLOCKS = 2
    OCCUPIES = 3


# Categories a detection may carry: groups are derived, regions are static.
PHYSICAL = (Category.OBSTACLE, Category.PERSON, Category.OBJECT, Category.ROBOT)
_AGENTS = (Category.PERSON, Category.GROUP, Category.ROBOT)
_THINGS = (Category.OBSTACLE, Category.OBJECT)

Size = tuple[float, float, float]
Box = tuple[float, float, float, float]     # xmin, ymin, xmax, ymax


@dataclass
class Detection:
    """One observation in the MAP frame."""
    category: int
    x: float
    y: float
    z: float = 0.0
    size: Size = (0.0, 0.0, 0.0)
    confidence: float = 1.0
    label: str = ''
    stamp: float = 0.0
    source: str = ''        # LIDAR_SOURCE: range evidence, category unknown


@dataclass
class Entity:
    id: str
    category: int
    label: str
    x: float
    y: float
    z: float = 0.0
    size: Size = (0.0, 0.0, 0.0)
    vx: float = 0.0
    vy: float = 0.0
    confidence: float = 0.0
    first_seen: float = 0.0
    last_seen: float = 0.0
    member_ids: list[str] = field(default_factory=list)
    source_ids: list[str] = field(default_factory=list)    # robots feeding it; central graph only
    attributes: dict[str, str] = field(default_factory=dict)
    hits: int = 0


@dataclass(frozen=True)
class Relation:
    subject_id: str
    predicate: int
    object_id: str
    confidence: float


@dataclass
class Delta:
    """SceneObservation payload."""
    entities: list[Entity]      # reported ones added, re-observed or changed
    removed_ids: list[str]      # previously reported, deleted since; once each
    relations: list[Relation]   # the full current set


def _per_category(obstacle, person, thing, robot):
    return field(default_factory=lambda: {
        Category.OBSTACLE: obstacle, Category.PERSON: person,
        Category.OBJECT: thing, Category.ROBOT: robot})


@dataclass
class GraphParams:
    """Distances in m, times in sim s. Per-category dicts may be partial:
    missing categories keep their defaults."""
    gate_radius: dict[int, float] = _per_category(0.5, 0.6, 0.5, 0.6)
    ttl: dict[int, float] = _per_category(30.0, 5.0, 30.0, 5.0)
    footprint_radius: dict[int, float] = _per_category(0.2, 0.3, 0.2, 0.25)  # size unknown
    merge_radius: float = 0.3
    confidence_gain: float = 0.5
    miss_decay: float = 0.7
    remove_below: float = 0.1
    report_min_confidence: float = 0.5
    group_radius: float = 1.2
    near_distance: float = 0.5          # footprint surface to surface
    min_clearance: float = 0.6          # free width a robot needs to pass
    position_smoothing: float = 0.5     # weight kept by the previous estimate
    velocity_smoothing: float = 0.5
    change_position_eps: float = 0.05   # position and size
    change_confidence_eps: float = 0.05

    def __post_init__(self):
        for spec in fields(self):
            if spec.name in _PER_CATEGORY:
                merged = _merged(spec.name, spec.default_factory(), getattr(self, spec.name))
                setattr(self, spec.name, merged)
        checks = [
            ('confidence_gain', 0 < self.confidence_gain <= 1, 'in (0, 1]'),
            ('miss_decay', 0 <= self.miss_decay <= 1, 'in [0, 1]'),
            ('remove_below', 0 <= self.remove_below <= 1, 'in [0, 1]'),
            ('report_min_confidence', 0 <= self.report_min_confidence <= 1, 'in [0, 1]'),
            ('position_smoothing', 0 <= self.position_smoothing < 1, 'in [0, 1)'),
            ('velocity_smoothing', 0 <= self.velocity_smoothing < 1, 'in [0, 1)'),
        ]
        checks += [(name, getattr(self, name) >= 0, '>= 0') for name in _NON_NEGATIVE]
        for name, ok, expected in checks:
            if not ok:
                raise ValueError(f'{name} must be {expected}, got {getattr(self, name)!r}')


_PER_CATEGORY = ('gate_radius', 'ttl', 'footprint_radius')
_NON_NEGATIVE = ('merge_radius', 'group_radius', 'near_distance', 'min_clearance',
                 'change_position_eps', 'change_confidence_eps')


def _merged(name, defaults, overrides):
    for key, value in dict(overrides).items():
        if key not in PHYSICAL:
            raise ValueError(f'{name}: {_category_name(key)} is not a tracked category')
        if not value > 0:
            raise ValueError(f'{name}[{Category(key).name}] must be > 0, got {value!r}')
        defaults[Category(key)] = float(value)
    return defaults


class LocalSceneGraph:
    def __init__(self, robot_id: str, regions: list[Region],
                 params: GraphParams | None = None):
        if not isinstance(robot_id, str) or not robot_id:
            raise ValueError(f'robot_id must be a non-empty string, got {robot_id!r}')
        self._robot_id = robot_id
        self._params = params if params is not None else GraphParams()
        self._places = [region for region in regions if region.kind == 'place']
        self._traversables = [region for region in regions if region.kind == 'traversable']
        self._region_nodes: dict[str, Entity] = {}
        for region in regions:
            if region.id in self._region_nodes:
                raise ValueError(f'duplicate region id {region.id!r}')
            self._region_nodes[region.id] = _region_node(region)
        self._tracks: dict[str, Entity] = {}    # physical entities
        self._groups: dict[str, Entity] = {}
        self._serial: dict[str, int] = {}       # creation order of live ids
        self._next_serial = 1
        self._reported: dict[str, Entity] = {}  # last reported state of live entities
        self._removed: list[str] = []           # reported ids deleted since the last delta
        self._hit_ids: set[str] = set()         # re-observed since the last delta
        self._frame_hits: set[str] = set()      # re-observed in the current integrate()
        self._regions_reported = False

    # ---- queries

    def entities(self, include_tentative: bool = False) -> list[Entity]:
        tracks = self._tracks.values() if include_tentative else self._reportable_tracks()
        return [_snapshot(entity) for entity in
                (*self._region_nodes.values(), *tracks, *self._groups.values())]

    def get(self, entity_id: str) -> Entity | None:
        for table in (self._region_nodes, self._tracks, self._groups):
            if entity_id in table:
                return _snapshot(table[entity_id])
        return None

    def relations(self) -> list[Relation]:
        params = self._params
        present = self._reportable_tracks() + list(self._groups.values())
        relations = [Relation(entity.id, Predicate.INSIDE, place.id, entity.confidence)
                     for entity in present for place in self._places
                     if place.contains(entity.x, entity.y)]
        agents = [entity for entity in present if entity.category in _AGENTS]
        things = [entity for entity in present if entity.category in _THINGS]
        relations += [Relation(agent.id, Predicate.NEAR, thing.id,
                               min(agent.confidence, thing.confidence))
                      for agent in agents for thing in things
                      if self._clearance(agent, thing) <= params.near_distance]
        for passage in self._traversables:
            discs, boxes, inside = [], [], []
            for entity in present:
                if entity.category == Category.GROUP:
                    box = _box(entity)
                    if passage.intersects_box(*box):
                        boxes.append(box)
                        inside.append(entity)
                    continue
                disc = (entity.x, entity.y, self._footprint(entity))
                if passage.intersects_disc(*disc):
                    discs.append(disc)
                    inside.append(entity)
                elif (entity.category in _THINGS
                      and passage.distance_to_disc(*disc) <= params.near_distance):
                    relations.append(Relation(entity.id, Predicate.NEAR, passage.id,
                                              entity.confidence))
            if inside:
                blocked = passage.free_gap(discs, boxes) < params.min_clearance
                predicate = Predicate.BLOCKS if blocked else Predicate.OCCUPIES
                relations += [Relation(entity.id, predicate, passage.id, entity.confidence)
                              for entity in inside]
        return sorted(relations, key=lambda relation:
                      (relation.subject_id, relation.predicate, relation.object_id))

    def take_delta(self) -> Delta:
        entities = []
        if not self._regions_reported:
            entities = [_snapshot(node) for node in self._region_nodes.values()]
            self._regions_reported = True
        for entity in self._reportable_tracks() + list(self._groups.values()):
            last = self._reported.get(entity.id)
            if last is None or entity.id in self._hit_ids or self._changed(last, entity):
                entities.append(_snapshot(entity))
                self._reported[entity.id] = _snapshot(entity)
        removed, self._removed = self._removed, []
        self._hit_ids = set()
        return Delta(entities, removed, self.relations())

    # ---- update

    def integrate(self, detections: Iterable[Detection], now: float,
                  in_view: Callable[[float, float], bool] | None = None) -> None:
        """in_view(x, y): whether this frame's sensors covered that point, so
        an unmatched track there was missed. None: no track decays."""
        if not math.isfinite(now):
            raise ValueError(f'now must be finite, got {now!r}')
        detections = list(detections)
        for detection in detections:
            _validate(detection)
        params = self._params
        gate = params.gate_radius
        frame = self._collapse(detections)
        semantic = [i for i, detection in enumerate(frame) if detection.source != LIDAR_SOURCE]
        ranged = [i for i, detection in enumerate(frame) if detection.source == LIDAR_SOURCE]
        self._frame_hits = set()
        taken: set[str] = set()
        same = _greedy(self._pairs(frame, semantic, lambda detection, entity: (
            gate[entity.category] if entity.category == detection.category else None)), taken)
        unnamed = [i for i in semantic
                   if i not in same and frame[i].category != Category.OBSTACLE]
        upgrades = _greedy(self._pairs(frame, unnamed, lambda detection, entity: (
            gate[detection.category] if _lidar_born(entity) else None)), taken)
        for i in semantic:
            if i in same:
                self._hit(same[i], frame[i], now)
            elif i in upgrades:
                self._upgrade(upgrades[i], frame[i], now)
            else:
                self._create(frame[i], now)

        observed = set(self._frame_hits)

        def reach(detection, entity):
            if entity.id in observed:       # same-frame echo: footprints overlap
                return self._radius(detection.category, detection.size) + self._footprint(entity)
            return gate[entity.category]

        echoes, reinforced = set(), {}
        for _, i, _, entity in self._pairs(frame, ranged, reach):
            if i in echoes or i in reinforced:
                continue
            if entity.id in observed:
                echoes.add(i)
            elif entity.id not in taken:
                reinforced[i] = entity
                taken.add(entity.id)
        for i in ranged:
            if i in reinforced:
                self._hit(reinforced[i], frame[i], now)
            elif i not in echoes:
                self._create(frame[i], now)

        if in_view is not None:
            for entity in self._tracks.values():
                if entity.id not in self._frame_hits and in_view(entity.x, entity.y):
                    entity.confidence *= params.miss_decay
        for entity in list(self._tracks.values()):
            if (entity.confidence < params.remove_below
                    or now - entity.last_seen > params.ttl[entity.category]):
                self._delete(entity.id)
        self._merge_converged()
        self._update_groups(now)
        self._hit_ids |= self._frame_hits

    def _collapse(self, detections):
        """Same-frame duplicates: a detection closer than merge_radius to a
        stronger one of its category is the same thing seen twice. Lidar
        clusters yield to any detection, semantic ones never to lidar."""
        radius = self._params.merge_radius

        def strength(i):
            detection = detections[i]
            return (detection.source == LIDAR_SOURCE, -detection.confidence, i)

        kept = []
        for i in sorted(range(len(detections)), key=strength):
            candidate = detections[i]
            if not any(math.hypot(candidate.x - other.x, candidate.y - other.y) < radius
                       and (other.category == candidate.category
                            or candidate.source == LIDAR_SOURCE)
                       for other in (detections[j] for j in kept)):
                kept.append(i)
        return [detections[i] for i in sorted(kept)]

    def _pairs(self, frame, indices, radius_of):
        """(distance, frame index, serial, track) within radius_of(detection,
        track), which returns None for tracks that are no candidate; sorted
        nearest first with deterministic ties."""
        pairs = []
        for i in indices:
            detection = frame[i]
            for entity in self._tracks.values():
                radius = radius_of(detection, entity)
                if radius is None:
                    continue
                distance = math.hypot(detection.x - entity.x, detection.y - entity.y)
                if distance <= radius:
                    pairs.append((distance, i, self._serial[entity.id], entity))
        pairs.sort(key=lambda pair: pair[:3])
        return pairs

    def _hit(self, entity, detection, now):
        params = self._params
        keep = params.position_smoothing
        x = keep * entity.x + (1 - keep) * detection.x
        y = keep * entity.y + (1 - keep) * detection.y
        dt = now - entity.last_seen
        if dt > 0:
            vx, vy = (x - entity.x) / dt, (y - entity.y) / dt
            if entity.hits > 1:
                damp = params.velocity_smoothing
                vx = damp * entity.vx + (1 - damp) * vx
                vy = damp * entity.vy + (1 - damp) * vy
            entity.vx, entity.vy = vx, vy
        entity.x, entity.y = x, y
        lidar = detection.source == LIDAR_SOURCE
        # Lidar shapes only range-only tracks. The first semantic look at one
        # replaces its visible-surface size and placeholder label.
        if not lidar or _lidar_born(entity):
            refine = not lidar and _lidar_born(entity)
            if refine or not any(entity.size):
                entity.z, entity.size = detection.z, _floats(detection.size)
            elif any(detection.size):
                entity.z = keep * entity.z + (1 - keep) * detection.z
                entity.size = tuple(keep * old + (1 - keep) * new
                                    for old, new in zip(entity.size, detection.size))
            if refine or detection.label:
                entity.label = detection.label
            if not lidar:
                if detection.source:
                    entity.attributes['source'] = detection.source
                else:
                    entity.attributes.pop('source', None)
        entity.confidence += params.confidence_gain * detection.confidence * (1 - entity.confidence)
        entity.last_seen = now
        entity.hits += 1
        self._frame_hits.add(entity.id)

    def _upgrade(self, track, detection, now):
        """A semantic detection names a range-only obstacle: the entity lives
        on under a new id, since ids encode the category."""
        self._delete(track.id)
        category = Category(detection.category)
        entity = replace(track, id=self._new_id(category), category=category,
                         attributes=dict(track.attributes))
        self._tracks[entity.id] = entity
        self._hit(entity, detection, now)

    def _create(self, detection, now):
        category = Category(detection.category)
        entity = Entity(self._new_id(category), category, detection.label,
                        detection.x, detection.y, z=detection.z, size=_floats(detection.size),
                        confidence=self._params.confidence_gain * detection.confidence,
                        first_seen=now, last_seen=now, hits=1,
                        attributes={'source': detection.source} if detection.source else {})
        self._tracks[entity.id] = entity
        self._frame_hits.add(entity.id)

    def _new_id(self, category):
        entity_id = f'{self._robot_id}/{category.name.lower()}/{self._next_serial}'
        self._serial[entity_id] = self._next_serial
        self._next_serial += 1
        return entity_id

    def _delete(self, entity_id):
        self._tracks.pop(entity_id, None)
        self._groups.pop(entity_id, None)
        del self._serial[entity_id]
        if self._reported.pop(entity_id, None) is not None:
            self._removed.append(entity_id)

    def _merge_converged(self):
        """Two tracks of one thing that drifted together, or a range-only
        track and the classified one it really was, become one."""
        radius = self._params.merge_radius
        while True:
            tracks = list(self._tracks.values())
            closest = None
            for index, a in enumerate(tracks):
                for b in tracks[index + 1:]:
                    distance = math.hypot(a.x - b.x, a.y - b.y)
                    if (distance < radius and (closest is None or distance < closest[0])
                            and (a.category == b.category or _lidar_born(a) or _lidar_born(b))):
                        closest = (distance, a, b)
            if closest is None:
                return
            self._absorb(*self._survivor_first(closest[1], closest[2]))

    def _survivor_first(self, a, b):
        """The older track survives, except that a range-only obstacle yields
        to a track of another category (ids encode the category)."""
        if a.category != b.category:
            return (b, a) if _lidar_born(a) else (a, b)
        if (a.first_seen, self._serial[a.id]) <= (b.first_seen, self._serial[b.id]):
            return a, b
        return b, a

    def _absorb(self, survivor, absorbed):
        if _lidar_born(survivor) and not _lidar_born(absorbed):
            # same category: the classified description refines it, as in _hit
            survivor.label, survivor.z, survivor.size = absorbed.label, absorbed.z, absorbed.size
            survivor.attributes = dict(absorbed.attributes)
        if absorbed.last_seen > survivor.last_seen:
            survivor.x, survivor.y = absorbed.x, absorbed.y
            survivor.vx, survivor.vy = absorbed.vx, absorbed.vy
            survivor.last_seen = absorbed.last_seen
        survivor.first_seen = min(survivor.first_seen, absorbed.first_seen)
        survivor.confidence = max(survivor.confidence, absorbed.confidence)
        survivor.hits += absorbed.hits
        self._frame_hits.add(survivor.id)
        self._delete(absorbed.id)

    def _update_groups(self, now):
        """Reported persons chained within group_radius form a group. A group
        keeps its id while at least half of its members carry over."""
        persons = [entity for entity in self._reportable_tracks()
                   if entity.category == Category.PERSON]
        components = [members for members in _connected(persons, self._params.group_radius)
                      if len(members) >= 2]
        pairs = []
        for group in self._groups.values():
            old = set(group.member_ids)
            for index, members in enumerate(components):
                new = {member.id for member in members}
                overlap = len(old & new) / max(len(old), len(new))
                if overlap >= 0.5:
                    pairs.append((-overlap, index, self._serial[group.id], group))
        continued = _greedy(pairs, set())
        kept_ids = {group.id for group in continued.values()}
        for group_id in [group_id for group_id in self._groups if group_id not in kept_ids]:
            self._delete(group_id)
        for index, members in enumerate(components):
            group = continued.get(index)
            if group is None:
                group = Entity(self._new_id(Category.GROUP), Category.GROUP, 'group', 0.0, 0.0,
                               first_seen=now)
                self._groups[group.id] = group
            self._fill_group(group, members)

    def _fill_group(self, group, members):
        count = len(members)
        group.x = sum(member.x for member in members) / count
        group.y = sum(member.y for member in members) / count
        group.z = sum(member.z for member in members) / count
        # Centred on the centroid, so the box is symmetric and spans every
        # member footprint; nothing passes between members.
        half_x = max(abs(member.x - group.x) + self._footprint(member) for member in members)
        half_y = max(abs(member.y - group.y) + self._footprint(member) for member in members)
        group.size = (2 * half_x, 2 * half_y, max(member.size[2] for member in members))
        group.vx = sum(member.vx for member in members) / count
        group.vy = sum(member.vy for member in members) / count
        group.confidence = min(member.confidence for member in members)
        group.last_seen = max(member.last_seen for member in members)
        group.member_ids = sorted(member.id for member in members)
        if any(member.id in self._frame_hits for member in members):
            group.hits += 1
            self._frame_hits.add(group.id)

    # ---- geometry and bookkeeping

    def _reportable_tracks(self):
        threshold = self._params.report_min_confidence
        return [entity for entity in self._tracks.values() if entity.confidence >= threshold]

    def _radius(self, category, size):
        radius = max(size[0], size[1]) / 2
        return radius if radius > 0 else self._params.footprint_radius[category]

    def _footprint(self, entity):
        return self._radius(entity.category, entity.size)

    def _clearance(self, agent, thing):
        """Gap between the footprints of an agent (a group: its box) and a thing."""
        if agent.category == Category.GROUP:
            xmin, ymin, xmax, ymax = _box(agent)
            reach = math.hypot(max(xmin - thing.x, 0.0, thing.x - xmax),
                               max(ymin - thing.y, 0.0, thing.y - ymax))
        else:
            reach = math.hypot(agent.x - thing.x, agent.y - thing.y) - self._footprint(agent)
        return reach - self._footprint(thing)

    def _changed(self, last, entity):
        params = self._params
        moved = max(math.hypot(entity.x - last.x, entity.y - last.y),
                    *(abs(new - old) for new, old in zip(entity.size, last.size)))
        return (moved > params.change_position_eps
                or abs(entity.confidence - last.confidence) > params.change_confidence_eps
                or (entity.category, entity.label, entity.member_ids, entity.attributes)
                != (last.category, last.label, last.member_ids, last.attributes))


def _greedy(pairs, taken):
    """Nearest-first one-to-one assignment over (key, index, tie, entity)
    pairs: {index: entity}. Entities in `taken` are skipped; winners join it."""
    matches = {}
    for _, index, _, entity in sorted(pairs, key=lambda pair: pair[:3]):
        if index not in matches and entity.id not in taken:
            matches[index] = entity
            taken.add(entity.id)
    return matches


def _connected(entities, radius):
    """Single-linkage components: entities chained by distance <= radius."""
    parent = list(range(len(entities)))

    def root(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    for i, a in enumerate(entities):
        for j in range(i + 1, len(entities)):
            b = entities[j]
            if math.hypot(a.x - b.x, a.y - b.y) <= radius:
                parent[root(i)] = root(j)
    components: dict[int, list[Entity]] = {}
    for i, entity in enumerate(entities):
        components.setdefault(root(i), []).append(entity)
    return list(components.values())


def _lidar_born(entity):
    """Seen only by lidar so far: reported as OBSTACLE, category unknown."""
    return entity.attributes.get('source') == LIDAR_SOURCE


def _validate(detection):
    if detection.category not in PHYSICAL:
        names = ', '.join(category.name for category in PHYSICAL)
        raise ValueError(f'cannot track {_category_name(detection.category)} detections, '
                         f'only {names}')
    if detection.source == LIDAR_SOURCE and detection.category != Category.OBSTACLE:
        raise ValueError(f'lidar detections must be OBSTACLE, got '
                         f'{_category_name(detection.category)}')
    position = (detection.x, detection.y, detection.z)
    if not all(math.isfinite(value) for value in position):
        raise ValueError(f'detection position must be finite, got {position}')
    if len(detection.size) != 3 or not all(math.isfinite(value) and value >= 0
                                           for value in detection.size):
        raise ValueError(f'detection size must be 3 finite values >= 0, got {detection.size}')
    if not 0.0 <= detection.confidence <= 1.0:
        raise ValueError(f'detection confidence must be in [0, 1], got {detection.confidence!r}')


def _category_name(value):
    try:
        return Category(value).name
    except ValueError:
        return repr(value)


def _floats(values):
    return tuple(float(value) for value in values)


def _box(entity) -> Box:
    half_x, half_y = entity.size[0] / 2, entity.size[1] / 2
    return (entity.x - half_x, entity.y - half_y, entity.x + half_x, entity.y + half_y)


def _snapshot(entity):
    return replace(entity, member_ids=list(entity.member_ids), attributes=dict(entity.attributes))


def _region_node(region):
    (x, y), (size_x, size_y) = region.center(), region.size()
    if region.kind == 'traversable':
        return Entity(region.id, Category.TRAVERSABLE, region.label, x, y,
                      size=(size_x, size_y, 0.0), confidence=1.0,
                      attributes={'travel_axis': region.travel_axis})
    return Entity(region.id, Category.PLACE, region.label, x, y,
                  size=(size_x, size_y, 0.0), confidence=1.0)
