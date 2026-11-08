"""Assertion registry. Pure logic — no rclpy imports, unit-testable on
canned recordings (see unit/test_assertions.py).

Contract: fn(view, **params) -> None when PASS, else a one-line failure
string with observed values. view.stream(topic) -> list[(t, msg)];
view.duration() -> window length in the active timebase.

Adding an assertion type = one function + one REGISTRY entry. Scenario
YAML then uses it by name. Do not add types without a unit test that
proves both PASS and FAIL paths.
"""
import math

# Harness-owned fudge factors live HERE, not in scenario files. Rate
# estimators undershoot slightly at window edges and from jitter, so a
# publisher running at its nominal rate must pass. Scenarios state the
# NOMINAL spec (hz: 1.0 means "this topic should run at 1 Hz"); the
# harness derives the hard threshold. Override per-assertion only with
# a written reason.
DEFAULT_RATE_MARGIN = 0.2

REGISTRY = {}


def register(function):
    REGISTRY[function.__name__] = function
    return function


def _get_field(obj, dotted_path):
    """Follow a dotted field path on messages (getattr) or dicts ([])."""
    value = obj
    for part in dotted_path.split('.'):
        value = value[part] if isinstance(value, dict) else getattr(value, part)
    return value


def _distance_xy(position, x, y):
    return math.hypot(float(_get_field(position, 'x')) - float(x),
                      float(_get_field(position, 'y')) - float(y))


def _yaw_deg(orientation):
    x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
    return math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


@register
def publishes(view, topic, **_):
    if not view.stream(topic):
        return f"no message ever recorded on {topic}"
    return None


def _from_source(stream, source):
    """Contract messages whose Meta.source_id is `source` (all if None):
    one robot's share of a fleet-wide topic such as /fleet/robot_state."""
    if source is None:
        return stream
    return [(timestamp, msg) for timestamp, msg in stream if msg.meta.source_id == source]


def _where(topic, source):
    return topic if source is None else f"{topic} from {source}"


@register
def min_rate(view, topic, hz, margin=None, source=None, **_):
    stream = _from_source(view.stream(topic), source)
    where = _where(topic, source)
    if not stream:
        return f"no messages on {where}"
    if len(stream) < 2:
        return (f"only 1 message on {where}, "
                f"cannot estimate rate")
    span = stream[-1][0] - stream[0][0]
    if span <= 0:
        return f"messages on {where} span {span}s, cannot estimate rate"
    actual = (len(stream) - 1) / span
    margin_fraction = DEFAULT_RATE_MARGIN if margin is None else float(margin)
    required = float(hz) * (1.0 - margin_fraction)
    if actual < required:
        return (f"rate {actual:.2f} Hz < required {required:.2f} Hz "
                f"(nominal {hz} - {margin_fraction:.0%} margin) on {where}")
    return None


@register
def arrives_at(view, topic, x, y, tol, source=None, **_):
    stream = _from_source(view.stream(topic), source)
    where = _where(topic, source)
    for _, msg in stream:
        if _distance_xy(_get_field(msg, 'pose.pose.position'), x, y) <= float(tol):
            return None
    if not stream:
        return f"no messages on {where}"
    last = _get_field(stream[-1][1], 'pose.pose.position')
    return (f"never came within {tol} of ({x}, {y}) on {where}; "
            f"last pose ({float(_get_field(last, 'x')):.2f}, "
            f"{float(_get_field(last, 'y')):.2f}), "
            f"distance {_distance_xy(last, x, y):.2f}")


@register
def never_below(view, topic, field, threshold, **_):
    # NaN compares False against everything, so `min()` over a stream
    # containing NaN silently discards it. Check EVERY element; NaN
    # anywhere = violation (safety assertions fail closed on unknown).
    for _, msg in view.stream(topic):
        raw = _get_field(msg, field)
        values = list(raw) if isinstance(raw, (list, tuple)) else [raw]
        for raw_value in values:
            try:
                value = float(raw_value)
            except (TypeError, ValueError):
                continue
            if math.isnan(value):
                return f"{topic}.{field} produced NaN — treated as violation"
            if value < float(threshold):
                return f"{topic}.{field} hit {value:.3f} < {threshold}"
    if not view.stream(topic):
        return f"no messages on {topic}"
    return None


@register
def final_value(view, topic, field, equals, tol=0.01, **_):
    stream = view.stream(topic)
    if not stream:
        return f"no messages on {topic}"
    value = _get_field(stream[-1][1], field)
    if isinstance(value, (list, tuple)):
        return f"final_value on array field '{field}' is not supported"
    if abs(float(value) - float(equals)) > float(tol):
        return f"final {topic}.{field} = {value}, expected {equals} ±{tol}"
    return None


@register
def yaw_final(view, topic, deg, tol, **_):
    """Final /odom yaw (quaternion -> yaw, degrees) equals `deg` within tol.

    Wraps angular difference to [-180, 180], so 350 vs -10 is a 0-deg error.
    Evidence basis: probe measurements 2026-10-26 (experiments/report.md) —
    closed-loop 90-deg turns land within ~1.2 deg (odom frame) at wz_max 0.8.
    """
    stream = view.stream(topic)
    if not stream:
        return f"no messages on {topic}"
    yaw = _yaw_deg(_get_field(stream[-1][1], 'pose.pose.orientation'))
    error_deg = (yaw - float(deg) + 180.0) % 360.0 - 180.0
    if abs(error_deg) <= float(tol):
        return None
    return (f"final yaw {yaw:.2f} deg vs target {deg}, "
            f"|err| {abs(error_deg):.2f} deg > {tol}")


# ---------------------------------------------------------------- tasks
# Mirror of fleetgraph_interfaces/msg/TaskUpdate STATE_* (kept pure: no
# rclpy import here). unit/test_contract_mirror.py checks it against the
# generated message when the workspace is built.
TASK_STATES = {'RECEIVED': 0, 'PARSED': 1, 'QUEUED': 2, 'ASSIGNED': 3,
               'EXECUTING': 4, 'COMPLETED': 5, 'FAILED': 6, 'CANCELLED': 7,
               'REASSIGNED': 8, 'REJECTED': 9}
_STATE_NAMES = {value: name for name, value in TASK_STATES.items()}


def _meta_stamp(event):
    stamp = event[1].meta.stamp
    return (stamp.sec, stamp.nanosec)


def _task_events(view, topic, task_id, source=None):
    """The task's TaskUpdates (from source) sorted by Meta.stamp; the sort is
    stable, so equal stamps keep their arrival order. Arrival order is not
    publication order: under DDS discovery delay the mock's RECEIVED ..
    ASSIGNED can reach the recorder after the agent's REJECTED. Stamps of
    different nodes compare only on one clock (all on sim time or none)."""
    events = [(timestamp, msg) for timestamp, msg in _from_source(view.stream(topic), source)
              if msg.task_id == task_id]
    return sorted(events, key=_meta_stamp)


def _history(events):
    """State names in event order, consecutive repeats collapsed."""
    names = []
    for _, msg in events:
        name = _STATE_NAMES.get(msg.state, str(msg.state))
        if not names or names[-1] != name:
            names.append(name)
    return names


@register
def task_state(view, task_id, state, topic='/fleet/task_update',
               error_code=None, source=None, within_sec=None, **_):
    """Some TaskUpdate for task_id has `state` (+ error_code / source, and
    sim time <= within_sec if given: the earliest such update's arrival)."""
    wanted_state = TASK_STATES[state]
    events = _task_events(view, topic, task_id, source)
    first = min((timestamp for timestamp, msg in events if msg.state == wanted_state
                 and (error_code is None or msg.error_code == int(error_code))), default=None)
    if first is not None:
        if within_sec is not None and first > float(within_sec):
            return f"task {task_id} reached {state} only at t={first:.1f}s > {within_sec}s"
        return None
    codes = sorted({msg.error_code for _, msg in events})
    return (f"task {task_id} never reached {state}"
            f"{'' if error_code is None else f' with code {error_code}'}; "
            f"history {_history(events)}, codes {codes}")


@register
def seq_contiguous(view, topic, **_):
    """Contract Meta.seq: per source_id, recorded seq values are +1 steps
    (no gap = no lost event, no repeat = no duplicate). The first recorded
    seq per source may be > 1 (recorder joined late)."""
    stream = view.stream(topic)
    if not stream:
        return f"no messages on {topic}"
    last_seq = {}                  # source_id -> last recorded seq
    for timestamp, msg in stream:
        source, seq = msg.meta.source_id, int(msg.meta.seq)
        if source in last_seq and seq != last_seq[source] + 1:
            kind = 'duplicate/reorder' if seq <= last_seq[source] else 'gap (lost events)'
            return (f"{topic} source {source}: seq {last_seq[source]} -> {seq} at "
                    f"t={timestamp:.1f}s, {kind}")
        last_seq[source] = seq
    return None


@register
def task_sequence(view, task_id, states, topic='/fleet/task_update', **_):
    """The task's state history contains `states` in this order
    (other states may occur in between)."""
    history = _history(_task_events(view, topic, task_id))
    matched = 0
    for name in history:
        if matched < len(states) and name == states[matched]:
            matched += 1
    if matched == len(states):
        return None
    return (f"task {task_id}: expected order {states}, history {history} "
            f"(missing from {states[matched]})")


# ---------------------------------------------------------- scene graph
# Mirror of fleetgraph_interfaces/msg/SceneEntity CATEGORY_*
# (unit/test_central_assertions.py checks it against the generated message).
SCENE_CATEGORIES = {'PLACE': 0, 'TRAVERSABLE': 1, 'OBSTACLE': 2, 'PERSON': 3,
                    'GROUP': 4, 'OBJECT': 5, 'ROBOT': 6}
_CATEGORY_NAMES = {value: name for name, value in SCENE_CATEGORIES.items()}


def _graph_states(stream):
    """[(t, {id: SceneEntity})] after each recorded SceneGraphUpdate: a
    snapshot replaces the graph, a delta changes it. Versions are not
    checked here (graph_versions_consistent does)."""
    states, nodes = [], {}
    for timestamp, msg in stream:
        nodes = {} if msg.is_snapshot else dict(nodes)
        nodes.update((entity.id, entity) for entity in [*msg.added_nodes, *msg.updated_nodes])
        for entity_id in msg.removed_node_ids:
            nodes.pop(entity_id, None)
        states.append((timestamp, nodes))
    return states


def _in_force_from(states, t):
    """The states in force at or after window time t; the one already in
    force at t is re-stamped t."""
    earlier = [(t, nodes) for timestamp, nodes in states if timestamp <= t][-1:]
    return earlier + [(timestamp, nodes) for timestamp, nodes in states if timestamp > t]


def _attributes(entity):
    return {pair.key: pair.value for pair in entity.attributes}


def _matches(entity, category, x, y, tol, sources, attribute):
    attributes = _attributes(entity)
    return (entity.category == SCENE_CATEGORIES[category]
            and _distance_xy(entity.position, x, y) <= float(tol)
            and set(sources or ()) <= set(entity.source_ids)
            and all(attributes.get(str(key)) == str(value)
                    for key, value in (attribute or {}).items()))


def _describe(entity):
    position = entity.position
    return (f"{entity.id} at ({float(position.x):.2f}, {float(position.y):.2f}) "
            f"{_CATEGORY_NAMES.get(entity.category, entity.category)} "
            f"sources {list(entity.source_ids)} attributes {_attributes(entity)}")


def _wanted(category, x, y, tol, sources, attribute):
    wanted = f"{category} within {tol} of ({x}, {y})"
    if sources:
        wanted += f" with sources {list(sources)}"
    if attribute:
        wanted += f" with attributes {dict(attribute)}"
    return wanted


@register
def graph_has(view, topic, category, x, y, tol, sources=None, attribute=None,
              within_sec=None, after_sec=None, **_):
    """Some graph state reconstructed from `topic` (SceneGraphUpdate deltas
    and snapshots, e.g. a replica's snapshot topic) holds an entity of
    `category` within tol of (x, y), fed by every robot in `sources` and
    carrying every `attribute` key/value. Only states in force at or after
    after_sec count; the first match must come by window time within_sec."""
    stream = view.stream(topic)
    if not stream:
        return f"no messages on {topic}"
    states = _graph_states(stream)
    if after_sec is not None:
        states = _in_force_from(states, float(after_sec))
    for timestamp, nodes in states:
        if any(_matches(entity, category, x, y, tol, sources, attribute)
               for entity in nodes.values()):
            if within_sec is not None and timestamp > float(within_sec):
                return (f"{_wanted(category, x, y, tol, sources, attribute)} on {topic} "
                        f"only at t={timestamp:.1f}s > {within_sec}s")
            return None
    candidates = [(_distance_xy(entity.position, x, y), timestamp, entity)
                  for timestamp, nodes in states for entity in nodes.values()
                  if entity.category == SCENE_CATEGORIES[category]]
    where = topic if after_sec is None else f"{topic} at or after t={float(after_sec)}s"
    if not candidates:
        return f"no {category} ever in the graph on {where}"
    distance, timestamp, entity = min(candidates, key=lambda candidate: candidate[:2])
    return (f"no {_wanted(category, x, y, tol, sources, attribute)} on {where}; "
            f"nearest: {_describe(entity)}, distance {distance:.2f}, t={timestamp:.1f}s")


@register
def graph_lacks_after(view, topic, category, x, y, tol, t, sources=None, attribute=None, **_):
    """No graph state reconstructed from `topic` that is in force at or
    after window time t holds an entity matching as in graph_has. Fails on
    an empty stream: a silent topic proves no absence."""
    stream = view.stream(topic)
    if not stream:
        return f"no messages on {topic}"
    for timestamp, nodes in _in_force_from(_graph_states(stream), float(t)):
        for entity in nodes.values():
            if _matches(entity, category, x, y, tol, sources, attribute):
                return (f"{_describe(entity)} still in the graph at t={timestamp:.1f}s "
                        f">= {t}s on {topic}")
    return None


@register
def graph_versions_consistent(view, topic, **_):
    """Every recorded delta applies on top of the message before it:
    base_version = previous graph_version and graph_version = base_version
    + 1. A snapshot restarts the chain; the first delta may start anywhere
    (recorder joined late)."""
    stream = view.stream(topic)
    if not stream:
        return f"no messages on {topic}"
    previous = None
    for timestamp, msg in stream:
        if not msg.is_snapshot:
            step = f"delta {msg.base_version}->{msg.graph_version} at t={timestamp:.1f}s"
            if previous is not None and msg.base_version != previous:
                return f"{topic}: {step} does not follow version {previous}"
            if msg.graph_version != msg.base_version + 1:
                return f"{topic}: {step} is not +1"
        previous = msg.graph_version
    return None


@register
def graph_matches(view, topic, reference, tol=0.01, **_):
    """The final graph reconstructed from `topic` equals the final one from
    `reference`: same graph_version and the same entities (id, category,
    sources, attributes, position within tol)."""
    finals = {}
    for name in (topic, reference):
        stream = view.stream(name)
        if not stream:
            return f"no messages on {name}"
        finals[name] = (stream[-1][1].graph_version, _graph_states(stream)[-1][1])
    (version, nodes), (expected_version, expected) = finals[topic], finals[reference]
    if version != expected_version:
        return f"{topic} final version {version} != reference version {expected_version}"
    differences = [f"missing {entity_id}" for entity_id in sorted(set(expected) - set(nodes))]
    differences += [f"unexpected {entity_id}" for entity_id in sorted(set(nodes) - set(expected))]
    if differences:
        return f"{topic} at version {version}: {', '.join(differences)} (reference {reference})"
    for entity_id, entity in nodes.items():
        other = expected[entity_id]
        same = ((entity.category, list(entity.source_ids), _attributes(entity))
                == (other.category, list(other.source_ids), _attributes(other)))
        if not same or _distance_xy(entity.position, other.position.x,
                                    other.position.y) > float(tol):
            return (f"{topic} at version {version}: {entity_id} differs: {_describe(entity)} "
                    f"vs reference {_describe(other)}")


# ------------------------------------------------- Module 1 / Module 2 link
@register
def no_task_update(view, task_id=None, robot_id=None, source=None, state=None,
                   topic='/fleet/task_update', **_):
    """No TaskUpdate matches all given filters: task_id, msg.robot_id,
    Meta.source_id, state. `robot_id: robot_3` = never assigned to robot_3 and
    nothing reported by it. An empty recording fails: absence there proves
    nothing."""
    stream = view.stream(topic)
    if not stream:
        return f"no messages on {topic}"
    matches = [(timestamp, msg) for timestamp, msg in _from_source(stream, source)
               if (task_id is None or msg.task_id == task_id) and
               (robot_id is None or msg.robot_id == robot_id) and
               (state is None or msg.state == TASK_STATES[state])]
    if not matches:
        return None
    shown = ', '.join(f"{msg.task_id} {_STATE_NAMES.get(msg.state, msg.state)} "
                      f"t={timestamp:.1f}s" for timestamp, msg in matches[:5])
    return f"{len(matches)} matching TaskUpdates on {topic}: {shown}"


@register
def always_value(view, topic, field, equals, tol=0.01, source=None, **_):
    """Every message (from `source`) has numeric `field` == equals ± tol;
    NaN fails."""
    stream = _from_source(view.stream(topic), source)
    where = _where(topic, source)
    if not stream:
        return f"no messages on {where}"
    for timestamp, msg in stream:
        value = _get_field(msg, field)
        if not abs(float(value) - float(equals)) <= float(tol):
            return (f"{where}: {field} = {value} at t={timestamp:.1f}s, "
                    f"expected always {equals} ±{tol}")
    return None


# ------------------------------------------------------------ sim tools
# Windows are inclusive [from_sec, to_sec] in the recording's timebase.

def _in_window(stream, from_sec, to_sec):
    return [(timestamp, msg) for timestamp, msg in stream
            if float(from_sec) <= timestamp <= float(to_sec)]


def _window_text(topic, from_sec, to_sec):
    return f"{topic} in [{float(from_sec):g}, {float(to_sec):g}]"


def _sector_ranges(scan, bearing_deg, half_width_deg):
    """Ranges of the beams within half_width_deg of bearing_deg (wrapped)."""
    sector = []
    for index, distance in enumerate(scan.ranges):
        angle = math.degrees(scan.angle_min + index * scan.angle_increment)
        if abs((angle - bearing_deg + 180.0) % 360.0 - 180.0) <= half_width_deg:
            sector.append(distance)
    return sector


@register
def scan_range(view, topic, bearing_deg, min_m=0.0, max_m=math.inf, half_width_deg=2.0,
               from_sec=0.0, to_sec=math.inf, **_):
    """Every LaserScan in the window: the closest return within
    half_width_deg of bearing_deg (scan frame) lies in [min_m, max_m].
    REP 117 readings: -inf (too close) counts as 0, +inf as no return
    (passes an open max_m only); NaN fails closed."""
    window = _in_window(view.stream(topic), from_sec, to_sec)
    if not window:
        return f"no messages on {_window_text(topic, from_sec, to_sec)}"
    sector = f"within +-{float(half_width_deg):g} deg of {float(bearing_deg):g} deg"
    for timestamp, msg in window:
        ranges = _sector_ranges(msg, float(bearing_deg), float(half_width_deg))
        if not ranges:
            return f"{topic} at t={timestamp:.1f}: no beam {sector}"
        if any(math.isnan(distance) for distance in ranges):
            return f"{topic} at t={timestamp:.1f}: NaN {sector}, treated as violation"
        closest = min(0.0 if distance == -math.inf else distance for distance in ranges)
        if not float(min_m) <= closest <= float(max_m):
            seen = 'no return' if closest == math.inf else f"closest return {closest:.3f} m"
            return (f"{topic} at t={timestamp:.1f}: {seen} {sector}, "
                    f"outside [{float(min_m):g}, {float(max_m):g}] m")
    return None


def _entity_distances(msg, x, y, entity_id):
    """Distances from (x, y) to the entities of a SceneObservation-like
    message (in its own frame), only those with entity_id if given."""
    return [_distance_xy(entity.position, x, y) for entity in msg.entities
            if entity_id is None or entity.id == entity_id]


def _entity_text(entity_id, x, y, tol):
    name = 'entity' if entity_id is None else f"entity '{entity_id}'"
    return f"{name} within {tol} of ({x}, {y})"


@register
def entity_near(view, topic, x, y, tol, min_fraction=1.0, entity_id=None,
                from_sec=0.0, to_sec=math.inf, **_):
    """At least min_fraction of the messages in the window carry an entity
    within tol of (x, y)."""
    window = _in_window(view.stream(topic), from_sec, to_sec)
    where = _window_text(topic, from_sec, to_sec)
    if not window:
        return f"no messages on {where}"
    closest_by_message = [min(_entity_distances(msg, x, y, entity_id), default=math.inf)
                          for _, msg in window]
    matched = sum(distance <= float(tol) for distance in closest_by_message)
    if matched >= float(min_fraction) * len(window):
        return None
    return (f"{_entity_text(entity_id, x, y, tol)} in {matched}/{len(window)} messages "
            f"({matched / len(window):.0%}) < required {float(min_fraction):.0%} on {where}; "
            f"closest {min(closest_by_message):.2f}")


@register
def no_entity_near(view, topic, x, y, tol, entity_id=None, from_sec=0.0, to_sec=math.inf,
                   **_):
    """No message in the window carries an entity within tol of (x, y).
    Absence needs evidence: a window without messages fails."""
    window = _in_window(view.stream(topic), from_sec, to_sec)
    if not window:
        return f"no messages on {_window_text(topic, from_sec, to_sec)}"
    for timestamp, msg in window:
        distance = min(_entity_distances(msg, x, y, entity_id), default=math.inf)
        if distance <= float(tol):
            return (f"{_entity_text(entity_id, x, y, tol)} at t={timestamp:.1f} on {topic}, "
                    f"distance {distance:.2f}")
    return None


# ----------------------------------------------------------- perception
# Mirror of SceneRelation.PREDICATE_* (pure: no rclpy import; SCENE_CATEGORIES
# lives in the scene graph section above); unit/test_perception_assertions.py
# checks both against the generated messages when the workspace is built.
# Positions are compared as reported, i.e. in the message's frame_id ("map").
SCENE_PREDICATES = {'INSIDE': 0, 'NEAR': 1, 'BLOCKS': 2, 'OCCUPIES': 3}
_PREDICATE_NAMES = {value: name for name, value in SCENE_PREDICATES.items()}
# Sensor stamps may lead the publisher's /clock by a few ms (separate topics);
# a sensor stamp further ahead means the two stamps come from different clocks.
LATENCY_CLOCK_SKEW_SEC = 0.05


def _reports(stream, category):
    """(t, entity) for every reported entity of the category, in order."""
    wanted = SCENE_CATEGORIES[category]
    return [(timestamp, entity) for timestamp, msg in stream
            for entity in msg.entities if entity.category == wanted]


def _xy(entity):
    return f"({float(entity.position.x):.2f}, {float(entity.position.y):.2f})"


def _seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


@register
def observes(view, topic, source, category, x, y, tol, within_sec=None, **_):
    """Some SceneObservation from source reports an entity of the category
    within tol of (x, y), by sim time within_sec if given."""
    stream = _from_source(view.stream(topic), source)
    where = _where(topic, source)
    if not stream:
        return f"no messages on {where}"
    reports = _reports(stream, category)
    for timestamp, entity in reports:
        if _distance_xy(entity.position, x, y) <= float(tol):
            if within_sec is not None and timestamp > float(within_sec):
                return (f"{category} within {tol} of ({x}, {y}) on {where} first reported "
                        f"at t={timestamp:.1f}s > {within_sec}s")
            return None
    if not reports:
        return f"no {category} entity ever reported on {where}"
    timestamp, nearest = min(reports, key=lambda report: _distance_xy(report[1].position, x, y))
    return (f"no {category} within {tol} of ({x}, {y}) on {where}; nearest {nearest.id} at "
            f"{_xy(nearest)}, distance {_distance_xy(nearest.position, x, y):.2f}, "
            f"t={timestamp:.1f}s")


@register
def observes_only(view, topic, source, category, points, tol, **_):
    """Every entity of the category that source reports lies within tol of
    one of the listed [x, y] points: a false-positive guard (walls reported
    as obstacles, map misalignment). Fails closed without messages."""
    stream = _from_source(view.stream(topic), source)
    where = _where(topic, source)
    if not stream:
        return f"no messages on {where}"
    strays = []
    for timestamp, entity in _reports(stream, category):
        distance = min((_distance_xy(entity.position, px, py) for px, py in points),
                       default=math.inf)
        if distance > float(tol):
            strays.append((distance, timestamp, entity))
    if not strays:
        return None
    distance, timestamp, entity = max(strays, key=lambda stray: stray[0])
    ids = {stray_entity.id for _, _, stray_entity in strays}
    return (f"{len(strays)} {category} report(s) ({len(ids)} ids) farther than {tol} from "
            f"every point of {points} on {where}; farthest {entity.id} at {_xy(entity)}, "
            f"{distance:.2f} away, t={timestamp:.1f}s")


@register
def relation_seen(view, topic, source, predicate, object_id, subject_category=None, **_):
    """Some message from source holds `<subject> predicate object_id`; with
    subject_category, the subject is an entity of that category (as source
    reported it anywhere in the recording: a relation may name an entity
    reported in an earlier delta)."""
    stream = _from_source(view.stream(topic), source)
    where = _where(topic, source)
    if not stream:
        return f"no messages on {where}"
    wanted = SCENE_PREDICATES[predicate]
    wanted_category = None if subject_category is None else SCENE_CATEGORIES[subject_category]
    category_of = {entity.id: entity.category for _, msg in stream for entity in msg.entities}
    seen = set()
    for _, msg in stream:
        for relation in msg.relations:
            subject = category_of.get(relation.subject_id)
            if (relation.predicate == wanted and relation.object_id == object_id
                    and (wanted_category is None or subject == wanted_category)):
                return None
            seen.add(f"{relation.subject_id} ({_CATEGORY_NAMES.get(subject, '?')}) "
                     f"{_PREDICATE_NAMES.get(relation.predicate, relation.predicate)} "
                     f"{relation.object_id}")
    subject_name = '' if subject_category is None else f'{subject_category} '
    return (f"no {subject_name}subject {predicate} {object_id} on {where}; "
            f"seen {sorted(seen)[:10]}")


@register
def removal_seen(view, topic, source, category, x, y, tol, **_):
    """An entity of the category that source reported within tol of (x, y)
    appears in a LATER message's removed_entity_ids."""
    stream = _from_source(view.stream(topic), source)
    where = _where(topic, source)
    if not stream:
        return f"no messages on {where}"
    wanted = SCENE_CATEGORIES[category]
    last_report = {}               # entity id -> sim t of its last report near (x, y)
    for timestamp, msg in stream:
        if any(entity_id in last_report for entity_id in msg.removed_entity_ids):
            return None
        for entity in msg.entities:
            if entity.category == wanted and _distance_xy(entity.position, x, y) <= float(tol):
                last_report[entity.id] = timestamp
    if not last_report:
        return f"no {category} within {tol} of ({x}, {y}) ever reported on {where}"
    reports = ', '.join(f"{entity_id} (last t={timestamp:.1f}s)"
                        for entity_id, timestamp in sorted(last_report.items()))
    return (f"{category} reported within {tol} of ({x}, {y}) on {where} never appeared in "
            f"removed_entity_ids: {reports}")


@register
def latency_below(view, topic, max_sec, source=None, **_):
    """Perception latency below max_sec: per message, meta.stamp -
    sensor_pose.header.stamp (sim s), the age of the newest sensor data an
    observation is built from when it is published. Fails on the maximum."""
    stream = _from_source(view.stream(topic), source)
    where = _where(topic, source)
    if not stream:
        return f"no messages on {where}"
    latencies = [(_seconds(msg.meta.stamp) - _seconds(msg.sensor_pose.header.stamp), timestamp)
                 for timestamp, msg in stream]
    earliest, earliest_t = min(latencies)
    if earliest < -LATENCY_CLOCK_SKEW_SEC:
        return (f"negative latency {earliest:.3f} s at t={earliest_t:.1f}s on {where}: "
                f"sensor stamp after the publication stamp (different clocks?)")
    worst, worst_t = max(latencies)
    if worst < float(max_sec):
        return None
    return (f"latency {worst:.3f} s >= {max_sec} s at t={worst_t:.1f}s on {where} "
            f"(over {len(latencies)} messages)")
