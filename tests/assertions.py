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


@register
def min_rate(view, topic, hz, margin=None, **_):
    stream = view.stream(topic)
    if not stream:
        return f"no messages on {topic}"
    if len(stream) < 2:
        return (f"only 1 message on {topic}, "
                f"cannot estimate rate")
    span = stream[-1][0] - stream[0][0]
    if span <= 0:
        return f"messages on {topic} span {span}s, cannot estimate rate"
    actual = (len(stream) - 1) / span
    margin_fraction = DEFAULT_RATE_MARGIN if margin is None else float(margin)
    required = float(hz) * (1.0 - margin_fraction)
    if actual < required:
        return (f"rate {actual:.2f} Hz < required {required:.2f} Hz "
                f"(nominal {hz} - {margin_fraction:.0%} margin) on {topic}")
    return None


@register
def arrives_at(view, topic, x, y, tol, **_):
    stream = view.stream(topic)
    for _, msg in stream:
        if _distance_xy(_get_field(msg, 'pose.pose.position'), x, y) <= float(tol):
            return None
    if not stream:
        return f"no messages on {topic}"
    last = _get_field(stream[-1][1], 'pose.pose.position')
    return (f"never came within {tol} of ({x}, {y}); "
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


def _task_events(view, topic, task_id, source=None):
    return [(timestamp, msg) for timestamp, msg in view.stream(topic)
            if msg.task_id == task_id and
            (source is None or msg.meta.source_id == source)]


def _history(events):
    """State names in order, consecutive repeats collapsed."""
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
    sim time <= within_sec if given)."""
    wanted_state = TASK_STATES[state]
    events = _task_events(view, topic, task_id, source)
    for timestamp, msg in events:
        if msg.state != wanted_state:
            continue
        if error_code is not None and msg.error_code != int(error_code):
            continue
        if within_sec is not None and timestamp > float(within_sec):
            return (f"task {task_id} reached {state} only at t={timestamp:.1f}s "
                    f"> {within_sec}s")
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
