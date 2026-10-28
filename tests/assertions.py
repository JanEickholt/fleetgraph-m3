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


def register(fn):
    REGISTRY[fn.__name__] = fn
    return fn


def _get(obj, dotted):
    """Follow a dotted field path on messages (getattr) or dicts ([])."""
    cur = obj
    for part in dotted.split('.'):
        cur = cur[part] if isinstance(cur, dict) else getattr(cur, part)
    return cur


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
    m = DEFAULT_RATE_MARGIN if margin is None else float(margin)
    required = float(hz) * (1.0 - m)
    if actual < required:
        return (f"rate {actual:.2f} Hz < required {required:.2f} Hz "
                f"(nominal {hz} - {m:.0%} margin) on {topic}")
    return None


@register
def arrives_at(view, topic, x, y, tol, **_):
    for t, msg in view.stream(topic):
        pos = _get(msg, 'pose.pose.position')
        dx = float(_get(pos, 'x')) - float(x)
        dy = float(_get(pos, 'y')) - float(y)
        if math.hypot(dx, dy) <= float(tol):
            return None
    stream = view.stream(topic)
    if not stream:
        return f"no messages on {topic}"
    last = _get(stream[-1][1], 'pose.pose.position')
    dist = math.hypot(float(_get(last, 'x')) - float(x),
                      float(_get(last, 'y')) - float(y))
    return (f"never came within {tol} of ({x}, {y}); "
            f"last pose ({float(_get(last, 'x')):.2f}, "
            f"{float(_get(last, 'y')):.2f}), distance {dist:.2f}")


@register
def never_below(view, topic, field, threshold, **_):
    # NaN compares False against everything, so `min()` over a stream
    # containing NaN silently discards it. Check EVERY element; NaN
    # anywhere = violation (safety assertions fail closed on unknown).
    for t, msg in view.stream(topic):
        raw = _get(msg, field)
        values = list(raw) if isinstance(raw, (list, tuple)) else [raw]
        for v in values:
            try:
                v = float(v)
            except (TypeError, ValueError):
                continue
            if v != v:
                return f"{topic}.{field} produced NaN — treated as violation"
            if v < float(threshold):
                return f"{topic}.{field} hit {v:.3f} < {threshold}"
    if not view.stream(topic):
        return f"no messages on {topic}"
    return None


@register
def final_value(view, topic, field, equals, tol=0.01, **_):
    stream = view.stream(topic)
    if not stream:
        return f"no messages on {topic}"
    value = _get(stream[-1][1], field)
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
    q = _get(stream[-1][1], 'pose.pose.orientation')
    yaw = math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
    err = (yaw - float(deg) + 180.0) % 360.0 - 180.0
    if abs(err) <= float(tol):
        return None
    return (f"final yaw {yaw:.2f} deg vs target {deg}, "
            f"|err| {abs(err):.2f} deg > {tol}")


# ---------------------------------------------------------------- tasks
# Mirror of fleetgraph_interfaces/msg/TaskUpdate STATE_* (kept pure: no
# rclpy import here). unit/test_contract_mirror.py checks it against the
# generated message when the workspace is built.
TASK_STATES = {'RECEIVED': 0, 'PARSED': 1, 'QUEUED': 2, 'ASSIGNED': 3,
               'EXECUTING': 4, 'COMPLETED': 5, 'FAILED': 6, 'CANCELLED': 7,
               'REASSIGNED': 8, 'REJECTED': 9}
_STATE_NAMES = {v: k for k, v in TASK_STATES.items()}


def _task_events(view, topic, task_id, source=None):
    return [(t, m) for t, m in view.stream(topic)
            if m.task_id == task_id and
            (source is None or m.meta.source_id == source)]


def _history(events):
    names = []
    for _, m in events:
        n = _STATE_NAMES.get(m.state, str(m.state))
        if not names or names[-1] != n:
            names.append(n)
    return names


@register
def task_state(view, task_id, state, topic='/fleet/task_update',
               error_code=None, source=None, within_sec=None, **_):
    """Some TaskUpdate for task_id has `state` (+ error_code / source, and
    sim time <= within_sec if given)."""
    want = TASK_STATES[state]
    events = _task_events(view, topic, task_id, source)
    for t, m in events:
        if m.state != want:
            continue
        if error_code is not None and m.error_code != int(error_code):
            continue
        if within_sec is not None and t > float(within_sec):
            return (f"task {task_id} reached {state} only at t={t:.1f}s "
                    f"> {within_sec}s")
        return None
    codes = sorted({m.error_code for _, m in events})
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
    last = {}
    for t, m in stream:
        src, seq = m.meta.source_id, int(m.meta.seq)
        if src in last and seq != last[src] + 1:
            kind = 'duplicate/reorder' if seq <= last[src] else 'gap (lost events)'
            return (f"{topic} source {src}: seq {last[src]} -> {seq} at "
                    f"t={t:.1f}s, {kind}")
        last[src] = seq
    return None


@register
def task_sequence(view, task_id, states, topic='/fleet/task_update', **_):
    """The task's state history contains `states` in this order
    (other states may occur in between)."""
    hist = _history(_task_events(view, topic, task_id))
    i = 0
    for name in hist:
        if i < len(states) and name == states[i]:
            i += 1
    if i == len(states):
        return None
    return (f"task {task_id}: expected order {states}, history {hist} "
            f"(missing from {states[i]})")
