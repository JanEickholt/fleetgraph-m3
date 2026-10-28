"""Robot-side task state machine (fleetgraph_robot.task_machine). Pure — no
rclpy, no sim. Covers accept/reject reasons, leg planning per task type,
progress, retry-then-fail, cancel, and idempotent re-delivery.
"""
import pytest

from fleetgraph_robot.task_machine import (
    CancelNavigation, E, Goal, Navigate, S, T, TaskMachine, TaskSpec, Update,
    Wait)

LOC = {'dock': Goal(0, 0, 0, 'dock'), 'lab_1': Goal(6, 1, 0, 'lab_1'),
       'lab_2': Goal(6, -1, 0, 'lab_2'), 'storage': Goal(1, 1, 1.57, 'storage')}


def machine(**kw):
    kw.setdefault('capabilities', ['navigate', 'deliver', 'patrol'])
    return TaskMachine(resolve=LOC.get, **kw)


def spec(task_id='t1', type=T.GO_TO, locs=('lab_1',), rev=1, **kw):
    return TaskSpec(task_id=task_id, type=type,
                    locations=[(lid, None) for lid in locs],
                    assignment_rev=rev, **kw)


def updates(out):
    return [o for o in out if isinstance(o, Update)]


def navs(out):
    return [o for o in out if isinstance(o, Navigate)]


def only_update(out):
    ups = updates(out)
    assert len(ups) == 1, out
    return ups[0]


# ---------------------------------------------------------------- accept

def test_go_to_accepts_and_navigates():
    m = machine()
    out = m.assign(spec(), battery=0.9)
    u = only_update(out)
    assert (u.state, u.leg_count, u.leg_index, u.progress) == (S.EXECUTING, 1, 0, 0.0)
    assert navs(out)[0].goal == LOC['lab_1']
    assert m.active_task_id == 't1'


def test_pose_in_location_wins_over_registry():
    m = machine()
    explicit = Goal(3.0, 2.0, 0.5, 'lab_1')
    out = m.assign(TaskSpec('t1', T.GO_TO, [('lab_1', explicit)]), battery=0.9)
    assert navs(out)[0].goal == explicit


@pytest.mark.parametrize('kw, code', [
    (dict(type=T.GO_TO, locs=()), E.INVALID_TASK),
    (dict(type=T.DELIVERY, locs=('storage',)), E.INVALID_TASK),
    (dict(type=0, locs=('lab_1',)), E.INVALID_TASK),
    (dict(locs=('nowhere',)), E.UNKNOWN_LOCATION),
    (dict(required_capabilities=['arm']), E.MISSING_CAPABILITY),
])
def test_rejects_with_reason(kw, code):
    m = machine()
    u = only_update(m.assign(spec(**kw), battery=0.9))
    assert u.state == S.REJECTED and u.error_code == code and u.reason
    assert m.active_task_id == ''


def test_rejects_low_battery_and_unavailable():
    m = machine(min_battery=0.2)
    assert only_update(m.assign(spec(), battery=0.1)).error_code == E.LOW_BATTERY
    assert only_update(m.assign(spec(), battery=0.9, available=False)).error_code \
        == E.ROBOT_UNAVAILABLE


def test_busy_robot_rejects_second_task_keeps_first():
    m = machine()
    m.assign(spec('t1'), battery=0.9)
    u = only_update(m.assign(spec('t2'), battery=0.9))
    assert (u.task_id, u.state, u.error_code) == ('t2', S.REJECTED, E.ROBOT_BUSY)
    assert m.active_task_id == 't1'


# ---------------------------------------------------------- idempotency

def test_repeat_assignment_is_acked_without_new_navigation():
    m = machine()
    m.assign(spec(rev=2), battery=0.9)
    out = m.assign(spec(rev=2), battery=0.9)
    assert only_update(out).state == S.EXECUTING and not navs(out)


def test_outdated_rev_is_dropped():
    m = machine()
    m.assign(spec(rev=2), battery=0.9)
    assert m.assign(spec(rev=1), battery=0.9) == []


def test_higher_rev_restarts_task():
    m = machine()
    m.assign(spec(rev=1, locs=('lab_1',)), battery=0.9)
    out = m.assign(spec(rev=2, locs=('lab_2',)), battery=0.9)
    assert any(isinstance(o, CancelNavigation) for o in out)
    assert navs(out)[0].goal == LOC['lab_2']
    assert only_update(out).assignment_rev == 2


def test_repeat_after_completion_re_emits_terminal():
    m = machine()
    out = m.assign(spec(), battery=0.9)
    m.nav_result(navs(out)[0].generation, succeeded=True)
    u = only_update(m.assign(spec(), battery=0.9))
    assert u.state == S.COMPLETED and m.active_task_id == ''


# ------------------------------------------------------------ execution

def test_go_to_completes():
    m = machine()
    nav = navs(m.assign(spec(), battery=0.9))[0]
    u = only_update(m.nav_result(nav.generation, succeeded=True))
    assert (u.state, u.progress, u.error_code) == (S.COMPLETED, 1.0, E.NONE)
    assert m.active_task_id == ''


def test_delivery_two_legs_with_dwell():
    m = machine(dwell_sec=2.0)
    out = m.assign(spec(type=T.DELIVERY, locs=('storage', 'lab_1')), battery=0.9)
    assert only_update(out).leg_count == 2
    assert navs(out)[0].goal == LOC['storage']
    out = m.nav_result(navs(out)[0].generation, succeeded=True)
    wait = [o for o in out if isinstance(o, Wait)]
    assert wait and wait[0].seconds == 2.0 and not navs(out)   # loading
    out = m.dwell_done()
    assert navs(out)[0].goal == LOC['lab_1']
    assert only_update(out).leg_index == 1
    out = m.nav_result(navs(out)[0].generation, succeeded=True)
    assert [o for o in out if isinstance(o, Wait)]               # unloading
    assert only_update(m.dwell_done()).state == S.COMPLETED


def test_patrol_loops_expand_legs():
    m = machine()
    out = m.assign(spec(type=T.PATROL, locs=('lab_1', 'lab_2'), patrol_loops=2),
                   battery=0.9)
    assert only_update(out).leg_count == 4
    goals = []
    for _ in range(4):
        nav = navs(out)[0]
        goals.append(nav.goal.location_id)
        out = m.nav_result(nav.generation, succeeded=True)
    assert goals == ['lab_1', 'lab_2', 'lab_1', 'lab_2']
    assert only_update(out).state == S.COMPLETED


def test_progress_from_feedback_is_monotonic():
    m = machine()
    m.assign(spec(type=T.PATROL, locs=('lab_1', 'lab_2')), battery=0.9)
    p1 = only_update(m.nav_feedback(4.0)).progress   # first sample = leg length
    p2 = only_update(m.nav_feedback(2.0)).progress
    p3 = only_update(m.nav_feedback(3.0)).progress   # replanned detour: no regress
    assert p1 == 0.0 and p2 == pytest.approx(0.25) and p3 >= p2
    assert only_update(m.nav_feedback(2.0)).distance_remaining == 2.0


def test_stale_nav_result_is_ignored():
    m = machine()
    old = navs(m.assign(spec(rev=1), battery=0.9))[0]
    m.assign(spec(rev=2), battery=0.9)
    assert m.nav_result(old.generation, succeeded=False) == []
    assert m.active_task_id == 't1'


def test_nav_failure_retries_then_fails_requesting_reassignment():
    m = machine(max_leg_retries=1)
    nav = navs(m.assign(spec(), battery=0.9))[0]
    out = m.nav_result(nav.generation, succeeded=False, reason='planner failed')
    retry = navs(out)[0]
    assert retry.goal == nav.goal and retry.generation != nav.generation
    assert only_update(out).state == S.EXECUTING and m.replan_count == 1
    u = only_update(m.nav_result(retry.generation, succeeded=False,
                                 reason='planner failed'))
    assert (u.state, u.error_code, u.reassignment_requested) == \
        (S.FAILED, E.NAVIGATION_FAILED, True)
    assert 'planner failed' in u.reason and m.active_task_id == ''


# --------------------------------------------------------------- cancel

def test_cancel_active_task():
    m = machine()
    m.assign(spec(), battery=0.9)
    out = m.cancel('t1', 'operator')
    assert any(isinstance(o, CancelNavigation) for o in out)
    u = only_update(out)
    assert (u.state, u.error_code, u.reason) == \
        (S.CANCELLED, E.CANCELLED_BY_OPERATOR, 'operator')
    assert m.active_task_id == ''


def test_cancel_unknown_task_is_noop():
    m = machine()
    m.assign(spec('t1'), battery=0.9)
    assert m.cancel('t9', 'x') == []
    assert m.active_task_id == 't1'


def test_late_nav_result_after_cancel_is_ignored():
    m = machine()
    nav = navs(m.assign(spec(), battery=0.9))[0]
    m.cancel('t1', 'operator')
    assert m.nav_result(nav.generation, succeeded=False) == []
