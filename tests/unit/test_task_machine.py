"""Robot-side task state machine (fleetgraph_robot.task_machine). Pure — no
rclpy, no sim. Covers accept/reject reasons, leg planning per task type,
progress, retry-then-fail, cancel, and idempotent re-delivery.
"""
import pytest

from fleetgraph_robot.task_machine import (
    CancelNavigation, ErrorCode, Goal, Navigate, TaskMachine, TaskSpec, TaskState,
    TaskType, Update, Wait)

LOCATIONS = {'dock': Goal(0, 0, 0, 'dock'), 'lab_1': Goal(6, 1, 0, 'lab_1'),
             'lab_2': Goal(6, -1, 0, 'lab_2'), 'storage': Goal(1, 1, 1.57, 'storage')}


def make_machine(**options):
    options.setdefault('capabilities', ['navigate', 'deliver', 'patrol'])
    return TaskMachine(resolve=LOCATIONS.get, **options)


def make_spec(task_id='t1', task_type=TaskType.GO_TO, location_ids=('lab_1',), rev=1,
              **fields):
    return TaskSpec(task_id=task_id, type=task_type,
                    locations=[(location_id, None) for location_id in location_ids],
                    assignment_rev=rev, **fields)


def of_type(outputs, output_type):
    return [output for output in outputs if isinstance(output, output_type)]


def navigations(outputs):
    return of_type(outputs, Navigate)


def only_update(outputs):
    updates = of_type(outputs, Update)
    assert len(updates) == 1, outputs
    return updates[0]


# ---------------------------------------------------------------- accept

def test_go_to_accepts_and_navigates():
    machine = make_machine()
    outputs = machine.assign(make_spec(), battery=0.9)
    update = only_update(outputs)
    assert (update.state, update.leg_count, update.leg_index, update.progress) == \
        (TaskState.EXECUTING, 1, 0, 0.0)
    assert navigations(outputs)[0].goal == LOCATIONS['lab_1']
    assert machine.active_task_id == 't1'


def test_pose_in_location_wins_over_registry():
    machine = make_machine()
    explicit = Goal(3.0, 2.0, 0.5, 'lab_1')
    spec = TaskSpec('t1', TaskType.GO_TO, [('lab_1', explicit)])
    outputs = machine.assign(spec, battery=0.9)
    assert navigations(outputs)[0].goal == explicit


@pytest.mark.parametrize('spec_fields, error_code', [
    (dict(task_type=TaskType.GO_TO, location_ids=()), ErrorCode.INVALID_TASK),
    (dict(task_type=TaskType.DELIVERY, location_ids=('storage',)), ErrorCode.INVALID_TASK),
    (dict(task_type=0, location_ids=('lab_1',)), ErrorCode.INVALID_TASK),
    (dict(location_ids=('nowhere',)), ErrorCode.UNKNOWN_LOCATION),
    (dict(required_capabilities=['arm']), ErrorCode.MISSING_CAPABILITY),
])
def test_rejects_with_reason(spec_fields, error_code):
    machine = make_machine()
    update = only_update(machine.assign(make_spec(**spec_fields), battery=0.9))
    assert update.state == TaskState.REJECTED and update.error_code == error_code
    assert update.reason
    assert machine.active_task_id == ''


def test_rejects_low_battery_and_unavailable():
    machine = make_machine(min_battery=0.2)
    low_battery = only_update(machine.assign(make_spec(), battery=0.1))
    unavailable = only_update(machine.assign(make_spec(), battery=0.9, available=False))
    assert low_battery.error_code == ErrorCode.LOW_BATTERY
    assert unavailable.error_code == ErrorCode.ROBOT_UNAVAILABLE


def test_busy_robot_rejects_second_task_keeps_first():
    machine = make_machine()
    machine.assign(make_spec('t1'), battery=0.9)
    update = only_update(machine.assign(make_spec('t2'), battery=0.9))
    assert (update.task_id, update.state, update.error_code) == \
        ('t2', TaskState.REJECTED, ErrorCode.ROBOT_BUSY)
    assert machine.active_task_id == 't1'


# ---------------------------------------------------------- idempotency

def test_repeat_assignment_is_acked_without_new_navigation():
    machine = make_machine()
    machine.assign(make_spec(rev=2), battery=0.9)
    outputs = machine.assign(make_spec(rev=2), battery=0.9)
    assert only_update(outputs).state == TaskState.EXECUTING and not navigations(outputs)


def test_outdated_rev_is_dropped():
    machine = make_machine()
    machine.assign(make_spec(rev=2), battery=0.9)
    assert machine.assign(make_spec(rev=1), battery=0.9) == []


def test_higher_rev_restarts_task():
    machine = make_machine()
    machine.assign(make_spec(rev=1, location_ids=('lab_1',)), battery=0.9)
    outputs = machine.assign(make_spec(rev=2, location_ids=('lab_2',)), battery=0.9)
    assert of_type(outputs, CancelNavigation)
    assert navigations(outputs)[0].goal == LOCATIONS['lab_2']
    assert only_update(outputs).assignment_rev == 2


def test_repeat_after_completion_re_emits_terminal():
    machine = make_machine()
    outputs = machine.assign(make_spec(), battery=0.9)
    machine.nav_result(navigations(outputs)[0].generation, succeeded=True)
    update = only_update(machine.assign(make_spec(), battery=0.9))
    assert update.state == TaskState.COMPLETED and machine.active_task_id == ''


# ------------------------------------------------------------ execution

def test_go_to_completes():
    machine = make_machine()
    navigation = navigations(machine.assign(make_spec(), battery=0.9))[0]
    update = only_update(machine.nav_result(navigation.generation, succeeded=True))
    assert (update.state, update.progress, update.error_code) == \
        (TaskState.COMPLETED, 1.0, ErrorCode.NONE)
    assert machine.active_task_id == ''


def test_delivery_two_legs_with_dwell():
    machine = make_machine(dwell_sec=2.0)
    spec = make_spec(task_type=TaskType.DELIVERY, location_ids=('storage', 'lab_1'))
    outputs = machine.assign(spec, battery=0.9)
    assert only_update(outputs).leg_count == 2
    assert navigations(outputs)[0].goal == LOCATIONS['storage']
    outputs = machine.nav_result(navigations(outputs)[0].generation, succeeded=True)
    wait = of_type(outputs, Wait)
    assert wait and wait[0].seconds == 2.0 and not navigations(outputs)  # loading
    outputs = machine.dwell_done()
    assert navigations(outputs)[0].goal == LOCATIONS['lab_1']
    assert only_update(outputs).leg_index == 1
    outputs = machine.nav_result(navigations(outputs)[0].generation, succeeded=True)
    assert of_type(outputs, Wait)                                        # unloading
    assert only_update(machine.dwell_done()).state == TaskState.COMPLETED


def test_patrol_loops_expand_legs():
    machine = make_machine()
    spec = make_spec(task_type=TaskType.PATROL, location_ids=('lab_1', 'lab_2'),
                     patrol_loops=2)
    outputs = machine.assign(spec, battery=0.9)
    assert only_update(outputs).leg_count == 4
    goals = []
    for _ in range(4):
        navigation = navigations(outputs)[0]
        goals.append(navigation.goal.location_id)
        outputs = machine.nav_result(navigation.generation, succeeded=True)
    assert goals == ['lab_1', 'lab_2', 'lab_1', 'lab_2']
    assert only_update(outputs).state == TaskState.COMPLETED


def test_progress_from_feedback_is_monotonic():
    machine = make_machine()
    spec = make_spec(task_type=TaskType.PATROL, location_ids=('lab_1', 'lab_2'))
    machine.assign(spec, battery=0.9)
    first = only_update(machine.nav_feedback(4.0)).progress   # first sample = leg length
    halfway = only_update(machine.nav_feedback(2.0)).progress
    detour = only_update(machine.nav_feedback(3.0)).progress  # replanned detour: no regress
    assert first == 0.0 and halfway == pytest.approx(0.25) and detour >= halfway
    assert only_update(machine.nav_feedback(2.0)).distance_remaining == 2.0


def test_stale_nav_result_is_ignored():
    machine = make_machine()
    old = navigations(machine.assign(make_spec(rev=1), battery=0.9))[0]
    machine.assign(make_spec(rev=2), battery=0.9)
    assert machine.nav_result(old.generation, succeeded=False) == []
    assert machine.active_task_id == 't1'


def test_nav_failure_retries_then_fails_requesting_reassignment():
    machine = make_machine(max_leg_retries=1)
    navigation = navigations(machine.assign(make_spec(), battery=0.9))[0]
    outputs = machine.nav_result(navigation.generation, succeeded=False, reason='planner failed')
    retry = navigations(outputs)[0]
    assert retry.goal == navigation.goal and retry.generation != navigation.generation
    assert only_update(outputs).state == TaskState.EXECUTING and machine.replan_count == 1
    update = only_update(machine.nav_result(retry.generation, succeeded=False,
                                            reason='planner failed'))
    assert (update.state, update.error_code, update.reassignment_requested) == \
        (TaskState.FAILED, ErrorCode.NAVIGATION_FAILED, True)
    assert 'planner failed' in update.reason and machine.active_task_id == ''


# --------------------------------------------------------------- cancel

def test_cancel_active_task():
    machine = make_machine()
    machine.assign(make_spec(), battery=0.9)
    outputs = machine.cancel('t1', 'operator')
    assert of_type(outputs, CancelNavigation)
    update = only_update(outputs)
    assert (update.state, update.error_code, update.reason) == \
        (TaskState.CANCELLED, ErrorCode.CANCELLED_BY_OPERATOR, 'operator')
    assert machine.active_task_id == ''


def test_cancel_unknown_task_is_noop():
    machine = make_machine()
    machine.assign(make_spec('t1'), battery=0.9)
    assert machine.cancel('t9', 'x') == []
    assert machine.active_task_id == 't1'


def test_late_nav_result_after_cancel_is_ignored():
    machine = make_machine()
    navigation = navigations(machine.assign(make_spec(), battery=0.9))[0]
    machine.cancel('t1', 'operator')
    assert machine.nav_result(navigation.generation, succeeded=False) == []
