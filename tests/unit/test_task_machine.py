"""Robot-side task state machine (fleetgraph_robot.task_machine). Pure — no
rclpy, no sim. Covers accept/reject reasons, leg planning per task type,
progress, retry-then-fail, waiting out a blocked route, cancel, and idempotent
re-delivery.
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


# -------------------------------------------------------- blocked route

NO_PATH = 'planner: no valid path (208)'


class Clock:
    """The machine's clock, set by hand (s)."""

    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


def blocked_machine(**options):
    clock = Clock()
    options.setdefault('blocked_wait_sec', 5.0)
    options.setdefault('max_blocked_sec', 60.0)
    return make_machine(clock=clock, **options), clock


def report_blocked(machine, navigation):
    return machine.nav_result(navigation.generation, succeeded=False, reason=NO_PATH,
                              blocked=True)


def test_blocked_leg_waits_then_retries_and_completes():
    machine, clock = blocked_machine()
    navigation = navigations(machine.assign(make_spec(), battery=0.9))[0]
    clock.now = 12.0
    outputs = report_blocked(machine, navigation)
    assert of_type(outputs, Wait) == [Wait(5.0, blocked=True)] and not navigations(outputs)
    update = only_update(outputs)
    assert (update.state, update.error_code, update.reassignment_requested) == \
        (TaskState.EXECUTING, ErrorCode.NONE, False)
    assert update.reason == f'waiting 5 s for blocked route (0.0 of 60 s blocked): {NO_PATH}'
    assert machine.replan_count == 0 and machine.active_task_id == 't1'
    clock.now = 17.0
    outputs = machine.blocked_wait_done()
    retry = navigations(outputs)[0]
    assert retry.goal == navigation.goal and retry.generation != navigation.generation
    assert only_update(outputs).reason == 'retrying blocked route'
    assert machine.replan_count == 1
    update = only_update(machine.nav_result(retry.generation, succeeded=True))
    assert (update.state, update.error_code) == (TaskState.COMPLETED, ErrorCode.NONE)


def test_blocked_leg_fails_after_max_blocked_sec_from_its_first_blocked_report():
    machine, clock = blocked_machine(blocked_wait_sec=5.0, max_blocked_sec=60.0)
    navigation = navigations(machine.assign(make_spec(), battery=0.9))[0]
    # Nav2 reports blocked after its own recoveries, 30 s into the leg (not
    # counted), then 20 s into each retry: blocked 0, 25 and 50 s, wait and retry
    for now in (30.0, 55.0, 80.0):
        clock.now = now
        assert of_type(report_blocked(machine, navigation), Wait) == [Wait(5.0, blocked=True)], now
        clock.now = now + 5.0
        navigation = navigations(machine.blocked_wait_done())[0]
    clock.now = 90.0                                        # blocked 60 s: the limit
    update = only_update(report_blocked(machine, navigation))
    assert (update.state, update.error_code, update.reassignment_requested) == \
        (TaskState.FAILED, ErrorCode.BLOCKED, True)
    assert update.reason == f'leg 0 to lab_1 blocked for 60.0 s: {NO_PATH}'
    assert machine.replan_count == 3 and machine.active_task_id == ''


def test_a_completed_leg_resets_the_blocked_time():
    machine, clock = blocked_machine(blocked_wait_sec=5.0, max_blocked_sec=60.0)
    spec = make_spec(task_type=TaskType.PATROL, location_ids=('lab_1', 'lab_2'))
    navigation = navigations(machine.assign(spec, battery=0.9))[0]
    clock.now = 10.0
    report_blocked(machine, navigation)
    clock.now = 15.0
    navigation = navigations(machine.blocked_wait_done())[0]
    clock.now = 50.0
    navigation = navigations(machine.nav_result(navigation.generation, succeeded=True))[0]
    assert navigation.goal == LOCATIONS['lab_2']
    clock.now = 100.0                                       # 90 s after leg 0's first report
    assert of_type(report_blocked(machine, navigation), Wait) == [Wait(5.0, blocked=True)]
    clock.now = 105.0
    navigation = navigations(machine.blocked_wait_done())[0]
    clock.now = 159.9                                       # leg 1 blocked 59.9 s
    assert of_type(report_blocked(machine, navigation), Wait) == [Wait(5.0, blocked=True)]
    clock.now = 164.9
    navigation = navigations(machine.blocked_wait_done())[0]
    clock.now = 165.0
    update = only_update(report_blocked(machine, navigation))
    assert (update.state, update.error_code) == (TaskState.FAILED, ErrorCode.BLOCKED)
    assert update.reason.startswith('leg 1 to lab_2 blocked for 65.0 s')


def test_blocked_retries_leave_the_leg_retries_for_other_failures():
    machine, clock = blocked_machine(max_leg_retries=1)
    navigation = navigations(machine.assign(make_spec(), battery=0.9))[0]
    for now in (10.0, 20.0, 30.0):                          # more than max_leg_retries
        clock.now = now
        report_blocked(machine, navigation)
        navigation = navigations(machine.blocked_wait_done())[0]
    outputs = machine.nav_result(navigation.generation, succeeded=False, reason='tf error')
    retry = navigations(outputs)[0]
    assert only_update(outputs).reason == 'replanning: tf error'
    update = only_update(machine.nav_result(retry.generation, succeeded=False, reason='tf error'))
    assert (update.state, update.error_code, update.reassignment_requested) == \
        (TaskState.FAILED, ErrorCode.NAVIGATION_FAILED, True)
    assert machine.replan_count == 4


def test_max_blocked_sec_zero_fails_on_the_first_blocked_report():
    machine, _ = blocked_machine(max_blocked_sec=0.0)
    navigation = navigations(machine.assign(make_spec(), battery=0.9))[0]
    update = only_update(report_blocked(machine, navigation))
    assert (update.state, update.error_code) == (TaskState.FAILED, ErrorCode.BLOCKED)


def test_cancel_during_the_blocked_wait_leaves_no_stale_timer_effect():
    machine, _ = blocked_machine()
    navigation = navigations(machine.assign(make_spec('t1'), battery=0.9))[0]
    report_blocked(machine, navigation)
    outputs = machine.cancel('t1', 'operator')
    assert of_type(outputs, CancelNavigation)               # the agent stops its wait timer
    update = only_update(outputs)
    assert (update.state, update.error_code) == \
        (TaskState.CANCELLED, ErrorCode.CANCELLED_BY_OPERATOR)
    assert machine.blocked_wait_done() == []                # the timer fired anyway
    navigation = navigations(machine.assign(make_spec('t2'), battery=0.9))[0]
    assert machine.blocked_wait_done() == [] and machine.dwell_done() == []
    assert machine.nav_generation == navigation.generation and machine.replan_count == 0


def test_newer_revision_during_the_blocked_wait_restarts_without_the_wait():
    machine, _ = blocked_machine()
    navigation = navigations(machine.assign(make_spec(rev=1), battery=0.9))[0]
    report_blocked(machine, navigation)
    outputs = machine.assign(make_spec(rev=2, location_ids=('lab_2',)), battery=0.9)
    assert of_type(outputs, CancelNavigation)
    restart = navigations(outputs)[0]
    assert restart.goal == LOCATIONS['lab_2'] and only_update(outputs).assignment_rev == 2
    assert machine.blocked_wait_done() == [] and machine.nav_generation == restart.generation


def test_stale_nav_results_are_ignored_around_the_blocked_wait():
    machine, _ = blocked_machine()
    first = navigations(machine.assign(make_spec(), battery=0.9))[0]
    report_blocked(machine, first)
    assert machine.nav_result(first.generation, succeeded=True) == []      # duplicate
    assert report_blocked(machine, first) == []
    assert machine.nav_feedback(1.0) == []
    retry = navigations(machine.blocked_wait_done())[0]
    assert machine.nav_result(first.generation, succeeded=True) == []
    assert machine.nav_generation == retry.generation and machine.active_task_id == 't1'
    update = only_update(machine.nav_result(retry.generation, succeeded=True))
    assert update.state == TaskState.COMPLETED


def test_blocked_wait_and_delivery_dwell_are_not_confused():
    machine, _ = blocked_machine(dwell_sec=2.0)
    spec = make_spec(task_type=TaskType.DELIVERY, location_ids=('storage', 'lab_1'))
    pickup = navigations(machine.assign(spec, battery=0.9))[0]
    assert of_type(report_blocked(machine, pickup), Wait) == [Wait(5.0, blocked=True)]
    assert machine.dwell_done() == []                       # no dwell due: the leg stays
    retry = navigations(machine.blocked_wait_done())[0]
    assert retry.goal == LOCATIONS['storage'] and retry.leg_index == 0
    outputs = machine.nav_result(retry.generation, succeeded=True)
    assert of_type(outputs, Wait) == [Wait(2.0)]            # loading, not blocked
    assert machine.blocked_wait_done() == []                # no blocked wait due: no retry
    outputs = machine.dwell_done()
    assert navigations(outputs)[0].goal == LOCATIONS['lab_1']
    assert only_update(outputs).leg_index == 1


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
