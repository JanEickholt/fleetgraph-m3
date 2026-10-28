"""Robot-side task execution state machine. Pure Python (no rclpy): the ROS
node feeds it events and executes the commands it returns, so all task
logic is unit-testable without a simulator (tests/unit/test_task_machine.py).

Events in:   assign(), nav_feedback(), nav_result(), dwell_done(), cancel()
Outputs:     a list mixing Update (-> /fleet/task_update) and commands
             Navigate / CancelNavigation / Wait (-> Nav2 action, timers).

One active task at a time; Module 1 queues. A task is a list of navigation
legs: GO_TO [target], DELIVERY [pickup, dropoff] with a dwell after each,
PATROL waypoints * loops. A failed leg is retried max_leg_retries times
(counted as replanning events), then the task FAILS with
reassignment_requested so Module 1 can move it to another robot.
"""
from dataclasses import dataclass, field, replace


class S:
    """Mirror of TaskUpdate.STATE_* (checked against the .msg in tests)."""
    EXECUTING = 4
    COMPLETED = 5
    FAILED = 6
    CANCELLED = 7
    REJECTED = 9


class E:
    """Mirror of TaskUpdate.ERROR_*."""
    NONE = 0
    INVALID_TASK = 101
    ROBOT_BUSY = 200
    UNKNOWN_LOCATION = 201
    MISSING_CAPABILITY = 202
    LOW_BATTERY = 203
    ROBOT_UNAVAILABLE = 204
    NAVIGATION_FAILED = 300
    CANCELLED_BY_OPERATOR = 400


class T:
    """Mirror of TaskRequest.TYPE_*."""
    GO_TO = 1
    DELIVERY = 2
    PATROL = 3


TERMINAL = (S.COMPLETED, S.FAILED, S.CANCELLED, S.REJECTED)


@dataclass(frozen=True)
class Goal:
    x: float
    y: float
    yaw: float
    location_id: str = ''


@dataclass
class TaskSpec:
    task_id: str
    type: int
    locations: list                 # [(location_id, Goal or None)]
    required_capabilities: list = field(default_factory=list)
    patrol_loops: int = 1
    assignment_rev: int = 1


@dataclass
class Update:
    task_id: str
    assignment_rev: int
    state: int
    progress: float = -1.0
    leg_index: int = 0
    leg_count: int = 0
    distance_remaining: float = -1.0
    error_code: int = E.NONE
    reason: str = ''
    reassignment_requested: bool = False


@dataclass
class Navigate:
    goal: Goal
    leg_index: int
    generation: int                 # echo in nav_result; stale results are dropped


@dataclass
class CancelNavigation:
    pass


@dataclass
class Wait:
    seconds: float                  # call dwell_done() afterwards


@dataclass
class _Leg:
    goal: Goal
    dwell: bool


class TaskMachine:
    def __init__(self, capabilities, resolve, min_battery=0.15, dwell_sec=2.0,
                 max_leg_retries=1):
        self.capabilities = set(capabilities)
        self.resolve = resolve      # location_id -> Goal or None
        self.min_battery = min_battery
        self.dwell_sec = dwell_sec
        self.max_leg_retries = max_leg_retries
        self.replan_count = 0       # metric: retried legs over the lifetime
        self._task = None           # active TaskSpec
        self._legs = []
        self._leg = 0
        self._retries = 0
        self._leg_length = None     # first/max distance_remaining of this leg
        self._progress = 0.0
        self._distance = -1.0
        self._generation = 0        # current Navigate generation, 0 = none
        self._gen_counter = 0       # monotonic over the lifetime
        self._terminal = {}         # task_id -> last terminal Update

    # ------------------------------------------------------------ queries
    @property
    def active_task_id(self):
        return self._task.task_id if self._task else ''

    @property
    def nav_generation(self):
        """Generation of the running Navigate, 0 when none."""
        return self._generation

    # ------------------------------------------------------------- events
    def assign(self, spec, battery, available=True):
        cur = self._task
        if cur and spec.task_id == cur.task_id:
            if spec.assignment_rev < cur.assignment_rev:
                return []                                   # outdated
            if spec.assignment_rev == cur.assignment_rev:
                return [self._update(S.EXECUTING)]          # re-delivery: ack
            restart = [CancelNavigation()]                  # newer revision
            self._task = None
        elif cur:
            return [self._reject(spec, E.ROBOT_BUSY,
                                 f'busy with {cur.task_id}')]
        else:
            restart = []
            done = self._terminal.get(spec.task_id)
            if done and done.assignment_rev == spec.assignment_rev:
                return [done]                               # lost ack: repeat

        if not available:
            return restart + [self._reject(spec, E.ROBOT_UNAVAILABLE,
                                           'robot not available')]
        legs, err = self._plan(spec)
        if err:
            return restart + [self._reject(spec, *err)]
        missing = set(spec.required_capabilities) - self.capabilities
        if missing:
            return restart + [self._reject(
                spec, E.MISSING_CAPABILITY,
                f'missing capabilities: {sorted(missing)}')]
        if battery < self.min_battery:
            return restart + [self._reject(
                spec, E.LOW_BATTERY,
                f'battery {battery:.2f} < {self.min_battery:.2f}')]

        self._task, self._legs, self._leg = spec, legs, 0
        self._progress = 0.0
        return restart + self._start_leg(retry=False)

    def nav_feedback(self, distance_remaining):
        if not self._task or self._generation == 0:
            return []
        d = max(0.0, float(distance_remaining))
        if self._leg_length is None or d > self._leg_length:
            self._leg_length = d
        frac = 1.0 - d / self._leg_length if self._leg_length > 0 else 0.0
        overall = (self._leg + frac) / len(self._legs)
        self._progress = max(self._progress, min(overall, 1.0))
        self._distance = d
        return [self._update(S.EXECUTING)]

    def nav_result(self, generation, succeeded, reason=''):
        if not self._task or generation != self._generation:
            return []                                       # stale
        self._generation = 0
        if not succeeded:
            if self._retries < self.max_leg_retries:
                self.replan_count += 1
                return self._start_leg(retry=True, reason=f'replanning: {reason}')
            return [self._finish(
                S.FAILED, E.NAVIGATION_FAILED,
                f'leg {self._leg} to {self._legs[self._leg].goal.location_id} '
                f'failed: {reason}', reassign=True)]
        self._progress = max(self._progress, (self._leg + 1) / len(self._legs))
        if self._legs[self._leg].dwell:
            return [Wait(self.dwell_sec), self._update(S.EXECUTING)]
        return self._next_leg()

    def dwell_done(self):
        if not self._task or self._generation != 0:
            return []
        return self._next_leg()

    def cancel(self, task_id, reason):
        if not self._task or task_id != self._task.task_id:
            return []
        return [CancelNavigation(),
                self._finish(S.CANCELLED, E.CANCELLED_BY_OPERATOR, reason)]

    # ------------------------------------------------------------ helpers
    def _plan(self, spec):
        need = {T.GO_TO: lambda n: n == 1, T.DELIVERY: lambda n: n == 2,
                T.PATROL: lambda n: n >= 1}
        if spec.type not in need:
            return None, (E.INVALID_TASK, f'unsupported task type {spec.type}')
        if not need[spec.type](len(spec.locations)):
            return None, (E.INVALID_TASK,
                          f'wrong location count {len(spec.locations)} '
                          f'for task type {spec.type}')
        goals = []
        for lid, pose in spec.locations:
            goal = pose if pose is not None else self.resolve(lid)
            if goal is None:
                return None, (E.UNKNOWN_LOCATION, f'unknown location {lid!r}')
            goals.append(goal)
        if spec.type == T.PATROL:
            loops = max(1, int(spec.patrol_loops))
            return [_Leg(g, False) for _ in range(loops) for g in goals], None
        return [_Leg(g, spec.type == T.DELIVERY) for g in goals], None

    def _start_leg(self, retry, reason=''):
        self._retries = self._retries + 1 if retry else 0
        self._leg_length = None
        self._distance = -1.0
        self._gen_counter += 1
        self._generation = self._gen_counter
        nav = Navigate(self._legs[self._leg].goal, self._leg, self._generation)
        return [nav, self._update(S.EXECUTING, reason=reason)]

    def _next_leg(self):
        self._leg += 1
        if self._leg >= len(self._legs):
            self._progress = 1.0
            return [self._finish(S.COMPLETED, E.NONE, '')]
        return self._start_leg(retry=False)

    def _update(self, state, error_code=E.NONE, reason='', reassign=False):
        t = self._task
        return Update(t.task_id, t.assignment_rev, state,
                      progress=self._progress, leg_index=self._leg,
                      leg_count=len(self._legs),
                      distance_remaining=self._distance,
                      error_code=error_code, reason=reason,
                      reassignment_requested=reassign)

    def _finish(self, state, code, reason, reassign=False):
        u = self._update(state, code, reason, reassign)
        if state == S.COMPLETED:
            u = replace(u, leg_index=max(0, len(self._legs) - 1),
                        distance_remaining=0.0)
        self._terminal[u.task_id] = u
        self._task, self._legs, self._generation = None, [], 0
        return u

    def _reject(self, spec, code, reason):
        return Update(spec.task_id, spec.assignment_rev, S.REJECTED,
                      error_code=code, reason=reason)
