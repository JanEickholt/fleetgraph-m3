"""Robot selection for a queued task: the mock Module 1 allocator.
Pure Python (no rclpy), unit-tested in tests/unit/test_allocation.py.

Input: the latest RobotState per robot as RobotSnapshot, plus what Module 1
knows itself (the task it assigned and has not seen finish). Distance is the
straight line from the robot's localized pose to the task's first location.

Policies:
  distance  baseline: the nearest robot that holds no assignment, whatever
            its status, battery, capabilities or heartbeat age.
  context   also excludes MAINTENANCE / ERROR robots, stale heartbeats,
            missing capabilities and battery below a minimum; ranks robots
            reporting idle before robots reporting busy, then by distance.

A robot holding an assignment is never a candidate (one active task per
robot): the task waits in Module 1's queue until some robot is free, so the
choice is made when a robot can start, not earlier.
"""
import math
from dataclasses import dataclass

POLICIES = ('distance', 'context')


class RobotStatus:
    """Mirror of RobotState.STATUS_* (checked against the .msg in tests)."""
    AVAILABLE = 0
    BUSY = 1
    MAINTENANCE = 2
    ERROR = 4


@dataclass(frozen=True)
class RobotSnapshot:
    robot_id: str
    position: tuple                 # (x, y) in map; None = not localized
    status: int = RobotStatus.AVAILABLE
    battery: float = 1.0            # 0..1, NaN = unknown
    capabilities: frozenset = frozenset()
    current_task_id: str = ''       # as reported by the robot
    heard_at: float = 0.0           # receiver clock (s) when this state arrived
    assigned_task_id: str = ''      # Module 1's assignment not yet finished


@dataclass(frozen=True)
class Allocation:
    robot_id: str                   # None = no eligible robot now
    ranking: tuple                  # ((robot_id, distance_m), ...) best first
    excluded: dict                  # robot_id -> reason


def allocate(policy, robots, target, required_capabilities=(), now=0.0,
             min_battery=0.2, heartbeat_timeout_sec=3.0):
    """Choose a robot for a task whose first location is `target` (x, y),
    None when Module 1 cannot place it (then distance does not count)."""
    if policy not in POLICIES:
        raise ValueError(f'unknown allocation policy {policy!r}, expected one of {POLICIES}')
    required = set(required_capabilities)
    eligible, excluded = [], {}
    for robot in robots:
        reason = _exclusion(robot, policy, required, now, min_battery, heartbeat_timeout_sec)
        if reason:
            excluded[robot.robot_id] = reason
        else:
            eligible.append(robot)

    def distance(robot):
        return 0.0 if target is None else math.dist(robot.position, target)

    def rank(robot):
        reported_busy = policy == 'context' and not _idle(robot)    # False sorts first
        return reported_busy, distance(robot), robot.robot_id

    ranking = tuple((robot.robot_id, round(distance(robot), 2))
                    for robot in sorted(eligible, key=rank))
    return Allocation(ranking[0][0] if ranking else None, ranking, excluded)


def _idle(robot):
    return robot.status == RobotStatus.AVAILABLE and not robot.current_task_id


def _exclusion(robot, policy, required, now, min_battery, heartbeat_timeout_sec):
    """Why the robot cannot take the task now; '' when it can. Reasons are
    stable texts (no live numbers) so a waiting task is logged once per change."""
    if robot.assigned_task_id:
        return f'busy with {robot.assigned_task_id}'
    if robot.position is None:
        return 'not localized'
    if policy == 'distance':
        return ''
    if robot.status == RobotStatus.MAINTENANCE:
        return 'maintenance'
    if robot.status == RobotStatus.ERROR:
        return 'error'
    if now - robot.heard_at > heartbeat_timeout_sec:
        return 'heartbeat stale'
    missing = required - robot.capabilities
    if missing:
        return f'missing capabilities {sorted(missing)}'
    if not robot.battery >= min_battery:              # NaN (unknown) fails too
        return f'battery below {min_battery:.2f}'
    return ''
