"""Robot allocation of the mock Module 1 (fleetgraph_robot.allocation).
Pure: no rclpy, no sim. Geometry of the lab floor (locations.yaml): robots at
their docks robot_1 (0, 0), robot_2 (2.8, -2.3), robot_3 (0, -1.5)."""
import math

import pytest

from fleetgraph_robot.allocation import RobotSnapshot, RobotStatus, allocate

LAB_1 = (6.0, 1.2)       # robot_1 6.12 m, robot_2 4.74 m, robot_3 6.58 m
STORAGE = (1.5, 1.5)     # robot_1 2.12 m, robot_3 3.35 m, robot_2 4.02 m
DOCK_3 = (0.0, -1.5)     # robot_3 0 m, robot_1 1.5 m, robot_2 2.92 m
NOW = 100.0


def robot(robot_id, position, **fields):
    defaults = dict(status=RobotStatus.AVAILABLE, battery=1.0,
                    capabilities=frozenset({'navigate', 'deliver', 'patrol'}), heard_at=NOW)
    return RobotSnapshot(robot_id, position, **dict(defaults, **fields))


def fleet(**robot_3_fields):
    return [robot('robot_1', (0.0, 0.0)), robot('robot_2', (2.8, -2.3)),
            robot('robot_3', (0.0, -1.5), **robot_3_fields)]


def choose(policy, robots, target, **kwargs):
    return allocate(policy, robots, target, now=NOW, **kwargs)


@pytest.mark.parametrize('policy', ['distance', 'context'])
def test_nearest_robot_to_the_first_location(policy):
    assert choose(policy, fleet(), LAB_1).robot_id == 'robot_2'
    assert choose(policy, fleet(), STORAGE).robot_id == 'robot_1'


def test_ranking_lists_eligible_robots_nearest_first_with_distances():
    result = choose('distance', fleet(), LAB_1)
    assert result.ranking == (('robot_2', 4.74), ('robot_1', 6.12), ('robot_3', 6.58))
    assert result.excluded == {}


def test_distance_baseline_ignores_maintenance_battery_capabilities_and_heartbeat():
    tired = fleet(status=RobotStatus.MAINTENANCE, battery=0.01,
                  capabilities=frozenset(), heard_at=NOW - 60.0)
    assert choose('distance', tired, DOCK_3, required_capabilities=['deliver']).robot_id == \
        'robot_3'


def test_context_excludes_maintenance_even_when_nearest():
    result = choose('context', fleet(status=RobotStatus.MAINTENANCE), DOCK_3)
    assert result.robot_id == 'robot_1'
    assert result.excluded == {'robot_3': 'maintenance'}


@pytest.mark.parametrize('robot_3_fields, reason', [
    (dict(status=RobotStatus.ERROR), 'error'),
    (dict(heard_at=NOW - 3.5), 'heartbeat stale'),
    (dict(battery=0.19), 'battery below 0.20'),
    (dict(battery=math.nan), 'battery below 0.20'),        # unknown = not enough
    (dict(capabilities=frozenset({'navigate'})), "missing capabilities ['deliver']"),
])
def test_context_exclusions(robot_3_fields, reason):
    result = choose('context', fleet(**robot_3_fields), DOCK_3,
                    required_capabilities=['deliver'])
    assert result.robot_id == 'robot_1'
    assert result.excluded == {'robot_3': reason}


def test_context_limits_are_inclusive():
    result = choose('context', fleet(heard_at=NOW - 3.0, battery=0.2), DOCK_3)
    assert result.robot_id == 'robot_3' and result.excluded == {}


def test_context_limits_are_parameters():
    result = choose('context', fleet(heard_at=NOW - 3.0, battery=0.5), DOCK_3,
                    min_battery=0.6, heartbeat_timeout_sec=2.0)
    assert result.excluded == {'robot_3': 'heartbeat stale'}
    result = choose('context', fleet(battery=0.5), DOCK_3, min_battery=0.6)
    assert result.excluded == {'robot_3': 'battery below 0.60'}


@pytest.mark.parametrize('policy', ['distance', 'context'])
def test_both_policies_skip_robots_holding_an_assignment_or_not_localized(policy):
    robots = [robot('robot_1', (0.0, 0.0), assigned_task_id='task-7'),
              robot('robot_2', None), robot('robot_3', (0.0, -1.5))]
    result = choose(policy, robots, STORAGE)
    assert result.robot_id == 'robot_3'
    assert result.excluded == {'robot_1': 'busy with task-7', 'robot_2': 'not localized'}


def test_context_prefers_idle_over_nearer_robot_reporting_busy():
    robots = [robot('robot_1', (0.0, 0.0), status=RobotStatus.BUSY, current_task_id='other'),
              robot('robot_2', (2.8, -2.3))]
    assert choose('context', robots, STORAGE).robot_id == 'robot_2'
    assert choose('distance', robots, STORAGE).robot_id == 'robot_1'


def test_no_eligible_robot_names_every_reason():
    robots = [robot('robot_1', (0.0, 0.0), assigned_task_id='task-1'),
              robot('robot_2', (2.8, -2.3), assigned_task_id='task-2'),
              robot('robot_3', (0.0, -1.5), status=RobotStatus.MAINTENANCE)]
    result = choose('context', robots, LAB_1)
    assert result.robot_id is None and result.ranking == ()
    assert result.excluded == {'robot_1': 'busy with task-1', 'robot_2': 'busy with task-2',
                               'robot_3': 'maintenance'}
    assert choose('context', [], LAB_1).robot_id is None


def test_equal_distance_ties_break_by_robot_id():
    robots = [robot('robot_b', (1.0, 0.0)), robot('robot_a', (-1.0, 0.0))]
    assert choose('distance', robots, (0.0, 0.0)).robot_id == 'robot_a'


def test_target_unknown_to_the_registry_ranks_by_robot_id():
    # the robot then rejects the task itself (UNKNOWN_LOCATION)
    result = choose('distance', fleet(), None)
    assert result.robot_id == 'robot_1'
    assert [robot_id for robot_id, _ in result.ranking] == ['robot_1', 'robot_2', 'robot_3']


def test_unknown_policy():
    with pytest.raises(ValueError, match='allocation policy'):
        choose('random', fleet(), LAB_1)


def test_status_mirror():
    messages = pytest.importorskip('fleetgraph_interfaces.msg')
    for name, value in vars(RobotStatus).items():
        if name.isupper():
            assert getattr(messages.RobotState, f'STATUS_{name}') == value, name
