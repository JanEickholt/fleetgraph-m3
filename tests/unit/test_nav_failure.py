"""Nav2 failure classification (fleetgraph_robot.nav_failure). Pure — no rclpy,
no sim. Covers every code bt_navigator can report for NavigateToPose (FollowPath
and ComputePathToPose), no code, an unknown code, the reason text, and the code
mirror against nav2_msgs.
"""
import pytest

from fleetgraph_robot.nav_failure import (
    BLOCKED, FAILED, ComputePathError, FollowPathError, classify_failure)

CODES = [
    pytest.param(100, FAILED, id='controller UNKNOWN'),
    pytest.param(101, FAILED, id='controller INVALID_CONTROLLER'),
    pytest.param(102, FAILED, id='controller TF_ERROR'),
    pytest.param(103, FAILED, id='controller INVALID_PATH'),
    pytest.param(104, BLOCKED, id='controller PATIENCE_EXCEEDED'),
    pytest.param(105, BLOCKED, id='controller FAILED_TO_MAKE_PROGRESS'),
    pytest.param(106, BLOCKED, id='controller NO_VALID_CONTROL'),
    pytest.param(107, FAILED, id='controller CONTROLLER_TIMED_OUT'),
    pytest.param(200, FAILED, id='planner UNKNOWN'),
    pytest.param(201, FAILED, id='planner INVALID_PLANNER'),
    pytest.param(202, FAILED, id='planner TF_ERROR'),
    pytest.param(203, FAILED, id='planner START_OUTSIDE_MAP'),
    pytest.param(204, FAILED, id='planner GOAL_OUTSIDE_MAP'),
    pytest.param(205, BLOCKED, id='planner START_OCCUPIED'),
    pytest.param(206, BLOCKED, id='planner GOAL_OCCUPIED'),
    pytest.param(207, FAILED, id='planner TIMEOUT'),
    pytest.param(208, BLOCKED, id='planner NO_VALID_PATH'),
    pytest.param(0, FAILED, id='no code'),
    pytest.param(703, FAILED, id='code outside error_code_names'),   # Spin COLLISION_AHEAD
]


def mirrored(codes):
    return {name: value for name, value in vars(codes).items() if name.isupper()}


@pytest.mark.parametrize('code, kind', CODES)
def test_each_code_is_blocked_or_failed(code, kind):
    failure = classify_failure(code)
    assert failure.kind == kind and failure.blocked == (kind == BLOCKED)


def test_the_table_decides_every_mirrored_code():
    table = {param.values[0] for param in CODES}
    assert {*mirrored(FollowPathError).values(), *mirrored(ComputePathError).values()} <= table


def test_the_message_does_not_change_the_kind():
    assert classify_failure(208, 'GridBased plugin failed to plan').kind == BLOCKED
    assert classify_failure(0, 'Failed to create plan').kind == FAILED


def test_reason_names_the_code_then_adds_nav2s_message():
    assert classify_failure(208).reason == 'planner: no valid path (208)'
    assert classify_failure(105).reason == 'controller: failed to make progress (105)'
    assert classify_failure(104, 'Controller patience exceeded').reason == \
        'controller: patience exceeded (104): Controller patience exceeded'
    assert classify_failure(703).reason == 'nav2 error code 703'
    assert classify_failure(0, 'aborted').reason == 'aborted'
    assert classify_failure(0).reason == ''           # nothing to say: the caller adds the status


def test_codes_mirror_nav2_msgs():
    actions = pytest.importorskip('nav2_msgs.action')
    for codes, action in ((FollowPathError, actions.FollowPath),
                          (ComputePathError, actions.ComputePathToPose)):
        result = action.Result
        expected = {name: getattr(result, name) for name in dir(result)
                    if name.isupper() and name not in ('NONE', 'SLOT_TYPES')}
        assert mirrored(codes) == expected, action.__name__
