"""Harness self-tests, tier 1b: scenario file validation.
Bad scenarios must be rejected with actionable errors before anything runs.
"""
import pytest

from runner import ScenarioError, validate

GOOD = {
    'name': 'ok', 'topics': {'/t': 'std_msgs/msg/String'},
    'actions': [{'t': 0, 'run': {'pkg': 'a', 'exec': 'b'}}],
    'assert': [{'publishes': {'topic': '/t'}}],
    'window_sec': 5,
}


def test_valid_scenario_passes():
    assert validate(dict(GOOD)) is not None


def test_empty_scenario_file():
    # empty YAML file -> safe_load returns None, must be a clean
    # ScenarioError, not a raw TypeError
    with pytest.raises(ScenarioError, match='not a mapping'):
        validate(None)
    with pytest.raises(ScenarioError, match='not a mapping'):
        validate(['not', 'a', 'mapping'])


def test_missing_key():
    scenario = dict(GOOD)
    del scenario['window_sec']
    with pytest.raises(ScenarioError, match='window_sec'):
        validate(scenario)


def test_unknown_assertion_type():
    scenario = dict(GOOD)
    scenario['assert'] = [{'teleports_to': {'x': 1}}]
    with pytest.raises(ScenarioError, match="exactly one type"):
        validate(scenario)


def test_assert_with_two_types():
    scenario = dict(GOOD)
    scenario['assert'] = [{'publishes': {'topic': '/t'}, 'min_rate': {'topic': '/t', 'hz': 1}}]
    with pytest.raises(ScenarioError, match='exactly one type'):
        validate(scenario)


def test_unknown_action_type():
    scenario = dict(GOOD)
    scenario['actions'] = [{'t': 0, 'teleport': {'x': 1}}]
    with pytest.raises(ScenarioError, match="'run', 'launch', 'publish'"):
        validate(scenario)


def test_action_without_t():
    scenario = dict(GOOD)
    scenario['actions'] = [{'run': {'pkg': 'a', 'exec': 'b'}}]
    with pytest.raises(ScenarioError, match="'t'"):
        validate(scenario)
