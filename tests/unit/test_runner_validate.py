"""Harness self-tests, tier 1b: scenario file validation.
Bad scenarios must be rejected with actionable errors before anything runs.
"""
import glob
import inspect
import os

import pytest

import assertions
from runner import ScenarioError, load, validate

SCENARIOS = sorted(glob.glob(os.path.join(os.path.dirname(__file__), '..', 'scenarios',
                                          '*.yaml')))

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


def test_misspelled_assertion_parameter_is_rejected():
    # before: ignored, so the per-robot filter silently fell back to "any source"
    scenario = dict(GOOD, **{'assert': [{'min_rate': {'topic': '/t', 'hz': 2.0,
                                                      'sorce': 'robot_2'}}]})
    with pytest.raises(ScenarioError, match=r"min_rate: unknown parameter\(s\) \['sorce'\]") \
            as error:
        validate(scenario)
    assert "'source'" in str(error.value)          # names what it takes


def test_parameter_the_assertion_does_not_take_is_rejected():
    # a window on an assertion without one would be ignored: the check would
    # silently cover the whole recording
    scenario = dict(GOOD, **{'assert': [{'publishes': {'topic': '/t', 'from_sec': 10}}]})
    with pytest.raises(ScenarioError, match=r"unknown parameter\(s\) \['from_sec'\]"):
        validate(scenario)


def test_missing_required_parameter_is_rejected_before_the_run():
    scenario = dict(GOOD, **{'assert': [{'min_rate': {'topic': '/t'}}]})
    with pytest.raises(ScenarioError, match=r"missing required parameter\(s\) \['hz'\]"):
        validate(scenario)


def test_parameters_must_be_a_mapping():
    scenario = dict(GOOD, **{'assert': [{'publishes': '/t'}]})
    with pytest.raises(ScenarioError, match='must be a mapping'):
        validate(scenario)


def test_key_beside_the_assertion_type_is_rejected():
    # `source:` one indent too far left belongs to the entry, not to min_rate
    scenario = dict(GOOD, **{'assert': [{'min_rate': {'topic': '/t', 'hz': 2.0},
                                         'source': 'robot_2'}]})
    with pytest.raises(ScenarioError, match='nothing else'):
        validate(scenario)


def test_assertion_signatures_are_their_whole_vocabulary():
    # a catch-all **kwargs (or *args) would accept any misspelled key again
    for name, function in assertions.REGISTRY.items():
        kinds = {parameter.kind for parameter in inspect.signature(function).parameters.values()}
        assert not kinds & {inspect.Parameter.VAR_KEYWORD, inspect.Parameter.VAR_POSITIONAL}, name


def test_shipped_scenarios_are_valid():
    # one test, not one per file: ids named after scenarios would join
    # `./t -k <scenario>` selections; a ScenarioError names the file
    assert SCENARIOS
    for path in SCENARIOS:
        load(path)


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


def test_duplicate_process_tags():
    # one tag = one /tmp/harness_<tag>.log: two agents would clobber it
    agent = {'pkg': 'fleetgraph_robot', 'exec': 'agent'}
    scenario = dict(GOOD, actions=[{'t': 1, 'run': dict(agent)}, {'t': 1, 'run': dict(agent)}])
    with pytest.raises(ScenarioError, match="tag 'agent'"):
        validate(scenario)
    scenario['actions'][1]['run']['tag'] = 'agent_robot_2'
    assert validate(scenario) is not None
    scenario = dict(GOOD, sim={'launch': 'fleetgraph_sim sim.launch.py'},
                    actions=[{'t': 0, 'run': dict(agent, tag='sim')}])
    with pytest.raises(ScenarioError, match="tag 'sim'"):
        validate(scenario)
