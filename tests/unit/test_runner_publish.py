"""Scenario `publish` actions start at their action's t, the time the runner
orders actions by; a different t inside the publish spec is a scenario
error, not a guess. The publishers run on a fake node: no ROS graph."""
import glob
import os

import pytest

from recorder import Recorder
from runner import ScenarioError, _publishers, load, validate

SCENARIOS = sorted(glob.glob(os.path.join(os.path.dirname(__file__), '..', 'scenarios',
                                          '*.yaml')))
SPEC = {'topic': '/chatter', 'type': 'std_msgs/msg/String', 'duration': 2, 'hz': 2}


def scenario(*actions):
    return {'name': 'publish', 'topics': {'/chatter': 'std_msgs/msg/String'},
            'actions': list(actions), 'assert': [{'publishes': {'topic': '/chatter'}}],
            'window_sec': 10}


class FakeNode:
    """Recorder.make_publisher on a node that logs (t, data) per publish."""
    make_publisher = Recorder.make_publisher

    def __init__(self):
        self.now = 0.0
        self.sent = []

    def create_publisher(self, msg_class, topic, depth):
        node = self

        class Publisher:
            def publish(self, msg):
                node.sent.append((node.now, msg.data))
        return Publisher()


def publish_times(actions, window_sec=10.0, step=0.25):
    """{data: [t, ...]} over a window ticked like the runner's loop."""
    node = FakeNode()
    publishers = _publishers(node, validate(scenario(*actions))['actions'])
    for index in range(round(window_sec / step) + 1):
        node.now = index * step
        for publisher in publishers:
            publisher.tick(node.now)
    times = {}
    for t, data in node.sent:
        times.setdefault(data, []).append(t)
    return times


def test_each_publish_action_starts_at_its_own_t():
    # before the fix both started at 0: the specs carry no t of their own
    times = publish_times([{'t': 5, 'publish': dict(SPEC, msg={'data': 'late'})},
                           {'t': 0, 'run': {'pkg': 'a', 'exec': 'b'}},
                           {'t': 1.5, 'publish': dict(SPEC, msg={'data': 'early'})}])
    assert times == {'late': [5.0, 5.5, 6.0, 6.5, 7.0], 'early': [1.5, 2.0, 2.5, 3.0, 3.5]}


def test_an_inner_t_equal_to_the_action_t_is_redundant():
    assert publish_times([{'t': 5, 'publish': dict(SPEC, t=5.0, msg={'data': 'x'})}]) == \
        {'x': [5.0, 5.5, 6.0, 6.5, 7.0]}


def test_an_inner_t_that_disagrees_is_rejected():
    with pytest.raises(ScenarioError, match=r"t=5 also says t=2 inside 'publish'"):
        validate(scenario({'t': 5, 'publish': dict(SPEC, t=2)}))


def test_shipped_scenarios_are_valid():
    # one test, not one per file: ids named after scenarios would join
    # `./t -k <scenario>` selections; a ScenarioError names the file
    assert SCENARIOS
    for path in SCENARIOS:
        load(path)
