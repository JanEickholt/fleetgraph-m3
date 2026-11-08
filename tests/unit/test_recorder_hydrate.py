"""Scenario `publish` actions build their message with recorder.hydrate:
nested messages from dicts, message sequences from lists of dicts and
builtin Time from seconds or {sec, nanosec}. Skips without a built ROS
workspace."""
import pytest

pytest.importorskip('rclpy')
messages = pytest.importorskip('fleetgraph_interfaces.msg')

from builtin_interfaces.msg import Time  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402

from recorder import hydrate  # noqa: E402


def test_flat_and_nested_fields():
    twist = hydrate(Twist, {'linear': {'x': 0.2}, 'angular': {'z': -0.5}})
    assert (twist.linear.x, twist.angular.z) == (0.2, -0.5)
    assert hydrate(messages.SceneObservation, {'robot_id': 'robot_1'}).robot_id == 'robot_1'


def test_lists_of_dicts_become_message_sequences():
    msg = hydrate(messages.SceneObservation, {
        'robot_id': 'robot_1', 'frame_id': 'map',
        'entities': [{'id': 'robot_1/obstacle/1', 'category': 2,
                      'position': {'x': 3.0, 'y': 0.5},
                      'attributes': [{'key': 'persistence', 'value': 'temporary'}]},
                     {'id': 'robot_1/person/2', 'category': 3}],
        'relations': [{'subject_id': 'robot_1/person/2', 'predicate': 1,
                       'object_id': 'robot_1/obstacle/1', 'confidence': 0.5}],
        'removed_entity_ids': ['robot_1/obstacle/9']})
    crate, walker = msg.entities
    assert isinstance(crate, messages.SceneEntity) and isinstance(walker, messages.SceneEntity)
    assert (crate.id, crate.category, crate.position.x, crate.position.y) == \
        ('robot_1/obstacle/1', 2, 3.0, 0.5)
    assert [(pair.key, pair.value) for pair in crate.attributes] == [('persistence', 'temporary')]
    assert walker.id == 'robot_1/person/2'
    assert isinstance(msg.relations[0], messages.SceneRelation)
    assert msg.relations[0].object_id == 'robot_1/obstacle/1'
    assert list(msg.removed_entity_ids) == ['robot_1/obstacle/9']


def test_time_fields_from_seconds_or_sec_nanosec():
    msg = hydrate(messages.SceneObservation, {
        'meta': {'stamp': 12.5},
        'entities': [{'first_seen': {'sec': 3, 'nanosec': 250_000_000}, 'last_seen': 4}]})
    assert msg.meta.stamp == Time(sec=12, nanosec=500_000_000)
    assert msg.entities[0].first_seen == Time(sec=3, nanosec=250_000_000)
    assert msg.entities[0].last_seen == Time(sec=4)
