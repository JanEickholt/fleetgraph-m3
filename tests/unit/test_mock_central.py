"""JSONL record the mock central scene graph writes per published update,
with the central update latency the evaluation needs. Skips without a
built ROS workspace."""
import pytest

pytest.importorskip('rclpy')
pytest.importorskip('fleetgraph_interfaces.msg')

from fleetgraph_robot.central_graph import GraphUpdate  # noqa: E402
from fleetgraph_robot.mock_central_node import update_record  # noqa: E402
from fleetgraph_robot.scene_graph import Category, Entity  # noqa: E402

CRATE = Entity('g/obstacle/1', Category.OBSTACLE, 'crate', 3.1049, 0.5, confidence=0.92,
               source_ids=['robot_1', 'robot_2'], attributes={'persistence': 'temporary'})


def test_a_delta_record_names_its_cause_its_entities_and_the_central_update_latency():
    update = GraphUpdate(5, 4, False, 'robot_2', [], [CRATE], ['g/person/3'], [], [])
    assert update_record(update, stamp=12.75, seq=6, observation_stamp=12.5) == {
        'stamp': 12.75, 'seq': 6, 'graph_version': 5, 'base_version': 4,
        'is_snapshot': False, 'source_robot_id': 'robot_2', 'added': [],
        'updated': [{'id': 'g/obstacle/1', 'category': 'OBSTACLE', 'x': 3.105, 'y': 0.5,
                     'confidence': 0.92, 'sources': ['robot_1', 'robot_2'],
                     'attributes': {'persistence': 'temporary'}}],
        'removed': ['g/person/3'], 'relations_added': 0, 'relations_removed': 0,
        'latency_sec': 0.25}


def test_latency_is_unknown_without_a_stamped_observation_behind_the_update():
    snapshot = GraphUpdate(5, 0, True, '', [CRATE], [], [], [], [])
    record = update_record(snapshot, stamp=20.0, seq=7, observation_stamp=None)
    assert (record['is_snapshot'], record['latency_sec'], len(record['added'])) == \
        (True, None, 1)
