"""Recording QoS per topic: contract topics and their Module 2 echoes
(/mock_m2/*) come in bursts and are recorded reliable with a deep queue (a
best-effort depth-5 subscriber lost events), latched contract topics also
transient local, sensors best effort. Skips without rclpy."""
import pytest

pytest.importorskip('rclpy')

from rclpy.qos import DurabilityPolicy, ReliabilityPolicy  # noqa: E402

from recorder import qos_for  # noqa: E402


def test_contract_topics_and_their_module_2_echoes_are_reliable_with_a_deep_queue():
    for topic in ('/fleet/robot_state', '/fleet/scene_observation', '/mock_m2/robot_state',
                  '/mock_m2/task_update'):
        qos = qos_for(topic)
        assert qos.reliability == ReliabilityPolicy.RELIABLE, topic
        assert qos.durability == DurabilityPolicy.VOLATILE, topic    # rosbridge: volatile
        assert qos.depth >= 1000, topic


def test_latched_contract_topics_are_transient_local():
    for topic in ('/fleet/task_update', '/fleet/robot_connection', '/robot_3/central/scene'):
        qos = qos_for(topic)
        assert (qos.reliability, qos.durability) == (ReliabilityPolicy.RELIABLE,
                                                     DurabilityPolicy.TRANSIENT_LOCAL), topic


def test_sensor_topics_are_best_effort():
    for topic in ('/robot_1/scan', '/sim/truth', '/robot_1/perception/detections',
                  '/ground_truth/robot_1/odom'):
        assert qos_for(topic).reliability == ReliabilityPolicy.BEST_EFFORT, topic
