"""Scripted cancels in the mock Module 1 (mock_orchestrator): a task still
waiting for a robot or queued behind another task is cancelled by the mock
itself (before: only a TaskCommand nobody acted on went out, and the task
later ran to COMPLETED); only an assigned task gets a TaskCommand. The node
runs unspun with its publishers captured. Skips without a built ROS
workspace."""
import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('fleetgraph_interfaces.msg')

from fleetgraph_interfaces.msg import TaskCommand, TaskUpdate  # noqa: E402

from fleetgraph_robot.allocation import RobotSnapshot, RobotStatus  # noqa: E402
from fleetgraph_robot.mock_orchestrator import MockOrchestrator  # noqa: E402

# outside the runner's scenario domains (2-99 without the container's)
TEST_DOMAIN_ID = 1


class Captured:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)


@pytest.fixture
def mock(tmp_path):
    script = tmp_path / 'script.yaml'
    script.write_text('robot_id: robot_1\ntasks: []\n')
    rclpy.init(args=['--ros-args', '-p', f'scenario_file:={script}'], domain_id=TEST_DOMAIN_ID)
    node = MockOrchestrator()
    node.pub_update, node.pub_assignment, node.pub_command = Captured(), Captured(), Captured()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def receive(node, task_id, robot_id='robot_1'):
    node._receive({'t': 0, 'task_id': task_id, 'type': 'go_to', 'locations': ['lab_1'],
                   'robot_id': robot_id})


def updates(node, task_id):
    return [(msg.state, msg.error_code, msg.meta.source_id)
            for msg in node.pub_update.sent if msg.task_id == task_id]


CANCELLED = (TaskUpdate.STATE_CANCELLED, TaskUpdate.ERROR_CANCELLED_BY_OPERATOR, 'm1_mock')


def test_a_task_waiting_for_allocation_is_cancelled_by_the_mock(mock):
    receive(mock, 'auto-1', robot_id='auto')
    mock._send_command({'task_id': 'auto-1', 'reason': 'operator cancel'})
    assert updates(mock, 'auto-1')[-1] == CANCELLED
    assert mock.pub_update.sent[-1].reason == 'operator cancel'
    assert mock.pub_command.sent == [] and mock.waiting == []
    assert mock.tasks['auto-1']['state'] == TaskUpdate.STATE_CANCELLED
    # a robot that becomes eligible later gets nothing
    mock.robots['robot_2'] = RobotSnapshot('robot_2', (2.8, -2.3), status=RobotStatus.AVAILABLE,
                                           battery=1.0, capabilities=frozenset({'navigate'}),
                                           heard_at=mock.now_sec())
    mock._allocate_waiting(mock.now_sec())
    assert mock.pub_assignment.sent == []


def test_a_task_queued_behind_another_is_cancelled_by_the_mock(mock):
    receive(mock, 'first')
    receive(mock, 'second')
    mock._tick()                                     # assigns first; second waits behind it
    assert [msg.task.task_id for msg in mock.pub_assignment.sent] == ['first']
    mock._send_command({'task_id': 'second'})
    assert updates(mock, 'second')[-1] == CANCELLED
    assert mock.pub_command.sent == [] and mock.queues['robot_1'] == []
    del mock.in_flight['robot_1']                    # first finished
    mock._tick()
    assert [msg.task.task_id for msg in mock.pub_assignment.sent] == ['first']


def test_only_an_assigned_task_gets_a_task_command(mock):
    receive(mock, 'first')
    mock._tick()
    mock._send_command({'task_id': 'first', 'reason': 'operator cancel'})
    (command,) = mock.pub_command.sent
    assert (command.robot_id, command.task_id, command.command, command.reason) == \
        ('robot_1', 'first', TaskCommand.COMMAND_CANCEL, 'operator cancel')
    assert CANCELLED not in updates(mock, 'first')   # the robot reports its own CANCELLED
    mock._send_command({'task_id': 'never-received'})
    assert len(mock.pub_command.sent) == 1 and updates(mock, 'never-received') == []
