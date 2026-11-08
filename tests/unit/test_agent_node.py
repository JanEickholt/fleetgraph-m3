"""Agent callbacks around Nav2 goals (fleetgraph_robot.agent_node), run on a
stand-in node: a goal superseded before Nav2 accepted it is cancelled, the
startup poll for the Nav2 server destroys its timers, and a goal that ended
leaves no handle to cancel. Skips without a built ROS workspace."""
from types import SimpleNamespace

import pytest

pytest.importorskip('rclpy')
pytest.importorskip('nav2_msgs.action')
pytest.importorskip('fleetgraph_interfaces.msg')

from action_msgs.msg import GoalStatus  # noqa: E402
from builtin_interfaces.msg import Time  # noqa: E402

from fleetgraph_robot.agent_node import Agent  # noqa: E402
from fleetgraph_robot.task_machine import (  # noqa: E402
    Goal, Navigate, TaskMachine, TaskSpec, TaskState, TaskType, Update)

LAB_1 = Goal(6.0, 1.2, 0.0, 'lab_1')


class Future:
    """rclpy Future as the agent uses it: result() and done callbacks."""

    def __init__(self, result=None):
        self._result = result
        self.callbacks = []

    def result(self):
        return self._result

    def add_done_callback(self, callback):
        self.callbacks.append(callback)

    def finish(self, result):
        self._result = result
        for callback in self.callbacks:
            callback(self)


class GoalHandle:
    def __init__(self, accepted=True):
        self.accepted = accepted
        self.cancel_requests = 0
        self.result_future = Future()

    def cancel_goal_async(self):
        self.cancel_requests += 1

    def get_result_async(self):
        return self.result_future


class NavClient:
    def __init__(self, ready_after_polls=0):
        self.polls_left = ready_after_polls
        self.sent = []                      # (goal, goal-response future)

    def server_is_ready(self):
        if self.polls_left:
            self.polls_left -= 1
            return False
        return True

    def send_goal_async(self, goal, feedback_callback=None):
        future = Future()
        self.sent.append((goal, future))
        return future


class Timer:
    def __init__(self, callback):
        self.callback = callback


class StandIn:
    """What the agent's Nav2 callbacks touch of the node."""
    _navigate = Agent._navigate
    _on_goal_response = Agent._on_goal_response
    _on_result = Agent._on_result

    def __init__(self, nav_client=None):
        self.machine = TaskMachine(['navigate'], resolve={'lab_1': LAB_1}.get)
        self.params = {'map_frame': 'map', 'nav_server_timeout_sec': 30.0}
        self.nav_client = nav_client or NavClient()
        self._goal_handle = None
        self.dispatched = []
        self.timers = []
        self.destroyed = []

    def _dispatch(self, outputs):
        self.dispatched += outputs

    def create_timer(self, period, callback):
        timer = Timer(callback)
        self.timers.append(timer)
        return timer

    def destroy_timer(self, timer):
        self.destroyed.append(timer)

    def get_logger(self):
        return SimpleNamespace(info=lambda *_, **__: None, warn=lambda *_, **__: None)

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=Time))


def start_go_to(node):
    """Assign a go_to lab_1; returns its Navigate."""
    outputs = node.machine.assign(TaskSpec('t1', TaskType.GO_TO, [('lab_1', None)]), 1.0)
    return next(output for output in outputs if isinstance(output, Navigate))


def succeeded():
    return SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED, result=None)


def test_a_goal_superseded_before_nav2_accepted_it_is_cancelled_and_ignored():
    node = StandIn()
    navigate = start_go_to(node)
    node.machine.cancel('t1', 'operator')                   # nav_generation is 0 now
    handle = GoalHandle(accepted=True)
    node._on_goal_response(navigate.generation, Future(handle))
    assert handle.cancel_requests == 1
    assert node._goal_handle is None and node.dispatched == []
    assert handle.result_future.callbacks == []             # its result is never awaited


def test_a_superseded_goal_that_nav2_rejected_is_ignored():
    node = StandIn()
    navigate = start_go_to(node)
    node.machine.cancel('t1', 'operator')
    handle = GoalHandle(accepted=False)
    node._on_goal_response(navigate.generation, Future(handle))
    assert handle.cancel_requests == 0 and node.dispatched == []


def test_the_current_goal_is_kept_until_its_result_arrives():
    node = StandIn()
    navigate = start_go_to(node)
    handle = GoalHandle(accepted=True)
    node._on_goal_response(navigate.generation, Future(handle))
    assert node._goal_handle is handle and handle.cancel_requests == 0
    handle.result_future.finish(succeeded())
    assert node._goal_handle is None                        # nothing left to cancel
    (update,) = [output for output in node.dispatched if isinstance(output, Update)]
    assert update.state == TaskState.COMPLETED


def test_an_older_goal_ending_leaves_the_newer_handle():
    node = StandIn()
    navigate = start_go_to(node)
    older = GoalHandle(accepted=True)
    node._on_goal_response(navigate.generation, Future(older))
    newer = GoalHandle(accepted=True)
    node._goal_handle = newer                               # a later goal took over
    older.result_future.finish(succeeded())
    assert node._goal_handle is newer


def test_the_startup_poll_for_the_nav2_server_destroys_each_timer():
    node = StandIn(NavClient(ready_after_polls=2))
    navigate = start_go_to(node)
    node._navigate(navigate)
    assert len(node.timers) == 1 and node.destroyed == []
    node.timers[0].callback()                               # still not ready: polls again
    assert node.destroyed == node.timers[:1] and len(node.timers) == 2
    node.timers[1].callback()                               # ready: the goal goes out
    assert node.destroyed == node.timers and len(node.timers) == 2
    (goal, _), = node.nav_client.sent
    assert (goal.pose.pose.position.x, goal.pose.pose.position.y) == (6.0, 1.2)
