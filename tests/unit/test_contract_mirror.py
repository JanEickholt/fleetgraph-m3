"""Pure-Python mirrors of contract constants (task_machine TaskState,
ErrorCode, TaskType; scene_graph Category, Predicate; harness TASK_STATES)
must equal the generated fleetgraph_interfaces messages.
Skips when the workspace is not built (no generated messages)."""
import pytest

import assertions
from fleetgraph_robot import scene_graph, task_machine

messages = pytest.importorskip('fleetgraph_interfaces.msg')


def test_task_states_mirror():
    for name, value in assertions.TASK_STATES.items():
        assert getattr(messages.TaskUpdate, f'STATE_{name}') == value, name
    for name in ('EXECUTING', 'COMPLETED', 'FAILED', 'CANCELLED', 'REJECTED'):
        assert getattr(task_machine.TaskState, name) == \
            getattr(messages.TaskUpdate, f'STATE_{name}')


def test_error_codes_mirror():
    mirrored = {name: value for name, value in vars(task_machine.ErrorCode).items()
                if name.isupper()}
    assert 'BLOCKED' in mirrored                    # a leg blocked for max_blocked_sec
    for name, value in mirrored.items():
        assert getattr(messages.TaskUpdate, f'ERROR_{name}') == value, name


def test_task_types_mirror():
    for name, value in vars(task_machine.TaskType).items():
        if name.isupper():
            assert getattr(messages.TaskRequest, f'TYPE_{name}') == value, name


def msg_constants(message, prefix):
    return {name.removeprefix(prefix): getattr(message, name)
            for name in dir(message) if name.startswith(prefix)}


def test_scene_categories_mirror():
    assert msg_constants(messages.SceneEntity, 'CATEGORY_') == \
        {category.name: category.value for category in scene_graph.Category}


def test_scene_predicates_mirror():
    assert msg_constants(messages.SceneRelation, 'PREDICATE_') == \
        {predicate.name: predicate.value for predicate in scene_graph.Predicate}
