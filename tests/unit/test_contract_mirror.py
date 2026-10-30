"""Pure-Python mirrors of contract constants (task_machine TaskState,
ErrorCode, TaskType; harness TASK_STATES) must equal the generated
fleetgraph_interfaces messages.
Skips when the workspace is not built (no generated messages)."""
import pytest

import assertions
from fleetgraph_robot import task_machine

messages = pytest.importorskip('fleetgraph_interfaces.msg')


def test_task_states_mirror():
    for name, value in assertions.TASK_STATES.items():
        assert getattr(messages.TaskUpdate, f'STATE_{name}') == value, name
    for name in ('EXECUTING', 'COMPLETED', 'FAILED', 'CANCELLED', 'REJECTED'):
        assert getattr(task_machine.TaskState, name) == \
            getattr(messages.TaskUpdate, f'STATE_{name}')


def test_error_codes_mirror():
    for name, value in vars(task_machine.ErrorCode).items():
        if name.isupper():
            assert getattr(messages.TaskUpdate, f'ERROR_{name}') == value, name


def test_task_types_mirror():
    for name, value in vars(task_machine.TaskType).items():
        if name.isupper():
            assert getattr(messages.TaskRequest, f'TYPE_{name}') == value, name
