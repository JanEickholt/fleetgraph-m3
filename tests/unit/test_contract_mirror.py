"""Pure-Python mirrors of contract constants (task_machine S/E/T, harness
TASK_STATES) must equal the generated fleetgraph_interfaces messages.
Skips when the workspace is not built (no generated messages)."""
import pytest

import assertions
from fleetgraph_robot import task_machine as tm

msgs = pytest.importorskip('fleetgraph_interfaces.msg')


def test_task_states_mirror():
    for name, value in assertions.TASK_STATES.items():
        assert getattr(msgs.TaskUpdate, f'STATE_{name}') == value, name
    for name in ('EXECUTING', 'COMPLETED', 'FAILED', 'CANCELLED', 'REJECTED'):
        assert getattr(tm.S, name) == getattr(msgs.TaskUpdate, f'STATE_{name}')


def test_error_codes_mirror():
    for name, value in vars(tm.E).items():
        if name.isupper():
            assert getattr(msgs.TaskUpdate, f'ERROR_{name}') == value, name


def test_task_types_mirror():
    for name, value in vars(tm.T).items():
        if name.isupper():
            assert getattr(msgs.TaskRequest, f'TYPE_{name}') == value, name
