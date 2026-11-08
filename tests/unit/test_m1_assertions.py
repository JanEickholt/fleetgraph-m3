"""Assertions for the Module 1 / Module 2 link scenarios (no_task_update,
always_value) over canned recordings: PASS and FAIL paths, honest messages."""
import math
from types import SimpleNamespace

import assertions

STATES = assertions.TASK_STATES


class FakeRecording:
    def __init__(self, streams):
        self.streams = streams

    def stream(self, topic):
        return self.streams.get(topic, [])

    def duration(self):
        return 100.0


def update(timestamp, task_id, state, robot_id='', source='m1_mock'):
    return (timestamp, SimpleNamespace(task_id=task_id, state=STATES[state], robot_id=robot_id,
                                       error_code=0, meta=SimpleNamespace(source_id=source)))


UPDATES = FakeRecording({'/fleet/task_update': [
    update(1, 't1', 'RECEIVED'),
    update(1, 't1', 'ASSIGNED', robot_id='robot_2'),
    update(2, 't1', 'EXECUTING', robot_id='robot_2', source='robot_2'),
    update(3, 't2', 'ASSIGNED', robot_id='robot_3'),
    update(4, 't2', 'REJECTED', robot_id='robot_3', source='robot_3'),
]})


def test_no_task_update_pass():
    assert assertions.no_task_update(UPDATES, robot_id='robot_1') is None
    assert assertions.no_task_update(UPDATES, source='robot_1') is None
    assert assertions.no_task_update(UPDATES, robot_id='robot_3', state='EXECUTING') is None
    assert assertions.no_task_update(UPDATES, task_id='t3') is None


def test_no_task_update_fail_names_the_events():
    failure = assertions.no_task_update(UPDATES, robot_id='robot_3')
    assert failure.startswith('2 matching')
    assert 't2 ASSIGNED t=3.0s' in failure and 't2 REJECTED t=4.0s' in failure
    failure = assertions.no_task_update(UPDATES, source='robot_2', state='EXECUTING')
    assert 't1 EXECUTING t=2.0s' in failure
    failure = assertions.no_task_update(UPDATES, task_id='t2', state='ASSIGNED')
    assert failure == '1 matching TaskUpdates on /fleet/task_update: t2 ASSIGNED t=3.0s'


def test_no_task_update_fails_on_an_empty_recording():
    # absence on a topic that recorded nothing proves nothing
    failure = assertions.no_task_update(FakeRecording({}), robot_id='robot_3')
    assert 'no messages on /fleet/task_update' in failure


def robot_state(timestamp, source, status):
    return (timestamp, SimpleNamespace(status=status, meta=SimpleNamespace(source_id=source)))


def test_always_value_pass_per_source():
    recording = FakeRecording({'/fleet/robot_state': [
        robot_state(0.5, 'robot_3', 2), robot_state(0.6, 'robot_1', 0),
        robot_state(1.0, 'robot_3', 2)]})
    assert assertions.always_value(recording, topic='/fleet/robot_state', field='status',
                                   equals=2, source='robot_3') is None


def test_always_value_fail_reports_first_violation():
    recording = FakeRecording({'/fleet/robot_state': [
        robot_state(0.5, 'robot_3', 2), robot_state(1.0, 'robot_3', 0),
        robot_state(1.5, 'robot_3', 1)]})
    failure = assertions.always_value(recording, topic='/fleet/robot_state', field='status',
                                      equals=2, source='robot_3')
    assert 'from robot_3' in failure and '= 0 at t=1.0s' in failure
    nan = FakeRecording({'/fleet/robot_state': [robot_state(0.5, 'robot_3', math.nan)]})
    assert assertions.always_value(nan, topic='/fleet/robot_state', field='status', equals=2)


def test_always_value_fails_without_messages_from_the_source():
    recording = FakeRecording({'/fleet/robot_state': [robot_state(0.5, 'robot_1', 0)]})
    failure = assertions.always_value(recording, topic='/fleet/robot_state', field='status',
                                      equals=2, source='robot_3')
    assert 'no messages on /fleet/robot_state from robot_3' in failure
