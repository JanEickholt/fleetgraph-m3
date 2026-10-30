"""task_state / task_sequence assertions over canned TaskUpdate streams:
both PASS and FAIL paths, with honest failure messages."""
from types import SimpleNamespace

import assertions

STATES = assertions.TASK_STATES


class FakeRecording:
    def __init__(self, events):
        self.events = events

    def stream(self, topic):
        return self.events if topic == '/fleet/task_update' else []

    def duration(self):
        return 100.0


def event(timestamp, task_id, state, source='robot_1', code=0):
    return (timestamp, SimpleNamespace(task_id=task_id, state=STATES[state], error_code=code,
                               meta=SimpleNamespace(source_id=source)))


LIFECYCLE = FakeRecording([
    event(1, 't1', 'RECEIVED', 'm1'), event(1, 't1', 'QUEUED', 'm1'),
    event(2, 't1', 'ASSIGNED', 'm1'), event(3, 't1', 'EXECUTING'),
    event(5, 't1', 'EXECUTING'), event(30, 't1', 'COMPLETED'),
    event(4, 't2', 'REJECTED', code=201),
])


def test_task_state_pass():
    assert assertions.task_state(LIFECYCLE, task_id='t1', state='COMPLETED') is None
    assert assertions.task_state(LIFECYCLE, task_id='t2', state='REJECTED',
                                 error_code=201, source='robot_1') is None


def test_task_state_fail_wrong_code_and_source():
    failure = assertions.task_state(LIFECYCLE, task_id='t2', state='REJECTED', error_code=202)
    assert 'never reached REJECTED with code 202' in failure and '201' in failure
    assert assertions.task_state(LIFECYCLE, task_id='t1', state='COMPLETED', source='m1')


def test_task_state_fail_too_late():
    failure = assertions.task_state(LIFECYCLE, task_id='t1', state='COMPLETED', within_sec=20)
    assert 't=30.0s' in failure


def test_task_state_fail_unknown_task():
    assert 'never reached' in assertions.task_state(LIFECYCLE, task_id='t9', state='COMPLETED')


def seq_stream(pairs):
    return FakeRecording([(index, SimpleNamespace(meta=SimpleNamespace(source_id=source, seq=seq)))
                          for index, (source, seq) in enumerate(pairs)])


def test_seq_contiguous_pass_interleaved_sources_late_join():
    recording = seq_stream([('m1', 5), ('robot_1', 1), ('m1', 6), ('robot_1', 2)])
    assert assertions.seq_contiguous(recording, topic='/fleet/task_update') is None


def test_seq_contiguous_fail_gap_and_duplicate():
    gap_failure = assertions.seq_contiguous(seq_stream([('m1', 1), ('m1', 3)]),
                                    topic='/fleet/task_update')
    duplicate_failure = assertions.seq_contiguous(seq_stream([('m1', 1), ('m1', 1)]),
                                    topic='/fleet/task_update')
    assert 'gap' in gap_failure and '1 -> 3' in gap_failure
    assert 'duplicate' in duplicate_failure


def test_task_sequence_pass_with_gaps():
    assert assertions.task_sequence(
        LIFECYCLE, task_id='t1', states=['RECEIVED', 'ASSIGNED', 'COMPLETED']) is None


def test_task_sequence_fail_wrong_order():
    failure = assertions.task_sequence(LIFECYCLE, task_id='t1',
                                   states=['COMPLETED', 'EXECUTING'])
    assert 'missing from EXECUTING' in failure and 'history' in failure
