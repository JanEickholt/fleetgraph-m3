"""task_state / task_sequence assertions over canned TaskUpdate streams:
both PASS and FAIL paths, with honest failure messages."""
from types import SimpleNamespace

import assertions

S = assertions.TASK_STATES


class FakeRec:
    def __init__(self, events):
        self.events = events

    def stream(self, topic):
        return self.events if topic == '/fleet/task_update' else []

    def duration(self):
        return 100.0


def ev(t, task_id, state, source='robot_1', code=0):
    return (t, SimpleNamespace(task_id=task_id, state=S[state], error_code=code,
                               meta=SimpleNamespace(source_id=source)))


LIFE = FakeRec([
    ev(1, 't1', 'RECEIVED', 'm1'), ev(1, 't1', 'QUEUED', 'm1'),
    ev(2, 't1', 'ASSIGNED', 'm1'), ev(3, 't1', 'EXECUTING'),
    ev(5, 't1', 'EXECUTING'), ev(30, 't1', 'COMPLETED'),
    ev(4, 't2', 'REJECTED', code=201),
])


def test_task_state_pass():
    assert assertions.task_state(LIFE, task_id='t1', state='COMPLETED') is None
    assert assertions.task_state(LIFE, task_id='t2', state='REJECTED',
                                 error_code=201, source='robot_1') is None


def test_task_state_fail_wrong_code_and_source():
    err = assertions.task_state(LIFE, task_id='t2', state='REJECTED', error_code=202)
    assert 'never reached REJECTED with code 202' in err and '201' in err
    assert assertions.task_state(LIFE, task_id='t1', state='COMPLETED', source='m1')


def test_task_state_fail_too_late():
    err = assertions.task_state(LIFE, task_id='t1', state='COMPLETED', within_sec=20)
    assert 't=30.0s' in err


def test_task_state_fail_unknown_task():
    assert 'never reached' in assertions.task_state(LIFE, task_id='t9', state='COMPLETED')


def seq_stream(pairs):
    return FakeRec([(i, SimpleNamespace(meta=SimpleNamespace(source_id=s, seq=n)))
                    for i, (s, n) in enumerate(pairs)])


def test_seq_contiguous_pass_interleaved_sources_late_join():
    rec = seq_stream([('m1', 5), ('robot_1', 1), ('m1', 6), ('robot_1', 2)])
    assert assertions.seq_contiguous(rec, topic='/fleet/task_update') is None


def test_seq_contiguous_fail_gap_and_duplicate():
    gap = assertions.seq_contiguous(seq_stream([('m1', 1), ('m1', 3)]),
                                    topic='/fleet/task_update')
    dup = assertions.seq_contiguous(seq_stream([('m1', 1), ('m1', 1)]),
                                    topic='/fleet/task_update')
    assert 'gap' in gap and '1 -> 3' in gap
    assert 'duplicate' in dup


def test_task_sequence_pass_with_gaps():
    assert assertions.task_sequence(
        LIFE, task_id='t1', states=['RECEIVED', 'ASSIGNED', 'COMPLETED']) is None


def test_task_sequence_fail_wrong_order():
    err = assertions.task_sequence(LIFE, task_id='t1',
                                   states=['COMPLETED', 'EXECUTING'])
    assert 'missing from EXECUTING' in err and 'history' in err
