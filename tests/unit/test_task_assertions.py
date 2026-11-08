"""task_state / task_sequence assertions over canned TaskUpdate streams:
both PASS and FAIL paths, with honest failure messages. Histories follow
Meta.stamp, not the order the recorder heard the updates in."""
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


def event(timestamp, task_id, state, source='robot_1', code=0, stamp=None):
    """(arrival t, TaskUpdate); Meta.stamp defaults to the arrival t."""
    stamp = timestamp if stamp is None else stamp
    meta = SimpleNamespace(source_id=source, stamp=SimpleNamespace(
        sec=int(stamp), nanosec=round(stamp % 1 * 1e9)))
    return (timestamp, SimpleNamespace(task_id=task_id, state=STATES[state], error_code=code,
                                       meta=meta))


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


# Under DDS discovery delay the mock's updates reach the recorder after the
# agent's, although the mock published first (weekly_contract_rejections).
LATE_MOCK = FakeRecording([
    event(3.0, 'rej', 'REJECTED', code=201, stamp=1.4),
    event(3.1, 'rej', 'RECEIVED', 'm1', stamp=1.0), event(3.1, 'rej', 'PARSED', 'm1', stamp=1.0),
    event(3.2, 'rej', 'QUEUED', 'm1', stamp=1.1), event(3.2, 'rej', 'ASSIGNED', 'm1', stamp=1.2),
])
REJECTION = ['RECEIVED', 'PARSED', 'QUEUED', 'ASSIGNED', 'REJECTED']


def test_task_sequence_pass_when_the_mock_updates_arrive_late():
    assert assertions.task_sequence(LATE_MOCK, task_id='rej', states=REJECTION) is None
    failure = assertions.task_state(LATE_MOCK, task_id='rej', state='COMPLETED')
    assert f'history {REJECTION}' in failure


def test_task_sequence_fail_when_the_stamps_are_out_of_order():
    # heard in the expected order, but REJECTED was stamped before ASSIGNED
    recording = FakeRecording([
        event(1.0, 'rej', 'RECEIVED', 'm1'), event(1.2, 'rej', 'ASSIGNED', 'm1'),
        event(1.3, 'rej', 'REJECTED', code=201, stamp=1.1)])
    failure = assertions.task_sequence(recording, task_id='rej',
                                       states=['RECEIVED', 'ASSIGNED', 'REJECTED'])
    assert "history ['RECEIVED', 'REJECTED', 'ASSIGNED']" in failure
    assert 'missing from REJECTED' in failure


def test_equal_stamps_keep_their_arrival_order():
    heard_in_order = FakeRecording([event(2.0, 't', 'RECEIVED', 'm1', stamp=1.0),
                                    event(2.1, 't', 'PARSED', 'm1', stamp=1.0)])
    heard_swapped = FakeRecording([event(2.0, 't', 'PARSED', 'm1', stamp=1.0),
                                   event(2.1, 't', 'RECEIVED', 'm1', stamp=1.0)])
    states = ['RECEIVED', 'PARSED']
    assert assertions.task_sequence(heard_in_order, task_id='t', states=states) is None
    failure = assertions.task_sequence(heard_swapped, task_id='t', states=states)
    assert 'missing from PARSED' in failure


def test_task_state_within_sec_counts_the_earliest_arrival():
    # two COMPLETED updates whose stamp order and arrival order cross
    recording = FakeRecording([event(8.0, 't', 'COMPLETED', 'm1', stamp=6.0),
                               event(10.0, 't', 'COMPLETED', stamp=5.0)])
    assert assertions.task_state(recording, task_id='t', state='COMPLETED', within_sec=9) is None
    failure = assertions.task_state(recording, task_id='t', state='COMPLETED', within_sec=7)
    assert 't=8.0s > 7' in failure
