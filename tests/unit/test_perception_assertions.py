"""Perception assertions (observes, observes_only, relation_seen,
removal_seen, latency_below) over canned SceneObservation streams: PASS and
FAIL paths, with failure messages that carry the observed values."""
from types import SimpleNamespace

import pytest

import assertions

TOPIC = '/fleet/scene_observation'
CATEGORIES = assertions.SCENE_CATEGORIES
PREDICATES = assertions.SCENE_PREDICATES
CRATE = (2.5, 0.8)


class FakeRecording:
    def __init__(self, messages):
        self.messages = messages

    def stream(self, topic):
        return self.messages if topic == TOPIC else []

    def duration(self):
        return 100.0


def stamp(seconds):
    sec = int(seconds)
    return SimpleNamespace(sec=sec, nanosec=round((seconds - sec) * 1e9))


def entity(entity_id, category, x, y):
    return SimpleNamespace(id=entity_id, category=CATEGORIES[category],
                           position=SimpleNamespace(x=x, y=y, z=0.0))


def relation(subject_id, predicate, object_id):
    return SimpleNamespace(subject_id=subject_id, predicate=PREDICATES[predicate],
                           object_id=object_id, confidence=0.9)


def observation(t, entities=(), relations=(), removed=(), source='robot_1', latency=0.1):
    """(recorder t, msg) published at sim time t from sensor data latency s old."""
    msg = SimpleNamespace(
        meta=SimpleNamespace(source_id=source, seq=1, stamp=stamp(t)),
        robot_id=source, frame_id='map',
        sensor_pose=SimpleNamespace(header=SimpleNamespace(stamp=stamp(t - latency))),
        entities=list(entities), relations=list(relations), removed_entity_ids=list(removed))
    return (t, msg)


HALL = entity('hall', 'PLACE', 1.5, 0.0)
CRATE_TRACK = entity('robot_1/obstacle/1', 'OBSTACLE', 2.55, 0.78)
PERSON = entity('robot_1/person/2', 'PERSON', 1.0, -1.0)
DRIVE = FakeRecording([
    observation(20.0, [HALL]),
    observation(30.5, [CRATE_TRACK, PERSON],
                [relation(CRATE_TRACK.id, 'INSIDE', 'hall'),
                 relation(PERSON.id, 'INSIDE', 'hall')]),
    observation(31.0, [entity('robot_2/obstacle/1', 'OBSTACLE', 7.0, 2.5)], source='robot_2'),
    observation(80.0, [], [relation(PERSON.id, 'INSIDE', 'hall')], removed=[CRATE_TRACK.id]),
])
EMPTY = FakeRecording([])


# --------------------------------------------------------------- observes

def test_observes_pass():
    assert assertions.observes(DRIVE, topic=TOPIC, source='robot_1', category='OBSTACLE',
                               x=2.5, y=0.8, tol=0.25) is None
    assert assertions.observes(DRIVE, topic=TOPIC, source='robot_1', category='OBSTACLE',
                               x=2.5, y=0.8, tol=0.25, within_sec=40) is None


def test_observes_fail_too_far_names_the_nearest():
    failure = assertions.observes(DRIVE, topic=TOPIC, source='robot_1', category='OBSTACLE',
                                  x=2.5, y=1.5, tol=0.25)
    assert 'robot_1/obstacle/1' in failure and '(2.55, 0.78)' in failure
    assert 'distance 0.72' in failure


def test_observes_fail_wrong_category_source_or_time():
    assert 'no PERSON' in assertions.observes(DRIVE, topic=TOPIC, source='robot_1',
                                              category='PERSON', x=2.5, y=0.8, tol=0.25)
    # robot_2's obstacle is no evidence for robot_1
    assert assertions.observes(DRIVE, topic=TOPIC, source='robot_1', category='OBSTACLE',
                               x=7.0, y=2.5, tol=0.25)
    failure = assertions.observes(DRIVE, topic=TOPIC, source='robot_1', category='OBSTACLE',
                                  x=2.5, y=0.8, tol=0.25, within_sec=30)
    assert 't=30.5s' in failure
    never = assertions.observes(DRIVE, topic=TOPIC, source='robot_1', category='ROBOT',
                                x=0.0, y=0.0, tol=1.0)
    assert 'no ROBOT entity ever reported' in never


def test_observes_fail_without_messages():
    failure = assertions.observes(EMPTY, topic=TOPIC, source='robot_1', category='OBSTACLE',
                                  x=2.5, y=0.8, tol=0.25)
    assert 'no messages' in failure and 'robot_1' in failure


# ---------------------------------------------------------- observes_only

def test_observes_only_pass_ignores_other_categories_and_sources():
    assert assertions.observes_only(DRIVE, topic=TOPIC, source='robot_1', category='OBSTACLE',
                                    points=[list(CRATE)], tol=0.5) is None


def test_observes_only_fail_without_a_report_of_the_category():
    # a source that never reports the category proves nothing about its false
    # positives (a perception node that stopped tracking passed before)
    failure = assertions.observes_only(DRIVE, topic=TOPIC, source='robot_1', category='ROBOT',
                                       points=[], tol=0.5)
    assert failure and 'no ROBOT entity ever reported on /fleet/scene_observation from ' \
        'robot_1' in failure


def test_observes_only_fail_on_a_wall_obstacle():
    wall = entity('robot_1/obstacle/7', 'OBSTACLE', 3.95, -0.5)
    recording = FakeRecording(DRIVE.messages + [observation(40.0, [wall]),
                                                observation(40.5, [wall])])
    failure = assertions.observes_only(recording, topic=TOPIC, source='robot_1',
                                       category='OBSTACLE', points=[list(CRATE)], tol=0.5)
    assert 'robot_1/obstacle/7' in failure and '(3.95, -0.50)' in failure
    assert '2 OBSTACLE report' in failure and '1.95' in failure and 't=40.0s' in failure


def test_observes_only_fail_without_messages():
    failure = assertions.observes_only(EMPTY, topic=TOPIC, source='robot_1',
                                       category='OBSTACLE', points=[list(CRATE)], tol=0.5)
    assert 'no messages' in failure


# ---------------------------------------------------------- relation_seen

def test_relation_seen_pass():
    assert assertions.relation_seen(DRIVE, topic=TOPIC, source='robot_1', predicate='INSIDE',
                                    object_id='hall', subject_category='OBSTACLE') is None
    assert assertions.relation_seen(DRIVE, topic=TOPIC, source='robot_1', predicate='INSIDE',
                                    object_id='hall') is None


def test_relation_seen_fail_lists_what_was_seen():
    failure = assertions.relation_seen(DRIVE, topic=TOPIC, source='robot_1',
                                       predicate='INSIDE', object_id='hall',
                                       subject_category='ROBOT')
    assert 'no ROBOT subject INSIDE hall' in failure
    assert 'robot_1/obstacle/1 (OBSTACLE) INSIDE hall' in failure
    assert 'robot_1/person/2 (PERSON) INSIDE hall' in failure
    assert assertions.relation_seen(DRIVE, topic=TOPIC, source='robot_1', predicate='BLOCKS',
                                    object_id='hall')
    assert assertions.relation_seen(DRIVE, topic=TOPIC, source='robot_2', predicate='INSIDE',
                                    object_id='hall')


def test_relation_seen_fail_without_messages():
    assert 'no messages' in assertions.relation_seen(EMPTY, topic=TOPIC, source='robot_1',
                                                     predicate='INSIDE', object_id='hall')


# ----------------------------------------------------------- removal_seen

def test_removal_seen_pass():
    assert assertions.removal_seen(DRIVE, topic=TOPIC, source='robot_1', category='OBSTACLE',
                                   x=2.5, y=0.8, tol=0.25) is None


def test_removal_seen_fail_never_removed_or_removed_before_reported():
    still_there = FakeRecording(DRIVE.messages[:3])
    failure = assertions.removal_seen(still_there, topic=TOPIC, source='robot_1',
                                      category='OBSTACLE', x=2.5, y=0.8, tol=0.25)
    assert 'robot_1/obstacle/1' in failure and 'never appeared in removed_entity_ids' in failure
    assert 't=30.5s' in failure
    early = FakeRecording([observation(10.0, removed=[CRATE_TRACK.id]),
                           observation(11.0, [CRATE_TRACK])])
    assert assertions.removal_seen(early, topic=TOPIC, source='robot_1', category='OBSTACLE',
                                   x=2.5, y=0.8, tol=0.25)


def test_removal_seen_fail_nothing_reported_there():
    failure = assertions.removal_seen(DRIVE, topic=TOPIC, source='robot_1', category='PERSON',
                                      x=2.5, y=0.8, tol=0.25)
    assert 'no PERSON within 0.25 of (2.5, 0.8)' in failure
    assert 'no messages' in assertions.removal_seen(EMPTY, topic=TOPIC, source='robot_1',
                                                    category='OBSTACLE', x=2.5, y=0.8, tol=0.25)


# ---------------------------------------------------------- latency_below

def test_latency_below_pass_per_source():
    recording = FakeRecording([observation(10.0, latency=0.12), observation(10.5, latency=0.3),
                               observation(11.0, latency=2.0, source='robot_2')])
    assert assertions.latency_below(recording, topic=TOPIC, source='robot_1',
                                    max_sec=0.5) is None


def test_latency_below_fail_reports_the_worst():
    recording = FakeRecording([observation(10.0, latency=0.12), observation(10.5, latency=0.7),
                               observation(11.0, latency=0.2)])
    failure = assertions.latency_below(recording, topic=TOPIC, max_sec=0.5)
    assert 'latency 0.700 s' in failure and 't=10.5s' in failure and '3 messages' in failure


def test_latency_below_fail_on_sensor_stamps_from_another_clock():
    # sensor stamp in wall time, publication in sim time: hugely negative
    recording = FakeRecording([observation(40.0, latency=-1.79e9)])
    failure = assertions.latency_below(recording, topic=TOPIC, max_sec=0.5)
    assert 'negative' in failure


def test_latency_below_fail_without_messages():
    assert 'no messages' in assertions.latency_below(EMPTY, topic=TOPIC, max_sec=0.5)


# ----------------------------------------------------------------- mirror

def msg_constants(message, prefix):
    return {name.removeprefix(prefix): getattr(message, name)
            for name in dir(message) if name.startswith(prefix)}


def test_scene_constants_mirror_the_messages():
    messages = pytest.importorskip('fleetgraph_interfaces.msg')
    assert msg_constants(messages.SceneEntity, 'CATEGORY_') == CATEGORIES
    assert msg_constants(messages.SceneRelation, 'PREDICATE_') == PREDICATES
