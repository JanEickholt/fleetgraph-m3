"""Scene-graph assertions (graph_has, graph_lacks_after,
graph_versions_consistent, graph_matches) over canned SceneGraphUpdate
streams: both PASS and FAIL paths, with honest failure messages. Pure: no
rclpy, no sim (the category mirror check skips without a built workspace)."""
from types import SimpleNamespace

import pytest

import assertions

TOPIC = '/fleet/scene_graph_update'
REPLICA = '/robot_3/central/scene'


class FakeRecording:
    def __init__(self, streams):
        self.streams = streams

    def stream(self, topic):
        return self.streams.get(topic, [])

    def duration(self):
        return 30.0


def node(entity_id, x, y, category='OBSTACLE', sources=(), **attributes):
    return SimpleNamespace(
        id=entity_id, category=assertions.SCENE_CATEGORIES[category],
        position=SimpleNamespace(x=x, y=y), source_ids=list(sources),
        attributes=[SimpleNamespace(key=key, value=value) for key, value in attributes.items()])


def delta(version, added=(), updated=(), removed=(), base=None):
    return SimpleNamespace(graph_version=version,
                           base_version=version - 1 if base is None else base,
                           is_snapshot=False, added_nodes=list(added),
                           updated_nodes=list(updated), removed_node_ids=list(removed))


def snapshot(version, nodes=()):
    return SimpleNamespace(graph_version=version, base_version=0, is_snapshot=True,
                           added_nodes=list(nodes), updated_nodes=[], removed_node_ids=[])


ALONE = node('g/obstacle/1', 3.0, 0.5, sources=['robot_1'], persistence='temporary')
FUSED = node('g/obstacle/1', 3.1, 0.5, sources=['robot_1', 'robot_2'], persistence='temporary')
LEFT = node('g/obstacle/1', 3.2, 0.5, sources=['robot_2'], persistence='persistent')
# robot_1 reports at 2 s, robot_2 joins at 3 s, robot_1 leaves at 9 s, robot_2 at 12 s
LIFE = FakeRecording({TOPIC: [
    (2.0, delta(1, added=[ALONE])), (3.0, delta(2, updated=[FUSED])),
    (9.0, delta(3, updated=[LEFT])), (10.0, snapshot(3, [LEFT])),
    (12.0, delta(4, removed=['g/obstacle/1']))]})


def has(recording=LIFE, **params):
    return assertions.graph_has(recording, **{'topic': TOPIC, 'category': 'OBSTACLE',
                                              'x': 3.1, 'y': 0.5, 'tol': 0.3, **params})


def lacks_after(recording=LIFE, **params):
    return assertions.graph_lacks_after(recording, **{
        'topic': TOPIC, 'category': 'OBSTACLE', 'x': 3.1, 'y': 0.5, 'tol': 0.3, **params})


def test_scene_categories_mirror():
    messages = pytest.importorskip('fleetgraph_interfaces.msg')
    assert assertions.SCENE_CATEGORIES == {
        name.removeprefix('CATEGORY_'): getattr(messages.SceneEntity, name)
        for name in dir(messages.SceneEntity) if name.startswith('CATEGORY_')}


# ------------------------------------------------------------ graph_has

def test_graph_has_pass_on_category_position_sources_and_attribute():
    assert has() is None
    assert has(sources=['robot_1', 'robot_2'], attribute={'persistence': 'temporary'},
               within_sec=3.0) is None
    assert has(attribute={'persistence': 'persistent'}) is None


def test_graph_has_fail_reports_the_nearest_entity_of_the_category():
    failure = has(sources=['robot_3'])
    assert 'no OBSTACLE within 0.3 of (3.1, 0.5)' in failure and "'robot_3'" in failure
    assert 'nearest: g/obstacle/1 at (3.10, 0.50)' in failure
    failure = has(category='PERSON')
    assert 'no PERSON ever' in failure
    failure = has(x=5.0, y=5.0)
    assert 'nearest: g/obstacle/1' in failure and 'distance' in failure
    assert has(attribute={'persistence': 'gone'})


def test_graph_has_fail_too_late():
    failure = has(sources=['robot_1', 'robot_2'], within_sec=2.5)
    assert 'only at t=3.0s > 2.5s' in failure
    assert has(attribute={'persistence': 'persistent'}, within_sec=8.0)


def test_graph_has_after_counts_the_state_in_force_and_later_ones():
    # in force at 5 s: the fused state of 3 s; robot_2 alone from 9 s
    assert has(sources=['robot_1', 'robot_2'], after_sec=5.0, within_sec=5.0) is None
    assert has(sources=['robot_2'], attribute={'persistence': 'persistent'},
               after_sec=11.0) is None
    failure = has(sources=['robot_1'], after_sec=9.5)
    assert 'at or after t=9.5s' in failure
    assert has(after_sec=12.5)                  # removed at 12 s


def test_graph_has_fail_without_messages():
    assert 'no messages on' in has(FakeRecording({}))


def test_graph_has_reads_a_replica_topic_of_snapshots():
    replica = FakeRecording({TOPIC: [(4.0, snapshot(2, [FUSED])), (12.5, snapshot(4))]})
    assert has(replica, sources=['robot_2']) is None
    assert has(replica, after_sec=13.0)


# ---------------------------------------------------- graph_lacks_after

def test_graph_lacks_after_pass_once_removed():
    assert lacks_after(t=12.0) is None
    assert lacks_after(sources=['robot_1'], t=9.0) is None
    assert lacks_after(category='PERSON', t=0.0) is None
    assert lacks_after(attribute={'persistence': 'persistent'}, t=0.0, x=9.0) is None


def test_graph_lacks_after_fail_while_present_or_in_force():
    failure = lacks_after(t=11.0)               # in force since 10 s, removed only at 12 s
    assert 'g/obstacle/1' in failure and 'still in the graph at t=11.0s' in failure
    failure = lacks_after(attribute={'persistence': 'persistent'}, t=0.0)
    assert 'at t=9.0s' in failure
    assert lacks_after(sources=['robot_1'], t=8.0)


def test_graph_lacks_after_fails_closed_without_messages():
    assert 'no messages on' in lacks_after(FakeRecording({}), t=0.0)


def test_graph_lacks_after_a_snapshot_drops_what_it_does_not_list():
    stream = [(1.0, delta(1, added=[FUSED])), (5.0, snapshot(7))]
    assert lacks_after(FakeRecording({TOPIC: stream}), t=5.0) is None
    assert lacks_after(FakeRecording({TOPIC: stream}), t=4.0)


# -------------------------------------------- graph_versions_consistent

def consistent(stream):
    return assertions.graph_versions_consistent(FakeRecording({TOPIC: stream}), topic=TOPIC)


def test_graph_versions_consistent_pass():
    assert assertions.graph_versions_consistent(LIFE, topic=TOPIC) is None
    # joined late, snapshot after a central restart, a replica's snapshots only
    assert consistent([(1.0, delta(41)), (2.0, delta(42)), (3.0, snapshot(1)),
                       (4.0, delta(2))]) is None
    assert consistent([(1.0, snapshot(5)), (2.0, snapshot(9))]) is None


def test_graph_versions_consistent_fail_on_gap_duplicate_or_skipped_version():
    failure = consistent([(1.0, delta(1)), (2.0, delta(3))])
    assert 'delta 2->3 at t=2.0s does not follow version 1' in failure
    assert 'does not follow version 2' in consistent([(1.0, delta(1)), (2.0, delta(2)),
                                                      (3.0, delta(2))])
    assert 'does not follow version 4' in consistent([(1.0, snapshot(4)), (2.0, delta(6))])
    failure = consistent([(1.0, delta(1)), (2.0, delta(3, base=1))])
    assert 'delta 1->3 at t=2.0s is not +1' in failure
    assert 'no messages on' in consistent([])


# ------------------------------------------------------- graph_matches

def matches(replica, reference=LIFE.streams[TOPIC], **params):
    recording = FakeRecording({REPLICA: replica, TOPIC: reference})
    return assertions.graph_matches(recording, topic=REPLICA, reference=TOPIC, **params)


def test_graph_matches_pass_on_the_same_final_graph():
    assert matches([(12.1, snapshot(4))]) is None
    middle = LIFE.streams[TOPIC][:3]
    assert matches([(9.1, snapshot(3, [node('g/obstacle/1', 3.205, 0.5, sources=['robot_2'],
                                            persistence='persistent')]))], middle) is None


def test_graph_matches_fail_on_version_or_entity_differences():
    middle = LIFE.streams[TOPIC][:3]
    assert 'version 2 != reference version 4' in matches([(3.1, snapshot(2, [FUSED]))])
    failure = matches([(9.1, snapshot(3))], middle)
    assert 'missing g/obstacle/1' in failure
    failure = matches([(9.1, snapshot(3, [LEFT, node('g/person/2', 0.0, 0.0, 'PERSON')]))],
                      middle)
    assert 'unexpected g/person/2' in failure
    failure = matches([(9.1, snapshot(3, [FUSED]))], middle)
    assert 'g/obstacle/1 differs' in failure and "'robot_1'" in failure
    failure = matches([(9.1, snapshot(3, [node('g/obstacle/1', 3.5, 0.5, sources=['robot_2'],
                                               persistence='persistent')]))], middle)
    assert 'g/obstacle/1 differs' in failure and '3.50' in failure
    assert 'no messages on /robot_3/central/scene' in matches([])
