"""scan_range, entity_near and no_entity_near over canned recordings: PASS
and FAIL paths with honest failure messages. Pure: no rclpy, no sim."""
import math
from types import SimpleNamespace

import assertions

BEAMS = 360                                 # the robot's gpu_lidar: -pi..pi, both ends included
INCREMENT = 2 * math.pi / (BEAMS - 1)


class FakeRecording:
    def __init__(self, streams):
        self.streams = streams

    def stream(self, topic):
        return self.streams.get(topic, [])

    def duration(self):
        return 100.0


def scan(hits=(), default=3.85):
    """360-beam scan: `default` everywhere, hits = [(from_deg, to_deg, range)]."""
    ranges = []
    for index in range(BEAMS):
        bearing = math.degrees(-math.pi + index * INCREMENT)
        ranges.append(next((distance for low, high, distance in hits if low <= bearing <= high),
                           default))
    return SimpleNamespace(angle_min=-math.pi, angle_increment=INCREMENT, ranges=ranges)


BOX_AHEAD = scan([(-12.0, 12.0, 0.9)])
CLEAR = scan()


def scans(*entries):
    return FakeRecording({'/scan': list(entries)})


def test_scan_range_pass_for_presence_and_free_space():
    recording = scans((1.0, CLEAR), (5.0, BOX_AHEAD), (6.0, BOX_AHEAD), (9.0, CLEAR))
    assert assertions.scan_range(recording, topic='/scan', bearing_deg=0, min_m=0.85,
                                 max_m=0.95, from_sec=4, to_sec=7) is None
    assert assertions.scan_range(recording, topic='/scan', bearing_deg=0, min_m=3.5,
                                 from_sec=8, to_sec=10) is None
    # the box spans +-12 deg: 30 deg off it the wall is back
    assert assertions.scan_range(recording, topic='/scan', bearing_deg=30, min_m=3.5) is None


def test_scan_range_fail_names_time_bearing_and_range():
    recording = scans((5.0, BOX_AHEAD), (6.0, CLEAR))
    failure = assertions.scan_range(recording, topic='/scan', bearing_deg=0, min_m=0.85,
                                    max_m=0.95, from_sec=4, to_sec=7)
    assert failure and 't=6.0' in failure and '3.850' in failure and '0.85' in failure
    failure = assertions.scan_range(recording, topic='/scan', bearing_deg=0, min_m=3.5,
                                    from_sec=4, to_sec=7)
    assert failure and 't=5.0' in failure and '0.900' in failure


def test_scan_range_fail_without_scans_in_the_window():
    failure = assertions.scan_range(scans((1.0, CLEAR)), topic='/scan', bearing_deg=0,
                                    min_m=3.5, from_sec=4, to_sec=7)
    assert failure and 'no messages' in failure and '[4, 7]' in failure


def test_scan_range_no_return_is_far_too_close_is_zero_and_nan_fails_closed():
    nothing = scan(default=math.inf)
    assert assertions.scan_range(scans((1.0, nothing)), topic='/scan', bearing_deg=0,
                                 min_m=3.5) is None
    failure = assertions.scan_range(scans((1.0, nothing)), topic='/scan', bearing_deg=0,
                                    max_m=1.0)
    assert failure and 'no return' in failure
    too_close = scan([(-1.0, 1.0, -math.inf)])
    assert assertions.scan_range(scans((1.0, too_close)), topic='/scan', bearing_deg=0,
                                 min_m=0.5)
    broken = scan([(-1.0, 1.0, math.nan)])
    failure = assertions.scan_range(scans((1.0, broken)), topic='/scan', bearing_deg=0,
                                    min_m=0.5)
    assert failure and 'NaN' in failure


def test_scan_range_sector_wraps_around_the_back():
    behind_left = scan([(178.0, 180.0, 1.05)])      # beams at +179.0 and +180 deg only
    assert assertions.scan_range(scans((1.0, behind_left)), topic='/scan', bearing_deg=-179,
                                 min_m=1.0, max_m=1.1) is None


def observation(*entities):
    return SimpleNamespace(entities=[
        SimpleNamespace(id=entity_id, position=SimpleNamespace(x=x, y=y, z=0.0))
        for entity_id, x, y in entities])


SEEN = observation(('', 1.21, 0.02))
ELSEWHERE = observation(('', 2.5, 0.8))
NOTHING = observation()


def observations(*entries):
    return FakeRecording({'/det': list(entries)})


def test_entity_near_pass_every_message_or_a_fraction():
    recording = observations((1.0, SEEN), (2.0, SEEN), (3.0, NOTHING), (4.0, SEEN))
    assert assertions.entity_near(recording, topic='/det', x=1.2, y=0.0, tol=0.15,
                                  from_sec=0, to_sec=2.5) is None
    assert assertions.entity_near(recording, topic='/det', x=1.2, y=0.0, tol=0.15,
                                  min_fraction=0.75) is None


def test_entity_near_fail_reports_fraction_and_closest():
    recording = observations((1.0, SEEN), (2.0, ELSEWHERE), (3.0, NOTHING), (4.0, NOTHING))
    failure = assertions.entity_near(recording, topic='/det', x=1.2, y=0.0, tol=0.15,
                                     min_fraction=0.5)
    assert failure and '1/4' in failure and '50%' in failure
    failure = assertions.entity_near(recording, topic='/det', x=2.0, y=0.0, tol=0.15)
    assert failure and '0/4' in failure and 'closest 0.79' in failure


def test_entity_near_matches_the_entity_id_when_given():
    truth = observations((1.0, observation(('box', 1.2, 0.0), ('crate', 2.5, 0.8))))
    assert assertions.entity_near(truth, topic='/det', x=1.2, y=0.0, tol=0.01,
                                  entity_id='box') is None
    failure = assertions.entity_near(truth, topic='/det', x=1.2, y=0.0, tol=0.01,
                                     entity_id='crate')
    assert failure and "'crate'" in failure


def test_entity_near_fail_without_messages_in_the_window():
    failure = assertions.entity_near(observations((1.0, SEEN)), topic='/det', x=1.2, y=0.0,
                                     tol=0.15, from_sec=5, to_sec=9)
    assert failure and 'no messages' in failure


def test_no_entity_near_pass_and_fail():
    recording = observations((1.0, SEEN), (5.0, ELSEWHERE), (6.0, NOTHING))
    assert assertions.no_entity_near(recording, topic='/det', x=1.2, y=0.0, tol=0.5,
                                     from_sec=4) is None
    failure = assertions.no_entity_near(recording, topic='/det', x=1.2, y=0.0, tol=0.5)
    assert failure and 't=1.0' in failure and 'distance 0.02' in failure
    # another id at that place does not count
    truth = observations((1.0, observation(('crate', 1.2, 0.0))))
    assert assertions.no_entity_near(truth, topic='/det', x=1.2, y=0.0, tol=0.5,
                                     entity_id='box') is None


def test_no_entity_near_fail_without_messages_in_the_window():
    failure = assertions.no_entity_near(observations((1.0, NOTHING)), topic='/det', x=1.2,
                                        y=0.0, tol=0.5, from_sec=4, to_sec=9)
    assert failure and 'no messages' in failure
