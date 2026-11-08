"""Plain-text task parser of the mock Module 1 (fleetgraph_robot.task_text).
Pure: no rclpy, no sim. Accepted phrasings per task type, priorities, patrol
loops, and every rejection path with its reason."""
import pytest

from fleetgraph_robot.task_text import ParseError, ParsedTask, parse_task_text

# ids and labels as in src/fleetgraph_sim/config/locations.yaml
LOCATIONS = {'dock': 'Charging dock', 'storage': 'Storage', 'hall': 'Hall',
             'lab_1': 'Laboratory 1', 'lab_2': 'Laboratory 2',
             'dock_2': 'Charging dock 2', 'dock_3': 'Charging dock 3'}


def parse(text):
    return parse_task_text(text, LOCATIONS)


@pytest.mark.parametrize('text, location', [
    ('Go to laboratory 1', 'lab_1'),
    ('go to lab 1', 'lab_1'),
    ('Navigate to Lab2', 'lab_2'),
    ('Go to laboratory two', 'lab_2'),
    ('Drive to the storage room.', 'storage'),
    ('Please head to the hall', 'hall'),
    ('Return to the charging dock', 'dock'),
    ('Move to charging dock 2', 'dock_2'),
    ('go to dock_3', 'dock_3'),
])
def test_go_to(text, location):
    assert parse(text) == ParsedTask('go_to', (location,))


@pytest.mark.parametrize('text, pickup, dropoff', [
    ('Bring the parcel from the storage to lab 2', 'storage', 'lab_2'),
    ('Deliver a package from storage to laboratory 2', 'storage', 'lab_2'),
    ('Take the box to lab 1 from the hall', 'hall', 'lab_1'),
    ('Fetch the sample in lab 1 and bring it to the storage', 'lab_1', 'storage'),
    ('Go to the storage and bring the parcel to lab 2', 'storage', 'lab_2'),
])
def test_delivery_pickup_then_dropoff(text, pickup, dropoff):
    assert parse(text) == ParsedTask('delivery', (pickup, dropoff),
                                     required_capabilities=('deliver',))


@pytest.mark.parametrize('text, locations, loops', [
    ('Patrol the hall and the dock', ('hall', 'dock'), 1),
    ('Patrol lab 1 and lab 2 twice', ('lab_1', 'lab_2'), 2),
    ('patrol between storage and hall 3 times', ('storage', 'hall'), 3),
    # a count before "times" is a loop count, not a dock number
    ('Patrol the dock 2 times', ('dock',), 2),
    ('Patrol dock 2 and dock 3 2x', ('dock_2', 'dock_3'), 2),
])
def test_patrol(text, locations, loops):
    assert parse(text) == ParsedTask('patrol', locations, patrol_loops=loops,
                                     required_capabilities=('patrol',))


@pytest.mark.parametrize('text, priority', [
    ('Go to lab 1', 'normal'),
    ('Go to lab 1, high priority', 'high'),
    ('Go to lab 1 with priority high', 'high'),
    ('low priority: patrol the hall and the dock', 'low'),
    ('Urgent: bring the parcel from storage to lab 2', 'urgent'),
    ('Go to the hall immediately', 'urgent'),
])
def test_priority(text, priority):
    assert parse(text).priority == priority


@pytest.mark.parametrize('text, reason', [
    ('', 'empty'),
    ('   ', 'empty'),
    ('Make me a sandwich', 'no task verb'),
    ('Go to the cafeteria', 'no known location'),
    ('Go to the lab', "ambiguous location 'lab': lab_1 or lab_2"),
    ('Go charging', "ambiguous location 'charging': dock, dock_2 or dock_3"),
    ('Bring the parcel to lab 2', 'delivery needs a pickup and a drop-off location, got lab_2'),
    ('Go to lab 1 and lab 2', 'go to takes one location, got lab_1, lab_2'),
    ('Patrol the hall and bring the parcel from storage to lab 2', 'both patrol and delivery'),
    ('Go to lab 1 three times', 'repeat count only applies to patrol'),
    ('Patrol the hall 0 times', 'repeat count must be at least 1'),
    ('Patrol the hall twice, 3 times', 'repeat count given twice'),
])
def test_rejected_with_reason(text, reason):
    with pytest.raises(ParseError, match=reason):
        parse(text)


def test_unknown_location_reason_lists_known_ids():
    with pytest.raises(ParseError) as error:
        parse('Go to the cafeteria')
    assert 'lab_1' in str(error.value) and 'dock_3' in str(error.value)


def test_locations_come_from_the_registry():
    registry = {'kitchen': 'Kitchen', 'lab_1': 'Laboratory 1'}
    assert parse_task_text('Go to the kitchen', registry) == ParsedTask('go_to', ('kitchen',))
    # one lab only: "lab" alone is then no location, not an ambiguous one
    with pytest.raises(ParseError, match='no known location'):
        parse_task_text('Go to the lab', registry)


def test_same_name_for_two_locations_is_ambiguous():
    registry = {'lab_north': 'Lab', 'lab_south': 'Lab'}
    with pytest.raises(ParseError, match="ambiguous location 'lab': lab_north or lab_south"):
        parse_task_text('Go to the lab', registry)
