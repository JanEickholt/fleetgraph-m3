"""Floor-plan regions (fleetgraph_robot.regions). Pure — no rclpy, no sim.
Covers point/disc geometry, the free gap left across a passage, and spec
validation with readable errors.
"""
import pytest

from fleetgraph_robot.regions import Region, load_regions

# Door in the wall x = 4 between hall and lab_1 (tools/gen_world.py).
DOOR = Region('door_lab1', 'traversable', 3.8, 1.0, 4.2, 2.0, label='Door lab 1',
              travel_axis='x')
LAB_1 = Region('lab_1', 'place', 4.0, 0.0, 9.0, 3.0, label='Laboratory 1')
CORRIDOR = Region('corridor', 'traversable', 0.0, 0.0, 2.0, 10.0, travel_axis='y')


def test_contains_center_size():
    assert LAB_1.contains(6.0, 1.2)
    assert LAB_1.contains(4.0, 0.0)                     # boundary counts
    assert not LAB_1.contains(3.9, 1.2)
    assert LAB_1.center() == (6.5, 1.5)
    assert LAB_1.size() == (5.0, 3.0)


def test_distance_to_disc():
    assert DOOR.distance_to_disc(4.0, 1.5, 0.1) == 0.0                     # inside
    assert DOOR.distance_to_disc(3.5, 1.5, 0.5) == 0.0                     # overlapping
    assert DOOR.distance_to_disc(3.4, 1.5, 0.3) == pytest.approx(0.1)      # face
    assert DOOR.distance_to_disc(3.5, 2.4, 0.2) == pytest.approx(0.3)      # corner: 0.5 - 0.2


def test_intersects_disc_excludes_touching():
    assert DOOR.intersects_disc(3.5, 1.5, 0.35)
    assert not DOOR.intersects_disc(3.5, 1.5, 0.3)      # touches the face only


def test_free_gap_without_discs_is_the_passage_width():
    assert DOOR.free_gap([]) == pytest.approx(1.0)
    assert CORRIDOR.free_gap([]) == pytest.approx(2.0)


def test_free_gap_one_disc_leaves_the_wider_side():
    assert DOOR.free_gap([(4.0, 1.3, 0.1)]) == pytest.approx(0.6)      # blocks 1.2..1.4
    assert CORRIDOR.free_gap([(0.5, 5.0, 0.3)]) == pytest.approx(1.2)  # blocks x 0.2..0.8


def test_free_gap_two_discs_takes_the_widest_remaining_interval():
    # blocks 1.0..1.3 and 1.6..1.8: free 0.3 between them, 0.2 above
    assert DOOR.free_gap([(4.0, 1.1, 0.2), (4.0, 1.7, 0.1)]) == pytest.approx(0.3)
    # overlapping discs block one merged interval 1.2..1.8
    assert DOOR.free_gap([(4.0, 1.4, 0.2), (4.0, 1.6, 0.2)]) == pytest.approx(0.2)


def test_free_gap_ignores_discs_outside_and_uses_the_chord_at_the_face():
    assert DOOR.free_gap([(3.0, 1.5, 0.3)]) == pytest.approx(1.0)          # in the hall
    # centre 0.3 before the face, r 0.5: chord half-width sqrt(0.25 - 0.09) = 0.4
    assert DOOR.free_gap([(3.5, 1.0, 0.5)]) == pytest.approx(0.6)          # blocks 1.0..1.4


def test_free_gap_boxes_block_their_cross_span():
    assert DOOR.free_gap([], boxes=[(3.5, 1.2, 4.5, 1.9)]) == pytest.approx(0.2)
    assert DOOR.free_gap([], boxes=[(2.0, 1.2, 3.0, 1.9)]) == pytest.approx(1.0)   # outside


def test_free_gap_needs_a_traversable():
    with pytest.raises(ValueError, match='traversable'):
        LAB_1.free_gap([])


VALID_SPEC = {'regions': [
    {'id': 'lab_1', 'kind': 'place', 'xmin': 4, 'ymin': 0, 'xmax': 9, 'ymax': 3,
     'label': 'Laboratory 1'},
    {'id': 'door_lab1', 'kind': 'traversable', 'xmin': 3.8, 'ymin': 1.0, 'xmax': 4.2,
     'ymax': 2.0, 'travel_axis': 'x'},
]}


def test_load_regions_builds_regions_with_defaults():
    lab, door = load_regions(VALID_SPEC)
    assert lab == Region('lab_1', 'place', 4.0, 0.0, 9.0, 3.0, label='Laboratory 1')
    assert isinstance(lab.xmin, float)
    assert (door.label, door.travel_axis) == ('', 'x')


def with_region(**changes):
    region = dict(VALID_SPEC['regions'][1], **changes)
    return {'regions': [region]}


def without(key):
    region = dict(VALID_SPEC['regions'][1])
    del region[key]
    return {'regions': [region]}


@pytest.mark.parametrize('spec, message', [
    ([], 'dict'),
    ({}, "'regions'"),
    ({'regions': {}}, 'list'),
    ({'regions': ['door']}, 'dict'),
    (without('ymax'), 'ymax'),
    (with_region(width=1.0), 'width'),
    (with_region(xmin='3.8'), 'xmin'),
    (with_region(xmin=True), 'xmin'),
    (with_region(xmin=float('nan')), 'finite'),
    (with_region(xmax=3.8), 'xmin < xmax'),
    (with_region(ymin=2.5), 'ymin < ymax'),
    (with_region(kind='room'), 'kind'),
    (with_region(id=''), 'id'),
    (with_region(id='robot_1/door'), '/'),
    (with_region(travel_axis='z'), 'travel_axis'),
    (without('travel_axis'), 'travel_axis'),
    (with_region(kind='place'), 'travel_axis'),
    ({'regions': VALID_SPEC['regions'] * 2}, 'duplicate'),
])
def test_load_regions_rejects_invalid_specs(spec, message):
    with pytest.raises(ValueError, match=message):
        load_regions(spec)


def test_region_constructor_validates_too():
    with pytest.raises(ValueError, match='kind'):
        Region('x', 'room', 0.0, 0.0, 1.0, 1.0)
