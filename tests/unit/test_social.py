"""Social-navigation constraints (fleetgraph_robot.social). Pure — no rclpy,
no sim. Covers person and group zones, path conflicts, the keepout raster
and its robot-radius margin, the minimum-distance metric and zone-entry
counting.
"""
import math
import statistics
import time

import pytest

from fleetgraph_robot.scene_graph import Category, Entity
from fleetgraph_robot.social import (
    SocialParams, min_person_distance, path_conflicts, rasterize, social_zones, violations)

PARAMS = SocialParams(min_distance=0.5, person_radius=0.25, group_margin=0.2, disc_segments=16)
GROUP_REACH = 0.25 + 0.5 + 0.2             # unknown body + min_distance + group_margin


def person(entity_id, x, y, side=0.0):
    """side 0: size unknown, the body radius falls back to person_radius."""
    return Entity(entity_id, Category.PERSON, 'person', x, y, size=(side, side, 1.7 * bool(side)))


def group(entity_id, member_ids, x, y, size=(0.0, 0.0, 0.0)):
    return Entity(entity_id, Category.GROUP, 'group', x, y, size=size, member_ids=list(member_ids))


def directions(count):
    return [(math.cos(2 * math.pi * k / count), math.sin(2 * math.pi * k / count))
            for k in range(count)]


def by_key(zones):
    return {(zone.kind, zone.entity_id): zone for zone in zones}


def straight(start, end, step=0.1):
    """Points from start to end every `step` m, both ends included."""
    (x0, y0), (x1, y1) = start, end
    count = round(math.hypot(x1 - x0, y1 - y0) / step)
    return [(x0 + (x1 - x0) * k / count, y0 + (y1 - y0) * k / count) for k in range(count + 1)]


# ----------------------------------------------------------------- zones

@pytest.mark.parametrize('side, min_distance, radius', [
    (0.6, 0.5, 0.8),            # body 0.3 from the size
    (0.0, 0.5, 0.75),           # size unknown: person_radius 0.25
    (0.0, 1.0, 1.25),           # the minimum distance is configurable
])
def test_person_zone_is_a_disc_of_body_plus_min_distance(side, min_distance, radius):
    (zone,) = social_zones([person('p', 2.0, 1.0, side)], SocialParams(min_distance=min_distance))
    assert (zone.kind, zone.entity_id, len(zone.polygon)) == ('person', 'p', 16)
    # circumscribed polygon: holds the whole disc, touches it at edge midpoints
    corner = radius / math.cos(math.pi / 16)
    for dx, dy in directions(64):
        assert zone.contains(2.0 + (radius - 1e-6) * dx, 1.0 + (radius - 1e-6) * dy)
        assert not zone.contains(2.0 + (corner + 1e-6) * dx, 1.0 + (corner + 1e-6) * dy)
    assert not zone.contains(2.0 + radius + 1e-6, 1.0)


def test_lone_person_zone_excludes_a_point_0_8_m_away():
    (zone,) = social_zones([person('p', 2.0, 0.0)], SocialParams(min_distance=0.5))
    for dx, dy in directions(64):
        assert not zone.contains(2.0 + 0.8 * dx, 0.8 * dy)


def test_zone_distance_is_zero_inside_and_euclidean_outside():
    (zone,) = social_zones([person('p', 0.0, 0.0)], PARAMS)     # r 0.75, flat edge facing +x
    assert zone.distance(0.0, 0.0) == 0.0 and zone.distance(0.7, 0.0) == 0.0
    assert zone.distance(1.75, 0.0) == pytest.approx(1.0)
    assert zone.distance(0.0, -1.25) == pytest.approx(0.5)


def test_group_zone_covers_the_space_between_its_members():
    members = [person('a', 2.0, 0.4), person('b', 2.0, -0.4)]       # 0.8 m apart
    zones = by_key(social_zones(members + [group('g', ['a', 'b'], 2.0, 0.0)], PARAMS))
    assert set(zones) == {('person', 'a'), ('person', 'b'), ('group', 'g')}
    assert zones['group', 'g'].contains(2.0, 0.0)


def test_group_zone_closes_the_gap_person_zones_leave():
    members = [person('a', 2.0, 1.0), person('b', 2.0, -1.0)]       # person zones 0.5 m apart
    zones = by_key(social_zones(members + [group('g', ['a', 'b'], 2.0, 0.0)], PARAMS))
    assert not zones['person', 'a'].contains(2.0, 0.0)
    assert not zones['person', 'b'].contains(2.0, 0.0)
    assert zones['group', 'g'].contains(2.0, 0.0)


def test_group_zone_reaches_min_distance_plus_margin_beyond_each_member():
    members = [person('a', 2.0, 1.0), person('b', 2.0, -1.0)]
    zone = by_key(social_zones(members + [group('g', ['a', 'b'], 2.0, 0.0)], PARAMS))['group', 'g']
    for x, y, dx, dy in ((2.0, 1.0, 0, 1), (2.0, -1.0, 0, -1), (2.0, 0.0, 1, 0), (2.0, 0.0, -1, 0)):
        assert zone.contains(x + (GROUP_REACH - 1e-6) * dx, y + (GROUP_REACH - 1e-6) * dy)
        assert not zone.contains(x + (GROUP_REACH + 1e-6) * dx, y + (GROUP_REACH + 1e-6) * dy)


def test_group_without_present_members_falls_back_to_a_disc_over_its_size_box():
    absent = group('g', ['gone/1', 'gone/2'], 2.0, 0.0, size=(0.6, 1.4, 1.7))
    (zone,) = social_zones([absent], PARAMS)
    reach = math.hypot(0.6, 1.4) / 2 + 0.5 + 0.2          # covers every corner of the box
    assert zone.kind == 'group'
    assert zone.contains(2.0 + reach - 1e-6, 0.0) and not zone.contains(2.0 + reach + 1e-6, 0.0)
    (unsized,) = social_zones([group('h', [], 2.0, 0.0)], PARAMS)
    assert unsized.contains(2.0, GROUP_REACH - 1e-6)
    assert not unsized.contains(2.0, GROUP_REACH + 1e-6)


def test_only_persons_and_groups_make_zones():
    others = [Entity('box', Category.OBJECT, 'box', 1.0, 1.0, size=(0.4, 0.4, 0.5)),
              Entity('crate', Category.OBSTACLE, 'crate', 2.0, 1.0),
              Entity('robot_2', Category.ROBOT, 'robot', 3.0, 1.0),
              Entity('lab_1', Category.PLACE, 'Laboratory 1', 6.5, 1.5, size=(5.0, 3.0, 0.0))]
    assert social_zones(others, PARAMS) == []


def test_zone_polygons_are_convex_and_counter_clockwise():
    entities = [person('a', 2.0, 0.4, side=0.5), person('b', 2.6, -0.3), person('c', 1.5, -0.6),
                group('g', ['a', 'b', 'c'], 2.0, -0.2)]
    for zone in social_zones(entities, PARAMS):
        polygon = zone.polygon
        for a, b, c in zip(polygon, polygon[1:] + polygon[:1], polygon[2:] + polygon[:2]):
            assert (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0]) > 0, zone.entity_id


# -------------------------------------------------------- path conflicts

def test_path_through_a_group_conflicts_and_a_path_around_it_does_not():
    members = [person('a', 2.0, 1.0), person('b', 2.0, -1.0)]       # person zones 0.5 m apart
    zones = social_zones(members + [group('g', ['a', 'b'], 2.0, 0.0)], PARAMS)
    through = straight((0.0, 0.0), (4.0, 0.0))                       # between the members
    assert path_conflicts(through, zones) == [(11, zones[2])]        # x 1.1: group zone from 1.05
    around = (straight((0.0, 0.0), (1.0, 2.5)) + straight((1.0, 2.5), (3.0, 2.5))[1:]
              + straight((3.0, 2.5), (4.0, 0.0))[1:])
    assert path_conflicts(around, zones) == []


def test_conflicts_are_sorted_by_where_the_path_first_meets_each_zone():
    west, east = zones = social_zones([person('west', 1.0, 0.0), person('east', 3.0, 0.0)], PARAMS)
    path = straight((4.0, 0.0), (0.0, 0.0))                          # east to west
    assert path_conflicts(path, zones) == [(3, east), (23, west)]    # first points inside: 3.7, 1.7


def test_a_segment_crossing_a_zone_between_outside_points_is_a_conflict():
    (zone,) = social_zones([person('p', 0.0, 0.0)], PARAMS)         # r 0.75
    assert path_conflicts([(0.0, -2.0), (0.0, 2.0)], [zone]) == [(0, zone)]
    assert path_conflicts([(5.0, 5.0), (0.0, -2.0), (0.0, 2.0)], [zone]) == [(1, zone)]
    # the segment ends inside: the point inside is the conflict
    assert path_conflicts([(0.0, -2.0), (0.0, 0.0)], [zone]) == [(1, zone)]
    assert path_conflicts([(0.1, 0.1), (3.0, 3.0)], [zone]) == [(0, zone)]
    assert path_conflicts([(-2.0, 1.0), (2.0, 1.0)], [zone]) == []   # 0.25 m clear
    assert path_conflicts([], [zone]) == []
    assert path_conflicts([(0.0, 0.0)], [zone]) == [(0, zone)]


def test_robot_radius_widens_what_counts_as_a_conflict():
    (zone,) = social_zones([person('p', 0.0, 0.0)], PARAMS)         # flat edge 0.75 m above
    passing = [(-2.0, 1.0), (2.0, 1.0)]                              # 0.25 m from the zone
    assert path_conflicts(passing, [zone], robot_radius=0.2) == []
    assert path_conflicts(passing, [zone], robot_radius=0.3) == [(0, zone)]
    assert path_conflicts([(0.0, 1.1), (0.0, 3.0)], [zone], robot_radius=0.4) == [(0, zone)]
    with pytest.raises(ValueError, match='robot_radius'):
        path_conflicts(passing, [zone], robot_radius=-0.1)


# ---------------------------------------------------------------- raster

def test_rasterize_marks_exactly_the_cells_whose_centre_lies_in_a_zone():
    entities = [person('a', 0.6, 0.5, side=0.5), person('b', 1.9, 1.1), person('c', 3.5, 0.2),
                group('g', ['b', 'c'], 2.7, 0.65)]
    zones = social_zones(entities, PARAMS)
    # odd origin: no cell centre sits exactly on an edge; zones a, c and g cross the border
    origin_x, origin_y, resolution, width, height = -0.013, -0.37, 0.05, 90, 50
    data = rasterize(zones, origin_x, origin_y, resolution, width, height)
    assert len(data) == width * height and 100 in data
    for row in range(height):
        y = origin_y + (row + 0.5) * resolution
        for col in range(width):
            x = origin_x + (col + 0.5) * resolution
            expected = 100 if any(zone.contains(x, y) for zone in zones) else 0
            assert data[row * width + col] == expected, (row, col)


def test_rasterize_is_row_major_from_the_origin_and_clips_to_the_grid():
    params = SocialParams(min_distance=0.02, person_radius=0.1)          # r 0.12
    zones = social_zones([person('p', 0.0, 0.0)], params)
    # 4 x 3 cells of 0.1 m, centres x -0.15 .. 0.15 and y 0.0 .. 0.2: row 0 is the lowest
    assert rasterize(zones, -0.2, -0.05, 0.1, 4, 3, value=7) == [
        0, 7, 7, 0,
        0, 7, 7, 0,
        0, 0, 0, 0]
    far = social_zones([person('q', 9.0, 9.0)], params)
    assert rasterize(far, -0.2, -0.05, 0.1, 4, 3) == [0] * 12
    assert rasterize([], -0.2, -0.05, 0.1, 4, 3) == [0] * 12
    with pytest.raises(ValueError, match='resolution'):
        rasterize(zones, 0.0, 0.0, 0.0, 4, 3)


def cell_at(zones, x, y, robot_radius):
    """The value of a one-cell grid centred on (x, y)."""
    (value,) = rasterize(zones, x - 0.025, y - 0.025, 0.05, 1, 1, robot_radius=robot_radius)
    return value


@pytest.mark.parametrize('robot_radius', [0.0, 0.25])
def test_rasterize_grows_a_person_disc_by_the_robot_radius(robot_radius):
    (zone,) = social_zones([person('p', 2.0, 1.0)], PARAMS)         # r 0.75, flat edge facing +x
    # towards the middle of an edge, and through a corner, where the margin is an arc
    half_edge = math.pi / 16
    for angle, reach in ((0.0, 0.75), (half_edge, 0.75 / math.cos(half_edge))):
        dx, dy = math.cos(angle), math.sin(angle)
        inside, outside = reach + robot_radius - 1e-6, reach + robot_radius + 1e-6
        assert cell_at([zone], 2.0 + inside * dx, 1.0 + inside * dy, robot_radius) == 100, angle
        assert cell_at([zone], 2.0 + outside * dx, 1.0 + outside * dy, robot_radius) == 0, angle


@pytest.mark.parametrize('robot_radius', [0.0, 0.25])
def test_rasterize_grows_a_group_hull_by_the_robot_radius(robot_radius):
    members = [person('a', 2.0, 1.0), person('b', 2.0, -1.0)]       # person zones 0.5 m apart
    zone = by_key(social_zones(members + [group('g', ['a', 'b'], 2.0, 0.0)], PARAMS))['group', 'g']
    reach = GROUP_REACH + robot_radius
    # sideways from the gap between the members, and beyond each member
    for x, y, dx, dy in ((2.0, 0.0, 1, 0), (2.0, 0.0, -1, 0), (2.0, 1.0, 0, 1), (2.0, -1.0, 0, -1)):
        inside, outside = reach - 1e-6, reach + 1e-6
        assert cell_at([zone], x + inside * dx, y + inside * dy, robot_radius) == 100, (dx, dy)
        assert cell_at([zone], x + outside * dx, y + outside * dy, robot_radius) == 0, (dx, dy)


def test_rasterize_marks_exactly_the_cells_whose_centre_lies_within_the_robot_radius():
    entities = [person('a', 0.6, 0.5, side=0.5), person('b', 1.9, 1.1), person('c', 3.5, 0.2),
                group('g', ['b', 'c'], 2.7, 0.65)]
    zones = social_zones(entities, PARAMS)
    origin_x, origin_y, resolution, width, height = -0.513, -0.87, 0.05, 110, 70
    robot_radius = 0.2537
    data = rasterize(zones, origin_x, origin_y, resolution, width, height,
                     robot_radius=robot_radius)
    assert 0 < data.count(100) < len(data)
    for row in range(height):
        y = origin_y + (row + 0.5) * resolution
        for col in range(width):
            x = origin_x + (col + 0.5) * resolution
            expected = 100 if any(zone.distance(x, y) <= robot_radius for zone in zones) else 0
            assert data[row * width + col] == expected, (row, col)
    with pytest.raises(ValueError, match='robot_radius'):
        rasterize(zones, origin_x, origin_y, resolution, width, height, robot_radius=-0.1)


@pytest.mark.parametrize('robot_radius', [0.0, 0.25])
def test_rasterize_10_zones_on_the_lab_map_is_fast(robot_radius):
    spots = [(0.0, -2.0), (1.0, 2.0), (2.0, -1.0), (3.0, 1.0),
             (5.0, 2.0), (6.0, -1.5), (7.0, 1.0), (8.0, -2.0)]
    persons = [person(f'p{index}', x, y) for index, (x, y) in enumerate(spots)]
    groups = [group('g1', ['p0', 'p2'], 1.0, -1.5), group('g2', ['p5', 'p6'], 6.5, -0.25)]
    zones = social_zones(persons + groups, PARAMS)
    assert len(zones) == 10
    samples = []
    for _ in range(15):
        start = time.perf_counter()
        data = rasterize(zones, -1.5, -3.5, 0.05, 220, 140,               # the lab_floor map
                         robot_radius=robot_radius)
        samples.append(time.perf_counter() - start)
    median = statistics.median(samples)
    print(f'\nrasterize(10 zones, 220 x 140 cells, robot_radius {robot_radius}): '
          f'median {median * 1e3:.2f} ms, max {max(samples) * 1e3:.2f} ms')
    assert data.count(100) > 0
    assert median < 0.050


# --------------------------------------------------------------- metrics

def test_min_person_distance_is_measured_to_the_nearest_body_surface():
    entities = [person('near', 2.0, 0.0, side=0.6), person('far', 5.0, 0.0),
                Entity('box', Category.OBJECT, 'box', 3.1, 0.0, size=(0.2, 0.2, 0.2)),
                group('g', ['near', 'far'], 3.5, 0.0, size=(3.6, 0.6, 1.7))]
    assert min_person_distance(3.0, 0.0, entities, PARAMS) == pytest.approx(0.7)    # 1 - 0.3
    assert min_person_distance(4.0, 0.0, entities, PARAMS) == pytest.approx(0.75)   # 1 - 0.25
    assert min_person_distance(2.1, 0.0, entities, PARAMS) == 0.0                   # in the body
    assert min_person_distance(0.0, 0.0, entities[2:], PARAMS) == math.inf


def test_violations_count_zone_entries_not_samples_inside():
    walker = [person('p', 2.0, 0.0)]                                    # zone r 0.75
    passing = [(float(t), 0.5 * t, 0.5) for t in range(9)]               # inside at x 1.5 .. 2.5
    assert violations(passing, lambda t: walker, robot_radius=0.2, params=PARAMS) == \
        [(3.0, 'person', 'p')]
    back_and_forth = [(0.0, 0.0, 0.5), (1.0, 1.5, 0.5), (2.0, 2.0, 0.5), (3.0, 2.0, 1.5),
                      (4.0, 2.0, 0.6)]
    assert violations(back_and_forth, lambda t: walker, robot_radius=0.2, params=PARAMS) == \
        [(1.0, 'person', 'p'), (4.0, 'person', 'p')]


def test_violations_use_the_robot_footprint():
    walker = [person('p', 2.0, 0.0)]                         # flat zone edge 0.75 m above
    near = [(0.0, 2.0, 0.85)]                                # centre 0.1 m from the zone
    assert violations(near, lambda t: walker, robot_radius=0.2, params=PARAMS) == \
        [(0.0, 'person', 'p')]
    assert violations(near, lambda t: walker, robot_radius=0.05, params=PARAMS) == []
    with pytest.raises(ValueError, match='robot_radius'):
        violations(near, lambda t: walker, robot_radius=-0.1, params=PARAMS)


def test_a_person_walking_into_a_standing_robot_counts_as_an_entry():
    standing = [(float(t), 3.0, 0.0) for t in range(5)]
    found = violations(standing, lambda t: [person('p', t, 0.0)], robot_radius=0.2, params=PARAMS)
    assert found == [(3.0, 'person', 'p')]


def test_crossing_between_group_members_is_a_group_entry():
    entities = [person('a', 2.0, 1.0), person('b', 2.0, -1.0), group('g', ['a', 'b'], 2.0, 0.0)]
    track = [(0.5 * step, 0.5 * step, 0.0) for step in range(9)]       # through the gap
    assert violations(track, lambda t: entities, robot_radius=0.1, params=PARAMS) == \
        [(1.0, 'group', 'g')]                                         # x 1.0: 0.05 m away


# ---------------------------------------------------------------- params

@pytest.mark.parametrize('name, value', [
    ('min_distance', -0.1), ('person_radius', 0.0), ('group_margin', -0.1), ('disc_segments', 2)])
def test_social_params_validate(name, value):
    with pytest.raises(ValueError, match=name):
        SocialParams(**{name: value})
