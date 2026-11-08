"""Keepout mask of the robot's social zones (fleetgraph_robot.social_mask).
Pure, no rclpy, no sim. Covers the entity store per source (local deltas,
central snapshots, expiry of unreported entities), the mask on the static
map's grid (keepout cells, no information elsewhere, robot radius plus
margin) and publishing only changed masks."""
import math

import pytest

from fleetgraph_robot.scene_graph import Category, Entity
from fleetgraph_robot.social import SocialParams
from fleetgraph_robot.social_mask import (
    KEEPOUT, NO_INFORMATION, EntityStore, Grid, SocialMask, keepout_mask)

PARAMS = SocialParams(min_distance=0.5, person_radius=0.25, group_margin=0.2)
# the lab-floor map: 11 x 7 m of 0.05 m cells, origin (-1.5, -3.5)
LAB = Grid(origin_x=-1.5, origin_y=-3.5, resolution=0.05, width=220, height=140)
SMALL = Grid(origin_x=0.0, origin_y=0.0, resolution=0.1, width=40, height=20)


def person(entity_id, x, y):
    return Entity(entity_id, Category.PERSON, 'person', x, y)


def group(entity_id, member_ids, x, y):
    return Entity(entity_id, Category.GROUP, 'group', x, y, member_ids=list(member_ids))


def cell_value(data, grid, x, y):
    col = math.floor((x - grid.origin_x) / grid.resolution)
    row = math.floor((y - grid.origin_y) / grid.resolution)
    return data[row * grid.width + col]


def ids(store):
    return sorted(entity.id for entity in store.entities())


# ------------------------------------------------------------------ store

def test_a_delta_adds_and_updates_entities_by_id_and_removes_listed_ids():
    store = EntityStore()
    store.apply_delta('local', [person('r1/person/1', 1.0, 0.0),
                                person('r1/person/2', 2.0, 0.0)], [], now=1.0)
    store.apply_delta('local', [person('r1/person/1', 1.5, 0.5)], ['r1/person/2'], now=2.0)
    (moved,) = store.entities()
    assert (moved.id, moved.x, moved.y) == ('r1/person/1', 1.5, 0.5)
    store.apply_delta('local', [], ['r1/person/9'], now=3.0)       # never heard of: ignored
    assert ids(store) == ['r1/person/1']


def test_a_snapshot_replaces_only_its_own_source():
    store = EntityStore()
    store.apply_delta('local', [person('r1/person/1', 1.0, 0.0)], [], now=1.0)
    store.replace('central', [person('g/person/1', 3.0, 0.0), person('g/person/2', 4.0, 0.0)])
    store.replace('central', [person('g/person/2', 4.2, 0.0)])
    assert ids(store) == ['g/person/2', 'r1/person/1']


def test_unreported_delta_entities_expire_and_snapshots_do_not():
    store = EntityStore(stale_after_sec=10.0)
    store.apply_delta('local', [person('r1/person/1', 1.0, 0.0),
                                person('r1/person/2', 2.0, 0.0)], [], now=0.0)
    store.replace('central', [person('g/person/1', 3.0, 0.0)])
    store.apply_delta('local', [person('r1/person/2', 2.0, 0.0)], [], now=6.0)
    assert not store.expire(now=10.0)                       # 10 s is not more than 10 s
    assert store.expire(now=10.5)                           # r1/person/1: 10.5 s unreported
    assert ids(store) == ['g/person/1', 'r1/person/2']
    assert not store.expire(now=16.0)
    assert store.expire(now=16.1) and ids(store) == ['g/person/1']
    assert not store.expire(now=1000.0)                     # a snapshot holds until the next


def test_stale_after_zero_keeps_everything():
    store = EntityStore(stale_after_sec=0.0)
    store.apply_delta('local', [person('r1/person/1', 1.0, 0.0)], [], now=0.0)
    assert not store.expire(now=1e6) and ids(store) == ['r1/person/1']


# ------------------------------------------------------------------- mask

def test_mask_is_keepout_within_robot_radius_plus_margin_and_no_information_elsewhere():
    # person zone: body 0.25 + min_distance 0.5; grown by 0.25 + 0.15 -> 1.15 m
    data = keepout_mask([person('p', 2.0, 1.0)], SMALL, PARAMS, robot_radius=0.25, margin=0.15)
    assert len(data) == SMALL.width * SMALL.height
    assert set(data) == {KEEPOUT, NO_INFORMATION}
    assert cell_value(data, SMALL, 2.0, 1.0) == KEEPOUT
    assert cell_value(data, SMALL, 2.0 + 1.05, 1.0) == KEEPOUT
    assert cell_value(data, SMALL, 2.0 + 1.25, 1.0) == NO_INFORMATION
    assert cell_value(data, SMALL, 0.05, 1.95) == NO_INFORMATION


def test_mask_without_persons_or_groups_holds_no_keepout():
    others = [Entity('crate', Category.OBSTACLE, 'crate', 1.0, 1.0),
              Entity('robot_2', Category.ROBOT, 'robot', 2.0, 1.0)]
    assert set(keepout_mask(others, SMALL, PARAMS, 0.25, 0.15)) == {NO_INFORMATION}


def test_mask_closes_the_gap_between_group_members():
    members = [person('a', 1.5, 0.0), person('b', 1.5, 0.8)]
    data = keepout_mask(members + [group('g', ['a', 'b'], 1.5, 0.4)], LAB, PARAMS, 0.25, 0.15)
    # 1.35 m around each member: body 0.25, min_distance 0.5, group margin 0.2, 0.25 + 0.15
    assert cell_value(data, LAB, 1.5 + 1.3, 0.4) == KEEPOUT
    assert cell_value(data, LAB, 1.5 + 1.4, 0.4) == NO_INFORMATION
    assert cell_value(data, LAB, 0.0, 0.0) == NO_INFORMATION     # the dock, 1.5 m away


def test_negative_margin_is_rejected():
    with pytest.raises(ValueError, match='margin'):
        keepout_mask([], SMALL, PARAMS, robot_radius=0.25, margin=-0.1)
    with pytest.raises(ValueError, match='margin'):
        SocialMask(PARAMS, robot_radius=0.25, margin=-0.1)


# --------------------------------------------------------------- publishing

def test_no_mask_before_the_map_grid_is_known():
    mask = SocialMask(PARAMS, robot_radius=0.25, margin=0.15)
    mask.apply_delta('local', [person('p', 2.0, 1.0)], [], now=0.0)
    assert mask.update(now=0.5) is None
    mask.set_grid(SMALL)
    data = mask.update(now=1.0)
    assert data is not None and cell_value(data, SMALL, 2.0, 1.0) == KEEPOUT
    assert mask.zone_count == 1


def test_an_empty_mask_goes_out_once_then_only_changes():
    mask = SocialMask(PARAMS, robot_radius=0.25, margin=0.15)
    mask.set_grid(SMALL)
    assert set(mask.update(now=0.0)) == {NO_INFORMATION}            # the filter gets a mask
    assert mask.update(now=0.5) is None
    mask.apply_delta('local', [person('p', 2.0, 1.0)], [], now=1.0)
    assert mask.update(now=1.0) is not None
    mask.apply_delta('local', [person('p', 2.0, 1.0)], [], now=1.5)  # re-observed, same cells
    assert mask.update(now=1.5) is None
    mask.apply_delta('local', [person('p', 2.5, 1.0)], [], now=2.0)  # walked on
    moved = mask.update(now=2.0)
    assert moved is not None and cell_value(moved, SMALL, 3.55, 1.0) == KEEPOUT
    mask.apply_delta('local', [], ['p'], now=2.5)                    # gone
    assert set(mask.update(now=2.5)) == {NO_INFORMATION} and mask.zone_count == 0


def test_an_expired_entity_clears_its_zone():
    mask = SocialMask(PARAMS, robot_radius=0.25, margin=0.15, stale_after_sec=10.0)
    mask.set_grid(SMALL)
    mask.apply_delta('local', [person('p', 2.0, 1.0)], [], now=0.0)
    assert mask.update(now=0.0) is not None
    assert mask.update(now=9.0) is None
    assert set(mask.update(now=10.5)) == {NO_INFORMATION}


def test_a_new_grid_rebuilds_the_mask():
    mask = SocialMask(PARAMS, robot_radius=0.25, margin=0.15)
    mask.set_grid(SMALL)
    mask.apply_delta('local', [person('p', 2.0, 1.0)], [], now=0.0)
    assert len(mask.update(now=0.0)) == SMALL.width * SMALL.height
    mask.set_grid(LAB)
    assert len(mask.update(now=0.5)) == LAB.width * LAB.height


@pytest.mark.parametrize('field, value', [('resolution', 0.0), ('width', -1), ('height', -1)])
def test_grid_validates(field, value):
    fields = dict(origin_x=0.0, origin_y=0.0, resolution=0.1, width=10, height=10)
    fields[field] = value
    with pytest.raises(ValueError, match=field):
        Grid(**fields)
