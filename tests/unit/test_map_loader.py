"""Nav2 map files to OccupancyMap (fleetgraph_sim_tools.map_loader). Pure: no
rclpy. The grid must equal what nav2_map_server publishes for the same files
(trinary mode), which was checked on the lab-floor map: map_server reads its
205 border pixels as free, because 1 - 205/255 = 0.196 < free_thresh 0.25.
"""
import os

import pytest

from fleetgraph_sim_tools import map_loader

PACKAGE = os.path.join(os.path.dirname(os.path.abspath(map_loader.__file__)), '..')
LAB_MAP = os.path.join(PACKAGE, '..', 'fleetgraph_sim', 'maps', 'lab_floor.yaml')
FREE, OCCUPIED, UNKNOWN = 0, 100, -1
MAP_YAML = {'image': 'tiny.pgm', 'mode': 'trinary', 'resolution': 0.5,
            'origin': [-1.0, 2.0, 0.0], 'negate': 0, 'occupied_thresh': 0.65,
            'free_thresh': 0.25}


def pgm(rows, maxval=255, header=None):
    """Binary PGM, rows listed top (max y) first as in the file."""
    height, width = len(rows), len(rows[0])
    header = header or f'P5\n# made by test_map_loader\n{width} {height}\n{maxval}\n'
    return header.encode() + bytes(pixel for row in rows for pixel in row)


def test_pixels_map_like_map_server_and_rows_flip_to_grid_order():
    grid = map_loader.from_nav2(MAP_YAML, pgm([[0, 254, 205], [128, 230, 100]]))
    assert (grid.resolution, grid.origin_x, grid.origin_y, grid.width, grid.height) == \
        (0.5, -1.0, 2.0, 3, 2)
    # grid row 0 is the bottom file row; 205 -> occ 0.196 < 0.25 -> free,
    # 128 -> occ 0.498 -> unknown, 100 -> occ 0.608 -> unknown
    assert list(grid.data) == [UNKNOWN, FREE, UNKNOWN, OCCUPIED, FREE, FREE]
    assert grid.is_occupied(-0.9, 2.6) and not grid.is_occupied(-0.4, 2.6)


def test_negate_reads_dark_pixels_as_free():
    grid = map_loader.from_nav2(dict(MAP_YAML, negate=1), pgm([[0, 254]]))
    assert list(grid.data) == [FREE, OCCUPIED]


def test_thresholds_come_from_the_yaml_and_scale_with_maxval():
    strict = dict(MAP_YAML, occupied_thresh=0.9, free_thresh=0.05)
    # maxval 100: pixel 50 -> occ 0.5 (unknown), 5 -> 0.95 (occupied), 97 -> 0.03 (free)
    assert list(map_loader.from_nav2(strict, pgm([[50, 5, 97]], maxval=100)).data) == \
        [UNKNOWN, OCCUPIED, FREE]


def test_header_comments_and_whitespace_are_skipped():
    data = pgm([[0, 254]], header='P5 # magic\n2\t1\n# between\n255\n')
    assert list(map_loader.from_nav2(MAP_YAML, data).data) == [OCCUPIED, FREE]


@pytest.mark.parametrize('changes, data, word', [
    ({}, b'P2\n1 1\n255\n0', 'P5'),
    ({}, pgm([[0]], header='P5\n1 1\n65535\n'), 'maxval'),
    ({}, pgm([[0, 0]], header='P5\n3 1\n255\n'), 'pixels'),
    ({'origin': [0.0, 0.0, 0.3]}, pgm([[0]]), 'origin'),
    ({'mode': 'scale'}, pgm([[0]]), 'mode'),
    ({'resolution': None}, pgm([[0]]), 'resolution'),
])
def test_unsupported_or_broken_input_is_rejected(changes, data, word):
    map_yaml = {key: value for key, value in dict(MAP_YAML, **changes).items()
                if value is not None}
    with pytest.raises(ValueError, match=word):
        map_loader.from_nav2(map_yaml, data)


def test_lab_floor_map_has_the_walls_and_furniture_but_not_the_crate():
    grid = map_loader.load(LAB_MAP)          # image path is relative to the yaml
    assert (grid.width, grid.height, grid.resolution, grid.origin_x, grid.origin_y) == \
        (220, 140, 0.05, -1.5, -3.5)
    assert set(grid.data) == {FREE, OCCUPIED}
    for x, y in ((4.0, 0.5), (4.0, 2.5), (-1.0, 0.0), (1.5, 2.6), (7.5, 2.0)):
        assert grid.is_occupied(x, y), (x, y)         # walls, shelf, lab table
    for x, y in ((0.5, 0.5), (2.5, 0.8), (4.0, 1.5), (-1.4, -3.4)):
        assert not grid.is_occupied(x, y), (x, y)     # hall, crate, door, outside
    assert grid.line_of_sight(3.0, 1.5, 6.0, 1.5)            # through door_lab1
    assert not grid.line_of_sight(3.0, 0.0, 6.0, 0.5)        # through the wall
