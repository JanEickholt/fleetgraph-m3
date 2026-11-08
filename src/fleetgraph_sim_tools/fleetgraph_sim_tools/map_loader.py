"""Nav2 map files (map_server yaml + binary PGM) to a lidar_obstacles
OccupancyMap equal, cell for cell, to what nav2_map_server publishes. Pure
Python (no rclpy; tests/unit/test_map_loader.py).

Trinary mode as in nav2_map_server: occ = 1 - pixel / maxval (pixel / maxval
with negate); occupied (100) above occupied_thresh, free (0) below
free_thresh, unknown (-1) in between, both thresholds from the yaml. The
maps of tools/gen_world.py carry free_thresh 0.196 (the ROS 1 map_saver's
value): their 205 border pixels give occ 0.196078 and read as unknown, as
in map_server.
"""
import os

import yaml

from fleetgraph_robot.lidar_obstacles import OccupancyMap

OCCUPIED, FREE, UNKNOWN = 100, 0, -1


def load(yaml_path) -> OccupancyMap:
    """The map of a map_server yaml; a relative image path is relative to
    the yaml's directory, as in map_server."""
    with open(yaml_path) as file:
        map_yaml = yaml.safe_load(file)
    if not isinstance(map_yaml, dict) or not isinstance(map_yaml.get('image'), str):
        raise ValueError(f'{yaml_path}: image: required path of the PGM file')
    image = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), map_yaml['image'])
    with open(image, 'rb') as file:
        return from_nav2(map_yaml, file.read())


def from_nav2(map_yaml: dict, pgm: bytes) -> OccupancyMap:
    mode = map_yaml.get('mode', 'trinary')
    if mode != 'trinary':
        raise ValueError(f'map yaml: mode: only trinary is supported, got {mode!r}')
    resolution = map_yaml.get('resolution')
    if not isinstance(resolution, (int, float)) or not resolution > 0:
        raise ValueError(f'map yaml: resolution: must be a number > 0, got {resolution!r}')
    origin = map_yaml.get('origin')
    if not isinstance(origin, list) or len(origin) != 3:
        raise ValueError(f'map yaml: origin: must be [x, y, yaw], got {origin!r}')
    origin_x, origin_y, yaw = (float(value) for value in origin)
    if yaw != 0.0:
        raise ValueError(f'map yaml: origin: rotated maps are not supported, yaw {yaw}')
    negate = bool(map_yaml.get('negate', 0))
    occupied_thresh = float(map_yaml.get('occupied_thresh', 0.65))
    free_thresh = float(map_yaml.get('free_thresh', 0.25))
    width, height, maxval, pixels = parse_pgm(pgm)
    data = []
    for row in reversed(range(height)):     # file rows run from max y down
        for pixel in pixels[row * width:(row + 1) * width]:
            shade = pixel / maxval
            occ = shade if negate else 1.0 - shade
            if occ > occupied_thresh:
                data.append(OCCUPIED)
            elif occ < free_thresh:
                data.append(FREE)
            else:
                data.append(UNKNOWN)
    return OccupancyMap(float(resolution), origin_x, origin_y, width, height, data)


def parse_pgm(data: bytes) -> tuple[int, int, int, bytes]:
    """(width, height, maxval, pixels) of a binary 8-bit PGM (P5), pixel rows
    top first. Header fields may be separated by comments (# to line end)."""
    fields, position = [], 0
    while len(fields) < 4:
        position = _skip_space_and_comments(data, position)
        start = position
        while position < len(data) and not data[position:position + 1].isspace():
            position += 1
        if start == position:
            raise ValueError('PGM header is truncated')
        fields.append(data[start:position])
    magic, width, height, maxval = fields
    if magic != b'P5':
        raise ValueError(f'only binary PGM (P5) is supported, got {magic!r}')
    width, height, maxval = int(width), int(height), int(maxval)
    if not 0 < maxval < 256:
        raise ValueError(f'PGM maxval must be 1..255 (one byte per pixel), got {maxval}')
    pixels = data[position + 1:]            # one whitespace byte ends the header
    if len(pixels) != width * height:
        raise ValueError(f'PGM has {len(pixels)} pixels, header says {width} x {height}')
    return width, height, maxval, pixels


def _skip_space_and_comments(data, position):
    while position < len(data):
        if data[position:position + 1] == b'#':
            line_end = data.find(b'\n', position)
            position = len(data) if line_end < 0 else line_end + 1
        elif data[position:position + 1].isspace():
            position += 1
        else:
            break
    return position
