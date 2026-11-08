"""Social-navigation constraints around people. Pure Python (no rclpy): a
ROS node turns social_zones() of the current scene graph into a Nav2
keepout-filter mask with rasterize(), checks the active plan with
path_conflicts() to decide whether to wait or replan, and scores runs with
min_person_distance() and violations() (tests/unit/test_social.py).

A person is a dynamic obstacle wrapped in a disc of personal space: its
body plus min_distance. A group is one socially occupied region, the
convex hull of its members' discs grown by group_margin, so no route leads
between members. Discs become circumscribed polygons, which never undercut
the configured distance. The mask, conflicts and metrics test these very
polygons with one footprint rule: the robot's disc of robot_radius must not
touch a zone.
"""
import math
from collections.abc import Callable, Iterable, Sequence
from dataclasses import dataclass

from fleetgraph_robot.scene_graph import Category, Entity

Point = tuple[float, float]


@dataclass(frozen=True)
class SocialParams:
    """Distances in m."""
    min_distance: float = 0.5       # kept clear around a person's body
    person_radius: float = 0.25     # body radius when the entity size is unknown
    group_margin: float = 0.2       # extra clearance around a group's region
    disc_segments: int = 16         # polygon edges per disc

    def __post_init__(self):
        checks = [
            ('min_distance', self.min_distance >= 0, '>= 0'),
            ('person_radius', self.person_radius > 0, '> 0'),
            ('group_margin', self.group_margin >= 0, '>= 0'),
            ('disc_segments', isinstance(self.disc_segments, int) and self.disc_segments >= 3,
             'an int >= 3'),
        ]
        for name, ok, expected in checks:
            if not ok:
                raise ValueError(f'{name} must be {expected}, got {getattr(self, name)!r}')


@dataclass(frozen=True)
class Zone:
    kind: str                       # 'person' | 'group'
    entity_id: str
    polygon: tuple[Point, ...]      # convex, counter-clockwise, map frame

    def contains(self, x: float, y: float) -> bool:
        """The boundary counts as inside. Fewer than 3 corners enclose
        nothing (every point passes the edge test of an empty polygon)."""
        if len(self.polygon) < 3:
            return False
        for (ax, ay), (bx, by) in _edges(self.polygon):
            if (bx - ax) * (y - ay) - (by - ay) * (x - ax) < 0:
                return False
        return True

    def distance(self, x: float, y: float) -> float:
        """0 inside, inf for an empty polygon."""
        if self.contains(x, y):
            return 0.0
        return min((_point_segment_distance(x, y, a, b) for a, b in _edges(self.polygon)),
                   default=math.inf)


def social_zones(entities: Iterable[Entity], params: SocialParams | None = None) -> list[Zone]:
    """One zone per PERSON and per GROUP, in input order. Group members are
    looked up by member_ids among `entities` and keep their own zones."""
    params = params if params is not None else SocialParams()
    entities = list(entities)
    by_id = {entity.id: entity for entity in entities}
    segments = params.disc_segments
    zones = []
    for entity in entities:
        if entity.category == Category.PERSON:
            radius = _body_radius(entity, params) + params.min_distance
            zones.append(Zone('person', entity.id, _disc(entity.x, entity.y, radius, segments)))
        elif entity.category == Category.GROUP:
            reach = params.min_distance + params.group_margin
            members = [by_id[member_id] for member_id in entity.member_ids if member_id in by_id]
            if members:
                discs = [(member.x, member.y, _body_radius(member, params) + reach)
                         for member in members]
            else:
                # the size box spans every member footprint: cover all of it
                half_diagonal = math.hypot(entity.size[0], entity.size[1]) / 2
                body = half_diagonal if half_diagonal > 0 else params.person_radius
                discs = [(entity.x, entity.y, body + reach)]
            corners = [corner for disc in discs for corner in _disc(*disc, segments)]
            zones.append(Zone('group', entity.id, _convex_hull(corners)))
    return zones


def path_conflicts(path: Sequence[Point], zones: Iterable[Zone],
                   robot_radius: float = 0.0) -> list[tuple[int, Zone]]:
    """(index, zone) for each zone the path runs into: the first path point
    whose footprint (disc of robot_radius) touches the zone, or the first
    point whose segment to the next one touches it while both ends stay
    clear. Sorted by index, ties in zone order."""
    _check_radius(robot_radius)
    points = list(path)
    conflicts = []
    for zone in zones:
        index = _first_conflict(zone, points, robot_radius)
        if index is not None:
            conflicts.append((index, zone))
    conflicts.sort(key=lambda conflict: conflict[0])
    return conflicts


def _first_conflict(zone, points, reach):
    xs, ys = [x for x, _ in zone.polygon], [y for _, y in zone.polygon]
    xmin, xmax, ymin, ymax = min(xs) - reach, max(xs) + reach, min(ys) - reach, max(ys) + reach
    touching = [xmin <= x <= xmax and ymin <= y <= ymax and zone.distance(x, y) <= reach
                for x, y in points]
    for index in range(len(points)):
        if touching[index]:
            return index
        if index + 1 < len(points) and not touching[index + 1]:
            (ax, ay), (bx, by) = start, end = points[index], points[index + 1]
            if (min(ax, bx) <= xmax and max(ax, bx) >= xmin and min(ay, by) <= ymax
                    and max(ay, by) >= ymin and _segment_gap(zone, start, end) <= reach):
                return index
    return None


def _segment_gap(zone, start, end):
    """Distance between a segment and the zone, 0 when they meet."""
    if _clips(zone.polygon, start, end):
        return 0.0
    return min(zone.distance(*start), zone.distance(*end),
               *(_point_segment_distance(x, y, start, end) for x, y in zone.polygon))


def _clips(polygon, start, end):
    """Cyrus-Beck: whether part of the segment lies inside the polygon."""
    (ax, ay), (bx, by) = start, end
    dx, dy = bx - ax, by - ay
    low, high = 0.0, 1.0
    for (px, py), (qx, qy) in _edges(polygon):
        ex, ey = qx - px, qy - py
        inner = ex * (ay - py) - ey * (ax - px)     # >= 0: start on the inner side
        rate = ex * dy - ey * dx
        if rate == 0:
            if inner < 0:
                return False
        elif rate > 0:
            low = max(low, -inner / rate)
        else:
            high = min(high, -inner / rate)
        if low > high:
            return False
    return True


def rasterize(zones: Iterable[Zone], origin_x: float, origin_y: float, resolution: float,
              width: int, height: int, value: int = 100,
              robot_radius: float = 0.0) -> list[int]:
    """Row-major nav_msgs/OccupancyGrid data, row 0 at origin_y: `value`
    where a cell centre lies within robot_radius of a zone (in it for 0), 0
    elsewhere. Nav2 applies costmap filters after the inflation layer, so a
    keepout mask gets no inflation: the margin keeps the robot's footprint,
    not just its centre, out of every zone. Scanline fill over each zone's
    own rows only."""
    if not resolution > 0:
        raise ValueError(f'resolution must be > 0, got {resolution!r}')
    if width < 0 or height < 0:
        raise ValueError(f'width and height must be >= 0, got {width} x {height}')
    _check_radius(robot_radius)
    data = [0] * (width * height)
    for zone in zones:
        edges = list(_edges(zone.polygon))
        ys = [y for _, y in zone.polygon]
        first_row = max(0, math.ceil((min(ys) - robot_radius - origin_y) / resolution - 0.5))
        last_row = min(height - 1,
                       math.floor((max(ys) + robot_radius - origin_y) / resolution - 0.5))
        for row in range(first_row, last_row + 1):
            y = origin_y + (row + 0.5) * resolution
            low, high = _row_span(edges, y, robot_radius)
            if low > high:          # rounding at the zone's top or bottom
                continue
            first_col = max(0, math.ceil((low - origin_x) / resolution - 0.5))
            last_col = min(width - 1, math.floor((high - origin_x) / resolution - 0.5))
            if first_col <= last_col:
                start = row * width
                data[start + first_col:start + last_col + 1] = [value] * (last_col - first_col + 1)
    return data


def _row_span(edges, y, reach):
    """(low, high) x of the points on the line at height y at most `reach`
    from the convex polygon with these edges; low > high when there are none.
    Those points are the polygon, a disc around each corner and a band along
    each edge. Together they are convex, so the outermost ends are exact."""
    low, high = math.inf, -math.inf
    for (ax, ay), (bx, by) in edges:
        if y < min(ay, by) - reach or y > max(ay, by) + reach:
            continue
        if ay == by:
            if ay == y:
                low, high = min(low, ax, bx), max(high, ax, bx)
        elif min(ay, by) <= y <= max(ay, by):
            x = ax + (y - ay) * (bx - ax) / (by - ay)
            low, high = min(low, x), max(high, x)
        if reach > 0:
            if abs(y - ay) <= reach:
                half = math.sqrt(reach * reach - (y - ay) ** 2)
                low, high = min(low, ax - half), max(high, ax + half)
            start, end = _band_span(ax, ay, bx, by, y, reach)
            if start <= end:
                low, high = min(low, start), max(high, end)
    return low, high


def _band_span(ax, ay, bx, by, y, reach):
    """(start, end) x on the line at height y of the points at most `reach`
    from the line through a and b whose projection falls between a and b;
    start > end when there are none."""
    dx, dy = bx - ax, by - ay
    length = math.hypot(dx, dy)
    if length == 0:
        return math.inf, -math.inf
    v = y - ay
    # with u = x - ax: distance from the line (u * dy - v * dx) / length in
    # [-reach, reach], projection (u * dx + v * dy) / length in [0, length]
    across = _linear_span(dy, -v * dx, -reach * length, reach * length)
    along = _linear_span(dx, v * dy, 0.0, length * length)
    return ax + max(across[0], along[0]), ax + min(across[1], along[1])


def _linear_span(slope, offset, low, high):
    """(first, last) u with low <= slope * u + offset <= high; first > last
    when there is none."""
    if slope == 0:
        return (-math.inf, math.inf) if low <= offset <= high else (math.inf, -math.inf)
    first, last = (low - offset) / slope, (high - offset) / slope
    return min(first, last), max(first, last)


def min_person_distance(x: float, y: float, entities: Iterable[Entity],
                        params: SocialParams | None = None) -> float:
    """Distance from a point to the nearest PERSON body surface, 0 inside a
    body; inf without persons."""
    params = params if params is not None else SocialParams()
    return min((max(0.0, math.hypot(entity.x - x, entity.y - y) - _body_radius(entity, params))
                for entity in entities if entity.category == Category.PERSON), default=math.inf)


def violations(track: Sequence[tuple[float, float, float]],
               entities_at: Callable[[float], list[Entity]], robot_radius: float,
               params: SocialParams | None = None) -> list[tuple[float, str, str]]:
    """(t, kind, entity_id) each time the robot footprint (disc of
    robot_radius) at track sample (t, x, y) starts touching a zone: one
    entry per stay, whether the robot or the people moved."""
    _check_radius(robot_radius)
    entries, touched = [], set()
    for t, x, y in track:
        touching = set()
        for zone in social_zones(entities_at(t), params):
            key = (zone.kind, zone.entity_id)
            if key not in touching and zone.distance(x, y) <= robot_radius:
                touching.add(key)
                if key not in touched:
                    entries.append((t, zone.kind, zone.entity_id))
        touched = touching
    return entries


def _check_radius(robot_radius):
    if not robot_radius >= 0:
        raise ValueError(f'robot_radius must be >= 0, got {robot_radius!r}')


def _body_radius(entity, params):
    radius = max(entity.size[0], entity.size[1]) / 2
    return radius if radius > 0 else params.person_radius


def _disc(x, y, radius, segments):
    """Circumscribed regular polygon: its edges touch the circle at their
    midpoints, one of them facing +x."""
    corner = radius / math.cos(math.pi / segments)
    angles = (2 * math.pi * (k + 0.5) / segments for k in range(segments))
    return tuple((x + corner * math.cos(angle), y + corner * math.sin(angle)) for angle in angles)


def _convex_hull(points):
    """Andrew's monotone chain; counter-clockwise, collinear points dropped."""
    points = sorted(set(points))
    if len(points) < 3:
        return tuple(points)

    def chain(sequence):
        hull = []
        for point in sequence:
            while len(hull) >= 2 and _cross(hull[-2], hull[-1], point) <= 0:
                hull.pop()
            hull.append(point)
        return hull

    lower, upper = chain(points), chain(reversed(points))
    return tuple(lower[:-1] + upper[:-1])


def _cross(origin, a, b):
    return (a[0] - origin[0]) * (b[1] - origin[1]) - (a[1] - origin[1]) * (b[0] - origin[0])


def _edges(polygon):
    return zip(polygon, polygon[1:] + polygon[:1])


def _point_segment_distance(x, y, a, b):
    (ax, ay), (bx, by) = a, b
    dx, dy = bx - ax, by - ay
    length_squared = dx * dx + dy * dy
    if length_squared == 0:
        t = 0.0
    else:
        t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / length_squared))
    return math.hypot(x - (ax + t * dx), y - (ay + t * dy))
