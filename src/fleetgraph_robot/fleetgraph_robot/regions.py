"""Named regions of the floor plan as axis-aligned boxes in the map frame:
places (rooms, docks) and traversables (doors, corridors). Pure Python; the
local scene graph (scene_graph.py) turns them into PLACE / TRAVERSABLE nodes
and tests entity footprints against them. Unit-tested in
tests/unit/test_regions.py.
"""
import math
from collections.abc import Iterable
from dataclasses import dataclass

KINDS = ('place', 'traversable')
AXES = ('x', 'y')
_BOUNDS = ('xmin', 'ymin', 'xmax', 'ymax')
_REQUIRED_KEYS = ('id', 'kind') + _BOUNDS
_OPTIONAL_KEYS = ('label', 'travel_axis')

Disc = tuple[float, float, float]            # x, y, radius
Box = tuple[float, float, float, float]      # xmin, ymin, xmax, ymax
# Overlaps thinner than this count as touching, so float noise in
# coordinates like 3.8 - 3.5 cannot flip an intersection test.
_EPSILON = 1e-9


@dataclass(frozen=True)
class Region:
    id: str
    kind: str                # 'place' | 'traversable'
    xmin: float
    ymin: float
    xmax: float
    ymax: float
    label: str = ''
    # traversables: 'x' | 'y', the direction robots pass through; the cross
    # axis is the passage width
    travel_axis: str = ''

    def __post_init__(self):
        problem = self._problem()
        if problem:
            raise ValueError(f'region {self.id!r}: {problem}')

    def _problem(self):
        if not isinstance(self.id, str) or not self.id:
            return 'id must be a non-empty string'
        if '/' in self.id:
            return "id must not contain '/' (reserved for track ids)"
        if self.kind not in KINDS:
            return f'kind must be one of {KINDS}, got {self.kind!r}'
        if not isinstance(self.label, str):
            return f'label must be a string, got {self.label!r}'
        for name in _BOUNDS:
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                return f'{name} must be a number, got {value!r}'
            if not math.isfinite(value):
                return f'{name} must be finite, got {value!r}'
        if not self.xmin < self.xmax:
            return f'needs xmin < xmax, got {self.xmin} >= {self.xmax}'
        if not self.ymin < self.ymax:
            return f'needs ymin < ymax, got {self.ymin} >= {self.ymax}'
        if self.kind == 'traversable' and self.travel_axis not in AXES:
            return f'traversable needs travel_axis in {AXES}, got {self.travel_axis!r}'
        if self.kind == 'place' and self.travel_axis:
            return 'travel_axis only applies to traversables'
        return ''

    def contains(self, x: float, y: float) -> bool:
        return self.xmin <= x <= self.xmax and self.ymin <= y <= self.ymax

    def center(self) -> tuple[float, float]:
        return ((self.xmin + self.xmax) / 2, (self.ymin + self.ymax) / 2)

    def size(self) -> tuple[float, float]:
        return (self.xmax - self.xmin, self.ymax - self.ymin)

    def distance_to_point(self, x: float, y: float) -> float:
        dx = max(self.xmin - x, 0.0, x - self.xmax)
        dy = max(self.ymin - y, 0.0, y - self.ymax)
        return math.hypot(dx, dy)

    def distance_to_disc(self, x: float, y: float, r: float) -> float:
        """Clearance between the disc and the box; 0 when they intersect."""
        return max(0.0, self.distance_to_point(x, y) - r)

    def intersects_disc(self, x: float, y: float, r: float) -> bool:
        """Interiors overlap; a disc that only touches the boundary does not."""
        return self.distance_to_point(x, y) < r - _EPSILON

    def intersects_box(self, xmin: float, ymin: float, xmax: float, ymax: float) -> bool:
        return (xmin < self.xmax - _EPSILON and xmax > self.xmin + _EPSILON and
                ymin < self.ymax - _EPSILON and ymax > self.ymin + _EPSILON)

    def free_gap(self, discs: Iterable[Disc], boxes: Iterable[Box] = ()) -> float:
        """Widest free interval across a traversable, left by the discs and
        boxes that intersect it. Each footprint blocks its cross-axis shadow
        inside the region wherever it sits along the travel axis: exact for
        doors, conservative for long corridors (staggered obstacles a robot
        could slalom between count as one blockage)."""
        if self.kind != 'traversable':
            raise ValueError(f'free_gap needs a traversable, {self.id!r} is a {self.kind}')
        travel_is_x = self.travel_axis == 'x'
        travel_low, travel_high = (self.xmin, self.xmax) if travel_is_x else (self.ymin, self.ymax)
        cross_low, cross_high = (self.ymin, self.ymax) if travel_is_x else (self.xmin, self.xmax)
        shadows = []
        for x, y, r in discs:
            if not self.intersects_disc(x, y, r):
                continue
            along, across = (x, y) if travel_is_x else (y, x)
            # widest chord of the disc inside the travel slab: at its centre,
            # or at the slab face nearest to it
            outside = max(travel_low - along, 0.0, along - travel_high)
            half_width = math.sqrt(r * r - outside * outside)
            shadows.append((across - half_width, across + half_width))
        for xmin, ymin, xmax, ymax in boxes:
            if self.intersects_box(xmin, ymin, xmax, ymax):
                shadows.append((ymin, ymax) if travel_is_x else (xmin, xmax))
        return _widest_free_interval(cross_low, cross_high, shadows)


def _widest_free_interval(low: float, high: float, blocked: list[tuple[float, float]]) -> float:
    widest, cursor = 0.0, low
    for start, end in sorted(blocked):
        if start > cursor:
            widest = max(widest, min(start, high) - cursor)
        cursor = max(cursor, end)
        if cursor >= high:
            break
    return max(widest, high - cursor)


def load_regions(spec: dict) -> list[Region]:
    """Already-parsed spec {'regions': [{id, kind, xmin, ymin, xmax, ymax,
    label?, travel_axis?}, ...]} -> regions; ValueError names the offending
    entry and field."""
    if not isinstance(spec, dict):
        raise ValueError(f'region spec must be a dict, got {type(spec).__name__}')
    if 'regions' not in spec:
        raise ValueError("region spec needs a 'regions' list")
    entries = spec['regions']
    if not isinstance(entries, list):
        raise ValueError(f"'regions' must be a list, got {type(entries).__name__}")
    regions, seen = [], set()
    for index, entry in enumerate(entries):
        where = f'regions[{index}]'
        if not isinstance(entry, dict):
            raise ValueError(f'{where} must be a dict, got {type(entry).__name__}')
        where = f"{where} ({entry.get('id', '?')!r})"
        missing = [key for key in _REQUIRED_KEYS if key not in entry]
        if missing:
            raise ValueError(f'{where}: missing {missing}')
        unknown = sorted(set(entry) - set(_REQUIRED_KEYS) - set(_OPTIONAL_KEYS))
        if unknown:
            raise ValueError(f'{where}: unknown keys {unknown}')
        for name in _BOUNDS:
            value = entry[name]
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f'{where}: {name} must be a number, got {value!r}')
        region = Region(entry['id'], entry['kind'],
                        *(float(entry[name]) for name in _BOUNDS),
                        label=entry.get('label', ''),
                        travel_axis=entry.get('travel_axis', ''))
        if region.id in seen:
            raise ValueError(f'{where}: duplicate id')
        seen.add(region.id)
        regions.append(region)
    return regions
