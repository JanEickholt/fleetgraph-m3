"""Keepout mask of one robot's social zones for Nav2's KeepoutFilter. Pure
Python (no rclpy, tests/unit/test_social_mask.py): social_mask_node feeds it
the robot's scene-graph reports and publishes update() as a
nav_msgs/OccupancyGrid on the static map's grid.

Entities come per source. Delta sources (the robot's own SceneObservation
deltas, 'local') upsert entities by id and delete removed_entity_ids; an
entity they stop reporting for stale_after_sec is dropped as well, since the
local graph re-reports every person it sees and removes one unseen for its
time-to-live, so only a lost removal or a restarted publisher leaves one
behind. Snapshot sources (the central replica, 'central') replace all their
entities at once and never expire. Zones (social.social_zones) come from the
union of all sources.

The mask holds KEEPOUT where the robot's centre would bring its footprint
(robot_radius) within margin of a zone (social.rasterize) and NO_INFORMATION
elsewhere, which the filter skips: 0 would mark unknown costmap cells free.
Nav2 applies filters after inflation, so the mask gets no inflation; margin
covers what separates the zones from the robot's own pose estimate (the
detections are placed with the scan-matched pose, Nav2 drives on AMCL's).
"""
from collections.abc import Iterable
from dataclasses import dataclass

from fleetgraph_robot.scene_graph import Entity
from fleetgraph_robot.social import SocialParams, rasterize, social_zones

KEEPOUT = 100           # OccupancyGrid value the KeepoutFilter turns into a lethal cell
NO_INFORMATION = -1     # the filter leaves the costmap cell as it is


@dataclass(frozen=True)
class Grid:
    """Cell layout of the static map (nav_msgs/MapMetaData without rotation):
    cell (0, 0) has its corner at the origin, rows run along +y."""
    origin_x: float
    origin_y: float
    resolution: float
    width: int
    height: int

    def __post_init__(self):
        checks = [('resolution', self.resolution > 0, '> 0'),
                  ('width', self.width >= 0, '>= 0'), ('height', self.height >= 0, '>= 0')]
        for name, ok, expected in checks:
            if not ok:
                raise ValueError(f'{name} must be {expected}, got {getattr(self, name)!r}')


class EntityStore:
    """The latest entities per source, each with the time it was last heard."""

    def __init__(self, stale_after_sec: float = 10.0):
        self._stale_after_sec = stale_after_sec     # 0: delta entities never expire
        self._deltas: dict[str, dict[str, tuple[Entity, float]]] = {}
        self._snapshots: dict[str, list[Entity]] = {}

    def apply_delta(self, source: str, entities: Iterable[Entity],
                    removed_ids: Iterable[str], now: float) -> None:
        table = self._deltas.setdefault(source, {})
        for entity in entities:
            table[entity.id] = (entity, now)
        for entity_id in removed_ids:
            table.pop(entity_id, None)

    def replace(self, source: str, entities: Iterable[Entity]) -> None:
        self._snapshots[source] = list(entities)

    def expire(self, now: float) -> bool:
        """Drop delta entities unheard for more than stale_after_sec; whether
        any were."""
        if not self._stale_after_sec > 0:
            return False
        expired = False
        for table in self._deltas.values():
            for entity_id, (_, heard) in list(table.items()):
                if now - heard > self._stale_after_sec:
                    del table[entity_id]
                    expired = True
        return expired

    def entities(self) -> list[Entity]:
        reported = [entity for table in self._deltas.values() for entity, _ in table.values()]
        return reported + [entity for entities in self._snapshots.values() for entity in entities]


def keepout_mask(entities: Iterable[Entity], grid: Grid, params: SocialParams,
                 robot_radius: float, margin: float) -> list[int]:
    """Row-major OccupancyGrid data: KEEPOUT within robot_radius + margin of
    a social zone, NO_INFORMATION elsewhere."""
    return _mask(social_zones(entities, params), grid, robot_radius, margin)


def _mask(zones, grid, robot_radius, margin):
    _check_margin(margin)
    marked = rasterize(zones, grid.origin_x, grid.origin_y, grid.resolution, grid.width,
                       grid.height, value=KEEPOUT, robot_radius=robot_radius + margin)
    return [cell if cell else NO_INFORMATION for cell in marked]


def _check_margin(margin):
    if not margin >= 0:
        raise ValueError(f'margin must be >= 0, got {margin!r}')


class SocialMask:
    """The mask to publish: rebuilt when entities or the grid changed, handed
    out only when its cells differ from the last one handed out."""

    def __init__(self, params: SocialParams, robot_radius: float, margin: float,
                 stale_after_sec: float = 10.0):
        _check_margin(margin)
        self.params = params
        self.robot_radius = robot_radius
        self.margin = margin
        self.store = EntityStore(stale_after_sec)
        self.zone_count = 0
        self._grid = None
        self._published = None
        self._dirty = True

    def set_grid(self, grid: Grid) -> None:
        self._grid, self._published, self._dirty = grid, None, True

    def apply_delta(self, source: str, entities: Iterable[Entity],
                    removed_ids: Iterable[str], now: float) -> None:
        self.store.apply_delta(source, entities, removed_ids, now)
        self._dirty = True

    def replace(self, source: str, entities: Iterable[Entity]) -> None:
        self.store.replace(source, entities)
        self._dirty = True

    def update(self, now: float) -> list[int] | None:
        """The new mask, or None while it is unchanged or no grid is known."""
        if self.store.expire(now):
            self._dirty = True
        if self._grid is None or not self._dirty:
            return None
        self._dirty = False
        zones = social_zones(self.store.entities(), self.params)
        self.zone_count = len(zones)
        data = _mask(zones, self._grid, self.robot_radius, self.margin)
        if data == self._published:
            return None
        self._published = data
        return data
