"""Scene scripts: reproducible people and obstacles on the Gazebo lab floor.
Pure Python (no rclpy; tests/unit/test_scene_script.py). scene_director plays
a script in Gazebo and publishes its ground truth on TOPIC_TRUTH.

Scene file (YAML; shipped ones in share/fleetgraph_sim_tools/scenes/):

  name: door_lab1_box_temporary
  entities:
    - id: door_box                # unique, [A-Za-z0-9_-]; Gazebo model "scene_door_box"
      category: obstacle          # person | obstacle | object
      label: box
      shape: {type: box, size: [0.6, 0.6, 0.5]}       # x, y, z extent in m, or
                                  # {type: cylinder, radius: 0.25, height: 1.7}
      spawn: 10                   # optional, default 0
      remove: 50                  # optional; absent = stays
      path:                       # waypoints {t, x, y, yaw?}, map frame, m and rad
        - {t: 0, x: 4.0, y: 1.5}
    - id: crate
      category: obstacle
      label: crate
      shape: {type: box, size: [0.4, 0.4, 0.5]}
      preexisting: true           # optional: already part of the world SDF
      path: [{t: 0, x: 2.5, y: 0.8}]

Times (spawn, remove, t) are sim seconds after the director's first /clock
tick; an entity exists while spawn <= t < remove. One waypoint is static;
more are piecewise-linear motion with strictly increasing t (yaw along the
shorter arc, default 0), held at the first waypoint before it and at the
last after it. x, y is the footprint centre and models stand on the floor,
so truth reports z = height / 2. A preexisting entity is truth only, never
spawned, moved or removed: spawn 0, no remove, one waypoint. Unknown fields
are errors. Validation errors raise ValueError naming the entity and field.
"""
import math
import re
from dataclasses import dataclass

import yaml

from fleetgraph_robot.scene_graph import Category
from fleetgraph_sim_tools.detector_model import TruthEntity

TOPIC_TRUTH = '/sim/truth'
MODEL_PREFIX = 'scene_'
CATEGORIES = {'person': Category.PERSON, 'obstacle': Category.OBSTACLE,
              'object': Category.OBJECT}
COLORS = {Category.PERSON: '0.2 0.4 0.8 1', Category.OBSTACLE: '0.8 0.3 0.1 1',
          Category.OBJECT: '0.2 0.6 0.3 1'}
# ids also go into SDF model names: no markup characters
_ID_PATTERN = re.compile(r'[A-Za-z0-9_-]+')
_SCENE_FIELDS = {'name', 'entities'}
_ENTITY_FIELDS = {'id', 'category', 'label', 'shape', 'spawn', 'remove', 'path', 'preexisting'}
_SHAPE_FIELDS = {'cylinder': {'type', 'radius', 'height'}, 'box': {'type', 'size'}}
_WAYPOINT_FIELDS = {'t', 'x', 'y', 'yaw'}

Pose = tuple[float, float, float]           # x, y, yaw in the map frame
Size = tuple[float, float, float]


@dataclass(frozen=True)
class Waypoint:
    t: float
    x: float
    y: float
    yaw: float = 0.0


@dataclass(frozen=True)
class Shape:
    type: str                               # 'cylinder' or 'box'
    size: Size                              # extent; cylinder: 2 radius, 2 radius, height


@dataclass(frozen=True)
class ScriptedEntity:
    id: str
    category: int
    label: str
    shape: Shape
    path: tuple[Waypoint, ...]
    spawn: float = 0.0
    remove: float | None = None
    preexisting: bool = False

    @property
    def model_name(self) -> str:
        return MODEL_PREFIX + self.id

    def active(self, t: float) -> bool:
        return self.spawn <= t and (self.remove is None or t < self.remove)

    def pose(self, t: float) -> Pose:
        start, end, fraction = self._segment(t)
        turn = math.remainder(end.yaw - start.yaw, 2 * math.pi)
        return (start.x + fraction * (end.x - start.x), start.y + fraction * (end.y - start.y),
                start.yaw + fraction * turn)

    def velocity(self, t: float) -> tuple[float, float]:
        start, end, _ = self._segment(t)
        if start is end:
            return 0.0, 0.0
        duration = end.t - start.t
        return (end.x - start.x) / duration, (end.y - start.y) / duration

    def truth(self, t: float) -> TruthEntity:
        x, y, yaw = self.pose(t)
        vx, vy = self.velocity(t)
        size_x, size_y, size_z = self.shape.size
        if self.shape.type == 'box':        # axis-aligned extent of the turned box
            cos_yaw, sin_yaw = abs(math.cos(yaw)), abs(math.sin(yaw))
            size_x, size_y = (size_x * cos_yaw + size_y * sin_yaw,
                              size_x * sin_yaw + size_y * cos_yaw)
        return TruthEntity(self.id, self.category, self.label, x, y, z=size_z / 2,
                           size=(size_x, size_y, size_z), vx=vx, vy=vy)

    def _segment(self, t):
        """(start, end, fraction) of the path at t; start is end while held."""
        if t < self.path[0].t:
            return self.path[0], self.path[0], 0.0
        for start, end in zip(self.path, self.path[1:]):
            if t < end.t:
                return start, end, (t - start.t) / (end.t - start.t)
        return self.path[-1], self.path[-1], 0.0


@dataclass(frozen=True)
class Scene:
    name: str
    entities: tuple[ScriptedEntity, ...]

    def truth(self, t: float) -> list[TruthEntity]:
        """The entities active at t, in scene order."""
        return [entity.truth(t) for entity in self.entities if entity.active(t)]


def load(path) -> Scene:
    with open(path) as file:
        return parse(yaml.safe_load(file))


def parse(data) -> Scene:
    if not isinstance(data, dict):
        raise ValueError(f'scene: must be a mapping with name and entities, got {data!r}')
    _check_fields('scene', data, _SCENE_FIELDS)
    name = data.get('name')
    if not isinstance(name, str) or not name:
        raise ValueError(f'scene: name: required non-empty string, got {name!r}')
    if not isinstance(data.get('entities'), list):
        raise ValueError(f"scene: entities: must be a list, got {data.get('entities')!r}")
    entities = []
    for index, item in enumerate(data['entities']):
        entity = _parse_entity(index, item)
        if any(other.id == entity.id for other in entities):
            raise ValueError(f'entity {entity.id!r}: id: duplicate')
        entities.append(entity)
    return Scene(name, tuple(entities))


def model_sdf(entity: ScriptedEntity) -> str:
    """Static Gazebo model of the entity, centred on its origin: spawn it at
    z = height / 2. Visual (seen by the gpu_lidar) and collision (physics)
    share the geometry."""
    size_x, size_y, size_z = entity.shape.size
    if entity.shape.type == 'cylinder':
        geometry = f'<cylinder><radius>{size_x / 2}</radius><length>{size_z}</length></cylinder>'
    else:
        geometry = f'<box><size>{size_x} {size_y} {size_z}</size></box>'
    color = COLORS[entity.category]
    return f"""<?xml version="1.0"?>
<sdf version="1.8">
  <model name="{entity.model_name}">
    <static>true</static>
    <link name="link">
      <collision name="collision"><geometry>{geometry}</geometry></collision>
      <visual name="visual"><geometry>{geometry}</geometry>
        <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>
    </link>
  </model>
</sdf>
"""


@dataclass
class Step:
    """What the director does at one tick."""
    spawn: list[tuple[ScriptedEntity, Pose]]
    move: list[tuple[ScriptedEntity, Pose]]
    remove: list[ScriptedEntity]
    truth: list[TruthEntity]
    removed_ids: list[str]                  # reported at the previous step, gone now


class Playback:
    """A scene played against Gazebo tick by tick. Remembers which models
    exist and the pose each was last given, so step(t) spawns only models
    that became active, moves only those whose pose changed and removes
    those whose time is up. Preexisting entities only get truth."""

    def __init__(self, scene: Scene):
        self.scene = scene
        self._placed: dict[str, Pose] = {}
        self._reported: set[str] = set()

    def step(self, t: float) -> Step:
        step = Step([], [], [], [], [])
        for entity in self.scene.entities:
            if not entity.active(t):
                if entity.id in self._placed:
                    del self._placed[entity.id]
                    step.remove.append(entity)
                if entity.id in self._reported:
                    step.removed_ids.append(entity.id)
                continue
            step.truth.append(entity.truth(t))
            if entity.preexisting:
                continue
            pose = entity.pose(t)
            if entity.id not in self._placed:
                step.spawn.append((entity, pose))
            elif pose != self._placed[entity.id]:
                step.move.append((entity, pose))
            self._placed[entity.id] = pose
        self._reported = {truth.id for truth in step.truth}
        return step


def _parse_entity(index, data):
    entity_id = data.get('id') if isinstance(data, dict) else None
    who = f'entity #{index}' if entity_id is None else f'entity {entity_id!r}'
    if not isinstance(data, dict):
        raise ValueError(f'{who}: must be a mapping, got {data!r}')
    if not isinstance(entity_id, str) or not _ID_PATTERN.fullmatch(entity_id):
        raise ValueError(f'{who}: id: required, letters, digits, _ and - only, got {entity_id!r}')
    _check_fields(who, data, _ENTITY_FIELDS)
    category = data.get('category')
    if category not in CATEGORIES:
        raise ValueError(f'{who}: category: must be one of {", ".join(CATEGORIES)}, '
                         f'got {category!r}')
    label = data.get('label')
    if not isinstance(label, str) or not label:
        raise ValueError(f'{who}: label: required non-empty string, got {label!r}')
    shape = _parse_shape(who, data.get('shape'))
    spawn = _number(who, 'spawn', data.get('spawn', 0))
    if spawn < 0:
        raise ValueError(f'{who}: spawn: must be >= 0, got {spawn:g}')
    remove = data.get('remove')
    if remove is not None:
        remove = _number(who, 'remove', remove)
        if remove <= spawn:
            raise ValueError(f'{who}: remove: must be > spawn ({spawn:g}), got {remove:g}')
    path = _parse_path(who, data.get('path'))
    preexisting = data.get('preexisting', False)
    if not isinstance(preexisting, bool):
        raise ValueError(f'{who}: preexisting: must be true or false, got {preexisting!r}')
    if preexisting:
        if spawn != 0:
            raise ValueError(f'{who}: spawn: a preexisting entity is there from the start')
        if remove is not None:
            raise ValueError(f'{who}: remove: a preexisting entity is never removed')
        if len(path) != 1:
            raise ValueError(f'{who}: path: a preexisting entity never moves (one waypoint)')
    return ScriptedEntity(entity_id, CATEGORIES[category], label, shape, path, spawn, remove,
                          preexisting)


def _parse_shape(who, data):
    kind = data.get('type') if isinstance(data, dict) else None
    if kind not in _SHAPE_FIELDS:
        raise ValueError(f'{who}: shape.type: must be cylinder or box, got {kind!r}')
    _check_fields(who, data, _SHAPE_FIELDS[kind], prefix='shape.')
    if kind == 'cylinder':
        radius = _positive(who, 'shape.radius', data.get('radius'))
        height = _positive(who, 'shape.height', data.get('height'))
        return Shape(kind, (2 * radius, 2 * radius, height))
    size = data.get('size')
    if not isinstance(size, list) or len(size) != 3:
        raise ValueError(f'{who}: shape.size: must be [x, y, z], got {size!r}')
    return Shape(kind, tuple(_positive(who, f'shape.size[{axis}]', value)
                             for axis, value in enumerate(size)))


def _parse_path(who, data):
    if not isinstance(data, list) or not data:
        raise ValueError(f'{who}: path: must be a non-empty list of {{t, x, y, yaw?}}, '
                         f'got {data!r}')
    waypoints = []
    for index, item in enumerate(data):
        field = f'path[{index}]'
        if not isinstance(item, dict):
            raise ValueError(f'{who}: {field}: must be a mapping {{t, x, y, yaw?}}, got {item!r}')
        _check_fields(who, item, _WAYPOINT_FIELDS, prefix=f'{field}.')
        t, x, y = (_number(who, f'{field}.{key}', item.get(key)) for key in ('t', 'x', 'y'))
        yaw = _number(who, f'{field}.yaw', item['yaw']) if 'yaw' in item else 0.0
        if waypoints and t <= waypoints[-1].t:
            raise ValueError(f'{who}: {field}.t: must increase, got {t:g} after '
                             f'{waypoints[-1].t:g}')
        waypoints.append(Waypoint(t, x, y, yaw))
    return tuple(waypoints)


def _check_fields(who, data, allowed, prefix=''):
    for key in data:
        if key not in allowed:
            raise ValueError(f'{who}: {prefix}{key}: unknown field, expected one of '
                             f'{", ".join(sorted(allowed))}')


def _number(who, field, value):
    # bool is an int in Python, but `x: true` is a typo, not a coordinate
    if isinstance(value, bool) or not isinstance(value, (int, float)) or \
            not math.isfinite(value):
        raise ValueError(f'{who}: {field}: must be a finite number, got {value!r}')
    return float(value)


def _positive(who, field, value):
    number = _number(who, field, value)
    if not number > 0:
        raise ValueError(f'{who}: {field}: must be > 0, got {value!r}')
    return number
