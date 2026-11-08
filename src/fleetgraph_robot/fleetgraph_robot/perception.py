"""Pure glue between ROS data and the local perception core (scene_graph,
lidar_obstacles): planar frame transforms, the sensor coverage that decides
where unmatched tracks decay, semantic detections in the map frame and the
localization score. No rclpy and no message imports: messages are read by
attribute (tests/unit/test_perception.py).
"""
import math

from fleetgraph_robot.lidar_obstacles import OccupancyMap
from fleetgraph_robot.scene_graph import PHYSICAL, Detection

Pose = tuple[float, float, float]           # x, y, yaw of a frame in the map frame
SEMANTIC_SOURCE = 'detector'                # Detection.source of semantic detections


def to_map(pose: Pose, x: float, y: float) -> tuple[float, float]:
    """A point given in the frame at pose (x forward, y left) -> map frame."""
    origin_x, origin_y, yaw = pose
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    return (origin_x + cos_yaw * x - sin_yaw * y, origin_y + sin_yaw * x + cos_yaw * y)


def coverage(occupancy: OccupancyMap, sensor_x: float, sensor_y: float, max_range: float):
    """in_view(x, y) of one lidar frame for LocalSceneGraph.integrate: within
    max_range of the sensor and a line of sight through the STATIC map.
    Unmapped obstacles do not shadow: a track hidden behind one decays too."""
    def in_view(x, y):
        return (math.hypot(x - sensor_x, y - sensor_y) <= max_range
                and occupancy.line_of_sight(sensor_x, sensor_y, x, y))
    return in_view


def occupancy_from_grid(grid) -> OccupancyMap:
    """nav_msgs/OccupancyGrid -> OccupancyMap. ValueError for a rotated map
    origin, which OccupancyMap cannot represent."""
    info = grid.info
    orientation = info.origin.orientation
    if max(abs(orientation.x), abs(orientation.y), abs(orientation.z)) > 1e-6:
        raise ValueError(f'map origin is rotated ({orientation.x}, {orientation.y}, '
                         f'{orientation.z}, {orientation.w}); only unrotated maps are supported')
    return OccupancyMap(float(info.resolution), float(info.origin.position.x),
                        float(info.origin.position.y), int(info.width), int(info.height),
                        list(grid.data))


def semantic_detections(entities, pose: Pose, stamp: float) -> list[Detection]:
    """SceneEntity detections given in the frame at pose -> map-frame
    Detections. Drops what the graph cannot track (regions, groups,
    non-finite or negative values) and clamps confidence into [0, 1]."""
    detections = []
    for entity in entities:
        position, size = entity.position, entity.size
        values = (position.x, position.y, position.z, size.x, size.y, size.z, entity.confidence)
        if (entity.category not in PHYSICAL or not all(map(math.isfinite, values))
                or min(size.x, size.y, size.z) < 0):
            continue
        x, y = to_map(pose, position.x, position.y)
        detections.append(Detection(
            int(entity.category), x, y, z=float(position.z),
            size=(float(size.x), float(size.y), float(size.z)),
            confidence=min(1.0, max(0.0, float(entity.confidence))), label=entity.label,
            stamp=stamp, source=SEMANTIC_SOURCE))
    return detections


def localization_score(covariance) -> float:
    """AMCL pose covariance (6x6, row-major) -> 0..1, the agent_node
    heuristic: 1.0 at 0 m position std, 0.5 at 0.25 m."""
    std_xy = math.sqrt(max(covariance[0], 0.0) + max(covariance[7], 0.0))
    return 1.0 / (1.0 + std_xy / 0.25)
