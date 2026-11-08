"""The social mask node takes its grid from the static map's metadata
(fleetgraph_robot.social_mask_node.grid_from_info). Skips without a built
ROS workspace."""
import math

import pytest

pytest.importorskip('rclpy')
pytest.importorskip('nav2_msgs.msg')
pytest.importorskip('fleetgraph_interfaces.msg')

from nav_msgs.msg import MapMetaData  # noqa: E402

from fleetgraph_robot.social_mask import Grid  # noqa: E402
from fleetgraph_robot.social_mask_node import grid_from_info  # noqa: E402


def lab_info(yaw=0.0):
    info = MapMetaData(resolution=0.05, width=220, height=140)
    info.origin.position.x, info.origin.position.y = -1.5, -3.5
    info.origin.orientation.z, info.origin.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return info


def test_the_lab_map_metadata_gives_its_grid():
    assert grid_from_info(lab_info()) == Grid(-1.5, -3.5, 0.05, 220, 140)


def test_a_turned_map_origin_is_refused():
    with pytest.raises(ValueError, match='axis-aligned'):
        grid_from_info(lab_info(yaw=0.1))
