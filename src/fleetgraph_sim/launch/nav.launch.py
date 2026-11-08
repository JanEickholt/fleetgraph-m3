"""Nav2 per robot (map server + AMCL + navigation) on the generated lab-floor map.

  ros2 launch fleetgraph_sim nav.launch.py              # robot_1
  ros2 launch fleetgraph_sim nav.launch.py robots:=3    # robot_1..robot_3

One nav2_bringup/bringup_launch.py stack per robot in namespace /robot_N
(TF on /robot_N/tf), AMCL initial pose = the robot's dock. Params:
config/nav2_params.yaml (Jazzy defaults, adapted — see header there).
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml

SHARE_DIR = get_package_share_directory('fleetgraph_sim')
MAX_ROBOTS = 3                      # docks in the location registry


def dock_pose(robot_index):
    """Home of robot_N: dock (robot_1) or dock_N. Same rule in sim.launch.py."""
    with open(os.path.join(SHARE_DIR, 'config', 'locations.yaml')) as file:
        locations = yaml.safe_load(file)['locations']
    dock = locations['dock' if robot_index == 1 else f'dock_{robot_index}']
    return dock['x'], dock['y'], dock['yaw']


def stacks(context):
    count = int(LaunchConfiguration('robots').perform(context))
    if not 1 <= count <= MAX_ROBOTS:
        raise ValueError(f'robots must be 1..{MAX_ROBOTS}, got {count}')
    bringup = os.path.join(get_package_share_directory('nav2_bringup'),
                           'launch', 'bringup_launch.py')
    actions = []
    for index in range(1, count + 1):
        x, y, yaw = dock_pose(index)
        params = RewrittenYaml(
            source_file=LaunchConfiguration('params_file'),
            # always with a '.': RewrittenYaml turns '0' into an int, which
            # AMCL's double parameters reject
            param_rewrites={f'amcl.ros__parameters.initial_pose.{key}': f'{value:.4f}'
                            for key, value in (('x', x), ('y', y), ('yaw', yaw))},
            convert_types=True)
        # scoped group: each include's launch configurations stay its own
        actions.append(GroupAction([IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup),
            launch_arguments={
                'namespace': f'robot_{index}',
                'use_namespace': 'True',
                'map': LaunchConfiguration('map'),
                'params_file': params,
                'use_sim_time': 'true',
                'autostart': 'true',
                'use_composition': 'True',
                'log_level': LaunchConfiguration('log_level'),
            }.items())]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robots', default_value='1',
                              description=f'number of robots, 1..{MAX_ROBOTS}'),
        DeclareLaunchArgument('map',
                              default_value=os.path.join(SHARE_DIR, 'maps', 'lab_floor.yaml')),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(SHARE_DIR, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('log_level', default_value='warn'),
        OpaqueFunction(function=stacks),
    ])
