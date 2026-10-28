"""Nav2 (map server + AMCL + navigation) on the generated lab-floor map.

  ros2 launch fleetgraph_sim nav.launch.py

Wraps nav2_bringup/bringup_launch.py with this package's map and params
(config/nav2_params.yaml: Jazzy defaults, adapted — see header there).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory('fleetgraph_sim')
    bringup = os.path.join(get_package_share_directory('nav2_bringup'),
                           'launch', 'bringup_launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=os.path.join(pkg, 'maps', 'lab_floor.yaml')),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(pkg, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('log_level', default_value='warn'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'params_file': LaunchConfiguration('params_file'),
                'use_sim_time': 'true',
                'autostart': 'true',
                'use_composition': 'True',
                'log_level': LaunchConfiguration('log_level'),
            }.items()),
    ])
