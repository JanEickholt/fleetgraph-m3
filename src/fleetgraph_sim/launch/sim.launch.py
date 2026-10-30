"""Gazebo lab floor + one robot + ros_gz bridge.

  ros2 launch fleetgraph_sim sim.launch.py              # headless (default)
  ros2 launch fleetgraph_sim sim.launch.py headless:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    share_dir = get_package_share_directory('fleetgraph_sim')
    world = os.path.join(share_dir, 'worlds', 'lab_floor.sdf')
    xacro_file = os.path.join(share_dir, 'urdf', 'robot.urdf.xacro')
    bridge_file = os.path.join(share_dir, 'config', 'bridge.yaml')
    gz_sim_launch = os.path.join(
        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')

    headless = LaunchConfiguration('headless')
    robot_name = 'robot_1'   # bridge.yaml joint_state topic depends on it

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, f' robot_name:={robot_name}']),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='true',
                              description='Gazebo server only, no GUI'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_sim_launch),
            launch_arguments={'gz_args': ['-s -r ', world]}.items(),
            condition=IfCondition(headless)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_sim_launch),
            launch_arguments={'gz_args': ['-r ', world]}.items(),
            condition=UnlessCondition(headless)),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description,
                          'use_sim_time': True}],
             output='screen'),
        # spawn at the dock = map origin (AMCL initial pose matches)
        Node(package='ros_gz_sim', executable='create',
             arguments=['-topic', 'robot_description', '-name', robot_name,
                        '-x', '0.0', '-y', '0.0', '-z', '0.05'],
             output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             name='ros_gz_bridge',
             parameters=[{'config_file': bridge_file, 'use_sim_time': True}],
             output='screen'),
    ])
