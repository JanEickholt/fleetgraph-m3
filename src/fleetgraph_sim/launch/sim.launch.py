"""Gazebo lab floor + robot_1..robot_N + one ros_gz bridge.

  ros2 launch fleetgraph_sim sim.launch.py                # robot_1, headless (default)
  ros2 launch fleetgraph_sim sim.launch.py robots:=3      # robot_1..robot_3
  ros2 launch fleetgraph_sim sim.launch.py camera:=true headless:=false

robot_N spawns at its dock (config/locations.yaml: dock, dock_2, dock_3)
and lives in namespace /robot_N: scan, odom, cmd_vel, joint_states,
robot_description, TF on /robot_N/tf + /robot_N/tf_static with unprefixed
frames (map = Gazebo world frame for every robot). Ground truth for tests:
/ground_truth/robot_N/odom. /clock is bridged once.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

SHARE_DIR = get_package_share_directory('fleetgraph_sim')
MAX_ROBOTS = 3                      # docks in the location registry
TF_REMAPS = [('/tf', 'tf'), ('/tf_static', 'tf_static')]


def dock_pose(robot_index):
    """Home of robot_N: dock (robot_1) or dock_N. Same rule in nav.launch.py."""
    with open(os.path.join(SHARE_DIR, 'config', 'locations.yaml')) as file:
        locations = yaml.safe_load(file)['locations']
    dock = locations['dock' if robot_index == 1 else f'dock_{robot_index}']
    return dock['x'], dock['y'], dock['yaw']


def robot_bridges(name, camera):
    """parameter_bridge arguments for one robot (gz topic = ROS topic)."""
    bridges = [
        f'/{name}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        f'/{name}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        f'/{name}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',     # DiffDrive odom -> base_footprint
        f'/{name}/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        f'/{name}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        # TEST VERIFICATION ONLY: robot software must not subscribe
        f'/ground_truth/{name}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
    ]
    if camera:
        bridges += [f'/{name}/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
                    f'/{name}/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo']
    return bridges


def robots(context):
    count = int(LaunchConfiguration('robots').perform(context))
    if not 1 <= count <= MAX_ROBOTS:
        raise ValueError(f'robots must be 1..{MAX_ROBOTS}, got {count}')
    camera = LaunchConfiguration('camera').perform(context).lower() in ('true', '1')
    xacro_file = os.path.join(SHARE_DIR, 'urdf', 'robot.urdf.xacro')
    actions = []
    bridges = ['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']
    for index in range(1, count + 1):
        name = f'robot_{index}'
        x, y, yaw = dock_pose(index)
        description = ParameterValue(Command(
            ['xacro ', xacro_file, f' robot_name:={name} camera:={str(camera).lower()}']),
            value_type=str)
        actions += [
            Node(package='robot_state_publisher', executable='robot_state_publisher',
                 namespace=name, remappings=TF_REMAPS,
                 parameters=[{'robot_description': description, 'use_sim_time': True}],
                 output='screen'),
            Node(package='ros_gz_sim', executable='create', namespace=name,
                 arguments=['-topic', f'/{name}/robot_description', '-name', name,
                            '-x', str(x), '-y', str(y), '-z', '0.05', '-Y', str(yaw)],
                 output='screen'),
        ]
        bridges += robot_bridges(name, camera)
    actions.append(Node(package='ros_gz_bridge', executable='parameter_bridge',
                        name='ros_gz_bridge', arguments=bridges,
                        parameters=[{'use_sim_time': True}], output='screen'))
    return actions


def generate_launch_description():
    world = os.path.join(SHARE_DIR, 'worlds', 'lab_floor.sdf')
    gz_sim_launch = os.path.join(
        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    headless = LaunchConfiguration('headless')
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='true',
                              description='Gazebo server only, no GUI'),
        DeclareLaunchArgument('robots', default_value='1',
                              description=f'number of robots, 1..{MAX_ROBOTS}'),
        DeclareLaunchArgument('camera', default_value='false',
                              description='camera sensors (software rendering: costly)'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_sim_launch),
            launch_arguments={'gz_args': ['-s -r ', world]}.items(),
            condition=IfCondition(headless)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_sim_launch),
            launch_arguments={'gz_args': ['-r ', world]}.items(),
            condition=UnlessCondition(headless)),
        OpaqueFunction(function=robots),
    ])
