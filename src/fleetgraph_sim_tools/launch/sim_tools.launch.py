"""Scripted scene and simulated detectors on top of fleetgraph_sim's
sim.launch.py (simulation only).

  ros2 launch fleetgraph_sim_tools sim_tools.launch.py robots:=3 scene:=walker_hall seed:=1

scene: a name in share/fleetgraph_sim_tools/scenes or a .yaml path; empty
for none (no scene_director, no /sim/truth: the detectors see only the
robots). robots must match sim.launch.py. One sim_detector per robot in
/robot_N unless detector:=false.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

MAX_ROBOTS = 3                      # docks in fleetgraph_sim's location registry


def nodes(context):
    count = int(LaunchConfiguration('robots').perform(context))
    if not 1 <= count <= MAX_ROBOTS:
        raise ValueError(f'robots must be 1..{MAX_ROBOTS}, got {count}')
    scene = LaunchConfiguration('scene').perform(context)
    seed = int(LaunchConfiguration('seed').perform(context))
    actions = []
    if scene:
        actions.append(Node(package='fleetgraph_sim_tools', executable='scene_director',
                            parameters=[{'scene': scene, 'use_sim_time': True}],
                            output='screen'))
    if LaunchConfiguration('detector').perform(context).lower() in ('true', '1'):
        for index in range(1, count + 1):
            robot_id = f'robot_{index}'
            actions.append(Node(package='fleetgraph_sim_tools', executable='sim_detector',
                                namespace=robot_id,
                                parameters=[{'robot_id': robot_id, 'robots': count,
                                             'seed': seed, 'use_sim_time': True}],
                                output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robots', default_value='1',
                              description=f'number of robots, 1..{MAX_ROBOTS}'),
        DeclareLaunchArgument('scene', default_value='',
                              description='scene name (share/scenes) or .yaml path; '
                                          'empty: none'),
        DeclareLaunchArgument('detector', default_value='true',
                              description='one sim_detector per robot'),
        DeclareLaunchArgument('seed', default_value='1',
                              description='detector noise seed'),
        OpaqueFunction(function=nodes),
    ])
