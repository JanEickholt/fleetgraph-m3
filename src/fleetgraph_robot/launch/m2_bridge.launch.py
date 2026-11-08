"""Module 2 link: rosbridge websocket server (JSON protocol) + rosapi.

  ros2 launch fleetgraph_robot m2_bridge.launch.py              # ws://<host>:9090
  ros2 launch fleetgraph_robot m2_bridge.launch.py \
      probe:='python3 /home/ubuntu/tools/ws_probe.py --text "Go to the hall"'

A websocket client may subscribe the /fleet/* contract topics and publish
plain-text tasks to /mock_m1/task_text (plus /mock_m2/*, the test probe's
echoes). It cannot publish /fleet/* (assignments stay with Module 1) and
cannot read or set ROS parameters. `probe` is a command line started next to
the bridge, so a harness scenario (which only runs ROS launch files) can run
the websocket probe in the same process group.
"""
import shlex

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# rosbridge / rosapi glob syntax: "['pattern', ...]"; '[]' allows nothing
GLOBS = {
    'topics_sub_glob': "['/fleet/*']",
    'topics_pub_glob': "['/mock_m1/task_text', '/mock_m2/*']",
    'services_glob': "['/fleet/*']",
    'params_glob': '[]',
}


def probe(context):
    command = LaunchConfiguration('probe').perform(context)
    return [ExecuteProcess(cmd=shlex.split(command), output='screen')] if command else []


def generate_launch_description():
    # forced to str: launch would otherwise parse "[...]" into a YAML list
    globs = {name: ParameterValue(value, value_type=str) for name, value in GLOBS.items()}
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='9090'),
        DeclareLaunchArgument('probe', default_value='',
                              description='command line of a websocket client to run'),
        Node(package='rosbridge_server', executable='rosbridge_websocket',
             name='rosbridge_websocket', output='screen',
             parameters=[{'port': ParameterValue(LaunchConfiguration('port'), value_type=int),
                          **globs}]),
        Node(package='rosapi', executable='rosapi_node', name='rosapi', output='screen',
             parameters=[globs]),
        OpaqueFunction(function=probe),
    ])
