from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('drone_id'),
        DeclareLaunchArgument('system_architecture_events_enabled', default_value='false'),
        Node(
            package='task_manager', executable='task_manager_node', name='task_manager',
            namespace=['drone', LaunchConfiguration('drone_id')], output='screen',
            parameters=[{
                'drone_id': ParameterValue(LaunchConfiguration('drone_id'), value_type=int),
                'system_architecture_events_enabled': ParameterValue(
                    LaunchConfiguration('system_architecture_events_enabled'), value_type=bool),
            }]),
    ])
