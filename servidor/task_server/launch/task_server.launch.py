from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mission_config'),
        DeclareLaunchArgument('voxel_size', default_value='0.25'),
        DeclareLaunchArgument('mission_flow_events_enabled', default_value='false'),
        DeclareLaunchArgument('system_architecture_events_enabled', default_value='false'),
        Node(
            package='task_server', executable='task_server_node', name='task_server',
            output='screen', parameters=[{
                'mission_config': LaunchConfiguration('mission_config'),
                'voxel_size': LaunchConfiguration('voxel_size'),
                'mission_flow_events_enabled': LaunchConfiguration(
                    'mission_flow_events_enabled'),
                'system_architecture_events_enabled': LaunchConfiguration(
                    'system_architecture_events_enabled'),
            }]),
    ])
