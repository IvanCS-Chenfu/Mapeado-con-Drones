from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    web_root = PathJoinSubstitution([
        FindPackageShare('simulacion_dron'),
        'web',
        'pipeline_flow',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('pipeline_flow_port', default_value='8765'),
        DeclareLaunchArgument(
            'open_pipeline_flow_browser', default_value='true'),
        Node(
            package='simulacion_dron',
            executable='pipeline_flow_bridge.py',
            name='pipeline_flow_bridge',
            parameters=[{
                'topic': '/global_mapping/flow_events',
                'port': LaunchConfiguration('pipeline_flow_port'),
                'web_root': web_root,
            }],
            output='screen',
        ),
        Node(
            package='simulacion_dron',
            executable='pipeline_flow_browser.py',
            name='pipeline_flow_browser',
            arguments=[
                '--port', LaunchConfiguration('pipeline_flow_port'),
                '--timeout', '20.0',
            ],
            output='screen',
            condition=IfCondition(
                LaunchConfiguration('open_pipeline_flow_browser')),
        ),
    ])
