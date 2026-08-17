from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    server_launch = PathJoinSubstitution([
        FindPackageShare('orbslam3_server'),
        'launch',
        'global_orb_map_server.launch.py',
    ])
    web_root = PathJoinSubstitution([
        FindPackageShare('simulacion_dron'),
        'web',
        'pipeline_flow',
    ])

    bridge = Node(
        package='simulacion_dron',
        executable='pipeline_flow_bridge.py',
        name='pipeline_flow_bridge',
        parameters=[{
            'topic': '/global_mapping/flow_events',
            'port': LaunchConfiguration('pipeline_flow_port'),
            'web_root': web_root,
        }],
        output='screen',
    )
    server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(server_launch),
        launch_arguments={
            'use_sim_time': 'false',
            'primary_worker_debug_delay_ms': '0',
            'rawdb_record_enabled': 'false',
            'rawdb_replay_path': LaunchConfiguration('rawdb_replay_path'),
            'pose_store_debug_anchor_enabled': 'true',
            'pose_store_debug_anchor_drone_id': '1',
            'pose_store_debug_anchor_map_epoch': '1',
            'pose_store_debug_anchor_x': '10.0',
            'pose_store_debug_anchor_y': '0.0',
            'pose_store_debug_anchor_z': '0.0',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('rawdb_replay_path'),
        DeclareLaunchArgument('pipeline_flow_port', default_value='8766'),
        DeclareLaunchArgument(
            'open_pipeline_flow_browser', default_value='false'),
        bridge,
        TimerAction(period=1.0, actions=[server]),
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
