import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_environment = {}
    for key, value in os.environ.items():
        if key.startswith('SNAP') or key.startswith('VSCODE_'):
            continue
        cleaned_parts = [
            part for part in value.split(':')
            if '/snap/' not in part and '/snapd/' not in part
        ]
        cleaned_value = ':'.join(cleaned_parts)
        if cleaned_value:
            rviz_environment[key] = cleaned_value
    rviz_environment['XDG_DATA_HOME'] = os.path.expanduser('~/.local/share')
    rviz_environment['XDG_CONFIG_HOME'] = os.path.expanduser('~/.config')
    rviz_environment['XDG_CACHE_HOME'] = os.path.expanduser('~/.cache')

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
    rviz_config = PathJoinSubstitution([
        FindPackageShare('simulacion_dron'),
        'rviz',
        'sparse_global_debug.rviz',
    ])
    config_dir = PathJoinSubstitution([
        FindPackageShare('simulacion_dron'),
        'config',
        'global_map',
    ])
    replay_config = PathJoinSubstitution([config_dir, 'replay_debug.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('rawdb_replay_path'),
        DeclareLaunchArgument('rawdb_replay_entry_delay_ms', default_value='100'),
        DeclareLaunchArgument('pipeline_flow_port', default_value='8768'),
        DeclareLaunchArgument('open_pipeline_flow_browser', default_value='true'),
        DeclareLaunchArgument('launch_sparse_global_rviz', default_value='true'),
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
            condition=IfCondition(LaunchConfiguration('open_pipeline_flow_browser')),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='sparse_global_rviz',
            arguments=['-d', rviz_config],
            env=rviz_environment,
            output='screen',
            condition=IfCondition(LaunchConfiguration('launch_sparse_global_rviz')),
        ),
        TimerAction(period=1.0, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(server_launch),
            launch_arguments={
                'config_dir': config_dir,
                'replay_debug_config': replay_config,
                'use_sim_time': 'false',
                'rawdb_record_enabled': 'false',
                'rawdb_replay_path': LaunchConfiguration('rawdb_replay_path'),
                'rawdb_replay_entry_delay_ms': LaunchConfiguration(
                    'rawdb_replay_entry_delay_ms'),
                'pose_store_debug_anchor_enabled': 'false',
            }.items(),
        )]),
    ])
