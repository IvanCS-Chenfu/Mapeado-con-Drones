from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
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

    return LaunchDescription([
        DeclareLaunchArgument('rawdb_replay_path'),
        DeclareLaunchArgument('pipeline_flow_port', default_value='8767'),
        DeclareLaunchArgument('open_pipeline_flow_browser', default_value='false'),
        DeclareLaunchArgument(
            'fiducial_translation_threshold_m', default_value='0.35'),
        DeclareLaunchArgument(
            'fiducial_rotation_threshold_rad', default_value='0.35'),
        DeclareLaunchArgument('fiducial_yaw_threshold_rad', default_value='0.25'),
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
        TimerAction(period=1.0, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(server_launch),
            launch_arguments={
                'use_sim_time': 'false',
                'rawdb_record_enabled': 'false',
                'rawdb_replay_path': LaunchConfiguration('rawdb_replay_path'),
                'pose_store_debug_anchor_enabled': 'false',
                'fiducial_sim_enabled': 'true',
                'fiducial_translation_threshold_m': LaunchConfiguration(
                    'fiducial_translation_threshold_m'),
                'fiducial_rotation_threshold_rad': LaunchConfiguration(
                    'fiducial_rotation_threshold_rad'),
                'fiducial_yaw_threshold_rad': LaunchConfiguration(
                    'fiducial_yaw_threshold_rad'),
            }.items(),
        )]),
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
    ])
