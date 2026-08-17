from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
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
        DeclareLaunchArgument('pipeline_flow_port', default_value='8766'),
        DeclareLaunchArgument('open_pipeline_flow_browser', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(server_launch),
            launch_arguments={
                'use_sim_time': 'false',
                'primary_worker_debug_delay_ms': '0',
                'rawdb_record_enabled': 'false',
                'rawdb_replay_path': LaunchConfiguration('rawdb_replay_path'),
            }.items(),
        ),
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
        ExecuteProcess(
            cmd=[
                'python3', '-m', 'webbrowser', '-t',
                PythonExpression([
                    "'http://127.0.0.1:' + str(",
                    LaunchConfiguration('pipeline_flow_port'), ")",
                ]),
            ],
            output='screen',
            condition=IfCondition(LaunchConfiguration('open_pipeline_flow_browser')),
        ),
    ])
