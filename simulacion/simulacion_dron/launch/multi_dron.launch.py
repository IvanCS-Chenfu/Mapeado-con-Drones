from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import PushRosNamespace, Node

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

import yaml
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Launch del dron individual
    single_launch_path = os.path.join(
        get_package_share_directory('dron_individual'),
        'launch',
        'generar_dron.launch.py'
    )

    # Launch del servidor global ORB-SLAM
    global_server_launch_path = os.path.join(
        get_package_share_directory('orbslam3_server'),
        'launch',
        'global_orb_map_server.launch.py'
    )

    # Leer parámetros de simulación
    params_sim_path = os.path.join(
        get_package_share_directory('simulacion_dron'),
        'config',
        'sim_dron.yaml'
    )

    with open(params_sim_path, 'r') as f:
        params_sim_yaml = yaml.safe_load(f) or {}

    params_sim_read = params_sim_yaml.get('/**', {}).get('ros__parameters', {})

    debug_path = os.path.join(
        get_package_share_directory('simulacion_dron'),
        'config',
        'debug.yaml'
    )
    with open(debug_path, 'r') as f:
        debug_yaml = yaml.safe_load(f) or {}
    debug_values = debug_yaml.get('debug', {})

    def debug_default(name):
        value = debug_values.get(name)
        if not isinstance(value, bool):
            raise RuntimeError(
                f'debug.{name} debe existir y ser booleano')
        return str(value).lower()

    default_n = int(params_sim_read.get('dron.numero', 1))
    default_namespace_base = str(
        params_sim_read.get('dron.namespace_base', 'drone'))
    default_elegir_mundo = str(params_sim_read.get('world.activar', 'empty'))

    # World
    world_path = PathJoinSubstitution([
        FindPackageShare('simulacion_dron'),
        'worlds',
        PythonExpression(["'", default_elegir_mundo, "' + '.world'"])
    ])

    # Parámetros
    params_xacro = PathJoinSubstitution([
        FindPackageShare('dron_individual'),
        'config',
        'hardware.yaml'
    ])

    params_sim = PathJoinSubstitution([
        FindPackageShare('simulacion_dron'),
        'config',
        'sim_dron.yaml'
    ])
    global_map_config_dir = PathJoinSubstitution([
        FindPackageShare('simulacion_dron'),
        'config',
        'global_map',
    ])
    calibration_dron = PathJoinSubstitution([
        FindPackageShare('simulacion_dron'),
        'config',
        'calibration_dron.yaml',
    ])

    sparse_global_rviz_config = PathJoinSubstitution([
        FindPackageShare('simulacion_dron'),
        'rviz',
        'sparse_global_debug.rviz'
    ])
    pipeline_flow_web_root = PathJoinSubstitution([
        FindPackageShare('simulacion_dron'),
        'web',
        'pipeline_flow'
    ])
    system_architecture_web_root = PathJoinSubstitution([
        FindPackageShare('simulacion_dron'),
        'web',
        'system_architecture'
    ])
    compact_orb_vocabulary = PathJoinSubstitution([
        FindPackageShare('dron_individual'),
        'config',
        'orbslam',
        'vocabulary',
        'ORBvoc_L5.txt',
    ])
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

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        'launch_gazebo_gui', default_value='true'))
    ld.add_action(DeclareLaunchArgument(
        'launch_mission_gui', default_value='true'))
    ld.add_action(DeclareLaunchArgument(
        'drone_start_stagger_sec', default_value='8.0'))
    ld.add_action(DeclareLaunchArgument(
        'orb_vocabulary_path', default_value=compact_orb_vocabulary))
    ld.add_action(DeclareLaunchArgument(
        'debug_sparse_global_rviz',
        default_value=debug_default('debug_sparse_global_rviz')))
    ld.add_action(DeclareLaunchArgument(
        'debug_pipeline_flow_web',
        default_value=debug_default('debug_pipeline_flow_web')))
    ld.add_action(DeclareLaunchArgument(
        'debug_open_pipeline_flow_browser',
        default_value=debug_default('debug_open_pipeline_flow_browser')))
    ld.add_action(DeclareLaunchArgument(
        'debug_fase3_logs_terminal',
        default_value=debug_default('debug_fase3_logs_terminal')))
    ld.add_action(DeclareLaunchArgument(
        'debug_system_architecture_web',
        default_value=debug_default('debug_system_architecture_web')))
    ld.add_action(DeclareLaunchArgument(
        'debug_open_system_architecture_browser',
        default_value=debug_default(
            'debug_open_system_architecture_browser')))
    ld.add_action(DeclareLaunchArgument(
        'debug_architecture_telemetry',
        default_value=debug_default('debug_architecture_telemetry')))
    ld.add_action(DeclareLaunchArgument(
        'pipeline_flow_port', default_value='8765'))
    ld.add_action(DeclareLaunchArgument(
        'system_architecture_port', default_value='8775'))
    ld.add_action(DeclareLaunchArgument(
        'rawdb_record_enabled', default_value='false'))
    ld.add_action(DeclareLaunchArgument(
        'rawdb_record_path', default_value='/tmp/f3c_raw.record'))

    # Gazebo visual o servidor headless, nunca ambos a la vez.
    ld.add_action(
        ExecuteProcess(
            cmd=[
                'gazebo',
                '--verbose',
                world_path,
                '-s',
                'libgazebo_ros_factory.so'
            ],
            output='screen',
            condition=IfCondition(LaunchConfiguration('launch_gazebo_gui'))
        )
    )
    ld.add_action(
        ExecuteProcess(
            cmd=[
                'gzserver',
                '--verbose',
                world_path,
                '-s',
                'libgazebo_ros_factory.so'
            ],
            output='screen',
            condition=UnlessCondition(LaunchConfiguration('launch_gazebo_gui'))
        )
    )

    # Clock
    ld.add_action(
        Node(
            package='simulacion_dron',
            executable='clock',
            name='clock',
            output='screen'
        )
    )

    ld.add_action(
        Node(
            package='simulacion_dron',
            executable='system_architecture_bridge.py',
            name='system_architecture_bridge',
            parameters=[{
                'port': LaunchConfiguration('system_architecture_port'),
                'web_root': system_architecture_web_root,
                'telemetry_enabled': ParameterValue(
                    LaunchConfiguration('debug_architecture_telemetry'),
                    value_type=bool),
                'drone_count': default_n,
                'drone_namespace_base': default_namespace_base,
            }],
            output='screen',
            condition=IfCondition(
                LaunchConfiguration('debug_system_architecture_web'))
        )
    )

    ld.add_action(
        Node(
            package='simulacion_dron',
            executable='system_architecture_browser.py',
            name='system_architecture_browser',
            arguments=[
                '--port', LaunchConfiguration('system_architecture_port'),
                '--timeout', '20.0',
            ],
            output='screen',
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('debug_system_architecture_web'),
                "'.lower() == 'true' and '",
                LaunchConfiguration(
                    'debug_open_system_architecture_browser'),
                "'.lower() == 'true'",
            ]))
        )
    )

    ld.add_action(
        Node(
            package='simulacion_dron',
            executable='pipeline_flow_bridge.py',
            name='pipeline_flow_bridge',
            parameters=[{
                'topic': '/global_mapping/flow_events',
                'port': LaunchConfiguration('pipeline_flow_port'),
                'web_root': pipeline_flow_web_root,
            }],
            output='screen',
            condition=IfCondition(
                LaunchConfiguration('debug_pipeline_flow_web'))
        )
    )

    ld.add_action(
        Node(
            package='simulacion_dron',
            executable='pipeline_flow_browser.py',
            name='pipeline_flow_browser',
            arguments=[
                '--port', LaunchConfiguration('pipeline_flow_port'),
                '--timeout', '20.0',
            ],
            output='screen',
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('debug_pipeline_flow_web'),
                "'.lower() == 'true' and '",
                LaunchConfiguration('debug_open_pipeline_flow_browser'),
                "'.lower() == 'true'",
            ]))
        )
    )

    ld.add_action(
        Node(
            package='rviz2',
            executable='rviz2',
            name='sparse_global_rviz',
            arguments=['-d', sparse_global_rviz_config],
            parameters=[{'use_sim_time': True}],
            env=rviz_environment,
            output='screen',
            condition=IfCondition(
                LaunchConfiguration('debug_sparse_global_rviz'))
        )
    )

    # GUI
    ld.add_action(
        Node(
            package='simulacion_dron',
            executable='gui_tray_multi.py',
            name='gui_tray_multi',
            parameters=[params_sim],
            output='screen',
            condition=IfCondition(LaunchConfiguration('launch_mission_gui'))
        )
    )

    # Drones: drone_1, drone_2, ..., drone_n
    for i in range(1, default_n + 1):
        drone_name = f'{default_namespace_base}_{i}'
        local_map_frame = f'{drone_name}_orb_map'

        drone_group = GroupAction([
                PushRosNamespace(drone_name),

                Node(
                    package='simulacion_dron',
                    executable='generador_URDF',
                    name='generador_URDF',
                    output='screen',
                    parameters=[
                        params_xacro,
                        params_sim,
                        {
                            'use_sim_time': True,
                            'drone_id': i,
                            'drone_name': drone_name,
                        }
                    ]
                ),

                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(single_launch_path),
                    launch_arguments={
                        'drone_id': str(i),
                        'drone_name': drone_name,
                        'local_map_frame': local_map_frame,
                        'use_sim_time': 'true',
                        'orb_vocabulary_path': LaunchConfiguration(
                            'orb_vocabulary_path'),
                    }.items()
                ),
            ])

        if i == 1:
            ld.add_action(drone_group)
        else:
            ld.add_action(
                TimerAction(
                    period=PythonExpression([
                        LaunchConfiguration('drone_start_stagger_sec'),
                        ' * ',
                        str(i - 1),
                    ]),
                    actions=[drone_group],
                )
            )

    # Servidor global ORB-SLAM3
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(global_server_launch_path),
            launch_arguments={
                'config_dir': global_map_config_dir,
                'calibration_config': calibration_dron,
                'use_sim_time': 'true',
                'drone_count': str(default_n),
                'drone_namespace_base': default_namespace_base,
                'rawdb_record_enabled': LaunchConfiguration(
                    'rawdb_record_enabled'),
                'rawdb_record_path': LaunchConfiguration('rawdb_record_path'),
                'log_level': PythonExpression([
                    "'info' if '",
                    LaunchConfiguration('debug_fase3_logs_terminal'),
                    "'.lower() == 'true' else 'error'",
                ]),
            }.items()
        )
    )

    return ld
