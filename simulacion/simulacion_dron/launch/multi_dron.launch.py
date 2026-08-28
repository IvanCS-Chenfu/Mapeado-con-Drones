import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    single_launch_path = os.path.join(
        get_package_share_directory('dron_individual'),
        'launch', 'generar_dron.launch.py')
    global_server_launch_path = os.path.join(
        get_package_share_directory('orbslam3_server'),
        'launch', 'global_orb_map_server.launch.py')

    params_sim_path = os.path.join(
        get_package_share_directory('simulacion_dron'),
        'config', 'sim_dron.yaml')
    with open(params_sim_path, 'r', encoding='utf-8') as stream:
        params_sim_yaml = yaml.safe_load(stream) or {}
    params_sim_read = params_sim_yaml.get('/**', {}).get('ros__parameters', {})

    debug_path = os.path.join(
        get_package_share_directory('simulacion_dron'), 'config', 'debug.yaml')
    with open(debug_path, 'r', encoding='utf-8') as stream:
        debug_yaml = yaml.safe_load(stream) or {}
    debug_values = debug_yaml.get('debug', {})

    def debug_default(name):
        value = debug_values.get(name)
        if not isinstance(value, bool):
            raise RuntimeError(f'debug.{name} debe existir y ser booleano')
        return str(value).lower()

    def numeric_debug_default(name):
        value = debug_values.get(name)
        if not isinstance(value, (int, float)) or isinstance(value, bool):
            raise RuntimeError(f'debug.{name} debe ser numerico')
        return str(value)

    default_n = int(params_sim_read.get('dron.numero', 1))
    default_namespace_base = str(params_sim_read.get('dron.namespace_base', 'dron'))
    default_world = str(params_sim_read.get('world.activar', 'empty'))

    world_path = PathJoinSubstitution([
        FindPackageShare('simulacion_dron'),
        'worlds',
        PythonExpression(["'", default_world, "' + '.world'"])
    ])

    params_physical_model = PathJoinSubstitution([
        FindPackageShare('simulacion_dron'), 'config', 'physical_dron.yaml'])
    params_actuators_replica = PathJoinSubstitution([
        FindPackageShare('simulacion_dron'), 'config', 'actuators_dron.yaml'])
    params_simulated_sensors = PathJoinSubstitution([
        FindPackageShare('simulacion_dron'), 'config', 'simulated_sensors.yaml'])
    params_sim = PathJoinSubstitution([
        FindPackageShare('simulacion_dron'), 'config', 'sim_dron.yaml'])
    global_map_config_dir = PathJoinSubstitution([
        FindPackageShare('simulacion_dron'), 'config', 'global_map'])
    fiducial_objects_config = PathJoinSubstitution([
        FindPackageShare('simulacion_dron'), 'config', 'fiducial_objects.yaml'])
    fiducial_rendering_config = PathJoinSubstitution([
        FindPackageShare('simulacion_dron'), 'config', 'fiducial_rendering.yaml'])
    sparse_global_rviz_config = PathJoinSubstitution([
        FindPackageShare('simulacion_dron'), 'rviz', 'sparse_global_debug.rviz'])
    pipeline_flow_web_root = PathJoinSubstitution([
        FindPackageShare('simulacion_dron'), 'web', 'pipeline_flow'])
    system_architecture_web_root = PathJoinSubstitution([
        FindPackageShare('simulacion_dron'), 'web', 'system_architecture'])
    full_orb_vocabulary = PathJoinSubstitution([
        FindPackageShare('dron_individual'),
        'config', 'orbslam', 'vocabulary', 'ORBvoc.txt'])

    rviz_environment = {}
    for key, value in os.environ.items():
        if key.startswith('SNAP') or key.startswith('VSCODE_'):
            continue
        cleaned_parts = [
            part for part in value.split(':')
            if '/snap/' not in part and '/snapd/' not in part]
        cleaned_value = ':'.join(cleaned_parts)
        if cleaned_value:
            rviz_environment[key] = cleaned_value
    rviz_environment['XDG_DATA_HOME'] = os.path.expanduser('~/.local/share')
    rviz_environment['XDG_CONFIG_HOME'] = os.path.expanduser('~/.config')
    rviz_environment['XDG_CACHE_HOME'] = os.path.expanduser('~/.cache')

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('launch_gazebo_gui', default_value='true'))
    ld.add_action(DeclareLaunchArgument('launch_mission_gui', default_value='true'))
    ld.add_action(DeclareLaunchArgument('spawn_fiducials', default_value='true'))
    ld.add_action(DeclareLaunchArgument('drone_start_stagger_sec', default_value='8.0'))
    ld.add_action(DeclareLaunchArgument(
        'dron_spawn_override_enabled', default_value='false'))
    ld.add_action(DeclareLaunchArgument('dron_spawn_y', default_value='-10.8'))
    ld.add_action(DeclareLaunchArgument(
        'dron_spawn_yaw_deg', default_value='90.0'))
    ld.add_action(DeclareLaunchArgument(
        'orb_vocabulary_path', default_value=full_orb_vocabulary))
    for flag in (
        'debug_sparse_global_rviz',
        'debug_pipeline_flow_web',
        'debug_open_pipeline_flow_browser',
        'debug_fase3_logs_terminal',
        'debug_system_architecture_web',
        'debug_open_system_architecture_browser',
        'debug_architecture_telemetry',
        'debug_fiducial_visualization',
        'debug_orb_control_state',
    ):
        ld.add_action(DeclareLaunchArgument(flag, default_value=debug_default(flag)))
    ld.add_action(DeclareLaunchArgument(
        'debug_fiducial_display_seconds',
        default_value=numeric_debug_default(
            'debug_fiducial_display_seconds')))
    ld.add_action(DeclareLaunchArgument('pipeline_flow_port', default_value='8765'))
    ld.add_action(DeclareLaunchArgument('system_architecture_port', default_value='8775'))
    ld.add_action(DeclareLaunchArgument(
        'fiducial_visual_min_distance_m', default_value='1.0'))
    ld.add_action(DeclareLaunchArgument(
        'fiducial_visual_max_distance_m', default_value='5.0'))
    ld.add_action(DeclareLaunchArgument(
        'fiducial_visual_consistency_translation_m', default_value='0.15'))
    ld.add_action(DeclareLaunchArgument(
        'fiducial_visual_consistency_rotation_rad', default_value='0.2617993877991494'))
    ld.add_action(DeclareLaunchArgument(
        'fiducial_visual_visit_gap_sec', default_value='2.0'))
    ld.add_action(DeclareLaunchArgument(
        'fiducial_visual_recent_capacity_per_drone', default_value='50'))
    ld.add_action(DeclareLaunchArgument('rawdb_record_enabled', default_value='false'))
    ld.add_action(DeclareLaunchArgument(
        'rawdb_record_path', default_value='/tmp/f3c_raw.record'))
    ld.add_action(DeclareLaunchArgument(
        'phase5_pose_metrics_enabled', default_value='false'))
    ld.add_action(DeclareLaunchArgument(
        'phase5_pose_metrics_output_dir', default_value='/tmp/fase5_pose_metrics'))
    ld.add_action(DeclareLaunchArgument(
        'phase5_global_pose_rviz_enabled', default_value='false'))
    ld.add_action(DeclareLaunchArgument(
        'gt_fallback_enabled', default_value='true'))
    ld.add_action(DeclareLaunchArgument(
        'orb_qualification_samples', default_value='20'))

    architecture_telemetry_enabled = PythonExpression([
        "'", LaunchConfiguration('debug_system_architecture_web'),
        "'.lower() == 'true' and '",
        LaunchConfiguration('debug_architecture_telemetry'),
        "'.lower() == 'true'",
    ])

    ld.add_action(ExecuteProcess(
        cmd=['gazebo', '--verbose', world_path, '-s', 'libgazebo_ros_factory.so'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_gazebo_gui'))))
    ld.add_action(ExecuteProcess(
        cmd=['gzserver', '--verbose', world_path, '-s', 'libgazebo_ros_factory.so'],
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('launch_gazebo_gui'))))

    # Este nodo es la autoridad que publica /clock a partir de wall-time;
    # por definición NO debe consumir el reloj simulado que él mismo genera.
    ld.add_action(Node(
        package='simulacion_dron', executable='clock', name='clock',
        output='screen'))

    ld.add_action(Node(
        package='simulacion_dron',
        executable='fiducial_spawner.py',
        name='fiducial_spawner',
        parameters=[{
            'use_sim_time': True,
            'config_file': fiducial_objects_config,
            'render_config_file': fiducial_rendering_config,
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('spawn_fiducials'))))

    ld.add_action(Node(
        package='simulacion_dron',
        executable='pose_metrics_node.py',
        name='phase5_pose_metrics',
        parameters=[{
            'use_sim_time': True,
            'drone_count': default_n,
            'namespace_base': default_namespace_base,
            'output_dir': LaunchConfiguration('phase5_pose_metrics_output_dir'),
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('phase5_pose_metrics_enabled'))))

    ld.add_action(Node(
        package='simulacion_dron',
        executable='global_drone_pose_visualizer.py',
        name='global_drone_pose_visualizer',
        parameters=[{
            'use_sim_time': True,
            'drone_count': default_n,
            'namespace_base': default_namespace_base,
            'global_frame': 'world',
        }],
        output='screen',
        condition=IfCondition(
            LaunchConfiguration('phase5_global_pose_rviz_enabled'))))

    ld.add_action(Node(
        package='simulacion_dron',
        executable='system_architecture_bridge.py',
        name='system_architecture_bridge',
        parameters=[{
            'use_sim_time': True,
            'port': LaunchConfiguration('system_architecture_port'),
            'web_root': system_architecture_web_root,
            'telemetry_enabled': ParameterValue(
                architecture_telemetry_enabled, value_type=bool),
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('debug_system_architecture_web'))))

    ld.add_action(Node(
        package='simulacion_dron',
        executable='system_architecture_browser.py',
        name='system_architecture_browser',
        arguments=['--port', LaunchConfiguration('system_architecture_port'), '--timeout', '20.0'],
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('debug_system_architecture_web'),
            "'.lower() == 'true' and '",
            LaunchConfiguration('debug_open_system_architecture_browser'),
            "'.lower() == 'true'",
        ]))))

    ld.add_action(Node(
        package='simulacion_dron',
        executable='pipeline_flow_bridge.py',
        name='pipeline_flow_bridge',
        parameters=[{
            'use_sim_time': True,
            'topic': '/global_mapping/flow_events',
            'port': LaunchConfiguration('pipeline_flow_port'),
            'web_root': pipeline_flow_web_root,
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('debug_pipeline_flow_web'))))

    ld.add_action(Node(
        package='simulacion_dron',
        executable='pipeline_flow_browser.py',
        name='pipeline_flow_browser',
        arguments=['--port', LaunchConfiguration('pipeline_flow_port'), '--timeout', '20.0'],
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('debug_pipeline_flow_web'),
            "'.lower() == 'true' and '",
            LaunchConfiguration('debug_open_pipeline_flow_browser'),
            "'.lower() == 'true'",
        ]))))

    ld.add_action(Node(
        package='rviz2', executable='rviz2', name='sparse_global_rviz',
        arguments=['-d', sparse_global_rviz_config],
        parameters=[{'use_sim_time': True}], env=rviz_environment,
        output='screen',
        condition=IfCondition(LaunchConfiguration('debug_sparse_global_rviz'))))

    ld.add_action(Node(
        package='simulacion_dron', executable='gui_tray_multi.py',
        name='gui_tray_multi',
        parameters=[params_sim, {'use_sim_time': True}], output='screen',
        condition=IfCondition(LaunchConfiguration('launch_mission_gui'))))

    for i in range(1, default_n + 1):
        drone_name = f'{default_namespace_base}_{i}'
        local_map_frame = f'{drone_name}_orb_map'
        drone_group = GroupAction([
            PushRosNamespace(drone_name),
            Node(
                package='simulacion_dron', executable='generador_URDF',
                name='generador_URDF', output='screen',
                parameters=[
                    params_physical_model,
                    params_actuators_replica,
                    params_simulated_sensors,
                    params_sim,
                    {
                        'use_sim_time': True,
                        'drone_id': i,
                        'drone_name': drone_name,
                        'dron.spawn_override_enabled': ParameterValue(
                            LaunchConfiguration('dron_spawn_override_enabled'),
                            value_type=bool),
                        'dron.spawn_x': -1.0 if i % 2 == 1 else 1.0,
                        'dron.spawn_y': ParameterValue(
                            LaunchConfiguration('dron_spawn_y'),
                            value_type=float),
                        'dron.spawn_yaw_deg': ParameterValue(
                            LaunchConfiguration('dron_spawn_yaw_deg'),
                            value_type=float),
                    },
                ]),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(single_launch_path),
                launch_arguments={
                    'drone_id': str(i),
                    'drone_name': drone_name,
                    'local_map_frame': local_map_frame,
                    'use_sim_time': 'true',
                    'orb_vocabulary_path': LaunchConfiguration('orb_vocabulary_path'),
                    'debug_architecture_telemetry': architecture_telemetry_enabled,
                    'debug_fiducial_visualization': LaunchConfiguration(
                        'debug_fiducial_visualization'),
                    'debug_fiducial_display_seconds': LaunchConfiguration(
                        'debug_fiducial_display_seconds'),
                    'debug_orb_control_state': LaunchConfiguration(
                        'debug_orb_control_state'),
                    'gt_fallback_enabled': LaunchConfiguration(
                        'gt_fallback_enabled'),
                    'orb_qualification_samples': LaunchConfiguration(
                        'orb_qualification_samples'),
                }.items()),
        ])
        if i == 1:
            ld.add_action(drone_group)
        else:
            ld.add_action(TimerAction(
                period=PythonExpression([
                    LaunchConfiguration('drone_start_stagger_sec'), ' * ', str(i - 1)]),
                actions=[drone_group]))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(global_server_launch_path),
        launch_arguments={
            'config_dir': global_map_config_dir,
            'fiducial_objects_config': fiducial_objects_config,
            'fiducial_visual_min_distance_m': LaunchConfiguration(
                'fiducial_visual_min_distance_m'),
            'fiducial_visual_max_distance_m': LaunchConfiguration(
                'fiducial_visual_max_distance_m'),
            'fiducial_visual_consistency_translation_m': LaunchConfiguration(
                'fiducial_visual_consistency_translation_m'),
            'fiducial_visual_consistency_rotation_rad': LaunchConfiguration(
                'fiducial_visual_consistency_rotation_rad'),
            'fiducial_visual_visit_gap_sec': LaunchConfiguration(
                'fiducial_visual_visit_gap_sec'),
            'fiducial_visual_recent_capacity_per_drone': LaunchConfiguration(
                'fiducial_visual_recent_capacity_per_drone'),
            'use_sim_time': 'true',
            'drone_count': str(default_n),
            'drone_namespace_base': default_namespace_base,
            'rawdb_record_enabled': LaunchConfiguration('rawdb_record_enabled'),
            'rawdb_record_path': LaunchConfiguration('rawdb_record_path'),
            'debug_pipeline_flow_events': LaunchConfiguration('debug_pipeline_flow_web'),
            'debug_architecture_telemetry': architecture_telemetry_enabled,
            'log_level': PythonExpression([
                "'info' if '", LaunchConfiguration('debug_fase3_logs_terminal'),
                "'.lower() == 'true' else 'error'",
            ]),
        }.items()))

    return ld
