import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _yaml_bool(value, name):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in ('true', '1', 'yes', 'on'):
            return True
        if normalized in ('false', '0', 'no', 'off'):
            return False
    raise RuntimeError(f'{name} debe ser booleano')


def generate_launch_description():
    orbslam_launch_path = os.path.join(
        get_package_share_directory('dron_individual'),
        'launch',
        'orbslam_use.launch.py'
    )

    params_vision_path = os.path.join(
        get_package_share_directory('dron_individual'),
        'config',
        'vision.yaml'
    )
    with open(params_vision_path, 'r', encoding='utf-8') as stream:
        params_vision = yaml.safe_load(stream) or {}
    params_vision = params_vision.get('/**', {}).get('ros__parameters', {})
    activar_orbslam_default = _yaml_bool(
        params_vision.get('orbslam.activar', True), 'orbslam.activar')

    params_physical = PathJoinSubstitution([
        FindPackageShare('dron_individual'), 'config', 'physical.yaml'])
    params_control = PathJoinSubstitution([
        FindPackageShare('dron_individual'), 'config', 'control.yaml'])
    params_trajectory = PathJoinSubstitution([
        FindPackageShare('dron_individual'), 'config', 'trajectory.yaml'])
    params_actuators = PathJoinSubstitution([
        FindPackageShare('dron_individual'), 'config', 'actuators.yaml'])

    args = [
        DeclareLaunchArgument(
            'activar_orbslam',
            default_value=str(activar_orbslam_default).lower()),
        DeclareLaunchArgument('drone_id', default_value='1'),
        DeclareLaunchArgument('drone_name', default_value='drone_1'),
        DeclareLaunchArgument('local_map_frame', default_value='drone_1_orb_map'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'debug_architecture_telemetry', default_value='false'),
        DeclareLaunchArgument(
            'debug_fiducial_visualization', default_value='false'),
        DeclareLaunchArgument(
            'debug_fiducial_display_seconds', default_value='5.0'),
        DeclareLaunchArgument('debug_orb_control_state', default_value='false'),
        DeclareLaunchArgument('gt_fallback_enabled', default_value='false'),
        DeclareLaunchArgument('orb_qualification_samples', default_value='20'),
        DeclareLaunchArgument(
            'orb_vocabulary_path',
            default_value=PathJoinSubstitution([
                FindPackageShare('dron_individual'),
                'config', 'orbslam', 'vocabulary', 'ORBvoc.txt']))
    ]

    activar_orbslam = LaunchConfiguration('activar_orbslam')
    drone_id = LaunchConfiguration('drone_id')
    drone_name = LaunchConfiguration('drone_name')
    local_map_frame = LaunchConfiguration('local_map_frame')
    use_sim_time = LaunchConfiguration('use_sim_time')
    debug_architecture_telemetry = LaunchConfiguration(
        'debug_architecture_telemetry')
    debug_fiducial_visualization = LaunchConfiguration(
        'debug_fiducial_visualization')
    debug_fiducial_display_seconds = LaunchConfiguration(
        'debug_fiducial_display_seconds')
    debug_orb_control_state = LaunchConfiguration('debug_orb_control_state')
    gt_fallback_enabled = LaunchConfiguration('gt_fallback_enabled')
    orb_qualification_samples = LaunchConfiguration('orb_qualification_samples')
    orb_vocabulary_path = LaunchConfiguration('orb_vocabulary_path')

    common_debug = {
        'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        'drone_id': ParameterValue(drone_id, value_type=int),
        'debug_architecture_telemetry': ParameterValue(
            debug_architecture_telemetry, value_type=bool),
    }

    return LaunchDescription(args + [
        Node(
            package='dron_individual',
            executable='navigation_state_mux',
            name='navigation_state_mux',
            parameters=[common_debug, {
                'gt_fallback_enabled': ParameterValue(
                    gt_fallback_enabled, value_type=bool),
                'orb_qualification_samples': ParameterValue(
                    orb_qualification_samples, value_type=int),
            }],
        ),
        Node(
            package='dron_individual',
            executable='gen_tray',
            name='gen_tray',
            parameters=[common_debug, params_trajectory],
        ),
        Node(
            package='dron_individual',
            executable='control_calcular_fuerzas',
            name='control_calcular_fuerzas',
            parameters=[
                {
                    'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                    'debug_orb_control_state': ParameterValue(
                        debug_orb_control_state, value_type=bool),
                },
                params_physical,
                params_control,
            ],
        ),
        Node(
            package='dron_individual',
            executable='aplicar_fuerzas_dron',
            name='aplicar_fuerzas_dron',
            parameters=[common_debug, params_actuators],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(orbslam_launch_path),
            condition=IfCondition(activar_orbslam),
            launch_arguments={
                'drone_id': drone_id,
                'drone_name': drone_name,
                'local_map_frame': local_map_frame,
                'use_sim_time': use_sim_time,
                'debug_architecture_telemetry': debug_architecture_telemetry,
                'debug_fiducial_visualization':
                    debug_fiducial_visualization,
                'debug_fiducial_display_seconds':
                    debug_fiducial_display_seconds,
                'debug_orb_control_state': debug_orb_control_state,
                'vocab': orb_vocabulary_path,
            }.items(),
        ),
    ])
