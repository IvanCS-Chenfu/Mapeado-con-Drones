import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
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
    trajectory_config = LaunchConfiguration('trajectory_config')
    params_trajectory = PathJoinSubstitution([
        FindPackageShare('dron_individual'), 'config', trajectory_config])
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
        DeclareLaunchArgument('debug_fase_1', default_value='false'),
        DeclareLaunchArgument(
            'debug_fiducial_visualization', default_value='false'),
        DeclareLaunchArgument(
            'debug_fiducial_display_seconds', default_value='5.0'),
        DeclareLaunchArgument('debug_fiducial_gt_error', default_value='false'),
        DeclareLaunchArgument(
            'debug_fiducial_gt_error_objects_config', default_value=''),
        DeclareLaunchArgument(
            'debug_fiducial_gt_error_rendering_config', default_value=''),
        DeclareLaunchArgument(
            'debug_fiducial_gt_error_max_skew_sec', default_value='0.075'),
        DeclareLaunchArgument('debug_fase_5', default_value='false'),
        DeclareLaunchArgument('debug_orb_control_state', default_value='false'),
        DeclareLaunchArgument('debug_f6i_trajectory', default_value='false'),
        DeclareLaunchArgument('debug_orb_visual_evidence', default_value='false'),
        DeclareLaunchArgument('debug_visual_risk_display', default_value='false'),
        DeclareLaunchArgument('visual_risk_empty_region_fraction', default_value='0.75'),
        DeclareLaunchArgument('orb_visual_evidence_output_dir', default_value=''),
        DeclareLaunchArgument(
            'orb_navigation_prediction_mode', default_value='dynamic'),
        DeclareLaunchArgument('depth_observation_enabled', default_value='false'),
        DeclareLaunchArgument('depth_max_points', default_value='128'),
        DeclareLaunchArgument('depth_max_disparity_gradient_px_per_pixel', default_value='2.0'),
        DeclareLaunchArgument('depth_min_texture_gradient', default_value='8.0'),
        DeclareLaunchArgument('depth_texture_window_radius_px', default_value='2'),
        DeclareLaunchArgument('depth_stop_enabled', default_value='false'),
        DeclareLaunchArgument('depth_stop_distance_m', default_value='1.2'),
        DeclareLaunchArgument('depth_stop_cooldown_sec', default_value='5.0'),
        DeclareLaunchArgument('phase5_navigation_source', default_value='orb'),
        DeclareLaunchArgument('camera_pitch_enabled', default_value='false'),
        DeclareLaunchArgument('gt_fallback_enabled', default_value='false'),
        DeclareLaunchArgument('orb_loss_protocol_enabled', default_value='true'),
        DeclareLaunchArgument('orb_loss_hold_sec', default_value='10.0'),
        DeclareLaunchArgument('waypoint_blend_sec', default_value='3.0'),
        DeclareLaunchArgument('trajectory_config', default_value='trajectory.yaml'),
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
    debug_fase_1 = LaunchConfiguration('debug_fase_1')
    debug_fiducial_visualization = LaunchConfiguration(
        'debug_fiducial_visualization')
    debug_fiducial_display_seconds = LaunchConfiguration(
        'debug_fiducial_display_seconds')
    debug_fiducial_gt_error = LaunchConfiguration('debug_fiducial_gt_error')
    debug_fiducial_gt_error_objects_config = LaunchConfiguration(
        'debug_fiducial_gt_error_objects_config')
    debug_fiducial_gt_error_rendering_config = LaunchConfiguration(
        'debug_fiducial_gt_error_rendering_config')
    debug_fiducial_gt_error_max_skew_sec = LaunchConfiguration(
        'debug_fiducial_gt_error_max_skew_sec')
    debug_fase_5 = LaunchConfiguration('debug_fase_5')
    debug_orb_control_state = LaunchConfiguration('debug_orb_control_state')
    debug_f6i_trajectory = LaunchConfiguration('debug_f6i_trajectory')
    debug_orb_visual_evidence = LaunchConfiguration('debug_orb_visual_evidence')
    debug_visual_risk_display = LaunchConfiguration('debug_visual_risk_display')
    visual_risk_empty_region_fraction = LaunchConfiguration('visual_risk_empty_region_fraction')
    orb_visual_evidence_output_dir = LaunchConfiguration(
        'orb_visual_evidence_output_dir')
    orb_navigation_prediction_mode = LaunchConfiguration(
        'orb_navigation_prediction_mode')
    depth_observation_enabled = LaunchConfiguration('depth_observation_enabled')
    depth_max_points = LaunchConfiguration('depth_max_points')
    depth_max_disparity_gradient_px_per_pixel = LaunchConfiguration(
        'depth_max_disparity_gradient_px_per_pixel')
    depth_min_texture_gradient = LaunchConfiguration('depth_min_texture_gradient')
    depth_texture_window_radius_px = LaunchConfiguration('depth_texture_window_radius_px')
    depth_stop_enabled = LaunchConfiguration('depth_stop_enabled')
    depth_stop_distance_m = LaunchConfiguration('depth_stop_distance_m')
    depth_stop_cooldown_sec = LaunchConfiguration('depth_stop_cooldown_sec')
    phase5_navigation_source = LaunchConfiguration('phase5_navigation_source')
    camera_pitch_enabled = LaunchConfiguration('camera_pitch_enabled')
    gt_fallback_enabled = LaunchConfiguration('gt_fallback_enabled')
    orb_loss_protocol_enabled = LaunchConfiguration('orb_loss_protocol_enabled')
    orb_loss_hold_sec = LaunchConfiguration('orb_loss_hold_sec')
    waypoint_blend_sec = LaunchConfiguration('waypoint_blend_sec')
    orb_qualification_samples = LaunchConfiguration('orb_qualification_samples')
    orb_vocabulary_path = LaunchConfiguration('orb_vocabulary_path')

    common_debug = {
        'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        'drone_id': ParameterValue(drone_id, value_type=int),
        'debug_architecture_telemetry': ParameterValue(
            debug_architecture_telemetry, value_type=bool),
    }
    phase1_log_level = PythonExpression([
        "'info' if '", debug_fase_1, "'.lower() == 'true' else 'warn'",
    ])

    return LaunchDescription(args + [
        Node(
            package='dron_individual',
            executable='navigation_state_mux',
            name='navigation_state_mux',
            parameters=[common_debug, {
                'gt_fallback_enabled': ParameterValue(
                    gt_fallback_enabled, value_type=bool),
                'phase5_navigation_source': ParameterValue(
                    phase5_navigation_source, value_type=str),
                'orb_loss_hold_sec': ParameterValue(orb_loss_hold_sec, value_type=float),
                'body_frame': ParameterValue(
                    [drone_name, '/base_link'], value_type=str),
                'orb_qualification_samples': ParameterValue(
                    orb_qualification_samples, value_type=int),
            }],
            arguments=['--ros-args', '--log-level', phase1_log_level],
        ),
        Node(
            package='dron_individual',
            executable='gen_tray',
            name='gen_tray',
            parameters=[common_debug, params_trajectory, {
                'orb_loss_protocol_enabled': ParameterValue(
                    orb_loss_protocol_enabled, value_type=bool),
                'orb_loss_hold_sec': ParameterValue(orb_loss_hold_sec, value_type=float),
                'waypoint_blend_sec': ParameterValue(waypoint_blend_sec, value_type=float),
                'debug_f6i_trajectory': ParameterValue(
                    debug_f6i_trajectory, value_type=bool),
            }],
            arguments=['--ros-args', '--log-level', phase1_log_level],
        ),
        Node(
            package='dron_individual',
            executable='control_calcular_fuerzas',
            name='control_calcular_fuerzas',
            parameters=[
                {
                    'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                    'debug_orb_control_state': ParameterValue(
                        PythonExpression([
                            "'", debug_fase_5, "'.lower() == 'true' and '",
                            debug_orb_control_state, "'.lower() == 'true'",
                        ]), value_type=bool),
                    'debug_f6i_trajectory': ParameterValue(
                        debug_f6i_trajectory, value_type=bool),
                },
                params_physical,
                params_control,
            ],
            arguments=['--ros-args', '--log-level', phase1_log_level],
        ),
        Node(
            package='dron_individual',
            executable='aplicar_fuerzas_dron',
            name='aplicar_fuerzas_dron',
            parameters=[common_debug, params_actuators],
            arguments=['--ros-args', '--log-level', phase1_log_level],
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
                'debug_fiducial_gt_error': debug_fiducial_gt_error,
                'debug_fiducial_gt_error_objects_config':
                    debug_fiducial_gt_error_objects_config,
                'debug_fiducial_gt_error_rendering_config':
                    debug_fiducial_gt_error_rendering_config,
                'debug_fiducial_gt_error_max_skew_sec':
                    debug_fiducial_gt_error_max_skew_sec,
                'debug_fase_5': debug_fase_5,
                'debug_orb_control_state': debug_orb_control_state,
                'debug_orb_visual_evidence': debug_orb_visual_evidence,
                'debug_visual_risk_display': debug_visual_risk_display,
                'visual_risk_empty_region_fraction': visual_risk_empty_region_fraction,
                'orb_visual_evidence_output_dir':
                    orb_visual_evidence_output_dir,
                'orb_navigation_prediction_mode':
                    orb_navigation_prediction_mode,
                'depth_observation_enabled': depth_observation_enabled,
                'depth_max_disparity_gradient_px_per_pixel':
                    depth_max_disparity_gradient_px_per_pixel,
                'depth_min_texture_gradient': depth_min_texture_gradient,
                'depth_texture_window_radius_px': depth_texture_window_radius_px,
                'depth_stop_enabled': depth_stop_enabled,
                'depth_stop_distance_m': depth_stop_distance_m,
                'depth_stop_cooldown_sec': depth_stop_cooldown_sec,
                'depth_max_points': depth_max_points,
                'camera_pitch_enabled': camera_pitch_enabled,
                'vocab': orb_vocabulary_path,
            }.items(),
        ),
    ])
