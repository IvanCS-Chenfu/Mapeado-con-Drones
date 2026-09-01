import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
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
    orbslam_environment = {'MALLOC_ARENA_MAX': '2'}
    for key, value in os.environ.items():
        if key.startswith('SNAP') or key.startswith('VSCODE_'):
            orbslam_environment[key] = ''
            continue
        cleaned_parts = [
            part for part in value.split(':')
            if '/snap/' not in part and '/snapd/' not in part]
        if len(cleaned_parts) != len(value.split(':')):
            orbslam_environment[key] = ':'.join(cleaned_parts)

    params_vision_path = os.path.join(
        get_package_share_directory('dron_individual'),
        'config',
        'vision.yaml'
    )
    with open(params_vision_path, 'r', encoding='utf-8') as stream:
        params_vision = yaml.safe_load(stream) or {}
    params_vision = params_vision.get('/**', {}).get('ros__parameters', {})
    usar_estereo_default = _yaml_bool(
        params_vision.get('orbslam.estereo', True), 'orbslam.estereo')

    default_vocab = PathJoinSubstitution([
        FindPackageShare('dron_individual'),
        'config', 'orbslam', 'vocabulary', 'ORBvoc.txt'])
    default_yaml_mono = PathJoinSubstitution([
        FindPackageShare('dron_individual'),
        'config', 'orbslam', 'orbslam_mono.yaml'])
    default_yaml_stereo = PathJoinSubstitution([
        FindPackageShare('dron_individual'),
        'config', 'orbslam', 'orbslam_stereo.yaml'])
    calibration_params = PathJoinSubstitution([
        FindPackageShare('dron_individual'), 'config', 'calibration.yaml'])
    navigation_state_params = PathJoinSubstitution([
        FindPackageShare('dron_individual'), 'config', 'navigation_state.yaml'])
    physical_params = PathJoinSubstitution([
        FindPackageShare('dron_individual'), 'config', 'physical.yaml'])

    args = [
        DeclareLaunchArgument('vocab', default_value=default_vocab),
        DeclareLaunchArgument('yaml_mono', default_value=default_yaml_mono),
        DeclareLaunchArgument('yaml_stereo', default_value=default_yaml_stereo),
        DeclareLaunchArgument('rectify', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'usar_estereo', default_value=str(usar_estereo_default).lower()),
        DeclareLaunchArgument('drone_id', default_value='1'),
        DeclareLaunchArgument('drone_name', default_value='drone_1'),
        DeclareLaunchArgument('local_map_frame', default_value='drone_1_orb_map'),
        DeclareLaunchArgument(
            'debug_architecture_telemetry', default_value='false'),
        DeclareLaunchArgument(
            'debug_fiducial_visualization', default_value='false'),
        DeclareLaunchArgument(
            'debug_fiducial_display_seconds', default_value='5.0'),
        DeclareLaunchArgument('debug_fase_5', default_value='false'),
        DeclareLaunchArgument('debug_orb_control_state', default_value='false'),
        DeclareLaunchArgument('debug_orb_visual_evidence', default_value='false'),
        DeclareLaunchArgument('orb_visual_evidence_output_dir', default_value=''),
        DeclareLaunchArgument(
            'orb_navigation_prediction_mode', default_value='dynamic'),
        DeclareLaunchArgument('camera_pitch_enabled', default_value='false'),
    ]

    vocab = LaunchConfiguration('vocab')
    yaml_mono = LaunchConfiguration('yaml_mono')
    yaml_stereo = LaunchConfiguration('yaml_stereo')
    rectify = LaunchConfiguration('rectify')
    use_sim_time = LaunchConfiguration('use_sim_time')
    usar_estereo = LaunchConfiguration('usar_estereo')
    drone_id = LaunchConfiguration('drone_id')
    drone_name = LaunchConfiguration('drone_name')
    local_map_frame = LaunchConfiguration('local_map_frame')
    debug_architecture_telemetry = LaunchConfiguration(
        'debug_architecture_telemetry')
    debug_fiducial_visualization = LaunchConfiguration(
        'debug_fiducial_visualization')
    debug_fiducial_display_seconds = LaunchConfiguration(
        'debug_fiducial_display_seconds')
    debug_fase_5 = LaunchConfiguration('debug_fase_5')
    debug_orb_control_state = LaunchConfiguration('debug_orb_control_state')
    debug_orb_visual_evidence = LaunchConfiguration('debug_orb_visual_evidence')
    orb_visual_evidence_output_dir = LaunchConfiguration(
        'orb_visual_evidence_output_dir')
    orb_navigation_prediction_mode = LaunchConfiguration(
        'orb_navigation_prediction_mode')
    camera_pitch_enabled = LaunchConfiguration('camera_pitch_enabled')

    common_params = {
        'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        'drone_id': ParameterValue(drone_id, value_type=int),
        'drone_name': ParameterValue(drone_name, value_type=str),
        'local_map_frame': ParameterValue(local_map_frame, value_type=str),
        'odom_frame': ParameterValue(
            [drone_name, '_odom'], value_type=str),
        'body_frame': ParameterValue(
            [drone_name, '/base_link'], value_type=str),
        'use_corrected_keyframes': ParameterValue(True, value_type=bool),
        'max_nearest_kf_distance_m': ParameterValue(2.0, value_type=float),
    }
    stereo_params = dict(common_params)
    stereo_params['debug_architecture_telemetry'] = ParameterValue(
        debug_architecture_telemetry, value_type=bool)
    stereo_params['debug_fiducial_visualization'] = ParameterValue(
        debug_fiducial_visualization, value_type=bool)
    stereo_params['debug_orb_control_state'] = ParameterValue(
        PythonExpression([
            "'", debug_fase_5, "'.lower() == 'true' and '",
            debug_orb_control_state, "'.lower() == 'true'",
        ]), value_type=bool)
    stereo_params['debug_orb_visual_evidence'] = ParameterValue(
        PythonExpression([
            "'", debug_fase_5, "'.lower() == 'true' and '",
            debug_orb_visual_evidence, "'.lower() == 'true'",
        ]), value_type=bool)
    stereo_params['orb_visual_evidence_output_dir'] = ParameterValue(
        orb_visual_evidence_output_dir, value_type=str)
    stereo_params['navigation_prediction_mode'] = ParameterValue(
        orb_navigation_prediction_mode, value_type=str)
    stereo_params['body_camera_transform_mode'] = ParameterValue(
        PythonExpression([
            "'tf' if '", camera_pitch_enabled,
            "'.lower() == 'true' else 'static'",
        ]), value_type=str)
    stereo_params['camera_frame'] = ParameterValue(
        [drone_name, '/camera_izq_optical_frame'], value_type=str)

    mono_node = Node(
        condition=UnlessCondition(usar_estereo),
        package='orbslam3', executable='mono', name='orbslam3_mono',
        output='screen', additional_env=orbslam_environment,
        arguments=[vocab, yaml_mono], parameters=[common_params],
        remappings=[('camera', 'sensor/camara_mono/image_raw')])

    stereo_node = Node(
        condition=IfCondition(usar_estereo),
        package='orbslam3', executable='stereo', name='orbslam3_stereo',
        output='screen', additional_env=orbslam_environment,
        arguments=[vocab, yaml_stereo, rectify],
        parameters=[
            calibration_params, navigation_state_params, physical_params,
            stereo_params],
        remappings=[
            ('camera/left', 'sensor/camara_izq/image_raw'),
            ('camera/right', 'sensor/camara_der/image_raw'),
            ('orbslam/navigation_state',
             'orbslam/navigation_state_orb'),
        ])

    fiducial_visualizer_node = Node(
        condition=IfCondition(debug_fiducial_visualization),
        package='orbslam3', executable='fiducial_visualizer',
        name='fiducial_visualizer', output='screen',
        additional_env=orbslam_environment,
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'drone_id': ParameterValue(drone_id, value_type=int),
            'drone_name': ParameterValue(drone_name, value_type=str),
            'display_seconds': ParameterValue(
                debug_fiducial_display_seconds, value_type=float),
        }])

    return LaunchDescription(
        args + [mono_node, stereo_node, fiducial_visualizer_node])
