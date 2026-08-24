import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
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

    common_params = {
        'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        'drone_id': ParameterValue(drone_id, value_type=int),
        'drone_name': ParameterValue(drone_name, value_type=str),
        'local_map_frame': ParameterValue(local_map_frame, value_type=str),
        'use_corrected_keyframes': ParameterValue(True, value_type=bool),
        'max_nearest_kf_distance_m': ParameterValue(2.0, value_type=float),
    }
    stereo_params = dict(common_params)
    stereo_params['debug_architecture_telemetry'] = ParameterValue(
        debug_architecture_telemetry, value_type=bool)

    mono_node = Node(
        condition=UnlessCondition(usar_estereo),
        package='orbslam3', executable='mono', name='orbslam3_mono',
        output='screen', additional_env={'MALLOC_ARENA_MAX': '2'},
        arguments=[vocab, yaml_mono], parameters=[common_params],
        remappings=[('camera', 'sensor/camara_mono/image_raw')])

    stereo_node = Node(
        condition=IfCondition(usar_estereo),
        package='orbslam3', executable='stereo', name='orbslam3_stereo',
        output='screen', additional_env={'MALLOC_ARENA_MAX': '2'},
        arguments=[vocab, yaml_stereo, rectify], parameters=[stereo_params],
        remappings=[
            ('camera/left', 'sensor/camara_izq/image_raw'),
            ('camera/right', 'sensor/camara_der/image_raw'),
        ])

    return LaunchDescription(args + [mono_node, stereo_node])
