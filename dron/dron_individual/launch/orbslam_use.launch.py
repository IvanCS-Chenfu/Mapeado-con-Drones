from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.conditions import IfCondition, UnlessCondition

import yaml
import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    params_sim_path = os.path.join(
        get_package_share_directory('dron_individual'),
        'config',
        'vision.yaml'
    )

    with open(params_sim_path, 'r') as f:
        params_sim = yaml.safe_load(f) or {}

    params_sim = params_sim.get('/**', {}).get('ros__parameters', {})
    usar_estereo_default = str(params_sim.get('orbslam.estereo', 'true'))

    default_vocab = PathJoinSubstitution([
        FindPackageShare('dron_individual'),
        'config',
        'orbslam',
        'vocabulary',
        'ORBvoc.txt'
    ])

    default_yaml_mono = PathJoinSubstitution([
        FindPackageShare('dron_individual'),
        'config',
        'orbslam',
        'orbslam_mono.yaml'
    ])

    default_yaml_stereo = PathJoinSubstitution([
        FindPackageShare('dron_individual'),
        'config',
        'orbslam',
        'orbslam_stereo.yaml'
    ])

    vocab_arg = DeclareLaunchArgument('vocab', default_value=default_vocab)
    yaml_mono_arg = DeclareLaunchArgument('yaml_mono', default_value=default_yaml_mono)
    yaml_stereo_arg = DeclareLaunchArgument('yaml_stereo', default_value=default_yaml_stereo)
    rectify_arg = DeclareLaunchArgument('rectify', default_value='false')

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    usar_estereo_arg = DeclareLaunchArgument('usar_estereo', default_value=usar_estereo_default)

    drone_id_arg = DeclareLaunchArgument('drone_id', default_value='1')
    drone_name_arg = DeclareLaunchArgument('drone_name', default_value='drone_1')
    local_map_frame_arg = DeclareLaunchArgument('local_map_frame', default_value='drone_1_orb_map')

    vocab = LaunchConfiguration('vocab')
    yaml_mono = LaunchConfiguration('yaml_mono')
    yaml_stereo = LaunchConfiguration('yaml_stereo')
    rectify = LaunchConfiguration('rectify')

    use_sim_time = LaunchConfiguration('use_sim_time')
    usar_estereo = LaunchConfiguration('usar_estereo')

    drone_id = LaunchConfiguration('drone_id')
    drone_name = LaunchConfiguration('drone_name')
    local_map_frame = LaunchConfiguration('local_map_frame')

    common_params = {
        'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        'drone_id': ParameterValue(drone_id, value_type=int),
        'drone_name': ParameterValue(drone_name, value_type=str),
        'local_map_frame': ParameterValue(local_map_frame, value_type=str),
        'use_corrected_keyframes': ParameterValue(True, value_type=bool),
        'max_nearest_kf_distance_m': ParameterValue(2.0, value_type=float),
    }

    mono_node = Node(
        condition=UnlessCondition(usar_estereo),
        package='orbslam3',
        executable='mono',
        name='orbslam3_mono',
        output='screen',
        additional_env={'MALLOC_ARENA_MAX': '2'},
        arguments=[vocab, yaml_mono],
        parameters=[common_params],
        remappings=[
            ('camera', 'sensor/camara_mono/image_raw')
        ],
    )

    stereo_node = Node(
        condition=IfCondition(usar_estereo),
        package='orbslam3',
        executable='stereo',
        name='orbslam3_stereo',
        output='screen',
        additional_env={'MALLOC_ARENA_MAX': '2'},
        arguments=[vocab, yaml_stereo, rectify],
        parameters=[common_params],
        remappings=[
            ('camera/left',  'sensor/camara_izq/image_raw'),
            ('camera/right', 'sensor/camara_der/image_raw')
        ],
    )

    return LaunchDescription([
        vocab_arg,
        yaml_mono_arg,
        yaml_stereo_arg,
        rectify_arg,

        use_sim_time_arg,
        usar_estereo_arg,

        drone_id_arg,
        drone_name_arg,
        local_map_frame_arg,

        mono_node,
        stereo_node,
    ])
