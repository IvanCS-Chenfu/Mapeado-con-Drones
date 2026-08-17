from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node

from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

import yaml
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    orbslam_launch_path = os.path.join(
        get_package_share_directory('dron_individual'),
        'launch',
        'orbslam_use.launch.py'
    )

    # Parámetros trayectoria
    params_tray_path = os.path.join(
        get_package_share_directory('dron_individual'),
        'config',
        'tray_dron.yaml'
    )

    with open(params_tray_path, 'r') as f:
        params_tray_get = yaml.safe_load(f) or {}

    params_tray_get = params_tray_get.get('/**', {}).get('ros__parameters', {})
    default_usar_veltrap = str(params_tray_get.get('control.tray.usar_veltrap', 'false'))

    # Parámetros visión
    params_vision_path = os.path.join(
        get_package_share_directory('dron_individual'),
        'config',
        'vision.yaml'
    )

    with open(params_vision_path, 'r') as f:
        params_vision = yaml.safe_load(f) or {}

    params_vision = params_vision.get('/**', {}).get('ros__parameters', {})
    default_activar_orbslam = str(params_vision.get('orbslam.activar', 'false'))

    params_tray = PathJoinSubstitution([
        FindPackageShare('dron_individual'),
        'config',
        'tray_dron.yaml'
    ])

    # Argumentos
    usar_veltrap_arg = DeclareLaunchArgument(
        'usar_veltrap',
        default_value=default_usar_veltrap
    )

    activar_orbslam_arg = DeclareLaunchArgument(
        'activar_orbslam',
        default_value=default_activar_orbslam
    )

    drone_id_arg = DeclareLaunchArgument(
        'drone_id',
        default_value='1'
    )

    drone_name_arg = DeclareLaunchArgument(
        'drone_name',
        default_value='drone_1'
    )

    local_map_frame_arg = DeclareLaunchArgument(
        'local_map_frame',
        default_value='drone_1_orb_map'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true'
    )

    orb_vocabulary_path_arg = DeclareLaunchArgument(
        'orb_vocabulary_path',
        default_value=PathJoinSubstitution([
            FindPackageShare('dron_individual'),
            'config',
            'orbslam',
            'vocabulary',
            'ORBvoc.txt',
        ])
    )

    activar_orbslam = LaunchConfiguration('activar_orbslam')
    drone_id = LaunchConfiguration('drone_id')
    drone_name = LaunchConfiguration('drone_name')
    local_map_frame = LaunchConfiguration('local_map_frame')
    use_sim_time = LaunchConfiguration('use_sim_time')
    orb_vocabulary_path = LaunchConfiguration('orb_vocabulary_path')

    return LaunchDescription([
        usar_veltrap_arg,
        activar_orbslam_arg,
        drone_id_arg,
        drone_name_arg,
        local_map_frame_arg,
        use_sim_time_arg,
        orb_vocabulary_path_arg,

        Node(
            package='dron_individual',
            executable='gen_tray',
            name='gen_tray',
            parameters=[
                {'use_sim_time': use_sim_time},
                params_tray
            ]
        ),

        Node(
            package='dron_individual',
            executable='control_calcular_fuerzas',
            name='control_calcular_fuerzas',
            parameters=[
                {'use_sim_time': use_sim_time},
                params_tray
            ]
        ),

        Node(
            package='dron_individual',
            executable='aplicar_fuerzas_dron',
            name='aplicar_fuerzas_dron',
            parameters=[
                {'use_sim_time': use_sim_time},
                params_tray
            ]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(orbslam_launch_path),
            condition=IfCondition(activar_orbslam),
            launch_arguments={
                'drone_id': drone_id,
                'drone_name': drone_name,
                'local_map_frame': local_map_frame,
                'use_sim_time': use_sim_time,
                'vocab': orb_vocabulary_path,
            }.items()
        ),
    ])
