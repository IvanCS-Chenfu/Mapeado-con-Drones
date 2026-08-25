import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


CONFIG_FILES = (
    'runtime.yaml',
    'fiducials.yaml',
    'optimization.yaml',
    'loop_fusion.yaml',
    'scoring.yaml',
)
YAML_SENTINEL = '__from_yaml__'


def _as_bool(value):
    normalized = value.strip().lower()
    if normalized in ('true', '1', 'yes', 'on'):
        return True
    if normalized in ('false', '0', 'no', 'off'):
        return False
    raise ValueError(f'booleano invalido: {value}')


def _optional_override(context, overrides, name, cast):
    value = LaunchConfiguration(name).perform(context)
    if value != YAML_SENTINEL:
        overrides[name] = cast(value)


def _launch_server(context):
    config_dir = LaunchConfiguration('config_dir').perform(context)
    parameters = [os.path.join(config_dir, name) for name in CONFIG_FILES]
    replay_debug_config = LaunchConfiguration('replay_debug_config').perform(context)
    if replay_debug_config:
        parameters.append(replay_debug_config)

    # Reloj, identidad y observabilidad son autoridad del deployment/launch.
    overrides = {
        'use_sim_time': _as_bool(LaunchConfiguration('use_sim_time').perform(context)),
        'drone_count': int(LaunchConfiguration('drone_count').perform(context)),
        'drone_namespace_base': LaunchConfiguration(
            'drone_namespace_base').perform(context),
        'debug_pipeline_flow_events': _as_bool(
            LaunchConfiguration('debug_pipeline_flow_events').perform(context)),
        'debug_architecture_telemetry': _as_bool(
            LaunchConfiguration('debug_architecture_telemetry').perform(context)),
        'fiducial_objects_config': LaunchConfiguration(
            'fiducial_objects_config').perform(context),
    }
    optional = (
        ('rawdb_record_enabled', _as_bool),
        ('rawdb_record_path', str),
        ('rawdb_replay_path', str),
        ('rawdb_replay_entry_delay_ms', int),
        ('pose_store_debug_anchor_enabled', _as_bool),
        ('pose_store_debug_anchor_drone_id', int),
        ('pose_store_debug_anchor_map_epoch', int),
        ('pose_store_debug_anchor_x', float),
        ('pose_store_debug_anchor_y', float),
        ('pose_store_debug_anchor_z', float),
        ('fiducial_translation_threshold_m', float),
        ('fiducial_rotation_threshold_rad', float),
        ('fiducial_yaw_threshold_rad', float),
        ('fiducial_visual_min_distance_m', float),
        ('fiducial_visual_max_distance_m', float),
        ('fiducial_visual_consistency_translation_m', float),
        ('fiducial_visual_consistency_rotation_rad', float),
        ('fiducial_visual_visit_gap_sec', float),
        ('fiducial_visual_recent_capacity_per_drone', int),
    )
    for name, cast in optional:
        _optional_override(context, overrides, name, cast)

    parameters.append(overrides)
    log_level = LaunchConfiguration('log_level').perform(context).lower()
    if log_level not in ('debug', 'info', 'warn', 'error', 'fatal'):
        raise ValueError(f'nivel de log ROS invalido: {log_level}')

    config_server = Node(
        package='orbslam3_server',
        executable='fiducial_config_server.py',
        name='fiducial_config_server',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration(
                'fiducial_objects_config').perform(context),
            'use_sim_time': overrides['use_sim_time'],
            'debug_architecture_telemetry': overrides[
                'debug_architecture_telemetry'],
        }],
    )

    global_server = Node(
        package='orbslam3_server',
        executable='global_map_server',
        name='global_map_server',
        output='screen',
        parameters=parameters,
        arguments=['--ros-args', '--log-level', log_level],
    )
    return [config_server, global_server]


def generate_launch_description():
    default_config_dir = PathJoinSubstitution([
        FindPackageShare('orbslam3_server'), 'config', 'global_map'])
    default_fiducial_objects = PathJoinSubstitution([
        FindPackageShare('orbslam3_server'), 'config',
        'fiducial_objects.yaml'])

    arguments = [
        DeclareLaunchArgument('config_dir', default_value=default_config_dir),
        DeclareLaunchArgument(
            'fiducial_objects_config', default_value=default_fiducial_objects),
        DeclareLaunchArgument('replay_debug_config', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('drone_count', default_value='2'),
        DeclareLaunchArgument('drone_namespace_base', default_value='dron'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument(
            'debug_pipeline_flow_events', default_value='false'),
        DeclareLaunchArgument(
            'debug_architecture_telemetry', default_value='false'),
    ]
    for name in (
        'rawdb_record_enabled',
        'rawdb_record_path',
        'rawdb_replay_path',
        'rawdb_replay_entry_delay_ms',
        'pose_store_debug_anchor_enabled',
        'pose_store_debug_anchor_drone_id',
        'pose_store_debug_anchor_map_epoch',
        'pose_store_debug_anchor_x',
        'pose_store_debug_anchor_y',
        'pose_store_debug_anchor_z',
        'fiducial_translation_threshold_m',
        'fiducial_rotation_threshold_rad',
        'fiducial_yaw_threshold_rad',
        'fiducial_visual_min_distance_m',
        'fiducial_visual_max_distance_m',
        'fiducial_visual_consistency_translation_m',
        'fiducial_visual_consistency_rotation_rad',
        'fiducial_visual_visit_gap_sec',
        'fiducial_visual_recent_capacity_per_drone',
    ):
        arguments.append(DeclareLaunchArgument(name, default_value=YAML_SENTINEL))

    return LaunchDescription(arguments + [OpaqueFunction(function=_launch_server)])
