from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('drone_count', default_value='2'),
        DeclareLaunchArgument('drone_namespace_base', default_value='dron'),
        DeclareLaunchArgument('primary_queue_high_watermark', default_value='8'),
        DeclareLaunchArgument('primary_queue_low_watermark', default_value='2'),
        DeclareLaunchArgument('secondary_queue_high_watermark', default_value='64'),
        DeclareLaunchArgument('secondary_queue_low_watermark', default_value='16'),
        DeclareLaunchArgument('primary_worker_debug_delay_ms', default_value='0'),
        DeclareLaunchArgument('rawdb_record_enabled', default_value='false'),
        DeclareLaunchArgument('rawdb_record_path', default_value='/tmp/f3c_raw.record'),
        DeclareLaunchArgument('rawdb_replay_path', default_value=''),
        DeclareLaunchArgument('rawdb_replay_entry_delay_ms', default_value='0'),
        DeclareLaunchArgument('full_snapshot_enabled', default_value='true'),
        DeclareLaunchArgument('full_snapshot_startup_delay_sec', default_value='35.0'),
        DeclareLaunchArgument('full_snapshot_period_sec', default_value='35.0'),
        DeclareLaunchArgument(
            'debug_drop_one_delta_for_snapshot_test', default_value='false'),
        DeclareLaunchArgument('debug_drop_delta_drone_id', default_value='1'),
        DeclareLaunchArgument('pose_store_debug_anchor_enabled', default_value='false'),
        DeclareLaunchArgument('pose_store_debug_anchor_drone_id', default_value='1'),
        DeclareLaunchArgument('pose_store_debug_anchor_map_epoch', default_value='0'),
        DeclareLaunchArgument('pose_store_debug_anchor_x', default_value='10.0'),
        DeclareLaunchArgument('pose_store_debug_anchor_y', default_value='0.0'),
        DeclareLaunchArgument('pose_store_debug_anchor_z', default_value='0.0'),
        DeclareLaunchArgument('fiducial_sim_enabled', default_value='true'),
        DeclareLaunchArgument('fiducial_gt_max_dt_sec', default_value='1.0'),
        DeclareLaunchArgument('fiducial_translation_threshold_m', default_value='0.35'),
        DeclareLaunchArgument('fiducial_rotation_threshold_rad', default_value='0.35'),
        DeclareLaunchArgument('fiducial_yaw_threshold_rad', default_value='0.25'),
        DeclareLaunchArgument('pose_graph_control_vertex_ratio', default_value='0.30'),
        DeclareLaunchArgument(
            'pose_graph_endpoint_neighborhood_ratio', default_value='0.20'),
        DeclareLaunchArgument(
            'fiducial_max_correction_fraction_per_pass', default_value='1.0'),
        DeclareLaunchArgument('fiducial_max_refinement_passes', default_value='4'),
        DeclareLaunchArgument('body_T_camera_x', default_value='0.10'),
        DeclareLaunchArgument('body_T_camera_y', default_value='0.03'),
        DeclareLaunchArgument('body_T_camera_z', default_value='0.03'),
        DeclareLaunchArgument('body_T_camera_roll_deg', default_value='0.0'),
        DeclareLaunchArgument('body_T_camera_pitch_deg', default_value='-90.0'),
        DeclareLaunchArgument('body_T_camera_yaw_deg', default_value='90.0'),
        DeclareLaunchArgument(
            'use_camera_optical_frame_convention', default_value='true'),
    ]
    server = Node(
        package='orbslam3_server',
        executable='global_map_server',
        name='global_map_server',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(
                LaunchConfiguration('use_sim_time'), value_type=bool),
            'drone_count': ParameterValue(
                LaunchConfiguration('drone_count'), value_type=int),
            'drone_namespace_base': LaunchConfiguration('drone_namespace_base'),
            'primary_queue_high_watermark': ParameterValue(
                LaunchConfiguration('primary_queue_high_watermark'), value_type=int),
            'primary_queue_low_watermark': ParameterValue(
                LaunchConfiguration('primary_queue_low_watermark'), value_type=int),
            'secondary_queue_high_watermark': ParameterValue(
                LaunchConfiguration('secondary_queue_high_watermark'), value_type=int),
            'secondary_queue_low_watermark': ParameterValue(
                LaunchConfiguration('secondary_queue_low_watermark'), value_type=int),
            'primary_worker_debug_delay_ms': ParameterValue(
                LaunchConfiguration('primary_worker_debug_delay_ms'), value_type=int),
            'rawdb_record_enabled': ParameterValue(
                LaunchConfiguration('rawdb_record_enabled'), value_type=bool),
            'rawdb_record_path': LaunchConfiguration('rawdb_record_path'),
            'rawdb_replay_path': LaunchConfiguration('rawdb_replay_path'),
            'rawdb_replay_entry_delay_ms': ParameterValue(
                LaunchConfiguration('rawdb_replay_entry_delay_ms'), value_type=int),
            'full_snapshot_enabled': ParameterValue(
                LaunchConfiguration('full_snapshot_enabled'), value_type=bool),
            'full_snapshot_startup_delay_sec': ParameterValue(
                LaunchConfiguration('full_snapshot_startup_delay_sec'), value_type=float),
            'full_snapshot_period_sec': ParameterValue(
                LaunchConfiguration('full_snapshot_period_sec'), value_type=float),
            'debug_drop_one_delta_for_snapshot_test': ParameterValue(
                LaunchConfiguration('debug_drop_one_delta_for_snapshot_test'),
                value_type=bool),
            'debug_drop_delta_drone_id': ParameterValue(
                LaunchConfiguration('debug_drop_delta_drone_id'), value_type=int),
            'pose_store_debug_anchor_enabled': ParameterValue(
                LaunchConfiguration('pose_store_debug_anchor_enabled'), value_type=bool),
            'pose_store_debug_anchor_drone_id': ParameterValue(
                LaunchConfiguration('pose_store_debug_anchor_drone_id'), value_type=int),
            'pose_store_debug_anchor_map_epoch': ParameterValue(
                LaunchConfiguration('pose_store_debug_anchor_map_epoch'), value_type=int),
            'pose_store_debug_anchor_x': ParameterValue(
                LaunchConfiguration('pose_store_debug_anchor_x'), value_type=float),
            'pose_store_debug_anchor_y': ParameterValue(
                LaunchConfiguration('pose_store_debug_anchor_y'), value_type=float),
            'pose_store_debug_anchor_z': ParameterValue(
                LaunchConfiguration('pose_store_debug_anchor_z'), value_type=float),
            'fiducial_sim_enabled': ParameterValue(
                LaunchConfiguration('fiducial_sim_enabled'), value_type=bool),
            'fiducial_gt_max_dt_sec': ParameterValue(
                LaunchConfiguration('fiducial_gt_max_dt_sec'), value_type=float),
            'fiducial_translation_threshold_m': ParameterValue(
                LaunchConfiguration('fiducial_translation_threshold_m'), value_type=float),
            'fiducial_rotation_threshold_rad': ParameterValue(
                LaunchConfiguration('fiducial_rotation_threshold_rad'), value_type=float),
            'fiducial_yaw_threshold_rad': ParameterValue(
                LaunchConfiguration('fiducial_yaw_threshold_rad'), value_type=float),
            'pose_graph_control_vertex_ratio': ParameterValue(
                LaunchConfiguration('pose_graph_control_vertex_ratio'), value_type=float),
            'pose_graph_endpoint_neighborhood_ratio': ParameterValue(
                LaunchConfiguration('pose_graph_endpoint_neighborhood_ratio'),
                value_type=float),
            'fiducial_max_correction_fraction_per_pass': ParameterValue(
                LaunchConfiguration('fiducial_max_correction_fraction_per_pass'),
                value_type=float),
            'fiducial_max_refinement_passes': ParameterValue(
                LaunchConfiguration('fiducial_max_refinement_passes'), value_type=int),
            'body_T_camera_x': ParameterValue(
                LaunchConfiguration('body_T_camera_x'), value_type=float),
            'body_T_camera_y': ParameterValue(
                LaunchConfiguration('body_T_camera_y'), value_type=float),
            'body_T_camera_z': ParameterValue(
                LaunchConfiguration('body_T_camera_z'), value_type=float),
            'body_T_camera_roll_deg': ParameterValue(
                LaunchConfiguration('body_T_camera_roll_deg'), value_type=float),
            'body_T_camera_pitch_deg': ParameterValue(
                LaunchConfiguration('body_T_camera_pitch_deg'), value_type=float),
            'body_T_camera_yaw_deg': ParameterValue(
                LaunchConfiguration('body_T_camera_yaw_deg'), value_type=float),
            'use_camera_optical_frame_convention': ParameterValue(
                LaunchConfiguration('use_camera_optical_frame_convention'),
                value_type=bool),
        }],
    )
    return LaunchDescription(arguments + [server])
