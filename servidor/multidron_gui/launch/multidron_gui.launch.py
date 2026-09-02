from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    drone_count = LaunchConfiguration('drone_count')
    namespace_base = LaunchConfiguration('drone_namespace_base')
    use_sim_time = LaunchConfiguration('use_sim_time')
    sparse_topic = LaunchConfiguration('sparse_topic')
    keyframes_topic = LaunchConfiguration('keyframes_topic')
    fiducial_config_path = LaunchConfiguration('fiducial_config_path')
    stale_timeout = LaunchConfiguration('drone_stale_timeout_sec')

    default_fiducials = PathJoinSubstitution([
        FindPackageShare('orbslam3_server'),
        'config',
        'fiducial_objects.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('drone_count', default_value='2'),
        DeclareLaunchArgument('drone_namespace_base', default_value='dron'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('sparse_topic', default_value='/global_sparse_cloud'),
        DeclareLaunchArgument('keyframes_topic', default_value='/global_keyframes'),
        DeclareLaunchArgument(
            'fiducial_config_path',
            default_value=default_fiducials,
            description='YAML canónico de objetos fiduciales de Fase 4.'),
        DeclareLaunchArgument('drone_stale_timeout_sec', default_value='1.0'),
        Node(
            package='multidron_gui',
            executable='multidron_gui',
            name='multidron_gui',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'drone_count': drone_count,
                'drone_namespace_base': namespace_base,
                'sparse_topic': sparse_topic,
                'keyframes_topic': keyframes_topic,
                'fiducial_config_path': fiducial_config_path,
                'navigation_topic_suffix': 'orbslam/navigation_state',
                'drone_stale_timeout_sec': stale_timeout,
            }],
        ),
    ])
