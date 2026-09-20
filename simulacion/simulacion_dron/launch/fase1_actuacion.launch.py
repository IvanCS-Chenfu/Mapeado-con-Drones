import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, TimerAction
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    simulation_share = get_package_share_directory("simulacion_dron")
    empty_world = os.path.join(simulation_share, "worlds", "empty.world")
    physical_config = os.path.join(simulation_share, "config", "physical_dron.yaml")
    actuators_config = os.path.join(simulation_share, "config", "actuators_dron.yaml")
    sensors_config = os.path.join(simulation_share, "config", "simulated_sensors.yaml")
    simulation_config = os.path.join(simulation_share, "config", "sim_dron.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("mission_profile", default_value=""),
        DeclareLaunchArgument("actuation_mode", default_value="force"),
        DeclareLaunchArgument("force_value", default_value="14.0"),
        DeclareLaunchArgument("torque_x_nm", default_value="0.01"),
        DeclareLaunchArgument("torque_z_nm", default_value="0.01"),
        DeclareLaunchArgument("idle_duration_sec", default_value="10.0"),
        DeclareLaunchArgument("actuation_duration_sec", default_value="20.0"),
        ExecuteProcess(
            cmd=["gzserver", "--verbose", empty_world, "-s", "libgazebo_ros_factory.so"],
            output="screen",
        ),
        ExecuteProcess(
            cmd=["gzclient"],
            output="screen",
        ),
        Node(
            package="simulacion_dron",
            executable="clock",
            name="clock",
            parameters=[{"use_sim_time": False}],
            output="screen",
        ),
        Node(
            package="simulacion_dron",
            executable="generador_URDF",
            name="generador_URDF",
            namespace="dron_1",
            parameters=[
                physical_config,
                actuators_config,
                sensors_config,
                simulation_config,
                {
                    "use_sim_time": True,
                    "dron.numero": 1,
                    "dron.spawn_override_enabled": True,
                    "dron.spawn_x": 0.0,
                    "dron.spawn_y": 0.0,
                    "dron.spawn_yaw_deg": 0.0,
                    "fisico.camera_pitch.enabled": False,
                    "debug_fase_1": False,
                },
            ],
            output="screen",
        ),
        Node(
            package="dron_individual",
            executable="aplicar_fuerzas_dron",
            name="aplicar_fuerzas_dron",
            namespace="dron_1",
            parameters=[actuators_config, {"use_sim_time": True, "drone_id": 1}],
            output="screen",
        ),
        Node(
            package="simulacion_dron",
            executable="fase1_actuacion_publisher.py",
            name="fase1_actuacion_publisher",
            parameters=[{
                "actuation_mode": LaunchConfiguration("actuation_mode"),
                "force_value": ParameterValue(
                    LaunchConfiguration("force_value"), value_type=float),
                "torque_x_nm": ParameterValue(
                    LaunchConfiguration("torque_x_nm"), value_type=float),
                "torque_z_nm": ParameterValue(
                    LaunchConfiguration("torque_z_nm"), value_type=float),
                "idle_duration_sec": ParameterValue(
                    LaunchConfiguration("idle_duration_sec"), value_type=float),
                "actuation_duration_sec": ParameterValue(
                    LaunchConfiguration("actuation_duration_sec"), value_type=float),
            }],
            output="screen",
        ),
        TimerAction(
            period=PythonExpression([
                "float('", LaunchConfiguration("idle_duration_sec"),
                "') + float('", LaunchConfiguration("actuation_duration_sec"),
                "') + 1.0"
            ]),
            actions=[EmitEvent(event=Shutdown(reason="F1 4.4A actuation window completed"))],
        ),
    ])
