import re
from pathlib import Path

import yaml


SRC_ROOT = Path(__file__).resolve().parents[3]
DRONE_CONFIG = SRC_ROOT / 'dron/dron_individual/config'
SERVER_PACKAGE = SRC_ROOT / 'servidor/orbslam3_server'
SIMULATION_PACKAGE = SRC_ROOT / 'simulacion/simulacion_dron'
SERVER_CONFIG = SERVER_PACKAGE / 'config/global_map'
SIMULATION_CONFIG = SIMULATION_PACKAGE / 'config/global_map'
CONFIG_FILES = (
    'runtime.yaml',
    'fiducials.yaml',
    'optimization.yaml',
    'loop_fusion.yaml',
    'scoring.yaml',
    'replay_debug.yaml',
)


def parameter_map(path):
    document = yaml.safe_load(path.read_text(encoding='utf-8'))
    return document['/**']['ros__parameters']


def test_server_and_simulation_profiles_are_identical():
    for filename in CONFIG_FILES:
        assert parameter_map(SERVER_CONFIG / filename) == parameter_map(
            SIMULATION_CONFIG / filename)


def test_drone_calibration_replicas_are_identical():
    canonical = parameter_map(DRONE_CONFIG / 'calibration.yaml')
    assert parameter_map(
        SERVER_PACKAGE / 'config/calibration_dron.yaml') == canonical
    assert parameter_map(
        SIMULATION_PACKAGE / 'config/calibration_dron.yaml') == canonical


def test_each_parameter_has_one_yaml_owner():
    owners = {}
    for filename in CONFIG_FILES:
        for parameter in parameter_map(SERVER_CONFIG / filename):
            assert parameter not in owners, (
                f'{parameter} is defined by {owners.get(parameter)} and '
                f'{filename}')
            owners[parameter] = filename
    for parameter in parameter_map(
            SERVER_PACKAGE / 'config/calibration_dron.yaml'):
        assert parameter not in owners
        owners[parameter] = 'calibration_dron.yaml'


def test_yaml_profiles_cover_the_server_parameter_contract():
    source = (
        SERVER_PACKAGE / 'src/global_map_server.cpp'
    ).read_text(encoding='utf-8')
    declared = set(re.findall(
        r'declare_parameter(?:<[^>]+>)?\s*\(\s*"([^"]+)"', source))
    configured = set()
    for filename in CONFIG_FILES:
        configured.update(parameter_map(SERVER_CONFIG / filename))
    configured.update(parameter_map(
        SERVER_PACKAGE / 'config/calibration_dron.yaml'))

    # use_sim_time is provided by rclcpp rather than declared by this node.
    assert configured - {'use_sim_time'} == declared


def test_normal_launches_keep_debug_replay_profile_opt_in():
    server_launch = (
        SERVER_PACKAGE / 'launch/global_orb_map_server.launch.py'
    ).read_text(encoding='utf-8')
    simulation_launch = (
        SIMULATION_PACKAGE / 'launch/multi_dron.launch.py'
    ).read_text(encoding='utf-8')

    assert (
        "DeclareLaunchArgument('replay_debug_config', default_value='')"
        in server_launch)
    assert 'replay_debug.yaml' not in server_launch
    assert 'replay_debug.yaml' not in simulation_launch
    assert "'config_dir': global_map_config_dir" in simulation_launch


def test_server_installs_its_configuration_profile():
    cmake = (
        SERVER_PACKAGE / 'CMakeLists.txt'
    ).read_text(encoding='utf-8')
    assert re.search(
        r'install\s*\(\s*DIRECTORY\s+config\s+launch\s+', cmake)


def test_debug_profile_controls_the_simulation_launch():
    profile = yaml.safe_load(
        (SIMULATION_PACKAGE / 'config/debug.yaml').read_text(
            encoding='utf-8'))['debug']
    expected = {
        'debug_sparse_global_rviz',
        'debug_pipeline_flow_web',
        'debug_open_pipeline_flow_browser',
        'debug_fase3_logs_terminal',
        'debug_system_architecture_web',
        'debug_open_system_architecture_browser',
        'debug_architecture_telemetry',
    }
    assert set(profile) == expected
    assert all(value is False for value in profile.values())

    simulation_launch = (
        SIMULATION_PACKAGE / 'launch/multi_dron.launch.py'
    ).read_text(encoding='utf-8')
    server_launch = (
        SERVER_PACKAGE / 'launch/global_orb_map_server.launch.py'
    ).read_text(encoding='utf-8')
    for parameter in expected:
        assert parameter in simulation_launch
    assert (
        "DeclareLaunchArgument('log_level', default_value='info')"
        in server_launch)
    assert "'--ros-args', '--log-level', log_level" in server_launch
