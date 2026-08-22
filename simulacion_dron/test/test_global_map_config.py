import re
from pathlib import Path

import yaml


SRC_ROOT = Path(__file__).resolve().parents[2]
SERVER_CONFIG = SRC_ROOT / 'orbslam3_server/config/global_map'
SIMULATION_CONFIG = SRC_ROOT / 'simulacion_dron/config/global_map'
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


def test_each_parameter_has_one_yaml_owner():
    owners = {}
    for filename in CONFIG_FILES:
        for parameter in parameter_map(SERVER_CONFIG / filename):
            assert parameter not in owners, (
                f'{parameter} is defined by {owners.get(parameter)} and '
                f'{filename}')
            owners[parameter] = filename


def test_yaml_profiles_cover_the_server_parameter_contract():
    source = (
        SRC_ROOT / 'orbslam3_server/src/global_map_server.cpp'
    ).read_text(encoding='utf-8')
    declared = set(re.findall(
        r'declare_parameter(?:<[^>]+>)?\s*\(\s*"([^"]+)"', source))
    configured = set()
    for filename in CONFIG_FILES:
        configured.update(parameter_map(SERVER_CONFIG / filename))

    # use_sim_time is provided by rclcpp rather than declared by this node.
    assert configured - {'use_sim_time'} == declared


def test_normal_launches_keep_debug_replay_profile_opt_in():
    server_launch = (
        SRC_ROOT / 'orbslam3_server/launch/global_orb_map_server.launch.py'
    ).read_text(encoding='utf-8')
    simulation_launch = (
        SRC_ROOT / 'simulacion_dron/launch/multi_dron.launch.py'
    ).read_text(encoding='utf-8')

    assert (
        "DeclareLaunchArgument('replay_debug_config', default_value='')"
        in server_launch)
    assert 'replay_debug.yaml' not in server_launch
    assert 'replay_debug.yaml' not in simulation_launch
    assert "'config_dir': global_map_config_dir" in simulation_launch


def test_server_installs_its_configuration_profile():
    cmake = (
        SRC_ROOT / 'orbslam3_server/CMakeLists.txt'
    ).read_text(encoding='utf-8')
    assert re.search(
        r'install\s*\(\s*DIRECTORY\s+config\s+launch\s+', cmake)


def test_fase3_debug_profile_controls_the_simulation_launch():
    profile = yaml.safe_load(
        (SRC_ROOT / 'simulacion_dron/config/fase3_debug.yaml').read_text(
            encoding='utf-8'))['fase3_debug']
    expected = {
        'fase3_rviz2',
        'fase3_grafo_web',
        'fase3_abrir_navegador_web',
        'fase3_logs_terminal',
    }
    assert set(profile) == expected
    assert all(value is False for value in profile.values())

    simulation_launch = (
        SRC_ROOT / 'simulacion_dron/launch/multi_dron.launch.py'
    ).read_text(encoding='utf-8')
    server_launch = (
        SRC_ROOT / 'orbslam3_server/launch/global_orb_map_server.launch.py'
    ).read_text(encoding='utf-8')
    for parameter in expected:
        assert parameter in simulation_launch
    assert (
        "DeclareLaunchArgument('log_level', default_value='info')"
        in server_launch)
    assert "'--ros-args', '--log-level', log_level" in server_launch
