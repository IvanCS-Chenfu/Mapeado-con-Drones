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
LAUNCH_OWNED_SERVER_PARAMETERS = {
    'drone_count',
    'drone_namespace_base',
    'debug_pipeline_flow_events',
    'debug_architecture_telemetry',
}


def parameter_map(path):
    document = yaml.safe_load(path.read_text(encoding='utf-8'))
    return document['/**']['ros__parameters']


def test_server_and_simulation_profiles_are_identical():
    for filename in CONFIG_FILES:
        assert parameter_map(SERVER_CONFIG / filename) == parameter_map(
            SIMULATION_CONFIG / filename)


def test_drone_calibration_replicas_are_identical():
    canonical = parameter_map(DRONE_CONFIG / 'calibration.yaml')
    assert parameter_map(SERVER_PACKAGE / 'config/calibration_dron.yaml') == canonical
    assert parameter_map(SIMULATION_PACKAGE / 'config/calibration_dron.yaml') == canonical


def test_simulation_actuator_replica_contains_exact_declared_keys():
    canonical = parameter_map(DRONE_CONFIG / 'actuators.yaml')
    replica = parameter_map(SIMULATION_PACKAGE / 'config/actuators_dron.yaml')
    expected = {
        'fisico.brazos.longitud',
        'fisico.brazos.grados',
        'actuadores.conversor.fuerza2torque',
    }
    assert set(replica) == expected
    assert replica == {key: canonical[key] for key in expected}


def test_each_server_parameter_has_one_yaml_owner():
    owners = {}
    for filename in CONFIG_FILES:
        for parameter in parameter_map(SERVER_CONFIG / filename):
            assert parameter not in owners, (
                f'{parameter} is defined by {owners.get(parameter)} and {filename}')
            owners[parameter] = filename
    for parameter in parameter_map(SERVER_PACKAGE / 'config/calibration_dron.yaml'):
        assert parameter not in owners
        owners[parameter] = 'calibration_dron.yaml'


def test_yaml_profiles_plus_launch_cover_server_parameter_contract():
    source = (SERVER_PACKAGE / 'src/global_map_server.cpp').read_text(encoding='utf-8')
    declared = set(re.findall(
        r'declare_parameter(?:<[^>]+>)?\s*\(\s*"([^"]+)"', source))
    configured = set()
    for filename in CONFIG_FILES:
        configured.update(parameter_map(SERVER_CONFIG / filename))
    configured.update(parameter_map(SERVER_PACKAGE / 'config/calibration_dron.yaml'))
    assert configured == declared - LAUNCH_OWNED_SERVER_PARAMETERS


def test_identity_and_clock_are_launch_authority():
    for config_dir in (SERVER_CONFIG, SIMULATION_CONFIG):
        runtime = parameter_map(config_dir / 'runtime.yaml')
        assert 'use_sim_time' not in runtime
        assert 'drone_count' not in runtime
        assert 'drone_namespace_base' not in runtime

    dron_launch = (
        DRONE_CONFIG.parent / 'launch/generar_dron.launch.py'
    ).read_text(encoding='utf-8')
    orb_launch = (
        DRONE_CONFIG.parent / 'launch/orbslam_use.launch.py'
    ).read_text(encoding='utf-8')
    server_launch = (
        SERVER_PACKAGE / 'launch/global_orb_map_server.launch.py'
    ).read_text(encoding='utf-8')
    simulation_launch = (
        SIMULATION_PACKAGE / 'launch/multi_dron.launch.py'
    ).read_text(encoding='utf-8')
    assert "DeclareLaunchArgument('use_sim_time', default_value='false')" in dron_launch
    assert "DeclareLaunchArgument('use_sim_time', default_value='false')" in orb_launch
    assert "DeclareLaunchArgument('use_sim_time', default_value='false')" in server_launch
    assert "'use_sim_time': 'true'" in simulation_launch
    assert "'use_sim_time': True" in simulation_launch


def test_drone_config_is_split_and_simulation_does_not_load_cross_group_yaml():
    for filename in (
        'physical.yaml', 'control.yaml', 'trajectory.yaml', 'actuators.yaml'
    ):
        assert (DRONE_CONFIG / filename).is_file()
    assert not (DRONE_CONFIG / 'hardware.yaml').exists()
    assert not (DRONE_CONFIG / 'tray_dron.yaml').exists()
    for filename in (
        'physical_dron.yaml', 'actuators_dron.yaml', 'simulated_sensors.yaml'
    ):
        assert (SIMULATION_PACKAGE / 'config' / filename).is_file()
    launch = (
        SIMULATION_PACKAGE / 'launch/multi_dron.launch.py'
    ).read_text(encoding='utf-8')
    assert 'hardware.yaml' not in launch
    assert "FindPackageShare('simulacion_dron'), 'config', 'physical_dron.yaml'" in launch


def test_vision_booleans_are_real_yaml_booleans():
    vision = parameter_map(DRONE_CONFIG / 'vision.yaml')
    assert isinstance(vision['orbslam.activar'], bool)
    assert isinstance(vision['orbslam.estereo'], bool)


def test_full_orbvoc_is_the_normal_runtime_profile_and_remains_configurable():
    dron_launch = (
        DRONE_CONFIG.parent / 'launch/generar_dron.launch.py'
    ).read_text(encoding='utf-8')
    simulation_launch = (
        SIMULATION_PACKAGE / 'launch/multi_dron.launch.py'
    ).read_text(encoding='utf-8')
    bootstrap = (
        SRC_ROOT / 'codex/herramientas/bootstrap_orbvoc.sh'
    ).read_text(encoding='utf-8')
    assert 'ORBvoc.txt' in dron_launch
    assert 'ORBvoc_L5.txt' not in simulation_launch
    assert "'orb_vocabulary_path', default_value=full_orb_vocabulary" in simulation_launch
    assert "'config', 'orbslam', 'vocabulary', 'ORBvoc.txt'" in simulation_launch
    assert "'orb_vocabulary_path': LaunchConfiguration('orb_vocabulary_path')" in simulation_launch
    assert 'ORBvoc.txt.tar.gz' in bootstrap


def test_normal_launches_keep_debug_replay_profile_opt_in():
    server_launch = (
        SERVER_PACKAGE / 'launch/global_orb_map_server.launch.py'
    ).read_text(encoding='utf-8')
    simulation_launch = (
        SIMULATION_PACKAGE / 'launch/multi_dron.launch.py'
    ).read_text(encoding='utf-8')
    assert "DeclareLaunchArgument('replay_debug_config', default_value='')" in server_launch
    assert 'replay_debug.yaml' not in server_launch
    assert 'replay_debug.yaml' not in simulation_launch
    assert "'config_dir': global_map_config_dir" in simulation_launch


def test_server_installs_its_configuration_profile():
    cmake = (SERVER_PACKAGE / 'CMakeLists.txt').read_text(encoding='utf-8')
    assert re.search(r'install\s*\(\s*DIRECTORY\s+config\s+launch\s+', cmake)


def test_debug_profile_controls_simulation_and_server_producers():
    profile = yaml.safe_load(
        (SIMULATION_PACKAGE / 'config/debug.yaml').read_text(encoding='utf-8'))['debug']
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
        "DeclareLaunchArgument(\n            'debug_pipeline_flow_events', "
        "default_value='false')" in server_launch
    )
    assert (
        "DeclareLaunchArgument(\n            'debug_architecture_telemetry', "
        "default_value='false')" in server_launch
    )
    assert (
        "'debug_pipeline_flow_events': "
        "LaunchConfiguration('debug_pipeline_flow_web')" in simulation_launch
    )
    assert 'architecture_telemetry_enabled = PythonExpression' in simulation_launch
