import importlib.util
import json
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PACKAGE_ROOT.parents[1]
POLICY = yaml.safe_load(
    (SRC_ROOT / 'codex/contexto/workspace_architecture.yaml').read_text(encoding='utf-8'))


def load_bridge_module():
    path = PACKAGE_ROOT / 'src/visualizer/system_architecture_bridge.py'
    spec = importlib.util.spec_from_file_location('system_architecture_bridge', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_graph_definition():
    path = PACKAGE_ROOT / 'web/system_architecture/graph_definition.js'
    source = path.read_text(encoding='utf-8').strip()
    prefix = 'window.SYSTEM_ARCHITECTURE = '
    assert source.startswith(prefix)
    return json.loads(source[len(prefix):].removesuffix(';'))


def load_graph_metadata():
    path = PACKAGE_ROOT / 'web/system_architecture/graph_metadata.js'
    source = path.read_text(encoding='utf-8').strip()
    prefix = 'window.SYSTEM_ARCHITECTURE_METADATA = '
    assert source.startswith(prefix)
    return json.loads(source[len(prefix):].removesuffix(';'))


def load_graph_layout():
    path = PACKAGE_ROOT / 'web/system_architecture/graph_layout.js'
    source = path.read_text(encoding='utf-8').strip()
    prefix = 'window.SYSTEM_ARCHITECTURE_LAYOUT = '
    assert source.startswith(prefix)
    return json.loads(source[len(prefix):].removesuffix(';'))


def test_static_graph_matches_policy_packages_not_hardcoded_count():
    graph = load_graph_definition()
    assert graph['groups'] == ['dron', 'servidor', 'simulacion']
    expected = {
        (group, package_name)
        for group, definition in POLICY['groups'].items()
        for package_name in definition['packages'].values()
    }
    actual = {
        (node['data']['group'], node['data']['label'])
        for node in graph['nodes'] if node['data']['kind'] == 'package'
    }
    assert actual == expected


def test_layout_places_deployments_in_readable_architecture_bands():
    graph = load_graph_definition()
    positions = load_graph_layout()['positions']
    package_ids = {
        node['data']['id'] for node in graph['nodes']
        if node['data']['kind'] == 'package'}
    assert set(positions) == package_ids

    assert positions['simulacion_dron']['x'] < positions['orbslam3_server']['x']
    assert positions['simulacion_dron']['y'] < positions['dron_individual']['y']
    assert positions['orbslam3_server']['y'] < positions['orbslam3']['y']
    assert positions['orbslam3_multi']['y'] < positions['orbslam3_server']['y']
    assert positions['orbslam3_msgs_server']['x'] > positions['orbslam3_server']['x']

    assert positions['dron_individual']['x'] < positions['orbslam3']['x']
    assert positions['orbslam3']['x'] < positions['orbslam3_msgs_dron']['x']
    assert positions['lib_tray']['y'] > positions['dron_individual']['y']
    assert positions['ORB_SLAM3']['y'] > positions['orbslam3']['y']


def test_edges_use_four_semantic_layers_and_only_runtime_can_pulse():
    graph = load_graph_definition()
    assert graph['layers'] == ['runtime', 'build', 'config', 'deployment']
    expected_runtime = set(POLICY['system_architecture']['runtime_edges'])
    actual_runtime = set()
    for edge in graph['edges']:
        data = edge['data']
        assert data['interface']
        assert data['layer'] in {'runtime', 'build', 'config', 'deployment'}
        if data['layer'] == 'runtime':
            actual_runtime.add(data['id'])
            assert data['activity_mode'] == 'direct'
        else:
            assert data['activity_mode'] == 'none'
    assert actual_runtime == expected_runtime
    assert 'orbslam3_to_dron' not in {edge['data']['id'] for edge in graph['edges']}


def test_correct_current_runtime_topology_is_explicit():
    graph = load_graph_definition()
    edges = {edge['data']['id']: edge['data'] for edge in graph['edges']}
    assert edges['sim_to_orbslam_stereo']['target'] == 'orbslam3'
    assert edges['sim_to_dron_gt']['status'] == 'provisional_phase5'
    assert edges['dron_to_sim_motors']['interface'].endswith('motor/{arr_iz,ab_iz,ab_der,arr_der}')
    assert edges['server_to_orbslam_snapshot_request']['source'] == 'orbslam3_server'
    assert edges['orbslam_to_server_snapshot_response']['source'] == 'orbslam3'
    assert edges['fiducial_config_server_to_wrapper']['source'] == (
        'orbslam3_server')
    assert edges['fiducial_config_server_to_wrapper']['target'] == 'orbslam3'
    assert edges['wrapper_to_server_fiducial_observations']['source'] == 'orbslam3'
    assert edges['wrapper_to_server_fiducial_observations']['target'] == (
        'orbslam3_server')
    assert edges['sim_to_dron_action']['interface'].endswith('AccionTrayectoria')
    assert 'fiducial_objects.yaml' in edges['globalmap_profile_to_sim']['interface']


def test_packages_and_runtime_edges_have_operational_metadata():
    graph = load_graph_definition()
    metadata = load_graph_metadata()
    package_ids = {
        node['data']['id'] for node in graph['nodes']
        if node['data']['kind'] == 'package'}
    runtime_ids = {
        edge['data']['id'] for edge in graph['edges']
        if edge['data']['layer'] == 'runtime'}
    assert set(metadata['nodes']) == package_ids
    assert set(metadata['edges']) == runtime_ids
    for values in metadata['nodes'].values():
        assert {'path', 'ros_name', 'executables', 'owned_yaml', 'dependencies',
                'cross_group', 'status', 'docs'} <= set(values)
    for values in metadata['edges'].values():
        assert {'message_type', 'namespace', 'qos', 'data_transferred'} <= set(values)
    assert 'fiducial_spawner.py' in metadata['nodes']['simulacion_dron']['executables']
    assert 'config/fiducial_objects.yaml' in metadata['nodes']['orbslam3_server']['owned_yaml']


def test_frontend_checks_health_and_pulses_only_runtime_direct_edges():
    web_root = PACKAGE_ROOT / 'web/system_architecture'
    html = (web_root / 'index.html').read_text(encoding='utf-8')
    app = (web_root / 'app.js').read_text(encoding='utf-8')
    assert 'http://' not in html
    assert 'https://' not in html
    assert 'vendor/cytoscape.min.js' in html
    assert (web_root / 'vendor/cytoscape.min.js').is_file()
    assert 'graph_layout.js' in html
    assert 'SYSTEM_ARCHITECTURE_LAYOUT' in app
    assert "fetch('/health'" in app
    assert "new EventSource('/events')" in app
    assert "edge.data('layer') !== 'runtime'" in app
    assert "edge.data('activity_mode') !== 'direct'" in app


def test_bridge_has_one_lightweight_channel_and_unknown_events_are_ignored():
    bridge_path = PACKAGE_ROOT / 'src/visualizer/system_architecture_bridge.py'
    source = bridge_path.read_text(encoding='utf-8')
    assert '/system_architecture/activity' in source
    assert '/global_mapping/flow_events' not in source
    assert 'sensor_msgs.msg import Image' not in source
    assert 'PointCloud2' not in source
    assert 'orbslam3_msgs' not in source
    assert 'if edge_id not in RUNTIME_EDGES' in source
    assert "required = ('source', 'interface', 'interface_kind', 'timestamp')" in source
    assert 'deque(maxlen=capacity)' in source
    store = load_bridge_module().EventStore(capacity=2)
    store.append('one')
    store.append('two')
    store.append('three')
    cursor, reset_required = store.start_cursor('0')
    assert cursor == 3
    assert reset_required is True


def test_runtime_evidence_is_emitted_by_real_producers_or_consumers():
    server = (
        SRC_ROOT / 'servidor/orbslam3_server/src/global_map_server.cpp'
    ).read_text(encoding='utf-8')
    wrapper = (
        SRC_ROOT / 'dron/orbslam3_ros2/src/stereo/stereo-slam-node.cpp'
    ).read_text(encoding='utf-8')
    gen_tray = (
        SRC_ROOT / 'dron/dron_individual/src/control_tray/gen_tray.cpp'
    ).read_text(encoding='utf-8')
    motors = (
        SRC_ROOT / 'dron/dron_individual/src/control_tray/aplicar_fuerzas_dron.cpp'
    ).read_text(encoding='utf-8')
    for edge_id in (
        'orbslam_to_server_delta',
        'server_to_orbslam_snapshot_request',
        'orbslam_to_server_snapshot_response',
        'server_to_sim_backpressure',
        'server_to_sim_sparse_map',
        'wrapper_to_server_fiducial_observations',
    ):
        assert edge_id in server
    assert 'sim_to_orbslam_stereo' in wrapper
    config_server = (
        SRC_ROOT /
        'servidor/orbslam3_server/scripts/fiducial_config_server.py'
    ).read_text(encoding='utf-8')
    assert 'fiducial_config_server_to_wrapper' in wrapper
    assert 'fiducial_config_server_to_wrapper' in config_server
    assert 'wrapper_to_server_fiducial_observations' in wrapper
    assert 'wrapper_to_server_fiducial_observations' in server
    assert 'sim_to_dron_gt' in gen_tray
    assert 'sim_to_dron_action' in gen_tray
    assert 'dron_to_sim_motors' in motors


def test_master_web_flag_gates_architecture_producers():
    launch = (PACKAGE_ROOT / 'launch/multi_dron.launch.py').read_text(encoding='utf-8')
    for flag in (
        'debug_system_architecture_web',
        'debug_open_system_architecture_browser',
        'debug_architecture_telemetry',
    ):
        assert flag in launch
    assert 'architecture_telemetry_enabled = PythonExpression' in launch
    assert "'.lower() == 'true' and '" in launch
    assert "executable='system_architecture_bridge.py'" in launch
    assert "executable='system_architecture_browser.py'" in launch
