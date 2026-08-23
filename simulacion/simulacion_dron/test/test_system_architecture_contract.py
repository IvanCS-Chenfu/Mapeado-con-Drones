import importlib.util
import json
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def load_bridge_module():
    path = PACKAGE_ROOT / 'src/visualizer/system_architecture_bridge.py'
    spec = importlib.util.spec_from_file_location(
        'system_architecture_bridge', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_graph_definition():
    path = PACKAGE_ROOT / 'web/system_architecture/graph_definition.js'
    source = path.read_text(encoding='utf-8').strip()
    prefix = 'window.SYSTEM_ARCHITECTURE = '
    assert source.startswith(prefix)
    return json.loads(source[len(prefix):].removesuffix(';'))


def test_static_graph_contains_three_groups_and_nine_packages():
    graph = load_graph_definition()
    assert graph['groups'] == ['dron', 'servidor', 'simulacion']
    groups = [node for node in graph['nodes']
              if node['data']['kind'] == 'group']
    packages = [node for node in graph['nodes']
                if node['data']['kind'] == 'package']
    assert len(groups) == 3
    assert len(packages) == 9
    assert {node['data']['parent'] for node in packages} == {
        'group_dron', 'group_servidor', 'group_simulacion'}


def test_edges_have_real_interfaces_and_known_layers():
    graph = load_graph_definition()
    layers = {'sensor', 'control', 'map', 'observability', 'dependency'}
    assert graph['edges']
    for edge in graph['edges']:
        assert edge['data']['interface']
        assert edge['data']['layer'] in layers


def test_frontend_is_local_and_supports_live_activity():
    web_root = PACKAGE_ROOT / 'web/system_architecture'
    html = (web_root / 'index.html').read_text(encoding='utf-8')
    app = (web_root / 'app.js').read_text(encoding='utf-8')
    assert 'http://' not in html
    assert 'https://' not in html
    assert 'vendor/cytoscape.min.js' in html
    assert (web_root / 'vendor/cytoscape.min.js').is_file()
    assert "new EventSource('/events')" in app
    assert 'architecture_activity' in app


def test_bridge_uses_bounded_events_without_heavy_subscriptions():
    bridge_path = (
        PACKAGE_ROOT / 'src/visualizer/system_architecture_bridge.py')
    source = bridge_path.read_text(encoding='utf-8')
    assert 'deque(maxlen=capacity)' in source
    assert 'sensor_msgs.msg import Image' not in source
    assert 'PointCloud2' not in source
    store = load_bridge_module().EventStore(capacity=2)
    store.append('one')
    store.append('two')
    store.append('three')
    cursor, reset_required = store.start_cursor('0')
    assert cursor == 3
    assert reset_required is True


def test_launch_keeps_web_browser_and_telemetry_independent():
    launch = (
        PACKAGE_ROOT / 'launch/multi_dron.launch.py'
    ).read_text(encoding='utf-8')
    for flag in (
        'debug_system_architecture_web',
        'debug_open_system_architecture_browser',
        'debug_architecture_telemetry',
    ):
        assert flag in launch
    assert "executable='system_architecture_bridge.py'" in launch
    assert "executable='system_architecture_browser.py'" in launch
