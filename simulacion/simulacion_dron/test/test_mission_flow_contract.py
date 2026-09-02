import json
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PACKAGE_ROOT.parents[1]


def load_graph():
    source = (
        PACKAGE_ROOT / 'web/mission_flow/graph_definition.js'
    ).read_text(encoding='utf-8').strip()
    prefix = 'window.FLOW_GRAPH = '
    assert source.startswith(prefix)
    return json.loads(source[len(prefix):].removesuffix(';'))


def test_graph_starts_with_four_workers_and_versioned_transport():
    graph = load_graph()
    nodes = {node['id'] for node in graph['nodes']}
    assert {'task_worker', 'voxel_worker', 'planning_worker',
            'reservation_worker'} <= nodes
    edges = {edge['id'] for edge in graph['edges']}
    assert {'manager_to_registration', 'registration_to_task_worker',
            'task_worker_geometry'} <= edges


def test_level_view_consumes_runtime_regions_without_hardcoded_roi():
    app = (PACKAGE_ROOT / 'web/mission_flow/app.js').read_text(encoding='utf-8')
    html = (PACKAGE_ROOT / 'web/mission_flow/index.html').read_text(encoding='utf-8')
    assert 'payload.regions' in app
    assert 'level-select' in html
    assert 'level-canvas' in html
    assert 'new EventSource' in app
    assert 'mapping_roi' not in app
    assert (PACKAGE_ROOT / 'web/mission_flow/vendor/cytoscape.min.js').is_file()


def test_launch_gates_phase6_and_both_observers_without_rviz_dependency():
    launch = (PACKAGE_ROOT / 'launch/multi_dron.launch.py').read_text(encoding='utf-8')
    assert "'launch_phase6', default_value='true'" in launch
    assert "'debug_mission_flow_web'" in launch
    assert "'debug_open_mission_flow_browser'" in launch
    assert "'topic': '/mission/flow_events'" in launch
    assert "package='task_server'" in launch
    assert "package='task_manager'" in launch
    assert "'launch_rviz', default_value='false'" in launch


def test_phase6_sources_do_not_consume_ground_truth_or_assign_regions():
    sources = [
        SRC_ROOT / 'servidor/task_server/src/task_server_node.cpp',
        SRC_ROOT / 'dron/task_manager/src/task_manager_node.cpp',
    ]
    combined = '\n'.join(path.read_text(encoding='utf-8') for path in sources)
    assert 'sensor/GT' not in combined
    assert 'ground_truth' not in combined.lower()
    assert 'assigned_drone_id =' not in combined
    assert 'PublishEmptyTaskState' in combined


def test_mission_interfaces_are_exact_server_drone_replicas():
    server = SRC_ROOT / 'servidor/mission_msgs'
    drone = SRC_ROOT / 'dron/mission_msgs'
    server_files = {path.relative_to(server): path for path in server.rglob('*') if path.is_file()}
    drone_files = {path.relative_to(drone): path for path in drone.rglob('*') if path.is_file()}
    assert set(server_files) == set(drone_files)
    for relative, source in server_files.items():
        assert source.read_bytes() == drone_files[relative].read_bytes()
