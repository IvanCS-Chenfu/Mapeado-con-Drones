import re
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PACKAGE_ROOT.parents[1]


def graph_source():
    source = (PACKAGE_ROOT / 'web/mission_flow/graph_definition.js').read_text(
        encoding='utf-8').strip()
    assert source.startswith('window.FLOW_GRAPH = ')
    return source


def test_graph_starts_with_four_workers_and_versioned_transport():
    source = graph_source()
    nodes = set(re.findall(r'\{id: "([^"]+)", label:', source))
    assert {'task_worker', 'voxel_worker', 'planning_worker',
            'reservation_worker'} <= nodes
    edges = set(re.findall(r'\{id: "([^"]+)", source:', source))
    assert {'manager_to_registration', 'registration_to_task_worker',
            'task_worker_geometry'} <= edges


def test_flow_keeps_events_without_a_duplicate_subroi_view():
    app = (PACKAGE_ROOT / 'web/mission_flow/app.js').read_text(encoding='utf-8')
    html = (PACKAGE_ROOT / 'web/mission_flow/index.html').read_text(encoding='utf-8')
    assert 'payload.regions' not in app
    assert 'level-select' not in html
    assert 'level-canvas' not in html
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


def test_phase6_sources_gate_assignment_by_explicit_pose_authority():
    sources = [
        SRC_ROOT / 'servidor/task_server/src/task_server_node.cpp',
        SRC_ROOT / 'dron/task_manager/src/task_manager_node.cpp',
    ]
    combined = '\n'.join(path.read_text(encoding='utf-8') for path in sources)
    assert 'sensor/GT' not in combined
    assert 'ground_truth' not in combined.lower()
    assert "phase5_navigation_source" in combined
    assert 'assigned_drone_id =' in combined
    assert 'PublishTaskStates' in combined
    assert 'POSE_SOURCE_GT_FORCED' in combined
    assert 'GLOBAL_STATUS_AUTHORITATIVE' in combined
    assert '/mission/register_drone' in combined
    assert '/mission/geometry' in combined


def test_mission_interfaces_are_exact_server_drone_replicas():
    server = SRC_ROOT / 'servidor/mission_msgs'
    drone = SRC_ROOT / 'dron/mission_msgs'
    server_files = {path.relative_to(server): path for path in server.rglob('*') if path.is_file()}
    drone_files = {path.relative_to(drone): path for path in drone.rglob('*') if path.is_file()}
    assert set(server_files) == set(drone_files)
    for relative, source in server_files.items():
        assert source.read_bytes() == drone_files[relative].read_bytes()
