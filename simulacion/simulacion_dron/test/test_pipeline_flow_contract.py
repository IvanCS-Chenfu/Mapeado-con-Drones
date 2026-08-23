import importlib.util
import json
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PACKAGE_ROOT.parents[1]


def load_bridge_module():
    path = PACKAGE_ROOT / 'src/visualizer/pipeline_flow_bridge.py'
    spec = importlib.util.spec_from_file_location('pipeline_flow_bridge', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_browser_module():
    path = PACKAGE_ROOT / 'src/visualizer/pipeline_flow_browser.py'
    spec = importlib.util.spec_from_file_location(
        'pipeline_flow_browser', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_graph_definition():
    path = PACKAGE_ROOT / 'web/pipeline_flow/graph_definition.js'
    source = path.read_text(encoding='utf-8').strip()
    prefix = 'window.FLOW_GRAPH = '
    assert source.startswith(prefix)
    return json.loads(source[len(prefix):].removesuffix(';'))


def test_first_sse_connection_starts_live_without_replay():
    store = load_bridge_module().EventStore(capacity=4)
    store.append('{"seq": 1}')
    store.append('{"seq": 2}')

    cursor, reset_required = store.start_cursor(None)

    assert cursor == 2
    assert reset_required is False


def test_reconnect_recovers_available_events_and_resets_expired_cursor():
    store = load_bridge_module().EventStore(capacity=2)
    store.append('one')
    store.append('two')
    store.append('three')

    cursor, reset_required = store.start_cursor('1')
    assert cursor == 1
    assert reset_required is False
    assert store.wait_after(cursor, timeout=0.0) == [(2, 'two'), (3, 'three')]

    cursor, reset_required = store.start_cursor('0')
    assert cursor == 3
    assert reset_required is True


def test_phase_3q_graph_matches_primary_fiducial_loop_and_fusion_flow():
    graph = load_graph_definition()

    assert graph['phase'] == '3Q'
    assert [node['id'] for node in graph['nodes']] == [
        'wrappers',
        'server',
        'primary_queue',
        'primary_worker',
        'raw_db',
        'landmark_score_manager',
        'fiducial_anchor_manager',
        'global_pose_store',
        'global_map_builder',
        'secondary_queue',
        'secondary_worker',
        'covisibility_database',
        'loop_detector',
        'loop_bow_index',
        'subcloud_loop_verifier',
        'loop_decision',
        'fused_landmark_manager',
        'loop_anchor_store',
        'pose_graph_builder',
        'optimization_manager',
        'validation',
        'rviz',
        'mission_gate',
    ]
    assert [edge['id'] for edge in graph['edges']] == [
        'wrapper_server',
        'wrapper_server_snapshot',
        'server_primary_queue',
        'primary_queue_worker',
        'primary_worker_raw_db',
        'primary_worker_raw_db_snapshot',
        'raw_db_global_pose_store',
        'raw_db_global_pose_store_snapshot',
        'raw_db_landmark_score_manager',
        'server_fiducial_anchor_manager',
        'fiducial_anchor_manager_global_pose_store',
        'fiducial_anchor_manager_secondary_queue',
        'secondary_queue_secondary_worker',
        'secondary_worker_secondary_queue_retry',
        'secondary_worker_pose_graph_builder',
        'pose_graph_builder_optimization_manager',
        'optimization_manager_validation',
        'secondary_worker_validation',
        'validation_global_pose_store',
        'raw_db_secondary_queue_database',
        'secondary_worker_covisibility_database',
        'covisibility_database_secondary_queue',
        'secondary_worker_loop_detector',
        'loop_detector_loop_bow_index',
        'loop_bow_index_subcloud_loop_verifier',
        'subcloud_loop_verifier_loop_decision',
        'loop_decision_pose_graph_builder',
        'loop_decision_fused_landmark_manager',
        'validation_fused_landmark_manager',
        'fused_landmark_manager_covisibility_database',
        'fused_landmark_manager_landmark_score_manager',
        'fused_landmark_manager_global_map_builder',
        'loop_decision_loop_anchor_store',
        'loop_anchor_store_global_pose_store',
        'raw_db_global_map_builder',
        'landmark_score_manager_global_map_builder',
        'global_pose_store_global_map_builder',
        'global_map_builder_server',
        'server_rviz_cloud',
        'server_rviz_keyframes',
        'server_mission_gate',
    ]


def test_active_web_copy_uses_the_current_phase_dynamically():
    web_root = Path(__file__).parents[1] / 'web' / 'pipeline_flow'
    assert 'graph.phase' in (web_root / 'app.js').read_text(encoding='utf-8')


def test_frontend_uses_live_frame_drain_without_fixed_replay_queue():
    app = (
        PACKAGE_ROOT / 'web/pipeline_flow/app.js'
    ).read_text(encoding='utf-8')

    assert 'requestAnimationFrame' in app
    assert "state_reset" in app
    assert 'setInterval' not in app
    assert '110' not in app
    assert 'eventQueue' not in app


def test_secondary_task_path_is_latched_until_lifecycle_done():
    app = (
        PACKAGE_ROOT / 'web/pipeline_flow/app.js'
    ).read_text(encoding='utf-8')
    server = (
        SRC_ROOT / 'servidor/orbslam3_server/src/global_map_server.cpp'
    ).read_text(encoding='utf-8')

    assert 'secondaryTasks' in app
    assert 'latchSecondaryEdge' in app
    assert "event.kind === 'secondary_task_lifecycle'" in app
    assert 'SECONDARY_DONE_HOLD_MS' in app
    assert "event.flow_id.startsWith('secondary:')" in app
    assert 'secondary_task_lifecycle' in server
    assert 'EmitSecondaryLifecycleEvent(' in server
    assert '"start", queued' in server
    assert '"done", queued' in server


def test_bridge_advertises_the_current_3q_topology():
    bridge = (
        PACKAGE_ROOT / 'src/visualizer/pipeline_flow_bridge.py'
    ).read_text(encoding='utf-8')

    assert '[F3Q-FLOW-WEB-READY]' in bridge
    assert 'topology=23_nodes_41_edges' in bridge
    assert 'topology=2_nodes_0_edges' not in bridge


def test_static_assets_are_not_reused_from_an_old_phase():
    bridge = (
        PACKAGE_ROOT / 'src/visualizer/pipeline_flow_bridge.py'
    ).read_text(encoding='utf-8')
    index = (
        PACKAGE_ROOT / 'web/pipeline_flow/index.html'
    ).read_text(encoding='utf-8')

    assert "'Cache-Control', 'no-store, max-age=0'" in bridge
    assert 'pipelineFlowLoadError' in index
    assert 'app.js?fresh=' in index


def test_browser_opens_only_after_health_and_uses_a_fresh_url():
    browser = load_browser_module()
    source = (
        PACKAGE_ROOT / 'src/visualizer/pipeline_flow_browser.py'
    ).read_text(encoding='utf-8')
    multi_launch = (
        PACKAGE_ROOT / 'launch/multi_dron.launch.py'
    ).read_text(encoding='utf-8')

    assert '/health' in source
    assert 'wait_until_ready' in source
    assert browser.browser_url(8765).startswith(
        'http://127.0.0.1:8765/?fresh=')
    assert "executable='pipeline_flow_browser.py'" in multi_launch
    assert "'-m',\n                'webbrowser'" not in multi_launch
    assert '[F3B-FLOW-BROWSER-OPEN]' in source
    assert '[F3B-FLOW-BROWSER-ERROR]' in source
