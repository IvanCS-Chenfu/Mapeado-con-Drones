import re
from pathlib import Path

import yaml


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
    assert "'mission_profile', default_value=mission_profile_default_path" in launch
    assert "mission_mode" in launch
    assert "navigation_source" in launch
    assert "phase6_enabled = PythonExpression" in launch
    assert "'.lower() == 'autonomous' and '" in launch
    assert "'depth_observation_enabled': phase6_enabled" in launch
    assert "'depth_stop_enabled': phase6_enabled" in launch
    assert "'phase6_enabled': phase6_enabled" in launch
    assert "'score_drone_body_mask_enabled': LaunchConfiguration(" in launch
    assert "'raw_stats_telemetry_enabled': LaunchConfiguration(" in launch
    assert "'full_snapshot_enabled': LaunchConfiguration(" in launch
    assert "'orb_loss_protocol_enabled': LaunchConfiguration(" in launch
    assert "'debug_mission_flow_web'" in launch
    assert "'debug_open_mission_flow_browser'" in launch
    assert "'topic': '/mission/flow_events'" in launch
    assert "package='task_server'" in launch
    assert "package='task_manager'" in launch
    assert "'launch_rviz', default_value='false'" in launch


def test_autonomous_handoff_is_automatic_and_single_drone_profile_exists():
    runner = (PACKAGE_ROOT / 'src/control_tray/scenario_runner_node.cpp').read_text(
        encoding='utf-8')
    assert 'mission_profile' in runner
    assert 'trajectory_file' in runner
    assert 'mission_navigation_source_' in runner
    assert 'scenario_file_ = resolved_trajectory.string()' in runner
    assert 'EnableAutonomousExecution()' in runner
    assert '/mission/set_coverage_execution_enabled' in runner
    assert '[SCENARIO-RUNNER-AUTONOMOUS-HANDOFF]' in runner

    mission = yaml.safe_load(
        (SRC_ROOT / 'servidor/task_server/config/mission_house_single_drone.yaml').read_text(
            encoding='utf-8'))
    assert mission['drones'] == [1]

    scenario = yaml.safe_load(
        (PACKAGE_ROOT / 'config/scenarios/autonomous_gt_fiducial2.yaml').read_text(
            encoding='utf-8'))
    goals = scenario['steps'][-1]['goals']
    assert len(goals) == 1
    assert goals[0]['drone_id'] == 1
    assert goals[0]['navigation_source'] == 'GT'

    profile = yaml.safe_load(
        (PACKAGE_ROOT / 'config/mission_profiles/autonomous_gt.yaml').read_text(
            encoding='utf-8'))
    assert profile['trajectory_file'] == '../scenarios/autonomous_gt_fiducial2.yaml'


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


def test_wall_trajectory_profile_selects_five_gt_waypoints_with_fixed_yaw():
    profile = yaml.safe_load(
        (PACKAGE_ROOT / 'config/mission_profiles/trajectory_gt_wall.yaml').read_text(
            encoding='utf-8'))
    assert profile == {
        'mission_mode': 'trajectory',
        'navigation_source': 'gt',
        'trajectory_file': '../scenarios/trajectory_gt_wall.yaml'}

    scenario = yaml.safe_load(
        (PACKAGE_ROOT / 'config/scenarios/trajectory_gt_wall.yaml').read_text(
            encoding='utf-8'))
    goals = [step['goals'][0] for step in scenario['steps'] if step['type'] == 'move']
    assert [goal['target'] for goal in goals] == [
        [-10.0, -10.0, 1.0],
        [0.0, -10.0, 1.0],
        [10.0, -10.0, 1.0],
        [10.0, -10.0, 3.0],
        [-10.0, -10.0, 3.0],
    ]
    assert all(goal['drone_id'] == 1 for goal in goals)
    assert all(goal['navigation_source'] == 'GT' for goal in goals)
    assert all(goal['yaw_deg'] == 90.0 for goal in goals)


def test_high_wall_trajectory_extends_to_five_meters_and_returns_southeast():
    profile = yaml.safe_load(
        (PACKAGE_ROOT / 'config/mission_profiles/trajectory_gt_wall_high.yaml').read_text(
            encoding='utf-8'))
    assert profile == {
        'mission_mode': 'trajectory',
        'navigation_source': 'gt',
        'trajectory_file': '../scenarios/trajectory_gt_wall_high.yaml'}

    scenario = yaml.safe_load(
        (PACKAGE_ROOT / 'config/scenarios/trajectory_gt_wall_high.yaml').read_text(
            encoding='utf-8'))
    goals = [step['goals'][0] for step in scenario['steps'] if step['type'] == 'move']
    assert [goal['target'] for goal in goals] == [
        [-10.0, -10.0, 1.0],
        [0.0, -10.0, 1.0],
        [10.0, -10.0, 1.0],
        [10.0, -10.0, 3.0],
        [-10.0, -10.0, 3.0],
        [-10.0, -10.0, 5.0],
        [10.0, -10.0, 5.0],
    ]
    assert all(goal['drone_id'] == 1 for goal in goals)
    assert all(goal['navigation_source'] == 'GT' for goal in goals)
    assert all(goal['yaw_deg'] == 90.0 for goal in goals)


def test_three_level_square_trajectory_preserves_absolute_and_relative_yaw():
    profile = yaml.safe_load(
        (PACKAGE_ROOT / 'config/mission_profiles/trajectory_gt_square_3_levels.yaml').read_text(
            encoding='utf-8'))
    assert profile == {
        'mission_mode': 'trajectory',
        'navigation_source': 'gt',
        'trajectory_file': '../scenarios/trajectory_gt_square_3_levels.yaml'}

    scenario = yaml.safe_load(
        (PACKAGE_ROOT / 'config/scenarios/trajectory_gt_square_3_levels.yaml').read_text(
            encoding='utf-8'))
    goals = scenario['steps'][2]['goals']
    expected_targets = [
        [-10.0, -10.0, 1.0], [0.0, -10.0, 1.0], [10.0, -10.0, 1.0],
        [10.0, 0.0, 1.0], [10.0, 10.0, 1.0], [0.0, 10.0, 1.0],
        [-10.0, 10.0, 1.0], [-10.0, 0.0, 1.0], [-10.0, -10.0, 1.0],
        [-10.0, -10.0, 3.0], [0.0, -10.0, 3.0], [10.0, -10.0, 3.0],
        [10.0, 0.0, 3.0], [10.0, 10.0, 3.0], [0.0, 10.0, 3.0],
        [-10.0, 10.0, 3.0], [-10.0, 0.0, 3.0], [-10.0, -10.0, 3.0],
        [-10.0, -10.0, 5.0], [0.0, -10.0, 5.0], [10.0, -10.0, 5.0],
        [10.0, 0.0, 5.0], [10.0, 10.0, 5.0], [0.0, 10.0, 5.0],
        [-10.0, 10.0, 5.0], [-10.0, 0.0, 5.0], [-10.0, -10.0, 5.0],
    ]
    assert [goal['target'] for goal in goals] == expected_targets
    assert all(goal['drone_id'] == 1 for goal in goals)
    assert [goal['absoluto_yaw'] for goal in goals[:9]] == [
        True, True, True, False, False, False, True, True, True]
    assert [goal['absoluto_yaw'] for goal in goals[9:18]] == [
        True, True, True, False, False, False, True, True, True]
    assert [goal['absoluto_yaw'] for goal in goals[18:]] == [
        True, True, True, False, False, False, True, True, True]


def test_five_level_square_trajectory_uses_one_meter_vertical_steps():
    profile = yaml.safe_load(
        (PACKAGE_ROOT / 'config/mission_profiles/trajectory_gt_square_5_levels.yaml').read_text(
            encoding='utf-8'))
    assert profile == {
        'mission_mode': 'trajectory',
        'navigation_source': 'gt',
        'trajectory_file': '../scenarios/trajectory_gt_square_5_levels.yaml'}

    scenario = yaml.safe_load(
        (PACKAGE_ROOT / 'config/scenarios/trajectory_gt_square_5_levels.yaml').read_text(
            encoding='utf-8'))
    goals = scenario['steps'][2]['goals']
    expected_targets = []
    for z in (1.0, 2.0, 3.0, 4.0, 5.0):
        expected_targets.extend([
            [-10.0, -10.0, z], [0.0, -10.0, z], [10.0, -10.0, z],
            [10.0, 0.0, z], [10.0, 10.0, z], [0.0, 10.0, z],
            [-10.0, 10.0, z], [-10.0, 0.0, z], [-10.0, -10.0, z]])
    assert [goal['target'] for goal in goals] == expected_targets
    assert len(goals) == 45
    assert all(goal['drone_id'] == 1 for goal in goals)
    yaw_mask = [True, True, True, False, False, False, True, True, True]
    for offset in range(0, len(goals), 9):
        assert [goal['absoluto_yaw'] for goal in goals[offset:offset + 9]] == yaw_mask


def test_mission_interfaces_are_exact_server_drone_replicas():
    server = SRC_ROOT / 'servidor/mission_msgs'
    drone = SRC_ROOT / 'dron/mission_msgs'
    server_files = {path.relative_to(server): path for path in server.rglob('*') if path.is_file()}
    drone_files = {path.relative_to(drone): path for path in drone.rglob('*') if path.is_file()}
    assert set(server_files) == set(drone_files)
    for relative, source in server_files.items():
        assert source.read_bytes() == drone_files[relative].read_bytes()
