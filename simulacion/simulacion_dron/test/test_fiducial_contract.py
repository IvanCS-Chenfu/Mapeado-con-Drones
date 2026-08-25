import importlib.util
from pathlib import Path

import pytest
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PACKAGE_ROOT.parents[1]
MODULE_PATH = PACKAGE_ROOT / 'src/fiducials/fiducial_spawner.py'
SPEC = importlib.util.spec_from_file_location('fiducial_spawner', MODULE_PATH)
FIDUCIALS = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(FIDUCIALS)


def load(path):
    return yaml.safe_load(path.read_text(encoding='utf-8'))


def baseline():
    return load(PACKAGE_ROOT / 'config/fiducial_objects.yaml')


def test_server_and_simulation_fiducial_contracts_are_identical_and_valid():
    simulation = baseline()
    server = load(SRC_ROOT / 'servidor/orbslam3_server/config/fiducial_objects.yaml')
    assert simulation == server
    validated = FIDUCIALS.validate_contract(simulation, require_baseline=True)
    assert [obj['world_T_object']['translation_m'] for obj in validated['objects']] == [
        [0.0, 8.5, 1.0], [0.0, -8.5, 1.0], [8.5, 0.0, 1.0]]
    assert validated['anchor_eligibility'] == {
        'min_distance_m': 1.0, 'max_distance_m': 5.0}
    assert simulation['detector'] == {
        'corner_refinement': 'SUBPIX',
        'pose_solver': 'IPPE_SQUARE',
        'max_reprojection_error_px': 3.0,
    }


def test_phase4c_receipt_and_phase4d_async_contract_are_explicit():
    system_header = (
        SRC_ROOT / 'dron/ORB_SLAM3/include/System.h'
    ).read_text(encoding='utf-8')
    wrapper = (
        SRC_ROOT / 'dron/orbslam3_ros2/src/stereo/stereo-slam-node.cpp'
    ).read_text(encoding='utf-8')
    assert 'StereoTrackingReceipt' in system_header
    assert 'KeyFrameCreationEvent' in system_header
    assert 'GetLastKeyFrameInfo' not in wrapper
    assert 'EnqueueFiducialJob(tracking_receipt, msgLeft->header.frame_id)' in wrapper
    assert 'FID-QUEUE-DROP-OLDEST' in wrapper
    assert 'FiducialWorkerLoop' in wrapper
    assert 'PublishFiducialDebugImage' in wrapper
    assert 'orbslam/fiducial_debug/image' in wrapper
    assert 'cv::namedWindow' not in wrapper
    assert 'cv::imshow' not in wrapper
    orbslam_launch = (
        SRC_ROOT / 'dron/dron_individual/launch/orbslam_use.launch.py'
    ).read_text(encoding='utf-8')
    assert "if '/snap/' not in part" in orbslam_launch
    assert 'additional_env=orbslam_environment' in orbslam_launch
    assert "executable='fiducial_visualizer'" in orbslam_launch
    visualizer = (
        SRC_ROOT / 'dron/orbslam3_ros2/src/stereo/'
        'fiducial-visualizer-node.cpp'
    ).read_text(encoding='utf-8')
    assert 'cv::namedWindow' in visualizer
    assert 'cv::imshow' in visualizer
    assert 'window_was_visible_' in visualizer
    assert '[FID-VISUALIZER-CLOSE]' in visualizer


def test_empty_real_deployment_is_valid_but_not_the_simulation_baseline():
    document = baseline()
    document['objects'] = []
    FIDUCIALS.validate_contract(document)
    with pytest.raises(FIDUCIALS.ContractError):
        FIDUCIALS.validate_contract(document, require_baseline=True)


def test_ids_ranges_and_geometry_reject_invalid_contracts():
    duplicate = baseline()
    duplicate['objects'][1]['faces']['pos_x']['tag_id'] = 101
    with pytest.raises(FIDUCIALS.ContractError, match='duplicado'):
        FIDUCIALS.validate_contract(duplicate)

    invalid_range = baseline()
    invalid_range['anchor_eligibility']['min_distance_m'] = 5.0
    with pytest.raises(FIDUCIALS.ContractError, match='menor'):
        FIDUCIALS.validate_contract(invalid_range)

    invalid_size = baseline()
    invalid_size['objects'][0]['size_m']['x'] = float('nan')
    with pytest.raises(FIDUCIALS.ContractError, match='finito'):
        FIDUCIALS.validate_contract(invalid_size)


def test_id_zero_and_tag_larger_than_face_are_deliberately_accepted():
    document = baseline()
    document['objects'][0]['faces']['pos_x']['tag_id'] = 0
    document['objects'][0]['faces']['pos_x']['size_m'] = 0.80
    FIDUCIALS.validate_contract(document)


def test_textures_redetect_and_sdf_keeps_static_collision_and_tag_size(tmp_path):
    contract = FIDUCIALS.validate_contract(baseline(), require_baseline=True)
    rendering = load(PACKAGE_ROOT / 'config/fiducial_rendering.yaml')
    rendering['runtime_asset_dir'] = str(tmp_path / 'assets')
    rendering = FIDUCIALS.validate_rendering(rendering)
    root = FIDUCIALS.generate_assets(contract, rendering)
    assert len(list((root / 'materials/textures').glob('tag_*.png'))) == 15
    sdf = FIDUCIALS.build_model_sdf(contract['objects'][0], rendering, root)
    assert '<static>true</static>' in sdf
    assert 'body_collision' in sdf
    assert 'tag_101_pos_x' in sdf
    assert '<size>0.3 0.3</size>' in sdf


def test_launch_and_scenario_gate_motion_after_spawn():
    launch = (PACKAGE_ROOT / 'launch/multi_dron.launch.py').read_text(encoding='utf-8')
    scenario = load(
        PACKAGE_ROOT /
        'config/scenarios/prueba_tipica_rodeo_edificio_dos_fiduciales.yaml')
    assert "executable='fiducial_spawner.py'" in launch
    assert scenario['steps'][0]['type'] == 'wait_for_bool'
    assert scenario['steps'][0]['topic'] == '/fiducial_spawn_ready'
    targets = [
        goal['target']
        for step in scenario['steps'] if step['type'] == 'move'
        for goal in step['goals']]
    assert all(abs(value) != 9.0 for target in targets for value in target[:2])
    target_xy = {(target[0], target[1]) for target in targets}
    assert {(0.0, -10.0), (10.0, 0.0),
            (0.0, 10.0), (-10.0, 0.0)} <= target_xy
    auxiliary = load(
        SRC_ROOT / 'codex/archivos_auxiliares/trayectorias/'
        'prueba_tipica_rodeo_edificio_dos_fiduciales.yaml')
    assert auxiliary == scenario
