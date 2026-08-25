import importlib.util
from pathlib import Path

import pytest
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = PACKAGE_ROOT / 'scripts/fiducial_config.py'
SPEC = importlib.util.spec_from_file_location(
    'fiducial_config_server', MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def profile():
    path = PACKAGE_ROOT / 'config/fiducial_objects.yaml'
    return yaml.safe_load(path.read_text(encoding='utf-8'))


def write_profile(tmp_path, document):
    path = tmp_path / 'fiducials.yaml'
    path.write_text(yaml.safe_dump(document), encoding='utf-8')
    return path


def test_loads_canonical_detector_policy_and_flattens_tags():
    config = MODULE.load_fiducial_config(
        PACKAGE_ROOT / 'config/fiducial_objects.yaml')
    assert config['family'] == 'APRILTAG_36H11'
    assert config['corner_refinement'] == 'SUBPIX'
    assert config['pose_solver'] == 'IPPE_SQUARE'
    assert config['max_reprojection_error_px'] == 3.0
    assert len(config['tags']) == 15
    assert config['tags'] == sorted(config['tags'])


def test_empty_object_list_is_a_valid_disabled_configuration(tmp_path):
    document = profile()
    document['objects'] = []
    config = MODULE.load_fiducial_config(write_profile(tmp_path, document))
    assert config['tags'] == []


def test_rejects_duplicate_tag_ids(tmp_path):
    document = profile()
    document['objects'][1]['faces']['pos_x']['tag_id'] = 101
    with pytest.raises(ValueError, match='duplicado'):
        MODULE.load_fiducial_config(write_profile(tmp_path, document))
