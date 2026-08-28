import importlib.util
import math
from pathlib import Path

from orbslam3_msgs.msg import NavigationState
from visualization_msgs.msg import Marker


MODULE_PATH = (
    Path(__file__).resolve().parents[1] / 'src' / 'visualizer' /
    'global_drone_pose_visualizer.py')
SPEC = importlib.util.spec_from_file_location(
    'global_drone_pose_visualizer', MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def make_state(valid=True, yaw=0.0, drone_id=2, provisional=False,
               pose_source=NavigationState.POSE_SOURCE_ORB):
    message = NavigationState()
    message.drone_id = drone_id
    message.global_valid = valid
    message.local_valid = valid or provisional
    message.local_continuity_valid = valid or provisional
    message.velocity_valid = valid or provisional
    message.pose_source = pose_source
    message.global_status = (
        NavigationState.GLOBAL_STATUS_PROVISIONAL if provisional else
        NavigationState.GLOBAL_STATUS_AUTHORITATIVE if valid else
        NavigationState.GLOBAL_STATUS_INVALID)
    message.o_t_body.position.x = 1.0
    message.o_t_body.position.y = 2.0
    message.o_t_body.position.z = 3.0
    message.o_t_body.orientation.z = math.sin(0.5 * yaw)
    message.o_t_body.orientation.w = math.cos(0.5 * yaw)
    return message


def test_authoritative_pose_builds_xyz_axes_and_label():
    markers = MODULE.build_pose_markers(make_state()).markers
    assert len(markers) == 4
    assert [marker.id for marker in markers] == [0, 1, 2, 3]
    assert all(marker.header.frame_id == 'world' for marker in markers)
    assert [markers[index].type for index in range(3)] == [Marker.ARROW] * 3
    assert markers[3].type == Marker.TEXT_VIEW_FACING
    assert markers[3].text == 'drone_2 [ORB]'
    assert (markers[0].color.r, markers[0].color.g, markers[0].color.b) == (
        1.0, 0.0, 0.0)
    assert (markers[1].color.r, markers[1].color.g, markers[1].color.b) == (
        0.0, 1.0, 0.0)


def test_axes_follow_body_orientation():
    markers = MODULE.build_pose_markers(
        make_state(yaw=math.pi / 2.0), axis_length=1.0).markers
    x_axis_end = markers[0].points[1]
    assert math.isclose(x_axis_end.x, 1.0, abs_tol=1e-6)
    assert math.isclose(x_axis_end.y, 3.0, abs_tol=1e-6)
    assert math.isclose(x_axis_end.z, 3.0, abs_tol=1e-6)


def test_invalid_pose_deletes_all_markers():
    markers = MODULE.build_pose_markers(make_state(valid=False)).markers
    assert len(markers) == 4
    assert all(marker.action == Marker.DELETE for marker in markers)
    assert all(marker.ns == 'global_drone_pose_2' for marker in markers)


def test_provisional_pose_remains_visible_between_authoritative_updates():
    markers = MODULE.build_pose_markers(
        make_state(valid=False, provisional=True)).markers
    assert len(markers) == 4
    assert all(marker.action == Marker.ADD for marker in markers)


def test_fallback_pose_uses_control_pose_and_source_label():
    message = make_state(
        pose_source=NavigationState.POSE_SOURCE_GT_FALLBACK)
    message.w_t_body.position.x = 99.0
    markers = MODULE.build_pose_markers(message).markers
    assert math.isclose(markers[0].points[0].x, 1.0)
    assert markers[3].text == 'drone_2 [GT]'
