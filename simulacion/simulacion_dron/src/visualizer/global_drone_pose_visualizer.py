#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Point
from orbslam3_msgs.msg import NavigationState
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


AXIS_COLORS = (
    (1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
    (0.0, 0.35, 1.0),
)


def rotate_vector(quaternion, vector):
    x, y, z, w = quaternion
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        raise ValueError('quaternion nulo')
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    vx, vy, vz = vector
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + y * tz - z * ty,
        vy + w * ty + z * tx - x * tz,
        vz + w * tz + x * ty - y * tx,
    )


def marker_namespace(drone_id):
    return f'global_drone_pose_{drone_id}'


def pose_is_visible(message):
    return (
        message.local_valid and message.local_continuity_valid and
        message.velocity_valid)


def pose_source_name(message):
    if message.pose_source == NavigationState.POSE_SOURCE_ORB:
        return 'ORB'
    if message.pose_source == NavigationState.POSE_SOURCE_GT_FALLBACK:
        return 'GT'
    return 'INVALID'


def delete_markers(drone_id, stamp, frame_id):
    result = MarkerArray()
    for marker_id in range(4):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = frame_id
        marker.ns = marker_namespace(drone_id)
        marker.id = marker_id
        marker.action = Marker.DELETE
        result.markers.append(marker)
    return result


def build_pose_markers(message, frame_id='world', axis_length=0.8):
    if not pose_is_visible(message):
        return delete_markers(message.drone_id, message.header.stamp, frame_id)

    result = MarkerArray()
    pose = message.o_t_body
    origin = (pose.position.x, pose.position.y, pose.position.z)
    quaternion = (
        pose.orientation.x, pose.orientation.y,
        pose.orientation.z, pose.orientation.w)
    namespace = marker_namespace(message.drone_id)

    for marker_id, (basis, color) in enumerate(zip(
            ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
            AXIS_COLORS)):
        direction = rotate_vector(quaternion, basis)
        marker = Marker()
        marker.header = message.header
        marker.header.frame_id = frame_id
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.points = [
            Point(x=origin[0], y=origin[1], z=origin[2]),
            Point(
                x=origin[0] + axis_length * direction[0],
                y=origin[1] + axis_length * direction[1],
                z=origin[2] + axis_length * direction[2]),
        ]
        marker.scale.x = 0.055
        marker.scale.y = 0.11
        marker.scale.z = 0.16
        marker.color.r, marker.color.g, marker.color.b = color
        marker.color.a = 1.0
        result.markers.append(marker)

    label = Marker()
    label.header = message.header
    label.header.frame_id = frame_id
    label.ns = namespace
    label.id = 3
    label.type = Marker.TEXT_VIEW_FACING
    label.action = Marker.ADD
    label.pose.position.x = origin[0]
    label.pose.position.y = origin[1]
    label.pose.position.z = origin[2] + axis_length + 0.2
    label.pose.orientation.w = 1.0
    label.scale.z = 0.38
    label.color.r = 1.0
    label.color.g = 1.0
    label.color.b = 1.0
    label.color.a = 1.0
    label.text = f'drone_{message.drone_id} [{pose_source_name(message)}]'
    result.markers.append(label)
    return result


class GlobalDronePoseVisualizer(Node):
    def __init__(self):
        super().__init__('global_drone_pose_visualizer')
        self.declare_parameter('drone_count', 2)
        self.declare_parameter('namespace_base', 'dron')
        self.declare_parameter('global_frame', 'world')
        self.declare_parameter('axis_length', 0.8)
        self.drone_count = int(self.get_parameter('drone_count').value)
        self.namespace_base = str(self.get_parameter('namespace_base').value)
        self.global_frame = str(self.get_parameter('global_frame').value)
        self.axis_length = float(self.get_parameter('axis_length').value)
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(
            MarkerArray, '/global_drone_poses', qos)
        self.input_subscriptions = []
        self.last_state = {}
        for drone_id in range(1, self.drone_count + 1):
            topic = (
                f'/{self.namespace_base}_{drone_id}'
                '/orbslam/navigation_state')
            self.input_subscriptions.append(self.create_subscription(
                NavigationState, topic,
                lambda message, drone_id=drone_id: self.on_navigation(
                    drone_id, message),
                QoSProfile(
                    depth=20, reliability=ReliabilityPolicy.RELIABLE)))
        self.get_logger().info(
            f'[F5F-RVIZ-READY] drones={self.drone_count} '
            f'frame={self.global_frame}')

    def on_navigation(self, expected_drone_id, message):
        if int(message.drone_id) != expected_drone_id:
            return
        valid = pose_is_visible(message)
        previous = self.last_state.get(expected_drone_id)
        current = (
            valid, int(message.map_epoch), int(message.reference_keyframe_id),
            int(message.pose_revision))
        if valid:
            self.publisher.publish(build_pose_markers(
                message, self.global_frame, self.axis_length))
        if current != previous:
            self.get_logger().info(
                f'[F5H-RVIZ-CONTROL-POSE] drone_id={expected_drone_id} '
                f'visible={str(valid).lower()} epoch={message.map_epoch} '
                f'ref_kf={message.reference_keyframe_id} '
                f'revision={message.pose_revision} '
                f'source={pose_source_name(message)}')
        self.last_state[expected_drone_id] = current


def main(args=None):
    rclpy.init(args=args)
    node = GlobalDronePoseVisualizer()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
