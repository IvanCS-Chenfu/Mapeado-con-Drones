#!/usr/bin/env python3

import json

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from fiducial_config import load_fiducial_config

from orbslam3_msgs.msg import FiducialTagConfig
from orbslam3_msgs.srv import GetFiducialConfig


SERVICE_NAME = '/global_mapping/get_fiducial_config'


class FiducialConfigServer(Node):
    def __init__(self):
        super().__init__('fiducial_config_server')
        self.declare_parameter('config_file', '')
        self.declare_parameter('debug_architecture_telemetry', False)

        config_file = self.get_parameter('config_file').value
        if not config_file:
            raise RuntimeError('config_file no puede estar vacio')

        self.config = load_fiducial_config(config_file)
        self.activity_pub = None
        if self.get_parameter('debug_architecture_telemetry').value:
            self.activity_pub = self.create_publisher(
                String, '/system_architecture/activity', 32)

        self.service = self.create_service(
            GetFiducialConfig, SERVICE_NAME, self.handle_request)
        self.get_logger().info(
            '[FID-CONFIG-SERVER-READY] service=%s schema=%d family=%s '
            'tags=%d config=%s' % (
                SERVICE_NAME,
                self.config['schema_version'],
                self.config['family'],
                len(self.config['tags']),
                config_file,
            ))

    def emit_activity(self, drone_id):
        if self.activity_pub is None:
            return
        message = String()
        message.data = json.dumps({
            'event': 'activity',
            'edge_id': 'fiducial_config_server_to_wrapper',
            'interface': SERVICE_NAME,
            'source': 'orbslam3_server',
            'drone_id': drone_id,
            'timestamp': self.get_clock().now().nanoseconds / 1.0e9,
        }, separators=(',', ':'))
        self.activity_pub.publish(message)

    def handle_request(self, request, response):
        response.success = True
        response.message = (
            'ready' if self.config['tags'] else
            'fiducials disabled/not configured')
        response.schema_version = self.config['schema_version']
        response.family = self.config['family']
        response.corner_refinement = self.config['corner_refinement']
        response.pose_solver = self.config['pose_solver']
        response.max_reprojection_error_px = self.config[
            'max_reprojection_error_px']

        for tag_id, size_m in self.config['tags']:
            tag = FiducialTagConfig()
            tag.tag_id = tag_id
            tag.size_m = size_m
            response.tags.append(tag)

        self.get_logger().info(
            '[FID-CONFIG-SERVED] drone_id=%d drone_name=%s tags=%d' % (
                request.drone_id,
                request.drone_name,
                len(response.tags),
            ))
        self.emit_activity(request.drone_id)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FiducialConfigServer()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
