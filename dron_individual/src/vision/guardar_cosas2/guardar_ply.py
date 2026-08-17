#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from rclpy.qos import qos_profile_sensor_data

import numpy as np
import struct
import os


class GuardarPLY(Node):
    def __init__(self):
        super().__init__("guardar_ply")

        # Parámetros
        self.declare_parameter("topic", "/stereo/points_map")
        self.declare_parameter("output", "nube.ply")

        self.topic = self.get_parameter("topic").value
        self.output = self.get_parameter("output").value

        self.sub = self.create_subscription(
            PointCloud2, self.topic, self.cb, qos_profile_sensor_data
        )

        self.guardado = False
        self.get_logger().info(f"Esperando un mensaje en {self.topic} para guardar {self.output}...")

    def cb(self, msg: PointCloud2):
        if self.guardado:
            return

        pts, cols = self.read_xyzrgb_from_pc2(msg)
        if pts.shape[0] == 0:
            self.get_logger().warn("Nube vacía. No se guarda.")
            self.guardado = True
            rclpy.shutdown()
            return

        out_path = os.path.abspath(self.output)
        self.write_ply_binary(out_path, pts, cols)

        self.get_logger().info(f"Guardado PLY: {out_path}  (puntos: {pts.shape[0]})")

        self.guardado = True
        rclpy.shutdown()

    def read_xyzrgb_from_pc2(self, msg: PointCloud2):
        """
        Espera layout x,y,z,rgb (rgb packed uint32) con point_step=16 (como tus publicadores).
        """
        step = msg.point_step
        n_points = msg.width * msg.height
        if n_points == 0:
            return np.empty((0, 3), np.float32), np.empty((0, 3), np.uint8)

        pts = np.empty((n_points, 3), dtype=np.float32)
        cols = np.empty((n_points, 3), dtype=np.uint8)

        data = msg.data
        for i in range(n_points):
            offset = i * step
            x, y, z, rgb = struct.unpack_from("fffI", data, offset)
            pts[i] = (x, y, z)
            cols[i] = ((rgb >> 16) & 0xFF, (rgb >> 8) & 0xFF, rgb & 0xFF)

        valid = np.isfinite(pts).all(axis=1)
        return pts[valid], cols[valid]

    def write_ply_binary(self, filename: str, pts_xyz: np.ndarray, cols_rgb: np.ndarray):
        """
        Guarda PLY binario little-endian con campos:
        x y z (float) + red green blue (uchar)
        """
        n = pts_xyz.shape[0]

        header = "\n".join([
            "ply",
            "format binary_little_endian 1.0",
            f"element vertex {n}",
            "property float x",
            "property float y",
            "property float z",
            "property uchar red",
            "property uchar green",
            "property uchar blue",
            "end_header\n"
        ]).encode("ascii")

        with open(filename, "wb") as f:
            f.write(header)
            # Pack por punto: 3 floats + 3 uint8
            for (x, y, z), (r, g, b) in zip(pts_xyz, cols_rgb):
                f.write(struct.pack("<fffBBB", float(x), float(y), float(z), int(r), int(g), int(b)))


def main(args=None):
    rclpy.init(args=args)
    node = GuardarPLY()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == "__main__":
    main()