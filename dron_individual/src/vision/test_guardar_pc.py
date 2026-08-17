#!/usr/bin/env python3

import os
import csv

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped

import open3d as o3d
import numpy as np


class GuardarNubeYPose(Node):
    def __init__(self, numero_dron):
        super().__init__("guardar_nube_y_pose")

        self.numero_dron = numero_dron
        self.iteracion = 0
        self.latest_pose = None

        # Carpetas y archivos de salida
        self.output_dir = "./nubes_puntos"
        os.makedirs(self.output_dir, exist_ok=True)

        self.csv_path = os.path.join(self.output_dir, "poses.csv")

        # Crear CSV con cabecera si no existe
        if not os.path.exists(self.csv_path):
            with open(self.csv_path, mode="w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow([
                    "numero_dron",
                    "iteracion",
                    "px", "py", "pz",
                    "qx", "qy", "qz", "qw"
                ])

        # Suscriptores
        self.sub_pose = self.create_subscription(
            PoseStamped,
            "sensor/GT/pose",
            self.pose_cb,
            10
        )

        self.sub_cloud = self.create_subscription(
            PointCloud2,
            "nube_puntos/cuerpo",
            self.cloud_cb,
            10
        )

        self.get_logger().info(
            f"Nodo iniciado. Guardando nubes y poses del dron {self.numero_dron}"
        )

    def pose_cb(self, msg):
        self.latest_pose = msg

    def pointcloud2_to_open3d(self, msg):
        field_names = [f.name for f in msg.fields]

        has_rgb = "rgb" in field_names
        has_rgba = "rgba" in field_names

        if has_rgb:
            data = list(pc2.read_points(
                msg,
                field_names=("x", "y", "z", "rgb"),
                skip_nans=True
            ))
        elif has_rgba:
            data = list(pc2.read_points(
                msg,
                field_names=("x", "y", "z", "rgba"),
                skip_nans=True
            ))
        else:
            data = list(pc2.read_points(
                msg,
                field_names=("x", "y", "z"),
                skip_nans=True
            ))

        if len(data) == 0:
            return o3d.geometry.PointCloud()

        points = np.array([[p[0], p[1], p[2]] for p in data], dtype=np.float64)

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        colors = []
        if has_rgb or has_rgba:
            for p in data:
                packed = p[3]

                if isinstance(packed, float):
                    packed = np.frombuffer(
                        np.float32(packed).tobytes(),
                        dtype=np.uint32
                    )[0]
                else:
                    packed = int(packed)

                r = (packed >> 16) & 255
                g = (packed >> 8) & 255
                b = packed & 255
                colors.append([r / 255.0, g / 255.0, b / 255.0])

            pcd.colors = o3d.utility.Vector3dVector(
                np.array(colors, dtype=np.float64)
            )

        return pcd

    def guardar_pose_csv(self, pose_msg, iteracion_actual):
        p = pose_msg.pose.position
        q = pose_msg.pose.orientation

        with open(self.csv_path, mode="a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                self.numero_dron,
                iteracion_actual,
                p.x, p.y, p.z,
                q.x, q.y, q.z, q.w
            ])

    def guardar_nube_ply(self, pcd, iteracion_actual):
        nombre_fichero = f"pc{self.numero_dron}.{iteracion_actual}.ply"
        ruta_fichero = os.path.join(self.output_dir, nombre_fichero)

        o3d.io.write_point_cloud(ruta_fichero, pcd)
        return ruta_fichero

    def cloud_cb(self, cloud_msg):
        if self.latest_pose is None:
            self.get_logger().warn("Aún no se ha recibido ninguna pose")
            return

        pose_msg = self.latest_pose
        pcd = self.pointcloud2_to_open3d(cloud_msg)

        if len(pcd.points) == 0:
            self.get_logger().warn("Nube de puntos vacía")
            return

        self.iteracion += 1

        ruta_nube = self.guardar_nube_ply(pcd, self.iteracion)
        self.guardar_pose_csv(pose_msg, self.iteracion)

        self.get_logger().info(
            f"Guardados: {ruta_nube} y pose asociada en CSV"
        )


def main(args=None):
    rclpy.init(args=args)

    numero_dron = 2
    node = GuardarNubeYPose(numero_dron=numero_dron)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()