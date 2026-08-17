#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

import open3d as o3d
import numpy as np


class GlobalPointCloudMap(Node):
    def __init__(self):
        super().__init__("global_pointcloud_map")

        # Subscriptores y publicador
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
        self.pub_global = self.create_publisher(
            PointCloud2,
            "nube_puntos/global",
            10
        )

        # Última pose recibida
        self.latest_pose = None

        # Parámetros
        self.global_frame = "map"
        self.voxel_size = 0.03
        self.camara2cuerpo = [0.1, 0.03, 0.03]

        # Nube global acumulada
        self.global_cloud = o3d.geometry.PointCloud()

        self.get_logger().info("Nodo de mapa global por acumulación iniciado")

    def pose_cb(self, msg):
        self.latest_pose = msg

    def pointcloud2_to_open3d(self, msg):
        field_names = [f.name for f in msg.fields]

        has_rgb = "rgb" in field_names
        has_rgba = "rgba" in field_names

        if has_rgb:
            data = list(pc2.read_points(msg, field_names=("x", "y", "z", "rgb"), skip_nans=True))
        elif has_rgba:
            data = list(pc2.read_points(msg, field_names=("x", "y", "z", "rgba"), skip_nans=True))
        else:
            data = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))

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
                    packed = np.frombuffer(np.float32(packed).tobytes(), dtype=np.uint32)[0]
                else:
                    packed = int(packed)

                r = (packed >> 16) & 255
                g = (packed >> 8) & 255
                b = packed & 255
                colors.append([r / 255.0, g / 255.0, b / 255.0])
        else:
            colors = [[1.0, 1.0, 1.0] for _ in range(len(points))]

        pcd.colors = o3d.utility.Vector3dVector(np.array(colors, dtype=np.float64))
        return pcd

    def open3d_to_pointcloud2(self, pcd):
        points = np.asarray(pcd.points, dtype=np.float32)

        if len(points) == 0:
            colors = np.zeros((0, 3), dtype=np.uint32)
        else:
            if len(pcd.colors) == 0:
                colors = np.ones((len(points), 3), dtype=np.uint32) * 255
            else:
                colors = (np.asarray(pcd.colors) * 255).astype(np.uint32)

        rgb = (colors[:, 0] << 16) | (colors[:, 1] << 8) | colors[:, 2]

        cloud_data = np.column_stack((points, rgb)).tolist()

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.global_frame

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1),
        ]

        return pc2.create_cloud(header, fields, cloud_data)

    def quat_to_rot(self, qx, qy, qz, qw):
        n = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)

        if n == 0.0:
            return np.eye(3, dtype=np.float64)

        qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n

        xx, yy, zz = qx * qx, qy * qy, qz * qz
        xy, xz, yz = qx * qy, qx * qz, qy * qz
        wx, wy, wz = qw * qx, qw * qy, qw * qz

        return np.array([
            [1 - 2 * (yy + zz), 2 * (xy - wz),     2 * (xz + wy)],
            [2 * (xy + wz),     1 - 2 * (xx + zz), 2 * (yz - wx)],
            [2 * (xz - wy),     2 * (yz + wx),     1 - 2 * (xx + yy)]
        ], dtype=np.float64)

    def cloud_cb(self, cloud_msg):
        # El pipeline pedido es:
        # 1) llega nube en "nube_puntos/cuerpo"
        # 2) se usa la pose actual guardada
        # 3) se transforma la nube cámara -> cuerpo
        # 4) se transforma cuerpo -> mapa
        # 5) se acumula en la nube global
        # 6) se voxeliza y publica

        if self.latest_pose is None:
            self.get_logger().warn("Aún no se ha recibido ninguna pose")
            return

        # Guardar snapshot de la pose actual en el momento de recibir la nube
        pose_msg = self.latest_pose

        # Convertir PointCloud2 a Open3D
        pcd = self.pointcloud2_to_open3d(cloud_msg)

        if len(pcd.points) == 0:
            self.get_logger().warn("Nube de puntos vacía")
            return

        # Transformación cámara -> cuerpo
        # Girar y trasladar al centro del dron
        T_camara_cuerpo = np.array([
            [0,  0,  1,  self.camara2cuerpo[0]],
            [-1, 0,  0,  self.camara2cuerpo[2]],
            [0, -1,  0, -self.camara2cuerpo[1]],
            [0,  0,  0,  1]
        ], dtype=np.float64)

        pcd.transform(T_camara_cuerpo)

        # Transformación cuerpo -> mapa usando la pose actual del dron
        p = pose_msg.pose.position
        q = pose_msg.pose.orientation

        T_cuerpo_mapa = np.eye(4, dtype=np.float64)
        T_cuerpo_mapa[:3, :3] = self.quat_to_rot(q.x, q.y, q.z, q.w)
        T_cuerpo_mapa[:3, 3] = np.array([p.x, p.y, p.z], dtype=np.float64)

        pcd.transform(T_cuerpo_mapa)

        # Acumular directamente, sin ICP ni RANSAC ni ninguna corrección
        self.global_cloud += pcd

        # Voxelizar la nube global acumulada
        self.global_cloud = self.global_cloud.voxel_down_sample(self.voxel_size)

        # Publicar nube global
        msg_out = self.open3d_to_pointcloud2(self.global_cloud)
        self.pub_global.publish(msg_out)

        self.get_logger().info(
            f"Nube acumulada publicada con {len(self.global_cloud.points)} puntos"
        )


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPointCloudMap()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()