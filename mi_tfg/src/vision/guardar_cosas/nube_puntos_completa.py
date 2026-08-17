#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

import numpy as np
import struct

from rclpy.qos import qos_profile_sensor_data


def quat_to_rot(qx, qy, qz, qw) -> np.ndarray:
    """
    Quaternion (x,y,z,w) -> Rotation matrix 3x3
    Asumimos que la orientación del PoseStamped es la del dron (body) respecto a map:
    p_map = R_map_body * p_body + t_map_body
    """
    n = np.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if n == 0.0:
        return np.eye(3, dtype=np.float32)
    qx, qy, qz, qw = qx/n, qy/n, qz/n, qw/n

    xx, yy, zz = qx*qx, qy*qy, qz*qz
    xy, xz, yz = qx*qy, qx*qz, qy*qz
    wx, wy, wz = qw*qx, qw*qy, qw*qz

    return np.array([
        [1 - 2*(yy + zz),     2*(xy - wz),       2*(xz + wy)],
        [2*(xy + wz),         1 - 2*(xx + zz),   2*(yz - wx)],
        [2*(xz - wy),         2*(yz + wx),       1 - 2*(xx + yy)]
    ], dtype=np.float32)


def read_xyzrgb_from_pc2(msg: PointCloud2):
    """
    Lee PointCloud2 con layout x,y,z,rgb (rgb packed uint32) y devuelve:
    pts: Nx3 float32
    cols: Nx3 uint8 (RGB)
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


def xyzrgb_to_pointcloud2(pts_xyz: np.ndarray, cols_rgb_uint8: np.ndarray, frame_id: str, stamp_msg) -> PointCloud2:
    header = Header()
    header.stamp = stamp_msg
    header.frame_id = frame_id

    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1),
    ]

    data = bytearray()
    for (x, y, z), (r, g, b) in zip(pts_xyz, cols_rgb_uint8):
        rgb_packed = (int(r) << 16) | (int(g) << 8) | int(b)
        data.extend(struct.pack("fffI", float(x), float(y), float(z), rgb_packed))

    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = int(pts_xyz.shape[0])
    msg.is_dense = False
    msg.is_bigendian = False
    msg.fields = fields
    msg.point_step = 16
    msg.row_step = msg.point_step * msg.width
    msg.data = bytes(data)
    return msg


def voxel_downsample(pts: np.ndarray, cols: np.ndarray, voxel: float):
    """
    Voxel grid simple: mantiene 1 punto por voxel (el primero).
    """
    if pts.shape[0] == 0 or voxel <= 0:
        return pts, cols

    keys = np.floor(pts / voxel).astype(np.int32)
    h = keys[:, 0] * 73856093 ^ keys[:, 1] * 19349663 ^ keys[:, 2] * 83492791
    _, idx = np.unique(h, return_index=True)
    return pts[idx], cols[idx]


class AcumuladorNube(Node):
    def __init__(self):
        super().__init__("nube_puntos_completa")

        # Parámetros
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("voxel_size", 0.10)     # más grande para RViz
        self.declare_parameter("max_points", 150000)   # más manejable para RViz

        # Traslación cámara->body (si la sabes, ponla aquí)
        self.declare_parameter("cam_in_body_xyz", [0.0, 0.0, 0.0])

        self.global_frame = self.get_parameter("global_frame").value
        self.voxel_size = float(self.get_parameter("voxel_size").value)
        self.max_points = int(self.get_parameter("max_points").value)
        self.t_body_cam = np.array(self.get_parameter("cam_in_body_xyz").value, dtype=np.float32)

        # *** MODIFICACIÓN CLAVE ***
        # Rotación de frame óptico de cámara -> frame body del dron
        # Asume body: X delante, Y izquierda, Z arriba
        # Asume camera optical: X derecha, Y abajo, Z delante
        #
        # X_body =  Z_cam
        # Y_body = -X_cam
        # Z_body = -Y_cam
        self.R_body_cam = np.array([
            [0,  0,  1],
            [-1, 0,  0],
            [0, -1,  0]
        ], dtype=np.float32)

        # Subs
        self.sub_pc = self.create_subscription(
            PointCloud2, "/stereo/points", self.cb_pc, qos_profile_sensor_data
        )
        self.sub_pose = self.create_subscription(
            PoseStamped, "/sensor/GT/pose", self.cb_pose, qos_profile_sensor_data
        )

        # Pub acumulada
        self.pub_map = self.create_publisher(PointCloud2, "/stereo/points_map", 10)

        # Últimos mensajes
        self.last_pc = None
        self.last_pose = None

        # Acumulador
        self.acc_pts = np.empty((0, 3), dtype=np.float32)
        self.acc_cols = np.empty((0, 3), dtype=np.uint8)

        # Publicar a baja frecuencia para que RViz aguante
        self.timer = self.create_timer(1.0, self.fusionar_y_publicar)

        self.get_logger().info("Acumulador listo. Publica /stereo/points_map")

    def cb_pc(self, msg: PointCloud2):
        self.last_pc = msg

    def cb_pose(self, msg: PoseStamped):
        self.last_pose = msg

    def fusionar_y_publicar(self):
        if self.last_pc is None or self.last_pose is None:
            self.get_logger().warn("Esperando /stereo/points y /sensor/GT/pose ...", throttle_duration_sec=2.0)
            return

        pc_msg = self.last_pc
        pose_msg = self.last_pose

        pts_cam, cols = read_xyzrgb_from_pc2(pc_msg)
        if pts_cam.shape[0] < 500:
            self.get_logger().warn(f"Pocos puntos en nube instantánea: {pts_cam.shape[0]}", throttle_duration_sec=2.0)
            return

        # --- MODIFICACIÓN CLAVE: cámara(optical) -> body ---
        # pts_body = R_body_cam * pts_cam + t_body_cam
        pts_body = (pts_cam @ self.R_body_cam.T) + self.t_body_cam

        # --- body -> map usando pose GT ---
        p = pose_msg.pose.position
        q = pose_msg.pose.orientation

        R_map_body = quat_to_rot(q.x, q.y, q.z, q.w)
        t_map_body = np.array([p.x, p.y, p.z], dtype=np.float32)

        pts_map = (pts_body @ R_map_body.T) + t_map_body

        # Acumular
        self.acc_pts = np.vstack([self.acc_pts, pts_map])
        self.acc_cols = np.vstack([self.acc_cols, cols])

        # Voxel downsample
        self.acc_pts, self.acc_cols = voxel_downsample(self.acc_pts, self.acc_cols, self.voxel_size)

        # Limitar tamaño
        if self.acc_pts.shape[0] > self.max_points:
            idx = np.random.choice(self.acc_pts.shape[0], self.max_points, replace=False)
            self.acc_pts = self.acc_pts[idx]
            self.acc_cols = self.acc_cols[idx]

        out = xyzrgb_to_pointcloud2(
            self.acc_pts, self.acc_cols,
            frame_id=self.global_frame,
            stamp_msg=self.get_clock().now().to_msg()
        )
        self.pub_map.publish(out)

        self.get_logger().info(
            f"Publicado /stereo/points_map con {self.acc_pts.shape[0]} puntos "
            f"(voxel={self.voxel_size}m)",
            throttle_duration_sec=1.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = AcumuladorNube()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()