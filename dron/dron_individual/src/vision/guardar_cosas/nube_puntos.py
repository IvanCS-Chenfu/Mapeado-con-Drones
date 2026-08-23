#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from std_msgs.msg import Header

from cv_bridge import CvBridge
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
import struct


class NubePuntosDesdeDepth(Node):
    def __init__(self):
        super().__init__("nube_puntos_desde_depth")

        # Subs imágenes + camera_info (izquierda)
        self.sub_izq = self.create_subscription(
            Image, "/sensor/camara_izq/image_raw", self.cb_img_izq, 10
        )
        self.sub_der = self.create_subscription(
            Image, "/sensor/camara_der/image_raw", self.cb_img_der, 10
        )
        self.sub_info = self.create_subscription(
            CameraInfo, "/sensor/camara_izq/camera_info", self.cb_info, 10
        )

        # Pub PointCloud2
        self.pub_pc = self.create_publisher(PointCloud2, "/stereo/points", 10)

        self.bridge = CvBridge()
        self.img_izq = None
        self.img_der = None
        self.info = None
        self.last_header = None

        # Estéreo
        self.baseline = 0.06  # m
        
        plt.ion()
        self.fig = plt.figure("Camaras")

        # SGBM (ajusta si quieres)
        window_size = 7
        min_disp = 0
        num_disp = 16 * 8  # 128 (múltiplo de 16)
        self.stereo = cv.StereoSGBM_create(
            minDisparity=min_disp,
            numDisparities=num_disp,
            blockSize=window_size,
            P1=8 * 1 * window_size**2,
            P2=32 * 1 * window_size**2,
            disp12MaxDiff=1,
            uniquenessRatio=10,
            speckleWindowSize=50,
            speckleRange=2,
            preFilterCap=63,
            mode=cv.STEREO_SGBM_MODE_SGBM_3WAY
        )

        # Publicar a ~10 Hz
        self.timer = self.create_timer(0.1, self.procesar)

    def cb_img_izq(self, msg: Image):
        self.img_izq = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.last_header = msg.header

    def cb_img_der(self, msg: Image):
        self.img_der = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def cb_info(self, msg: CameraInfo):
        self.info = msg

    def procesar(self):
        if self.img_izq is None or self.img_der is None or self.info is None:
            return

        # --- 1) Disparidad
        left_gray = cv.cvtColor(self.img_izq, cv.COLOR_BGR2GRAY)
        right_gray = cv.cvtColor(self.img_der, cv.COLOR_BGR2GRAY)

        disp = self.stereo.compute(left_gray, right_gray).astype(np.float32) / 16.0

        # --- 2) Profundidad desde disparidad: Z = fx * B / d
        fx = float(self.info.k[0])
        fy = float(self.info.k[4])
        cx = float(self.info.k[2])
        cy = float(self.info.k[5])

        # Marcar disparidad inválida
        disp[disp <= 0.0] = np.nan

        depth = (fx * self.baseline) / disp  # metros

        # Limitar rango para limpiar nube
        z_min, z_max = 0.2, 20.0
        depth = np.where((depth >= z_min) & (depth <= z_max), depth, np.nan)
        
        plt.subplot(1,3,1)
        plt.imshow(self.img_izq)
        plt.title("Camara Izquierda")

        plt.subplot(1,3,2)
        plt.imshow(self.img_der)
        plt.title("Camara Derecha")
        
        plt.subplot(1,3,3)
        plt.imshow(depth, cmap='gray')
        plt.colorbar()
        plt.title("Profundidad")
        
        plt.pause(0.001)
        plt.clf()

        # --- 3) Reproyección pinhole (sin Open3D)
        h, w = depth.shape

        # Máscara válida
        valid = np.isfinite(depth)

        # (Muy importante) borde izquierdo inválido por rango de disparidad
        left_margin = int(self.stereo.getNumDisparities() + self.stereo.getMinDisparity())
        valid[:, :left_margin] = False

        # Si casi no hay puntos, no publiques
        if np.count_nonzero(valid) < 500:
            return

        v, u = np.indices((h, w), dtype=np.float32)

        Z = depth
        X = (u - cx) * Z / fx
        Y = (v - cy) * Z / fy

        X = X[valid].astype(np.float32)
        Y = Y[valid].astype(np.float32)
        Z = Z[valid].astype(np.float32)
        pts = np.stack((X, Y, Z), axis=1)

        # Colores (RGB) desde imagen izquierda
        rgb = cv.cvtColor(self.img_izq, cv.COLOR_BGR2RGB)
        cols = rgb[valid].astype(np.uint8)  # Nx3 uint8

        # --- 4) Publicar PointCloud2
        # Usa el frame_id del CameraInfo si existe; si no, pon uno fijo.
        frame_id = self.info.header.frame_id if self.info.header.frame_id else "camara_izq"

        msg = self.xyzrgb_to_pointcloud2(pts, cols, frame_id)
        self.pub_pc.publish(msg)

    def xyzrgb_to_pointcloud2(self, pts_xyz: np.ndarray, cols_rgb_uint8: np.ndarray, frame_id: str) -> PointCloud2:
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
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


def main(args=None):
    rclpy.init(args=args)
    node = NubePuntosDesdeDepth()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()