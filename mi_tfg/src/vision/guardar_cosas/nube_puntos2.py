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

import open3d as o3d


class NubePuntosDesdeDepth(Node):
    def __init__(self):
        super().__init__("nube_puntos_desde_depth_wls")

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

        # Estéreo
        self.baseline = 0.06  # m (ajusta al tuyo real)

        # Visualización rápida
        plt.ion()
        self.fig = plt.figure("Stereo + Depth (WLS)")

        # -------- SGBM (izq->der) ----------
        window_size = 7
        min_disp = 0
        num_disp = 16 * 8  # 128 (múltiplo de 16)

        self.left_matcher = cv.StereoSGBM_create(
            minDisparity=min_disp,
            numDisparities=num_disp,
            blockSize=window_size,
            P1=8 * 1 * window_size**2,
            P2=32 * 1 * window_size**2,
            disp12MaxDiff=1,
            uniquenessRatio=10,
            speckleWindowSize=80,
            speckleRange=2,
            preFilterCap=63,
            mode=cv.STEREO_SGBM_MODE_SGBM_3WAY
        )

        # -------- Right matcher + WLS ----------
        # Necesita opencv-contrib-python (cv2.ximgproc)
        self.right_matcher = cv.ximgproc.createRightMatcher(self.left_matcher)

        self.wls = cv.ximgproc.createDisparityWLSFilter(matcher_left=self.left_matcher)
        # lambda: suavizado global (más alto = más suave)
        self.wls.setLambda(8000)
        # sigmaColor: cuánto respeta bordes (1.0–2.0 típico)
        self.wls.setSigmaColor(1.5)
        # umbral de consistencia left-right (en unidades *16 en la implementación interna)
        self.wls.setLRCthresh(24)

        # Preproceso (opcional pero ayuda en zonas con poca textura)
        self.use_clahe = True
        self.clahe = cv.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

        # Máscara anti-bordes de disparidad (opcional, sube calidad a costa de densidad)
        self.use_disp_edge_mask = True
        self.disp_edge_thr = 2.0  # umbral de gradiente de disparidad (ajustable)

        # Limitar rango de profundidad (limpieza básica)
        self.z_min, self.z_max = 0.2, 10.0

        # Post-proceso con Open3D
        self.post_procesado = False

        # Parámetros de post-proceso (ajústalos según tu escala/ruido)
        self.o3d_voxel = 0.005        # metros (ej: 0.005 = 5mm, 0.01 = 1cm)
        self.o3d_sor_nb = 30
        self.o3d_sor_std = 2.5
        self.o3d_ror_nb = 15
        self.o3d_ror_radius = 0.02   # metros (típico 2–4x voxel)

        # Publicar a ~10 Hz
        self.timer = self.create_timer(0.1, self.procesar)

    def cb_img_izq(self, msg: Image):
        self.img_izq = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def cb_img_der(self, msg: Image):
        self.img_der = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def cb_info(self, msg: CameraInfo):
        self.info = msg

    def procesar(self):
        if self.img_izq is None or self.img_der is None or self.info is None:
            return

        # --- 1) Preparar grises
        left_gray = cv.cvtColor(self.img_izq, cv.COLOR_BGR2GRAY)
        right_gray = cv.cvtColor(self.img_der, cv.COLOR_BGR2GRAY)

        if self.use_clahe:
            left_gray = self.clahe.apply(left_gray)
            right_gray = self.clahe.apply(right_gray)

        # (opcional) Un bilateral muy suave puede ayudar; si te “emborrona” detalles, quítalo
        # left_gray = cv.bilateralFilter(left_gray, d=5, sigmaColor=25, sigmaSpace=5)
        # right_gray = cv.bilateralFilter(right_gray, d=5, sigmaColor=25, sigmaSpace=5)

        # --- 2) Disparidad L->R y R->L (int16 * 16)
        dispL_16 = self.left_matcher.compute(left_gray, right_gray).astype(np.int16)
        dispR_16 = self.right_matcher.compute(right_gray, left_gray).astype(np.int16)

        # --- 3) Filtrado WLS (devuelve int16 * 16 típicamente)
        disp_filt_16 = self.wls.filter(dispL_16, left_gray, None, dispR_16)

        # Convertir a float en píxeles
        disp = disp_filt_16.astype(np.float32) / 16.0

        # --- 4) Intrínsecos
        fx = float(self.info.k[0])
        fy = float(self.info.k[4])
        cx = float(self.info.k[2])
        cy = float(self.info.k[5])

        # --- 5) Máscara válida (más estricta que antes)
        # disparidad válida
        valid = np.isfinite(disp) & (disp > 0.5)

        # borde izquierdo inválido por rango de disparidad
        left_margin = int(self.left_matcher.getNumDisparities() + self.left_matcher.getMinDisparity())
        valid[:, :left_margin] = False

        # --- 6) (Opcional) máscara anti-bordes de disparidad (descarta zonas donde la disp es inestable)
        if self.use_disp_edge_mask:
            dx = cv.Sobel(disp, cv.CV_32F, 1, 0, ksize=3)
            dy = cv.Sobel(disp, cv.CV_32F, 0, 1, ksize=3)
            mag = np.sqrt(dx * dx + dy * dy)
            valid = valid & (mag < self.disp_edge_thr)

        # --- 7) Profundidad desde disparidad: Z = fx * B / d
        depth = np.full_like(disp, np.nan, dtype=np.float32)
        depth[valid] = (fx * self.baseline) / disp[valid]  # metros

        # Limitar rango Z
        depth = np.where((depth >= self.z_min) & (depth <= self.z_max), depth, np.nan)

        # Recalcular máscara válida
        valid = np.isfinite(depth)

        # Si casi no hay puntos, no publiques
        if np.count_nonzero(valid) < 500:
            return

        # --- 8) Visualización
        try:
            plt.subplot(1, 3, 1)
            plt.imshow(cv.cvtColor(self.img_izq, cv.COLOR_BGR2RGB))
            plt.title("Izquierda")

            plt.subplot(1, 3, 2)
            plt.imshow(cv.cvtColor(self.img_der, cv.COLOR_BGR2RGB))
            plt.title("Derecha")

            plt.subplot(1, 3, 3)
            plt.imshow(depth, cmap="gray")
            plt.title("Depth (WLS)")
            plt.colorbar()

            plt.pause(0.001)
            plt.clf()
        except Exception:
            pass

        # --- 9) Reproyección pinhole -> nube
        h, w = depth.shape
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
        cols = rgb[valid].astype(np.uint8)

        # Post-proceso con Open3D
        if self.post_procesado:
            # 1) Crear PointCloud de Open3D
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(pts.astype(np.float64))
            pcd.colors = o3d.utility.Vector3dVector((cols.astype(np.float64) / 255.0))

            # 2) Voxel downsample (uniformiza densidad y reduce ruido)
            if self.o3d_voxel is not None and self.o3d_voxel > 0.0:
                pcd = pcd.voxel_down_sample(voxel_size=float(self.o3d_voxel))

            # 3) Statistical Outlier Removal (quita puntos aislados estadísticamente)
            if self.o3d_sor_nb is not None and self.o3d_sor_nb > 0:
                pcd, _ = pcd.remove_statistical_outlier(
                    nb_neighbors=int(self.o3d_sor_nb),
                    std_ratio=float(self.o3d_sor_std)
                )

            # 4) Radius Outlier Removal (quita puntos sin suficientes vecinos en un radio)
            if self.o3d_ror_nb is not None and self.o3d_ror_nb > 0 and self.o3d_ror_radius is not None:
                pcd, _ = pcd.remove_radius_outlier(
                    nb_points=int(self.o3d_ror_nb),
                    radius=float(self.o3d_ror_radius)
                )

            # 5) Volver a numpy para publicar
            pts = np.asarray(pcd.points, dtype=np.float32)
            cols = (np.asarray(pcd.colors, dtype=np.float32) * 255.0).clip(0, 255).astype(np.uint8)

            # Si tras filtrar se queda vacío, no publiques
            if pts.shape[0] < 200:
                return

        # --- 10) Publicar PointCloud2
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