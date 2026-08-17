#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

import cv2 as cv
import numpy as np


class DepthDesdeStereoWLS(Node):
    def __init__(self):
        super().__init__("depth_desde_stereo_wls")

        self.sub_izq = self.create_subscription(
            Image, "/sensor/camara_izq/image_raw", self.cb_img_izq, 10
        )
        self.sub_der = self.create_subscription(
            Image, "/sensor/camara_der/image_raw", self.cb_img_der, 10
        )
        self.sub_info = self.create_subscription(
            CameraInfo, "/sensor/camara_izq/camera_info", self.cb_info, 10
        )

        self.pub_depth = self.create_publisher(Image, "/stereo/depth", 10)

        self.bridge = CvBridge()
        self.img_izq = None
        self.img_der = None
        self.info = None

        # CLAVE: guardamos el header real de la imagen izquierda
        self.left_header = None

        self.baseline = 0.06  # m

        window_size = 7
        min_disp = 0
        num_disp = 16 * 8

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

        self.right_matcher = cv.ximgproc.createRightMatcher(self.left_matcher)

        self.wls = cv.ximgproc.createDisparityWLSFilter(matcher_left=self.left_matcher)
        self.wls.setLambda(8000)
        self.wls.setSigmaColor(1.5)
        self.wls.setLRCthresh(24)

        self.use_clahe = True
        self.clahe = cv.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

        self.use_disp_edge_mask = True
        self.disp_edge_thr = 2.0

        self.z_min, self.z_max = 0.2, 10.0

        self.timer = self.create_timer(0.1, self.procesar)

    def cb_img_izq(self, msg: Image):
        self.img_izq = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.left_header = msg.header  # <-- AQUÍ guardas el stamp original

    def cb_img_der(self, msg: Image):
        self.img_der = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def cb_info(self, msg: CameraInfo):
        self.info = msg

    def procesar(self):
        if self.img_izq is None or self.img_der is None or self.info is None or self.left_header is None:
            return

        left_gray = cv.cvtColor(self.img_izq, cv.COLOR_BGR2GRAY)
        right_gray = cv.cvtColor(self.img_der, cv.COLOR_BGR2GRAY)

        if self.use_clahe:
            left_gray = self.clahe.apply(left_gray)
            right_gray = self.clahe.apply(right_gray)

        dispL_16 = self.left_matcher.compute(left_gray, right_gray).astype(np.int16)
        dispR_16 = self.right_matcher.compute(right_gray, left_gray).astype(np.int16)

        disp_filt_16 = self.wls.filter(dispL_16, left_gray, None, dispR_16)
        disp = disp_filt_16.astype(np.float32) / 16.0

        fx = float(self.info.k[0])

        valid = np.isfinite(disp) & (disp > 0.5)
        left_margin = int(self.left_matcher.getNumDisparities() + self.left_matcher.getMinDisparity())
        valid[:, :left_margin] = False

        if self.use_disp_edge_mask:
            dx = cv.Sobel(disp, cv.CV_32F, 1, 0, ksize=3)
            dy = cv.Sobel(disp, cv.CV_32F, 0, 1, ksize=3)
            mag = np.sqrt(dx * dx + dy * dy)
            valid = valid & (mag < self.disp_edge_thr)

        depth = np.full_like(disp, np.nan, dtype=np.float32)
        depth[valid] = (fx * self.baseline) / disp[valid]

        depth = np.where((depth >= self.z_min) & (depth <= self.z_max), depth, np.nan)
        depth_pub = np.where(np.isfinite(depth), depth, 0.0).astype(np.float32)

        # ======= TIMESTAMP CORRECTO =======
        depth_msg = self.bridge.cv2_to_imgmsg(depth_pub, encoding="32FC1")
        depth_msg.header = self.left_header  # <-- COPIAS stamp + frame_id + seq
        self.pub_depth.publish(depth_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DepthDesdeStereoWLS()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()