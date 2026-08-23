#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

from cv_bridge import CvBridge
import numpy as np
import struct
import open3d as o3d

from message_filters import Subscriber, ApproximateTimeSynchronizer


def quat_to_rot(qx, qy, qz, qw) -> np.ndarray:
    n = np.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if n == 0.0:
        return np.eye(3, dtype=np.float64)
    qx, qy, qz, qw = qx/n, qy/n, qz/n, qw/n

    xx, yy, zz = qx*qx, qy*qy, qz*qz
    xy, xz, yz = qx*qy, qx*qz, qy*qz
    wx, wy, wz = qw*qx, qw*qy, qw*qz

    return np.array([
        [1 - 2*(yy + zz),     2*(xy - wz),       2*(xz + wy)],
        [2*(xy + wz),         1 - 2*(xx + zz),   2*(yz - wx)],
        [2*(xz - wy),         2*(yz + wx),       1 - 2*(xx + yy)]
    ], dtype=np.float64)


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


class TSDFStereoFusion(Node):
    def __init__(self):
        super().__init__("tsdf_stereo_fusion")

        self.declare_parameter("global_frame", "map")
        self.declare_parameter("voxel_length", 0.02)
        self.declare_parameter("sdf_trunc", 0.08)
        self.declare_parameter("depth_trunc", 5.0)

        # Sync
        self.declare_parameter("sync_slop_sec", 0.05)   # tolerancia temporal
        self.declare_parameter("sync_queue", 30)

        # Publicación (evitar cuelgues)
        self.declare_parameter("publish_period_sec", 10.0)
        self.declare_parameter("max_points_publish", 200000)

        self.declare_parameter("cam_in_body_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("use_inverse_extrinsic", True)

        self.global_frame = self.get_parameter("global_frame").value
        self.voxel_length = float(self.get_parameter("voxel_length").value)
        self.sdf_trunc = float(self.get_parameter("sdf_trunc").value)
        self.depth_trunc = float(self.get_parameter("depth_trunc").value)
        self.sync_slop_sec = float(self.get_parameter("sync_slop_sec").value)
        self.sync_queue = int(self.get_parameter("sync_queue").value)
        self.publish_period_sec = float(self.get_parameter("publish_period_sec").value)
        self.max_points_publish = int(self.get_parameter("max_points_publish").value)
        self.t_body_cam = np.array(self.get_parameter("cam_in_body_xyz").value, dtype=np.float64)
        self.use_inverse_extrinsic = bool(self.get_parameter("use_inverse_extrinsic").value)

        # Rotación optical->body
        self.R_body_cam = np.array([
            [0,  0,  1],
            [-1, 0,  0],
            [0, -1,  0]
        ], dtype=np.float64)

        self.bridge = CvBridge()

        # CameraInfo (no lo metemos en sync; lo cacheamos)
        self.sub_info = self.create_subscription(CameraInfo, "/sensor/camara_izq/camera_info", self.cb_info, 10)
        self.o3d_intr = None

        # TSDF volume
        self.volume = o3d.pipelines.integration.ScalableTSDFVolume(
            voxel_length=self.voxel_length,
            sdf_trunc=self.sdf_trunc,
            color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8
        )

        self.pub_tsdf_pc = self.create_publisher(PointCloud2, "/stereo/points_map", 10)

        self.last_publish_time = self.get_clock().now()
        self.integrations = 0
        self.skipped_intr = 0
        self.skipped_depth_empty = 0

        # message_filters subscribers
        self.mf_color = Subscriber(self, Image, "/sensor/camara_izq/image_raw")
        self.mf_depth = Subscriber(self, Image, "/stereo/depth")
        self.mf_pose  = Subscriber(self, PoseStamped, "/sensor/GT/pose")

        self.sync = ApproximateTimeSynchronizer(
            [self.mf_color, self.mf_depth, self.mf_pose],
            queue_size=self.sync_queue,
            slop=self.sync_slop_sec
        )
        self.sync.registerCallback(self.synced_cb)

        self.timer_dbg = self.create_timer(2.0, self.print_status)

        self.get_logger().info("TSDFStereoFusion (message_filters) listo.")

    def cb_info(self, msg: CameraInfo):
        fx = float(msg.k[0])
        fy = float(msg.k[4])
        cx = float(msg.k[2])
        cy = float(msg.k[5])
        self.o3d_intr = o3d.camera.PinholeCameraIntrinsic()
        self.o3d_intr.set_intrinsics(width=msg.width, height=msg.height, fx=fx, fy=fy, cx=cx, cy=cy)

    def get_depth_in_meters(self, depth_msg: Image) -> np.ndarray:
        enc = (depth_msg.encoding or "").lower()
        if enc == "32fc1":
            return self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="32FC1").astype(np.float32)
        if enc in ["16uc1", "mono16"]:
            d_mm = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="16UC1").astype(np.float32)
            return d_mm / 1000.0
        return self.bridge.imgmsg_to_cv2(depth_msg).astype(np.float32)

    def synced_cb(self, color_msg: Image, depth_msg: Image, pose_msg: PoseStamped):
        if self.o3d_intr is None:
            self.skipped_intr += 1
            return

        # Convert color
        try:
            color_bgr = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
        except Exception:
            return
        color_rgb = color_bgr[:, :, ::-1].copy()

        # Convert depth
        try:
            depth = self.get_depth_in_meters(depth_msg)
        except Exception:
            return

        depth = np.where(np.isfinite(depth) & (depth > 0.0) & (depth < self.depth_trunc), depth, 0.0).astype(np.float32)
        if np.count_nonzero(depth > 0.0) < 500:
            self.skipped_depth_empty += 1
            return

        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d.geometry.Image(color_rgb),
            o3d.geometry.Image(depth),
            depth_scale=1.0,
            depth_trunc=self.depth_trunc,
            convert_rgb_to_intensity=False
        )

        # Pose
        p = pose_msg.pose.position
        q = pose_msg.pose.orientation

        R_map_body = quat_to_rot(q.x, q.y, q.z, q.w)
        t_map_body = np.array([p.x, p.y, p.z], dtype=np.float64)

        T_map_body = np.eye(4, dtype=np.float64)
        T_map_body[:3, :3] = R_map_body
        T_map_body[:3, 3] = t_map_body

        T_body_cam = np.eye(4, dtype=np.float64)
        T_body_cam[:3, :3] = self.R_body_cam
        T_body_cam[:3, 3] = self.t_body_cam

        T_map_cam = T_map_body @ T_body_cam
        extrinsic = np.linalg.inv(T_map_cam) if self.use_inverse_extrinsic else T_map_cam

        try:
            self.volume.integrate(rgbd, self.o3d_intr, extrinsic)
        except Exception:
            return

        self.integrations += 1

        # Publicar por tiempo (evita cuelgue)
        now = self.get_clock().now()
        if (now - self.last_publish_time).nanoseconds < int(self.publish_period_sec * 1e9):
            return
        self.last_publish_time = now

        pcd = self.volume.extract_point_cloud()
        if len(pcd.points) == 0:
            self.get_logger().warn("TSDF: extract vacío. Prueba use_inverse_extrinsic:=False.", throttle_duration_sec=2.0)
            return

        pts = np.asarray(pcd.points, dtype=np.float32)
        cols = (np.asarray(pcd.colors, dtype=np.float32) * 255.0).clip(0, 255).astype(np.uint8) if pcd.has_colors() else np.zeros((pts.shape[0], 3), dtype=np.uint8)

        if pts.shape[0] > self.max_points_publish:
            idx = np.random.choice(pts.shape[0], self.max_points_publish, replace=False)
            pts = pts[idx]
            cols = cols[idx]

        out = xyzrgb_to_pointcloud2(pts, cols, self.global_frame, now.to_msg())
        self.pub_tsdf_pc.publish(out)

    def print_status(self):
        self.get_logger().info(
            f"[TSDF status] integrations={self.integrations} skipped_intr={self.skipped_intr} skipped_depth_empty={self.skipped_depth_empty}",
            throttle_duration_sec=0.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = TSDFStereoFusion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()