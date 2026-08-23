#!/usr/bin/env python3
import struct
import numpy as np
import open3d as o3d

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

import sensor_msgs_py.point_cloud2 as pc2


def quat_to_rot(qx, qy, qz, qw) -> np.ndarray:
    n = np.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if n == 0.0:
        return np.eye(3, dtype=np.float64)

    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n

    xx, yy, zz = qx*qx, qy*qy, qz*qz
    xy, xz, yz = qx*qy, qx*qz, qy*qz
    wx, wy, wz = qw*qx, qw*qy, qw*qz

    return np.array([
        [1.0 - 2.0*(yy + zz), 2.0*(xy - wz),       2.0*(xz + wy)],
        [2.0*(xy + wz),       1.0 - 2.0*(xx + zz), 2.0*(yz - wx)],
        [2.0*(xz - wy),       2.0*(yz + wx),       1.0 - 2.0*(xx + yy)]
    ], dtype=np.float64)


def pose_to_transform(pose_msg: PoseStamped) -> np.ndarray:
    p = pose_msg.pose.position
    q = pose_msg.pose.orientation

    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = quat_to_rot(q.x, q.y, q.z, q.w)
    T[:3, 3] = np.array([p.x, p.y, p.z], dtype=np.float64)
    return T


def xyzrgb_to_pointcloud2(pts_xyz: np.ndarray,
                          cols_rgb_uint8: np.ndarray,
                          frame_id: str,
                          stamp_msg) -> PointCloud2:
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


def unpack_rgb_field(rgb_value):
    if isinstance(rgb_value, float):
        rgb_uint = struct.unpack('I', struct.pack('f', rgb_value))[0]
    else:
        rgb_uint = int(rgb_value)

    r = (rgb_uint >> 16) & 255
    g = (rgb_uint >> 8) & 255
    b = rgb_uint & 255
    return r, g, b


def pointcloud2_to_open3d(msg: PointCloud2) -> o3d.geometry.PointCloud:
    cloud = o3d.geometry.PointCloud()

    field_names = [f.name for f in msg.fields]
    has_rgb = "rgb" in field_names

    pts = []
    cols = []

    try:
        if has_rgb:
            for p in pc2.read_points(msg, field_names=("x", "y", "z", "rgb"), skip_nans=True):
                x, y, z, rgb = p
                r, g, b = unpack_rgb_field(rgb)
                pts.append([x, y, z])
                cols.append([r / 255.0, g / 255.0, b / 255.0])
        else:
            for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                x, y, z = p
                pts.append([x, y, z])
                cols.append([1.0, 1.0, 1.0])
    except Exception:
        return cloud

    if len(pts) == 0:
        return cloud

    cloud.points = o3d.utility.Vector3dVector(np.asarray(pts, dtype=np.float64))
    cloud.colors = o3d.utility.Vector3dVector(np.asarray(cols, dtype=np.float64))
    return cloud


class GlobalICPMap(Node):
    def __init__(self):
        super().__init__("global_icp_map")

        self.declare_parameter("global_frame", "map")
        self.declare_parameter("voxel_size", 0.03)
        self.declare_parameter("icp_threshold", 0.20)
        self.declare_parameter("max_iteration", 200)
        self.declare_parameter("merge_voxel_size", 0.02)
        self.declare_parameter("publish_period_sec", 1.0)
        self.declare_parameter("max_points_publish", 150000)
        self.declare_parameter("min_points_cloud", 200)
        self.declare_parameter("use_gt_directly_if_icp_fails", True)
        self.declare_parameter("fitness_min_threshold", 0.05)

        # Traslación de la cámara respecto al dron
        self.declare_parameter("cam_in_body_xyz", [0.0, 0.0, 0.0])

        self.global_frame = self.get_parameter("global_frame").value
        self.voxel_size = float(self.get_parameter("voxel_size").value)
        self.icp_threshold = float(self.get_parameter("icp_threshold").value)
        self.max_iteration = int(self.get_parameter("max_iteration").value)
        self.merge_voxel_size = float(self.get_parameter("merge_voxel_size").value)
        self.publish_period_sec = float(self.get_parameter("publish_period_sec").value)
        self.max_points_publish = int(self.get_parameter("max_points_publish").value)
        self.min_points_cloud = int(self.get_parameter("min_points_cloud").value)
        self.use_gt_directly_if_icp_fails = bool(self.get_parameter("use_gt_directly_if_icp_fails").value)
        self.fitness_min_threshold = float(self.get_parameter("fitness_min_threshold").value)
        self.t_body_cam = np.array(self.get_parameter("cam_in_body_xyz").value, dtype=np.float64)

        self.global_cloud = o3d.geometry.PointCloud()
        self.latest_pose = None
        self.last_publish_time = self.get_clock().now()

        self.num_pose = 0
        self.num_cloud = 0
        self.num_integrated = 0
        self.num_icp_ok = 0
        self.num_icp_fail = 0
        self.num_empty = 0
        self.num_skipped_no_pose = 0

        self.sub_pose = self.create_subscription(
            PoseStamped,
            "/sensor/GT/pose",
            self.pose_cb,
            10
        )

        self.sub_cloud = self.create_subscription(
            PointCloud2,
            "/NubePuntos",
            self.cloud_cb,
            10
        )

        self.pub_global = self.create_publisher(
            PointCloud2,
            "/MapaGlobalICP",
            10
        )

        self.timer_dbg = self.create_timer(2.0, self.print_status)

        self.get_logger().info("GlobalICPMap listo.")

    def get_T_body_cam(self) -> np.ndarray:
        """
        Transformación fija cámara -> dron.

        Convención usada:
        x_cam -> -y_body
        y_cam -> -z_body
        z_cam ->  x_body
        """
        R_body_cam = np.array([
            [0.0,  0.0,  -1.0],
            [-1.0, 0.0,  0.0],
            [0.0, 1.0,  0.0]
        ], dtype=np.float64)

        T_body_cam = np.eye(4, dtype=np.float64)
        T_body_cam[:3, :3] = R_body_cam
        T_body_cam[:3, 3] = self.t_body_cam
        return T_body_cam

    def pose_cb(self, msg: PoseStamped):
        self.latest_pose = msg
        self.num_pose += 1

    def preprocess_cloud(self, cloud: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
        if len(cloud.points) == 0:
            return cloud

        proc = cloud.voxel_down_sample(self.voxel_size)
        if len(proc.points) == 0:
            return proc

        proc.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=self.voxel_size * 2.5,
                max_nn=30
            )
        )
        return proc

    def publish_global_cloud(self, stamp_msg):
        if len(self.global_cloud.points) == 0:
            return

        pts_all = np.asarray(self.global_cloud.points, dtype=np.float32)

        if self.global_cloud.has_colors():
            cols_all = (
                np.asarray(self.global_cloud.colors, dtype=np.float32) * 255.0
            ).clip(0, 255).astype(np.uint8)
        else:
            cols_all = np.full((pts_all.shape[0], 3), 255, dtype=np.uint8)

        if pts_all.shape[0] > self.max_points_publish:
            idx = np.random.choice(pts_all.shape[0], self.max_points_publish, replace=False)
            pts = pts_all[idx]
            cols = cols_all[idx]
        else:
            pts = pts_all
            cols = cols_all

        msg = xyzrgb_to_pointcloud2(pts, cols, self.global_frame, stamp_msg)
        self.pub_global.publish(msg)

    def fuse_cloud(self, new_cloud: o3d.geometry.PointCloud):
        self.global_cloud += new_cloud
        self.global_cloud = self.global_cloud.voxel_down_sample(self.merge_voxel_size)

    def cloud_cb(self, cloud_msg: PointCloud2):
        self.num_cloud += 1

        if self.latest_pose is None:
            self.num_skipped_no_pose += 1
            return

        source_raw = pointcloud2_to_open3d(cloud_msg)
        if len(source_raw.points) < self.min_points_cloud:
            self.num_empty += 1
            return

        T_map_body = pose_to_transform(self.latest_pose)
        T_body_cam = self.get_T_body_cam()
        T_init = T_map_body @ T_body_cam

        if len(self.global_cloud.points) == 0:
            first_cloud = o3d.geometry.PointCloud(source_raw)
            first_cloud.transform(T_init)
            self.global_cloud = first_cloud.voxel_down_sample(self.merge_voxel_size)
            self.num_integrated += 1

            self.get_logger().info(
                f"Primera nube integrada. puntos={len(self.global_cloud.points)}"
            )

            self.publish_global_cloud(cloud_msg.header.stamp)
            return

        source = self.preprocess_cloud(source_raw)
        target = self.preprocess_cloud(self.global_cloud)

        if len(source.points) == 0 or len(target.points) == 0:
            self.num_empty += 1
            return

        try:
            result = o3d.pipelines.registration.registration_colored_icp(
                source,
                target,
                self.icp_threshold,
                T_init,
                o3d.pipelines.registration.TransformationEstimationForColoredICP(),
                o3d.pipelines.registration.ICPConvergenceCriteria(
                    relative_fitness=1e-6,
                    relative_rmse=1e-6,
                    max_iteration=self.max_iteration
                )
            )
        except Exception as e:
            self.num_icp_fail += 1
            self.get_logger().warn(f"ICP excepción: {e}")

            if self.use_gt_directly_if_icp_fails:
                aligned_gt = o3d.geometry.PointCloud(source_raw)
                aligned_gt.transform(T_init)
                self.fuse_cloud(aligned_gt)
                self.num_integrated += 1
            return

        use_icp = np.isfinite(result.fitness) and (result.fitness >= self.fitness_min_threshold)

        if use_icp:
            aligned = o3d.geometry.PointCloud(source_raw)
            aligned.transform(result.transformation)
            self.fuse_cloud(aligned)
            self.num_integrated += 1
            self.num_icp_ok += 1

            self.get_logger().info(
                f"ICP ok | fitness={result.fitness:.4f} rmse={result.inlier_rmse:.4f} "
                f"| mapa={len(self.global_cloud.points)}"
            )
        else:
            self.num_icp_fail += 1

            if self.use_gt_directly_if_icp_fails:
                aligned_gt = o3d.geometry.PointCloud(source_raw)
                aligned_gt.transform(T_init)
                self.fuse_cloud(aligned_gt)
                self.num_integrated += 1

                self.get_logger().warn(
                    f"ICP pobre (fitness={result.fitness:.4f}). "
                    f"Usando T_init de GT+extrínseca. mapa={len(self.global_cloud.points)}"
                )

        now = self.get_clock().now()
        if (now - self.last_publish_time).nanoseconds >= int(self.publish_period_sec * 1e9):
            self.last_publish_time = now
            self.publish_global_cloud(cloud_msg.header.stamp)

    def print_status(self):
        self.get_logger().info(
            f"[ICP status] poses={self.num_pose} nubes={self.num_cloud} "
            f"integradas={self.num_integrated} icp_ok={self.num_icp_ok} "
            f"icp_fail={self.num_icp_fail} vacias={self.num_empty} "
            f"sin_pose={self.num_skipped_no_pose} mapa={len(self.global_cloud.points)}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = GlobalICPMap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()