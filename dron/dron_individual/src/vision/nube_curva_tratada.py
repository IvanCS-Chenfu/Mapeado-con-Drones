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

        # Parámetros base
        self.global_frame = "map"
        self.voxel_size = 0.03
        self.camara2cuerpo = [0.1, 0.03, 0.03]

        # Preprocesado
        self.outlier_nb_neighbors = 20
        self.outlier_std_ratio = 2.0
        self.normal_radius = 0.10
        self.normal_max_nn = 30

        # DBSCAN
        self.dbscan_eps = 0.12
        self.dbscan_min_points = 30
        self.min_cluster_points = 60

        # Selección de puntos "buenos" por normales
        # Como X es la dirección cámara->puntos, pedimos |nx| alto
        self.normal_x_alignment_thr = 0.90

        # Separación de subplanos dentro del cluster por X
        self.x_split_gap = 0.12
        self.min_seed_group_points = 20

        # Ajuste de plano
        self.plane_dist_thresh = 0.04
        self.plane_ransac_n = 3
        self.plane_num_iter = 800

        self.get_logger().info("Nodo de corrección por frame iniciado")

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

    # ============================================================
    # UTILIDADES
    # ============================================================

    def preprocess_cloud(self, pcd):
        if len(pcd.points) == 0:
            return pcd

        pcd = pcd.voxel_down_sample(self.voxel_size)

        if len(pcd.points) < 10:
            return pcd

        pcd, _ = pcd.remove_statistical_outlier(
            nb_neighbors=self.outlier_nb_neighbors,
            std_ratio=self.outlier_std_ratio
        )

        if len(pcd.points) < 10:
            return pcd

        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=self.normal_radius,
                max_nn=self.normal_max_nn
            )
        )

        return pcd

    def split_cluster_by_normal_alignment_x(self, cluster_pcd):
        """
        Separa puntos del cluster en:
        - seed_idx: normales alineadas con el eje X (|nx| alto)
        - dev_idx: resto
        """
        if len(cluster_pcd.points) == 0 or len(cluster_pcd.normals) == 0:
            return [], []

        normals = np.asarray(cluster_pcd.normals, dtype=np.float64)
        norms = np.linalg.norm(normals, axis=1, keepdims=True) + 1e-12
        normals_unit = normals / norms

        x_alignment = np.abs(normals_unit[:, 0])

        seed_idx = np.where(x_alignment >= self.normal_x_alignment_thr)[0]
        dev_idx = np.where(x_alignment < self.normal_x_alignment_thr)[0]

        return seed_idx.tolist(), dev_idx.tolist()

    def split_seed_points_by_x(self, cluster_pcd, seed_idx):
        """
        Divide los seed points en grupos según saltos en X.
        X es la profundidad / separación perpendicular a la cámara.
        """
        if len(seed_idx) == 0:
            return []

        pts = np.asarray(cluster_pcd.points, dtype=np.float64)
        seed_idx_np = np.array(seed_idx, dtype=np.int32)

        seed_pts = pts[seed_idx_np]
        order = np.argsort(seed_pts[:, 0])  # ordenar por X

        sorted_idx = seed_idx_np[order]
        sorted_pts = seed_pts[order]

        groups = []
        current_group = [int(sorted_idx[0])]

        for i in range(1, len(sorted_idx)):
            x_prev = sorted_pts[i - 1, 0]
            x_curr = sorted_pts[i, 0]

            if abs(x_curr - x_prev) <= self.x_split_gap:
                current_group.append(int(sorted_idx[i]))
            else:
                groups.append(current_group)
                current_group = [int(sorted_idx[i])]

        groups.append(current_group)
        return groups

    def fit_plane_to_group(self, cluster_pcd, group_idx):
        """
        Ajusta un plano a un grupo de seed points.
        """
        if len(group_idx) < self.min_seed_group_points:
            return None, []

        group_cloud = cluster_pcd.select_by_index(group_idx)

        if len(group_cloud.points) < self.plane_ransac_n:
            return None, []

        plane_model, inliers = group_cloud.segment_plane(
            distance_threshold=self.plane_dist_thresh,
            ransac_n=self.plane_ransac_n,
            num_iterations=self.plane_num_iter
        )

        return plane_model, inliers

    def project_points_to_plane(self, points, plane_model):
        a, b, c, d = plane_model
        n = np.array([a, b, c], dtype=np.float64)
        n_norm = np.linalg.norm(n) + 1e-12
        n_unit = n / n_norm

        signed_dist = (points @ n + d) / n_norm
        projected = points - np.outer(signed_dist, n_unit)
        return projected

    def assign_deviated_points_to_groups_yz(self, cluster_pcd, dev_idx, valid_groups):
        """
        Asigna cada punto desviado al grupo más cercano en Y,Z
        ignorando X.
        """
        pts = np.asarray(cluster_pcd.points, dtype=np.float64)

        assignments = {}

        if len(valid_groups) == 0:
            return assignments

        group_yz_points = []
        for g in valid_groups:
            group_pts = pts[np.array(g, dtype=np.int32)]
            group_yz_points.append(group_pts[:, 1:3])  # Y,Z

        for idx in dev_idx:
            p_yz = pts[idx, 1:3]

            best_group = None
            best_dist = np.inf

            for gi, g_yz in enumerate(group_yz_points):
                dists = np.linalg.norm(g_yz - p_yz[None, :], axis=1)
                min_dist = np.min(dists)

                if min_dist < best_dist:
                    best_dist = min_dist
                    best_group = gi

            assignments[idx] = best_group

        return assignments

    def planarize_cluster_multi_plane(self, cluster_pcd):
        """
        Pipeline dentro de cada cluster:
        1) separar seed points por normales alineadas con X
        2) separar esos seeds en subgrupos por X
        3) ajustar un plano por subgrupo
        4) asignar puntos desviados al subplano más cercano en Y,Z
        5) proyectarlos al plano asignado
        """
        if len(cluster_pcd.points) == 0:
            return cluster_pcd, {
                "status": "empty_cluster",
                "seed_points": 0,
                "deviated_points": 0,
                "num_subplanes": 0
            }

        seed_idx, dev_idx = self.split_cluster_by_normal_alignment_x(cluster_pcd)

        if len(seed_idx) < self.min_seed_group_points:
            return cluster_pcd, {
                "status": "not_enough_seed_points",
                "seed_points": len(seed_idx),
                "deviated_points": len(dev_idx),
                "num_subplanes": 0
            }

        seed_groups = self.split_seed_points_by_x(cluster_pcd, seed_idx)

        valid_groups = []
        plane_models = []

        for g in seed_groups:
            if len(g) < self.min_seed_group_points:
                continue

            plane_model, _ = self.fit_plane_to_group(cluster_pcd, g)
            if plane_model is not None:
                valid_groups.append(g)
                plane_models.append(plane_model)

        if len(valid_groups) == 0:
            return cluster_pcd, {
                "status": "no_valid_subplanes",
                "seed_points": len(seed_idx),
                "deviated_points": len(dev_idx),
                "num_subplanes": 0
            }

        points = np.asarray(cluster_pcd.points, dtype=np.float64).copy()

        assignments = self.assign_deviated_points_to_groups_yz(
            cluster_pcd,
            dev_idx,
            valid_groups
        )

        for pt_idx, group_id in assignments.items():
            if group_id is None:
                continue

            plane_model = plane_models[group_id]
            p = points[pt_idx:pt_idx + 1]
            points[pt_idx:pt_idx + 1] = self.project_points_to_plane(p, plane_model)

        out = o3d.geometry.PointCloud()
        out.points = o3d.utility.Vector3dVector(points)

        if len(cluster_pcd.colors) > 0:
            out.colors = o3d.utility.Vector3dVector(np.asarray(cluster_pcd.colors).copy())

        return out, {
            "status": "ok",
            "seed_points": len(seed_idx),
            "deviated_points": len(dev_idx),
            "num_subplanes": len(valid_groups)
        }

    def clusterize_cloud(self, pcd):
        if len(pcd.points) == 0:
            return []

        labels = np.array(
            pcd.cluster_dbscan(
                eps=self.dbscan_eps,
                min_points=self.dbscan_min_points,
                print_progress=False
            )
        )

        if labels.size == 0:
            return []

        max_label = labels.max()
        if max_label < 0:
            return []

        clusters = []
        for lbl in range(max_label + 1):
            idx = np.where(labels == lbl)[0]
            if len(idx) < self.min_cluster_points:
                continue

            cluster = pcd.select_by_index(idx)
            clusters.append((lbl, idx, cluster))

        return clusters

    def process_frame_cloud(self, pcd_body):
        """
        Procesa la nube en frame cuerpo.
        """
        pcd_body = self.preprocess_cloud(pcd_body)

        if len(pcd_body.points) == 0:
            return pcd_body, {
                "clusters": 0,
                "processed": 0
            }

        clusters = self.clusterize_cloud(pcd_body)

        if len(clusters) == 0:
            return pcd_body, {
                "clusters": 0,
                "processed": 0
            }

        processed_clusters = []
        processed_count = 0

        for lbl, _, cluster in clusters:
            cluster_out, info = self.planarize_cluster_multi_plane(cluster)

            if info["status"] == "ok":
                processed_count += 1
                self.get_logger().info(
                    f"Cluster {lbl}: subplanos={info['num_subplanes']} | "
                    f"seed={info['seed_points']} | desviados={info['deviated_points']}"
                )
            else:
                self.get_logger().warn(
                    f"Cluster {lbl}: status={info['status']} | "
                    f"seed={info['seed_points']} | desviados={info['deviated_points']}"
                )

            processed_clusters.append(cluster_out)

        merged = processed_clusters[0]
        for c in processed_clusters[1:]:
            merged += c

        merged = merged.voxel_down_sample(self.voxel_size)

        return merged, {
            "clusters": len(clusters),
            "processed": processed_count
        }

    def transform_body_to_map(self, pcd_body, pose_msg):
        if len(pcd_body.points) == 0:
            return pcd_body

        pcd_map = o3d.geometry.PointCloud()
        pcd_map.points = o3d.utility.Vector3dVector(np.asarray(pcd_body.points).copy())

        if len(pcd_body.colors) > 0:
            pcd_map.colors = o3d.utility.Vector3dVector(np.asarray(pcd_body.colors).copy())

        p = pose_msg.pose.position
        q = pose_msg.pose.orientation

        T_cuerpo_mapa = np.eye(4, dtype=np.float64)
        T_cuerpo_mapa[:3, :3] = self.quat_to_rot(q.x, q.y, q.z, q.w)
        T_cuerpo_mapa[:3, 3] = np.array([p.x, p.y, p.z], dtype=np.float64)

        pcd_map.transform(T_cuerpo_mapa)
        return pcd_map

    def cloud_cb(self, cloud_msg):
        if self.latest_pose is None:
            self.get_logger().warn("Aún no se ha recibido ninguna pose")
            return

        pose_msg = self.latest_pose

        # PointCloud2 -> Open3D
        pcd = self.pointcloud2_to_open3d(cloud_msg)

        if len(pcd.points) == 0:
            self.get_logger().warn("Nube de puntos vacía")
            return

        # Cámara -> cuerpo
        T_camara_cuerpo = np.array([
            [0,  0,  1,  self.camara2cuerpo[0]],
            [-1, 0,  0,  self.camara2cuerpo[2]],
            [0, -1,  0, -self.camara2cuerpo[1]],
            [0,  0,  0,  1]
        ], dtype=np.float64)

        pcd.transform(T_camara_cuerpo)

        # Procesado por frame en cuerpo
        processed_body, stats = self.process_frame_cloud(pcd)

        if len(processed_body.points) == 0:
            self.get_logger().warn("Frame procesado vacío")
            return

        # Cuerpo -> mapa
        processed_map = self.transform_body_to_map(processed_body, pose_msg)

        # Publicar solo el frame corregido, sin acumulación global
        msg_out = self.open3d_to_pointcloud2(processed_map)
        self.pub_global.publish(msg_out)

        self.get_logger().info(
            f"Frame publicado con {len(processed_map.points)} puntos | "
            f"clusters={stats['clusters']} | procesados={stats['processed']}"
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