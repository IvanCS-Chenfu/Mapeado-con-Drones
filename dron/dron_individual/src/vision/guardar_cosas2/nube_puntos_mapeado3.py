#!/usr/bin/env python3

import copy
import numpy as np
import open3d as o3d

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

import sensor_msgs_py.point_cloud2 as pc2


class GlobalVoxelMap(Node):
    def __init__(self):
        super().__init__("global_voxel_map")

        # ============================================================
        # Subscriptores y publicadores
        # ============================================================
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

        # ============================================================
        # Estado
        # ============================================================
        self.latest_pose = None
        self.global_cloud = o3d.geometry.PointCloud()

        # ============================================================
        # Parámetros generales
        # ============================================================
        self.global_frame = "map"
        self.voxel_size = 0.03

        # ============================================================
        # Flags de métodos
        # ============================================================
        self.usar_dbscan = True
        self.usar_eliminacion_por_visibilidad = False

        # ============================================================
        # Parámetros DBSCAN
        # Nos quedamos SOLO con el cluster principal
        # ============================================================
        self.dbscan_eps_factor = 5.0
        self.dbscan_min_points = 5

        # ============================================================
        # Parámetros de visibilidad
        # Solo se eliminan puntos antiguos que estén dentro del cono
        # visible desde la cámara actual.
        # ============================================================
        self.cono_distancia_max = 8.0
        self.cono_fov_deg = 40.0

        # ============================================================
        # Transformación cámara -> cuerpo
        # Debe coincidir con la del primer nodo depth2pc()
        # ============================================================
        self.camara2cuerpo = [0.1, 0.03, 0.03]

    # ================================================================
    # Callbacks ROS
    # ================================================================
    def pose_cb(self, msg: PoseStamped):
        self.latest_pose = msg

    def cloud_cb(self, cloud_msg: PointCloud2):
        if self.latest_pose is None:
            self.get_logger().warn("Aún no hay pose disponible; nube ignorada.")
            return

        source_body = self.pointcloud2_to_open3d(cloud_msg)
        if len(source_body.points) == 0:
            self.get_logger().warn("Nube recibida vacía; se ignora.")
            return

        # Pose del cuerpo en mapa
        T_cuerpo_mapa = self.pose_msg_to_matrix(self.latest_pose)

        # Transformar nube actual a mapa
        source_map = copy.deepcopy(source_body)
        source_map.transform(T_cuerpo_mapa)

        # Filtrado DBSCAN sobre la observación actual
        if self.usar_dbscan:
            source_map = self.filtrar_cluster_principal_dbscan(source_map)
            if len(source_map.points) == 0:
                self.get_logger().warn("La nube actual quedó vacía tras DBSCAN.")
                return

        # Primer caso: mapa vacío
        if len(self.global_cloud.points) == 0:
            self.global_cloud = copy.deepcopy(source_map)
            self.get_logger().info(
                f"Mapa inicializado con {len(self.global_cloud.points)} puntos."
            )
        else:
            # 1) Eliminar puntos antiguos que deberían verse y ya no se ven
            if self.usar_eliminacion_por_visibilidad:
                n_antes = len(self.global_cloud.points)
                self.global_cloud = self.eliminar_puntos_no_observados(
                    self.global_cloud,
                    source_map,
                    T_cuerpo_mapa
                )
                n_despues = len(self.global_cloud.points)
                self.get_logger().info(
                    f"Eliminación por visibilidad: {n_antes - n_despues} puntos eliminados."
                )

            # 2) Fusionar por voxel, priorizando la observación actual
            n_antes = len(self.global_cloud.points)
            self.global_cloud = self.fusionar_por_voxeles(
                self.global_cloud,
                source_map
            )
            n_despues = len(self.global_cloud.points)
            self.get_logger().info(
                f"Fusión por voxel: {n_antes} -> {n_despues} puntos."
            )

            # 3) DBSCAN sobre el mapa global final
            if self.usar_dbscan:
                n_antes = len(self.global_cloud.points)
                self.global_cloud = self.filtrar_cluster_principal_dbscan(self.global_cloud)
                n_despues = len(self.global_cloud.points)
                self.get_logger().info(
                    f"Cluster principal DBSCAN: {n_antes - n_despues} puntos eliminados."
                )

        # Publicar mapa
        if len(self.global_cloud.points) > 0:
            msg = self.open3d_to_pointcloud2(self.global_cloud)
            self.pub_global.publish(msg)
            self.get_logger().info(
                f"Mapa global publicado con {len(self.global_cloud.points)} puntos."
            )
        else:
            self.get_logger().warn("El mapa global quedó vacío; no se publica.")

    # ================================================================
    # Conversiones
    # ================================================================
    def pointcloud2_to_open3d(self, msg: PointCloud2) -> o3d.geometry.PointCloud:
        data = list(pc2.read_points(msg, skip_nans=True))
        pcd = o3d.geometry.PointCloud()

        if len(data) == 0:
            return pcd

        points = np.array([[p[0], p[1], p[2]] for p in data], dtype=np.float64)
        pcd.points = o3d.utility.Vector3dVector(points)

        colors = []
        for p in data:
            rgb = int(p[3])
            r = (rgb >> 16) & 255
            g = (rgb >> 8) & 255
            b = rgb & 255
            colors.append([r / 255.0, g / 255.0, b / 255.0])

        if len(colors) == len(points):
            pcd.colors = o3d.utility.Vector3dVector(
                np.array(colors, dtype=np.float64)
            )

        return pcd

    def open3d_to_pointcloud2(self, pcd: o3d.geometry.PointCloud) -> PointCloud2:
        points = np.asarray(pcd.points, dtype=np.float32)

        if len(points) == 0:
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = self.global_frame

            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
            ]
            return pc2.create_cloud(header, fields, [])

        if len(pcd.colors) == len(pcd.points):
            colors = (np.asarray(pcd.colors) * 255).astype(np.uint32)
        else:
            colors = np.zeros((len(points), 3), dtype=np.uint32)

        rgb = (colors[:, 0] << 16) | (colors[:, 1] << 8) | colors[:, 2]
        cloud_data = np.column_stack((points, rgb)).tolist()

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.global_frame

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
        ]

        return pc2.create_cloud(header, fields, cloud_data)

    # ================================================================
    # Geometría / transformaciones
    # ================================================================
    def quat_to_rot(self, qx, qy, qz, qw):
        n = np.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
        if n == 0.0:
            return np.eye(3, dtype=np.float64)

        qx, qy, qz, qw = qx/n, qy/n, qz/n, qw/n

        xx, yy, zz = qx*qx, qy*qy, qz*qz
        xy, xz, yz = qx*qy, qx*qz, qy*qz
        wx, wy, wz = qw*qx, qw*qy, qw*qz

        R = np.array([
            [1 - 2*(yy + zz),     2*(xy - wz),     2*(xz + wy)],
            [    2*(xy + wz), 1 - 2*(xx + zz),     2*(yz - wx)],
            [    2*(xz - wy),     2*(yz + wx), 1 - 2*(xx + yy)]
        ], dtype=np.float64)

        return R

    def pose_msg_to_matrix(self, pose_msg: PoseStamped) -> np.ndarray:
        p = pose_msg.pose.position
        q = pose_msg.pose.orientation

        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = self.quat_to_rot(q.x, q.y, q.z, q.w)
        T[:3, 3] = np.array([p.x, p.y, p.z], dtype=np.float64)
        return T

    def get_T_camara_cuerpo(self) -> np.ndarray:
        tx, ty, tz = self.camara2cuerpo

        T = np.array([
            [ 0.0,  0.0,  1.0,  tx],
            [-1.0,  0.0,  0.0,  tz],
            [ 0.0, -1.0,  0.0, -ty],
            [ 0.0,  0.0,  0.0, 1.0]
        ], dtype=np.float64)

        return T

    # ================================================================
    # Utilidades voxel
    # ================================================================
    def puntos_a_voxeles(self, points: np.ndarray) -> np.ndarray:
        if len(points) == 0:
            return np.empty((0, 3), dtype=np.int32)

        return np.floor(points / self.voxel_size).astype(np.int32)

    def nube_a_diccionario_voxeles(self, pcd: o3d.geometry.PointCloud):
        points = np.asarray(pcd.points)
        has_colors = len(pcd.colors) == len(pcd.points)

        if has_colors:
            colors = np.asarray(pcd.colors)
        else:
            colors = np.zeros((len(points), 3), dtype=np.float64)

        vox = self.puntos_a_voxeles(points)

        vox_dict = {}
        for i in range(len(points)):
            key = tuple(vox[i])
            vox_dict[key] = (points[i], colors[i])

        return vox_dict

    def construir_nube_desde_diccionario(self, vox_dict) -> o3d.geometry.PointCloud:
        pcd = o3d.geometry.PointCloud()

        if len(vox_dict) == 0:
            return pcd

        points = []
        colors = []

        for _, value in vox_dict.items():
            p, c = value
            points.append(p)
            colors.append(c)

        pcd.points = o3d.utility.Vector3dVector(np.asarray(points, dtype=np.float64))
        pcd.colors = o3d.utility.Vector3dVector(np.asarray(colors, dtype=np.float64))
        return pcd

    # ================================================================
    # DBSCAN: quedarse solo con el grupo principal
    # ================================================================
    def filtrar_cluster_principal_dbscan(self, pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
        if len(pcd.points) == 0:
            return pcd

        eps = self.voxel_size * self.dbscan_eps_factor

        labels = np.array(
            pcd.cluster_dbscan(
                eps=eps,
                min_points=self.dbscan_min_points,
                print_progress=False
            )
        )

        # Todos ruido
        labels_validos = labels[labels != -1]
        if len(labels_validos) == 0:
            self.get_logger().warn(
                f"DBSCAN no encontró clusters válidos (eps={eps:.3f}, min_points={self.dbscan_min_points})."
            )
            return o3d.geometry.PointCloud()

        # Cluster principal = etiqueta con mayor número de puntos
        etiquetas, conteos = np.unique(labels_validos, return_counts=True)
        etiqueta_principal = etiquetas[np.argmax(conteos)]

        idx_cluster_principal = np.where(labels == etiqueta_principal)[0]

        self.get_logger().info(
            f"DBSCAN: cluster principal={etiqueta_principal}, "
            f"puntos_cluster={len(idx_cluster_principal)}, "
            f"puntos_totales={len(labels)}"
        )

        return pcd.select_by_index(idx_cluster_principal)

    # ================================================================
    # Cono de visibilidad
    # ================================================================
    def mascara_puntos_en_cono(
        self,
        points: np.ndarray,
        cam_pos: np.ndarray,
        cam_axis: np.ndarray,
        max_range: float,
        fov_deg: float
    ) -> np.ndarray:
        if len(points) == 0:
            return np.zeros((0,), dtype=bool)

        v = points - cam_pos.reshape(1, 3)
        dist = np.linalg.norm(v, axis=1)

        valid_dist = dist > 1e-9
        mask = np.zeros(len(points), dtype=bool)

        if not np.any(valid_dist):
            return mask

        v_unit = np.zeros_like(v)
        v_unit[valid_dist] = v[valid_dist] / dist[valid_dist][:, None]

        cos_half_fov = np.cos(np.deg2rad(fov_deg * 0.5))
        cosang = v_unit @ cam_axis

        mask = (dist <= max_range) & (cosang >= cos_half_fov)
        return mask

    def eliminar_puntos_no_observados(
        self,
        global_cloud: o3d.geometry.PointCloud,
        source_map: o3d.geometry.PointCloud,
        T_cuerpo_mapa: np.ndarray
    ) -> o3d.geometry.PointCloud:
        if len(global_cloud.points) == 0:
            return global_cloud

        if len(source_map.points) == 0:
            return global_cloud

        T_camara_cuerpo = self.get_T_camara_cuerpo()
        T_camara_mapa = T_cuerpo_mapa @ T_camara_cuerpo

        cam_pos = T_camara_mapa[:3, 3]

        cam_axis = T_camara_mapa[:3, :3] @ np.array([0.0, 0.0, 1.0], dtype=np.float64)
        norm_axis = np.linalg.norm(cam_axis)
        if norm_axis < 1e-9:
            self.get_logger().warn("El eje de la cámara es degenerado; no se elimina por visibilidad.")
            return global_cloud
        cam_axis = cam_axis / norm_axis

        global_points = np.asarray(global_cloud.points)
        global_colors = np.asarray(global_cloud.colors) if len(global_cloud.colors) == len(global_cloud.points) \
            else np.zeros((len(global_points), 3), dtype=np.float64)

        mask_cono = self.mascara_puntos_en_cono(
            global_points,
            cam_pos,
            cam_axis,
            self.cono_distancia_max,
            self.cono_fov_deg
        )

        source_points = np.asarray(source_map.points)
        source_vox = self.puntos_a_voxeles(source_points)
        source_vox_set = {tuple(v) for v in source_vox}

        global_vox = self.puntos_a_voxeles(global_points)

        mask_keep = np.ones(len(global_points), dtype=bool)

        for i in range(len(global_points)):
            if mask_cono[i]:
                if tuple(global_vox[i]) not in source_vox_set:
                    mask_keep[i] = False

        idx_keep = np.where(mask_keep)[0]

        new_cloud = o3d.geometry.PointCloud()
        if len(idx_keep) > 0:
            new_cloud.points = o3d.utility.Vector3dVector(global_points[idx_keep])
            new_cloud.colors = o3d.utility.Vector3dVector(global_colors[idx_keep])

        return new_cloud

    # ================================================================
    # Fusión sin sumar directamente
    # ================================================================
    def fusionar_por_voxeles(
        self,
        global_cloud: o3d.geometry.PointCloud,
        source_map: o3d.geometry.PointCloud
    ) -> o3d.geometry.PointCloud:
        global_dict = self.nube_a_diccionario_voxeles(global_cloud)
        source_dict = self.nube_a_diccionario_voxeles(source_map)

        # La observación nueva sobrescribe voxeles existentes
        global_dict.update(source_dict)

        return self.construir_nube_desde_diccionario(global_dict)


def main(args=None):
    rclpy.init(args=args)

    node = GlobalVoxelMap()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()