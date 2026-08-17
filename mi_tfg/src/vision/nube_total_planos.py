#!/usr/bin/env python3

import copy

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

import sensor_msgs_py.point_cloud2 as pc2

import open3d as o3d
import numpy as np


class PublicadorMapaGlobalTiempoReal(Node):
    def __init__(self):
        super().__init__("publicador_mapa_global_tiempo_real")

        # Parámetros ROS 2
        self.declare_parameter("voxel_size", 0.03)
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("usar_pose_mensaje", True)
        self.declare_parameter("flag_mostrar_planos", False)
        self.declare_parameter("proyectar_plano", True)
        self.declare_parameter("camara2cuerpo", [0.1, 0.03, 0.03])

        self.voxel_size = float(self.get_parameter("voxel_size").value)
        self.global_frame = str(self.get_parameter("global_frame").value)
        self.usar_pose_mensaje = bool(self.get_parameter("usar_pose_mensaje").value)
        self.flag_mostrar_planos = bool(self.get_parameter("flag_mostrar_planos").value)
        self.proyectar_plano = bool(self.get_parameter("proyectar_plano").value)
        self.camara2cuerpo = list(self.get_parameter("camara2cuerpo").value)

        # Transformación cámara -> cuerpo
        self.T_camara_cuerpo = np.array([
            [0,  0,  1,  self.camara2cuerpo[0]],
            [-1, 0,  0,  self.camara2cuerpo[2]],
            [0, -1,  0, -self.camara2cuerpo[1]],
            [0,  0,  0,  1]
        ], dtype=np.float64)

        # Publicador
        self.pub_global = self.create_publisher(
            PointCloud2,
            "nube_puntos/global",
            10
        )

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

        # Nube global acumulada
        self.global_cloud = o3d.geometry.PointCloud()

        # Última pose recibida
        self.ultima_pose = None
        self.ultimo_stamp_pose = None

        # Contadores
        self.num_nubes_integradas = 0

        # Timer para publicar periódicamente la nube reconstruida
        self.timer = self.create_timer(1.0, self.timer_cb)

        self.get_logger().info("Nodo de mapeado global en tiempo real iniciado")

    def pose_cb(self, msg: PoseStamped):
        self.ultima_pose = {
            "px": msg.pose.position.x,
            "py": msg.pose.position.y,
            "pz": msg.pose.position.z,
            "qx": msg.pose.orientation.x,
            "qy": msg.pose.orientation.y,
            "qz": msg.pose.orientation.z,
            "qw": msg.pose.orientation.w,
        }
        self.ultimo_stamp_pose = msg.header.stamp

    def cloud_cb(self, msg: PointCloud2):
        if self.ultima_pose is None:
            self.get_logger().warn(
                "Aún no se ha recibido ninguna pose. Se descarta la nube."
            )
            return

        pcd = self.pointcloud2_to_open3d(msg)

        if len(pcd.points) == 0:
            self.get_logger().warn("Nube recibida vacía")
            return

        pose = copy.deepcopy(self.ultima_pose)

        self.integracion_pc(pcd, pose)
        self.num_nubes_integradas += 1

        self.get_logger().info(
            f"Nube integrada #{self.num_nubes_integradas}. "
            f"Puntos globales: {len(self.global_cloud.points)}"
        )

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

    def pointcloud2_to_open3d(self, msg: PointCloud2):
        field_names = [field.name for field in msg.fields]
        tiene_rgb = "rgb" in field_names
        tiene_rgba = "rgba" in field_names

        points = []
        colors = []

        if tiene_rgb or tiene_rgba:
            rgb_field = "rgb" if tiene_rgb else "rgba"

            for p in pc2.read_points(
                msg,
                field_names=("x", "y", "z", rgb_field),
                skip_nans=True
            ):
                x, y, z, rgb = p
                points.append([x, y, z])

                rgb_uint32 = np.asarray([rgb]).view(np.uint32)[0]
                r = (rgb_uint32 >> 16) & 255
                g = (rgb_uint32 >> 8) & 255
                b = rgb_uint32 & 255
                colors.append([r / 255.0, g / 255.0, b / 255.0])
        else:
            for p in pc2.read_points(
                msg,
                field_names=("x", "y", "z"),
                skip_nans=True
            ):
                x, y, z = p
                points.append([x, y, z])

        pcd = o3d.geometry.PointCloud()

        if len(points) == 0:
            return pcd

        pcd.points = o3d.utility.Vector3dVector(
            np.asarray(points, dtype=np.float64)
        )

        if len(colors) == len(points):
            pcd.colors = o3d.utility.Vector3dVector(
                np.asarray(colors, dtype=np.float64)
            )

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

    ###############################################################################################################

    def _asegurar_colores(self, pcd, color=(0.7, 0.7, 0.7)):
        if len(pcd.points) == 0:
            return

        if len(pcd.colors) == 0:
            colores = np.tile(np.array(color, dtype=np.float64), (len(pcd.points), 1))
            pcd.colors = o3d.utility.Vector3dVector(colores)

    def _suavizar_histograma(self, hist):
        if len(hist) < 5:
            return hist.astype(np.float64)

        kernel = np.array([1, 2, 3, 2, 1], dtype=np.float64)
        kernel /= kernel.sum()
        return np.convolve(hist.astype(np.float64), kernel, mode="same")

    def _estimar_planos_dominantes_x(self, pcd):
        """
        Estima planos dominantes x = c sobre la subnube.
        Devuelve una lista de diccionarios:
            [
                {
                    "x_plano": float,
                    "mask": np.ndarray(bool)
                },
                ...
            ]
        donde mask indica qué puntos pertenecen a ese plano.
        """
        puntos = np.asarray(pcd.points)
        if puntos.shape[0] < 30:
            return []

        x = puntos[:, 0]

        x_p2 = np.percentile(x, 2.0)
        x_p98 = np.percentile(x, 98.0)

        if abs(x_p98 - x_p2) < 1e-6:
            x_med = float(np.median(x))
            tol = max(1.5 * self.voxel_size, 0.02)
            mask = np.abs(x - x_med) <= tol
            return [{"x_plano": x_med, "mask": mask}]

        n = len(x)
        num_bins = int(np.clip(np.sqrt(n), 30, 120))
        hist, edges = np.histogram(x, bins=num_bins, range=(x_p2, x_p98))
        hist_s = self._suavizar_histograma(hist)

        if hist_s.max() <= 0:
            return []

        candidatos = []
        umbral_relativo = 0.20 * hist_s.max()

        for i in range(1, len(hist_s) - 1):
            if hist_s[i] >= hist_s[i - 1] and hist_s[i] >= hist_s[i + 1]:
                if hist_s[i] >= umbral_relativo:
                    x_centro = 0.5 * (edges[i] + edges[i + 1])
                    candidatos.append((x_centro, hist_s[i]))

        if len(candidatos) == 0:
            x_med = float(np.median(x))
            tol = max(1.5 * self.voxel_size, 0.02)
            mask = np.abs(x - x_med) <= tol
            return [{"x_plano": x_med, "mask": mask}]

        candidatos.sort(key=lambda t: t[1], reverse=True)

        ancho_bin = edges[1] - edges[0]
        separacion_min = max(2.5 * ancho_bin, 1.5 * self.voxel_size)
        tol_asignacion = max(1.5 * ancho_bin, 1.5 * self.voxel_size, 0.02)

        planos_x = []
        for x_centro, _ in candidatos:
            if all(abs(x_centro - xp) > separacion_min for xp in planos_x):
                planos_x.append(float(x_centro))

        planos_x = sorted(planos_x[:6])

        resultados = []
        for xp in planos_x:
            mask = np.abs(x - xp) <= tol_asignacion

            if np.count_nonzero(mask) < 20:
                continue

            x_refinado = float(np.median(x[mask]))

            resultados.append({
                "x_plano": x_refinado,
                "mask": mask
            })

        return resultados

    def _crear_nube_visualizacion_planos(self, pcd, planos_info):
        """
        Crea una nube roja solo donde realmente hay puntos soporte del plano.
        """
        if len(planos_info) == 0 or len(pcd.points) == 0:
            return o3d.geometry.PointCloud()

        puntos = np.asarray(pcd.points)
        todos_pts = []
        todos_col = []
        rojo = np.array([1.0, 0.0, 0.0], dtype=np.float64)

        for plano in planos_info:
            mask = plano["mask"]
            if mask is None or np.count_nonzero(mask) == 0:
                continue

            pts_sel = puntos[mask].copy()
            pts_sel[:, 0] = plano["x_plano"]

            todos_pts.append(pts_sel)
            todos_col.append(np.tile(rojo, (pts_sel.shape[0], 1)))

        if len(todos_pts) == 0:
            return o3d.geometry.PointCloud()

        nube_planos = o3d.geometry.PointCloud()
        nube_planos.points = o3d.utility.Vector3dVector(np.vstack(todos_pts))
        nube_planos.colors = o3d.utility.Vector3dVector(np.vstack(todos_col))
        return nube_planos

    def _proyectar_puntos_a_planos_en_x(self, pcd, planos_info):
        """
        Proyecta puntos sobre planos dominantes usando ocupación real en Y-Z.

        Reglas:
        - No se usa clustering.
        - Cada plano ocupa solo las celdas Y-Z donde realmente tiene puntos soporte.
        - Un punto se proyecta solo si su celda Y-Z pertenece a algún plano.
        - Si cae en varios planos, se elige el de x más cercana.
        - Si no cae en ningún plano, se elimina.
        """
        if len(planos_info) == 0 or len(pcd.points) == 0:
            pcd.clear()
            return

        puntos = np.asarray(pcd.points).copy()

        if len(pcd.colors) == len(puntos):
            colores = np.asarray(pcd.colors).copy()
        else:
            colores = None

        paso_yz = max(self.voxel_size, 0.02)

        iy_puntos = np.floor(puntos[:, 1] / paso_yz).astype(np.int32)
        iz_puntos = np.floor(puntos[:, 2] / paso_yz).astype(np.int32)

        mapa_celdas = {}

        for plano in planos_info:
            mask = plano["mask"]
            if mask is None or np.count_nonzero(mask) == 0:
                continue

            pts_soporte = puntos[mask]
            iy = np.floor(pts_soporte[:, 1] / paso_yz).astype(np.int32)
            iz = np.floor(pts_soporte[:, 2] / paso_yz).astype(np.int32)

            celdas_unicas = np.unique(
                np.column_stack((iy, iz)),
                axis=0
            )

            x_plano = plano["x_plano"]
            for celda in celdas_unicas:
                key = (int(celda[0]), int(celda[1]))
                if key not in mapa_celdas:
                    mapa_celdas[key] = [x_plano]
                else:
                    mapa_celdas[key].append(x_plano)

        if len(mapa_celdas) == 0:
            pcd.clear()
            return

        x_nueva = puntos[:, 0].copy()
        keep_mask = np.zeros(len(puntos), dtype=bool)

        for i in range(len(puntos)):
            key = (int(iy_puntos[i]), int(iz_puntos[i]))
            candidatos_x = mapa_celdas.get(key)

            if not candidatos_x:
                continue

            x0 = puntos[i, 0]
            mejor_x = min(candidatos_x, key=lambda xp: abs(x0 - xp))
            x_nueva[i] = mejor_x
            keep_mask[i] = True

        if not np.any(keep_mask):
            pcd.clear()
            return

        puntos_finales = puntos[keep_mask].copy()
        puntos_finales[:, 0] = x_nueva[keep_mask]
        pcd.points = o3d.utility.Vector3dVector(puntos_finales)

        if colores is not None:
            pcd.colors = o3d.utility.Vector3dVector(colores[keep_mask])

    def integracion_pc(self, pcd, pose):
        # Aplicar cámara -> cuerpo
        pcd.transform(self.T_camara_cuerpo)

        # Asegurar que la subnube tenga color para no perder la visualización
        self._asegurar_colores(pcd, color=(0.7, 0.7, 0.7))
        self._asegurar_colores(self.global_cloud, color=(0.7, 0.7, 0.7))

        # ------------------------------------------------------------------
        # ESTIMAR PLANOS EN FRAME CUERPO (antes de cuerpo -> mapa)
        # ------------------------------------------------------------------
        planos_info = self._estimar_planos_dominantes_x(pcd)

        if len(planos_info) > 0:
            self.get_logger().info(
                "Planos dominantes detectados en subnube (frame cuerpo): "
                f"{['{:.3f}'.format(p['x_plano']) for p in planos_info]}"
            )
        else:
            self.get_logger().warn(
                "No se detectaron planos dominantes en esta subnube"
            )

        nube_planos = o3d.geometry.PointCloud()
        if self.flag_mostrar_planos and len(planos_info) > 0:
            nube_planos = self._crear_nube_visualizacion_planos(pcd, planos_info)

        if self.proyectar_plano and len(planos_info) > 0:
            self._proyectar_puntos_a_planos_en_x(pcd, planos_info)

        if len(pcd.points) == 0:
            self.get_logger().warn(
                "La nube quedó vacía tras la proyección a planos"
            )
            return

        # Transformación cuerpo -> mapa a partir de la pose
        T_cuerpo_mapa = np.eye(4, dtype=np.float64)
        T_cuerpo_mapa[:3, :3] = self.quat_to_rot(
            pose["qx"], pose["qy"], pose["qz"], pose["qw"]
        )
        T_cuerpo_mapa[:3, 3] = np.array(
            [pose["px"], pose["py"], pose["pz"]],
            dtype=np.float64
        )

        # Aplicar cuerpo -> mapa a la nube original
        pcd.transform(T_cuerpo_mapa)

        # Aplicar cuerpo -> mapa también a la nube de planos
        if len(nube_planos.points) > 0:
            nube_planos.transform(T_cuerpo_mapa)

        # Acumular nube original
        self.global_cloud += pcd

        # Acumular planos rojos solo si existen
        if len(nube_planos.points) > 0:
            self.global_cloud += nube_planos

        if len(self.global_cloud.points) == 0:
            self.get_logger().warn("No se pudo construir ninguna nube global")
            return

        # Voxelizado final tras cada integración
        self.global_cloud = self.global_cloud.voxel_down_sample(self.voxel_size)

    ###############################################################################################################

    def timer_cb(self):
        if len(self.global_cloud.points) == 0:
            return

        msg_out = self.open3d_to_pointcloud2(self.global_cloud)
        self.pub_global.publish(msg_out)

        self.get_logger().info(
            f"Publicando nube global con {len(self.global_cloud.points)} puntos"
        )


def main(args=None):
    rclpy.init(args=args)
    node = PublicadorMapaGlobalTiempoReal()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()