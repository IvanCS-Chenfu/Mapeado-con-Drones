#!/usr/bin/env python3

import os
import csv

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header

import open3d as o3d
import numpy as np


class PublicadorMapaGlobalDesdeArchivos(Node):
    def __init__(self):
        super().__init__("publicador_mapa_global_desde_archivos")

        # Parámetros ROS 2
        self.declare_parameter("numero_dron", 2)
        self.declare_parameter("it_min", 1)
        self.declare_parameter("it_max", 188)

        self.numero_dron = self.get_parameter("numero_dron").value
        self.it_min_param = self.get_parameter("it_min").value
        self.it_max_param = self.get_parameter("it_max").value

        # Parámetros internos
        self.global_frame = "map"
        self.voxel_size = 0.03
        self.carpeta_nubes = "./nubes_puntos"
        self.csv_path = os.path.join(self.carpeta_nubes, "poses.csv")
        self.camara2cuerpo = [0.1, 0.03, 0.03]

        # Flags de comportamiento
        self.flag_mostrar_planos = False
        self.proyectar_plano = True

        # ICP
        self.usar_icp = True
        self.icp_max_correspondence_distance = 0.30
        self.icp_fitness_min = 0.0         # Obligar ICP (0.12)
        self.icp_rmse_max = 0.12
        self.icp_voxel_size = 0.03
        self.icp_max_translation_correction = 9999.0  # Obligar ICP (0.60)

        # Estado secuencial
        self.prev_cloud_local = None
        self.prev_T_cuerpo_mapa = None

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

        # Nube global acumulada
        self.global_cloud = o3d.geometry.PointCloud()

        # Timer para publicar periódicamente la nube reconstruida
        self.timer = self.create_timer(1.0, self.timer_cb)

        # Construir nube global una sola vez al arrancar
        self.reconstruir_mapa_global()

        self.get_logger().info(
            f"Nodo iniciado. numero_dron={self.numero_dron}, "
            f"it_min={self.it_min_param}, it_max={self.it_max_param}"
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

    def pose_a_transformada(self, pose):
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = self.quat_to_rot(
            pose["qx"], pose["qy"], pose["qz"], pose["qw"]
        )
        T[:3, 3] = np.array(
            [pose["px"], pose["py"], pose["pz"]],
            dtype=np.float64
        )
        return T

    def open3d_to_pointcloud2(self, pcd):
        points = np.asarray(pcd.points, dtype=np.float32)

        if len(points) == 0:
            colors = np.zeros((0, 3), dtype=np.uint32)
        else:
            if len(pcd.colors) == 0:
                colors = np.ones((len(points), 3), dtype=np.uint32) * 255
            else:
                colors = np.clip(
                    (np.asarray(pcd.colors) * 255.0), 0, 255
                ).astype(np.uint32)

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

    def leer_poses_csv(self):
        poses = []

        if not os.path.exists(self.csv_path):
            self.get_logger().error(f"No existe el CSV: {self.csv_path}")
            return poses

        with open(self.csv_path, mode="r", newline="") as f:
            reader = csv.DictReader(f)

            for row in reader:
                try:
                    pose = {
                        "numero_dron": int(row["numero_dron"]),
                        "iteracion": int(row["iteracion"]),
                        "px": float(row["px"]),
                        "py": float(row["py"]),
                        "pz": float(row["pz"]),
                        "qx": float(row["qx"]),
                        "qy": float(row["qy"]),
                        "qz": float(row["qz"]),
                        "qw": float(row["qw"]),
                    }
                    poses.append(pose)
                except Exception as e:
                    self.get_logger().warn(
                        f"Fila inválida en CSV: {row}. Error: {e}"
                    )

        return poses

    def _asegurar_colores(self, pcd, color=(0.7, 0.7, 0.7)):
        if len(pcd.points) == 0:
            return

        if len(pcd.colors) == 0:
            colores = np.tile(
                np.array(color, dtype=np.float64),
                (len(pcd.points), 1)
            )
            pcd.colors = o3d.utility.Vector3dVector(colores)

    def _suavizar_histograma(self, hist):
        if len(hist) < 5:
            return hist.astype(np.float64)

        kernel = np.array([1, 2, 3, 2, 1], dtype=np.float64)
        kernel /= kernel.sum()
        return np.convolve(hist.astype(np.float64), kernel, mode="same")

    def _estimar_planos_dominantes_x(self, pcd):
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

    def preparar_nube_para_icp(self, pcd):
        pcd_out = o3d.geometry.PointCloud(pcd)

        if len(pcd_out.points) == 0:
            return pcd_out

        pcd_out = pcd_out.voxel_down_sample(self.icp_voxel_size)

        if len(pcd_out.points) < 20:
            return pcd_out

        pcd_out.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=2.0 * self.icp_voxel_size,
                max_nn=30
            )
        )

        return pcd_out

    def icp_es_bueno(self, resultado_icp):
        f = resultado_icp.fitness
        r = resultado_icp.inlier_rmse

        if f >= 0.25 and r <= 0.12:
            return True
        if f >= 0.15 and r <= 0.08:
            return True
        if f >= 0.10 and r <= 0.05:
            return True

        return False

    def correccion_icp_razonable(self, T_corr):
        t = np.linalg.norm(T_corr[:3, 3])
        return t <= self.icp_max_translation_correction

    def refinar_transformada_con_icp(self, pcd_local_actual, pose_actual):
        """
        Devuelve:
            T_final, icp_usado, resultado_icp
        donde T_final transforma cuerpo -> mapa.
        """
        T_pose_actual = self.pose_a_transformada(pose_actual)

        if (not self.usar_icp or
            self.prev_cloud_local is None or
            self.prev_T_cuerpo_mapa is None):
            return T_pose_actual, False, None

        pcd_prev_mapa = o3d.geometry.PointCloud(self.prev_cloud_local)
        pcd_prev_mapa.transform(self.prev_T_cuerpo_mapa)

        pcd_actual_mapa = o3d.geometry.PointCloud(pcd_local_actual)
        pcd_actual_mapa.transform(T_pose_actual)

        target = self.preparar_nube_para_icp(pcd_prev_mapa)
        source = self.preparar_nube_para_icp(pcd_actual_mapa)

        if len(source.points) < 20 or len(target.points) < 20:
            self.get_logger().warn(
                "ICP omitido: muy pocos puntos tras el preprocesado"
            )
            return T_pose_actual, False, None

        resultado_icp = o3d.pipelines.registration.registration_icp(
            source,
            target,
            self.icp_max_correspondence_distance,
            np.eye(4, dtype=np.float64),
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(
                relative_fitness=1e-6,
                relative_rmse=1e-6,
                max_iteration=50
            )
        )

        self.get_logger().info(
            f"ICP -> fitness={resultado_icp.fitness:.4f}, "
            f"rmse={resultado_icp.inlier_rmse:.4f}"
        )

        if (self.icp_es_bueno(resultado_icp) and
                self.correccion_icp_razonable(resultado_icp.transformation)):
            T_corr = resultado_icp.transformation
            T_final = T_corr @ T_pose_actual
            self.get_logger().info("ICP aceptado: se usa transformada refinada")
            return T_final, True, resultado_icp

        self.get_logger().warn(
            "ICP rechazado: se usa transformada de la pose"
        )
        return T_pose_actual, False, resultado_icp

    def integracion_pc(self, pcd, pose):
        # 1) Cámara -> cuerpo
        pcd.transform(self.T_camara_cuerpo)

        # Asegurar colores
        self._asegurar_colores(pcd, color=(0.7, 0.7, 0.7))
        self._asegurar_colores(self.global_cloud, color=(0.7, 0.7, 0.7))

        # 2) Detectar/proyectar planos en frame cuerpo
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
            self.get_logger().warn("La subnube quedó vacía tras la proyección")
            return

        # 3) Obtener transformada final usando ICP secuencial o pose
        T_final, icp_usado, _ = self.refinar_transformada_con_icp(pcd, pose)

        if icp_usado:
            self.get_logger().info("Integración con ICP")
        else:
            self.get_logger().info("Integración usando pose directa")

        # 4) Transformar nube actual a mapa
        pcd_mapa = o3d.geometry.PointCloud(pcd)
        pcd_mapa.transform(T_final)

        nube_planos_mapa = o3d.geometry.PointCloud()
        if len(nube_planos.points) > 0:
            nube_planos_mapa = o3d.geometry.PointCloud(nube_planos)
            nube_planos_mapa.transform(T_final)

        # 5) Acumular en la nube global
        self.global_cloud += pcd_mapa

        if len(nube_planos_mapa.points) > 0:
            self.global_cloud += nube_planos_mapa

        if len(self.global_cloud.points) == 0:
            self.get_logger().warn("No se pudo construir ninguna nube global")
            return

        self.global_cloud = self.global_cloud.voxel_down_sample(self.voxel_size)

        # 6) Guardar estado para la siguiente iteración
        self.prev_cloud_local = o3d.geometry.PointCloud(pcd)
        self.prev_T_cuerpo_mapa = T_final.copy()

    def reconstruir_mapa_global(self):
        poses = self.leer_poses_csv()

        if len(poses) == 0:
            self.get_logger().warn("No se encontraron poses en el CSV")
            return

        poses_dron = [
            pose for pose in poses
            if pose["numero_dron"] == self.numero_dron
        ]

        if len(poses_dron) == 0:
            self.get_logger().warn(
                f"No se encontraron poses para numero_dron={self.numero_dron}"
            )
            return

        iteraciones = [pose["iteracion"] for pose in poses_dron]
        iteracion_min_real = min(iteraciones)
        iteracion_max_real = max(iteraciones)

        it_min_ajustada = max(self.it_min_param, iteracion_min_real)
        it_max_ajustada = min(self.it_max_param, iteracion_max_real)

        if it_min_ajustada > it_max_ajustada:
            self.get_logger().warn(
                f"Rango inválido tras ajuste: it_min={it_min_ajustada}, "
                f"it_max={it_max_ajustada}. Se usará solo la iteración mínima válida."
            )
            it_min_ajustada = it_max_ajustada

        self.get_logger().info(
            f"numero_dron={self.numero_dron} | "
            f"iteracion_min_real={iteracion_min_real} | "
            f"iteracion_max_real={iteracion_max_real} | "
            f"it_min_usada={it_min_ajustada} | "
            f"it_max_usada={it_max_ajustada}"
        )

        poses_filtradas = [
            pose for pose in poses_dron
            if it_min_ajustada <= pose["iteracion"] <= it_max_ajustada
        ]

        if len(poses_filtradas) == 0:
            self.get_logger().warn("No hay poses dentro del rango solicitado")
            return

        poses_filtradas.sort(key=lambda x: x["iteracion"])

        self.global_cloud = o3d.geometry.PointCloud()
        self.prev_cloud_local = None
        self.prev_T_cuerpo_mapa = None
        nubes_cargadas = 0

        for pose in poses_filtradas:
            numero_dron = pose["numero_dron"]
            iteracion = pose["iteracion"]

            nombre_nube = f"pc{numero_dron}.{iteracion}.ply"
            ruta_nube = os.path.join(self.carpeta_nubes, nombre_nube)

            if not os.path.exists(ruta_nube):
                self.get_logger().warn(f"No existe la nube: {ruta_nube}")
                continue

            pcd = o3d.io.read_point_cloud(ruta_nube)

            if len(pcd.points) == 0:
                self.get_logger().warn(f"Nube vacía: {ruta_nube}")
                continue

            self.integracion_pc(pcd, pose)
            nubes_cargadas += 1

        if len(self.global_cloud.points) == 0:
            self.get_logger().warn("No se pudo construir ninguna nube global")
            return

        self.get_logger().info(
            f"Mapa global reconstruido con {nubes_cargadas} nubes, "
            f"numero_dron={self.numero_dron}, "
            f"rango_iteraciones=[{it_min_ajustada}, {it_max_ajustada}] y "
            f"{len(self.global_cloud.points)} puntos"
        )

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
    node = PublicadorMapaGlobalDesdeArchivos()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()