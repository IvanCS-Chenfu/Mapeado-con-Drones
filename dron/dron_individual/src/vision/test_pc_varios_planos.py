#!/usr/bin/env python3

import os
import csv
import math

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

        # Flags
        self.flag_mostrar_planos = False
        self.proyectar_plano = True

        # RANSAC para planos verticales
        self.distancia_ransac_plano = 0.03
        self.ransac_n = 3
        self.num_iter_ransac = 600
        self.max_planos_verticales = 6
        self.min_inliers_plano = 50
        self.max_inclinacion_vertical_deg = 20.0   # cuánto puede desviarse de vertical el plano
        self.umbral_familia_deg = 12.0             # agrupación angular de direcciones
        self.voxel_direccion = 0.04                # downsample opcional para estimar direcciones

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

        # Estado entre iteraciones
        self.pending_cloud = None
        self.pending_planes_vis = None
        self.pending_pose = None
        self.pending_direction = None

        self.timer = self.create_timer(1.0, self.timer_cb)

        self.reconstruir_mapa_global()

        self.get_logger().info(
            f"Nodo iniciado. numero_dron={self.numero_dron}, "
            f"it_min={self.it_min_param}, it_max={self.it_max_param}"
        )

    # -------------------------------------------------------------------------
    # Utilidades básicas
    # -------------------------------------------------------------------------

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

    def _copiar_nube(self, pcd):
        nueva = o3d.geometry.PointCloud()
        if len(pcd.points) > 0:
            nueva.points = o3d.utility.Vector3dVector(np.asarray(pcd.points).copy())
        if len(pcd.colors) > 0:
            nueva.colors = o3d.utility.Vector3dVector(np.asarray(pcd.colors).copy())
        if len(pcd.normals) > 0:
            nueva.normals = o3d.utility.Vector3dVector(np.asarray(pcd.normals).copy())
        return nueva

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

    # -------------------------------------------------------------------------
    # Planos x=c y proyección local
    # -------------------------------------------------------------------------

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

    # -------------------------------------------------------------------------
    # Dirección dominante usando varios planos verticales
    # -------------------------------------------------------------------------

    def _canonizar_direccion(self, d):
        d = np.asarray(d, dtype=np.float64)
        n = np.linalg.norm(d)
        if n < 1e-9:
            return None
        d = d / n

        if abs(d[0]) > 1e-9:
            if d[0] < 0:
                d = -d
        else:
            if d[1] < 0:
                d = -d
        return d

    def _alinear_signo_con_referencia(self, d, ref):
        d = np.asarray(d, dtype=np.float64)
        ref = np.asarray(ref, dtype=np.float64)

        nd = np.linalg.norm(d)
        nr = np.linalg.norm(ref)
        if nd < 1e-9 or nr < 1e-9:
            return self._canonizar_direccion(d)

        d = d / nd
        ref = ref / nr

        if np.dot(d, ref) < 0.0:
            d = -d
        return d

    def _angulo_yaw_entre_direcciones(self, d_from, d_to):
        a = np.asarray(d_from, dtype=np.float64)
        b = np.asarray(d_to, dtype=np.float64)

        a /= max(np.linalg.norm(a), 1e-12)
        b /= max(np.linalg.norm(b), 1e-12)

        ang_a = math.atan2(a[1], a[0])
        ang_b = math.atan2(b[1], b[0])
        d = ang_b - ang_a

        while d > math.pi:
            d -= 2.0 * math.pi
        while d < -math.pi:
            d += 2.0 * math.pi

        return d

    def _matriz_rotacion_z(self, yaw):
        c = math.cos(yaw)
        s = math.sin(yaw)
        return np.array([
            [c, -s, 0.0],
            [s,  c, 0.0],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)

    def _rotar_nube_en_frame_cuerpo(self, pcd, yaw):
        if pcd is None or len(pcd.points) == 0:
            return
        R = self._matriz_rotacion_z(yaw)
        pcd.rotate(R, center=(0.0, 0.0, 0.0))

    def _es_plano_vertical(self, normal):
        """
        Un plano vertical tiene una normal casi horizontal.
        Evaluamos que |nz| sea pequeño.
        """
        normal = np.asarray(normal, dtype=np.float64)
        n = np.linalg.norm(normal)
        if n < 1e-9:
            return False

        normal = normal / n
        nz = abs(normal[2])

        angulo_con_horizontal = math.degrees(math.asin(min(1.0, nz)))
        return angulo_con_horizontal <= self.max_inclinacion_vertical_deg

    def _extraer_planos_verticales_candidatos(self, pcd):
        """
        Extrae varios planos verticales candidatos por RANSAC iterativo.
        Devuelve lista de dicts:
        [
            {
                "normal_xy": np.ndarray(2,),
                "num_inliers": int,
                "plane_model": [a,b,c,d]
            },
            ...
        ]
        """
        resultados = []

        if pcd is None or len(pcd.points) < self.min_inliers_plano:
            return resultados

        pcd_work = self._copiar_nube(pcd)
        if self.voxel_direccion > 0.0:
            pcd_work = pcd_work.voxel_down_sample(self.voxel_direccion)

        if len(pcd_work.points) < self.min_inliers_plano:
            return resultados

        for _ in range(self.max_planos_verticales):
            if len(pcd_work.points) < self.min_inliers_plano:
                break

            try:
                plane_model, inliers = pcd_work.segment_plane(
                    distance_threshold=self.distancia_ransac_plano,
                    ransac_n=self.ransac_n,
                    num_iterations=self.num_iter_ransac
                )
            except Exception:
                break

            if len(inliers) < self.min_inliers_plano:
                break

            a, b, c, d = plane_model
            normal = np.array([a, b, c], dtype=np.float64)
            nn = np.linalg.norm(normal)
            if nn < 1e-9:
                break
            normal /= nn

            if self._es_plano_vertical(normal):
                dir_xy = np.array([normal[0], normal[1]], dtype=np.float64)
                nd = np.linalg.norm(dir_xy)
                if nd > 1e-9:
                    dir_xy /= nd
                    dir_xy = self._canonizar_direccion(dir_xy)

                    resultados.append({
                        "normal_xy": dir_xy,
                        "num_inliers": int(len(inliers)),
                        "plane_model": plane_model
                    })

            pcd_work = pcd_work.select_by_index(inliers, invert=True)

        return resultados

    def _angulo_abs_entre_direcciones(self, d1, d2):
        """
        Ángulo absoluto mínimo entre direcciones 2D, ignorando el signo.
        Resultado en radianes entre 0 y pi/2.
        """
        d1 = self._canonizar_direccion(d1)
        d2 = self._canonizar_direccion(d2)
        if d1 is None or d2 is None:
            return math.pi / 2.0

        dot = abs(float(np.dot(d1, d2)))
        dot = min(1.0, max(-1.0, dot))
        return math.acos(dot)

    def _agrupar_direcciones_en_familias(self, candidatos):
        """
        Agrupa direcciones similares en familias.
        Cada familia acumula soporte ponderado por inliers.
        """
        familias = []
        umbral = math.radians(self.umbral_familia_deg)

        for cand in candidatos:
            d = cand["normal_xy"]
            peso = cand["num_inliers"]

            asignada = False
            for fam in familias:
                ang = self._angulo_abs_entre_direcciones(d, fam["direccion_media"])
                if ang <= umbral:
                    d_alineada = self._alinear_signo_con_referencia(d, fam["direccion_media"])
                    fam["vectores"].append(d_alineada)
                    fam["pesos"].append(peso)

                    v = np.zeros(2, dtype=np.float64)
                    for vi, wi in zip(fam["vectores"], fam["pesos"]):
                        v += wi * vi
                    nv = np.linalg.norm(v)
                    if nv > 1e-9:
                        fam["direccion_media"] = v / nv
                    fam["peso_total"] += peso
                    fam["num_planos"] += 1
                    asignada = True
                    break

            if not asignada:
                familias.append({
                    "direccion_media": d.copy(),
                    "vectores": [d.copy()],
                    "pesos": [peso],
                    "peso_total": peso,
                    "num_planos": 1
                })

        return familias

    def _seleccionar_familia_dominante(self, familias):
        if len(familias) == 0:
            return None

        # Criterio principal: mayor peso_total
        # Secundario: más planos
        familias.sort(
            key=lambda f: (f["peso_total"], f["num_planos"]),
            reverse=True
        )
        return familias[0]

    def _estimar_direccion_dominante_multiplanos(self, pcd):
        """
        Estima la dirección dominante a partir de varios planos verticales.
        Devuelve vector 2D unitario en XY o None.
        """
        candidatos = self._extraer_planos_verticales_candidatos(pcd)

        if len(candidatos) == 0:
            return None

        familias = self._agrupar_direcciones_en_familias(candidatos)
        familia_dom = self._seleccionar_familia_dominante(familias)

        if familia_dom is None:
            return None

        d = familia_dom["direccion_media"]
        d = self._canonizar_direccion(d)

        resumen = []
        for i, fam in enumerate(familias):
            ang_deg = math.degrees(math.atan2(fam["direccion_media"][1], fam["direccion_media"][0]))
            resumen.append(
                f"F{i}: ang={ang_deg:.1f}deg, peso={fam['peso_total']}, planos={fam['num_planos']}"
            )

        self.get_logger().info(
            "Familias de direcciones detectadas: " + " | ".join(resumen)
        )
        self.get_logger().info(
            f"Dirección dominante elegida: [{d[0]:.4f}, {d[1]:.4f}]"
        )

        return d

    def _media_direcciones(self, d_prev, d_curr):
        if d_prev is None and d_curr is None:
            return None
        if d_prev is None:
            return self._canonizar_direccion(d_curr)
        if d_curr is None:
            return self._canonizar_direccion(d_prev)

        d_prev = self._canonizar_direccion(d_prev)
        d_curr = self._alinear_signo_con_referencia(d_curr, d_prev)

        suma = d_prev + d_curr
        ns = np.linalg.norm(suma)

        if ns < 1e-9:
            return d_prev.copy()

        return suma / ns

    # -------------------------------------------------------------------------
    # Preprocesado local
    # -------------------------------------------------------------------------

    def _preprocesar_subnube_local(self, pcd):
        pcd_local = self._copiar_nube(pcd)
        pcd_local.transform(self.T_camara_cuerpo)

        self._asegurar_colores(pcd_local, color=(0.7, 0.7, 0.7))

        dir_principal = self._estimar_direccion_dominante_multiplanos(pcd_local)
        if dir_principal is not None:
            self.get_logger().info(
                f"Dirección dominante multi-plano (frame cuerpo, XY): "
                f"[{dir_principal[0]:.4f}, {dir_principal[1]:.4f}]"
            )
        else:
            self.get_logger().warn(
                "No se pudo estimar dirección dominante multi-plano; se reutilizará la previa si existe"
            )

        planos_info = self._estimar_planos_dominantes_x(pcd_local)

        if len(planos_info) > 0:
            self.get_logger().info(
                "Planos dominantes detectados en subnube (frame cuerpo): "
                f"{['{:.3f}'.format(p['x_plano']) for p in planos_info]}"
            )
        else:
            self.get_logger().warn(
                "No se detectaron planos dominantes x=c en esta subnube"
            )

        nube_planos = o3d.geometry.PointCloud()
        if self.flag_mostrar_planos and len(planos_info) > 0:
            nube_planos = self._crear_nube_visualizacion_planos(pcd_local, planos_info)

        if self.proyectar_plano and len(planos_info) > 0:
            self._proyectar_puntos_a_planos_en_x(pcd_local, planos_info)

        return pcd_local, nube_planos, dir_principal

    # -------------------------------------------------------------------------
    # Integración
    # -------------------------------------------------------------------------

    def _integrar_nube_en_global(self, pcd_local, pose, yaw_correccion=0.0, planos_vis=None):
        if pcd_local is None or len(pcd_local.points) == 0:
            return

        nube = self._copiar_nube(pcd_local)
        self._asegurar_colores(nube, color=(0.7, 0.7, 0.7))

        self._rotar_nube_en_frame_cuerpo(nube, yaw_correccion)

        nube_planos = None
        if planos_vis is not None and len(planos_vis.points) > 0:
            nube_planos = self._copiar_nube(planos_vis)
            self._rotar_nube_en_frame_cuerpo(nube_planos, yaw_correccion)

        T_cuerpo_mapa = np.eye(4, dtype=np.float64)
        T_cuerpo_mapa[:3, :3] = self.quat_to_rot(
            pose["qx"], pose["qy"], pose["qz"], pose["qw"]
        )
        T_cuerpo_mapa[:3, 3] = np.array(
            [pose["px"], pose["py"], pose["pz"]],
            dtype=np.float64
        )

        nube.transform(T_cuerpo_mapa)
        self.global_cloud += nube

        if nube_planos is not None and len(nube_planos.points) > 0:
            nube_planos.transform(T_cuerpo_mapa)
            self.global_cloud += nube_planos

        if len(self.global_cloud.points) > 0:
            self.global_cloud = self.global_cloud.voxel_down_sample(self.voxel_size)

    def _procesar_iteracion_con_memoria_direccion(self, pcd, pose):
        pcd_proj, nube_planos, dir_actual = self._preprocesar_subnube_local(pcd)

        if len(pcd_proj.points) == 0:
            self.get_logger().warn(
                f"La subnube iteración={pose['iteracion']} quedó vacía tras el preprocesado"
            )
            return

        if dir_actual is None and self.pending_direction is not None:
            dir_actual = self.pending_direction.copy()

        if self.pending_cloud is None:
            self.pending_cloud = self._copiar_nube(pcd_proj)
            self.pending_planes_vis = self._copiar_nube(nube_planos)
            self.pending_pose = dict(pose)

            if dir_actual is not None:
                self.pending_direction = dir_actual.copy()
            else:
                self.pending_direction = np.array([1.0, 0.0], dtype=np.float64)

            self.get_logger().info(
                f"Primera nube guardada (iter={pose['iteracion']}). "
                f"Dirección almacenada = "
                f"[{self.pending_direction[0]:.4f}, {self.pending_direction[1]:.4f}]"
            )
            return

        dir_media = self._media_direcciones(self.pending_direction, dir_actual)
        if dir_media is None:
            dir_media = self.pending_direction.copy()

        yaw_corr = self._angulo_yaw_entre_direcciones(self.pending_direction, dir_media)

        self.get_logger().info(
            f"Iter={pose['iteracion']} | "
            f"dir_prev=[{self.pending_direction[0]:.4f}, {self.pending_direction[1]:.4f}] | "
            f"dir_act=[{dir_actual[0]:.4f}, {dir_actual[1]:.4f}] | "
            f"dir_media=[{dir_media[0]:.4f}, {dir_media[1]:.4f}] | "
            f"yaw_corr_prev={math.degrees(yaw_corr):.3f} deg"
        )

        self._integrar_nube_en_global(
            self.pending_cloud,
            self.pending_pose,
            yaw_correccion=yaw_corr,
            planos_vis=self.pending_planes_vis
        )

        self.pending_cloud = self._copiar_nube(pcd_proj)
        self.pending_planes_vis = self._copiar_nube(nube_planos)
        self.pending_pose = dict(pose)
        self.pending_direction = dir_media.copy()

    def _flush_ultima_nube_pendiente(self):
        if self.pending_cloud is None or self.pending_pose is None:
            return

        self.get_logger().info(
            "Integrando última nube pendiente con la última dirección media almacenada"
        )

        self._integrar_nube_en_global(
            self.pending_cloud,
            self.pending_pose,
            yaw_correccion=0.0,
            planos_vis=self.pending_planes_vis
        )

        self.pending_cloud = None
        self.pending_planes_vis = None
        self.pending_pose = None
        self.pending_direction = None

    # -------------------------------------------------------------------------
    # Reconstrucción global
    # -------------------------------------------------------------------------

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
        self.pending_cloud = None
        self.pending_planes_vis = None
        self.pending_pose = None
        self.pending_direction = None

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

            self._procesar_iteracion_con_memoria_direccion(pcd, pose)
            nubes_cargadas += 1

        self._flush_ultima_nube_pendiente()

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