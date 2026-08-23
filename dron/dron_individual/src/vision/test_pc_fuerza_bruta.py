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
        self.declare_parameter("it_min", 1)    # 17
        self.declare_parameter("it_max", 99)    # 18

        self.numero_dron = self.get_parameter("numero_dron").value
        self.it_min_param = self.get_parameter("it_min").value
        self.it_max_param = self.get_parameter("it_max").value

        # Parámetros internos
        self.global_frame = "map"
        self.voxel_size = 0.03
        self.carpeta_nubes = "./nubes_puntos"
        self.csv_path = os.path.join(self.carpeta_nubes, "poses.csv")
        self.camara2cuerpo = [0.1, 0.03, 0.03]

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

    def integracion_pc(self, pcd, pose):
        
        # Aplicar cámara -> cuerpo
        pcd.transform(self.T_camara_cuerpo)
        
        # Transformación cuerpo -> mapa a partir de la pose
        T_cuerpo_mapa = np.eye(4, dtype=np.float64)
        T_cuerpo_mapa[:3, :3] = self.quat_to_rot(
            pose["qx"], pose["qy"], pose["qz"], pose["qw"]
        )
        T_cuerpo_mapa[:3, 3] = np.array(
            [pose["px"], pose["py"], pose["pz"]],
            dtype=np.float64
        )

        # Aplicar cuerpo -> mapa
        pcd.transform(T_cuerpo_mapa)
            
        # Acumular sobre la nube global actual
        self.global_cloud += pcd

        if len(self.global_cloud.points) == 0:
            self.get_logger().warn("No se pudo construir ninguna nube global")
            return

        # Voxelizado final tras cada integración
        self.global_cloud = self.global_cloud.voxel_down_sample(self.voxel_size)

    def reconstruir_mapa_global(self):
        poses = self.leer_poses_csv()

        if len(poses) == 0:
            self.get_logger().warn("No se encontraron poses en el CSV")
            return

        # Filtrar por numero_dron
        poses_dron = [
            pose for pose in poses
            if pose["numero_dron"] == self.numero_dron
        ]

        if len(poses_dron) == 0:
            self.get_logger().warn(
                f"No se encontraron poses para numero_dron={self.numero_dron}"
            )
            return

        # Obtener iteraciones reales mínima y máxima para ese dron
        iteraciones = [pose["iteracion"] for pose in poses_dron]
        iteracion_min_real = min(iteraciones)
        iteracion_max_real = max(iteraciones)

        # Saturar it_min e it_max al rango real
        it_min_ajustada = max(self.it_min_param, iteracion_min_real)
        it_max_ajustada = min(self.it_max_param, iteracion_max_real)

        # Si vienen invertidas, corregirlas
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

        # Filtrar por rango de iteraciones
        poses_filtradas = [
            pose for pose in poses_dron
            if it_min_ajustada <= pose["iteracion"] <= it_max_ajustada
        ]

        if len(poses_filtradas) == 0:
            self.get_logger().warn("No hay poses dentro del rango solicitado")
            return

        # Ordenar por iteración
        poses_filtradas.sort(key=lambda x: x["iteracion"])

        # Reiniciar nube global antes de reconstruir
        self.global_cloud = o3d.geometry.PointCloud()
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

            # Integrar nube local en la global
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