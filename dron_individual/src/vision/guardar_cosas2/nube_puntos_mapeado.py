#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

import open3d as o3d
import numpy as np

class GlobalICPMap(Node):
    def __init__(self):
        super().__init__("global_icp_map")

        ## Subscriptores y Publicadores
        # Obtención Nube de Puntos Dron y Pose Dron
        self.sub_pose = self.create_subscription(PoseStamped,"sensor/GT/pose",self.pose_cb,10)
        self.sub_cloud = self.create_subscription(PointCloud2,"nube_puntos/cuerpo",self.cloud_cb,10)
        
        # Publicar Mapa Nube de Puntos
        self.pub_global = self.create_publisher(PointCloud2,"nube_puntos/global",10)
        
        ## Callbacks
        # Ultima Pose
        self.latest_pose = None

        
        # Parámetros
        self.global_frame = "map"
        self.icp_threshold = 0.30
        self.max_iteration = 100
        self.fitness_min_threshold = 0.05
        self.max_delta_traslacion = 10.0#0.85
        self.min_puntos_val_ICP = 200
        self.max_rmse_icp = 0.15
        
        # Parámetros (Tratar Nube Puntos)
        self.voxel_size = 0.03
        #self.n_vecinos_outliers = 30
        #self.std_ratio_outliers = 0.1
        
        # Variables
        self.global_cloud = o3d.geometry.PointCloud()
    
    
    def pointcloud2_to_open3d(self, msg):
        
        # Obtener los puntos
        data = list(pc2.read_points(msg, skip_nans=True))
        points = np.array([[p[0], p[1], p[2]] for p in data], dtype=np.float64)

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        
        # Obtener Colores
        colors = []
        for p in data:
            rgb = int(p[3])

            r = (rgb >> 16) & 255
            g = (rgb >> 8) & 255
            b = rgb & 255
            colors.append([r / 255.0, g / 255.0, b / 255.0])

        pcd.colors = o3d.utility.Vector3dVector(np.array(colors, dtype=np.float64))

        return pcd
    
    
    
    def open3d_to_pointcloud2(self, pcd):

        points = np.asarray(pcd.points, dtype=np.float32)
        colors = (np.asarray(pcd.colors) * 255).astype(np.uint32)
        rgb = (colors[:,0] << 16) | (colors[:,1] << 8) | colors[:,2]

        cloud_data = np.column_stack((points, rgb)).tolist()

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.global_frame

        fields = [
            pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='rgb', offset=12, datatype=pc2.PointField.UINT32, count=1),
        ]

        return pc2.create_cloud(header, fields, cloud_data)


    def quat_to_rot(self, qx, qy, qz, qw):
        
        R = None
        
        n = np.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
        if n == 0.0:
            R = np.eye(3, dtype=np.float64)
        else:
            qx, qy, qz, qw = qx/n, qy/n, qz/n, qw/n

            xx, yy, zz = qx*qx, qy*qy, qz*qz
            xy, xz, yz = qx*qy, qx*qz, qy*qz
            wx, wy, wz = qw*qx, qw*qy, qw*qz

            R = np.array([[1 - 2*(yy + zz), 2*(xy - wz), 2*(xz + wy)],
                        [2*(xy + wz), 1 - 2*(xx + zz), 2*(yz - wx)],
                        [2*(xz - wy), 2*(yz + wx), 1 - 2*(xx + yy)]], dtype=np.float64)
        return R


    def extraer_submapa(self, centro, radio):
        puntos = np.asarray(self.global_cloud.points)
        colores = np.asarray(self.global_cloud.colors)

        # Distancia de cada punto al centro
        dist = np.linalg.norm(puntos - centro.reshape(1, 3), axis=1)
        
        # Empezar con un radio e ir creciendo hasta tener un radio que tome los puntos necesarios
        submapa = o3d.geometry.PointCloud()
        
        while (not (len(submapa.points) >= self.min_puntos_val_ICP) and (len(submapa.points) < len(self.global_cloud.points))):
            mask = dist < radio
        
            # Solo cogemos los que esten cerca
            submapa.points = o3d.utility.Vector3dVector(puntos[mask])
            submapa.colors = o3d.utility.Vector3dVector(colores[mask])

            radio = radio*1.5
        
        return submapa


    def icp(self, source, target, T):
        
        source = source.voxel_down_sample(self.voxel_size)
        _, ind = source.remove_radius_outlier(nb_points = 15, radius = 0.2)
        source = source.select_by_index(ind)
        source.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        
        target = target.voxel_down_sample(self.voxel_size)
        target.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        
        result_geom = o3d.pipelines.registration.registration_icp(
            source, target, self.icp_threshold, T,
            o3d.pipelines.registration.TransformationEstimationPointToPlane()
        )
        
        result = o3d.pipelines.registration.registration_colored_icp(
                source, target, self.icp_threshold, result_geom.transformation,
                o3d.pipelines.registration.TransformationEstimationForColoredICP(),
                o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=1e-6, relative_rmse=1e-6, max_iteration=self.max_iteration)
            )
        
        # Si el resultado de ICP no es infinito pero es mayor al umbral, se solapan las nubes
        delta_t = np.linalg.norm(result.transformation[:3, 3] - T[:3, 3])
        
        self.get_logger().info(f"Fitness: {result.fitness}. Umbral Fitness: {self.fitness_min_threshold}. RMSE: {result.inlier_rmse}. Umbral RMSE: {self.max_rmse_icp}. Delta T: {delta_t}. Umbral Delta T: {self.max_delta_traslacion}")
        if not (np.isfinite(result.fitness) and (result.fitness >= self.fitness_min_threshold) and (np.isfinite(result.inlier_rmse) and (delta_t <= self.max_delta_traslacion) and (result.inlier_rmse <= self.max_rmse_icp))):
            result = None
        
        return result
    
    
    def fusionar_en_mapa(self, source_raw, target_local, T_final):
        
        nueva_pc = o3d.geometry.PointCloud(source_raw)
        nueva_pc.transform(T_final)
        
        #distancias = np.asarray(nueva_pc.compute_point_cloud_distance(target_local))
        #mask = distancias < 0.08
        #nueva_pc_filtrada = nueva_pc.select_by_index(np.where(mask)[0])

        self.global_cloud += nueva_pc
    
    
    
    def pose_cb(self, msg):
        self.latest_pose = msg

    def cloud_cb(self, cloud_msg):
        
        # Si todavía no hay pose, no hacemos nada
        if self.latest_pose is not None:
            # Pasamos a formato Open3D para tratarla
            source_raw = self.pointcloud2_to_open3d(cloud_msg)

            # Obtenemos T para ICP
            p = self.latest_pose.pose.position
            q = self.latest_pose.pose.orientation

            T_cuerpo_mapa = np.eye(4, dtype=np.float64)
            T_cuerpo_mapa[:3, :3] = self.quat_to_rot(q.x, q.y, q.z, q.w)
            T_cuerpo_mapa[:3, 3] = np.array([p.x, p.y, p.z], dtype=np.float64)
            
            # Si la nube global es 0, estamos en el primer caso -> no ICP
            if len(self.global_cloud.points) == 0:
                self.global_cloud = o3d.geometry.PointCloud(source_raw)
                self.global_cloud.transform(T_cuerpo_mapa)
            else:
                # No queremos utilizar todo el mapa
                target_local = self.extraer_submapa(centro=T_cuerpo_mapa[:3,3], radio=3.0)
                """
                result = self.icp(source_raw, target_local, T_cuerpo_mapa)
                
                
                if result is not None:
                    self.fusionar_en_mapa(source_raw, target_local, result.transformation)
                    self.get_logger().info("ICP aceptado y nube fusionada")
                else:
                    self.get_logger().warn("ICP rechazado, No hay fusión")
                """
                self.fusionar_en_mapa(source_raw, target_local, T_cuerpo_mapa)
                
            # Una vez solapado (o no si es la primera), publicamos
            self.global_cloud = self.global_cloud.voxel_down_sample(self.voxel_size)
            
            _, ind = self.global_cloud.remove_radius_outlier(nb_points = 15, radius = 0.2)
            self.global_cloud = self.global_cloud.select_by_index(ind)
            
            msg = self.open3d_to_pointcloud2(self.global_cloud)
            self.pub_global.publish(msg)
    



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