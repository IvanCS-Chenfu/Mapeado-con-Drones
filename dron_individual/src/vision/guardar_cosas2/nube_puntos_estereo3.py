#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo

from cv_bridge import CvBridge
import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np

from action_msgs.msg import GoalStatusArray
from sensor_msgs.msg import PointCloud2

import open3d as o3d
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2

class Clase_Subscriber(Node):
    def __init__(self):
        super().__init__("nube_puntos_estereo")
        
        ## Subscriptores y Publicadores
        # Obtención Imagenes Cámaras
        self.objeto_subscriber_izq = self.create_subscription(Image,"sensor/camara_izq/image_raw", self.callback_izq, 10) 
        self.objeto_subscriber_der = self.create_subscription(Image,"sensor/camara_der/image_raw", self.callback_der, 10) 
        
        # Obtención Parámetros Intrínsecos Cámara
        self.info_izq = self.create_subscription(CameraInfo, "sensor/camara_izq/camera_info", self.callback_info_izq, 10)
        
        # Saber cuando termina la acción (el dron está parado)
        self.sub_status = self.create_subscription(GoalStatusArray,"AccionTrayectoria/_action/status",self.callback_status,10)
        
        # Publicar Nube de Puntos
        self.pub_pc = self.create_publisher(PointCloud2, "nube_puntos/cuerpo", 10)
    
    
    
    
        ## Tratar Imágenes para mapa de profundidad.
        # Bridge para convertir mensajes ROS2 a imágenes OpenCV
        self.bridge = CvBridge()
        
        # Variables para almacenar las imágenes
        self.img_der = None
        self.img_izq = None
        
        # Para hacer que la figura no sea bloqueante
        self.flag_mostrar_imagenes = True
        plt.ion()
        self.fig = None
        
        if (self.flag_mostrar_imagenes):
            self.fig = plt.figure("Camaras")
        
        # Para repetir la función cada x segundos.
        self.timer_ = self.create_timer(0.05, self.mapa_SGBM)
        
        # Datos Necesarios (parámetros cámaras)
        self.baseline = 0.06
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.frame_id = None
        self.camara2cuerpo = [0.1, 0.03, 0.03]
        
        # Datos Necesarios (tratamiendo mapa profundidad)
        self.umbral_gradiente = 1
        self.umbral_textura = 0
        self.n_promediado = 20
        self.buffer_disparidad = []
        self.im_profundidad_media = None
        self.im_disparidad = None
        self.profundidad_maxima = 10.0
        self.error_profundidad = 1.0
        
        # Datos Necesarios (realizar proceso y publicar nube de puntos)
        self.estado_accion = None
        self.flag_publicado = False
        
        
        ## Objeto Estéreo
        window_size = 7
        min_disp = 0
        nDispFactor = 12        # parámetro a ajustar
        canales = 1             # Es gris
        self.objeto_estereo = cv.StereoSGBM_create(minDisparity=min_disp,
                                                numDisparities=16*nDispFactor,
                                                blockSize=window_size,
                                                P1 = 8*canales*window_size**2,      # Fórmula OpenCV
                                                P2 = 32*canales*window_size**2,     # Fórmula OpenCV
                                                disp12MaxDiff=1,
                                                uniquenessRatio=15,
                                                speckleWindowSize=100,
                                                speckleRange=2,
                                                preFilterCap=63,
                                                mode = cv.STEREO_SGBM_MODE_SGBM_3WAY)
    
    
    def callback_izq(self, msg):
        self.img_izq = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        

    def callback_der(self, msg):
        self.img_der = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
    def callback_info_izq(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.frame_id = msg.header.frame_id
        
    # 4 si está quieto y 2 si no
    def callback_status(self, msg: GoalStatusArray):
        if msg.status_list:
            ultimo = msg.status_list[-1]
            self.estado_accion = ultimo.status




    def depth2pc(self, rgb_img, depth_img):
        
        pcd = None
        
        if not (rgb_img is None or depth_img is None or self.fx is None or self.fy is None or self.cx is None or self.cy is None):
            
            # Eliminar NaN para Open3D
            depth_clean = np.copy(depth_img).astype(np.float32)
            depth_clean[np.isnan(depth_clean)] = 0.0
            depth_clean[depth_clean <= 0.0] = 0.0

            # OpenCV usa BGR; Open3D espera RGB
            rgb_o3d_np = cv.cvtColor(rgb_img, cv.COLOR_BGR2RGB)

            rgb = o3d.geometry.Image(rgb_o3d_np.astype(np.uint8))
            depth = o3d.geometry.Image(depth_clean.astype(np.float32))

            # Github Open3D
            rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb,depth,depth_scale=1.0,depth_trunc=self.profundidad_maxima,convert_rgb_to_intensity=False)
            intrinsics = o3d.camera.PinholeCameraIntrinsic(width=rgb_img.shape[1],height=rgb_img.shape[0],fx=self.fx,fy=self.fy,cx=self.cx,cy=self.cy)

            pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsics)

            # Girar -90º sobre X y trasladar al centro del dron
            pcd.transform([[0, 0, 1,  self.camara2cuerpo[0]],
                           [-1, 0, 0,  self.camara2cuerpo[2]],
                           [0,-1, 0, -self.camara2cuerpo[1]],
                           [0, 0, 0,  1]])
            

        return pcd    
    
    def open3d_to_pointcloud2(self, pcd):

        points = np.asarray(pcd.points, dtype=np.float32)
        colors = (np.asarray(pcd.colors) * 255).astype(np.uint32)
        rgb = (colors[:,0] << 16) | (colors[:,1] << 8) | colors[:,2]    # pc2 lo pide los 3 bytes en fila

        cloud_data = np.column_stack((points, rgb)).tolist()

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id if self.frame_id is not None else "camera_frame"

        fields = [
            pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='rgb', offset=12, datatype=pc2.PointField.UINT32, count=1),
        ]

        return pc2.create_cloud(header, fields, cloud_data)
    


    
    def filtrar_puntos_por_normal_hacia_camara(self, pcd, angulo_max_deg=70.0):
        if pcd is None or len(pcd.points) == 0:
            return pcd

        pcd = pcd.voxel_down_sample(voxel_size=0.08)
        
        # Estimar normales
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=0.10,   # ajusta según densidad/escala de tu nube
                max_nn=30
            )
        )
        """
        # La nube ya está transformada al marco "cuerpo",
        # así que la posición de la cámara en ese marco es:
        camera_location = np.array([
            self.camara2cuerpo[0],
            self.camara2cuerpo[2],
            -self.camara2cuerpo[1]
        ], dtype=np.float64)

        # Orientar normales hacia la cámara
        pcd.orient_normals_towards_camera_location(camera_location)

        # Filtrar por ángulo entre normal y dirección hacia la cámara
        points = np.asarray(pcd.points)
        normals = np.asarray(pcd.normals)

        vec_hacia_camara = camera_location[None, :] - points
        dist = np.linalg.norm(vec_hacia_camara, axis=1)

        mask_valid = dist > 1e-8
        vec_hacia_camara[mask_valid] /= dist[mask_valid][:, None]

        # cos(theta) = n · v
        cos_theta = np.einsum("ij,ij->i", normals, vec_hacia_camara)

        # Mantener puntos cuya normal forme como mucho angulo_max_deg con la cámara
        cos_min = np.cos(np.deg2rad(angulo_max_deg))
        mask = mask_valid & (cos_theta >= cos_min)

        pcd_filtrado = pcd.select_by_index(np.where(mask)[0])
        """
        return pcd



    def mapa_SGBM(self):
        # Solo hace cosas si están las imagenes, si el dron está parado y si todavía no se ha publicado dicha nube de puntos
        if (self.img_der is not None) and (self.img_izq is not None) and (self.estado_accion == 4) and not (self.flag_publicado):
            im_left = cv.cvtColor(self.img_izq, cv.COLOR_BGR2GRAY)
            im_right = cv.cvtColor(self.img_der, cv.COLOR_BGR2GRAY)
            
            disparidad = self.objeto_estereo.compute(im_left, im_right).astype(np.float32)/16.0
            
            disparidad = cv.medianBlur(disparidad,5)
            
            disparidad[disparidad <= 0.0] = np.nan
            
            
            
            # Eliminamos zonas de mucho cambio
            disp_grad_x = cv.Sobel(disparidad, cv.CV_32F, 1, 0, ksize=3)
            disp_grad_y = cv.Sobel(disparidad, cv.CV_32F, 0, 1, ksize=3)
            disp_grad = np.sqrt(disp_grad_x**2 + disp_grad_y**2)

            disparidad[disp_grad > self.umbral_gradiente] = np.nan
            
            # Eliminamos zonas de poca textura
            img_grad_x = cv.Sobel(im_left, cv.CV_32F, 1, 0, ksize=3)
            img_grad_y = cv.Sobel(im_right, cv.CV_32F, 0, 1, ksize=3)
            textura = np.sqrt(img_grad_x**2 + img_grad_y**2)

            disparidad[textura < self.umbral_textura] = np.nan

            
            if (self.flag_mostrar_imagenes):
                self.im_disparidad = disparidad
            
            # Guardar la disparidad actual en el buffer
            self.buffer_disparidad.append(disparidad)
            
            # Solo mostrar cuando haya N mapas
            if len(self.buffer_disparidad) >= self.n_promediado:
                
                # Uno todos los mapas y limpio el buffer
                depth_stack = np.stack(self.buffer_disparidad, axis=0)
                self.buffer_disparidad.clear()
                
                ## DISPARIDAD -> PROFUNDIDAD
                profundidad_stack = self.fx * self.baseline / depth_stack
                
                # Aquellos cuya profundidad sea mayor que la máxima, se eliminan
                profundidad_stack[profundidad_stack > self.profundidad_maxima] = np.nan
                
                # Se detectan los pixeles donde ha habido algún "NaN" y todos ellos se vuelven a poner como "NaN"
                mask_nan = np.isnan(profundidad_stack).any(axis=0)
                profundidad_media = np.mean(profundidad_stack, axis=0)
                profundidad_media[mask_nan] = np.nan
                
                # Se detectan cambios bruscos en profundidad y se pone un "NaN" en aquellos que superen el error máximo
                error = np.nanmax(profundidad_stack, axis=0) - np.nanmin(profundidad_stack, axis=0)
                mask_error = error > self.error_profundidad
                profundidad_media[mask_error] = np.nan
                
                if (self.flag_mostrar_imagenes):
                    self.im_profundidad_media = profundidad_media
                
                # Obtener Nube de Puntos 
                pcd = self.depth2pc(self.img_izq, profundidad_media)

                if (pcd is not None) and (len(pcd.points) > 0):
                    # Filtrar puntos cuya normal apunte hacia la cámara
                    pcd = self.filtrar_puntos_por_normal_hacia_camara(pcd, angulo_max_deg=70.0)
                    o3d.visualization.draw_geometries([pcd])

                    if (pcd is not None) and (len(pcd.points) > 0):
                        # Pasar nube de puntos a mensaje ROS2
                        pcd_msg = self.open3d_to_pointcloud2(pcd)
                        
                        if (pcd_msg is not None):
                            self.pub_pc.publish(pcd_msg)
                            self.get_logger().info("Nube de puntos publicada")
                            self.flag_publicado = True
        
        # Cuando el dron tenga otro objetivo, se reinicia el flag.
        if (self.estado_accion == 2):
            self.flag_publicado = False 
                
        
        if (self.img_der is not None) and (self.img_izq is not None) and (self.flag_mostrar_imagenes):
            
            im_left = cv.cvtColor(self.img_izq, cv.COLOR_BGR2GRAY)
            im_right = cv.cvtColor(self.img_der, cv.COLOR_BGR2GRAY)
            
            disparidad = self.objeto_estereo.compute(im_left, im_right).astype(np.float32)/16.0
            
            profundidad = self.fx * self.baseline / disparidad
            
            plt.subplot(3,2,1)
            plt.imshow(self.img_izq)
            plt.title("Camara Izquierda")

            plt.subplot(3,2,2)
            plt.imshow(self.img_der)
            plt.title("Camara Derecha")
            
            plt.subplot(3,2,3)
            plt.imshow(disparidad)
            plt.title("Disparidad Actual")
            
            plt.subplot(3,2,4)
            plt.imshow(profundidad)
            plt.title("Profundidad Actual")
            
            plt.subplot(3,2,5)
            if self.im_disparidad is not None:
                plt.imshow(self.im_disparidad, cmap='gray')
            plt.colorbar()
            plt.title("Disparidad Actual")
            
            plt.subplot(3,2,6)
            if self.im_profundidad_media is not None:
                plt.imshow(self.im_profundidad_media, cmap='gray')
            plt.colorbar()
            plt.title("Profundidad Media")
                
            plt.pause(0.001)
            plt.clf()
            

        
def main(args=None):
    rclpy.init(args=args) 
    
    objeto_nodo = Clase_Subscriber() 
    rclpy.spin(objeto_nodo) 
    
    rclpy.shutdown() 

if __name__ == '__main__':
    main()