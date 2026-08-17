#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo

from cv_bridge import CvBridge
import cv2 as cv
import open3d as o3d

import matplotlib.pyplot as plt
import numpy as np

from std_msgs.msg import UInt8
from std_msgs.msg import Float64MultiArray

from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2

class Clase_Subscriber(Node):
    def __init__(self):
        super().__init__("nube_puntos_estereo")
        
        ## Subscriptores y Publicadores
        # Obtención Imagenes Cámaras
        self.objeto_subscriber_izq = self.create_subscription(Image,"sensor/camara_izq/image_raw", self.callback_izq, 10) 
        self.objeto_subscriber_der = self.create_subscription(Image,"sensor/camara_der/image_raw", self.callback_der, 10) 
        
        # Obtención Parámetros Intrínsecos Cámara
        self.info_izq = self.create_subscription(CameraInfo, "sensor/camara_izq/camera_info", self.callback_info_izq, 10)
        
        # Obtención de la señal de control
        self.byte_control_sub = self.create_subscription(UInt8,"vision/byte_control",self.callback_byte_control,10)

        # Publicar al controlador
        self.publicador_control = self.create_publisher(Float64MultiArray,"vision/keypoint_cercano",10)

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
        self.timer_ = self.create_timer(0.05, self.control)
        
        # Datos Necesarios (parámetros cámaras)
        self.baseline = 0.057
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        
        self.frame_id = None
        self.camara2cuerpo = [0.1, 0.03, 0.03]
        
        self.flag_rectificacion = False
        self.K = None
        self.D = None
        self.R = None
        self.P = None
        
        self.map1 = None
        self.map2 = None
        self.rectificacion_lista = False
        
        # Datos Necesarios (tratamiendo mapa profundidad)
        self.umbral_gradiente = 1
        self.umbral_textura = 0
        self.n_promediado = 2   # Había 20
        self.buffer_disparidad = []
        self.im_profundidad_media = None
        self.im_disparidad = None
        self.profundidad_maxima = 4.0
        self.error_profundidad = 20.0    # Cambiado 
        
        # Datos Necesarios (realizar proceso y publicar nube de puntos)
        self.estado_accion = None
        self.flag_publicado = False
        
        
        ## Objeto Estéreo
        window_size = 7
        min_disp = 0
        nDispFactor = 12        # parámetro a ajustar
        canales = 1             # Es gris
        self.objeto_estereo_izq = cv.StereoSGBM_create(minDisparity=min_disp,
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
        
        self.objeto_estereo_der = cv.ximgproc.createRightMatcher(self.objeto_estereo_izq)
        
        self.wls_filter = cv.ximgproc.createDisparityWLSFilter(matcher_left=self.objeto_estereo_izq)
        self.wls_filter.setLambda(8000.0)
        self.wls_filter.setSigmaColor(1.5)
        
        # Para "keypoint_cercano()"
        self.max_features = 100
        self.orb = cv.ORB_create(nfeatures=self.max_features)
        self.matcher = cv.BFMatcher(cv.NORM_HAMMING, crossCheck=True)
        
        self.orb_disparidad = 0.0
        self.x_orb = 0.0
        self.y_orb = 0.0
        
        self.byte_control = [0]*8
        self.byte_control_anterior = [0]*8
        self.disparidad_a_buscar = 0.0
        self.flag_primera_vez = True
        
        # Para "normales_punto()"
        self.voxel_size = 0.08
        self.max_bins_separacion = 2
        
        # Para "enviar_distancia_cercana()"
        self.distancia_maxima_permitida = 5.0
        
        # Para "pub_nube_puntos_flags()"
        self.umbral_z_planos = 1.0
    
    
    ##################### CALBACKS #####################
    
    def callback_izq(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        if self.rectificacion_lista and self.flag_rectificacion:
            img = cv.remap(img, self.map1, self.map2, interpolation=cv.INTER_LINEAR)

        self.img_izq = img
        

    def callback_der(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        if self.rectificacion_lista and self.flag_rectificacion:
            img = cv.remap(img, self.map1, self.map2, interpolation=cv.INTER_LINEAR)

        self.img_der = img
        
    def callback_info_izq(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.frame_id = msg.header.frame_id
        
        if self.flag_rectificacion:
            # Parámetros necesarios para rectificar
            self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.D = np.array(msg.d, dtype=np.float64)
            self.R = np.array(msg.r, dtype=np.float64).reshape(3, 3)
            self.P = np.array(msg.p, dtype=np.float64).reshape(3, 4)

            # Crear mapas de rectificación una sola vez
            width = msg.width
            height = msg.height
            size = (width, height)

            self.map1, self.map2 = cv.initUndistortRectifyMap(
                self.K,
                self.D,
                self.R,
                self.P[:, :3],
                size,
                cv.CV_32FC1
            )

        self.rectificacion_lista = True
        
    def callback_byte_control(self, msg):
        for i in range(8):
            self.byte_control[i] = (msg.data >> i) & 1
            
            self.get_logger().info(f"Bit {i}: {(msg.data >> i) & 1}")
        
    ##################### FUNCIONES #####################
    
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

            """
            # Girar -90º sobre X y trasladar al centro del dron
            pcd.transform([[0, 0, 1,  self.camara2cuerpo[0]],
                           [-1, 0, 0,  self.camara2cuerpo[2]],
                           [0,-1, 0, -self.camara2cuerpo[1]],
                           [0, 0, 0,  1]])
            """

        return pcd 
    
    
    def im2prof(self, im_left, im_right, flag_tratar_profundidad):
        
        im_left = cv.cvtColor(self.img_izq, cv.COLOR_BGR2GRAY)
        im_right = cv.cvtColor(self.img_der, cv.COLOR_BGR2GRAY)
        
        disparidad_izq = self.objeto_estereo_izq.compute(im_left, im_right)
        disparidad_der = self.objeto_estereo_der.compute(im_right, im_left)
        
        disparidad = self.wls_filter.filter(disparidad_izq, self.img_izq, disparity_map_right=disparidad_der)
        disparidad = disparidad.astype(np.float32) / 16.0
        
        disparidad[disparidad <= 0.0] = np.nan
        
        mask = disparidad > 0  # solo valores válidos

        profundidad_filtrada = np.zeros_like(disparidad, dtype=np.float32)
        profundidad_filtrada[mask] = self.fx * self.baseline / disparidad[mask]
        
        profundidad_filtrada[profundidad_filtrada > self.profundidad_maxima] = np.nan
        profundidad_filtrada[profundidad_filtrada < 0.1] = np.nan
        
        profundidad_textura_areas = profundidad_filtrada.copy()
        
        if flag_tratar_profundidad:
            """
            # Máscara Bordes (todo 1 menos bordes)
            mask_bordes, _, _ = self.eliminar_bordes_fuertes(profundidad_filtrada, umbral_grad=0.1)
            # Máscara textura (todo 1 menos zonas de baja textura)
            mask_textura, std_map_textura = self.mascara_poca_textura(im_left, ksize=15, threshold=1)
            # Máscara Corte Profundidad
            mask_corte = cv.bitwise_and(mask_textura,mask_bordes)
            mask_corte[np.isnan(profundidad_filtrada)] = 0  
            
            kernel_elipse = cv.getStructuringElement(cv.MORPH_ELLIPSE, (5,5))
            # Eliminar zonas blancas pequeñas
            mask_text_erode = cv.erode(mask_corte, kernel_elipse)
            mask_textura_Ablancas = self.eliminar_areas_cerradas_grandes(mask_text_erode, 3000, False)
            
            # Eliminar zonas negras pequeñas
            #mask_textura_Ablancas[np.isnan(profundidad_filtrada)] = 255.0 # Necesario para cerrar áreas que estén a la izquierda (NO SE POR QUE NO VA)
            mask_textura_areas = cv.bitwise_not(mask_textura_Ablancas)
            mask_textura_areas = self.eliminar_areas_cerradas_grandes(mask_textura_areas, 3000, False)
            mask_textura_areas = cv.bitwise_not(mask_textura_areas)
            
            for _ in range(5):
                mask_textura_areas = cv.morphologyEx(mask_textura_areas, cv.MORPH_CLOSE, kernel_elipse)
            
            profundidad_textura_areas[mask_textura_areas == 0.0] = np.nan
            """
            
            # Máscara Bordes de la imagen de profundidad (todo 1 menos bordes)
            mask_bordes_prof, _, _ = self.eliminar_bordes_fuertes(profundidad_filtrada, umbral_grad=0.1)
            # Máscara Bordes de la imagen gris (todo 1 menos bordes)
            mask_bordes_gray, _, _ = self.eliminar_bordes_fuertes(im_left, umbral_grad=100)
            
            # Máscara Bordes
            mask_bordes = cv.bitwise_and(mask_bordes_prof,mask_bordes_gray)
            
            # Máscara textura (todo 1 menos zonas de baja textura)
            mask_textura, std_map_textura = self.mascara_poca_textura(im_left, ksize=15, threshold=1)
            # Máscara Corte Profundidad
            mask_corte = cv.bitwise_and(mask_textura,mask_bordes)
            mask_corte[np.isnan(disparidad)] = 0  
            
            kernel_elipse = cv.getStructuringElement(cv.MORPH_ELLIPSE, (5,5))
            # Eliminar zonas blancas pequeñas
            mask_text_erode = cv.erode(mask_corte, kernel_elipse)
            mask_textura_Ablancas = self.eliminar_areas_cerradas_grandes(mask_text_erode, 3000, False)
            
            # Eliminar zonas negras pequeñas
            mask_nans = np.isnan(disparidad).astype(np.uint8)
            mask_nans = cv.dilate(mask_nans, kernel_elipse)
            mask_nans = mask_nans.astype(bool)
            
            mask_textura_Ablancas[mask_nans] = 255 # Necesario para cerrar áreas que estén a la izquierda
            
            mask_textura_areas = cv.bitwise_not(mask_textura_Ablancas)
            mask_textura_areas = self.eliminar_areas_cerradas_grandes(mask_textura_areas, 3000, False)
            mask_textura_areas = cv.bitwise_not(mask_textura_areas)
            
            mask_textura_areas[mask_nans] = 0 # Necesario para cerrar áreas que estén a la izquierda
            
            # Aplicar las máscaras para mostrar lo necesario para la nube de puntos
            profundidad_textura_areas = profundidad_filtrada.copy()
            profundidad_textura_areas[mask_textura_areas == 0.0] = np.nan
            
            plt.subplot(3,4,1)
            plt.imshow(im_left, cmap="gray")
            plt.title("Imagen Izquierda")
            
            plt.subplot(3,4,2)
            plt.imshow(im_right, cmap="gray")
            plt.title("Imagen Derecha")
            
            plt.subplot(3,4,5)
            plt.imshow(disparidad_izq, cmap="gray")
            plt.title("Disparidad Izq sin Tratar")
            
            plt.subplot(3,4,6)
            plt.imshow(disparidad_der, cmap="gray")
            plt.title("Disparidad Der sin Trata")
            
            plt.subplot(3,4,9)
            plt.imshow(disparidad, cmap="gray")
            plt.title("Disparidad Filtrada Izq-Der")
            
            plt.subplot(3,4,10)
            plt.imshow(profundidad_filtrada, cmap="gray")
            plt.title("Profundidad Filtrada Izq-Der")
            
            plt.subplot(3,4,3)
            plt.imshow(mask_bordes, cmap="gray")
            plt.title("mask_bordes")
            
            plt.subplot(3,4,4)
            plt.imshow(mask_textura, cmap="gray")
            plt.title("mask_textura")

            plt.subplot(3,4,7)
            plt.imshow(mask_corte, cmap="gray")
            plt.title("mask_corte")
            
            plt.subplot(3,4,8)
            plt.imshow(mask_textura_Ablancas, cmap="gray")
            plt.title("mask_textura_Anegras")
            
            plt.subplot(3,4,11)
            plt.imshow(mask_textura_areas, cmap="gray")
            plt.title("mask_textura_areas")
            
            plt.subplot(3,4,12)
            plt.imshow(profundidad_textura_areas, cmap="gray")
            plt.title("profundidad_textura_areas")

            plt.pause(0.001)
            plt.clf()  
        
        return profundidad_textura_areas

    
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
    
    
    
    def mascara_poca_textura(self, img, ksize=15, threshold=1):
        """
        img: imagen BGR o gris
        ksize: tamaño de ventana local
        threshold: umbral sobre la desviación estándar local
        """
        # Convertir a gris si hace falta
        if len(img.shape) == 3:
            gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        else:
            gray = img.copy()

        gray = gray.astype(np.float32)

        # Media local
        mean = cv.blur(gray, (ksize, ksize))

        # Media del cuadrado
        mean_sq = cv.blur(gray * gray, (ksize, ksize))

        # Varianza = E[x^2] - (E[x])^2
        var = mean_sq - mean * mean
        var = np.clip(var, 0, None)

        # Desviación estándar local
        std = np.sqrt(var)
        """
        # Reduir la máscara debido a los bordes
        r = ksize // 2
        kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (2*r+1, 2*r+1))
        std = cv.erode(std, kernel)
        """
        # Máscara de poca textura: std baja
        mask = (std >= threshold).astype(np.uint8) * 255

        return mask, std

    def eliminar_areas_cerradas_grandes(self, mask, area_umbral, flag_bordes_imagen):
        """
        mask: imagen binaria uint8, con regiones de interés en 255
        area_umbral: si una región cerrada tiene área > area_umbral, se pone a 0

        Devuelve:
            mask_out: máscara modificada
        """

        # Asegurar binaria
        mask_bin = (mask > 0).astype(np.uint8)

        # Componentes conexas
        num_labels, labels, stats, _ = cv.connectedComponentsWithStats(mask_bin, connectivity=8)

        h, w = mask.shape
        mask_out = mask.copy()

        for label in range(1, num_labels):  # 0 es el fondo
            x = stats[label, cv.CC_STAT_LEFT]
            y = stats[label, cv.CC_STAT_TOP]
            ww = stats[label, cv.CC_STAT_WIDTH]
            hh = stats[label, cv.CC_STAT_HEIGHT]
            area = stats[label, cv.CC_STAT_AREA]

            # Comprobar si toca borde
            toca_borde = (flag_bordes_imagen and (x == 0 or y == 0 or (x + ww) == w or (y + hh) == h))

            # Si no toca borde y supera el umbral, borrar la región
            if (not toca_borde) and (area < area_umbral):
                mask_out[labels == label] = 0

        return mask_out


    def eliminar_bordes_fuertes(self, img, umbral_grad=0.5, ksize=3):
        if len(img.shape) == 3:
            gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        else:
            gray = img.copy()

        gray = cv.GaussianBlur(gray, (5, 5), 0)
        gray = gray.astype(np.float32)

        gx = cv.Sobel(gray, cv.CV_32F, 1, 0, ksize=ksize)
        gy = cv.Sobel(gray, cv.CV_32F, 0, 1, ksize=ksize)

        # Siempre no negativa
        mag = cv.magnitude(gx, gy)

        # Máscara de bordes pronunciados
        mask_bordes = (mag > umbral_grad).astype(np.uint8) * 255
        mask_out = cv.bitwise_not(mask_bordes)

        return mask_out, mag, mask_bordes
    
    
    
    ##################### CONTROL #####################
    
    def control(self):
        
        if (self.byte_control[0] == 1):
            self.disparidad_maxima_ORB()
        if (self.byte_control[1] == 1):
            self.avisar_punto_cercano()
        else:
            self.flag_primera_vez = True
        if (self.byte_control[2] == 1 and self.byte_control_anterior[2] == 0):  # Para que solo se llame una vez 
            self.normales_punto()
        if (self.byte_control[3] == 1 and self.byte_control_anterior[3] == 0):  # Para que solo se llame una vez 
            self.pub_nube_puntos()
        if (self.byte_control[4] == 1 and self.byte_control_anterior[4] == 0):  # Para que solo se llame una vez 
            self.pub_flags()
        if (self.byte_control[5] == 1 and self.byte_control_anterior[5] == 0):  # Para que solo se llame una vez 
            self.enviar_distancia_cercana()
            
            
        self.byte_control_anterior = self.byte_control.copy()
            
            
    # Mediante ORB obtiene keypoints. Hace match entre las dos vistas y saca disparidad. El keypoint con mayor disparidad -> más cercano
    def disparidad_maxima_ORB(self):
        
        self.keypoint_cercano_publicado = False     # Para reiniciar el flag y volver a publicar cuando termine
        
        if (self.img_der is not None) and (self.img_izq is not None):
            im_left = cv.cvtColor(self.img_izq, cv.COLOR_BGR2GRAY)
            im_right = cv.cvtColor(self.img_der, cv.COLOR_BGR2GRAY)
            
            kp_left, des_left = self.orb.detectAndCompute(im_left, None)
            kp_right, des_right = self.orb.detectAndCompute(im_right, None)

            if not(des_left is None or des_right is None or len(kp_left) == 0 or len(kp_right) == 0):
                matches = self.matcher.match(des_left, des_right)
                matches = sorted(matches, key=lambda m: m.distance)
                
                for m in matches:
                    if m.distance <= 5:    # Solo nos quedamos con los matches cuya distancia entre descriptores es pequeña
                        # Distancia en pixeles 
                        xL, yL = kp_left[m.queryIdx].pt
                        xR, yR = kp_right[m.trainIdx].pt

                        if abs(yL - yR) <= 3:   # Como están en estéreo, la distancia en y no puede ser muy grande (deberían estar en la misma línea)
                            disparidad = xL - xR

                            if disparidad > self.orb_disparidad:    # Disparidad grande -> distancia cercana -> lo queremos
                                self.orb_disparidad = disparidad 
                                self.x_orb = xL
                                self.y_orb = yL
                                
                if self.flag_mostrar_imagenes:
                    img_kp_left = im_left.copy()
                    img_kp_right = im_right.copy()
                    
                    if (self.orb_disparidad != 0.0) and (self.x_orb != 0) and (self.y_orb != 0):
                        img_kp_left = cv.circle(img_kp_left, (int(self.x_orb), int(self.y_orb)), 10, (255,0,0), -1) 
                        img_kp_right = cv.circle(img_kp_right, (int(self.x_orb + self.orb_disparidad), int(self.y_orb)), 10, (255,0,0), -1) 
                    
                    plt.subplot(1,2,1)
                    plt.imshow(img_kp_left)
                    plt.title("Camara Izquierda")

                    plt.subplot(1,2,2)
                    plt.imshow(img_kp_right)
                    plt.title("Camara Derecha")
                        
                    plt.pause(0.001)
                    plt.clf()
    
    # Utilizando la disparidad máxima de la función anterior, calcula la posición tridimensional del keypoint.
    def avisar_punto_cercano(self):
        
        if self.flag_primera_vez:
            self.flag_primera_vez = False
            self.disparidad_a_buscar = self.orb_disparidad
            self.orb_disparidad = 0.0       # Reiniciamos disparidad para que en "disparidad_maxima_ORB" se actualice
        
        #self.get_logger().info(f"Disparidad A Buscar: {self.disparidad_a_buscar}. Disparidad Máxima Atual: {self.orb_disparidad}")
        
        if (self.disparidad_a_buscar - self.orb_disparidad <= 0.1):  # Cuando la disparidad nueva con la maxima encontrada anteriomente sean iguales, se publica la pose para que el dron se mueva
            if self.orb_disparidad > 0.0:
                Z = (self.fx * self.baseline) / self.orb_disparidad
                X = ((self.x_orb - self.cx) * Z) / self.fx
                Y = ((self.y_orb  - self.cy) * Z) / self.fy
                
                msg = Float64MultiArray()
                # Pasar a coordenadas del cuerpo
                msg.data = [float(Z), float(-X), float(-Y), 0.0]
                self.publicador_control.publish(msg)
                
                self.orb_disparidad = 0.0
                
                self.flag_primera_vez = True

    # Mapa disparidad -> mapa profundidad -> nube de puntos densa -> voxelizar -> sacar normales -> histograma normales -> normal max -> punto cercano
    def normales_punto(self):
        
        if (self.img_der is not None) and (self.img_izq is not None):
            im_left = cv.cvtColor(self.img_izq, cv.COLOR_BGR2GRAY)
            im_right = cv.cvtColor(self.img_der, cv.COLOR_BGR2GRAY)
            
            # mapa profundidad -> nube de puntos densa
            profundidad = self.im2prof(im_left, im_right, True)
            pcd = self.depth2pc(self.img_izq, profundidad)
            
            error = False
            if len(pcd.points) == 0:
                error = True
            else:
                # voxelizar -> sacar normales
                pcd_voxel = pcd.voxel_down_sample(voxel_size=self.voxel_size)
                if len(pcd.points) == 0:
                    error = True
                
            # Si hay algún error, es porque el dron se ha perdido y está mirando a la nada. Esto puede romper el programa.
            if error:
                msg = Float64MultiArray()
                msg.data = [-1.0, -1.0, -1.0, -1.0]
                self.publicador_control.publish(msg)
            else: 
                pcd_voxel.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.20,max_nn=30))
                pcd_voxel.orient_normals_towards_camera_location(camera_location=np.array([0.0, 0.0, 0.0]))
                normales = np.asarray(pcd_voxel.normals)

                # Eliminar aquellas cuya magnitud proyectada en XZ sea < 0.6
                normales_xz = normales[:, [0, 2]]
                magnitudes_xz = np.linalg.norm(normales_xz, axis=1)
                normales_xz = normales_xz[magnitudes_xz >= 0.6]
                # Ángulo en el plano XZ respecto al eje Z [atan2(x, z): +Z -> 0º, +X -> 90º, -Z -> 180º, -X -> 270º]
                angulos = np.degrees(np.arctan2(normales_xz[:, 0], normales_xz[:, 1]))
                angulos[angulos < 0] += 360.0
                
                # Histograma [0, 360] en bins de 10 grados
                bins = np.arange(0, 361, 10)
                hist, edges = np.histogram(angulos, bins=bins)
                
                idx_ordenados = np.argsort(hist)[::-1]
                idx_max = idx_ordenados[0]
                idx_segundo = idx_ordenados[1]
                distancia_bins = abs(idx_max - idx_segundo)
                
                centro_max = (edges[idx_max] + edges[idx_max + 1]) / 2
                angulo_max = -centro_max    # Si los bins máximos están muy separados, el ángulo es negativo. Si no, es la media
                mask_intervalo = (angulos >= edges[idx_max]) & (angulos < edges[idx_max+1])
                
                if distancia_bins <= self.max_bins_separacion + 1:
                    centro_segundo = (edges[idx_segundo] + edges[idx_segundo + 1]) / 2
                    peso_max = hist[idx_max]
                    peso_segundo = hist[idx_segundo]
                    
                    angulo_max = (centro_max * peso_max + centro_segundo * peso_segundo ) / (peso_max + peso_segundo)
                    
                    mask_intervalo = ((angulos >= edges[idx_max]) & (angulos < edges[idx_max+1]) | (angulos >= edges[idx_segundo]) & (angulos < edges[idx_segundo+1]))

                
                # Punto más cercano al origen dentro del intervalo dominante
                points_filtrados = np.asarray(pcd_voxel.points)
                points_filtrados = points_filtrados[magnitudes_xz >= 0.6]
                
                puntos_intervalo = points_filtrados[mask_intervalo]

                tol_centro = 0.1   # Error de decímetro.

                mask_centro = ((np.abs(puntos_intervalo[:, 0]) <= tol_centro) & (np.abs(puntos_intervalo[:, 1]) <= tol_centro))
                mask_normal_170_180 = ((angulos[mask_intervalo] >= 175.0) &(angulos[mask_intervalo] <= 185.0))
                mask_punto_00 = mask_centro & mask_normal_170_180
                
                punto_mas_cercano = None
                # Se elije el punto del centro de la cámara si existe, y si su normal está a 180º y pertenece a la máscara
                if np.any(mask_punto_00):
                    puntos_00 = puntos_intervalo[mask_punto_00]
                    distancias_especiales = np.linalg.norm(puntos_00, axis=1)
                    idx_especial = np.argmin(distancias_especiales)
                    punto_mas_cercano = puntos_00[idx_especial]
                    
                    self.get_logger().info(f"Punto Cercano")
                    
                else:   # Si no, se elije el punto más cercano que cumpla la máscara
                    distancias = np.linalg.norm(puntos_intervalo, axis=1)
                    idx_punto_cercano = np.argmin(distancias)
                    punto_mas_cercano = puntos_intervalo[idx_punto_cercano]
                
                # Pasar punto cercano y ángulo
                msg = Float64MultiArray()
                msg.data = [punto_mas_cercano[2], -punto_mas_cercano[0], -punto_mas_cercano[1], angulo_max]
                self.publicador_control.publish(msg)
                
                if self.flag_mostrar_imagenes:
                    plt.hist(angulos, bins=bins)
                    plt.title("Histograma de normales proyectadas en plano XZ")
                    
                    plt.pause(1.0)
                    plt.clf()
    
    def enviar_distancia_cercana(self):
        
        if (self.img_der is not None) and (self.img_izq is not None):
            im_left = cv.cvtColor(self.img_izq, cv.COLOR_BGR2GRAY)
            im_right = cv.cvtColor(self.img_der, cv.COLOR_BGR2GRAY)
            
            profundidad = self.im2prof(im_left, im_right, False) # Debido a que si no obtiene distancias cercanas cuando en realidad son lejanas (cielo y suelo mala textura)
            ########## ASHFBJHBSAFJBA
            # Esto hay q verlo porque "True" a veces es demasiado restrictivo y las zonas donde hay poca textura las pone como NaN así que la distancia es máxima en vez de la real.
            profundidad_central = profundidad[200:300, 200:400]
        
            valores_profundidades, numero_pixeles = np.unique(profundidad_central, return_counts=True)
            mayor_profundidad = valores_profundidades[np.argmax(numero_pixeles)]
            
            if np.isnan(mayor_profundidad):
                mayor_profundidad = self.distancia_maxima_permitida
            
            self.get_logger().info(f"La profundidad máxima es :{mayor_profundidad}")
            
            msg = Float64MultiArray()
            msg.data = [0.0, 0.0, mayor_profundidad, 0.0]
            self.publicador_control.publish(msg)
            
            if self.flag_mostrar_imagenes:
                plt.subplot(1,2,1)
                plt.imshow(profundidad)
                plt.title("Profundidad")
                
                plt.subplot(1,2,2)
                plt.imshow(profundidad_central)
                plt.title("Profundidad Central")
                    
                plt.pause(1.0)
                plt.clf()
    
    def pub_nube_puntos(self):
         if (self.img_der is not None) and (self.img_izq is not None):
            
            im_left = cv.cvtColor(self.img_izq, cv.COLOR_BGR2GRAY)
            im_right = cv.cvtColor(self.img_der, cv.COLOR_BGR2GRAY)
                      
            profundidad = self.im2prof(im_left, im_right, True)
            
            pcd = self.depth2pc(self.img_izq, profundidad)

            if (pcd is not None) and (len(pcd.points) > 0):
                # Pasar nube de puntos a mensaje ROS2
                pcd_msg = self.open3d_to_pointcloud2(pcd)
                
                if (pcd_msg is not None):
                    self.pub_pc.publish(pcd_msg)
                    self.get_logger().info("Nube de puntos publicada")
                    self.flag_publicado = True
                    
            # Cuando envíe la nube, le digo al dron q se mueva.
            msg = Float64MultiArray()
            msg.data = [0.0, 0.0, 0.0, 1.0]
            self.publicador_control.publish(msg)
                        
                        
    def pub_flags(self):
        if (self.img_der is not None) and (self.img_izq is not None):
            im_left = cv.cvtColor(self.img_izq, cv.COLOR_BGR2GRAY)
            im_right = cv.cvtColor(self.img_der, cv.COLOR_BGR2GRAY)
            
            prof = self.im2prof(im_left, im_right, False)
            prof = prof[200:300, 200:]   # 200: offset debido a estereo (ponen NaN a la izquierda)

            plt.subplot(1,1,1)
            plt.imshow(prof)
            plt.title("Profundidad Región")
                
            plt.pause(1.0)
            plt.clf()
            
            # Dividir en dos mitades: izquierda [0:(640-offset/2)] y derecha [(640-offset/2):(640-offset)]
            # profundidad_central -> disparidad -> prof
            mitad = (640 - 200) // 2
            region_izquierda = prof[:, :mitad-1] 
            region_derecha   = prof[:, mitad:]

            msg = Float64MultiArray()
            x = 0.0
            y = 0.0

            # -------------------------
            # FLAG Y:
            # y = 1 si en la izquierda la mayoría son NaN
            #      y en la derecha la mayoría NO son NaN
            # -------------------------
            n_total_izq = region_izquierda.size
            n_total_der = region_derecha.size

            n_nan_izq = np.isnan(region_izquierda).sum()
            n_nan_der = np.isnan(region_derecha).sum()

            mayoria_nan_izq = n_nan_izq > (n_total_izq / 2)
            mayoria_no_nan_der = (n_total_der - n_nan_der) > (n_total_der / 2)

            if mayoria_nan_izq and mayoria_no_nan_der:
                y = 1.0

            # -------------------------
            # FLAG X:
            # x = 1 si |valor mayoritario izquierda - valor mayoritario derecha| > umbral
            # usando solo valores NO NaN
            # -------------------------
            valores_izq = region_izquierda[~np.isnan(region_izquierda)]
            valores_der = region_derecha[~np.isnan(region_derecha)]

            if valores_izq.size > 0 and valores_der.size > 0 and not(mayoria_nan_izq):
                # Redondear para agrupar profundidades parecidas
                valores_izq = np.round(valores_izq, 2)
                valores_der = np.round(valores_der, 2)

                vals_izq, counts_izq = np.unique(valores_izq, return_counts=True)
                vals_der, counts_der = np.unique(valores_der, return_counts=True)

                mayoritario_izq = vals_izq[np.argmax(counts_izq)]
                mayoritario_der = vals_der[np.argmax(counts_der)]

                if abs(mayoritario_izq - mayoritario_der) > self.umbral_z_planos:
                    x = 1.0

            msg.data = [x, y, 0.0, 0.0]
            self.publicador_control.publish(msg)
                    
            
def main(args=None):
    rclpy.init(args=args) 
    
    objeto_nodo = Clase_Subscriber() 
    rclpy.spin(objeto_nodo) 
    
    rclpy.shutdown() 

if __name__ == '__main__':
    main()
    
    
    
    
    
    
    
# LA IDEA ES MEJORAR EL MAPA DE PROFUNDIDAD PARA QUE NO TENGA ERRORES CON NANs (NO TIENE Q SER PERFECTO, SOLO ESOS ERRORES)