#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo

from cv_bridge import CvBridge
import cv2 as cv
import open3d as o3d

import matplotlib.pyplot as plt
import numpy as np

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
        self.timer_ = self.create_timer(0.5, self.test_profundidad)
        self.funcion_terminada = True
        
        # Datos Necesarios (parámetros cámaras)
        self.baseline = 0.057
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
        
        # Para "enviar_distancia_cercana()"
        self.distancia_maxima_permitida = 5.0
        
        # Para "pub_nube_puntos_flags()"
        self.umbral_z_planos = 1.0
    
    
    ##################### CALBACKS #####################
    
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
        
    
    def im2disp(self, im_left, im_right):
            
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
        
        return disparidad

    
    
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
    """
    def secciones_profundidad(self, im_gray, im_depth, diff_umbral=20, var_umbral=20, min_size=100):

        gx = cv.Sobel(im_depth, cv.CV_32F, 1, 0, ksize=3)
        gy = cv.Sobel(im_depth, cv.CV_32F, 0, 1, ksize=3)
        grad = np.sqrt(gx**2 + gy**2)

        umbral_grad = 0.5
        im_depth[grad > umbral_grad] = np.nan
        
        dmin = np.nanmin(im_depth)
        dmax = np.nanmax(im_depth)
        depth_norm = 255 * (im_depth - dmin) / (dmax - dmin)
        depth_norm = depth_norm.astype(np.uint8)

        gx = cv.Sobel(im_gray, cv.CV_32F, 1, 0, ksize=3)
        gy = cv.Sobel(im_gray, cv.CV_32F, 0, 1, ksize=3)
        grad = cv.magnitude(gx, gy)

        h, w = depth_norm.shape
        visitado = np.zeros((h, w), dtype=bool)

        # Imagen donde se pintarán todas las regiones
        img_todas = np.zeros((h, w, 3), dtype=np.uint8)

        # Imagen donde solo se pintarán regiones de baja textura
        img_baja_textura = np.zeros((h, w, 3), dtype=np.uint8)

        vecinos = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]

        # ---------- 3) Region growing ----------
        for y0 in range(h):
            for x0 in range(w):
                if visitado[y0, x0] or depth_norm[y0, x0] == 0:
                    continue

                cola = [(y0, x0)]
                visitado[y0, x0] = True
                region = [(y0, x0)]

                valor_min = int(depth_norm[y0, x0])
                valor_max = int(depth_norm[y0, x0])

                while cola:
                    y, x = cola.pop(0)

                    for dy, dx in vecinos:
                        ny, nx = y + dy, x + dx

                        if ny < 0 or ny >= h or nx < 0 or nx >= w:
                            continue
                        if visitado[ny, nx] or depth_norm[ny, nx] == 0:
                            continue

                        v = int(depth_norm[ny, nx])
                        nuevo_min = min(valor_min, v)
                        nuevo_max = max(valor_max, v)

                        if nuevo_max - nuevo_min <= diff_umbral:
                            visitado[ny, nx] = True
                            cola.append((ny, nx))
                            region.append((ny, nx))
                            valor_min = nuevo_min
                            valor_max = nuevo_max

                # ignorar regiones pequeñas
                if len(region) < min_size:
                    continue

                # ---------- 4) Crear máscara de la región ----------
                mask = np.zeros((h, w), dtype=np.uint8)
                for y, x in region:
                    mask[y, x] = 255

                # ---------- 5) Calcular textura ----------
                valores_textura = grad[mask > 0]
                varianza = np.var(valores_textura)

                # ---------- 6) Pintar región ----------
                color = np.random.randint(50, 255, 3).tolist()
                img_todas[mask > 0] = color

                if varianza < var_umbral:
                    img_baja_textura[mask > 0] = color
        
        return depth_norm, img_todas, img_baja_textura
    """

    def region_grow(self, seed, allow, kernel):
        prev = np.zeros_like(seed)
        curr = seed.copy()
        while True:
            dil = cv.dilate(curr, kernel)
            curr = cv.bitwise_and(dil, allow)
            if np.array_equal(curr, prev):
                break
            prev = curr.copy()
        return curr
    
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


    def test_profundidad(self):
        if (self.img_der is not None) and (self.img_izq is not None) and self.funcion_terminada:
            self.funcion_terminada = False
            
            im_left = cv.cvtColor(self.img_izq, cv.COLOR_BGR2GRAY)
            im_right = cv.cvtColor(self.img_der, cv.COLOR_BGR2GRAY)
            
            disparidad_izq = self.objeto_estereo_izq.compute(im_left, im_right)
            disparidad_der = self.objeto_estereo_der.compute(im_right, im_left)
            
            filtered_disp = self.wls_filter.filter(disparidad_izq, self.img_izq, disparity_map_right=disparidad_der)
            filtered_disp = filtered_disp.astype(np.float32) / 16.0
            filtered_disp[filtered_disp < 0] = np.nan
            
            mask = filtered_disp > 0  # solo valores válidos

            profundidad_filtrada = np.zeros_like(filtered_disp, dtype=np.float32)
            profundidad_filtrada[mask] = self.fx * self.baseline / filtered_disp[mask]
            
            profundidad_filtrada[profundidad_filtrada > self.profundidad_maxima] = np.nan
            profundidad_filtrada[profundidad_filtrada < 0.1] = np.nan
            
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
            mask_corte[np.isnan(filtered_disp)] = 0  
            
            kernel_elipse = cv.getStructuringElement(cv.MORPH_ELLIPSE, (5,5))
            # Eliminar zonas blancas pequeñas
            mask_text_erode = cv.erode(mask_corte, kernel_elipse)
            mask_textura_Ablancas = self.eliminar_areas_cerradas_grandes(mask_text_erode, 3000, False)
            
            # Eliminar zonas negras pequeñas
            mask_nans = np.isnan(filtered_disp).astype(np.uint8)
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
            
            rgb_pub = cv.cvtColor(self.img_izq, cv.COLOR_BGR2RGB)
            rgb_pub[mask_textura_areas == 0.0] = (255, 0, 0)
            
            plt.subplot(3,5,1)
            plt.imshow(im_left, cmap="gray")
            plt.title("Imagen Izquierda")
            
            plt.subplot(3,5,2)
            plt.imshow(im_right, cmap="gray")
            plt.title("Imagen Derecha")
            
            plt.subplot(3,5,6)
            plt.imshow(disparidad_izq, cmap="gray")
            plt.title("Disparidad Izq sin Tratar")
            
            plt.subplot(3,5,7)
            plt.imshow(disparidad_der, cmap="gray")
            plt.title("Disparidad Der sin Trata")
            
            plt.subplot(3,5,11)
            plt.imshow(filtered_disp, cmap="gray")
            plt.title("Disparidad Filtrada Izq-Der")
            
            plt.subplot(3,5,12)
            plt.imshow(profundidad_filtrada, cmap="gray")
            plt.title("Profundidad Filtrada Izq-Der")
            
            plt.subplot(3,5,3)
            plt.imshow(mask_bordes_prof, cmap="gray")
            plt.title("mask_bordes_prof")
            
            plt.subplot(3,5,4)
            plt.imshow(mask_bordes_gray, cmap="gray")
            plt.title("mask_bordes_gray")
            
            plt.subplot(3,5,5)
            plt.imshow(mask_bordes, cmap="gray")
            plt.title("mask_bordes")
            
            plt.subplot(3,5,8)
            plt.imshow(mask_textura, cmap="gray")
            plt.title("mask_textura")

            plt.subplot(3,5,9)
            plt.imshow(mask_corte, cmap="gray")
            plt.title("mask_corte")
            
            plt.subplot(3,5,10)
            plt.imshow(mask_textura_Ablancas, cmap="gray")
            plt.title("mask_textura_Anegras")
            
            plt.subplot(3,5,13)
            plt.imshow(mask_textura_areas, cmap="gray")
            plt.title("mask_textura_areas")
            
            plt.subplot(3,5,14)
            plt.imshow(profundidad_textura_areas, cmap="gray")
            plt.title("profundidad_textura_areas")

            plt.subplot(3,5,15)
            plt.imshow(rgb_pub)
            plt.title("rgb_pub")
            
            plt.pause(0.001)
            plt.clf()        
            
            pcd = self.depth2pc(self.img_izq, profundidad_textura_areas)

            if (pcd is not None) and (len(pcd.points) > 0):
                # Pasar nube de puntos a mensaje ROS2
                pcd_msg = self.open3d_to_pointcloud2(pcd)
                
                if (pcd_msg is not None):
                    #self.pub_pc.publish(pcd_msg)
                    #self.get_logger().info("Nube de puntos publicada")
                    self.flag_publicado = True
                    
            self.funcion_terminada = True
                    
                    
            
def main(args=None):
    rclpy.init(args=args) 
    
    objeto_nodo = Clase_Subscriber() 
    rclpy.spin(objeto_nodo) 
    
    rclpy.shutdown() 

if __name__ == '__main__':
    main()
    
    
    
    
    
    
    
# LA IDEA ES MEJORAR EL MAPA DE PROFUNDIDAD PARA QUE NO TENGA ERRORES CON NANs (NO TIENE Q SER PERFECTO, SOLO ESOS ERRORES)