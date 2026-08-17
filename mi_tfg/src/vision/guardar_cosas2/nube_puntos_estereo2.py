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
    
        ###################
        # ------------------------------------------------------------------
        # Parámetros para regularización planar de la nube de puntos
        # ------------------------------------------------------------------
        
        # Tamaño del voxel para reducir densidad y ruido antes de segmentar.
        # Cuanto mayor sea, más robusto y rápido será el algoritmo, pero
        # perderás detalle fino.
        self.voxel_size_planar = 0.05
        
        # Parámetros del filtrado estadístico de outliers.
        # nb_neighbors: vecinos usados para estimar si un punto es raro.
        # std_ratio: cuanto menor sea, más agresivo será el filtrado.
        self.nb_neighbors_outlier = 20
        self.std_ratio_outlier = 1.5
        
        # Radio usado para estimar normales en Open3D.
        # Debe ser suficientemente grande para estabilizar la normal, pero
        # no tanto como para mezclar superficies diferentes.
        self.radio_normales = 0.12
        
        # Número máximo de vecinos usados al estimar normales.
        self.max_nn_normales = 30
        
        # Radio de búsqueda de vecinos durante el crecimiento de regiones.
        # Define hasta dónde puede "crecer" una pared.
        self.radio_crecimiento = 0.12
        
        # Umbral máximo de distancia lateral entre dos puntos para considerar
        # que pertenecen a la misma superficie. Esta es la parte importante:
        # penalizamos mucho diferencias laterales.
        self.umbral_distancia_lateral = 0.08
        
        # Umbral máximo de separación en la dirección de visión de la cámara.
        # Aquí permitimos más error porque el estéreo suele equivocarse justo
        # en profundidad.
        self.umbral_distancia_profundidad = 0.35
        
        # Umbral angular máximo entre normales (en grados) para agrupar puntos
        # en una misma región.
        self.umbral_angulo_normal = 18.0
        
        # Umbral máximo de distancia euclídea entre colores RGB normalizados
        # (cada canal en [0,1]) para considerar que dos puntos son similares.
        self.umbral_color = 0.25
        
        # Número mínimo de puntos para aceptar una región como candidata a pared.
        self.min_puntos_region = 60
        
        # Umbral de distancia punto-plano usado en RANSAC.
        self.umbral_ransac_plano = 0.04
        
        # Iteraciones de RANSAC para ajustar el plano robustamente.
        self.iteraciones_ransac = 250
        
        # Número mínimo de inliers del plano dentro de una región para aceptar
        # que realmente es una pared/plano consistente.
        self.min_inliers_plano = 40
        
        # Error medio máximo permitido respecto al plano refinado.
        # Si una región ajusta mal, no se proyecta.
        self.error_medio_max_plano = 0.025
        
        # Cociente máximo de planaridad.
        # Se calcula con los autovalores de la covarianza de los puntos:
        # lambda_min / suma_lambdas. Cuanto más pequeño, más plano es el parche.
        self.planaridad_max = 0.02
        
        # Dirección de visión de la cámara expresada en el frame "cuerpo".
        # Según tu transformación:
        #   eje Z de cámara  -> eje X del cuerpo
        # por tanto "mirar hacia delante" equivale aproximadamente a +X.
        self.view_dir_body = np.array([1.0, 0.0, 0.0], dtype=np.float64)
        
        # Origen de la cámara en el frame del cuerpo.
        # Se usa para orientar las normales hacia la cámara y que sean coherentes.
        self.camera_origin_body = np.array(self.camara2cuerpo, dtype=np.float64)
        ###################
        
    
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


    ###################
    def angulo_entre_normales(self, n1, n2):
        # ------------------------------------------------------------------
        # Devuelve el ángulo en grados entre dos normales unitarias.
        # Se usa valor absoluto en el producto escalar para ser robustos
        # ante pequeñas inconsistencias de orientación.
        # ------------------------------------------------------------------
        dot_val = np.clip(np.abs(np.dot(n1, n2)), 0.0, 1.0)
        return np.degrees(np.arccos(dot_val))


    def distancia_lateral_y_profundidad(self, p1, p2, view_dir):
        # ------------------------------------------------------------------
        # Descompone el vector entre dos puntos en:
        #   - componente paralela a la dirección de visión (profundidad),
        #   - componente perpendicular a la visión (lateral).
        #
        # Esto es clave para tu problema:
        #   - se tolera más error en profundidad,
        #   - se exige mucha coherencia lateral para unir puntos.
        # ------------------------------------------------------------------
        d = p2 - p1
        
        # Componente paralela a la dirección de visión
        d_par_mod = np.dot(d, view_dir)
        d_par = d_par_mod * view_dir
        
        # Componente perpendicular a la dirección de visión
        d_perp = d - d_par
        
        # Distancia lateral (magnitud perpendicular a la cámara)
        distancia_lateral = np.linalg.norm(d_perp)
        
        # Distancia de profundidad (magnitud a lo largo de la cámara)
        distancia_profundidad = np.abs(d_par_mod)
        
        return distancia_lateral, distancia_profundidad


    def refinar_plano_svd(self, puntos):
        # ------------------------------------------------------------------
        # Refina un plano a partir de un conjunto de puntos usando SVD/PCA:
        #   1) calcula el centroide,
        #   2) calcula covarianza,
        #   3) el autovector asociado al menor autovalor es la normal.
        #
        # Devuelve:
        #   normal_unitaria, d_plano, centroide, ratio_planaridad
        #
        # Ecuación del plano:
        #   n.x + d = 0
        # ------------------------------------------------------------------
        if puntos.shape[0] < 3:
            return None, None, None, None
        
        # Centroide del conjunto
        centroide = np.mean(puntos, axis=0)
        
        # Puntos centrados
        puntos_centrados = puntos - centroide
        
        # Covarianza 3x3
        cov = np.cov(puntos_centrados.T)
        
        # Autovalores y autovectores
        eigvals, eigvecs = np.linalg.eigh(cov)
        
        # Ordenar por autovalor creciente
        orden = np.argsort(eigvals)
        eigvals = eigvals[orden]
        eigvecs = eigvecs[:, orden]
        
        # La normal del plano es la dirección de menor varianza
        normal = eigvecs[:, 0]
        normal = normal / (np.linalg.norm(normal) + 1e-12)
        
        # Término independiente del plano
        d = -np.dot(normal, centroide)
        
        # Medida simple de planaridad:
        # si el menor autovalor es muy pequeño comparado con la suma,
        # el conjunto es muy plano.
        ratio_planaridad = eigvals[0] / (np.sum(eigvals) + 1e-12)
        
        return normal, d, centroide, ratio_planaridad


    def proyectar_puntos_a_plano(self, puntos, normal, d):
        # ------------------------------------------------------------------
        # Proyecta todos los puntos sobre el plano:
        #   p' = p - (n.p + d) * n
        #
        # Asume que la normal está normalizada.
        # ------------------------------------------------------------------
        distancias_firmadas = np.dot(puntos, normal) + d
        puntos_proyectados = puntos - distancias_firmadas[:, None] * normal[None, :]
        return puntos_proyectados


    def region_growing_planar(self, puntos, colores, normales, pcd):
        # ------------------------------------------------------------------
        # Segmenta la nube en regiones que probablemente pertenecen a una misma
        # pared usando crecimiento de regiones con estas condiciones:
        #
        #   1) cercanía espacial local (por radio),
        #   2) distancia lateral pequeña,
        #   3) diferencia en profundidad tolerada pero limitada,
        #   4) normales similares,
        #   5) colores similares.
        #
        # Devuelve una lista de listas de índices, cada una una región.
        # ------------------------------------------------------------------
        n_puntos = puntos.shape[0]
        
        if n_puntos == 0:
            return []
        
        # Asegurar que la dirección de visión es unitaria
        view_dir = self.view_dir_body / (np.linalg.norm(self.view_dir_body) + 1e-12)
        
        # Estructura KD-Tree de Open3D para buscar vecinos rápidamente
        kdtree = o3d.geometry.KDTreeFlann(pcd)
        
        # Vector de visitados para no procesar un punto varias veces
        visitado = np.zeros(n_puntos, dtype=bool)
        
        regiones = []
        
        # Recorremos todos los puntos y arrancamos una región nueva desde
        # cada punto no visitado
        for idx_seed in range(n_puntos):
            if visitado[idx_seed]:
                continue
            
            # Cola BFS para crecimiento de región
            cola = [idx_seed]
            visitado[idx_seed] = True
            
            # Índices pertenecientes a la región actual
            region_actual = []
            
            while cola:
                idx_actual = cola.pop(0)
                region_actual.append(idx_actual)
                
                p_actual = puntos[idx_actual]
                n_actual = normales[idx_actual]
                c_actual = colores[idx_actual]
                
                # Vecinos del punto actual dentro de un radio
                _, idxs_vecinos, _ = kdtree.search_radius_vector_3d(p_actual, self.radio_crecimiento)
                
                for idx_vecino in idxs_vecinos:
                    if visitado[idx_vecino]:
                        continue
                    
                    p_vecino = puntos[idx_vecino]
                    n_vecino = normales[idx_vecino]
                    c_vecino = colores[idx_vecino]
                    
                    # Distancia lateral y en profundidad respecto a la cámara
                    dist_lateral, dist_profundidad = self.distancia_lateral_y_profundidad(
                        p_actual, p_vecino, view_dir
                    )
                    
                    # Diferencia angular entre normales
                    angulo = self.angulo_entre_normales(n_actual, n_vecino)
                    
                    # Diferencia de color RGB normalizado
                    dist_color = np.linalg.norm(c_actual - c_vecino)
                    
                    # Criterios para unir dos puntos en la misma región:
                    #   - muy parecidos lateralmente,
                    #   - normales parecidas,
                    #   - color parecido,
                    #   - permitiendo más error en profundidad que lateral.
                    if (
                        dist_lateral <= self.umbral_distancia_lateral and
                        dist_profundidad <= self.umbral_distancia_profundidad and
                        angulo <= self.umbral_angulo_normal and
                        dist_color <= self.umbral_color
                    ):
                        visitado[idx_vecino] = True
                        cola.append(idx_vecino)
            
            # Solo aceptamos regiones con suficiente tamaño
            if len(region_actual) >= self.min_puntos_region:
                regiones.append(region_actual)
        
        return regiones


    def regularizar_paredes(self, pcd):
        # ------------------------------------------------------------------
        # Pipeline completo de regularización planar:
        #
        #   1) voxel downsample para reducir ruido y coste,
        #   2) eliminación de outliers,
        #   3) estimación y orientación de normales,
        #   4) segmentación por region growing,
        #   5) ajuste robusto de plano con RANSAC por región,
        #   6) refinado del plano con SVD,
        #   7) proyección de puntos sobre el plano.
        #
        # Devuelve una nueva nube regularizada.
        # ------------------------------------------------------------------
        if pcd is None or len(pcd.points) < 30:
            return pcd
        
        # Copia para no alterar el objeto original por seguridad
        pcd_proc = o3d.geometry.PointCloud(pcd)
        
        # --------------------------------------------------------------
        # 1) Downsampling por voxel
        # Reduce densidad, mejora robustez y acelera el procesamiento.
        # --------------------------------------------------------------
        pcd_proc = pcd_proc.voxel_down_sample(voxel_size=self.voxel_size_planar)
        
        if len(pcd_proc.points) < 30:
            return pcd_proc
        
        # --------------------------------------------------------------
        # 2) Eliminación estadística de outliers
        # Borra puntos aislados incompatibles con su vecindad local.
        # --------------------------------------------------------------
        pcd_proc, _ = pcd_proc.remove_statistical_outlier(
            nb_neighbors=self.nb_neighbors_outlier,
            std_ratio=self.std_ratio_outlier
        )
        
        if len(pcd_proc.points) < 30:
            return pcd_proc
        
        # --------------------------------------------------------------
        # 3) Estimación de normales
        # Cada normal se estima en una vecindad local.
        # --------------------------------------------------------------
        pcd_proc.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=self.radio_normales,
                max_nn=self.max_nn_normales
            )
        )
        
        # --------------------------------------------------------------
        # 4) Orientación coherente de normales hacia la cámara
        # Esto reduce ambigüedades al comparar normales entre puntos.
        # --------------------------------------------------------------
        pcd_proc.orient_normals_towards_camera_location(self.camera_origin_body)
        
        # Convertimos a arrays NumPy para operar cómodamente
        puntos = np.asarray(pcd_proc.points).copy()
        colores = np.asarray(pcd_proc.colors).copy()
        normales = np.asarray(pcd_proc.normals).copy()
        
        if puntos.shape[0] < 30:
            return pcd_proc
        
        # --------------------------------------------------------------
        # 5) Segmentación en regiones candidatas a pared
        # --------------------------------------------------------------
        regiones = self.region_growing_planar(puntos, colores, normales, pcd_proc)
        
        if len(regiones) == 0:
            return pcd_proc
        
        # --------------------------------------------------------------
        # 6) Para cada región: ajuste de plano + proyección
        # --------------------------------------------------------------
        for region in regiones:
            idx_region = np.array(region, dtype=np.int32)
            puntos_region = puntos[idx_region]
            
            if puntos_region.shape[0] < self.min_puntos_region:
                continue
            
            # Crear subnube de la región para RANSAC
            subpcd = o3d.geometry.PointCloud()
            subpcd.points = o3d.utility.Vector3dVector(puntos_region)
            
            # Ajuste robusto del plano con RANSAC.
            # Esto elimina puntos que no pertenecen realmente al plano.
            try:
                plano, inliers_locales = subpcd.segment_plane(
                    distance_threshold=self.umbral_ransac_plano,
                    ransac_n=3,
                    num_iterations=self.iteraciones_ransac
                )
            except:
                continue
            
            if len(inliers_locales) < self.min_inliers_plano:
                continue
            
            # Índices globales de los inliers del plano
            idx_inliers_global = idx_region[np.array(inliers_locales, dtype=np.int32)]
            puntos_inliers = puntos[idx_inliers_global]
            
            # Refinar plano con SVD sobre todos los inliers
            normal_ref, d_ref, centroide, ratio_planaridad = self.refinar_plano_svd(puntos_inliers)
            
            if normal_ref is None:
                continue
            
            # Rechazar regiones poco planas
            if ratio_planaridad > self.planaridad_max:
                continue
            
            # Calcular el error medio absoluto al plano refinado
            distancias = np.abs(np.dot(puntos_inliers, normal_ref) + d_ref)
            error_medio = np.mean(distancias)
            
            if error_medio > self.error_medio_max_plano:
                continue
            
            # Proyección de los inliers sobre el plano refinado
            puntos_proyectados = self.proyectar_puntos_a_plano(puntos_inliers, normal_ref, d_ref)
            
            # Sustituimos en la nube procesada únicamente esos puntos
            puntos[idx_inliers_global] = puntos_proyectados
        
        # --------------------------------------------------------------
        # 7) Volcar los puntos regularizados de vuelta a la nube
        # --------------------------------------------------------------
        pcd_proc.points = o3d.utility.Vector3dVector(puntos)
        
        # Recalcular normales finales para que la nube publicada quede coherente
        pcd_proc.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=self.radio_normales,
                max_nn=self.max_nn_normales
            )
        )
        pcd_proc.orient_normals_towards_camera_location(self.camera_origin_body)
        
        return pcd_proc
    ###################

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
                
                ###################
                # ------------------------------------------------------------------
                # Regularización planar de la nube de puntos:
                # agrupa regiones con color, normal y posición lateral similares,
                # ajusta un plano robusto por región y proyecta los puntos al plano.
                # ------------------------------------------------------------------
                pcd = self.regularizar_paredes(pcd)
                ###################
                
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
            
            plt.subplot(2,2,1)
            plt.imshow(self.img_izq)
            plt.title("Camara Izquierda")

            plt.subplot(2,2,2)
            plt.imshow(self.img_der)
            plt.title("Camara Derecha")
            
            plt.subplot(2,2,3)
            if self.im_disparidad is not None:
                plt.imshow(self.im_disparidad, cmap='gray')
            plt.colorbar()
            plt.title("Disparidad Actual")
            
            plt.subplot(2,2,4)
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