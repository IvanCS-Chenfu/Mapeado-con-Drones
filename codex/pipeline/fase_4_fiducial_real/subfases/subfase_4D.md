# Subfase 4D — Detección de tags y estimación de pose en el wrapper

## Estado

```text
sin hacer
```

## Dependencia

`4A`, `4B` y `4C`.

## Objetivo técnico

Añadir al wrapper un detector visual que se ejecute únicamente cuando 4C confirme que la imagen actual ha creado un KeyFrame. El detector analizará la imagen izquierda exacta de ese KF, localizará todas las marcas `AprilTag 36h11` visibles y calculará de forma independiente `camera_T_tag` para cada `tag_id` válido.

El wrapper no debe conocer `object_id`, no debe agrupar caras del mismo cubo y no debe fusionar tags de cubos distintos. Su salida interna es una lista plana de observaciones de tag.

## Comportamiento esperado

Para un KF concreto:

```text
0 tags -> vector vacío
1 tag  -> [{tag_id, camera_T_tag, error}]
N tags -> N observaciones independientes ligadas al mismo KF
```

La detección no se ejecuta para frames que no son KF.

## Contexto obligatorio a leer


```text
AGENTS.md
codex/contexto/00_CONTEXTO_COMPACTACION.md
codex/contexto/CONTEXTO_MINIMO_ACTUAL.md
codex/contexto/08_POLITICA_TOKENS_DOCUMENTACION.md
codex/pipeline/PIPELINE_MAESTRO.md
codex/pipeline/fase_4_fiducial_real/pipeline_fase_4_RESUMEN.md
codex/pipeline/fase_4_fiducial_real/pipeline_fase_4.md
```


Además:

```text
subfases/subfase_4A.md
subfases/subfase_4B.md
subfases/subfase_4C.md
wrapper stereo-slam-node.hpp/.cpp vigente
configuración de cámara ORB estéreo vigente
```

## Diagnóstico de partida

El wrapper ya trabaja con `cv::Mat`, `cv_bridge` y la imagen izquierda/derecha. No existe todavía detector fiducial de producción ligado al evento exacto de KF. La información intrínseca `fx,fy,cx,cy,bf,width,height` ya se carga en el wrapper, pero debe comprobarse que corresponde a la imagen final rectificada/redimensionada que se entrega al detector.

## Método baseline acordado

```text
familia: AprilTag 36h11
API de detección: OpenCV aruco/AprilTag
entrada: imagen izquierda exacta del KF
pose: OpenCV solvePnP; preferencia planar SOLVEPNP_IPPE_SQUARE
salida: camera_T_tag por tag
```

Si la versión de OpenCV no dispone de `cv::aruco::ArucoDetector`, se puede usar la API compatible `cv::aruco::detectMarkers` conservando el mismo diccionario y criterios. Cambiar a la librería oficial AprilTag solo se justifica si las pruebas muestran insuficiencia real de OpenCV.

## Archivos permitidos a modificar

Rutas propuestas a localizar en la copia real del wrapper:

```text
ORB_SLAM3_ROS2/src/stereo/stereo-slam-node.hpp
ORB_SLAM3_ROS2/src/stereo/stereo-slam-node.cpp
ORB_SLAM3_ROS2/src/stereo/fiducial_detector.hpp        # nuevo si se separa componente
ORB_SLAM3_ROS2/src/stereo/fiducial_detector.cpp        # nuevo si se separa componente
config local del wrapper para detector fiducial
CMakeLists.txt/package.xml del wrapper
codex/contexto/paquetes/<paquete_wrapper_real>/
```

No es obligatorio usar esos nombres de archivo si el paquete tiene una estructura distinta; la clase detector sí debe quedar separada de la lógica ORB tanto como sea razonable.

## Archivos prohibidos

```text
ORB_SLAM3/src/Tracking.cc                 # no añadir detección aquí
ORB_SLAM3/src/System.cc                   # no añadir detección aquí
src/orbslam3_server/
src/orbslam3_multi/
src/orbslam3_msgs/                        # contrato ROS se hace en 4E
src/simulacion_dron/worlds/               # no codificar detector en el world
```

## Funciones, clases o nodos que hay que localizar

```text
StereoSlamNode::GrabStereo
LoadCameraInfoFromSettings
cv::aruco::getPredefinedDictionary
cv::aruco::ArucoDetector o cv::aruco::detectMarkers
cv::solvePnP / cv::solvePnPGeneric
cv::projectPoints
cv::Rodrigues
```

Crear una abstracción tipo `FiducialDetector` solo si no existe ya una equivalente. No asumir un nombre exacto sin búsqueda previa.

## Cambios requeridos

1. Añadir dependencia de `opencv_aruco`/módulo correspondiente y verificar en configure que `DICT_APRILTAG_36h11` existe.
2. Cargar desde configuración local del wrapper: familia, `tag_size_m`, parámetros de detección, error máximo preliminar y lista/rango de IDs admitidos si se decide filtrarla.
3. Ejecutar el detector solo tras un evento de KF válido de 4C.
4. Usar la imagen izquierda exacta del KF, no una captura posterior ni un frame por timestamp aproximado.
5. Detectar todos los tags de la imagen y conservar cuatro esquinas 2D por tag.
6. Refinar esquinas si la API/versión lo permite y medir el coste por separado.
7. Definir los cuatro puntos 3D del cuadrado con lado `tag_size_m` en un frame `tag` documentado.
8. Resolver PnP en el frame óptico de cámara. Para el cuadrado planar, usar `SOLVEPNP_IPPE_SQUARE` si las pruebas confirman la convención de orden de esquinas; en caso contrario registrar la alternativa usada.
9. Interpretar la salida de PnP como `camera_T_tag`, comprobando signo de Z, ortonormalidad y unidades en metros.
10. Reproyectar los puntos y calcular `reprojection_error_px` RMS o métrica explícitamente documentada.
11. Rechazar poses no finitas, Z no positiva, error excesivo o geometría degenerada.
12. No agrupar por cubo: dos tags del mismo cubo producen dos observaciones independientes.
13. Medir `detect_ms`, `pose_ms`, `total_ms` para cada KF procesado y emitir logs throttled/estructurados.
14. Asegurar que una excepción/fallo del detector devuelve vector vacío y no bloquea ni termina el callback de ORB-SLAM3.
15. Mantener la imagen derecha fuera del baseline funcional. Puede conservarse para una mejora futura, no para el cierre obligatorio.

## Cambios prohibidos

- No detectar en todos los frames.
- No buscar `object_id` en el wrapper.
- No combinar dos caras del mismo cubo.
- No calcular `world_T_camera` en el wrapper.
- No usar GT para validar/rechazar en runtime.
- No forzar un KF cuando se detecta una marca.
- No publicar todavía un mensaje ad-hoc no definido por 4E.
- No duplicar el detector dentro de ORB-SLAM3.

## Paquetes a compilar

```bash
./codex/herramientas/build_selected_packages.sh <paquete_wrapper_real>
```

Si OpenCV/aruco es una dependencia externa no encontrada, registrar el error y detenerse antes de cambiar de familia o instalar otra biblioteca sin acuerdo.

## Pruebas Gazebo requeridas

### Prueba 1 — Tag frontal en un KF

Colocar/recorrer el dron de forma que un tag ocupe una región amplia y frontal. Debe existir un KF con:

```text
FID-DETECT tag_count>=1
reprojection_error_px dentro del umbral acordado
camera_T_tag.z > 0
```

### Prueba 2 — Varias marcas en el mismo KF

Orientar un cubo para que se vean dos caras o disponer dos cubos en la misma imagen. El detector debe devolver todas las marcas, aunque pertenezcan al mismo objeto físico.

### Prueba 3 — Sin fiducial

Generar KFs sin tags visibles. `tag_count=0` debe ser un resultado normal y ORB-SLAM3 debe continuar publicando mapa/pose.

### Prueba 4 — Coste

Registrar una muestra suficiente de `detect_ms/pose_ms/total_ms` y comprobar que el detector solo corre en KFs. No fijar un límite absoluto antes de medir; cualquier degradación clara del tracking se considera fallo de integración.

## Patrones de reducción de logs

```text
KF-EVENT|FID-DETECT|FID-POSE|tag_id|tag_count|reprojection|detect_ms|pose_ms|total_ms|tracking_state|ERROR|FATAL|Segmentation fault|Killed
```

## Criterio de éxito

1. El wrapper compila con OpenCV fiducial.
2. Solo se procesan imágenes confirmadas como KF.
3. Se detectan `0..N` tags sin agrupación por cubo.
4. Cada detección válida produce `tag_id` y `camera_T_tag` finito en metros.
5. El error de reproyección se calcula y queda trazable.
6. KFs sin tag no afectan al SLAM.
7. El coste queda medido y no se observa bloqueo del tracking.
8. No se usa imagen derecha ni GT como requisito para la pose.

## Criterio de fallo o parcial

- `NO CONSEGUIDA`: detección asociada a otro frame, poses con convención incorrecta, detector bloquea tracking o no reconoce los tags baseline.
- `PARCIAL`: detección/pose frontal correctas pero falla multi-tag u oblicuidad básica.
- `BLOQUEADA`: OpenCV disponible no incluye aruco/AprilTag y no hay dependencia alternativa autorizada.

## Riesgos

- matriz K no correspondiente al resize final;
- usar distorsión no nula sobre imagen ya rectificada;
- orden incorrecto de esquinas para IPPE;
- ambigüedad planar con soluciones espejo;
- coste excesivo por refinamiento/configuración demasiado pesada.

## Documentación a actualizar

```text
codex/contexto/paquetes/<paquete_wrapper_real>/
codex/pipeline/fase_4_fiducial_real/historial/por_subfase/historial_4D.md
codex/pipeline/fase_4_fiducial_real/historial/por_subfase/historial_4D_RESUMEN.md
```
