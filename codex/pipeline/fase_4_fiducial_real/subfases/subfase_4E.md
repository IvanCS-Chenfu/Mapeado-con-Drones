# Subfase 4E — Contrato ROS 2 y publicación de observaciones visuales por KeyFrame

## Estado

```text
CONSEGUIDA — implementada, compilada y validada el 2026-08-25
```

## Condición previa

Cumplida: 4C aporta la identidad exacta del KF y 4D las observaciones internas
del worker. La preparación y autorización de 4E+4F quedaron cerradas antes de
la implementación.

## Objetivo

Crear el contrato ROS 2 definitivo para enviar desde cada wrapper al Servidor, por un **topic independiente de `orb_map_delta`**, todas las marcas visuales detectadas en un KF exacto.

Regla principal:

```text
KF sin tags válidos -> NO publicar mensaje fiducial
KF con 1 tag       -> 1 batch con 1 observation
KF con N tags      -> 1 batch con N observations
```

No se enviará un mensaje por tag ni un batch vacío por KF. El objetivo es mantener tráfico pequeño y semántica clara.

## Separación de responsabilidades

El mensaje representa percepción geométrica, no semántica de objetos.

El wrapper puede enviar:

```text
tag_id
camera_T_tag
quality_score
métricas originales de calidad
identidad exacta del KF
```

El wrapper NO puede enviar ni decidir:

```text
object_id / fiducial_id lógico
cara del cubo
object_T_tag
world_T_object
world_T_camera
fiducial seleccionado
anchor/revisit/optimization
GT
```

## Topic y QoS

```text
/dron_X/orbslam/fiducial_keyframe_observations
reliable + volatile + KeepLast(32)
```

Cada wrapper publica en el namespace de su dron. El batch tiene valor funcional:
el QoS no sustituye la sincronización fuera de orden de 4F.

## Interfaces propuestas

Salvo que la búsqueda estática encuentre nombres equivalentes ya aceptados:

```text
orbslam3_msgs/msg/FiducialTagObservation.msg
orbslam3_msgs/msg/FiducialKeyFrameObservations.msg
```

### FiducialTagObservation

Contenido conceptual recomendado:

```text
uint32 tag_id
geometry_msgs/Transform camera_T_tag
float64 quality_score
float64 reprojection_error_px
float64 tag_area_px2
float64 pose_ambiguity
```

Las métricas y `quality_score` calculadas por 4D se serializan sin reinterpretarlas
ni recalcular PnP en el callback ROS.

### Semántica de `quality_score`

- escala recomendada `[0,1]` con `1=mejor`, salvo razón técnica documentada;
- determinista para una misma detección/configuración;
- derivada de métricas explicables, no de un valor mágico;
- puede considerar error de reproyección, área aparente, oblicuidad/normal del tag, separación entre soluciones IPPE y otros datos disponibles;
- conserva exactamente la fórmula vigente y explicable de 4D.

La distancia no necesita duplicarse como campo si puede calcularse de `camera_T_tag`. El servidor podrá calcular `norm(translation)`.

### FiducialKeyFrameObservations

Contenido conceptual recomendado:

```text
std_msgs/Header header
uint32 drone_id
string drone_name
uint64 map_epoch
uint64 local_keyframe_id
uint64 source_frame_id
orbslam3_msgs/FiducialTagObservation[] observations
```

`header.stamp` es el único timestamp canónico y representa exactamente el del
KF tras usar el mismo helper de conversión temporal que `OrbKeyFrame`.
`header.frame_id` identifica el frame óptico efectivo usado para PnP y
`source_frame_id` conserva la identidad numérica del frame fuente.

## Dos copias de orbslam3_msgs

Fase 2 permite exactamente las copias Dron y Servidor y establece Servidor como canónico para la guarda. 4E debe añadir las mismas interfaces en ambas copias y ampliar/usar las guardas para comparar `.msg`, `.srv`, CMake, `package.xml` y cualquier auxiliar relevante.

No crear una tercera copia.

## Integración con el worker 4D

El worker produce un resultado inmutable asociado a:

```text
{drone_id, map_epoch, local_keyframe_id, source_frame_id, keyframe_stamp}
```

más el vector de tags.

4E transforma ese resultado en mensaje ROS. No vuelve a abrir la imagen, no repite PnP y no consulta mutablemente `map_epoch_` después de terminar el job.

## Publicación y tráfico

Se publica una única vez por resultado no vacío. Solo entran observaciones
`valid=true` y se ordenan de forma determinista por `tag_id`. Tres caras válidas
visibles del mismo cubo producen tres elementos dentro del mismo batch. Las
detecciones rechazadas pueden seguir apareciendo en el debug visual de 4D, pero
no cruzan este contrato funcional.

No se fusiona en Dron para ahorrar bytes, porque ello obligaría a distribuir semántica `tag→fiducial` al Dron y reduciría capacidad de diagnóstico.

## Grafos web

4E crea el extremo de publicación de la arista Dron→Servidor; 4F completa su
recepción funcional.

Actualizar metadata de `orbslam3_msgs`/wrapper con:

- topic;
- tipo;
- dirección;
- namespace;
- QoS;
- datos transportados;
- evidencia live de publicación.

El bloque conjunto 4E+4F actualiza también `pipeline_flow`. La arista completa
wrapper→Servidor solo se considera funcional cuando 4F implementa la recepción;
4E puede mostrar su extremo de publicación sin iluminar un consumo inexistente.
Con ambos debug apagados no debe construirse ni serializarse telemetría
específica de los visualizadores.

## Archivos probables

Rutas a revalidar antes de editar:

```text
dron/orbslam3_msgs/msg/FiducialTagObservation.msg
dron/orbslam3_msgs/msg/FiducialKeyFrameObservations.msg
dron/orbslam3_msgs/CMakeLists.txt
dron/orbslam3_msgs/package.xml
servidor/orbslam3_msgs/msg/FiducialTagObservation.msg
servidor/orbslam3_msgs/msg/FiducialKeyFrameObservations.msg
servidor/orbslam3_msgs/CMakeLists.txt
servidor/orbslam3_msgs/package.xml
dron/orbslam3_ros2/.../stereo-slam-node.*
metadata/documentación system_architecture
```

## Cambios prohibidos

- no modificar `OrbMap`/`OrbKeyFrame` para meter fiduciales;
- no enviar imágenes;
- no enviar GT;
- no enviar `object_id`/pose global;
- no publicar un mensaje por tag como contrato principal;
- no publicar batch vacío;
- no publicar observaciones `valid=false`;
- no hacer anchor en Dron;
- no ejecutar detección de nuevo en el callback de publicación.

## Pruebas

### 1. Un tag
Un KF con un tag válido produce exactamente un batch y un elemento.

### 2. Multi-tag
Un KF que ve 2–3 caras/tags produce un solo batch con todos los IDs/poses/calidades.

### 3. Cero tags
Se registrará internamente el KF procesado en debug si procede, pero no aparecerá mensaje en el topic fiducial.

### 4. Identidad
Comparar evento 4C, job 4D y mensaje 4E: mismo epoch/KF/frame/timestamp, sin
timestamp duplicado y con conversión temporal exacta compartida.

### 5. Interface duplicada
Guardas Dron/Servidor verdes y ninguna tercera copia.

### 6. Tráfico
Contar batches, tags y bytes aproximados para disponer de baseline; todavía no optimizar prematuramente.

### 7. Orden y validación
Comprobar orden ascendente por `tag_id`, batch no vacío, IDs únicos, valores
finitos, `quality_score` en `[0,1]` y quaternion normalizado.

### 8. Integración conjunta 4E+4F
Ejecutar la trayectoria típica completa con Gazebo y RViz2, ambos grafos web
activos y ventanas de debug fiducial desactivadas.

## Logs recomendados

```text
FID-BATCH-PUB drone=<id> epoch=<e> kf=<id> tags=<N> bytes=<aprox>
```

No imprimir matrices completas por defecto.

## Criterio de éxito

- interface generada en ambas copias;
- publicación no bloqueante;
- un batch por KF solo si hay detecciones;
- solo observaciones válidas, únicas y ordenadas;
- todas las observaciones de ese KF preservadas;
- calidad explicable;
- identidad exacta;
- cero semántica de fiducial lógico/global en Dron;
- `system_architecture`, `pipeline_flow` y guardas actualizados.
