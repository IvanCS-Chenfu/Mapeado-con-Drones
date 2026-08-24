# Subfase 4E — Contrato ROS 2 de observaciones fiduciales por KeyFrame

<!-- ACUERDOS_CIERRE_F2_2026_08_24_START -->
## Impacto obligatorio en system_architecture

> **Vigencia:** acuerdo cerrado el 2026-08-24. Este bloque prevalece sobre cualquier
> frase anterior incompatible del mismo documento. No borra ni reescribe evidencia
> histórica; distingue siempre entre estado actual, deuda conocida y arquitectura objetivo.

Cuando 4E cree/active el batch fiducial, actualizar metadata de `orbslam3_msgs` y
`orbslam3` en `system_architecture`: topic, tipo, patrón de namespace, dirección,
datos transportados y QoS. No iluminar todavía una recepción de Servidor que no exista
hasta 4F. La telemetría de debug del publisher debe ser ligera y quedar completamente
inactiva cuando `system_architecture` esté apagado.
<!-- ACUERDOS_CIERRE_F2_2026_08_24_END -->

## Estado

```text
sin hacer
```

## Dependencia

`4C` y `4D`.

## Objetivo técnico

Definir una interfaz ROS 2 compacta y explícita para transportar, en una única publicación por KF con detecciones, la identidad exacta del KeyFrame y un array de observaciones planas `tag_id + camera_T_tag` calculadas por el wrapper.

El mensaje no debe contener imágenes, Ground Truth, `world_T_fiducial`, agrupación por cubo ni una pose global calculada en el dron.

## Comportamiento esperado

Un KF que ve tres tags produce un mensaje conceptualmente equivalente a:

```text
KF 37
observations[0] = tag 101 + camera_T_tag101
observations[1] = tag 102 + camera_T_tag102
observations[2] = tag 405 + camera_T_tag405
```

Los tags 101/102 pueden pertenecer al mismo cubo; el wrapper no lo indica ni lo resuelve.

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
subfases/subfase_4C.md
subfases/subfase_4D.md
src/orbslam3_msgs/CMakeLists.txt
src/orbslam3_msgs/msg/FiducialObservation.msg
src/orbslam3_msgs/msg/OrbKeyFrame.msg
src/orbslam3_msgs/msg/OrbMap.msg
wrapper vigente
```

## Diagnóstico de partida

Existe `orbslam3_msgs/msg/FiducialObservation.msg`, pero el CMake actual no lo incluye en `rosidl_generate_interfaces` y su contenido está orientado a otro contrato: incluye `world_T_fiducial`, `local_camera_pose`, `confidence` y una sola observación. No es adecuado como contrato final para un KF con `0..N` tags y no debe activarse silenciosamente sin revisar usos.

## Contrato propuesto

Crear, salvo que la búsqueda estática encuentre nombres equivalentes ya aprobados:

```text
msg/FiducialTagObservation.msg
msg/FiducialKeyFrameObservations.msg
```

Contenido mínimo recomendado:

```text
# FiducialTagObservation.msg
uint32 tag_id
geometry_msgs/Transform camera_T_tag
float64 reprojection_error_px
float64 tag_area_px2
```

```text
# FiducialKeyFrameObservations.msg
std_msgs/Header header          # stamp = KF stamp; frame_id = camera optical frame
uint32 drone_id
string drone_name
uint64 map_epoch
uint64 local_keyframe_id
uint64 source_frame_id
builtin_interfaces/Time keyframe_stamp
orbslam3_msgs/FiducialTagObservation[] observations
```

`tag_area_px2` es una métrica geométrica simple y reproducible; si no se usa finalmente, no sustituirla por un `confidence` ambiguo sin definir su semántica.

## Archivos permitidos a modificar

```text
src/orbslam3_msgs/msg/FiducialTagObservation.msg
src/orbslam3_msgs/msg/FiducialKeyFrameObservations.msg
src/orbslam3_msgs/CMakeLists.txt
src/orbslam3_msgs/package.xml
wrapper stereo-slam-node.hpp/.cpp
CMakeLists.txt/package.xml del wrapper
codex/contexto/paquetes/orbslam3_msgs/
codex/contexto/paquetes/<paquete_wrapper_real>/
```

El archivo legacy `FiducialObservation.msg` solo puede borrarse/renombrarse tras búsqueda estática de consumidores y decisión documentada. No es necesario eliminarlo para cerrar esta subfase si queda claramente sin generar/legacy.

## Archivos prohibidos

```text
ORB_SLAM3/
src/orbslam3_server/src/global_map_server.cpp    # suscripción se hace en 4F
src/orbslam3_multi/
src/simulacion_dron/
```

## Funciones, clases o nodos que hay que localizar

```text
rosidl_generate_interfaces
StereoSlamNode publisher creation
StereoSlamNode::GrabStereo
cualquier uso actual de FiducialObservation.msg
```

No inventar un topic si el namespace del wrapper tiene una convención ya documentada. Si no existe, nombre recomendado:

```text
orbslam/fiducial_keyframe_observations
```

## Cambios requeridos

1. Crear mensajes de tag individual y batch por KF.
2. Registrar ambos en `rosidl_generate_interfaces` con dependencias `std_msgs`, `geometry_msgs` y `builtin_interfaces` si se necesita explícitamente.
3. Establecer que `camera_T_tag` está expresado en `header.frame_id` y que ese frame debe ser la cámara izquierda óptica usada por PnP.
4. Usar `keyframe_stamp` idéntico al evento de 4C; `header.stamp` debe copiar el mismo valor para evitar dos tiempos semánticamente distintos.
5. Transportar `map_epoch` y `local_keyframe_id` como identidad funcional; `source_frame_id` queda para auditoría de la asociación exacta.
6. Publicar un único batch cuando `observations` no esté vacío. No es obligatorio publicar batches vacíos en todos los KFs.
7. Copiar todas las detecciones válidas de 4D sin agrupar por `object_id`.
8. Añadir publisher con QoS fiable razonable y coherente con `orb_map_delta`; justificar cualquier QoS distinto.
9. Añadir log `FID-BATCH-PUB` con epoch/KF/tag_count y sin imprimir matrices completas por defecto.
10. Crear test de serialización/compilación que construya un batch con varios tags.

## Cambios prohibidos

- No incluir `sensor_msgs/Image` ni bytes de imagen.
- No incluir `world_T_object`, `world_T_tag`, GT o pose global del dron.
- No reutilizar `fiducial_id` para representar simultáneamente tag y cubo.
- No enviar un mensaje ROS separado por tag como contrato principal.
- No introducir lógica de anchor en el wrapper.
- No cambiar `OrbMap.msg` para transportar fiduciales.

## Paquetes a compilar

```bash
./codex/herramientas/build_selected_packages.sh orbslam3_msgs <paquete_wrapper_real>
```

Si otros paquetes compilan contra `orbslam3_msgs` y el cambio de interfaces exige reconstrucción transitiva, añadirlos solo si es una dependencia real y registrarlo en historial.

## Pruebas Gazebo requeridas

### Prueba 1 — Publicación multi-tag

Usar el escenario de 4D donde un KF observa varias marcas. Verificar con `ros2 topic echo --once` o herramienta equivalente reducida que:

```text
map_epoch correcto
local_keyframe_id correcto
source_frame_id correcto
header/keyframe_stamp iguales al KF
observations.size() >= 2
cada elemento tiene tag_id distinto según detección
```

### Prueba 2 — KF sin tags

Confirmar que no se publica un batch falso y que el resto de topics ORB continúa.

### Prueba 3 — Compatibilidad de interface

Ejecutar un pequeño test o nodo que cree/deserialice un batch con varios elementos; no depender exclusivamente de `ros2 interface show`.

## Patrones de reducción de logs

```text
FID-BATCH-PUB|fiducial_keyframe_observations|tag_count|map_epoch|keyframe_id|rosidl|ERROR|FATAL|Segmentation fault|Killed
```

## Criterio de éxito

1. `orbslam3_msgs` y wrapper compilan.
2. La nueva interface se genera y puede inspeccionarse con ROS 2.
3. Un KF puede transportar N tags en un único batch.
4. El mensaje contiene pose relativa cámara-tag y no contiene GT/global/cubo.
5. La identidad `(drone_id,map_epoch,local_keyframe_id)` es inequívoca.
6. El timestamp coincide con el KF exacto.
7. El wrapper publica el batch sin bloquear el transporte ORB.

## Criterio de fallo o parcial

- `NO CONSEGUIDA`: interface no generada, batch no soporta N tags, mezcla tag/cubo o pierde identidad de KF.
- `PARCIAL`: mensajes compilan pero la publicación no está integrada o falta validación de timestamp/frame.
- `BLOQUEADA`: incompatibilidad real de versión ROS 2/IDL que requiera rediseño no acordado.

## Riesgos

- mantener dos `FiducialObservation` con semánticas incompatibles;
- usar `Header.frame_id` distinto al frame de PnP;
- QoS que pierda observaciones mientras el delta sí llega;
- `confidence` sin definición reproducible.

## Documentación a actualizar

```text
codex/contexto/paquetes/orbslam3_msgs/
codex/contexto/paquetes/<paquete_wrapper_real>/
codex/pipeline/fase_4_fiducial_real/historial/por_subfase/historial_4E.md
codex/pipeline/fase_4_fiducial_real/historial/por_subfase/historial_4E_RESUMEN.md
```
