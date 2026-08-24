# Pipeline Fase 4 — Fiducial Real — resumen

## Obligación de mantener system_architecture durante Fase 4

Fase 4 no solo sustituye el fiducial GT: las subfases que cambien interfaces deben
actualizar el visualizador arquitectónico.

Puntos explícitos:
- 4E: nuevo contrato ROS de observaciones por KF; metadata declarativa.
- 4F: nace el consumo wrapper→Servidor; nueva arista runtime y telemetría directa.
- 4H: el anchor visual sustituye funcionalmente al feed GT; retirar/marcar la arista
  GT fiducial como no funcional.
- 4K: integración multi-dron y verificación final de topología/live.

La instrumentación del grafo sigue siendo debug opcional, ligera y totalmente dormida
cuando se desactiva.

Usar este archivo como primera lectura antes de `pipeline_fase_4.md` y de los contratos de subfase.

## Estado vigente

```text
Fase 4: actual, sin ejecutar
Subfase siguiente: 4A, preparación no iniciada
Subfases previstas: 4A ... 4L
Historial: carpetas creadas y vacías; no existen ejecuciones registradas
```

## Objetivo

Reemplazar el fiducial simulado basado en GT por observaciones visuales ligadas al KeyFrame exacto que vio cada marca. ORB-SLAM3 solo notificará la creación del KF; el wrapper detectará tags en la imagen izquierda exacta de ese KF; el servidor interpretará los tags, conocerá los cubos y realizará anchor/revisitas sin GT funcional.

## Decisiones fijas

```text
objeto físico        = cubo multicara
ID por cara          = tag_id único
familia baseline     = AprilTag 36h11
motor de detección   = OpenCV aruco/AprilTag
pose planar          = solvePnP / IPPE_SQUARE
imagen funcional     = izquierda exacta del KF
ORB-SLAM3             = no detecta fiduciales
wrapper               = no agrupa por cubo
servidor              = agrupa/interpreta tags y aplica geometría global
identidad de KF       = (drone_id, map_epoch, local_keyframe_id)
GT                    = solo métricas/debug externo
```

## Secuencia

```text
4A configuración física/lógica
 -> 4B spawn Gazebo
 +  4C evento exacto de KeyFrame
 -> 4D detector en wrapper
 -> 4E mensajes ROS 2
 -> 4F sincronización exacta servidor
 -> 4G tag -> cubo
 -> 4H primer anchor visual
 -> 4I revisitas/optimización
 -> 4J robustez/degradación
 -> 4K multi-dron
 -> 4L cámara real/cierre
```

## Contrato de datos central

Para cada KF con detecciones válidas, el wrapper publicará conceptualmente:

```text
drone_id
map_epoch
local_keyframe_id
keyframe_stamp
source_frame_id
camera frame
observations[]:
  - tag_id
  - camera_T_tag
  - reprojection_error_px
  - métricas geométricas mínimas acordadas
```

Puede haber muchos tags en el mismo KF. El wrapper no decide si pertenecen al mismo cubo.

## Fórmula de anchor

Con `world_T_object` y `object_T_tag` conocidos por el servidor:

```text
world_T_tag    = world_T_object * object_T_tag
world_T_camera = world_T_tag * inverse(camera_T_tag)
world_T_local  = world_T_camera * inverse(local_T_camera)
```

Antes de usar estas relaciones se deben validar las convenciones reales de frame y `Tcw/Twc`.

## Entrada al detalle

```text
pipeline_fase_4.md
subfases/subfase_4A.md
...
subfases/subfase_4L.md
```

No usar las carpetas de historial como fuente de resultados hasta que existan ejecuciones reales.
