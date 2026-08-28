# FiducialObjectInterpreter

## Responsabilidad

Interpreta un `SynchronizedFiducialBatch` ya asociado al KF raw exacto. Carga
`fiducial_objects.yaml` con `yaml-cpp`, resuelve `tag_id -> object_id`, deriva
`object_T_tag` por cara y produce candidatos `world_T_camera`.

La base de cada tag coincide con OpenCV/IPPE: X horizontal, Y hacia arriba en
la textura y Z normal saliente. `FaceTransform()` expresa esa base en el objeto
y `Interpret()` compone
`camera_T_object=camera_T_tag*inverse(object_T_tag)` y despues
`world_T_camera=world_T_object*inverse(camera_T_object)`. Por tanto su salida es
la pose de la camara optica; no convierte a body ni usa `body_T_camera`.

Por objeto aplica:

- rango inclusivo configurable por cada `camera_T_tag`; un tag fuera de rango
  vuelve no apto al objeto completo sin borrar sus observaciones;
- peso base `max(quality_score, epsilon) * sqrt(area/max_area)`;
- tres iteraciones de reponderacion robusta sobre residual SE(3);
- primary unico por calidad y desempate por `object_id`;
- FIFO de los ultimos KFs interpretados por dron, default 50.

Las visitas se mantienen por `(drone_id,map_epoch,object_id)` como intervalos
temporales. Una llegada fuera de orden conserva el ID si queda a menos de
`fiducial_visual_visit_gap_sec` del intervalo mas cercano.

## Referencias

```text
include/orbslam3_server/fiducial_object_interpreter.hpp
  -> FiducialObjectInterpreter, VisitState
src/fiducial_object_interpreter.cpp
  -> Load, Interpret, AssignVisitLocked
  -> rg -n "FaceTransform|camera_T_object|Interpret\(|AssignVisitLocked"
test/test_fiducial_object_interpreter.cpp
  -> configuracion, rango, fusion robusta, orden temporal y FIFO
```

No consulta GT, no modifica `RawMapDatabase` y no llama directamente al
optimizer. El handoff al manager pertenece a `GlobalMapServer`.
