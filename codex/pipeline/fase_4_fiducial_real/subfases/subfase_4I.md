# Subfase 4I — Revisitas visuales y optimización fiducial

## Estado

```text
sin hacer
```

## Dependencia

`4H`.

## Objetivo técnico

Conectar las observaciones visuales posteriores al primer anchor con la lógica de revisita ya existente. Un residual pequeño debe validarse sin crear una optimización innecesaria; un residual alto y coherente debe crear una tarea de optimización fiducial prioritaria, manteniendo los mecanismos de validación, commit y rollback de la Fase 3.

Los fiduciales siguen siendo restricciones absolutas y no loops.

## Comportamiento esperado

```text
revisit visual
  -> calcular target world_T_camera desde tag/cubo
  -> comparar con pose global estimada del mismo KF
  -> residual bajo  -> revisit OK, sin task
  -> residual alto  -> FiducialOptimizationTask
```

Varias caras del mismo cubo en un único KF constituyen una sola visita lógica a ese cubo. Varias observaciones de cubos diferentes pueden producir restricciones independientes, pero nunca deben duplicar una misma task por reentrega del batch.

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
subfases/subfase_4H.md
src/orbslam3_multi/include/orbslam3_multi/fiducial_anchor_manager.hpp
src/orbslam3_multi/include/orbslam3_multi/fiducial_optimization_task.hpp
src/orbslam3_multi/src/fiducial_anchor_manager.cpp
src/orbslam3_multi/include/orbslam3_multi/optimization_manager.hpp
src/orbslam3_server/src/global_map_server.cpp
```

Leer los resúmenes/historial reales de las subfases de Fase 3 que implementaron revisitas y worker secundario antes de tocar su lógica.

## Diagnóstico de partida

El backend actual ya dispone de:

- umbrales de traslación/rotación/yaw;
- detección de revisit;
- `FiducialOptimizationTask`;
- deduplicación por submapa/fiducial;
- hard fiducial keyframes;
- cola/worker secundario y validación posterior.

El problema de Fase 4 no es reescribir esa arquitectura, sino sustituir su observación GT por la observación visual normalizada de 4H y definir correctamente la semántica multi-tag del mismo KF.

## Archivos permitidos a modificar

```text
src/orbslam3_multi/include/orbslam3_multi/fiducial_anchor_manager.hpp
src/orbslam3_multi/src/fiducial_anchor_manager.cpp
src/orbslam3_multi/include/orbslam3_multi/fiducial_optimization_task.hpp
src/orbslam3_multi/src/optimization_manager.cpp            # solo si la nueva observación lo exige
src/orbslam3_server/src/global_map_server.cpp
src/orbslam3_server/launch/global_orb_map_server.launch.py
codex/contexto/paquetes/orbslam3_multi/
codex/contexto/paquetes/orbslam3_server/
```

## Archivos prohibidos

```text
ORB_SLAM3/
ORB_SLAM3_ROS2/
src/simulacion_dron/
legacy optimizers no usados
```

## Funciones, clases o nodos que hay que localizar

```text
FiducialAnchorManager::RegisterFiducialObservation
FiducialOptimizationTask
GetPendingFiducialOptimizationTasks
CompleteFiducialOptimizationTask
AcceptOptimizedFiducialTask
secondary task ordering/worker en orbslam3_server
pose graph builder fiducial constraints
```

## Cambios requeridos

1. Alimentar la lógica de residual con `world_T_camera_target` visual de 4H.
2. Mantener los umbrales existentes como baseline; no retocarlos hasta medir la nueva fuente visual.
3. Definir `visit_id` funcionalmente mediante el KF exacto: dos caras del mismo cubo en el mismo KF no son dos revisitas.
4. Deduplicar por `(submap, object_id, local_keyframe_id)` o semántica equivalente antes de crear tasks repetidas por múltiples tags del mismo cubo.
5. Si el mismo KF ve varios cubos, calcular residual por objeto. No fusionar a ciegas restricciones incompatibles.
6. Mantener la política: residual bajo no crea task; residual alto crea/actualiza una `FiducialOptimizationTask` según las reglas ya existentes.
7. Conservar prioridad de tareas fiduciales sobre loops sin interrumpir una tarea ya activa.
8. Mantener hard fiducials y restricciones del pose graph, adaptando solo los campos que antes procedían de GT.
9. Mantener commit seguro, validación post-apply y rollback existentes.
10. Registrar `object_id`, `tag_id` representativo, KF, residual y origen `visual_fiducial` en logs de revisit/task.
11. No usar GT para clasificar residual; GT puede calcularse después como métrica externa.
12. Añadir test donde un residual alto simulado se obtenga modificando la observación/configuración controladamente, no alterando GT funcional.

## Cambios prohibidos

- No convertir fiducial en loop candidate.
- No aumentar thresholds para hacer pasar las pruebas.
- No desactivar post-apply/rollback.
- No generar una task por cada cara del mismo cubo en el mismo KF.
- No interrumpir una tarea activa para ejecutar otra fiducial.
- No reescribir el worker secundario si la interfaz existente basta.
- No leer GT dentro del manager.

## Paquetes a compilar

```bash
./codex/herramientas/build_selected_packages.sh orbslam3_multi orbslam3_server
```

Ejecutar también los tests unitarios existentes del backend relacionados con pose store, task ordering y optimizer cuando estén disponibles.

## Pruebas Gazebo requeridas

### Prueba 1 — Revisit con residual pequeño

Primero anclar visualmente un submapa. Volver a observar el mismo cubo desde otro KF con geometría coherente. Debe aparecer revisit OK y no crear una task.

### Prueba 2 — Revisit con error real controlado

Introducir una discrepancia reproducible en la pose configurada/observación de prueba suficientemente grande para superar el umbral, sin usar GT como fuente. Debe crearse una task fiducial prioritaria.

### Prueba 3 — Dos caras del mismo cubo en revisit

El mismo KF ve dos tags del mismo objeto. Debe existir una sola visita lógica y no duplicarse la task.

### Prueba 4 — Commit/rollback

Ejecutar la ruta de optimización y comprobar que se conservan validaciones y rollback de Fase 3. Una solución inválida no puede quedar aplicada.

## Patrones de reducción de logs

```text
FID-REVISIT|FID-TASK|FID-POSE-ERROR|visual_fiducial|secondary_queue|OPT|POST-APPLY|ROLLBACK|object_id|keyframe_id|ERROR|FATAL|Segmentation fault|Killed
```

Incluir también los marcadores legacy vigentes de Fase 3 si siguen siendo los únicos emitidos; renombrarlos solo con justificación y actualizando documentación.

## Criterio de éxito

1. Build y tests backend pasan.
2. Residual pequeño no crea optimización.
3. Residual alto crea task fiducial.
4. Dos caras del mismo cubo/KF no duplican revisit/task.
5. Prioridad, worker, hard-fiducial, validación y rollback se conservan.
6. Ninguna decisión funcional usa GT.
7. Los logs identifican el objeto y KF visual de origen.

## Criterio de fallo o parcial

- `NO CONSEGUIDA`: revisitas visuales no llegan al optimizer, tasks duplicadas por cara o GT sigue decidiendo residual.
- `PARCIAL`: clasificación residual funciona pero falla commit/rollback o deduplicación multi-tag.
- `BLOQUEADA`: una invariancia de Fase 3 impide aceptar `world_T_camera_target` sin rediseño mayor no aprobado.

## Riesgos

- cambiar thresholds antes de caracterizar ruido visual;
- confundir varias caras con varias visitas;
- crear varias tasks simultáneas redundantes por un mismo KF;
- degradar la política del worker secundario.

## Documentación a actualizar

```text
codex/contexto/paquetes/orbslam3_multi/
codex/contexto/paquetes/orbslam3_server/
codex/contexto/01_ESTADO_ACTUAL.md si cambia el estado real
codex/pipeline/fase_4_fiducial_real/historial/por_subfase/historial_4I.md
codex/pipeline/fase_4_fiducial_real/historial/por_subfase/historial_4I_RESUMEN.md
```
