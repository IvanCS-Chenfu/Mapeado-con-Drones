# `global_map_server.cpp`

## Rol

Orquestador ROS 2 de los flujos principal y secundario. Las decisiones de mapa,
grafo y optimizacion se delegan a `SparseGlobalBackend`.

## Flujo principal

```text
delta/full snapshot -> PrimaryQueue -> WorkerLoop
delta -> backend -> fiduciales -> builder -> PointCloud2 + MarkerArray
snapshot -> backend reconcile -> dirty diferido, sin publish
```

Conserva FIFO por `arrival_id`, backpressure 8/2, snapshots globalmente
serializados, record incremental y replay streaming. Cloud y frustums se
publican juntos solo al final de un delta con cambio publico.

## Flujo fiducial secundario

`ProcessFiducialObservation()` normaliza live/replay, delega al manager y
encola MAX si hay error alto. `SecondaryWorkerLoop()` ejecuta de principio a
fin:

```text
dequeue -> revalidate -> graph -> solve -> validate -> atomic commit/refine
```

- una tarea activa no se interrumpe;
- cada KF distinto se conserva; solo una identidad exacta pendiente/activa se
  deduplica;
- `STALE` termina sin grafo si el target ya esta dentro de umbral;
- un conflicto de revision descarta la propuesta y revalida/reconstruye hasta
  el limite de pasadas;
- un commit full mueve el control al target; un parcial conserva el anterior;
- un fallo duro no compromete y mantiene `blocking_failure`.

`optimization_active` activa el mission gate durante solve/commit. El flujo
principal sigue admitiendo, construyendo y publicando mientras el calculo
privado esta activo.

## Flujo 3M-3P

`EnqueueSecondaryWork()` se ejecuta despues del commit raw/pose principal. Si
el `ChangeSet` modifica covisibilidad encola una MEDIA; si no, encola las
`LoopTask` BAJAS directamente.

`SecondaryWorkerLoop()` aplica una MEDIA mediante
`SparseGlobalBackend::ProcessDatabaseUpdate()` y solo despues encola sus loops.
Para una BAJA ejecuta `ProcessLoopTask()` completo. Un commit de anchor loop
emite anchor/pose dirty y reencola todos los KFs afectados bajo la nueva
revision, pero no llama al builder ni publica.

Si la decision es fusion, la misma BAJA continua por
`FusedLandmarkManager -> CovisibilityDatabase/LandmarkScoreManager ->
GlobalMapBuilder dirty`; no crea otra tarea. `[F3P-FUSION]` resume el lote. La
publicacion siguiente incluye `recalculated_tracks` y `fusion_revision` en
`[F3F-GLOBALMAP-PUBLISH]`.

Si ese intento termina `stale` o revierte un commit parcial, el worker cierra
primero lifecycle y llama a `SecondaryTaskQueue::Complete()`. Despues crea una
`LoopTask` BAJA nueva para el mismo KF con revisiones actuales y la encola por
la via explicita de retry. No hay limite fijo; pendientes equivalentes se
coalescen y un KF ausente/inactivo/bad no genera tarea. El nuevo intento repite
BoW, regiones, RANSAC, fusion y score desde estado fresco.

El primer fiducial directo de un submapa con dependencia loop emite
`[F3O-FID-LOOP-REANCHOR]`: el backend reancla todo el submapa como hard y el
servidor solo comunica los KFs dirty. `[F3K-ATOMIC-COMMIT]` distingue
`moved_kfs` de `control_propagated` para hacer observable el movimiento rigido
de hijos blandos.

La telemetria usa un unico lifecycle `start/done` por tarea. Las etapas BoW,
geometria, decision y commit se acumulan dentro de ese owner visual.

Cada etapa secundaria emite progreso al visualizador. Ademas, el servidor emite
`secondary_task_lifecycle` con `start` al dequeue y `done` al finalizar,
identificados por `task_id`/`flow_id`; permite mantener una sola tarea visual
activa durante todo su ciclo sin cambiar la ejecucion ROS.

Todo el despacho de una tarea esta protegido por `try/catch`. Una excepcion
restaura `optimization_active`, se convierte en fallo duro observable y pasa
por el cierre comun (`Complete`, contadores, backpressure y lifecycle `done`),
evitando un aborto sin diagnostico del nodo.

## Visitas y replay

Live detecta transiciones dentro/fuera del radio por dron y asigna un ID global
estable. El record v3 lo guarda con la observacion. Para records v1/v2, replay
ordena metadatos por submapa/timestamp/KF antes de inferir visitas; despues
ordena las observaciones de cada arrival por timestamp/KF para evitar controles
temporales invertidos.

## Referencias

```text
orbslam3_server/src/global_map_server.cpp
  -> GlobalMapServer::{WorkerLoop,SecondaryWorkerLoop,ProcessFiducialObservation}
  -> rg -n "SecondaryWorkerLoop|EnqueueSecondaryWork|EnqueueLoopTasks|ProcessFiducialObservation"
```

## Telemetria 3H-3L

```text
[F3H-FID-POSE-ERROR] [F3H-FID-TASK-ENQUEUE]
[F3H-SECONDARY-START] [F3H-FID-REVALIDATE] [F3H-SECONDARY-DONE]
[F3I-GRAPH-BUILD] [F3J-OPTIMIZE]
[F3L-VALIDATE] [F3L-HARD-FAILURE]
[F3K-COMMIT-STALE] [F3K-ATOMIC-COMMIT]
[F3K-CONTINUATION-UPDATE] [F3K-FUTURE-KF-PROPAGATE]
[F3C-BACKPRESSURE] [F3H-SECONDARY-SHUTDOWN]
[F3M-DATABASE-ENQUEUE] [F3M-DATABASE-UPDATE]
[F3N-LOOP-ENQUEUE] [F3O-RANSAC] [F3O-LOOP-DONE]
[F3O-FID-LOOP-REANCHOR]
[F3P-FUSION] [F3P-FUSION-RETRY] [F3P-FUSION-RETRY-SKIP]
[F3H-SECONDARY-EXCEPTION]
[F3F-GLOBALMAP-PUBLISH ... recalculated_tracks=... fusion_revision=...]
```

No publica desde callbacks, snapshots ni el worker secundario.

## Color de keyframes

`SubmapColor(drone_id,map_epoch)` vive en
`include/orbslam3_server/submap_color.hpp`. Combina una base por dron con un
salto aureo de 137.507764 grados por epoch; asi conserva estabilidad por
`(drone_id,map_epoch)` y evita que epochs consecutivos del mismo dron queden
visualmente casi iguales. `test_submap_color` exige distancia RGB minima tanto
entre epochs como entre drones.
