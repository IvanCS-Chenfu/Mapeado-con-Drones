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
- un target que ya no es posterior al control vigente tambien termina `STALE`;
- un conflicto de revision descarta la propuesta y revalida/reconstruye hasta
  el limite de pasadas;
- un commit full mueve el control al target; un parcial conserva el anterior;
- un fallo duro no compromete y mantiene `blocking_failure`.

`optimization_active` activa el mission gate durante solve/commit. El flujo
principal sigue admitiendo, construyendo y publicando mientras el calculo
privado esta activo.

## Flujo 3M-3Q

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

Cuando `ProcessLoopTask()` devuelve `OptimizationEvidence`, la misma BAJA
continua sin reenviarse a la cola:

```text
stop_drones=true -> graph -> solve -> validate -> atomic multi-submap commit
                  -> fusion 3P directa -> stop_drones=false -> task done
```

No cambia la prioridad ni existe preemption. Una MAX que llegue durante el
solve espera al final de la BAJA activa. El flag se restaura ante accept,
stale o excepcion. `[F3Q-LOOP-OPT]` publica ventana, controles, aristas,
iteraciones, errores/coste, KFs movidos/propagados y tiempos separados de
graph, solve, validation y commit; cada `[F3Q-OPT-START]` debe tener
`[F3Q-OPT-END]`. Tanto el lifecycle de dequeue como el inicio 3Q exponen el
`intent` efectivo; esto distingue un `FusionRefresh` solicitado de una tarea
coalescida donde prevalecio `Full`.

Los KFs movidos por commits loop o fiducial se reencolan con intent
`FusionRefresh`: pueden detectar y comprometer fusiones/scores, pero no iniciar
otra optimizacion. Una tarea `Full` pendiente prevalece al coalescer y los
retries conservan el intent de la tarea que los origino. El servidor usa
`CreateFusionRefreshTasks()` para agrupar KFs movidos por region temporal y
publica `moved/grouped/created/enqueued` en `[F3Q-POST-OPT-LOOPS]`.

El backpressure secundario se calcula con pendientes `critical`: fiduciales,
MEDIA y loops `Full`. Los `FusionRefresh` pendientes se cuentan como
`maintenance` y no mantienen por si solos el mission gate, aunque siguen
ejecutandose en el unico worker. Una optimizacion real, incluida la rama 3Q de
una BAJA, mantiene `optimization_active=true` hasta su final.

Antes del builder, el backend puede finalizar una tarea como
`protected_region_far_repeated_loop` o `regional_rejection_ledger_hit`.
`[F3O-LOOP-DONE]` expone los extremos protegidos, error, ledger y conteos del
filtro espacial de refresh para demostrar que no se consumio el solver.

Cada etapa secundaria emite progreso al visualizador. Ademas, el servidor emite
`secondary_task_lifecycle` con `start` al dequeue y `done` al finalizar,
identificados por `task_id`/`flow_id`; permite mantener una sola tarea visual
activa durante todo su ciclo sin cambiar la ejecucion ROS.

Todo el despacho de una tarea esta protegido por `try/catch`. Una excepcion
restaura `optimization_active`, se convierte en fallo duro observable y pasa
por el cierre comun (`Complete`, contadores, backpressure y lifecycle `done`),
evitando un aborto sin diagnostico del nodo.

## Scoring 3S

El constructor declara parametros `score_*`,
`fusion_score_inlier_reward=0.04` y `fusion_score_member_bonus=0.04`. Los
parametros sparse negativos de 3P se retiraron porque la oclusion es solo
diagnostica hasta Fase 8.

Los defaults de distancia son `score_suspicious_near_distance_m=1.0`,
`score_suspicious_near_min_factor=0.05`,
`score_far_baseline_multiplier=83.333333`,
`score_far_distance_fallback_m=5.0` y `score_far_min_factor=0.25`. Con baseline
`0.06 m`, el factor queda neutro entre 1 y 5 m.

El principal emite `[F3S-RAW-SCORE-COMMIT]` con dirty raw/fused y
`[F3S-SCORE-STATS]` cada 25 arrivals live. El secundario emite
`[F3S-FUSED-SCORE-COMMIT]`; rejected/stale muestran `committed=false dirty=0`.
`PointCloud2` mantiene `score` y `rgb`, con rojo en 0, amarillo en 0.5 y verde
en 1, sin filtrar puntos.

La prueba 194 cierra con principal/secundario `pending=0`, 23.564 puntos,
`score_field=true`, `rgb_field=true`, 53 commits fused y cero penalizaciones
sparse. Stats finales: 24.977 anchored, 99 near, 11.433 far y media `0.2596`.
El exit 255 de Gazebo ocurre durante cleanup posterior a `success=true`, no
durante el escenario.

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
[F3Q-OPT-START] [F3Q-LOOP-OPT] [F3Q-OPT-END]
[F3H-SECONDARY-EXCEPTION]
[F3S-RAW-SCORE-COMMIT] [F3S-SCORE-STATS] [F3S-FUSED-SCORE-COMMIT]
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
