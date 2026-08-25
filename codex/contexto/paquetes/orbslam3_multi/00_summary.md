# 00_summary - orbslam3_multi

## Estado activo

Backend reconstruido hasta 3R. Separa estado raw, poses world, score, tracks
fusionados y vista publica; integra optimizacion fiducial, deteccion geometrica
de loops y la rama de fusion de error bajo:

```text
SparseGlobalBackend
|- RawMapDatabase
|- FiducialAnchorManager
|- GlobalPoseStore
|- LandmarkScoreManager
|- CovisibilityDatabase
|- LoopPipeline: BoW -> regiones -> subnubes/RANSAC -> decision
|- FusedLandmarkManager: tracks transitivos -> patch/rollback
|- PoseGraphBuilder -> OptimizationManager -> OptimizationValidator
`- GlobalMapBuilder
```

El flujo principal de 3G permanece incremental. Un full snapshot reconcilia
autoridades y deja dirty para el siguiente delta; no publica. El flujo
secundario 3H-3L captura snapshots acotados, calcula fuera de locks y termina
con un commit breve en `GlobalPoseStore`. `RawMapDatabase` nunca se modifica por
la optimizacion.

Desde 4F, `RawMapDatabase` posee un sidecar de batches fiduciales pendientes:
lookup exacto O(1), FIFO configurable por dron con capacidad inicial 10, sin
TTL y bajo el mutex raw existente. Resuelve tanto batch→KF como KF→batch,
conserva el primer `arrival_id`, elimina al consumir y distingue
duplicate/conflict mediante digest. El backend solo expone configuracion,
submission, matches y estadisticas; todavia no interpreta objetos.

## Covisibilidad y loops

- `CovisibilityDatabase` prepara patches ORB fuera de lock y los compromete de
  forma breve, canonica, versionada e idempotente.
- `LoopPipeline` mantiene un indice BoW derivado, agrupa hasta tres regiones y
  construye subnubes acotadas de hasta 320 puntos para matching ORB y RANSAC.
- Una region robusta de error alto con apoyo independiente domina sobre
  regiones fusionables de la misma tarea. Entre una y tres regiones coherentes
  alimentan `CurrentLoop`; una fusion mixta ya no oculta la optimizacion.
- El commit 3P coordina tracks, covisibilidad geometrica server y score. Si
  cambia una dependencia, descarta el patch o revierte lo ya aplicado antes de
  devolver `STALE`; nunca modifica raw ni poses.
- La visibilidad sparse simetrica procesa contradicciones como diagnostico,
  sin deltas negativos hasta Fase 8. Reutiliza un depth buffer por
  direccion/region; sus tiempos son telemetria, no criterio de rechazo.
- Dos queries independientes y coherentes pueden anclar un submapa no anclado.
  `CommitLoopAnchorBatch()` incluye todos sus KFs vigentes y el builder recibe
  solo IDs dirty.
- El anchor loop es blando: sigue rigidamente al submapa padre. El primer
  fiducial posterior reancla atomica y absolutamente todo el hijo, lo vuelve
  hard y elimina esa dependencia. El KF de loop no se registra como control
  fiducial aceptado.
- Cada `LoopTask` lleva una huella de apariencia/geometria para scheduling y
  una `validation_revision` exacta independiente. La geometria de scheduling
  usa pose gruesa, madurez `insuficiente/suficiente/densa` segun
  `min_query_mappoints` y presencia de covisibilidad fuerte; los MapPoints
  exactos siguen protegiendo stale y commit sin provocar reevaluaciones por
  cada refinamiento ORB.
- `LoopTaskIntent::Full` conserva el flujo completo. Los reruns de KFs movidos
  usan `FusionRefresh`: repiten BoW/RANSAC/fusion/score, pero una evidencia de
  error alto se difiere para evitar optimizaciones recursivas. Los refresh se
  agrupan por region temporal y filtran candidatos por proximidad AABB de sus
  subnubes world; no hacen BoW global lejano.

## Optimizacion comun

- `FiducialAnchorManager` crea el primer anchor o compara una revisita contra
  `target_world_T_kf` con umbrales 0.35 m, 0.35 rad y 0.25 rad de yaw.
- Cada KF distinto con error alto produce una `FiducialOptimizationTask`; el
  dequeue vuelve a consultar pose y ultimo control.
- `PoseGraphBuilder` construye ventanas loop y fiduciales multi-submapa con
  controles base 30 %, hard fijos, temporal, covisibilidad sparse,
  loops/fusiones server transitivas y dependencias blandas.
- `OptimizationManager` resuelve ambos tipos sobre una propuesta privada; el
  loop mueve conjuntamente ambos lados segun su relacion RANSAC.
- `OptimizationValidator` exige cobertura de cada `CurrentLoop` y rechaza sin
  escribir degradacion temporal, covisible, de fusiones previas, hard o de
  corredores hard-hard.
- El commit atomico rebasa sobre poses actuales, omite controles intermedios
  caducados y puede conservar un extremo culled con raw estable como apoyo
  virtual sin reactivarlo ni escribirlo. Exige dos controles activos por
  submapa e incluye KFs tardios/tail compatibles. Solo los KFs activos movidos
  se notifican dirty.
- Antes del builder, las regiones respaldadas en ambos extremos por
  fiduciales/corredores o vecinos temporal-covisibles protegidos se comparan
  contra RANSAC. Una discrepancia mayor de 5 m/20 grados se rechaza y se guarda
  en un ledger regional revisionado; con solo un extremo fiable se mantiene la
  rama asimetrica. Los cambios de anchor/loop/fusion invalidan las entradas
  afectadas.

`GlobalPoseStore` conserva un `ContinuationRecord` atomico por submapa. Un
commit full actualiza poses y continuidad; todo KF posterior al control mantiene
su `raw_world_pose` bajo el anchor inicial y deriva `world_pose` desde el ultimo
control aceptado. Un parcial no cambia esa frontera.

## Scoring 3R

- raw = base ORB por factores recuperables de distancia/aislamiento mas
  `+0.04` por inlier;
- indice voxel incremental, madurez minima y geometria neutra sin anchor;
- fused = media de todos los raw miembros mas `0.04*N`;
- cambios ORB/pose/vecindad se propagan solo a tracks afectados y builder dirty;
- builder publica todos los puntos con score, sin filtro ni GT.
- distancia 3R usa near fijo 1 m y far `83.333333*baseline`; con baseline
  `0.06 m`, la banda neutra es 1-5 m y ambas caidas son cuadraticas acotadas.

El primer KF observado de una visita reserva el control: si es coherente se
acepta directamente y si requiere optimizacion pasa a control tras
`ACCEPT_FULL`. Los demas KFs de la visita se evaluan individualmente, pero no
pueden adelantarse y volverse hard durante esa tarea. El
`fiducial_visit_id` se conserva en el record v3; los loaders v1/v2 siguen
aceptados.

## Archivos principales

```text
include/orbslam3_multi/{fiducial_types,fiducial_anchor_manager}.hpp
include/orbslam3_multi/{fiducial_optimization_task,pose_graph_problem}.hpp
include/orbslam3_multi/{pose_graph_builder,optimization_manager}.hpp
include/orbslam3_multi/{optimization_validator,pose_geometry}.hpp
include/orbslam3_multi/{global_pose_types,global_pose_store}.hpp
include/orbslam3_multi/{raw_map_types,raw_map_database}.hpp
include/orbslam3_multi/{covisibility_database,loop_task,loop_pipeline}.hpp
include/orbslam3_multi/{fused_landmark_types,fused_landmark_manager}.hpp
include/orbslam3_multi/{global_map_builder,sparse_global_backend}.hpp
src/{covisibility_database,loop_pipeline,fused_landmark_manager}.cpp
src/{pose_graph_builder,optimization_manager,optimization_validator}.cpp
src/{fiducial_anchor_manager,global_pose_store,sparse_global_backend}.cpp
```

Detalle: `fiducial_anchor_manager.md`, `pose_graph_builder.md`,
`optimization_manager.md`, `optimization_validator.md`,
`global_pose_store.md`, `covisibility_database.md`, `loop_pipeline.md`,
`fused_landmark_manager.md`, `raw_map_database.md`, `global_map_builder.md` y
`sparse_global_backend.md`.

## Limites

No contiene callbacks ROS, GT, colas ni publicacion. No filtra la nube por
score; ese consumo queda para fases posteriores. Las copias antiguas fueron
retiradas en 3T tras validar esta implementación.

## Validacion vigente

- build final de `orbslam3_multi` y `orbslam3_server`: 2/2, exit 0;
- CTests funcionales finales: `orbslam3_multi` 9/9, servidor 4/4 y contrato
  web 1/1;
- replay 149: 7 tareas, 3 commits, 4 `STALE`, cero fallos;
- replay de regresion 150: reproduce la carrera de live 148 y compromete KF149
  como control sin promocionar KF150 ni producir hard failure;
- live 151: 11 tareas, 3 commits, 8 `STALE`, cero hard, cola final vacia; los
  KFs posteriores al control187 siguen dentro del umbral;
- replay 153: 806 tareas secundarias procesadas, `pending=0`, cero hard y
  latencia final de loop de unos 0.16-0.18 s;
- live 154: `(2,0)` se ancla por loop tras dos queries independientes, sin
  fiducial propio; 8 KFs dirty y backfill posterior correcto.
- prueba 157: el apoyo loop A/KF72 se mueve en la optimizacion fiducial y el
  mismo commit propaga rigidamente 78 KFs del hijo;
- prueba tipica 156: reanchor hard post-loop de 32 KFs, tres commits
  fiduciales completos y 1060 tareas secundarias para 486 poses, frente a
  2301/248 en live 154; cola final vacia y cero fallos duros.
- prueba 159: intento 3P fallido conservado; una union transitiva dejo un track
  retirado en `touched_tracks` y produjo `std::out_of_range`.
- prueba 160: 62 intentos de fusion, 56 commits, cinco stale por dependencias y
  un rollback por score; servidor limpio, cola vacia y cero fallos duros.
- prueba 161: 27 intentos, ocho commits y 19 stale, incluidos cuatro rollback;
  `56/56` regiones completas. Los retries frescos pertenecen al servidor, pero
  todos vuelven a ejecutar este pipeline y la cola termina vacia. Cuatro
  optimizaciones fiduciales full confirman que la ruta anterior no regresa.
- prueba 175: ocho optimizaciones loop iniciadas/finalizadas, siete commits
  iniciales y un stale reintentado hasta commit; ventanas de hasta tres
  submapas, 174 KFs y 89 controles; cola final cero y sin hard failures.
- optimizacion de visibilidad 175: 129/155 fusiones comprometidas, 10 stale,
  prepare maximo 142.766 ms y proyecciones maximas 1848. En 174, antes del
  cambio, fueron 63/696, 316 stale, 2001.86 ms y 70568 proyecciones.
- prueba 187: tres optimizaciones/tres commits, 16 refresh altos diferidos,
  1047 tareas, `pending=0` y cero hard failures;
- prueba 188: recorrido doble completo, nueve commits loop, ocho fiduciales,
  dos anchors loop, 17 rechazos estructurales sin escritura y 995 fusiones;
  el error medio loop baja de 0.469849 a 0.089286 m;
- tests finales: grafos/validacion 14/14, pipeline 9/9, cola 6/6, CTest
  `orbslam3_multi` 9/9, servidor funcional 4/4 y web 1/1.
- cierre 3T: CTest `orbslam3_multi` 9/9 y prueba 195 con 11 commits loop,
  453 KFs activos publicados, colas drenadas y cero hard failures tras retirar
  las copias legacy.
