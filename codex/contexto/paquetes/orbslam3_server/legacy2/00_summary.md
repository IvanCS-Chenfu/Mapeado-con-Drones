# 00_summary - orbslam3_server

Adaptador ROS 2 y coordinador del mapa global. El ejecutable activo es
`global_map_server` y el nodo `global_orb_map_server`.

## Flujo principal

Entradas:

```text
/dron_X/orbslam/orb_map_delta
/dron_X/orbslam/get_full_map
observaciones fiduciales simuladas
```

Cada delta/snapshot se compromete en `RawMapDatabase`; el servidor distribuye
los cambios a poses, covisibilidad y scores y solicita publicacion. El primer
fiducial ancla. Los KFs posteriores se registran desde el ultimo control world
aceptado.

Un worker de publicacion captura estado coherente, construye nube/marcadores
fuera de `live_state_mutex_` y publica:

```text
/global_sparse_cloud
/global_keyframes
```

No espera al worker secundario, RViz2 o web.

La captura si toma `live_state_mutex_`. El diagnostico 75/76 muestra que la
espera mas copia llega a `18.875/27.225 s`; ademas los callbacks de delta/full
snapshot mantienen ese mismo mutex durante trabajo posterior al commit. La
separacion temporal permanece `PARCIAL`.

## Worker secundario

Existe un unico thread persistente con dos colas FIFO:

```text
FIDUCIAL  prioridad maxima
LOOP      prioridad normal
```

La tarea activa no se interrumpe. `LoopTask` contiene BoW, filtros,
matching/RANSAC, decision, fusion y/o optimizacion. Las capturas y calculos son
privados; el lock live se usa solo para validar revisiones y hacer commits
breves. Una tarea termina sin esperar publicacion.

La cola de loops admite solo KFs materialmente nuevos con apariencia. La
capacidad dura por defecto es `4096`. La carga se mide, pero
`/global_mapping/backpressure_active` se publica siempre `false` para que
ninguna tarea secundaria pause el escenario o la ingesta.

## Optimizacion por loop

- requiere soportes geometricos previos (`loop_optimization_min_prior_supports`,
  default `2`);
- usa `PoseGraphBuilder::BuildForLoopTask` y `LOOP_RELATIVE`;
- no convierte la medida en un target world;
- fija el lado candidato y conserva hard fiducials;
- suprime optimizacion para candidatos `near_same_submap`;
- valida/apply sobre una copia privada y solo intercambia al aceptar;
- fusiona inliers dentro del mismo `task_id`.

Las trazas `[F1Q-LOOP-OPT-COMPUTE]` incluyen query/candidate, errores, inliers,
soportes y dimensiones del grafo. El commit aceptado incluye KFs
optimizados/propagados, KFs fijos y deltas maximos.

## Telemetria 3U

`TryTraceFlow` introduce eventos JSON ligeros en una cola acotada no
bloqueante y los publica en:

```text
/global_mapping/flow_events   std_msgs/msg/String
```

Hay IDs distintos para delta/snapshot, commits raw, poses, anchors, cola,
etapas `3N/3O/decision/grafo/solver`, bases derivadas, builder y nube/KFs de
RViz2. Si la cola esta ocupada, se pierde telemetria antes que bloquear el
pipeline.

Los parametros y marcadores `f1*`/`F1*` se conservan como nombres legacy del
runtime. Desde la limpieza documental de Fase 3, los artefactos escritos en
`codex/archivos_auxiliares` usan nombres `f3l_*` y `f3i_*`.

## Archivos y tests

```text
src/global_map_server.cpp
src/test_secondary_task_order.cpp
launch/global_orb_map_server.launch.py
```

- `test_secondary_task_order`: prioridad estable y tarea activa no
  interrumpible.
- `prueba_75`: escenario completo, prioridad fiducial observada y un solo
  worker; revela un loop temporal cercano que se corrige despues.
- `prueba_76`: `489` loops encolados, `84` ejecutados, pico `429`, ultimo valor
  `414`, `144` publicaciones y cero commits de poses por loop.

## Parametros destacados

```text
rawdb_record_enabled / rawdb_replay_enabled
loop_bow_*
loop_verify_*
loop_task_max_pending
loop_optimization_min_prior_supports
mapping_backpressure_topic
flow_telemetry_enabled
flow_telemetry_topic
global_sparse_cloud_topic
global_keyframes_topic
pose_graph_*
f1j_*
f1k_apply_enabled
f1l_validation_enabled
```

Detalles actuales en `global_map_server.md` y `launches.md`; la evidencia
cronologica vive en los historiales de subfase.
