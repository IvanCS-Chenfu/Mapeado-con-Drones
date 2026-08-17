# `global_map_server.cpp`

## Rol

`orbslam3_server/src/global_map_server.cpp` define internamente
`GlobalMapServer : rclcpp::Node`. Coordina ROS y las clases de
`orbslam3_multi`; no posee algoritmos duplicados de raw, poses, BoW, fusion o
optimizacion.

## Entradas y salidas

```text
consume /dron_X/orbslam/orb_map_delta
solicita /dron_X/orbslam/get_full_map
consume GT solo para asociacion fiducial simulada

publica /global_sparse_cloud
publica /global_keyframes
publica /global_mapping/backpressure_active
publica /global_mapping/flow_events
```

## Flujo principal

Callbacks de delta/full snapshot:

1. decodifican y comprometen `RawMapDatabase`;
2. registran KFs nuevos anclados en `GlobalPoseStore`;
3. importan covisibilidad ORB y actualizan scores;
4. procesan asociaciones fiduciales baratas;
5. solicitan una nueva revision publicable;
6. admiten `LoopTask` para KFs materialmente nuevos o con apariencia cambiada.

Estado real diagnosticado el 2026-08-09: `OnOrbMapDelta` mantiene
`live_state_mutex_` desde antes de `InsertDelta` hasta despues de scores,
registro de poses, covisibilidad, fiduciales y admision. Full snapshots y GT
comparten el mismo mutex. Por tanto esta ruta todavia no cumple el contrato de
secciones criticas breves.

El primer fiducial crea el anchor. Una revisit de error alto encola
`FiducialOptimizationTask`; no ejecuta solver dentro del callback.

## Publicacion

`RequestGlobalStatePublication` coalesce solicitudes. El worker de publicacion
captura `RawMapDatabase::CreateStateSnapshot`, poses, tracks y scores bajo
`live_state_mutex_`; `GlobalMapBuilder` y los mensajes se construyen fuera. La
captura no es breve en runtime: `mutex_capture_ms` mezcla espera por el mutex y
coste de copia, con maximos `18.875 s` en `prueba_75` y `27.225 s` en
`prueba_76`.

Nube y `MarkerArray` comparten revision. Un commit raw, anchor, fusion o pose
solicita publicacion y retorna; no existe ACK funcional.

Marcadores principales:

```text
[F1F-GLOBALMAP-BUILD]
[F1F-GLOBALMAP-PUBLISH]
[F1P-CLOUD-BUILD-TIMING]
[F1T-RVIZ-KF-MARKERS]
[F1T-RVIZ-PUBLICATION-COMMIT]
```

## Cola secundaria

Estado:

```text
fiducial_tasks_  FIFO, prioridad maxima
loop_tasks_      FIFO, prioridad normal
secondary_worker_thread_  unico consumidor
```

El worker termina la tarea activa, luego escoge la fiducial mas antigua si
existe y, en otro caso, el loop mas antiguo. `test_secondary_task_order`
protege esta semantica.

`LoopTask` conserva `task_id`, KF query, trigger y revision de admision. La
admission:

- usa KFs nuevos con BoW/descriptores;
- coalesce una clave pendiente equivalente;
- no admite refresh redundante;
- tiene capacidad dura `loop_task_max_pending=4096`;
- no bloquea callbacks si se alcanza el limite.

Limitaciones vigentes:

- no consulta si el submapa esta anclado antes de encolar;
- `RunLoopTask` captura raw, poses, covisibilidad, fusion y scores actuales al
  empezar, potencialmente mucho despues del enqueue;
- no existe marcador por tarea con timestamp/arrival de enqueue que permita
  medir su edad causal exacta;
- la capacidad `4096` evita drops tempranos, pero permite backlog de cientos.

`UpdateMappingBackpressure` calcula metricas de carga, pero publica siempre
`false`. Esto es intencional: el flujo principal puede perder CPU, nunca
esperar a la cola secundaria.

Logs:

```text
[F1K-LOOP-TASK-ADMISSION]
[F1K-SECONDARY-TASK-START]
[F1K-SECONDARY-TASK-END]
[F1K-QUEUE-SUMMARY]
[F1K-BACKPRESSURE]
```

## Ejecucion de `LoopTask`

`RunLoopTask`:

1. captura snapshots raw/pose/covisibilidad/fusion/score;
2. llama a `LoopDetector`;
3. captura/prepara/verifica candidato con `SubcloudLoopVerifier`;
4. aplica salidas tempranas por covisibilidad, identidad o stale;
5. decide fusion o optimizacion;
6. prepara resultados fuera del lock;
7. valida revisiones y compromete bases derivadas;
8. solicita publicacion y termina.

No existe una segunda tarea para fusion u optimizacion.

### Causalidad intra-submapa

`LoopDetector` ignora candidatos con:

```text
same_submap && candidate.local_kf_id > query.local_kf_id
```

Ademas, si la verificacion de un candidato `near_same_submap` propone
`LoopOptimizationCandidate`, el servidor la degrada a
`HoldInsufficientEvidence` y emite `[F1Q-LOOP-OPT-SUPPRESSED]`.

### Rama de fusion

`FusionCandidate` actualiza una copia de `CovisibilityDatabase` y
`FusedLandmarkManager`; tras validar revisiones intercambia el estado y solicita
`loop_fusion_commit`. `RawMapDatabase` queda intacta.

### Rama de optimizacion

Antes del grafo se cuentan soportes `ServerLoopGeometric` previos entre los
submapas implicados. Con soporte insuficiente se registra evidencia de error
alto, pero no se construye grafo.

Con soporte suficiente:

```text
PoseGraphBuilder::BuildForLoopTask
  -> OptimizationManager::RunDryRun
  -> ApplyCandidateResult sobre GlobalPoseStore candidato
  -> ValidatePostApply
  -> ConfirmApply
  -> intercambio live + covisibilidad/fusion
```

El grafo usa `LOOP_RELATIVE`; el candidato es fijo. El log compute incluye IDs,
errores, inliers, soporte y dimensiones. El commit aceptado incluye conteos,
hard fixed y deltas maximos.

## Tarea fiducial

`FiducialOptimizationTask` reutiliza `PoseGraphBuilder`,
`OptimizationManager`, backup/validacion y autoridad de cola de
`GlobalPoseStore`. Solo el commit final toca el pose store live. Una tarea
fiducial pendiente tiene prioridad sobre loops, pero no interrumpe la tarea
activa.

## Telemetria

`TryTraceFlow(edge, phase, payload, task_id, count)` intenta insertar JSON en
una cola acotada. El drenaje publica `std_msgs/msg/String` en
`flow_telemetry_topic`. No serializa payloads pesados ni espera subscribers.

IDs de arista se definen en la topologia web. Entre otros:

```text
wrapper_server_delta / wrapper_server_snapshot
server_raw_delta / server_raw_snapshot
raw_db_pose_new_kf / raw_db_pose_snapshot
fiducial_pose_anchor / fiducial_secondary_queue
secondary_queue_loop_detector / loop_detector_loop_verifier
loop_verifier_loop_decision / loop_decision_pose_graph
pose_graph_optimizer / optimizer_pose_db
raw_db_map_builder / pose_db_map_builder / fused_db_map_builder
server_rviz_cloud / server_rviz_keyframes
```

## Parametros nuevos/relevantes

| Parametro | Default | Uso |
|---|---:|---|
| `loop_task_max_pending` | `4096` | capacidad dura de loops pendientes |
| `loop_optimization_min_prior_supports` | `2` | soportes previos antes del grafo |
| `loop_bow_min_kf_gap_same_submap` | `20` | etiqueta `near_same_submap` |
| `mapping_backpressure_topic` | `/global_mapping/backpressure_active` | compatibilidad; siempre false |
| `flow_telemetry_enabled` | `true` | habilita eventos descartables |
| `flow_telemetry_topic` | `/global_mapping/flow_events` | topic JSON |

Los parametros historicos de grafo, solver, fiducial, raw y publicacion siguen
declarados en el nodo/launch. Consultar el launch para defaults concretos.

## Validacion reciente

- Build final `orbslam3_multi orbslam3_server`: exit `0`.
- Tests de loop/causalidad y prioridad: PASS.
- `prueba_75`: prioridad fiducial correcta y par cercano que mueve `248` KFs;
  `561` loops encolados, `66` ejecutados y `496` aun pendientes en el ultimo
  start.
- `prueba_76`: filtro causal activo y cero commits de poses por loop; `489`
  loops encolados, `84` ejecutados, pico `429`, ultimo valor `414`.
- `prueba_75/76`: `31/66` y `38/84` loops ejecutados pre-anchor;
  request->commit de publicacion maximo `20.283/27.951 s`.

El `gazebo exit 255` de `prueba_76` es posterior a `SIM-DONE` y pertenece al
cleanup del launch.
