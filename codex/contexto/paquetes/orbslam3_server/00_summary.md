# 00_summary - orbslam3_server

## Estado activo

Servidor ROS 2 reconstruido hasta 3P. Mantiene dos workers independientes:

```text
wrapper -> PrimaryQueue FIFO -> PrimaryWorker -> SparseGlobalBackend -> ROS
raw ChangeSet -> SecondaryTaskQueue MEDIA -> covisibilidad -> LoopTask BAJA
fiducial MAX / loop BAJA -> SecondaryWorker -> backend commit dirty
```

`PrimaryWorker` sigue siendo el unico que construye/publica
`/global_sparse_cloud` y `/global_keyframes`. `SecondaryWorker` revalida,
construye grafo, optimiza, valida y compromete; no publica ni despierta al
principal.

La cola secundaria tiene carriles MAX/HIGH/NORMAL, mostrados funcionalmente
como MAXIMA/MEDIA/BAJA, FIFO por carril y un unico worker no preemptivo. Los
tres payloads reales son `FiducialOptimizationTask`, `DatabaseUpdateTask` y
`LoopTask`.

El backpressure es OR de cola principal alta, cola secundaria >=64,
optimizacion activa o fallo bloqueante. Libera a <=16 secundarios, principal en
low y sin optimizacion. Un goal activo no se cancela; el siguiente espera.

Live asigna `fiducial_visit_id` al entrar en el radio. Replay v3 usa el valor
persistido; v1/v2 lo infiere por submapa, fiducial y timestamp ordenado. El
record incremental activo es v3 y sigue siendo delta-only.

El primer KF observado reserva la candidatura de control de la visita. Los
eventos `secondary_task_lifecycle` delimitan visualmente cada tarea desde
dequeue hasta `done`. Los frustums usan una paleta determinista con saltos
amplios por `map_epoch` mediante `SubmapColor`.

Tras cada commit raw, el principal encola trabajo secundario sin esperarlo. Una
tarea MEDIA aplica covisibilidad y encola las BAJAS causales; cada BAJA abarca
BoW, geometria, decision y, si el error es bajo, fusion 3P. Un anchor loop o un
commit de fusion solo marca dirty: el siguiente principal actualiza y publica.

`SecondaryWorkerLoop()` contiene una barrera de excepciones por tarea. Una
excepcion inesperada se registra como `[F3H-SECONDARY-EXCEPTION]`, completa la
tarea y activa fallo bloqueante, en vez de abortar el proceso del servidor.

Las BAJAS se coalescen por huellas semanticas y se revalidan con geometria
exacta al dequeue. Un hijo loop blando sigue rigidamente al KF de apoyo; su
primer fiducial directo reancla todo el submapa como hard y corta la
dependencia.

Un intento de fusion stale o con rollback termina su lifecycle y se completa
antes de encolar una BAJA fresca para el mismo KF. Este retry puede atravesar
el ledger completado, pero no la deduplicacion pendiente/activa; no tiene
limite fijo y queda gobernado por coalescencia y backpressure.

Componentes: `global_map_server.md`, `primary_queue.md`,
`secondary_queue.md`, `ground_truth_buffer.md` y `launches.md`.

## Interfaces

```text
subscriptions: /dron_X/orbslam/orb_map_delta
               /dron_X/sensor/GT/pose   solo fiducial simulado live
clients:       /dron_X/orbslam/get_full_map
publishers:    /global_mapping/backpressure_active
               /global_mapping/flow_events
               /global_sparse_cloud
               /global_keyframes
```

## Validacion

- build final correcto; CTests funcionales 9/9 en dominio, 4/4 en servidor y
  contrato web 1/1;
- live 148 preservada como fallida: una carrera de control produjo hard
  constraint y backpressure bloqueante;
- replay 150 reproduce esa entrada y confirma la correccion con un commit,
  cero hard y cola vacia;
- live 151 completa: 11 tareas, 3 commits, 8 `STALE`, cero hard;
- replay 153: secundaria `pending=0`, 806 procesadas y cero hard;
- live 154: B anclado por loop, 371 principales procesadas, 2424 secundarias,
  `max_active=1`, cola vacia y servidor RSS maximo 146.9 MiB.
- prueba tipica 156: 497 principales y 1060 secundarias, cola final vacia,
  `max_active=1`, cero hard failures, reanchor post-loop de 32 KFs y tres
  optimizaciones fiduciales completas.
- prueba 160: 486 principales, 1116 secundarias, `pending=0`, `hard_failed=0`,
  `max_active=1`, 56 commits de fusion y cierre limpio del servidor.
- prueba 161: 484 principales y 1162 secundarias, `pending=0`, `hard_failed=0`,
  19/19 retries encolados e iniciados, ocho commits de fusion y cuatro commits
  fiduciales completos; recursos y cierre correctos.
