# 00_summary - orbslam3_server

## Estado activo

Servidor ROS 2 reconstruido hasta 3R. Mantiene dos workers independientes:

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

El backpressure es OR de cola principal alta, pendientes secundarios criticos
>=64, optimizacion activa o fallo bloqueante. Libera a <=16 criticos, principal
en low y sin optimizacion. Los `FusionRefresh` son mantenimiento observable y
no cierran por si solos el gate. Un goal activo no se cancela; el siguiente
espera.

La fuente fiducial live es exclusivamente visual. `FiducialObjectInterpreter`
asigna visitas por intervalos temporales de `(drone_id,map_epoch,object_id)`;
admite batches fuera de orden dentro del gap configurable. Replay solo reinyecta
observaciones `source=visual_fiducial`; las grabaciones GT legacy se rechazan.
La replica `config/calibration_dron.yaml` conserva el mismo `B_T_C` optico
frontal de Dron (`RPY=-90,0,-90`); la ruta fiducial visual trabaja en camara y
no consume esta extrinseca.

El primer KF observado reserva la candidatura de control de la visita. Los
eventos `secondary_task_lifecycle` delimitan visualmente cada tarea desde
dequeue hasta `done`. Los frustums usan una paleta determinista con saltos
amplios por `map_epoch` mediante `SubmapColor`.

Tras cada commit raw, el principal encola trabajo secundario sin esperarlo. Una
tarea MEDIA aplica covisibilidad y encola las BAJAS causales; cada BAJA abarca
BoW, geometria, decision, fusion 3P o grafo/solver/commit 3Q y fusion posterior.
Un anchor/commit solo marca dirty: el siguiente principal publica.

`SecondaryWorkerLoop()` contiene una barrera de excepciones por tarea. Una
excepcion inesperada se registra como `[F3H-SECONDARY-EXCEPTION]`, completa la
tarea y activa fallo bloqueante, en vez de abortar el proceso del servidor.

Desde 5D, `/global_mapping/get_global_keyframe_pose` responde inmediatamente
con el estado 5C y conserva un unico interes por dron. Los commits principal y
secundario publican revisiones nuevas por
`/dron_X/orbslam/global_keyframe_pose` con QoS reliable/volatile; no hay polling
ni heartbeat.

Las BAJAS se coalescen por huellas semanticas y se revalidan con geometria
exacta al dequeue. Un hijo loop blando sigue rigidamente al KF de apoyo; su
primer fiducial directo reancla todo el submapa como hard y corta la
dependencia.

Un intento de fusion stale o con rollback termina su lifecycle y se completa
antes de encolar una BAJA fresca para el mismo KF. Este retry puede atravesar
el ledger completado, pero no la deduplicacion pendiente/activa; no tiene
limite fijo y queda gobernado por coalescencia y backpressure.

Tras un commit loop o fiducial, todos los KFs movidos se encolan como
`FusionRefresh`. Conservan BoW, geometria, fusion y score, pero no pueden
iniciar otra optimizacion. Las BAJAS nacidas de delta/snapshot son `Full`; si
ambas coalescen, `Full` prevalece. El intent efectivo aparece en dequeue y en
`[F3Q-OPT-START]`. Los KFs movidos se agrupan por region y cada refresh limita
sus candidatos por proximidad de subnubes world.

El apoyo loop es adaptativo 2/4/6. El corredor hard-hard eleva el riesgo, pero
no inmoviliza sus KFs internos. El limite 5 m/20 grados se aplica al movimiento
propuesto de poses ya optimizadas; solo hard permanece fijo permanentemente.
Consenso server 3/60 fija soportes de forma privada durante el solve.

3R configura score geometrico raw y bonus fused, emite telemetria live cada 25
arrivals y conserva `score` en la nube completa. Desde 7E, el RGB temporal de
presentacion se retiro: la GUI deriva el gradiente y la nube publica identidad
estable `(drone_id,map_epoch,local_mp_id)` con `point_step=36`. La
visibilidad sparse solo diagnostica; oclusion numerica queda para Fase 8.
Los defaults de distancia dejan banda neutra 1-5 m con baseline `0.06 m`;
prueba 194 cierra colas en cero con 99 near, 11.433 far y media `0.2596`.

3T separa parámetros en `config/global_map/{runtime,fiducials,optimization,
loop_fusion,scoring,replay_debug}.yaml`. El launch directo usa esta copia;
`replay_debug.yaml` solo se añade explícitamente y `CMakeLists.txt` instala
configuración y launches.

4A añade `config/fiducial_objects.yaml` como contrato semantico canonico de
los objetos visuales. Simulacion instala una replica exacta para deployment;
Servidor conserva `object_id`, poses, caras, `object_T_tag` y el rango inicial
configurable `[1,5] m`. Desde 4D, `fiducial_config_server.py` valida este perfil
y sirve a los wrappers familia, refinamiento, solver, umbral y `tag_id/size_m`.

Desde 4F, el servidor se suscribe por dron a
`orbslam/fiducial_keyframe_observations` con reliable/volatile KeepLast(32).
Entrega cada batch al sidecar de `RawMapDatabase`, procesa matches fuera del
mutex y expone pending/evicted/duplicate/conflict/rejected. El parametro
`fiducial_pending_capacity_per_drone=10` vive en `runtime.yaml`.

Desde 4G+4H, cada match se interpreta en Servidor: rango inclusivo por tag,
fusion robusta multicara, primary determinista, FIFO reciente 50 y visita con
gap 2 s. Cada primary se entrega al `FiducialAnchorManager` existente mediante
`world_T_camera_target` y `source=visual_fiducial`. Se eliminaron por completo
la subscription, buffer, conversion body-camera, parametros y grafo GT
fiducial; el GT de control/Fase 5 permanece independiente.

3S añade el argumento launch `log_level`. `multi_dron.launch.py` pasa `error`
cuando `fase3_logs_terminal=false` e `info` cuando esta activo. Asi se ocultan
los diagnosticos `[F3*]` sin silenciar errores o fallos reales del nodo.

Componentes: `global_map_server.md`, `fiducial_object_interpreter.md`,
`primary_queue.md`, `secondary_queue.md` y `launches.md`.

## Interfaces

```text
subscriptions: /dron_X/orbslam/orb_map_delta
               /dron_X/orbslam/fiducial_keyframe_observations
clients:       /dron_X/orbslam/get_full_map
services:      /global_mapping/get_fiducial_config
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
- prueba 175: runner/tool correctos, ocho lifecycle 3Q completos, un stale
  reintentado, cola final cero, cero fallos duros, servidor RSS maximo
  146.4 MiB y PSI de memoria cero.
- prueba 187: tres commits, 16 refresh altos diferidos, 1047 tareas,
  `pending=0` y cero hard failures;
- prueba 188: scenario/tool correctos, nueve commits loop `Full`, ocho
  fiduciales, dos anchors loop y cero hard failures. Tras el drenaje quedan
  refresh/fusiones descendiendo 323 -> 310, sin `blocking_failure`;
- politica vigente posterior a prueba 212: los fallos secundarios conservan
  log y contador, pero no existe `secondary_blocking_failure_`; el backpressure
  se libera al terminar la optimizacion y depende solo de colas e
  `optimization_active`. El contrato dedicado y la suite del servidor pasan;
- recursos 188: servidor RSS maximo 423.4 MiB, grupo 2014.6 MiB, PSI memoria
  cero y guarda inactiva.
- cierre 3T: CTest 10/10; prueba 195 procesa 741 entradas principales y 1262
  tareas secundarias, termina `pending=0`/`hard_failed=0`, publica 23.978
  puntos con score/rgb y alcanza 249.5 MiB RSS maximo.
- cierre 3S: prueba 196 mantiene el servidor operativo con nivel `error`,
  completa el escenario y no emite ningun marcador `[F3*]`.
- correccion 3Q: build correcto y CTest 12/12. Prueba 219 completa 17/17 pasos,
  logra 22 commits loop de 30 solves y no mueve hard ni supera 5 m/20 grados;
  queda carga residual de reruns y mejora visual desigual en zonas multi-epoch.
