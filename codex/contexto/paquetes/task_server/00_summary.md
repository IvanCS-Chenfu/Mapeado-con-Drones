# task_server - resumen vigente

## Responsabilidad

`task_server` coordina tareas regionales, vuelos de inspeccion y evidencia
voxel global. No controla motores ni calcula depth local: cada dron acepta una
orden corta, la ejecuta autonomamente y devuelve un resultado correlacionado.

Archivo principal: `servidor/task_server/src/task_server_node.cpp` ->
`TaskServerNode`. Buscar workers con `rg -n 'Run.*Worker|PointSelection|TrajectoryPlanning|VoxelMapBuilder' servidor/task_server`.

## Infraestructura que ya existe

La puerta `execution_enabled_` bloquea tambien la puesta en cola y asignacion
inicial de drones elegibles. En modo autonomo el servidor arranca con esa
puerta cerrada durante el bootstrap GT; al recibir
`/mission/set_coverage_execution_enabled=true` vuelve a encolar los drones
elegibles y comienza la asignacion normal. La configuracion monodron de las
pruebas es `config/mission_house_single_drone.yaml` con `drones: [1]`.

Los callbacks que mutan estado de misión, evidencia y mapa comparten `map_callback_group_` mutuamente exclusivo. Aunque el ejecutor tenga dos hilos, la entrada sparse, los resultados autónomos, integración depth, materialización voxel, selección, planificación, monitor y puerta de ejecución se serializan para no acceder concurrentemente a `EvidenceDatabase`, tareas, poses o contextos de workflow.

Para que la exclusión mutua no convierta una ráfaga sparse en una pausa de
misión, `VoxelMapWorker` materializa como máximo una transacción por tick
(`voxel_worker_max_transactions_per_tick=1`). Prioriza de forma general una
transacción depth independiente, pero nunca adelanta otra previa del mismo
keyframe. Además, recibe las `source_id` exactas de
`pending_depth_continuations_`: su transacción y solo su cadena causal previa
del mismo keyframe saltan la cola de evidencia pasiva. Esto evita que una
captura que desbloquea D1/D2 quede detrás de cientos de muestras `depth_*` no
asociadas a un workflow. Cada transacción mantiene atómicos sus sources
reversibles. El marcador `F6G-VOXEL-DELTAS-APPLIED` expone transacciones
pendientes, `continuation_sources` y `elapsed_ms`.

La materialización individual no obliga a recalcular navegación e inflación por
cada muestra pasiva. `voxel_worker_flush_interval_ms=500` agrupa ese refresh y
la publicación de fondo. Si una `source_id` esperada por una continuación
profunda se materializa, fuerza `FlushVoxelChanges` inmediatamente: el
workflow, coverage, reservas y STOP ven esa revisión sin esperar el intervalo.
Esto deja turnos al callback de pose global de keyframe, que hace materializable
la evidencia depth, y evita que el refresco de fondo retrase a D1/D2.

`WorkflowScheduler` mantiene las colas FIFO `TASK_ASSIGNMENT`,
`POINT_SELECTION`, `TRAJECTORY_PLANNING`, `DEPTH_INTEGRATION` y
`ACTIVE_TRAJECTORY_MONITOR`. Toda entrada transporta como minimo
`drone_id`, `task_id`, `workflow_id`, `command_id` y `map_epoch`; se deduplica
por identidad y un reintento vuelve al final de su cola.

El servidor envía `/<drone_id>/autonomous_command` de forma asincrona. El
`accepted` solo confirma que el dron guardo la orden; el worker queda libre.
Al terminar, el dron llama `/mission/report_autonomous_result`. Su callback
responde rapido y encola el resultado; no planifica, integra ni espera al
dron dentro de la llamada ROS.

Existe una unica orden normal activa por dron. STOP es asincrono, reemplaza la
referencia local por una trayectoria de retencion y usa generacion para
invalidar terminales antiguos. Nunca se cancela una trayectoria sin enviar una
referencia de parada.

`EvidenceDatabase` conserva evidencia local/reversible por KF y fuente.
`KeyframeEvidenceWorker` escribe fuentes nuevas; `VoxelMapBuilder` es el unico
que las proyecta al mapa mundial por deltas y publica `map_revision`, el delta
de voxeles y `sources_applied`. Una continuacion depth se libera solo al
recibir los `source_id` de su propio resultado, nunca por una senal global ni
por KFs de otro dron.

## Contrato objetivo de la migracion 6H--6J

El comportamiento antiguo de elegir un voxel por score `0.2..0.6`, registrar
intervalos lineales de fachada o ejecutar `RunFacadeWorker` como barrido es
transitorio y debe desaparecer. El contrato vigente a implementar es:

```text
asignar subROI
  -> seleccionar seccion U pendiente y pose de inspeccion
  -> LOOK_AND_CAPTURE solo si pose/corredor sigue UNKNOWN
  -> D* estrictamente FREE
  -> ActiveTrajectoryMonitor reserva, publica y vigila
  -> MOVE_AND_CAPTURE frente a la fachada
  -> resultado local -> DepthIntegration
  -> EvidenceDatabase -> VoxelMapBuilder
  -> sources_applied -> siguiente seleccion o revalidacion
```

La U de coverage ocupa las tres caras del subROI opuestas a la cara mas cercana
al centro del ROI global, a dos voxeles de sus limites. Sus secciones son
volumenes perpendiculares al avance local y se recortan por diagonales de
45 grados en las esquinas. La U no entra en D*, reservas ni inflacion; GUI la
muestra solo al seleccionar la tarea.

Cada seccion pendiente vale `0` y una activa vale `10`, sin suma. El estado se
deriva de claims reversibles por fuente depth/KF: un `depth_occupied` frontal
de `vista_pared` o `VIEW_ADVANCE` reclama su seccion espacial y

Cada observación depth válida siempre puede dejar rayos `depth_free`, pero solo
emite `depth_direct_free` y `depth_occupied` cuando `normal_valid` es cierto,
la normal se puede normalizar y el rayo cámara-superficie cumple la incidencia
frontal configurada. El nodo aislado conserva `30 grados`, mientras el launch
Fase 6 entrega `45 grados` por defecto
(`abs(dot(normal, rayo)) >= cos(incidencia_maxima)`).
Llegar a una pose de inspección no garantiza esa condición: una pared oblicua,
una normal no fiable o una vista sin impacto directo deja únicamente `FREE` y
no activa coverage. La normal no se infiere del yaw de la orden: `stereo` la
estima por normales locales de vecinos de la nube depth aceptada, descarta
orientaciones poco horizontales y exige al menos 12 muestras locales con un
clúster angular dominante de confianza `>= 0.7`. Textura pobre, discontinuidades
o la mezcla de superficies en el frustum pueden fragmentar ese clúster aunque
la cámara apunte visualmente hacia una pared.

`depth_coverage_neighbor_sections=2` secciones contiguas a cada lado de la U,
sin cerrarla por la cara abierta. Cada voxel depth `OCCUPIED=1` reclama tambien
la seccion volumetrica donde cae, de modo que una captura puede activar varias.
Al reproyectar, invalidar o borrar una fuente se retiran sus claims y la
seccion vuelve a `0` si no conserva otra evidencia. En una esquina sin fachada,
una vista valida que da FREE fiable hasta 4 m sin impacto la activa como
`SIN_FACHADA`; cuenta exactamente igual. Evidencia incompleta o
`TRACKING_RISK` no la activa.

Los claims se resuelven por `task_id`, no por el dron que poseia la tarea al
capturar la evidencia. Asi un subROI que pasa a `TO_FINISH` conserva tanto sus
claims existentes como la posibilidad de retirarlos cuando se reproyecte o se
elimine la fuente, aunque otro dron retome la tarea.

`PointSelectionWorker` elige una seccion pendiente, busca un voxel de fachada
`OCCUPIED` score `>0.4` o una sonda geometrica si falta, y calcula la pose por
coste de cobertura, pared, altura y desplazamiento. La preferencia de pared es
`facade_preferred_wall_distance_m=4.0`. La orientacion mira a la fachada por
la normal local de la seccion; D* no modifica ese objetivo perceptivo.

Si pose y ruta son FREE, se despacha `MOVE_AND_CAPTURE` y el dron toma depth
de pared al llegar. Si la pose es UNKNOWN, `LOOK_AND_CAPTURE` solo produce
FREE; tras materializar se revalida la misma pose. Si un corredor contiene
UNKNOWN, D* solo ejecuta el prefijo FREE hasta el ultimo waypoint anterior,
captura `VIEW_ADVANCE` mirando al objetivo visual original y espera sus fuentes
materializadas antes de reseleccionar. Esa captura aporta FREE y OCCUPIED
frontal fiable; este ultimo activa exclusivamente la seccion U del candidato
que origino el avance. D* no cruza UNKNOWN.
Si la mirada deja la pose original UNKNOWN pero existe una pose FREE cercana
al objetivo visual, se ejecuta el mismo `VIEW_ADVANCE` con captura que un
prefijo FREE de D* y se reelige tras materializarlo; no se presenta como una
vista de pared.
Si el recheck posterior a depth confirma que el objetivo sigue `FREE` pero
queda no transitable por inflación, `RequeueAppliedUnknownDepthContinuation`
no lo reencola como una nueva selección silenciosa: conserva
`TRAJECTORY_PLANNING`, marca `fallback_free_advance` y registra
`action=prefix_free`. La primera selección aplica la misma regla: un objetivo
`FREE` que el perfil inflado no puede transitar entra directamente en
`TRAJECTORY_PLANNING` y registra
`F6K-POINT-SELECTION-FREE-INFLATED action=prefix_free`; no queda girando en
`POINT_SELECTION`. `RunTrajectoryPlanningWorker` reutiliza entonces su plan
exploratorio solo para hallar el waypoint FREE estricto más lejano y despacha
`VIEW_ADVANCE`. Si no existe ningún prefijo seguro, marca solo la sección U
candidata como excluida temporalmente, registra
`F6I-PREFIX-FREE-UNAVAILABLE action=reselect` y selecciona otro punto: no la
marca como cubierta ni repite la misma pose. No relaja los vetos de `UNKNOWN`,
`OCCUPIED` ni `RESERVED`.

`TrajectoryPlanningWorker` no espera al vuelo. `ActiveTrajectoryMonitor`
reserva antes de enviar la orden, publica una sola polilinea activa y la capa
`RESERVED`, solicita STOP si aparece ocupacion relevante y limpia al terminal.
Una reserva no cambia el estado base y desaparece al liberar el corredor.

`VoxelMapBuilder` devuelve las fuentes ocupadas que proyecta y las que retira.
`TaskServerNode` mantiene el indice `source_id -> claims`, derivado en ese
mismo tick: un impacto frontal reclama el candidato y sus vecinos U, y cada
celda `OCCUPIED=1` reclama tambien su rebanada espacial. Una fuente que se
mueve vuelve a calcular sus claims; una tombstone los elimina. El vector de
secciones activas es solo una cache derivada de ese indice. No usa score sparse
ni ocupacion sin procedencia depth. Asi la continuacion posterior consume el
mapa y el coverage de una misma revision.

Un STOP puede cancelar el control antes del terminal normal, que el dron
reporta como `REJECTED`. Si el resultado incluye depth valido, se persiste e
integra; el estado del movimiento solo impide reanudarlo y su continuacion pasa
por `sources_applied` antes de seleccionar de nuevo.

Al activar todas las secciones, la tarea es `COMPLETED`. Si el dron alcanza el
extremo junto a la cara abierta dejando secciones pendientes, la tarea es
`TO_FINISH` y vuelve a la cola de asignacion para que otro dron proximo pueda
retomarla.

## Estado de codigo y limites

La infraestructura de colas, resultados correlacionados, fuentes por KF,
materializador incremental, U reversible, monitor de trayectorias y STOP ya
esta presente. La validacion integrada pendiente debe confirmar la progresion
real de secciones, reservas visibles y relevo; no debe reinterpretarse como
una vuelta a la seleccion legacy `0.2..0.6`.

`execute_facade_sweeps` es una puerta independiente y por defecto esta en
`false`. Solo activa `RunFacadeWorker`, la ruta legacy que selecciona candidatos
propios y llama `inspect_facade`; no es el dispatcher correlacionado moderno de
`MOVE_AND_CAPTURE`. Las pruebas que requieran un destino concreto no deben
activarla ni usarla como sustituto de su workflow.

La inflacion de ocupados usa coste alto `100`; `OCCUPIED`, `RESERVED` y
`UNKNOWN` siguen vetados. El margen adicional
`extra_obstacle_clearance_voxels=1` se suma a la semidimension registrada del
dron; D1 queda con inflacion `(3,3,2)` y reserva fisica `(2,2,1)`. La burbuja
`start_escape_radius_voxels=4` permite
salir de una inflacion que cubra celdas raw FREE junto al dron, pero no relaja
los tres vetos. `unknown_fallback_free_radius_voxels=8` es el radio de una
alternativa FREE tras una mirada que no despeja la pose objetivo.

`debug_facade_dstar_failure=false` solo diagnostica fallos de ruta estricta:
emite estados raw/navegables de inicio y meta, mas frontera FREE. No altera la
politica de planificacion.



### Arranque de autonomia

La asignación solo marca la tarea como ASSIGNED y espera su TaskReport. Al recibir
la aceptación, TaskServerNode crea o comprueba el runtime de fachada y encola
POINT_SELECTION con la revisión posterior a la aceptación. Así el primer
workflow no puede despacharse antes de que la tarea y su runtime estén listos.
Si una pose o un runtime deja de estar disponible transitoriamente, el worker
reencola el item en vez de perderlo; una tarea obsoleta o reasignada sí se
elimina al no coincidir su propietario.

/mission/set_coverage_execution_enabled sigue usando map_callback_group_ para
preservar la exclusión mutua del estado compartido. El scenario runner trata
la operación idempotente como reintentable: envía hasta tres solicitudes y
espera 30 s por cada una. Una respuesta tardía ya no declara por sí sola que el
handoff haya fallado ni cancela el escenario.

## Instrumentacion 8.4-A

`TaskServerNode` ofrece el servicio `std_srvs/Trigger`
`/mission/test_view_unknown_right` solo cuando
`test_view_unknown_right_enabled=true` (default `false`). Para D1 con pose
global vigente, calcula un `visual_target` a
`test_view_unknown_right_distance_m` en `yaw_actual -
test_view_unknown_right_angle_deg` y lo despacha por el mismo
`DispatchAutonomousWorkflowCommand` como `LOOK_AND_CAPTURE`. No llama depth de
forma directa ni cambia seleccion, D*, coverage o reglas de materializacion.

El workflow se etiqueta `test_view_unknown_right:<revision>`. Una vez aplicadas
sus fuentes, `ProcessAppliedDepthContinuations` publica
`[F8A-VIEW-UNKNOWN-TEST-APPLIED]` y suprime solo su continuacion para aislar la
prueba de un `VIEW_ADVANCE` posterior. El marcador
`[F6F-DEPTH-SOURCES-WRITTEN]` incluye los contadores por tipo `free`,
`direct_free` y `occupied`; para `VIEW_UNKNOWN` la evidencia esperada es
`free>0`, `direct_free=0`, `occupied=0`.


## Instrumentacion 8.4-B

`/mission/test_view_wall_fixed` existe solo cuando
`test_view_wall_fixed_enabled=true` (default `false`). El Trigger asigna a D1
la tarea U que contiene `test_view_wall_fixed_(x,y,z)`, conserva ese contexto y
despacha el `MOVE_AND_CAPTURE` correlacionado a ese objetivo con
`test_view_wall_fixed_yaw_deg`. No realiza un movimiento GT ni permite que el
selector sustituya el objetivo. La ruta exige un corredor navegable FREE; si
aun no existe, queda armada y espera nueva evidencia sin cruzar UNKNOWN u
OCCUPIED.

El workflow `test_view_wall_fixed:<revision>` usa `VIEW_WALL`, conserva las
fuentes y claims normales, y solo despues de materializarlas publica
`[F8B-VIEW-WALL-FIXED-APPLIED]` para suprimir su continuacion. Es
instrumentacion opt-in de la prueba, no una politica productiva.

El parametro test_view_wall_fixed_ignore_corridor_stops=false conserva por
defecto la parada ante cambios OCCUPIED/INFLATED en el corredor. Solo para
repetir 8.4-B puede activarse en ese workflow y publica el marcador
F8B-VIEW-WALL-FIXED-CORRIDOR-STOP-SUPPRESSED; no altera otros workflows.
El parametro test_view_wall_fixed_require_known_free=true preserva el corredor
FREE estricto; solo al fijarlo a false para 8.4-B permite atravesar UNKNOWN y
mantiene esa misma politica en la replanificacion del workflow de prueba.

## Validacion pendiente

Con D1 anclado en fiducial 2 y depth habilitado, la prueba integrada debe
mostrar avance lateral por secciones U, `MOVE_AND_CAPTURE` predominante en
corredores libres, impactos depth que activan coverage, reservas visibles,
relevo `TO_FINISH` y ninguna espera bloqueante entre workers. La simulacion no
debe usar el barrido legacy ni mezclar dos politicas de seleccion.
