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

## Validacion pendiente

Con D1 anclado en fiducial 2 y depth habilitado, la prueba integrada debe
mostrar avance lateral por secciones U, `MOVE_AND_CAPTURE` predominante en
corredores libres, impactos depth que activan coverage, reservas visibles,
relevo `TO_FINISH` y ninguna espera bloqueante entre workers. La simulacion no
debe usar el barrido legacy ni mezclar dos politicas de seleccion.
