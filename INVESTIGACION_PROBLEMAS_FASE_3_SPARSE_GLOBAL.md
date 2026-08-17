# Investigación y propuesta de solución - Fase 3 Sparse Global

## 0. Estado y alcance

Este documento sustituye la lista inicial de hipótesis por un diagnóstico ya
contrastado con el código, los contratos, los historiales y los logs reducidos
de `prueba_74`, `prueba_75` y `prueba_76`.

```text
Preparación: EN_DEBATE
Acuerdo cerrado: no
Autorización funcional: PENDIENTE
Cambios de código realizados por este documento: ninguno
Pruebas nuevas ejecutadas: ninguna
```

El objetivo es preparar una reestructuración que cumpla simultáneamente:

1. el flujo principal recibe y conserva deltas/snapshots, mantiene poses world
   disponibles y publica KFs/MapPoints aunque el trabajo secundario sea lento;
2. existe un único worker secundario y una sola tarea secundaria activa;
3. una tarea activa no se interrumpe;
4. una tarea fiducial pendiente tiene prioridad sobre loops pendientes;
5. un submapa no anclado no produce trabajo de loop;
6. las bases nunca se observan parcialmente actualizadas;
7. ni los cálculos secundarios ni el visualizador bloquean el flujo principal;
8. se conservan como máximo 50 muestras GT antiguas por dron;
9. anchors y optimizaciones se representan completos y con baja latencia en el
   visualizador web.

No se propone cambiar thresholds de BoW, RANSAC o solver. Tampoco se propone
paralelizar las tareas secundarias.

### Índice de lectura

- `1-3`: conclusión, evidencia y alternativas.
- `4-5`: arquitectura recomendada y sincronización.
- `6-10`: solución individual de los seis problemas.
- `11-14`: ownership, subfases, archivos y migración.
- `15-18`: pruebas, criterios, decisiones pendientes y recomendación final.

## 1. Conclusión ejecutiva

La causa estructural no es simplemente que haya "demasiados mutex". El
problema es que un único `live_state_mutex_` protege a la vez bases distintas y
se mantiene durante copias completas, reconciliación, score, covisibilidad,
fiduciales, admisión y commits complejos. Además, el nodo ROS usa un executor
monohilo. Por ello una operación lenta impide incluso ejecutar callbacks de GT
que conceptualmente no deberían depender de ella.

La solución recomendada es:

```text
callbacks ROS ligeros
        |
        v
MappingStateCoordinator (único escritor de las bases)
        |                         ^
        | entradas inmutables     | propuestas de commit
        v                         |
SecondaryTaskWorker único --------+
        |
        +-> BoW / matching / RANSAC / decisión / fusión u optimización

MappingStateCoordinator
        |
        +-> PublicationChange -> PublicationWorker -> GlobalMapBuilder -> RViz2
        |
        +-> FlowEvent -> telemetría no bloqueante -> navegador
```

El worker secundario no tocaría contenedores live ni mantendría un lock que
pueda necesitar la ingesta. Calcula con entradas privadas y entrega una
`DerivedCommitProposal`. El coordinador valida solo las revisiones consumidas,
aplica un lote corto y publica una nueva revisión coherente. Así se preserva la
idea original de que el flujo principal es la autoridad que modifica las
bases, aunque el cálculo que origina el cambio sea secundario.

Esta opción exige rehacer una parte importante de `global_map_server.cpp`, pero
ofrece una garantía más fuerte y comprobable que repartir el mutex actual.

## 2. Evidencia y causas demostradas

| Problema | Evidencia vigente | Causa inmediata | Causa estructural |
|---|---|---|---|
| Publicación atrasada | request -> commit máximo `20.283/27.951 s` en 75/76 | captura esperando/copiando bajo `live_state_mutex_` | bases y consumidores comparten un lock global |
| Callback de delta lento | tramo posterior al commit raw hasta `12.829/17.881 s` | score, poses, covisibilidad, fiduciales y scheduling dentro del lock | callback demasiado ancho y executor monohilo |
| Loops pre-anchor | `31/66` starts en 75 y `38/84` en 76 | `ScheduleLoopTasks()` no consulta anchor | falta estado explícito de elegibilidad |
| Backlog | 75: `561` encoladas y al menos `496` pendientes; 76: `489`, pico `429`, al menos `414` pendientes | se admite trabajo más rápido de lo que se consume | pre-anchor, capturas completas, deduplicación insuficiente y backpressure desactivado |
| Falta de optimización en 76 | `fid=1` aparece 13 veces en 75 y 0 en 76 | no se creó observación absoluta, luego no podía existir tarea fiducial | GT comparte callback/lock y el executor no atiende su cola durante callbacks largos |
| Movimiento tras optimización | en 75 hay otros dos commits de loop y otro fiducial posteriores | una corrección aceptada fue sustituida por commits aceptados posteriores | falta trazabilidad/linaje visible y validación de no regresión global |
| Grafo web atrasado | cola cliente `400 x 110 ms`, pulsos `520 ms`, replay SSE de hasta 512 eventos | frontend reproduce lentamente eventos antiguos | no existe separación live/replay ni política por importancia |

### 2.1 Puntos concretos del código actual

- `GlobalMapServer::OnOrbMapDelta()` adquiere `live_state_mutex_` antes de
  `RawMapDatabase::InsertDelta()` y lo conserva durante reconciliación, score,
  poses, publicación, covisibilidad, GT, fiduciales y admisión de loops.
- `OnFullSnapshotResponse()` tiene la misma frontera amplia.
- `BuildAndPublishGlobalState()` copia `RawMapDatabase`, `GlobalPoseStore`,
  `LandmarkScoreManager` y `FusedLandmarkManager` bajo ese mutex.
- `RunLoopTask()` vuelve a copiar raw, poses, covisibilidad, fusión, score y el
  estado BoW bajo el mismo mutex para cada tarea.
- `CommitComputedLoopCandidate()` copia bases candidatas y ejecuta decisión,
  apply, validación y fusión mientras mantiene el mutex live.
- `RawMapDatabase::CreateStateSnapshot()` copia todos los submapas con KFs,
  MapPoints y mensajes ORB vigentes.
- `RawMapDatabase::CreateSubsetSnapshot()` sigue recorriendo todos los
  submapas/KFs y copia revisiones globales, aunque limite parte de la geometría.
- `ScheduleLoopTasks()` coalesce un KF solo mientras ya está pendiente; no
  comprueba anchor ni conserva un ledger completo de trabajo terminado.
- `UpdateMappingBackpressure()` calcula el latch `high/low`, pero llama
  deliberadamente a `PublishMappingBackpressure(false, ...)`.
- `main()` usa `rclcpp::spin(...)`, por lo que todos los callbacks ROS del nodo
  se sirven con un `SingleThreadedExecutor`.
- el subscriber GT usa `KeepLast(50)`, pero simulación configura un buffer
  interno de 5000 muestras. Si el executor está ocupado, las muestras se
  pierden antes de llegar a ese buffer.
- el buffer GT es un `std::vector` y elimina el primer elemento con
  `erase(begin())`, operación O(N) repetida cuando está lleno.
- el evento web actual ya es pequeño: secuencia, timestamp, arista, fase,
  detalle, `task_id` y cantidad. El tamaño del payload no explica por sí solo
  decenas de segundos de retraso.

## 3. Alternativas de arquitectura

### 3.1 Alternativa A - Corrección incremental con mutex por base

Cambios principales:

- sustituir `live_state_mutex_` por mutex independientes de raw, poses,
  covisibilidad, fusión, score, cola, GT y publicación;
- sacar logs, construcción de mensajes, BoW, geometría, solver y fusión fuera
  de todos los locks;
- crear capturas específicas por tarea, sin `CreateStateSnapshot()` completo;
- usar `MultiThreadedExecutor` y callback groups separados;
- aplicar cada commit secundario bajo el mutex exclusivo de la base afectada.

Ventajas:

- menor cantidad de código nuevo;
- permite corregir primero GT, gating y publicación;
- conserva gran parte de las clases actuales.

Limitaciones:

- una lectura secundaria protegida con `shared_mutex` todavía puede retrasar a
  un writer principal;
- un commit que afecte varias bases necesita orden de locks y rollback;
- es fácil reintroducir secciones críticas grandes;
- no demuestra de forma estricta que un secundario nunca bloquee la ingesta.

Conclusión: útil como transición, pero no es la solución final recomendada.

### 3.2 Alternativa B - Coordinador de estado con único escritor

El `MappingStateCoordinator` es el único propietario mutable de:

```text
RawMapDatabase
GlobalPoseStore
CovisibilityDatabase
FusedLandmarkManager
LandmarkScoreManager
LoopPairAttemptDatabase
```

Los callbacks le envían eventos ligeros. El worker recibe datos privados y
devuelve propuestas. El publicador recibe cambios/versiones que puede consumir
sin consultar las bases live.

Ventajas:

- ningún secundario adquiere un mutex que necesite el flujo principal;
- no hay dos writers de una base ni orden de locks entre bases;
- un commit multibase es atómico porque el coordinador no expone la nueva
  revisión hasta terminar el lote;
- simplifica la autoridad de poses y la trazabilidad;
- encaja exactamente con la idea de flujo principal como escritor.

Costes:

- exige dividir responsabilidades del servidor de más de 9000 líneas;
- `GlobalMapBuilder` y loop necesitan read models específicos, no copias
  completas improvisadas;
- requiere migración por bloques y tests deterministas antes de Gazebo.

Conclusión: es la alternativa recomendada.

### 3.3 Alternativa C - Bases completamente inmutables/RCU

Cada commit produciría una nueva versión estructuralmente compartida y los
lectores obtendrían un `shared_ptr<const State>` mediante intercambio atómico.

Ventajas:

- lecturas O(1) sin lock;
- snapshot causal natural por tarea;
- rollback y publicación de revisiones resultan sencillos.

Riesgos:

- los `std::map` y mensajes ROS actuales no son estructuras persistentes;
- copiar mapas de KFs/MPs en cada delta seguiría siendo caro;
- una cola larga puede retener muchas versiones y memoria;
- implementarlo bien requeriría estructuras por páginas/chunks o una nueva
  dependencia persistente.

Conclusión: no se recomienda como primera reescritura completa. Sí conviene
usar inmutabilidad en los DTO/read models que cruzan threads.

## 4. Arquitectura recomendada en detalle

### 4.1 Threads y responsabilidades

| Thread/callback group | Responsabilidad | Puede escribir bases live |
|---|---|---|
| ROS ingress | validar mensaje, asignar metadatos de recepción y encolar | no |
| ROS GT | convertir pose y actualizar ring de 50 muestras | solo `GroundTruthBuffer` |
| `MappingStateCoordinator` | commits raw, anchors, poses principales y propuestas secundarias | sí, único writer |
| `SecondaryTaskWorker` | una tarea completa de BoW a fusión/optimización | no, propone commit |
| `PublicationWorker` | mantener read model, construir y publicar RViz2 | no |
| telemetría | transportar/coalescer eventos | no |

El nodo debe usar `MultiThreadedExecutor`, pero eso no significa ejecutar dos
tareas secundarias. Los callback groups separan recepción ROS/GT; el scheduler
mantiene exactamente un `SecondaryTaskWorker`.

### 4.2 Eventos del coordinador

Tipos mínimos:

```text
IngestDelta
IngestFullSnapshot
RegisterFiducialObservation
ApplyDerivedCommit
PublicationCommitted
Shutdown
```

La cola principal nunca contiene BoW, RANSAC, nubes ni grafos. Los eventos de
ingesta tienen precedencia sobre trabajo administrativo. Para evitar starvation
del commit secundario puede aplicarse una política acotada, por ejemplo procesar
como máximo N ingestas consecutivas antes de un commit ya preparado. El commit
debe tener presupuesto de milisegundos, no segundos.

### 4.3 Read models en vez de snapshots completos

No se debe crear una copia completa de todas las bases por publicación o tarea.
Se proponen tres vistas con propósito concreto:

1. `PublicationChange`:
   - IDs y registros raw nuevos/modificados/retirados;
   - poses world cambiadas;
   - cambios de fused tracks y scores;
   - revisión coherente final.

2. `LoopIndexUpdate`:
   - KF elegible, submapa, revisión de apariencia, BoW compacto,
     `arrival_id` y covisibilidad necesaria para filtros baratos.

3. `LoopGeometryInput`:
   - únicamente query, candidato y ventanas seleccionadas;
   - poses y MapPoints/descriptores de esos KFs;
   - revisiones que deberán validarse al hacer commit.

El publicador mantiene su propia réplica derivada e incremental. El worker
mantiene el índice BoW incremental y solo solicita/materializa geometría para
los candidatos que realmente pasan filtros. Ninguno vuelve a copiar todo
`RawMapDatabase`.

### 4.4 Commit secundario

Una `DerivedCommitProposal` debe contener:

```text
task_id
task_type
base_revisions consumidas por entidad
pose_patch
covisibility_patch
fused_track_patch
score_patch, si procede
loop_attempt_patch
validation_summary
```

Secuencia:

1. el worker calcula y valida el candidato privado;
2. encola la propuesta al coordinador y puede esperar su resultado;
3. el coordinador compara solo revisiones realmente consumidas;
4. si son válidas, aplica el lote sin publicar estado intermedio;
5. incrementa `DerivedStateRevision` y `pose_revision` cuando corresponda;
6. emite `PublicationChange` y el resultado del commit;
7. la tarea termina como `COMPLETED`, `REJECTED` o `STALE`;
8. el worker escoge la siguiente tarea.

Que el worker espere la respuesta de su commit no bloquea el flujo principal.
La tarea sigue terminando exactamente cuando las bases han quedado escritas.

## 5. Solución del problema 1 - Mutex y flujo principal

### 5.1 Regla de sincronización

La frase "leer sin mutex" solo es segura si se lee una copia privada, un DTO
inmutable o un objeto con vida estable. Leer directamente un `std::map` mientras
otro thread lo modifica es una condición de carrera aunque el lector no escriba.

Regla final propuesta:

```text
contenedor live mutable -> solo MappingStateCoordinator
lectores externos       -> DTO/read model inmutable
mutex                    -> colas pequeñas y GroundTruthBuffer
```

No se usará un `shared_mutex` mantenido durante una iteración global como
solución final, porque el lector secundario podría volver a bloquear al writer.

### 5.2 Secuencia de delta

```text
ROS recibe delta
-> encola IngestDelta y retorna
-> coordinador ejecuta InsertDelta
-> obtiene RawInsertResult/ChangeSet
-> actualiza poses principales, covisibilidad y score incrementales
-> procesa asociación fiducial disponible
-> emite PublicationChange
-> actualiza elegibilidad/ledger de loops
-> termina evento principal
```

No forman parte de esta secuencia: BoW global, matching, RANSAC, construcción de
subnubes, grafo, solver, fusión calculada, HTML ni espera de RViz2/web.

### 5.3 Secuencia de snapshot

El snapshot se procesa como diff mediante `RawInsertResult`. Un snapshot
materialmente idéntico solo actualiza journal/metadatos necesarios y no crea
publicación, BoW o loops nuevos. Un KF existente solo vuelve a ser elegible si
cambia una revisión que el loop consume realmente.

## 6. Solución del problema 2 - Gating pre-anchor

### 6.1 Estado explícito por submapa

```text
UNANCHORED
ANCHOR_COMMITTED_WAITING_PUBLICATION
ANCHORED_BACKFILL
ACTIVE
```

Mientras está `UNANCHORED`:

- raw conserva todos los KFs/MapPoints;
- se actualiza como máximo un registro compacto de elegibilidad por KF;
- no se crea `LoopTask` ni se inserta el KF en el índice BoW global;
- no consume capacidad del worker.

### 6.2 Primer anchor

Orden recomendado:

1. commit atómico de anchor y poses world;
2. `PublicationChange` de prioridad crítica para mostrar el submapa;
3. publicación ROS de esa revisión;
4. `PublicationCommitted(anchor_revision)` notifica al scheduler;
5. transición a `ANCHORED_BACKFILL`;
6. liberar KFs acumulados por `local_kf_id` ascendente y después `arrival_id`;
7. transición a `ACTIVE` cuando todos tengan ticket o estén deduplicados.

No se espera un ACK de RViz2. Se espera únicamente a que el servidor haya
publicado el mensaje ROS de la revisión de anchor. Esa espera afecta al
backfill secundario, nunca a ingesta, poses o publicación principal.

### 6.3 KFs posteriores

En estado `ACTIVE`, cada KF materialmente nuevo obtiene pose world y
`PublicationChange` inmediatamente. Su ticket de loop se crea después y no es
precondición para RViz2.

## 7. Solución del problema 3 - Duplicados, backlog y backpressure

### 7.1 Ledger idempotente

Clave propuesta:

```text
LoopWorkKey = (RawKeyFrameId, appearance_revision, geometry_revision_relevante)
```

Estados:

```text
DEFERRED_UNANCHORED
QUEUED
ACTIVE
COMPLETED
STALE
DIRTY_AFTER_RUN
```

Reglas:

- el mismo `LoopWorkKey` no se ejecuta dos veces;
- si llega la misma revisión por delta y snapshot, se coalesce;
- si un KF cambia de verdad mientras está pendiente, se conserva solo la
  revisión más nueva;
- si cambia mientras está activo, se marca `DIRTY_AFTER_RUN` y al terminar se
  crea como máximo un ticket nuevo;
- metadata como `found_ratio` no cambia la clave;
- una actualización real de apariencia/geometría sí puede justificar otra
  ejecución. Prohibir para siempre repetir el mismo ID perdería información
  material recuperada por snapshot.

La cola contiene tickets pequeños, no copias de mapas.

### 7.2 No perder trabajo al alcanzar capacidad

Al saturarse la cola no se descarta raw ni un KF materialmente nuevo. El ledger
mantiene `needs_loop=true` y el scheduler va admitiendo tickets cuando baja la
profundidad. Se pueden descartar tickets stale y revisiones superseded, pero no
trabajo único vigente.

### 7.3 Recuperar backpressure de movimiento

El historial confirma que `/global_mapping/backpressure_active` funcionó con
histéresis. El `scenario_runner` actual todavía sabe:

1. dejar terminar el goal activo;
2. ejecutar waits normalmente;
3. retener el siguiente lote de goals;
4. enviarlo una vez cuando el flag vuelve a `false`.

Se recomienda recuperar esta puerta. No bloquea subscriptions ni bases: reduce
la generación futura de KFs esperando entre movimientos.

Esta recomendación no cancela un goal activo. La versión histórica que
cancelaba, enviaba una meta corta y reconstruía el destino dependía de control
simulado y tuvo más puntos de fallo. Si se quisiera detener inmediatamente un
dron a mitad de goal haría falta un contrato separado de `pause/hover/resume`
en el control de trayectoria. Para esta corrección se recomienda conservar la
puerta segura que el runner ya implementa entre lotes de movimiento.

El flag debe considerar:

```text
loop_queue_depth
oldest_loop_age
active_task_duration
estimated_drain_time (EWMA por tipo de tarea)
fiducial_queued_or_active
```

No basta con profundidad. Una tarea activa de 60 segundos debe activar presión
aunque no haya diez tickets pendientes.

Política inicial propuesta:

- activar si se supera el high watermark de profundidad, edad o tiempo de
  drenaje, o mientras exista una optimización fiducial ejecutable/en vuelo;
- liberar cuando todos los valores estén bajo sus low watermarks y no haya
  tarea fiducial pendiente/activa;
- exigir un pequeño dwell estable antes de liberar para evitar oscilaciones;
- ante worker muerto o presión excesivamente larga, fallar la prueba con un
  diagnóstico explícito, no mantener una espera infinita silenciosa.

Los valores exactos deben fijarse tras el benchmark corto. Los históricos
`10/3` prueban la mecánica, pero no son automáticamente los mejores límites
para el worker actual.

## 8. Solución del problema 4 - Autoridad y "desoptimización"

Los logs no demuestran una sobrescritura raw. En `prueba_75` hubo tres commits
de poses posteriores al primer apply fiducial. El problema real es que RViz2 no
permite saber qué commit sustituyó al anterior ni si el nuevo estado era mejor.

### 8.1 Registro de autoridad por pose

Cada pose debe conservar:

```text
pose_revision
source_kind = ANCHOR | DERIVED_TAIL | FIDUCIAL_OPT | LOOP_OPT | ROLLBACK
source_commit_id
source_task_id
parent_commit_id
base_raw_revision
accepted_at
```

`ReconcileAfterRawIngestResult` solo puede mover `DERIVED_TAIL`. Nunca puede
reemplazar `ANCHOR`, `FIDUCIAL_OPT` o `LOOP_OPT` aceptadas.

### 8.2 Política de sustitución

Una pose aceptada solo se sustituye por:

- un commit posterior explícitamente aceptado;
- un rollback explícito al estado padre;
- la eliminación/invalidación explícita definida por contrato futuro.

No existe un camino `raw -> overwrite server accepted`.

### 8.3 Validación de no regresión

Antes de aceptar otra optimización:

- todos los hard fiducials permanecen dentro de tolerancia;
- no empeoran por encima de tolerancia las observaciones absolutas ya
  aceptadas;
- el coste sobre el conjunto de validación común no regresa materialmente;
- se registran `max_delta_t/yaw`, KFs afectados y comparación before/after;
- una revisión publicable antigua no puede publicarse después de otra nueva.

No se propone que una pose optimizada sea eternamente inmóvil. Se propone que
cualquier sustitución sea explícita, validada, trazable y monotónica respecto
a las restricciones duras.

## 9. Solución del problema 5 - GT limitado a 50 sin perder fiduciales

### 9.1 Problema actual

En simulación GT llega a 50 Hz. El subscriber conserva 50 mensajes, pero el
executor monohilo puede estar ocupado muchos segundos. DDS descarta entonces
muestras antes de ejecutar `OnGroundTruthPose()`. Aumentar el vector interno a
5000 no recupera mensajes que nunca llegaron al callback.

Además, `vector.erase(begin())` desplaza el contenido del buffer lleno.

### 9.2 Cambio propuesto

- `MultiThreadedExecutor` con callback group GT separado del ingress;
- `GroundTruthBuffer` con `std::deque`/ring fijo de exactamente 50 muestras por
  dron;
- inserción O(1) con `pop_front()`/`push_back()`;
- mutex propio y breve, nunca `live_state_mutex_`;
- búsqueda entre como máximo 50 muestras;
- asociación justo después del commit raw de KFs nuevos;
- marcador explícito cuando no hay match, con timestamp del KF, rango del
  buffer, `nearest_dt` y motivo.

Con callbacks atendidos continuamente, 50 muestras representan cerca de un
segundo, coherente con el `fiducial_gt_max_dt_sec=1.0` vigente.

### 9.3 Snapshots con KFs antiguos

Un full snapshot que revele por primera vez un KF de hace muchos segundos no
debe justificar conservar miles de poses GT. Opciones:

1. recomendada para esta fase: solo asociarlo si su timestamp sigue dentro del
   ring; en caso contrario registrar `gt_history_expired` y no inventar anchor;
2. mejora futura: persistir en origen una observación fiducial ya asociada al
   KF, no un historial largo de poses GT.

La segunda opción es más robusta ante pérdida de deltas, pero ampliaría el
contrato wrapper/servidor y debe acordarse por separado.

## 10. Solución del problema 6 - Visualizador realmente live

### 10.1 El payload no es el cuello principal

El JSON actual ya es pequeño. Se puede sustituir `detail` libre por códigos y
añadir revisiones compactas, pero la mejora principal debe estar en la política
de colas y render.

### 10.2 Clases de eventos

```text
CRITICAL
  anchor begin/commit/publish
  fiducial optimization enqueue/start/stages/commit/rollback/end
  loop optimization start/stages/commit/rollback/end

IMPORTANT
  delta/snapshot raw commit agregados
  pose/fusion commit
  publicación RViz2

SAMPLEABLE
  loop enqueue
  candidatos/rechazos repetidos
  builds globalmap intermedios
  contadores de actividad
```

Reglas:

- `CRITICAL` tiene capacidad reservada y nunca es expulsado por loops;
- `IMPORTANT` puede coalescer por `(edge, revision)` conservando la última;
- `SAMPLEABLE` puede muestrearse o descartarse;
- descartar telemetría nunca descarta la tarea algorítmica correspondiente;
- una `LoopTask` real no se omite porque se omita alguno de sus pulsos web;
- el productor nunca espera al navegador.

Una garantía infinita es imposible con memoria finita si el consumidor está
desconectado indefinidamente. Para conservar el proceso completo sin bloquear,
el bridge debe mantener un estado reconstruible de tareas críticas activas y
las últimas N tareas críticas. Al reconectar envía ese snapshot de estado, no
un replay indiscriminado de actividad antigua.

### 10.3 Bridge SSE

- respetar `Last-Event-ID`;
- en primera conexión empezar en live/latest, salvo modo replay explícito;
- si el ID solicitado ya expiró, enviar un evento `state_reset` con el snapshot
  crítico actual;
- añadir timestamp de recepción del bridge y contadores de gaps/drops;
- no reenviar automáticamente los 512 eventos al conectar.

### 10.4 Frontend

- eliminar el consumo fijo de un evento cada 110 ms;
- procesar cada frame con `requestAnimationFrame`;
- renderizar todos los críticos FIFO disponibles;
- coalescer sampleables por arista/tarea y representar solo el estado más
  reciente;
- mantener una etapa secundaria persistente hasta la siguiente etapa o fin;
- usar pulsos breves para ingesta/publicación, sin acumular contadores durante
  520 ms;
- medir `backend -> bridge`, `bridge -> browser` y `browser -> render`;
- mostrar gaps/coalescing como diagnóstico, no como eventos antiguos animados.

### 10.5 Payload propuesto

```text
seq
emitted_wall_ns
sim_stamp_ns
priority
event_type
edge
phase
task_id
state_revision
count
```

Los tooltips siguen viviendo en `graph_definition.js`; no hace falta repetir
descripciones largas en cada evento.

## 11. Contrato de bases de datos

| Estado | Writer | Lectura de worker/publicador | Commit secundario |
|---|---|---|---|
| `RawMapDatabase` | coordinador | `LoopIndexUpdate`, `LoopGeometryInput`, `PublicationChange` | prohibido |
| `GlobalPoseStore` | coordinador | vista de poses/revisiones inmutable | `pose_patch` propuesto y validado |
| `CovisibilityDatabase` | coordinador | actualización/vista específica | patch propuesto |
| `FusedLandmarkManager` | coordinador | `fused_track_patch`/read model | patch propuesto |
| `LandmarkScoreManager` | coordinador | scores necesarios por ID | patch solo si la rama lo necesita |
| `LoopPairAttemptDatabase` | coordinador | resumen causal por par | patch propuesto |
| `GroundTruthBuffer` | callback GT | copia/búsqueda acotada de 50 | no aplica |

El coordinador puede aplicar varios patches como una transacción lógica. La
revisión nueva solo se emite cuando todos han terminado.

## 12. Reparto por subfases

Esta corrección no pertenece completa a `3K`.

| Subfase | Modificación propietaria |
|---|---|
| `3C` | commit raw corto, `RawInsertResult` y eventos incrementales |
| `3D` | revisión/autoridad de poses y patches de KFs nuevos |
| `3E` | buffer GT de 50, callback group y primer anchor |
| `3F` | `PublicationChange`, read model y publicación sin snapshot live |
| `3G` | diff/idempotencia de full snapshots |
| `3H` | creación prioritaria de tarea fiducial |
| `3K` | coordinador/worker, prioridades y commits propuestos |
| `3M` | patch/versiones de covisibilidad |
| `3N/3O` | índice incremental y entrada geométrica acotada |
| `3P/3Q` | resultados privados y patches de fusión/optimización |
| `3S/3T` | scores, revisiones, autoridad y trazabilidad |
| `3U` | publicación RViz2 y visualizador live |
| `3W` | presupuestos, backpressure, límites y estrés |

Los contratos de estas subfases deberán actualizarse durante la implementación
correspondiente. No se debe describir toda la reestructuración únicamente en
`subfase_3K_worker_secundario.md`.

## 13. Archivos probables

### Servidor

```text
orbslam3_server/src/global_map_server.cpp
orbslam3_server/src/mapping_state_coordinator.cpp        (nuevo, si se extrae)
orbslam3_server/include/.../mapping_state_coordinator.hpp
orbslam3_server/src/secondary_task_worker.cpp             (nuevo, si se extrae)
orbslam3_server/include/.../secondary_task_worker.hpp
orbslam3_server/launch/global_orb_map_server.launch.py
orbslam3_server/CMakeLists.txt
```

Solo se justifican esas extracciones si reducen de verdad la responsabilidad de
`global_map_server.cpp`. No se propone crear un manager por cada contador.

### Backend

```text
orbslam3_multi/include/orbslam3_multi/raw_map_database.hpp
orbslam3_multi/src/raw_map_database.cpp
orbslam3_multi/include/orbslam3_multi/global_pose_store.hpp
orbslam3_multi/src/global_pose_store.cpp
orbslam3_multi/include/orbslam3_multi/global_map_builder.hpp
orbslam3_multi/src/global_map_builder.cpp
orbslam3_multi/include/orbslam3_multi/*_database.hpp
orbslam3_multi/src/*_database.cpp
```

### Simulación y visualizador

```text
simulacion_dron/src/control_tray/scenario_runner_node.cpp
simulacion_dron/src/visualizer/pipeline_flow_bridge.py
simulacion_dron/web/pipeline_flow/app.js
simulacion_dron/web/pipeline_flow/graph_definition.js
simulacion_dron/launch/multi_dron.launch.py
```

No debería ser necesario modificar `ORB_SLAM3`, `orbslam3_ros2` ni
`orbslam3_msgs` para la primera implementación de esta arquitectura.

## 14. Plan de implementación recomendado

### Bloque A - Instrumentación y tests de concurrencia

1. separar `lock_wait` y `lock_hold` de costes de copia/cálculo;
2. añadir timestamps de enqueue/start/end y revisiones por tarea;
3. registrar fallos de asociación GT;
4. crear tests con una tarea secundaria artificialmente lenta.

### Bloque B - GT, executor y gating

1. `MultiThreadedExecutor` con callback groups;
2. ring GT de 50;
3. eliminar buffer 5000 y `vector.erase(begin())`;
4. estado pre-anchor y ledger diferido;
5. publicación de anchor antes de liberar backfill.

Este bloque corrige dos regresiones visibles sin cambiar algoritmos de loop.

### Bloque C - Coordinador y commits

1. introducir eventos de ingesta y `DerivedCommitProposal`;
2. convertir las bases live en propiedad exclusiva del coordinador;
3. retirar escrituras directas del worker;
4. validar revisiones por entidad;
5. retirar progresivamente `live_state_mutex_`.

### Bloque D - Read models incrementales

1. `LoopIndexUpdate` incremental;
2. `LoopGeometryInput` limitado a query/candidatos;
3. `PublicationChange` y cache del publicador;
4. eliminar `CreateStateSnapshot()` de loops/publicación runtime.

### Bloque E - Backpressure y visualizador

1. ledger idempotente completo y métricas de edad/drenaje;
2. reactivar la puerta entre movimientos con histéresis;
3. prioridades de telemetría;
4. SSE live/reset y frontend sin replay amortiguado.

### Bloque F - Autoridad de poses y limpieza

1. linaje de commits y política de no regresión;
2. eliminar rutas legacy que puedan sustituir autoridad;
3. simplificar clases/miembros obsoletos del servidor;
4. actualizar documentación por subfase y paquete.

Cada bloque debe compilar y pasar tests antes de continuar. La simulación larga
solo debe ejecutarse después de los tests cortos de concurrencia/gating.

## 15. Pruebas propuestas para la futura implementación

### 15.1 Tests deterministas

1. **SecondaryDoesNotBlockIngress**: mantener una `LoopTask` lenta y comprobar
   commits raw/poses/publicaciones durante su ejecución.
2. **GtRingRemainsLive**: bloquear el flujo de ingesta artificialmente y
   comprobar que GT sigue entrando, el ring nunca supera 50 y la inserción es
   O(1).
3. **NoLoopBeforeAnchor**: muchos KFs raw pre-anchor producen cero tickets y
   cero inicios.
4. **AnchorPublishBeforeBackfill**: la revisión de anchor se publica antes del
   primer loop diferido.
5. **LoopWorkIdempotency**: delta + snapshot idénticos ejecutan una sola clave;
   un cambio material posterior crea exactamente una clave nueva.
6. **StablePriority**: tarea activa -> fiducial FIFO -> loops FIFO.
7. **AtomicDerivedCommit**: ningún snapshot publicable combina poses nuevas con
   fusión vieja a mitad del lote.
8. **PoseAuthority**: raw/snapshot no sobrescribe una pose aceptada; un cambio
   posterior siempre cita `commit_id` o rollback.
9. **BackpressureHysteresis**: activa high, retiene el siguiente goal, libera
   low una sola vez y nunca pausa callbacks.
10. **FlowLiveStress**: ráfaga de eventos sampleables más un lifecycle crítico;
    el lifecycle llega completo, sampleables se coalescen y no hay replay al
    reconectar.

### 15.2 Simulación corta

Usar primero `prueba_tipica_fiducial_2_a_1_dos_lados.yaml` para verificar:

- anchors y GT;
- publicación antes de backfill;
- tarea fiducial prioritaria;
- autoridad de poses;
- visualizador crítico en tiempo real.

### 15.3 Simulación larga

Después usar
`prueba_tipica_rodeo_edificio_dos_fiduciales.yaml`. Debido al no determinismo
observado, conviene ejecutar al menos dos repeticiones completas, conservando
una conclusión independiente para cada una.

## 16. Presupuestos iniciales de aceptación

Son objetivos de ingeniería revisables tras medir el bloque A, no thresholds
algorítmicos:

```text
loops iniciados antes de anchor                         = 0
muestras GT retenidas por dron                         <= 50
callbacks GT con huecos causados por callback de delta = 0
tareas exactas duplicadas                              = 0
active_secondary_workers                               <= 1
commits raw perdidos                                   = 0
overwrites raw de pose aceptada                        = 0
publicaciones con revisión regresiva                   = 0
eventos críticos perdidos con bridge conectado         = 0
```

Objetivos temporales iniciales:

- lock de cola/GT p99 inferior a 2 ms;
- commit derivado p99 inferior a 20 ms y sin cálculo geométrico dentro;
- ausencia de huecos de publicación de decenas de segundos por sincronización;
- anchor/optimización visible en web con p95 inferior a 250 ms y máximo
  inferior a 1 s cuando el bridge está conectado;
- backlog finalmente drenable y con edad máxima registrada;
- toda activación de backpressure termina en liberación o fallo explícito.

La latencia exacta delta -> RViz2 depende también del coste de reconstruir el
mensaje completo. Tras eliminar el lock global debe medirse antes de fijar un
presupuesto final, pero no se aceptarán nuevamente picos de 20-28 segundos.

## 17. Riesgos y decisiones que deben confirmarse

### Recomendaciones que considero firmes

- único writer mediante `MappingStateCoordinator`;
- worker secundario único y no interrumpible;
- cero loops pre-anchor;
- ring GT de 50 con callback independiente;
- read models incrementales en vez de copias globales;
- autoridad y linaje explícitos de poses;
- frontend live con eventos por prioridad.

### Decisiones funcionales pendientes de confirmación

1. **Backpressure**: recomiendo reactivar el Bool global únicamente como puerta
   entre movimientos, usando el comportamiento que el runner ya conserva.
2. **Orden anchor/backfill**: recomiendo liberar loops tras la publicación ROS
   del anchor, sin esperar ACK de RViz2.
3. **KF actualizado**: recomiendo impedir duplicados exactos, pero permitir una
   nueva tarea cuando cambie de verdad apariencia/geometría.
4. **Snapshots antiguos sin GT**: recomiendo no conservar más de 50 poses y
   registrar `gt_history_expired`; persistir la observación en origen quedaría
   como mejora aparte.
5. **Garantía visual**: recomiendo garantía completa para anchors y
   optimizaciones mientras el bridge está conectado, más reconstrucción de
   estado al reconectar; nunca bloquear ROS para garantizar telemetría.

## 18. Recomendación final

No conviene reparar esta regresión añadiendo otro mutex, ampliando colas o
reduciendo eventos de forma indiscriminada. La reestructuración debe cambiar la
propiedad del estado:

```text
principal = único escritor y emisor de cambios coherentes
secundario = cálculo privado + propuesta de commit
publicador = read model incremental y coalescido
visualizador = consumidor descartable con prioridad semántica
```

La alternativa A puede servir para estabilizar el sistema durante la
migración, pero el objetivo de cierre debe ser la alternativa B. Es la única de
las propuestas que permite demostrar que una tarea secundaria lenta consume
CPU, pero no mantiene un recurso que impida al flujo principal recibir,
guardar, anclar y publicar.
