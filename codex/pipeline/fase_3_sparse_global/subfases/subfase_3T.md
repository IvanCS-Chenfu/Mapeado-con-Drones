# Subfase 3T - Arquitectura runtime, ownership e invariantes

## Estado vigente

```text
CONSEGUIDA POR AUDITORIA TECNICA Y ACEPTACION DEL USUARIO
```

La arquitectura exigida quedo implantada incrementalmente entre `3C` y `3S`.
La auditoria final confirma dos workers persistentes, autoridades separadas,
propuestas privadas, commits revisionados, dirty sets y publicacion exclusiva
del flujo principal. El usuario considera bueno el rendimiento actual y decide
no estrechar mas los locks por ahora.

No se ejecuta una reimplementacion adicional en `3T`. Las mediciones globales
de carga y la regresion transversal permanecen en `3V/3W` y no reabren esta
subfase salvo que descubran una violacion real de ownership o atomicidad.

### Trabajo que se conserva

- invariantes de identidad, frames, raw inmutable y GT restringido;
- separación entre fuentes de verdad y telemetría descartable;
- necesidad de commits coherentes y revisiones monotónicas.

### Implementación anterior incorrecta que no debe repetirse

- representar el servidor como propietario directo de todas las mutaciones;
- utilizar un snapshot/publication request global como frontera universal;
- tener varios writers sin contrato explícito de propuesta/commit;
- usar un mutex recursivo común para simular atomicidad;
- dejar IDs, revisiones y origen de poses/tracks implícitos.

### Contrato vigente validado

`GlobalMapServer` orquesta exactamente `PrimaryWorker` y `SecondaryWorker`.
Cada base define writer, vista inmutable, patch y revisión. Los secundarios
preparan propuestas y el commit valida por entidad; los cambios a publicación
se acumulan como dirty sets. Todo evento incluirá `flow_id`, `task_id`, origen,
revisiones. La telemetria web detallada es observabilidad descartable y su
presentacion pertenece a `3U`; no condiciona commits ni publicacion. Las
pruebas existentes y las simulaciones hasta 194 cubren un solo worker por
flujo, raw inmutable, commits atomicos y progreso del principal.

## Estado histórico anterior

Las secciones posteriores conservan el contrato que fue implantado
incrementalmente. Si una formulacion histórica contradice el estado de cierre
de esta cabecera, prevalece el runtime auditado.

```text
CONSEGUIDA: ownership y flujo vigentes auditados; 3V/3W los reutilizan en la
regresion y el stress globales.
```

## Objetivo

Convertir las decisiones transversales de `3C-3S` en invariantes verificables,
sin trasladar a `3T` las responsabilidades algoritmicas de cada subfase.

## Arquitectura vigente

```text
wrappers
   |
   | delta / full snapshot
   v
Server -----------------------------------------------+
   |                                                  |
   | commit + ChangeSet                               | observacion fiducial
   v                                                  v
RawMapDatabase                                 FiducialAnchorManager
   |                                                  |
   +--> GlobalPoseStore <-----------------------------+
   +--> CovisibilityDatabase
   +--> LandmarkScoreManager
   +--> PublicationRequest -----------------------+
   +--> enqueue LoopTask                          |
                                                 v
                                     GlobalMapBuilder -> ROS/RViz2

PriorityQueue -> SecondaryWorker
   |                |
   |                +--> FiducialOptimizationTask
   |                +--> LoopTask (3N -> 3O -> 3P/3Q)
   |                           |
   +---------------------------+--> commits derivados
                                      |
                                      +--> nueva PublicationRequest

FlowTracer -> telemetria descartable -> visualizador JavaScript
```

## Ownership

| Componente | Posee | Puede escribir | No puede hacer |
|---|---|---|---|
| `Server` | orquestacion, colas, timers, revisiones globales | coordinacion/estado de ejecucion | duplicar bases o ejecutar algoritmos largos en callbacks |
| `RawMapDatabase` | estado ORB crudo y journal | solo datos recibidos/reconciliados de ORB | anchors, poses world, fusion, optimizacion, ROS |
| `GlobalPoseStore` | anchors, poses world, propagacion y autoridad futura | commits de anchor/pose | descriptores, MPs, BoW, publishers |
| `FiducialAnchorManager` | semantica/visitas fiduciales | su estado ligero de visitas | pose DB duplicada, loops, solver |
| `CovisibilityDatabase` | aristas KF-KF confirmadas | import ORB y commits geometricos | candidatos/rechazos BoW |
| `LoopDetector` | indice/consulta BoW | su indice/caches propios | scheduling, RANSAC, bases globales |
| `SubcloudLoopVerifier` | calculo geometrico privado | ninguno live | cola, fusion, optimizacion |
| `LoopDecisionManager` | decision estructurada | memoria canonica acotada | crear workers o escribir bases por pasos |
| `FusedLandmarkManager` | equivalencias/tracks | commit de fusion | modificar raw/poses, publicar ROS |
| `LandmarkScoreManager` | scores raw/fused | commits de score | geometria, scheduling, publicacion |
| `PoseGraphBuilder` | grafo temporal privado | ninguno live | solver, commit, thread propio |
| `OptimizationManager` | solver/candidato privado | ninguno live | scheduler, publicacion, raw |
| `GlobalMapBuilder` | caches derivadas publicables, slots, indices inversos y dirty sets | ninguna base autoritativa | anchors, politica de score, fusion o poses |
| `FlowTracer` | cola de eventos descartables | solo telemetria | bloquear o decidir funcionalidad |

Los nombres existentes `GlobalPoseStore` y `FiducialAnchorManager` se conservan;
no se renombran a `KFPoseDatabase`/`FiducialManager` si eso solo genera churn.

## Flujo principal

El flujo principal reutiliza el unico `PrimaryWorker`; no existe un worker
propio de publicacion. Debe:

1. recibir y validar deltas/snapshots;
2. hacer commit raw y obtener `ChangeSet`;
3. registrar KFs nuevos en poses si existe anchor;
4. procesar el primer anchor o una observacion fiducial;
5. importar covisibilidad ORB y score base afectados;
6. drenar dirty sets y actualizar `GlobalMapBuilder` por IDs;
7. serializar/publicar cloud y KFs coherentes si cambia la vista;
8. encolar trabajo secundario cuando corresponda;
9. terminar la `PrimaryTask` y volver a atender ROS.

Nunca espera BoW, RANSAC, fusion, grafo, solver, HTML, RViz2 o navegador. Si el
worker secundario se detiene, este flujo sigue siendo util.

## Flujo secundario

- un worker persistente;
- una tarea activa como maximo;
- prioridad `FIDUCIAL > LOOP`;
- no preemption: la activa termina;
- FIFO dentro de cada prioridad;
- `LoopTask` unica desde BoW hasta fusion/optimizacion y commit;
- calculo sobre snapshots;
- tarea finalizada al comprometer/rechazar bases;
- publicacion solicitada despues, sin ACK.

No hay tareas independientes para RANSAC, fusion, optimizacion por loop,
scoring o reprocesado post-optimizacion.

## Sincronizacion

### Regla de locks

Los mutex protegen exclusivamente:

- push/pop/coalescing de cola;
- captura corta o intercambio de version;
- validacion y commit de lote.

Nunca cubren algoritmos, I/O o construccion de mensajes. Los lectores largos
usan snapshots inmutables. Cuando una base prepara copy-on-write, la version
anterior permanece disponible hasta el intercambio atomico.

### Revisiones

Separar como minimo:

```text
raw_revision
appearance_revision
association_revision
geometry_revision
pose_revision
covisibility_revision
fusion_revision
score_revision
derived_state_revision
publication_revision
```

No incrementar revisiones geometricas por metadata. Cada tarea declara sus
dependencias y valida solo esas revisiones.

### Commit multi-base

Una rama que modifica varias bases prepara todas las versiones fuera del lock.
El servidor publica una nueva `DerivedStateRevision` solo cuando los lotes son
compatibles. `GlobalMapBuilder` nunca combina estados parciales.

## Identidades y frames

```text
SubmapId       = (drone_id, map_epoch)
KeyFrameId     = (drone_id, map_epoch, local_kf_id)
RawMapPointId  = (drone_id, map_epoch, local_mp_id)
FusedTrackId   = ID estable del servidor
TaskId         = monotono, con tipo y clave canonica separados
ArrivalId      = orden de entrada, no identidad geometrica
```

Convenciones obligatorias:

- nombres `target_T_source`;
- pose world de KF autoritativa en `GlobalPoseStore`;
- pose local raw autoritativa en `RawMapDatabase`;
- fiducial como observacion absoluta;
- loop como relacion geometrica relativa;
- GT solo fiducial simulado/debug/metrica.

## Publicacion

`GlobalMapBuilder` drena IDs dirty y consulta por entidad las autoridades
raw/pose/fusion/score. Actualiza sus caches stateful, produce KFs y MPs de una
revision conjunta y deja que el servidor serialice. No existe timer de
reconciliacion pesado ni captura global. Los commits secundarios solo acumulan
IDs/revisiones; la siguiente `PrimaryTask` construye/publica y ninguna tarea
secundaria condiciona su scheduler a RViz2.

## Backpressure

El unico topic puede gobernar el siguiente movimiento del runner, pero nunca
pausa subscriptions, commits principales o publicacion. Fiduciales mantienen el
gate hasta commit/rechazo. Loops usan histeresis de cola. No se espera ACK
visual.

## Simplificacion obligatoria

Antes de añadir clases, auditar las actuales y eliminar solo tras tests:

- scheduling anterior ya sustituido por el worker unico de `3K`;
- scheduler separado de loop optimization;
- colas independientes de fusion/scoring;
- `PostOptimizationKeyFrameQueue`;
- estados `AWAITING_VISUAL_ACK`;
- caches/vistas duplicadas solo para RViz2;
- `FusionManager` si solo reenvia a `FusedLandmarkManager`;
- callbacks/timers que vuelven a escanear bases para suplir `ChangeSet`;
- parametros y logs sin consumidor.

La evidencia historica se conserva en historial; no se conserva codigo muerto
solo para reproducir arquitectura antigua.

## Visualizador

La observabilidad `3U` usa eventos best-effort. El tracer hace `try_push` en una
cola acotada; si falla, incrementa `dropped_events` y retorna. La topologia,
descripciones y estilos viven en JavaScript, fuera del backend algoritmico.

## Pruebas de contrato

1. `RawMapDatabase` no cambia tras fusion/optimizacion.
2. No hay poses globales fuera de `GlobalPoseStore` como autoridad.
3. No hay score escrito fuera de `LandmarkScoreManager`.
4. No hay dos tareas secundarias activas.
5. Orden: activa completa, fiduciales FIFO, loops FIFO.
6. KFs nuevos se publican durante una tarea lenta.
7. Commit multi-base no produce vista parcial.
8. No existe espera funcional de RViz2/JS.
9. Desconectar/saturar telemetria no cambia resultados.
10. No aparece `PostOptimizationKeyFrameQueue` ni scheduling duplicado.

## Criterio de exito

Los invariantes anteriores estan expresados en APIs/tests y la simulacion larga
los cumple. Cualquier excepcion de ownership debe documentarse y justificarse;
si dos clases poseen el mismo estado, `3T` no puede cerrarse.

## Archivos probables

Los contratos afectan a headers/componentes ya listados en `3C-3S`, al
orquestador `global_map_server.cpp` y a tests de ownership/concurrencia. No se
crea un `system_invariants` complejo si tests pequeños y APIs de tipos expresan
mejor las reglas.

## Exclusiones

No implementar hard real-time. No modificar wrappers, `ORB_SLAM3` ni mensajes
sin bloqueo demostrado y autorizacion.

## Incremento Visual Obligatorio

Se aplico `../CONTRATO_VISUAL_INCREMENTAL.md` sin añadir nodos documentales. El
grafo representa IDs, ownership y fronteras de autoridad suficientes para
seguir el runtime real; la telemetria sigue siendo best-effort y no funcional.
Una regresion futura que confunda autoridades o altere el pipeline reabriria
3T, pero no se exigen mas campos visuales para su cierre actual.
