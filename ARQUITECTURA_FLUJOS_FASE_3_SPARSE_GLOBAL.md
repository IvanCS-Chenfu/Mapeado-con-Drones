# Arquitectura acordada de flujos - Fase 3 Sparse Global

## 0. Estado del documento

Este documento define la arquitectura funcional que se quiere implementar tras
la investigación de los problemas observados en `prueba_74`, `prueba_75` y
`prueba_76`.

```text
Acuerdo arquitectónico: CERRADO
Preparación funcional para implementar: PENDIENTE
Autorización funcional: PENDIENTE
Código modificado por este documento: ninguno
Build o simulación ejecutados: ninguno
```

El diagnóstico y la evidencia permanecen en:

```text
INVESTIGACION_PROBLEMAS_FASE_3_SPARSE_GLOBAL.md
```

Este archivo no es historial ni autorización de implementación. Es el contrato
arquitectónico que después deberá distribuirse entre los MDs de las subfases
propietarias.

### Índice de lectura

- `1-2`: responsabilidades, dos colas y dos workers.
- `3-6`: flujo principal, fiduciales, prioridades y deduplicación.
- `7-8`: `GlobalMapBuilder` incremental y sincronización.
- `9-10`: backpressure y visualizador por flujos completos.
- `11-16`: modificaciones, subfases, pruebas, criterios y pipeline final.

## 1. Principio general

`global_map_server.cpp` es el orquestador ROS del mapa global. No implementa
algoritmos ni mantiene lógica duplicada de las clases de `orbslam3_multi`.

Sus responsabilidades son exclusivamente:

- recibir deltas, snapshots, GT y resultados internos;
- introducir trabajo en las colas correctas;
- respetar orden, prioridad e histéresis;
- llamar a las clases de la librería;
- publicar los mensajes producidos por `GlobalMapBuilder`;
- publicar el flag de backpressure;
- emitir telemetría ligera para el grafo web.

Las clases de la librería realizan el trabajo real:

```text
RawMapDatabase
GlobalPoseStore
FiducialAnchorManager
CovisibilityDatabase
LandmarkScoreManager
LoopPairAttemptDatabase
LoopDetector
SubcloudLoopVerifier
LoopDecisionManager
PoseGraphBuilder
OptimizationManager
FusedLandmarkManager
GlobalMapBuilder
```

No se crea otro gran coordinador paralelo a `GlobalMapServer`. Tampoco se crea
una clase por cada transición o contador.

## 2. Dos colas y dos flujos

Existen exactamente dos colas lógicas de ejecución:

```text
primary_queue
secondary_queue
```

Cada una tiene un único consumidor persistente:

```text
PrimaryWorker
SecondaryWorker
```

Consecuencias:

- solo hay un flujo principal activo;
- solo hay una tarea secundaria activa;
- una tarea activa nunca se interrumpe;
- el flujo principal y el secundario sí pueden ejecutarse simultáneamente;
- ambos comparten CPU, pero no mantienen un mutex global común;
- las colas son acotadas, medibles y participan en backpressure.

## 3. Cola y flujo principal

### 3.1 Entradas

La cola principal recibe datos completos ya recibidos por ROS:

```text
DeltaInput
SnapshotInput
```

El callback ROS valida lo mínimo, conserva el mensaje mediante ownership seguro,
lo encola y retorna. No ejecuta el pipeline dentro del callback.

### 3.2 Serialización completa

El `PrimaryWorker` no empieza otra entrada hasta que la activa haya terminado
todo el recorrido hasta la publicación ROS.

```text
dequeue delta/snapshot
-> commit raw
-> poses principales
-> fiduciales principales
-> notificaciones y tareas secundarias
-> GlobalMapBuilder incremental
-> servidor publica nube y KFs
-> fin de la entrada principal
-> siguiente entrada
```

El final de una tarea principal es la llamada de publicación ROS completada. No
se espera un ACK de RViz2 ni se comprueba cuándo termina de dibujarlo.

### 3.3 Procesamiento de un delta

Orden obligatorio:

1. `RawMapDatabase::InsertDelta()` conserva el estado ORB-SLAM3 raw.
2. Se obtiene `RawInsertResult`/`ChangeSet` con los IDs y revisiones realmente
   nuevos o modificados.
3. Para cada KF de submapa ya anclado, `GlobalPoseStore` registra o reconcilia
   su pose world.
4. Se comprueba si los KFs recibidos producen observaciones fiduciales.
5. Un primer fiducial ancla el submapa dentro del flujo principal.
6. Una revisit fiducial crea una tarea secundaria de prioridad máxima.
7. Se encola la actualización secundaria de covisibilidad/score.
8. Si el submapa está anclado y hay KFs materialmente elegibles, se encola su
   trabajo de loop con prioridad normal.
9. Se notifican a `GlobalMapBuilder` los cambios raw/pose del propio flujo.
10. `GlobalMapBuilder` incorpora cambios principales y secundarios pendientes.
11. El servidor publica la nube sparse y el mensaje de KFs devueltos.

Los pasos secundarios 7 y 8 se encolan, pero el flujo principal no espera a que
terminen.

### 3.4 Procesamiento de un snapshot

El snapshot sigue la misma ruta principal, con estas diferencias:

- usa `RawMapDatabase::InsertFullSnapshot()`;
- trabaja sobre el diff material devuelto por la base;
- un snapshot idéntico no crea trabajo redundante;
- encola actualización de covisibilidad/score;
- no crea por sí mismo nuevas búsquedas de loop;
- puede recuperar KFs raw ausentes en deltas y hacerlos publicables si ya existe
  anchor;
- nunca sobrescribe poses globales aceptadas con poses raw.

## 4. Fiduciales

### 4.1 Primer fiducial

El primer fiducial válido de `(drone_id, map_epoch)` pertenece al flujo
principal:

```text
asociar KF y observación absoluta
-> FiducialAnchorManager valida
-> GlobalPoseStore confirma que no existe anchor
-> calcular anchor
-> asignar poses world a todos los KFs raw del submapa
-> marcar KF hard fiducial
-> notificar KFs/MapPoints dirty a GlobalMapBuilder
-> publicar en la misma tarea principal
```

Los KFs pre-anchor no generan tareas de loop. Se conservan como elegibles
pendientes. Tras el anchor se liberan ordenadamente a la cola secundaria.

### 4.2 Revisit fiducial

Una observación posterior crea una tarea secundaria prioritaria:

```text
FiducialCheckOptimizationTask
```

La tarea:

1. lee la pose vigente en `GlobalPoseStore` y la observación GT permitida;
2. calcula el error fiducial;
3. si está bajo umbral, termina sin modificar poses;
4. si está sobre umbral, ejecuta grafo, solver, validación y commit completos;
5. escribe las poses aceptadas en `GlobalPoseStore`;
6. notifica a `GlobalMapBuilder` todos los KFs afectados;
7. termina después del commit, sin publicar ni esperar RViz2.

La tarea activa no se interrumpe. La prioridad se aplica al seleccionar la
siguiente tarea secundaria.

## 5. Cola secundaria y prioridades

Existe un único `SecondaryWorker` con tres niveles estables:

```text
MAX     FiducialCheckOptimizationTask
HIGH    DatabaseUpdateTask
NORMAL  LoopTask
```

FIFO se conserva dentro de cada prioridad.

Orden de selección:

```text
tarea activa termina
-> fiducial pendiente más antiguo
-> actualización de bases pendiente más antigua
-> loop pendiente más antiguo
```

### 5.1 `DatabaseUpdateTask`

Este es el trabajo que anteriormente se denominó de forma ambigua
`DerivedUpdate`.

No es una tercera cola ni un tercer flujo. Es una tarea de prioridad alta de la
misma cola secundaria.

Recibe el `ChangeSet` de un delta o snapshot y actualiza:

- covisibilidad raw importada;
- scores base/materiales;
- índices o revisiones baratos necesarios para evitar trabajo repetido;
- cualquier estado derivado ligero que dependa directamente de la llegada.

Al terminar:

- compromete solamente las bases derivadas correspondientes;
- notifica a `GlobalMapBuilder` los scores/relaciones que cambian la vista;
- no publica;
- no crea otra cola;
- no modifica `RawMapDatabase` ni las poses de `GlobalPoseStore`.

Su prioridad es superior a los loops porque estos deben consultar la
covisibilidad, scores y memoria de intentos más recientes. Así se evitan BoW,
matching o RANSAC que ya puedan descartarse mediante conocimiento barato.

### 5.2 `LoopTask`

Existe una `LoopTask` independiente por cada KF nuevo o materialmente
modificado que resulte elegible. No se crea una única tarea de loop para todo
el delta.

Un delta con N KFs elegibles puede crear entre cero y N tareas, después de
aplicar gating, revisiones, coalescing y deduplicación. Cada tarea contiene todo
el proceso para su KF query:

```text
BoW
-> filtros de identidad, causalidad, covisibilidad y pares ya probados
-> matching/RANSAC si sigue siendo necesario
-> decisión
-> fusión u optimización
-> commit de las bases afectadas
-> notificación de cambios a GlobalMapBuilder
-> fin
```

No se crean tareas separadas para decisión, fusión u optimización. Son ramas de
la misma `LoopTask` y tienen la misma prioridad de admisión.

Si la rama elegida es fusión, puede modificar mediante commits breves:

```text
CovisibilityDatabase
LandmarkScoreManager
FusedLandmarkManager
LoopPairAttemptDatabase
```

No modifica:

```text
RawMapDatabase
GlobalPoseStore
```

Si la rama elegida es optimización, también puede comprometer poses en
`GlobalPoseStore` tras validación. `RawMapDatabase` permanece inmutable para el
secundario.

## 6. Admisión y deduplicación de loops

No se crea una `LoopTask` si el submapa query no está anclado. La unidad de
admisión, deduplicación, cola y telemetría es siempre un KF, no un delta.

La identidad de trabajo incluye como mínimo:

```text
RawKeyFrameId
appearance_revision
geometry_revision relevante
```

Reglas:

- delta y snapshot con la misma revisión no repiten trabajo;
- dos solicitudes pendientes equivalentes se coalescen;
- si llega una revisión nueva mientras la tarea está pendiente, se conserva la
  última;
- si cambia mientras está activa, se marca una única revisión posterior;
- metadata irrelevante no crea loops;
- un cambio material real sí puede crear otra tarea;
- `LoopPairAttemptDatabase` evita repetir pares ya resueltos para las mismas
  revisiones;
- las actualizaciones HIGH se ejecutan antes de loops NORMAL pendientes.

## 7. `GlobalMapBuilder` incremental y con estado

### 7.1 Cambio de responsabilidad

La implementación actual de `GlobalMapBuilder::Build()` recorre todos los
submapas y MapPoints y crea una salida nueva en cada llamada. El diseño objetivo
lo convierte en una clase stateful que conserva la vista publicable anterior.

`GlobalMapBuilder` será propietario de:

```text
última nube sparse completa
último mensaje completo de KFs
cache de KFs publicables por RawKeyFrameId
cache de puntos raw/fused publicables por identidad estable
índice identidad -> posición dentro del mensaje/cache
índice KF usado para publicar -> puntos dependientes
índice fused track -> miembros/representante publicados
pending_raw_changes
pending_pose_changes
pending_score_changes
pending_fusion_changes
```

### 7.2 Comportamiento normal

Primera construcción:

```text
crear nube y KFs completos desde las bases
-> guardar cache y mensajes completos
-> devolverlos al servidor
```

Llegadas posteriores:

```text
recibir IDs nuevos/modificados
-> consultar solo esos elementos y sus dependencias
-> añadir, reemplazar o retirar slots afectados
-> conservar intacto el resto del mensaje anterior
-> devolver la nueva versión completa al servidor
```

El servidor sigue publicando un `PointCloud2` y un `MarkerArray` completos. Lo
incremental es su mantenimiento interno: no se repiten las transformaciones,
selección de observadores, score y fusión de todos los puntos.

### 7.3 Nuevos KFs y MapPoints

Para datos nuevos:

- se añade el KF al cache y al mensaje de KFs si tiene pose world;
- se calculan únicamente sus MapPoints nuevos o afectados;
- se añaden al final o a slots libres de la nube cacheada;
- se actualizan índices de identidad y dependencias;
- los datos anteriores no se recalculan.

Cuando se ancla por primera vez un submapa, todos sus KFs/MapPoints acumulados
pasan a ser un lote de altas incrementales.

### 7.4 Optimización de poses

Una optimización produce un `PoseChangeSet` con los KFs modificados.

`GlobalMapBuilder`:

1. actualiza las poses de esos KFs en el mensaje cacheado;
2. usa el índice inverso para localizar los puntos proyectados desde ellos;
3. vuelve a consultar raw y `GlobalPoseStore` solo para esos puntos/KFs;
4. reemplaza sus coordenadas en la nube cacheada;
5. actualiza fused tracks que dependan de alguno de esos puntos;
6. conserva intactos todos los demás slots.

Si cambia el KF observador elegido para publicar un punto, también se corrige el
índice inverso.

### 7.5 Scores

Un `ScoreChangeSet` identifica MapPoints o tracks cuyo score cambió.

Para cada identidad afectada:

- actualizar el campo score;
- retirarla si baja del umbral publicable;
- insertarla si pasa a cumplir el umbral;
- recalcular el representante fused si su ponderación depende del score.

No se recorren todos los scores de la base.

### 7.6 Fusiones

Un `FusionChangeSet` identifica tracks y miembros modificados.

`GlobalMapBuilder`:

- retira de la nube los miembros raw que pasan a estar fusionados;
- añade o actualiza el representante del track;
- restaura un miembro raw si deja de pertenecer al track por rollback;
- actualiza posición, score y observaciones del representante;
- corrige todos los índices de slots y dependencias afectados.

La estructura interna puede usar slots estables más un índice por identidad.
Una retirada puede reutilizar el último slot o dejar un slot libre, actualizando
el índice correspondiente. No se buscarán puntos recorriendo linealmente toda
la nube.

### 7.7 Mensaje de KFs

Los KFs usan identidades/marker IDs estables.

- un KF nuevo añade sus marcadores;
- una pose optimizada reemplaza los marcadores con los mismos IDs;
- una retirada explícita emite/retiene las acciones `DELETE` necesarias para
  que RViz2 no conserve marcadores antiguos;
- labels y frustums no se reconstruyen para KFs sin cambios.

### 7.8 Acumulación de cambios secundarios

Los secundarios no llaman a `Build()` ni publican. Solo notifican cambios:

```text
NotifyPoseChanges(...)
NotifyScoreChanges(...)
NotifyFusionChanges(...)
NotifyRawDerivedChanges(...)
```

Las notificaciones:

- contienen IDs, revisiones y origen, no mapas ni nubes completos;
- se fusionan en sets/mapas dirty;
- deduplican una misma identidad;
- conservan la revisión más nueva;
- no ejecutan transformaciones ni serialización.

Si terminan varias fusiones antes de la siguiente llamada principal, todos sus
cambios permanecen acumulados. La siguiente ejecución de `GlobalMapBuilder`
aplica la unión completa.

### 7.9 Cuándo se ejecuta

`GlobalMapBuilder` se activa exclusivamente como etapa de un flujo principal
provocado por un delta o snapshot.

Un commit secundario:

- modifica sus bases;
- registra los dirty sets;
- no despierta al flujo principal;
- no crea `RebuildAfterSecondaryCommit`;
- no publica.

La siguiente tarea principal drena todos los cambios acumulados y publica la
vista actualizada. Si no llega otra entrada principal, los cambios secundarios
permanecen pendientes y la vista de RViz2 no cambia todavía. Esta consecuencia
es deliberada en el diseño acordado.

## 8. Concurrencia y mutex

Se elimina el uso de un único `live_state_mutex_`.

Mutex permitidos:

```text
primary_queue_mutex
secondary_queue_mutex
ground_truth_ring_mutex
pending_global_map_changes_mutex
mutex interno breve por commit de base derivada
```

Reglas:

- `PrimaryWorker` es el único writer principal de raw y poses normales;
- una tarea secundaria nunca mantiene un lock durante BoW, matching, RANSAC,
  grafo, solver, fusión calculada o logs;
- la nueva versión de una base se prepara fuera del lock;
- el lock de commit solo valida revisiones y aplica/intercambia el resultado;
- `Notify*Changes()` solo inserta IDs/revisiones en dirty sets;
- `GlobalMapBuilder` intercambia los dirty sets bajo lock y trabaja con una
  copia privada;
- notificaciones que llegan durante una actualización quedan pendientes para
  la siguiente llamada;
- las lecturas largas usan vistas inmutables o copias específicas, nunca
  referencias live mutables.

`RawMapDatabase` nunca es modificada por fusión u optimización. Las poses de
`GlobalPoseStore` solo cambian por flujo principal, optimización aceptada o
rollback explícito.

## 9. Backpressure

El flag global se calcula como OR de:

```text
primary_queue por encima del high watermark
secondary_queue por encima del high watermark
optimización de loop activa
optimización fiducial activa
```

La cola secundaria registra profundidad por prioridad para diagnosticar si la
carga procede de fiduciales, actualizaciones HIGH o loops NORMAL.

La histéresis:

- activa en high watermark;
- permanece activa mientras continúe cualquier causa;
- libera en low watermark cuando no hay optimización activa;
- evita oscilaciones rápidas;
- registra edad de la tarea más antigua y tiempo estimado de drenaje.

Mientras el flag está activo:

- el goal de movimiento que ya está activo se deja terminar normalmente;
- el runner no envía el siguiente goal mientras el flag siga activo;
- no se cancela, sustituye ni reenvía el goal activo por backpressure;
- no se solicitan nuevos full snapshots periódicos;
- los ticks de snapshot omitidos no se acumulan;
- deltas o respuestas snapshot ya en vuelo se conservan y encolan;
- las colas continúan drenando;
- el flujo principal y el secundario no se detienen por el flag.

Al desactivarse:

- se reanudan movimientos;
- se solicita como máximo un snapshot fresco por dron;
- no se reproducen todas las solicitudes periódicas omitidas.

## 10. Visualizador web por flujos completos

### 10.1 Identidad de flujo

Todo evento incluye:

```text
flow_id
flow_kind
task_id
edge
phase
priority visual
revision
timestamp backend
```

Tipos:

```text
PRIMARY_DELTA
PRIMARY_SNAPSHOT
FIDUCIAL
DATABASE_UPDATE
LOOP
LOOP_OPTIMIZATION
GLOBAL_MAP
```

### 10.2 Regla de integridad visual

La unidad de descarte es el flujo completo, no una etapa individual.

- si un flujo principal se muestra, se muestra entero;
- si un flujo fiducial se muestra, se muestra entero y siempre es crítico;
- un flujo `DatabaseUpdateTask` puede descartarse, pero siempre como flujo
  completo y nunca eliminando etapas aisladas;
- si un loop normal se admite visualmente, se muestran todas sus etapas;
- si no hay capacidad live, se elimina el loop completo;
- no se elimina BoW dejando RANSAC, ni RANSAC dejando solo fusión;
- descartar un flujo visual nunca cancela la tarea real.

### 10.3 Loops que se convierten en optimización

Un loop normal puede haber sido descartado visualmente antes de conocer su
decisión.

Cuando `LoopDecisionManager` elige optimización:

```text
promover flow_id a LOOP_OPTIMIZATION
-> emitir OPTIMIZATION_DECIDED
-> mostrar obligatoriamente grafo, solver, validación, commit/rollback y fin
```

No es necesario reconstruir visualmente BoW/RANSAC ya descartados. Desde la
decisión de optimizar no se puede perder ninguna etapa.

### 10.4 `GlobalMapBuilder`

Las llamadas normales y repetidas de `GlobalMapBuilder` pueden coalescerse o
saltarse visualmente si la UI empieza a quedar atrasada.

No se puede saltar la actualización de `GlobalMapBuilder` que aplique cambios
pendientes originados por:

- optimización fiducial;
- optimización de loop;
- rollback de poses;
- fusión cuya visualización se haya promovido como relevante.

Los dirty sets conservan `source_task_id/source_kind` para que el builder y la
telemetría sepan si la actualización es descartable o crítica.

### 10.5 Política live

- no existe reproducción artificial de un evento cada 110 ms;
- el navegador procesa por frame;
- los flujos críticos siempre avanzan en tiempo real;
- al saturarse se eliminan flujos LOOP completos aún no representados;
- también pueden eliminarse flujos `DATABASE_UPDATE` completos aún no
  representados;
- un flujo ya empezado termina completo;
- una reconexión no reproduce indiscriminadamente cientos de eventos antiguos;
- el bridge reconstruye el estado crítico actual y continúa en live.

## 11. Cambios previstos por componente

### `global_map_server.cpp`

- reducirlo a orquestación ROS;
- callbacks ligeros que encolan;
- `PrimaryWorker` y `SecondaryWorker` persistentes;
- prioridad estable de la cola secundaria;
- backpressure e histéresis;
- forwarding de `ChangeSet` entre componentes;
- publicación de mensajes devueltos por `GlobalMapBuilder`;
- eliminar cálculo algorítmico y `live_state_mutex_` amplio.

### `RawMapDatabase`

- conservar autoridad raw;
- devolver diffs/revisiones precisos;
- facilitar consultas específicas por IDs para el builder y tareas;
- no ejecutar callbacks ni llamar a otras bases.

### `GlobalPoseStore`

- registrar poses principales de KFs anclados;
- devolver `PoseChangeSet` tras anchor, reconciliación, optimización o rollback;
- conservar autoridad y linaje de poses aceptadas;
- impedir overwrite silencioso desde raw.

### `CovisibilityDatabase` y `LandmarkScoreManager`

- consumir `DatabaseUpdateTask` HIGH;
- exponer cambios concretos y revisiones;
- commits breves;
- permitir filtros baratos antes de loops.

### `FusedLandmarkManager`

- devolver `FusionChangeSet` con tracks y miembros afectados;
- no modificar raw;
- permitir rollback explícito;
- no reconstruir la nube.

### `GlobalMapBuilder`

- pasar de builder puro a vista incremental stateful;
- conservar nube, KFs, índices y dirty sets;
- recalcular solo dependencias afectadas;
- producir mensajes completos listos para publicar;
- no escribir en ninguna base de datos.

### Visualizador

- eventos agrupados por `flow_id`;
- prioridad por tipo de flujo;
- descarte por flujo completo;
- promoción obligatoria al decidir optimización;
- coalescing específico de `GlobalMapBuilder`;
- SSE realmente live.

## 12. Reparto posterior entre subfases

| Subfase | Propiedad que deberá actualizarse |
|---|---|
| `3C` | ingesta delta, cola/worker principales, commit raw, `ChangeSet`, journal/replay y backpressure basico |
| `3D` | poses nuevas, `PoseChangeSet` y autoridad |
| `3E` | primer anchor dentro del flujo principal |
| `3F` | `GlobalMapBuilder` incremental y publicación serial |
| `3G` | snapshots diferenciales y no repetición |
| `3H` | revisit fiducial y tarea MAX |
| `3K` | cola/worker secundarios, prioridades y commits cortos; reutiliza el principal de 3C |
| `3M` | `DatabaseUpdateTask` HIGH de covisibilidad |
| `3N/3O` | `LoopTask` completa y filtros previos |
| `3P` | `FusionChangeSet` y cambios de tracks/scores |
| `3Q` | optimización dentro de la misma `LoopTask` |
| `3S/3T` | scores, IDs, revisiones y trazabilidad |
| `3U` | auditoria/hardening de RViz2 y política visual incremental |
| `3W` | límites, histéresis, latencia y estrés |

No se concentrará toda la modificación documental en `3K`.

## 13. Pruebas necesarias antes de la simulación larga

1. Dos entradas principales se procesan estrictamente una detrás de otra.
2. El secundario trabaja mientras el principal continúa procesando/publicando.
3. Una fiducial pendiente se ejecuta antes de HIGH/NORMAL al terminar la tarea
   activa.
4. Una `DatabaseUpdateTask` pendiente se ejecuta antes de loops pendientes.
5. Un delta con N KFs elegibles crea una tarea independiente por KF, nunca una
   tarea agregada por delta.
6. Cero loops se inician antes del anchor.
7. Delta y snapshot equivalentes no duplican trabajo.
8. Varias notificaciones secundarias antes del siguiente flujo principal se
   acumulan sin perder IDs ni revisiones.
9. `GlobalMapBuilder` recalcula solo elementos dirty y conserva el resto.
10. Una optimización mueve KFs, puntos dependientes y fused tracks afectados.
11. Una fusión retira miembros raw y actualiza el representante sin reconstruir
    geométricamente todos los puntos.
12. Ningún commit secundario provoca por sí mismo una publicación.
13. La siguiente entrada principal aplica todos los cambios secundarios
    pendientes.
14. Backpressure deja terminar el goal activo y bloquea el siguiente.
15. Mientras hay backpressure no se generan solicitudes snapshot acumuladas.
16. El grafo puede descartar `DatabaseUpdateTask` completas, nunca parciales.
17. El grafo conserva flujos completos y promueve optimizaciones desde la
    decisión.

## 14. Criterios de aceptación

```text
active_primary_workers                              <= 1
active_secondary_workers                            <= 1
loops pre-anchor                                    = 0
duplicados exactos de LoopTask                      = 0
LoopTask agregadas por delta                        = 0
overwrites raw de poses aceptadas                   = 0
reconstrucciones geométricas completas por llamada  = 0 tras bootstrap
IDs dirty perdidos entre commits y siguiente build = 0
flujos principales visualizados parcialmente        = 0
loops visualizados parcialmente                      = 0
DatabaseUpdateTask visualizadas parcialmente         = 0
etapas de optimización perdidas tras decisión        = 0
snapshots periódicos acumulados en backpressure      = 0
```

Debe medirse por llamada:

- número total y dirty de KFs/MapPoints/tracks;
- altas, reemplazos y retiradas;
- tiempo de aplicar dirty sets;
- tiempo de serializar/copiar los mensajes completos;
- profundidad/edad de ambas colas;
- causa de activación/liberación del flag;
- flujos web admitidos, descartados, promovidos y coalescidos.

## 15. Exclusiones

- no paralelizar el worker secundario;
- no crear un thread por tarea;
- no publicar desde tareas secundarias;
- no despertar al principal por cada commit secundario;
- no crear `RebuildAfterSecondaryCommit`;
- no permitir loops de submapas no anclados;
- no descartar deltas/snapshots ya recibidos;
- no usar GT fuera de fiducial simulado/debug/métricas;
- no cambiar BoW, RANSAC o solver como parte de esta reorganización;
- no modificar `ORB_SLAM3`, `orbslam3_ros2` u `orbslam3_msgs` salvo bloqueo
  posterior demostrado y nuevo acuerdo.

## 16. Pipeline final resumido

```text
WRAPPER
  -> primary_queue
  -> PrimaryWorker
       -> RawMapDatabase
       -> GlobalPoseStore
       -> FiducialAnchorManager
       -> enqueue DatabaseUpdateTask HIGH
       -> enqueue LoopTask NORMAL si anchored
       -> GlobalMapBuilder aplica dirty acumulado
       -> GlobalMapServer publica RViz2

SecondaryWorker
  -> FiducialCheckOptimizationTask MAX
  -> DatabaseUpdateTask HIGH
  -> LoopTask NORMAL
       -> BoW/filtros/RANSAC/decisión
       -> fusión u optimización
       -> commits de bases
       -> Notify*Changes a GlobalMapBuilder
       -> no publicación

Siguiente delta/snapshot
  -> GlobalMapBuilder consume todos los Notify*Changes pendientes
  -> modifica solo KFs/MapPoints/tracks afectados
  -> GlobalMapServer publica la vista completa actualizada
```
