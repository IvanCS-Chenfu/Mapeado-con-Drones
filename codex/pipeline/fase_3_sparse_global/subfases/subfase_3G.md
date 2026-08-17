# Subfase 3G - Full snapshot y reconciliacion incremental diferida

## Estado

```text
CONSEGUIDA
Preparacion: CERRADA
Acuerdo funcional: CERRADO
Autorizacion de implementacion: CONCEDIDA Y EJECUTADA
Validacion automatica: CONSEGUIDA; build 4/4, C++ 37/37 y web 8/8
Validacion live: CONSEGUIDA; visual 133, carga real 137 y restauracion 138
```

La implementacion anterior queda como evidencia en
`../historial/por_subfase/historial_3G.md`. No debe recuperarse literalmente:
guardaba snapshots completos, publicaba desde su propia tarea y hacia trabajo
innecesario ante snapshots no materiales.

## Objetivo

Pedir mapas completos a los wrappers para reconciliar deltas perdidos o cambios
antiguos de ORB-SLAM3, sin degradar la autoridad world ni recorrer ramas que no
hayan cambiado.

```text
RawMapDatabase  = autoridad de datos ORB-SLAM3 raw/locales
GlobalPoseStore = autoridad de anchors y poses world
LandmarkScoreManager = autoridad del score inicial
GlobalMapBuilder = vista materializada incremental
```

Un snapshot compromete autoridades y acumula IDs dirty. No ejecuta el builder,
no serializa ROS y no publica. La siguiente entrada delta normal consume todos
los dirty pendientes, actualiza solo las caches afectadas y publica si la vista
cambia. Si no llega otro delta, RViz2 conserva deliberadamente la ultima
revision publicada.

## Entradas y orden

`PrimaryInput` debe separar dos dimensiones:

```text
source = live | replay
kind   = delta | full_snapshot
```

- `live` significa recibido mediante ROS, desde Gazebo o un dron real.
- `replay` significa leido desde un `.record`.
- deltas y snapshots live comparten `PrimaryQueue`, `arrival_id` y el unico
  `PrimaryWorker`;
- no existe snapshot worker ni commit desde la callback ROS;
- la respuesta de `get_full_map` solo valida identidad/contenido, encola y
  emite telemetria.

## Solicitud de snapshots

`GlobalMapServer` crea un cliente asincrono por dron:

```text
/dron_X/orbslam/get_full_map
```

Parametros iniciales acordados:

```text
full_snapshot_enabled=true
full_snapshot_startup_delay_sec=35.0
full_snapshot_period_sec=35.0
```

Por dron se conserva como maximo una solicitud en vuelo. Durante backpressure
se omiten ticks periodicos nuevos; una respuesta ya en vuelo se acepta. Al
liberar backpressure se permite como maximo una solicitud fresca por dron, sin
reproducir ticks acumulados.

Un salto de `map_sequence` es solo un indicio de resincronizacion: el wrapper
tambien incrementa la secuencia al construir deltas vacios que no publica. Puede
adelantar una solicitud si no hay otra en vuelo, pero nunca se registra como
perdida confirmada.

## Diff autoritativo en `RawMapDatabase`

`InsertFullSnapshot(arrival_id, snapshot)` compara el snapshot completo con el
submapa `(drone_id,map_epoch)` y compromete un diff preciso. La comparacion de
un snapshot grande se prepara sobre estado privado/versionado; el commit live
es breve y se valida contra la revision raw capturada.

El resultado no debe limitarse a `updated_ids`. Debe separar al menos:

```text
new_keyframe_ids
pose_changed_keyframe_ids
association_changed_keyframe_ids
covisibility_changed_keyframe_ids
invalidated_keyframe_ids

new_mappoint_ids
geometry_changed_mappoint_ids
score_input_changed_mappoint_ids
association_changed_mappoint_ids
invalidated_mappoint_ids

has_material_changes
normalized_delta
```

Clasificacion minima de MapPoints:

- posicion, observador, KF de referencia u observaciones geometricas: dirty de
  geometria para `GlobalMapBuilder`;
- `observations_count`, `found_ratio`, descriptor valido o `is_bad`: entrada
  para `LandmarkScoreManager`;
- un MP puede pertenecer simultaneamente a varias categorias.

Como el wrapper omite entidades `bad` en un full snapshot, un KF/MP activo que
estuviera en raw y ya no aparezca se marca inactivo. No se borra fisicamente:
se conservan identidad, revisiones y trazabilidad.

## Ejecucion selectiva

### Snapshot no-op

```text
SnapshotInput -> InsertFullSnapshot -> has_material_changes=false -> END
```

No llama a pose store, score, builder, fiduciales ni publicacion; tampoco crea
entrada en el `.record`.

### Snapshot material

`RawMapDatabase` dirige solo las ramas necesarias mediante su `ChangeSet`:

| Cambio | Autoridad actualizada | Dirty acumulado |
|---|---|---|
| pose/validez de KF | `GlobalPoseStore` si el submapa esta anclado | KF |
| asociacion KF-MP | ninguna base derivada extra | KF y MPs añadidos/retirados |
| geometria/referencia de MP | raw ya comprometido | MP |
| entradas de score de MP | `LandmarkScoreManager` | MP si cambia el score |
| invalidacion | pose/score cuando corresponda | KF o MP retirado |

Notificar un KF dirty hace que, en el siguiente `GlobalMapBuilder::Update()`, el
builder consulte sus `mappoint_ids` actuales en raw y recalcule solo sus MPs
asociados. Una asociacion nueva debe ensuciar tanto el KF como los MPs afectados.

El snapshot termina despues de los commits selectivos y de acumular dirty:

```text
NO GlobalMapBuilder::Update()
NO PointCloud2/MarkerArray
NO observacion fiducial
NO publicacion ROS
```

El siguiente delta normal procesa su propio `ChangeSet`, drena la union de
dirty anteriores y nuevos, consulta por ID las autoridades y sustituye solo las
caches afectadas.

## Reconciliacion de poses

Ante un cambio de pose local raw:

- sin anchor: el KF permanece solo en raw;
- pose `SubmapAnchorDerived`: recalcular `world_T_kf` con el anchor vigente;
- pose fiducial aceptada u optimizada: conservar `world_T_kf`, actualizar
  `base_raw_revision` y recalcular su correccion raw-world;
- una invalidacion conserva el linaje y marca la pose inactiva.

Un KF con world preservado tambien se marca dirty: cambiar su pose local puede
recolocar sus MPs aunque la pose world no se mueva.

La optimizacion real no pertenece a 3G. Un test sintetico puede marcar un KF
como `LoopOptimized`, modificar despues su raw local y verificar que world no
cambia y que la correccion/revision raw si se actualiza.

## `.record` delta-only

El `.record` sigue siendo un diario de entradas reproducibles, no un volcado de
las bases. No se almacena ningun full snapshot.

- delta ROS normal: se guarda como hasta 3F;
- snapshot no-op: no se guarda;
- snapshot material: `RawMapDatabase` genera un delta normalizado con el mismo
  `arrival_id` y solo las entidades nuevas, modificadas o invalidadas;
- una ausencia invalidada se representa mediante su tombstone `is_bad`;
- el delta normalizado contiene objetos completos para cada ID afectado, no
  parches ambiguos de campos;
- pueden existir huecos de `arrival_id` por snapshots no-op; el orden debe ser
  estrictamente creciente, no necesariamente contiguo.

Replay deserializa exclusivamente deltas y los reinyecta como
`source=replay, kind=delta` por la misma FIFO/worker. Asi reconstruye los efectos
materiales del snapshot sin guardar mapas completos ni crear una ruta replay
especial.

Las observaciones fiduciales normalizadas continúan en su journal separado. Un
delta normalizado de snapshot no inventa asociaciones GT/fiduciales.

## Backpressure y prueba debug

Se conserva la histeresis principal `high=8`, `low=2`. La instrumentacion de
prueba puede omitir deliberadamente un unico delta live de un dron:

```text
debug_drop_one_delta_for_snapshot_test=false
```

Debe estar desactivada por defecto, ser one-shot, registrar identidad,
secuencia y conteos del delta omitido y no alterar el wrapper. Solo existe para
demostrar deterministicamente que el snapshot recupera lo perdido.

## Grafo web

Se conserva el layout 3F aceptado. No se añade un vertice nuevo ni una arista
de solicitud servidor->wrapper. Se incorporan tres aristas distintas:

```text
Wrappers -> GlobalMapServer       snapshot
PrimaryWorker -> RawMapDatabase   full commit
RawMapDatabase -> GlobalPoseStore snapshot reconcile
```

Las aristas existentes raw->score, raw/pose/score->builder se activan solo si
esa rama recibe datos. Notificar dirty al builder no significa que este se haya
ejecutado. Un snapshot no-op solo muestra recepcion y full commit/no-op.

## Archivos probables

```text
orbslam3_server/include/orbslam3_server/primary_queue.hpp
orbslam3_server/src/global_map_server.cpp
orbslam3_server/launch/global_orb_map_server.launch.py
orbslam3_multi/include/orbslam3_multi/raw_map_types.hpp
orbslam3_multi/include/orbslam3_multi/raw_map_database.hpp
orbslam3_multi/src/raw_map_database.cpp
orbslam3_multi/include/orbslam3_multi/global_pose_types.hpp
orbslam3_multi/include/orbslam3_multi/global_pose_store.hpp
orbslam3_multi/src/global_pose_store.cpp
orbslam3_multi/include/orbslam3_multi/sparse_global_backend.hpp
orbslam3_multi/src/sparse_global_backend.cpp
simulacion_dron/web/pipeline_flow/graph_definition.js
tests focales de los tres paquetes
```

No modificar `ORB_SLAM3`, `orbslam3_ros2` ni `orbslam3_msgs` salvo bug
bloqueante demostrado. No implementar loops, fusion, optimizacion real,
covisibilidad global ni worker secundario.

## Validacion acordada

### Tests focales

1. FIFO mixta conserva orden y distingue `source/kind`.
2. Snapshot identico produce no-op, sin ramas derivadas, dirty ni record.
3. Elementos activos ausentes se invalidan sin borrarse.
4. Cada categoria del `ChangeSet` activa solo su autoridad correspondiente.
5. Dirty de snapshot persiste hasta el siguiente delta normal.
6. El siguiente delta actualiza solo caches afectadas y publica una vez.
7. Una pose `LoopOptimized` sintetica conserva world y rebasa raw/correccion.
8. El delta normalizado reproduce el mismo estado raw/pose/score final.
9. Backpressure omite ticks y limita solicitudes en vuelo.

### Prueba 98 live

1. Arrancar Gazebo, RViz2 y grafo web desde el launch oficial.
2. Llevar ambos drones al fiducial 2 y confirmar los dos anchors.
3. Activar el drop one-shot para perder un delta de prueba.
4. Esperar la solicitud/recepcion de snapshot y comprobar su recuperacion.
5. Verificar ramas selectivas y dirty acumulado sin build/publicacion inmediata.
6. Provocar/esperar el siguiente delta normal.
7. Comprobar que el builder consume los dirty y RViz2 actualiza sin duplicados.
8. Guardar un `.record` compuesto solo por deltas originales y normalizados.

### Prueba 99 replay

Reproducir el `.record` de la prueba 98 sin Gazebo. Debe reconstruir el mismo
estado final raw/pose/score y la misma vista final, sin eventos
`kind=full_snapshot`. RViz2 no es obligatorio; la validacion principal es por
logs, revisiones, conteos y digest final.

## Criterios de exito

3G queda `CONSEGUIDA` solo si:

- build y tests focales terminan correctamente;
- los dos snapshots live se solicitan y reciben sin bloquear callbacks;
- el delta omitido se recupera de forma demostrable;
- no-op detiene el flujo y un snapshot material activa solo ramas necesarias;
- no se ejecuta/publica el builder durante la tarea snapshot;
- el siguiente delta consume dirty y produce una revision coherente;
- world aceptado/optimizado nunca es sobrescrito desde raw;
- el `.record` no contiene snapshots y el replay alcanza el mismo digest final;
- backpressure no acumula solicitudes;
- web refleja eventos reales y RViz2 no pierde ni duplica la vista;
- no aparecen errores graves ni se modifica codigo prohibido.

Marcar `PARCIAL` si live funciona pero replay/digest, pose optimizada o
diferimiento del builder no quedan demostrados. Marcar `NO CONSEGUIDA` ante
perdida de autoridad world, publicacion desde snapshot, divergencia final del
record, duplicacion masiva o fallo de build/runtime.

## Perfil operativo validado

El cierre de 3G añade una restriccion de operacion para no repetir la
saturacion de la prueba 98:

```text
desarrollo visual, 2 drones:
  Gazebo GUI + RViz2 + web + GUI de mision

escala, 3 o mas drones / futuras fases dense:
  Gazebo headless + RViz2/web/GUI de mision selectivos
```

El launch multi-dron usa por defecto stagger de 8 s, vocabulario ORB L5
compacto, camara 480x360 a 20 Hz y 900 features. El vocabulario completo sigue
seleccionable y es el default del launch individual. No bajar mas calidad sin
una prueba explicita de tracking, relocalizacion y loops.

Evidencia de cierre: live visual 133 completo con minimo disponible 612.3 MiB;
carga real 137 con tres drones, seis goals, tres anchors, 141 KFs activos y
minimo 878.8 MiB; prueba visual 138 del estado normal restaurado, minimo 946.6
MiB. Ninguna activo la guarda y el PSI de memoria fue nulo o despreciable. El
detalle cronologico, incluidos los intentos fallidos, esta en
`historial_3G.md`.

## Marcadores minimos

```text
[F3G-SNAPSHOT-CLIENT-READY]
[F3G-SNAPSHOT-REQUEST]
[F3G-SNAPSHOT-RX]
[F3G-SNAPSHOT-NOOP]
[F3G-SNAPSHOT-DIFF]
[F3G-SNAPSHOT-DIRTY-DEFERRED]
[F3G-NORMALIZED-DELTA-RECORD]
[F3G-DEBUG-DELTA-DROPPED]
[F3G-DIRTY-CONSUMED-BY-DELTA]
[F3G-POSE-WORLD-PRESERVED]
```

Los logs completos se conservan como artefactos, pero solo se analizan mediante
reducidos específicos conforme a `AGENTS.md`.
