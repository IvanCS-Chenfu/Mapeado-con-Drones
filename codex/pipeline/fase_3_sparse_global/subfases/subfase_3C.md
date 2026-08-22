# Subfase 3C - Ingesta raw, flujo principal y backpressure basico

## Estado vigente

```text
CONSEGUIDA
Preparacion: CERRADA
Acuerdo cerrado: si
Autorizacion funcional: CONCEDIDA Y EJECUTADA
Dudas abiertas: ninguna
```

Contrato ejecutado y cerrado con las pruebas 85 live y 86 replay. La evidencia
cronológica, incluidos los intentos no concluyentes, está en
`historial_3C.md` y `historial_3C_RESUMEN.md`.

## Objetivo

Reconstruir la primera ruta real del mapa global y dejar creada la base
definitiva del flujo principal:

```text
Wrappers ORB-SLAM3
  -> callback ROS ligero de GlobalMapServer
  -> PrimaryQueue FIFO
  -> PrimaryWorker unico y persistente
  -> RawMapDatabase::InsertDelta()
  -> RawInsertResult / RawChangeSet
  -> fin de PrimaryTask en 3C
```

En 3C cada tarea termina justo despues del commit raw. Las subfases siguientes
alargaran esta misma tarea; no crearan otra cola ni otro worker principal.

## Propiedad funcional

### `GlobalMapServer`

- Se suscribe a los topics `OrbMap` reales de ambos wrappers.
- Cada callback valida lo minimo, asigna un `arrival_id` monotono y construye
  un `DeltaInput` inmutable.
- La asignacion de `arrival_id` y el enqueue forman una unica operacion
  ordenada, incluso con callbacks concurrentes.
- El callback no inserta en bases, no serializa, no publica mapa y no ejecuta
  algoritmos; encola y retorna con rapidez.
- El servidor orquesta el flujo, pero delega el almacenamiento en
  `orbslam3_multi::RawMapDatabase`.

### `PrimaryQueue`

- Es una FIFO de `DeltaInput` ordenada por `arrival_id`.
- Puede recibir entradas mientras el worker procesa la anterior.
- En 3C la profundidad publicada es el numero de mensajes pendientes; la tarea
  activa se mide aparte.
- No descarta, sustituye ni agrupa deltas raw.
- El acceso a la cola usa un mutex breve y `condition_variable`, sin polling.
- No existe todavia una capacidad dura que pueda causar perdida raw. Los
  los limites finales, no-progreso y memoria se validan en la auditoria de
  rendimiento absorbida por 3T.

### `PrimaryWorker`

- Existe exactamente un consumidor persistente y como maximo una
  `PrimaryTask` activa.
- Hace dequeue FIFO, ejecuta `InsertDelta()`, registra el resultado y cierra la
  tarea.
- No mantiene el lock de la cola durante importacion, commit raw, I/O,
  telemetria o espera diagnostica.
- Expone enqueue/start/end, `arrival_id`, profundidad, duracion, resultado y
  `active_primary_tasks`, que solo puede valer 0 o 1.
- En shutdown se deja de admitir entrada, se drena lo aceptado y se hace
  `join`; una excepcion se registra sin dejar un commit parcial ni perder
  silenciosamente una entrada.

### Extensiones obligatorias posteriores

| Subfase | Nuevo punto final de la misma `PrimaryTask` |
|---|---|
| 3C | commit en `RawMapDatabase` |
| 3D | commit raw + actualizacion de `GlobalPoseStore` |
| 3E | ruta anterior + anclaje inicial aplicable |
| 3F | ruta anterior + build y publicacion ROS coherentes |
| 3G | incorpora `SnapshotInput` a la misma cola y ejecuta el flujo completo |

3K solo crea el flujo secundario. Debe reutilizar el principal nacido aqui.

## `RawMapDatabase`

La base pertenece a `orbslam3_multi` y es autoridad exclusiva del estado
ORB-SLAM3 crudo por `SubmapId = (drone_id, map_epoch)`.

Debe conservar:

- KFs y sus poses locales originales;
- MPs, observaciones y asociaciones KF-MP;
- BoW/apariencia recibida, covisibilidad y relaciones locales disponibles;
- revisiones por entidad y revision material del submapa;
- journal ordenado por `arrival_id` para record/replay.

`InsertDelta()` devuelve un `RawInsertResult`/`RawChangeSet` preciso con:

- submapa y `arrival_id` de origen;
- KFs y MPs creados, modificados materialmente o retirados;
- asociaciones y covisibilidades añadidas, cambiadas o retiradas;
- revisiones anteriores y nuevas de las entidades afectadas;
- contadores finales y clasificacion material/no-op.

La base debe ofrecer consultas acotadas por `SubmapId` e IDs. Queda prohibido
obligar a consumidores futuros a copiar o recorrer todo raw para resolver un
cambio local.

El commit raw es breve, no llama a otras bases y no contiene poses world,
anchors, fiduciales globales, loops multi-dron, fusion, optimizacion ni
publicacion. Ningun proceso posterior puede escribir sus resultados en raw.

## Record y replay

- Se define un `.record` nuevo, explicito y versionado; no se exige
  compatibilidad binaria con datasets legacy.
- En 3C solo contiene entradas `delta`. En 3G tampoco se guardan full snapshots:
  un snapshot material se convierte en un delta normalizado con el mismo
  `arrival_id`, y un snapshot no-op no crea entrada.
- Guardado, carga y serializacion ocurren fuera de callbacks y locks de cola.
- El replay carga el journal y reinyecta cada `DeltaInput` por la misma
  `PrimaryQueue` y el mismo `PrimaryWorker`; no llama directamente a raw.
- El resultado final debe conservar orden, numero de entradas, identidades,
  revisiones y estadisticas raw del record.
- En replay no se simula actividad wrapper. La telemetria desde servidor hasta
  raw se marca con `source=replay`.

## Backpressure basico

3C crea la primera politica funcional basada solo en pendientes de
`PrimaryQueue`:

```text
primary_queue_high_watermark = 8
primary_queue_low_watermark = 2
primary_worker_debug_delay_ms = 0
```

- Si `pending >= 8`, el estado pasa a `true`.
- Una vez activo, solo vuelve a `false` cuando `pending <= 2`.
- Entre ambos umbrales se conserva el estado anterior para evitar oscilacion.
- El servidor publica `/global_mapping/backpressure_active` con QoS `reliable`
  y `transient_local`, incluido el estado inicial `false` y cada transicion.
- No se cancelan goals ni se detienen callbacks o commits. El goal activo
  termina normalmente.
- `scenario_runner_node` retiene solo el siguiente lote de movimiento mientras
  el flag sea `true`; los pasos `wait` siguen avanzando.
- Al liberarse, el runner envia los goals originales del siguiente lote una
  sola vez, sin recrearlos ni duplicarlos.
- En 3C el gate solo cubre misiones del scenario runner. Clientes manuales,
  GUI y clientes action externos quedan fuera.
- Esta politica nunca autoriza drops raw.

`primary_worker_debug_delay_ms` es una ayuda de prueba modificable sin dormir
bajo locks. Su valor normal y real es `0`; solo se usa un valor no nulo en la
prueba dedicada para acumular cola, observar el gate y despues permitir el
drenaje. No se activa en pruebas de otras funciones.

La politica final amplio esta base con cola secundaria, edad, drenaje estimado,
optimizaciones activas, dwell, capacidad y deteccion de no-progreso. El umbral
por profundidad de 3C es un bootstrap deliberado, no la politica final.

## Grafo web obligatorio

La topologia de 3C debe representar el runtime real con estos vertices:

```text
Wrappers ORB-SLAM3
GlobalMapServer
PrimaryQueue
PrimaryWorker
RawMapDatabase
ScenarioRunner / MissionGate
```

Aristas y eventos:

- wrapper -> server: recepcion real de delta live;
- server -> `PrimaryQueue`: enqueue con `arrival_id` y profundidad;
- `PrimaryQueue` -> `PrimaryWorker`: dequeue/start;
- `PrimaryWorker` -> `RawMapDatabase`: commit y `RawInsertResult`;
- publisher de backpressure del servidor -> mission gate: cambio de flag.

Los eventos contienen solo metadatos: IDs, revisiones, cantidades,
profundidad, high/low, no-op, duracion, estado y `source`. Nunca transportan
nubes, descriptores, matrices ni mensajes `OrbMap` completos.

Durante replay no se ilumina wrapper -> server. Se muestran servidor, cola,
worker y raw con `source=replay`. La observabilidad sigue siendo acotada y no
bloqueante; un fallo web no afecta al pipeline.

## RViz2

RViz2 se abre en la prueba, pero debe permanecer sin nube sparse global ni KFs
globales durante toda 3C. Los publishers espaciales se crean en 3F.

## Archivos probables

```text
orbslam3_multi/include/orbslam3_multi/raw_map_types.hpp
orbslam3_multi/include/orbslam3_multi/raw_map_database.hpp
orbslam3_multi/src/raw_map_database.cpp
orbslam3_multi/CMakeLists.txt
orbslam3_server/include/orbslam3_server/primary_queue.hpp
orbslam3_server/src/global_map_server.cpp
orbslam3_server/test/test_primary_queue.cpp
orbslam3_server/launch/global_orb_map_server.launch.py
orbslam3_server/CMakeLists.txt
simulacion_dron/src/control_tray/scenario_runner_node.cpp
simulacion_dron/src/visualizer/pipeline_flow_bridge.py
simulacion_dron/web/pipeline_flow/*
simulacion_dron/launch/f3c_replay.launch.py
codex/archivos_auxiliares/trayectorias/tray_prueba_79.yaml
codex/archivos_auxiliares/trayectorias/tray_prueba_80.yaml
```

Se actualizarán los MD vigentes de cada paquete tocado. La baseline anterior
era solo referencia: no se compilaba, modificaba ni copiaba en bloque.

## Fuera de alcance

- full snapshots y sus clientes (`3G`);
- poses globales (`3D`), fiduciales (`3E/3H`) y publicaciones RViz2 (`3F`);
- covisibilidad derivada, score, BoW global, loops, matching o RANSAC;
- fusion, grafo, solver u optimizacion;
- `SecondaryQueue`/`SecondaryWorker` y sus prioridades (`3K`);
- politica completa de rendimiento y capacidad, absorbida por `3T`;
- compatibilidad obligatoria con `.record` legacy.

## Validacion acordada

### Tests deterministas

1. Dos productores concurrentes reciben `arrival_id` unico y el worker los
   procesa en el mismo orden FIFO.
2. `active_primary_tasks <= 1` siempre.
3. Identidad `(drone_id,map_epoch)` y cambios/revisiones/no-op del
   `RawChangeSet` son exactos y las consultas por IDs quedan acotadas.
4. Save/load conserva journal y estado final; replay atraviesa cola y worker.
5. La histeresis activa en 8, no oscila entre umbrales y libera en 2.
6. No hay drops raw y el shutdown drena o informa explicitamente un fallo.

### Prueba live

- Compilar `orbslam3_multi`, `orbslam3_server` y `simulacion_dron`.
- Abrir Gazebo, RViz2 y grafo web.
- Mover ambos drones en paralelo: fiducial 2 -> `x=-8` -> fiducial 2.
- Usar high=8, low=2 y delay diagnostico no nulo hasta provocar acumulacion;
  liberar el delay para observar drenaje y reanudacion.
- Verificar que el goal activo termina, el siguiente movimiento se retiene,
  el flag vuelve a `false` y los goals pendientes se envian exactamente una
  vez.
- Guardar el nuevo record de deltas.
- Confirmar actividad real y ordenada en todas las aristas del grafo y RViz2
  vacio durante toda la ejecucion.

### Prueba replay

- Ejecutar sin Gazebo usando el record live.
- Comprobar mismo orden, numero de entradas y estadisticas finales.
- Comprobar el recorrido server -> queue -> worker -> raw con
  `source=replay`, sin pulsos de wrapper.
- Repetir el modo normal con `primary_worker_debug_delay_ms=0`.

Marcadores minimos:

```text
[F3C-SERVER-SUBSCRIBED]
[F3C-PRIMARY-ENQUEUE]
[F3C-PRIMARY-START]
[F3C-RAW-COMMIT]
[F3C-PRIMARY-END]
[F3C-BACKPRESSURE]
[F3C-RECORD-SAVE]
[F3C-REPLAY-LOAD]
[F3C-REPLAY-DONE]
[F3C-PRIMARY-SHUTDOWN]
```

## Criterio de cierre

`CONSEGUIDA` exige build, tests, live, replay, grafo web y comprobacion visual
de RViz2. Sera `PARCIAL` si funciona raw/replay pero no se demuestra FIFO,
gate/drenaje/reanudacion o visualizacion real. Sera `NO CONSEGUIDA` si se
pierden deltas, hay mas de una tarea principal activa, el callback hace trabajo
pesado, replay evita la cola, el gate duplica/cancela goals o RViz2 muestra una
salida global prematura.

## Lo ya hecho que no debe repetirse

- Ejecutar el flujo principal dentro de callbacks ROS.
- Llamar `RawMapDatabase` directamente desde la subscription.
- Crear `PrimaryWorker` tarde en 3K o mantener dos rutas principales.
- Copiar todo raw mediante `CreateStateSnapshot()` por publicacion o loop.
- Recorrer todos los submapas para una consulta de pocos IDs.
- Mantener locks mientras se copian contenedores grandes o se hace I/O.
- Devolver diffs amplios, ambiguos o sin revisiones por entidad.
- Hacer que raw coordine poses, score, covisibilidad, loops o publicacion.
- Publicar backpressure permanentemente a `false` o usarlo para cancelar el
  goal activo.
- Dibujar eventos web ficticios o payloads pesados.

La capacidad historica de raw, journal, record y replay se usa como referencia
selectiva. No se recupera el servidor legacy completo.
