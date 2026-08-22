# Subfase 3U - RViz2 y diagrama JavaScript del flujo runtime

## Estado vigente

```text
CONSEGUIDA POR AUDITORIA, TESTS Y CIERRE VISUAL DEL USUARIO
```

La infraestructura base pertenece a 3B y cada subfase amplio su
topologia/instrumentacion. La auditoria final confirma que la implementacion
activa ya retiro la cola fija de 110 ms, conecta SSE desde el presente, usa
`Last-Event-ID` y `state_reset`, drena por frame y mantiene el camino secundario
por `flow_id` hasta `done`. El contrato fuente pasa 9/9 y el usuario confirma
que el grafo web es bueno y funciona bien.

Se acepta la politica visual actual, incluido su buffer acotado y la ausencia
de metricas adicionales de latencia por etapa. `3V/3W` comprobaran aislamiento,
reconexion y carga como regresion global, sin convertir esas comprobaciones en
una reimplementacion pendiente de `3U`.

### Trabajo que se conserva

- bridge HTTP/SSE live, aplicación JavaScript y tooltips creados en 3B;
- vertices, aristas y eventos añadidos por 3C-3T;
- telemetría JSON pequeña, acotada y no funcional para el mapa;
- regla de cierre visual definida en `../CONTRATO_VISUAL_INCREMENTAL.md`.

### Implementación anterior ya retirada que no debe recuperarse

- consumir exactamente un evento cada `110 ms` y acumular hasta `400`;
- pulsos de `520 ms` que solapan etapas distintas;
- iniciar SSE en secuencia cero y reenviar hasta `512` eventos antiguos;
- descartar etapas sueltas dejando flujos visuales incompletos;
- carecer de latencia backend->bridge->browser->render y señal de gaps.

### Contrato de cierre satisfecho

El bridge asigna secuencia SSE, conserva cursores validos y fuerza
`state_reset` cuando el historial ya no permite reconstruccion. Los eventos
funcionales llevan `flow_id` y las tareas secundarias un lifecycle estable por
`task_id`; la topologia diferencia flujo principal, fiducial, loop, fusion,
score y publicacion. La observabilidad permanece best-effort y nunca gobierna
ROS, commits, prioridades o backpressure.

## Estado histórico anterior

Las secciones posteriores conservan contrato y evidencia de la implementacion
anterior. Si una formulacion histórica contradice el estado de cierre de esta
cabecera, prevalece la implementacion live auditada.

```text
CONSEGUIDA: transporte live, reconexion, topologia, lifecycle y cierre visual
aceptados.
```

## Objetivo

Ofrecer simultaneamente:

1. RViz2 para inspeccionar KFs y nube sparse global.
2. Una aplicacion Cytoscape.js que muestra transferencias reales entre
   componentes sin modificar ni bloquear el pipeline.

`GlobalMapBuilder`, `LandmarkScoreManager`, RViz2, los frustums de KFs y las
aristas iniciales de publicacion ya se crean incrementalmente en 3F. 3U no
pospone ni reinventa esos elementos: audita la topologia completa acumulada,
corrige rendimiento/reconexion y endurece la experiencia visual final.

## Arquitectura vigente

```text
GlobalMapServer::TryTraceFlow
  -> /global_mapping/flow_events (std_msgs/msg/String, JSON)
  -> pipeline_flow_bridge.py
  -> EventStore acotado
  -> SSE /events
  -> app.js + Cytoscape.js
```

El bridge sirve los assets y SSE en `127.0.0.1`. No usa rosbridge ni WebSocket,
no requiere internet y no admite comandos desde el navegador.

Limitaciones runtime confirmadas en el diagnostico 75/76:

- `app.js` consume un evento cada `110 ms`, conserva `400` y solapa pulsos de
  `520 ms`;
- el handler SSE empieza siempre en secuencia cero y puede reenviar hasta
  `512` eventos historicos al conectar/reconectar;
- no se mide la latencia entre `stamp_ns`, recepcion SSE y render.

El cierre de `3U` exige eliminar ese comportamiento de replay y demostrar que
la UI converge al estado actual bajo rafagas sin ralentizar ROS.

## Topologia

`simulacion_dron/web/pipeline_flow/graph_definition.js` declara los nodos,
responsabilidades, datos propios y aristas. La topologia incluye:

```text
Wrappers ORB-SLAM3
GlobalMapServer
RawMapDatabase
FiducialAnchorManager
GlobalPoseStore
CovisibilityDatabase
LandmarkScoreManager
SecondaryTaskQueue
LoopDetector
SubcloudLoopVerifier
LoopDecisionManager
PoseGraphBuilder
OptimizationManager
FusedLandmarkManager
GlobalMapBuilder
RViz2
```

Aristas distintas representan contratos distintos aunque compartan origen y
destino. Casos obligatorios:

- wrapper -> server: `delta` y `snapshot` separadas;
- server -> raw: commit delta y commit snapshot separados;
- raw -> pose: KF nuevo y reconciliacion snapshot separados;
- queue -> detector/builder: inicio de loop y optimizacion fiducial;
- detector -> verifier -> decision -> fusion/grafo/solver;
- bases raw/poses/covisibilidad/scores/fusion -> `GlobalMapBuilder`;
- server -> RViz2: nube y KFs como aristas independientes.

## Semantica visual

Colores por categoria:

```text
azul     ingesta raw
ambar    anchors y poses
magenta  trabajo secundario
verde    estado derivado
turquesa publicacion
```

`app.js` recibe eventos y los reproduce uno a uno con un intervalo visual
pequeno. No activa de golpe todo el lote recibido. Para el worker:

- `task_id` identifica la tarea;
- al avanzar una etapa se apaga la etapa anterior de esa tarea;
- al empezar una tarea nueva se limpia el estado secundario previo;
- solo queda iluminada la etapa actual;
- los pulsos del flujo principal caducan de forma breve e independiente.

La barra de actividad muestra edge, phase, payload, cantidades y `task_id`.
Hover sobre nodo/arista ofrece responsabilidad y contrato de datos. Las
aristas paralelas usan offsets para no superponerse.

## Instrumentacion

`TryTraceFlow` emite metadatos ligeros y nunca nubes, descriptores o matrices.
La cola del servidor es acotada y no bloqueante; si esta ocupada, el evento se
descarta. Fronteras instrumentadas:

- recepcion wrapper delta/snapshot;
- commits raw y propagacion de poses;
- anchors/revisits fiduciales;
- actualizacion de covisibilidad y score;
- enqueue/start y etapas de `LoopTask`;
- grafo/solver y commits de poses/fusion;
- captura de `GlobalMapBuilder` y publicacion cloud/KFs.

El fallo o cierre del bridge no altera ROS, las bases, RViz2 ni backpressure.

## Archivos

```text
orbslam3_server/src/global_map_server.cpp
simulacion_dron/src/visualizer/pipeline_flow_bridge.py
simulacion_dron/web/pipeline_flow/index.html
simulacion_dron/web/pipeline_flow/styles.css
simulacion_dron/web/pipeline_flow/graph_definition.js
simulacion_dron/web/pipeline_flow/app.js
simulacion_dron/web/pipeline_flow/vendor/cytoscape.min.js
simulacion_dron/launch/multi_dron.launch.py
simulacion_dron/CMakeLists.txt
```

## Launch

`multi_dron.launch.py` expone:

```text
launch_pipeline_flow_visualizer:=true
open_pipeline_flow_browser:=true
pipeline_flow_port:=8765
```

El servidor global expone:

```text
flow_telemetry_enabled:=true
flow_telemetry_topic:=/global_mapping/flow_events
```

## RViz2

```text
/global_sparse_cloud   sensor_msgs/msg/PointCloud2
/global_keyframes      visualization_msgs/msg/MarkerArray
frame_id               world
```

Nube y KFs proceden de la misma revision de publicacion. RViz2 no envia ACK y
no mantiene abierta ninguna tarea.

Los frustums conservan el color determinista por submapa establecido en 3F. La
nube puede usar el RGB temporal rojo-amarillo-verde generado por el servidor a
partir de `score`; 3U no convierte ese color en dato autoritativo ni modifica
scores. La transferencia de esa presentacion a la GUI pertenece a 7E.

## Validacion

- Assets/Cytoscape: captura desktop no vacia y sin error JavaScript.
- Transporte: evento sintetico `wrapper_server_delta` recibido por ROS y
  expuesto por SSE.
- `prueba_75/76`: bridge `READY`, simulacion `success=true` y cierre limpio del
  bridge; la telemetria no activa backpressure.
- `prueba_76`: `144` publicaciones mientras el worker mantiene backlog.

No se pudo automatizar una captura headless del pulso SSE porque Chrome toma la
captura antes de procesar `EventSource`; la recepcion se verifico directamente
en el endpoint. Falta confirmacion humana final de que el comportamiento
mejorado satisface la lectura visual durante una ejecucion real.

## Criterios

- eventos reales, no animacion decorativa;
- delta/snapshot y cloud/KFs distinguibles;
- una sola etapa secundaria activa por tarea;
- tooltips con responsabilidad y payload;
- desconexion o saturacion inocua;
- ningun wait del pipeline por HTML, bridge o navegador.
