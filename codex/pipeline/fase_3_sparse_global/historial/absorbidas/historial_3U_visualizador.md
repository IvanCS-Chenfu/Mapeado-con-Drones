# Historial 3U

## 2026-08-05 - Diagrama de transferencias en tiempo real

- objetivo intentado: mostrar en una pagina web las transferencias reales entre
  clases, con aristas separadas por tipo de dato y sin bloquear ROS;
- archivos modificados: `global_map_server.cpp`, `multi_dron.launch.py`,
  `pipeline_flow_bridge.py`, `graph_definition.js`, `app.js`, `styles.css`,
  `index.html`, `CMakeLists.txt` y `package.xml`;
- implementacion: topic JSON ligero `/global_mapping/flow_events`, cola
  acotada, bridge SSE local y grafo Cytoscape.js con nodos/aristas estables;
- mejora tras revision del usuario: aristas wrapper-servidor y servidor-raw
  separadas para delta/snapshot, cloud/KFs separadas hacia RViz2, categorias de
  color y reproduccion de eventos uno a uno;
- worker visual: cada `task_id` apaga su etapa previa al avanzar; no se
  iluminan simultaneamente todas las fases secundarias de un lote;
- build: `orbslam3_multi orbslam3_server simulacion_dron`, codigo `0`;
- pruebas: render desktop estatico correcto; evento ROS sintetico recibido en
  SSE; `prueba_75` y `prueba_76` con bridge `READY`, escenario correcto y
  cierre limpio;
- evidencia de aislamiento: cola/backlog secundario y `144` publicaciones en
  `prueba_76`, con backpressure siempre `false`;
- evidencia negativa o ausente: Chrome headless capturo antes de procesar el
  primer `EventSource`; no existe confirmacion humana nueva del pulso live ni
  de RViz2 en la prueba 76;
- revision del 2026-08-09 tras observacion humana y lectura del codigo:
  - `app.js` consume `9.09` eventos/s y puede acumular unos `44 s`;
  - los pulsos de `520 ms` solapan varios eventos no-task;
  - cada reconexion SSE empieza desde cero y reenvia hasta `512` eventos;
  - no existe metrica de frescura backend/SSE/render;
- conclusion revisada: `PARCIAL`. Transporte y topologia estan validados; la
  semantica live no lo esta y reproduce eventos antiguos en rafagas/reconexion;
- siguiente paso recomendado: representar el estado mas reciente, respetar
  `Last-Event-ID` o conectar desde latest y medir lag antes de repetir live.

## 2026-08-22 19:21 - Subfase 3U - Cierre de la implementacion vigente

- objetivo intentado: comprobar si los defectos que motivaban `REHACER`
  seguian presentes en el visualizador activo;
- archivos modificados: solo documentacion de cierre;
- paquetes compilados: no aplica; no hubo cambios de codigo;
- resultado de build: se conserva el build final previo 3/3;
- pruebas Gazebo/replay: no se ejecuto una simulacion nueva; el grafo ya fue
  utilizado durante las pruebas integradas posteriores hasta 194;
- patrones de reduccion: no aplica; no se leyeron logs completos;
- evidencia positiva: primera conexion desde latest, reconexion por
  `Last-Event-ID`, `state_reset`, drenaje por frame, lifecycle secundario por
  `flow_id`, topologia 23/41 y contrato fuente 9/9 en 0.13 s;
- evidencia negativa o ausente: no se midio latencia backend-SSE-render ni se
  ejecuto una nueva prueba de saturacion; el buffer cliente sigue siendo
  acotado y best-effort;
- conclusion: `CONSEGUIDA`; el usuario confirma que el grafo web es muy bueno y
  funciona bien, y acepta no introducir mas cambios ahora;
- siguiente paso recomendado: comprobar aislamiento y carga dentro de 3V/3W,
  sin reabrir 3U salvo regresion observable.
