# Historial 3K - worker secundario vigente

## 2026-08-05 - Un worker priorizado; independencia temporal no cerrada

- objetivo intentado: sustituir el scheduling anterior por una unica tarea
  secundaria activa, sin interrumpirla, con fiduciales delante de loops
  pendientes y sin bloquear ingesta, trayectoria o publicacion;
- archivos modificados: `global_map_server.cpp`,
  `secondary_task_order.hpp`, launch del servidor y tests asociados;
- comportamiento implementado: dos colas FIFO por prioridad, un worker
  persistente, calculo principal de tareas sobre snapshots privados y final de
  tarea sin `publication_ack`; la captura y callbacks live aun usan una seccion
  critica demasiado amplia;
- admision: solo KFs nuevos/materiales con apariencia; limite duro `4096`, con
  metricas de cola y drops. El topic
  `/global_mapping/backpressure_active` permanece siempre `false`;
- paquetes compilados: `orbslam3_multi`, `orbslam3_server` y
  `simulacion_dron`; build correcto. El rebuild final de
  `orbslam3_multi orbslam3_server` tambien termino con codigo `0`;
- tests: `test_secondary_task_order`, `test_loop_optimization_task`,
  `test_global_pose_store_tail_anchor`, `test_fused_landmark_manager` y
  `test_covisibility_database`, todos `PASS`;
- `prueba_73`: el worker unico funciono, pero el limite inicial de cola
  descarto `2059` de `2741` admisiones solicitadas; conclusion `PARCIAL` y
  aumento posterior del limite;
- `prueba_74`: el escenario excedio el timeout por una tarea de loop muy larga
  y revelo ademas una formulacion de optimizacion incorrecta; conclusion
  `NO CONSEGUIDA`;
- `prueba_75`: scenario `success=true`, una sola tarea activa y orden
  `tarea actual -> fiducial pendiente -> loops pendientes`; `66` loops
  empiezan/terminan de `561` encolados y el ultimo start conserva `496`;
- `prueba_76`: scenario `success=true`, `84` loops empiezan/terminan de `489`,
  pico `429`, ultimo valor `414`, una sola tarea activa y `144`
  publicaciones;
- revision del 2026-08-09: `31/66` y `38/84` starts loop son pre-anchor. El
  callback mantiene el mutex hasta `12.829/17.881 s` despues del marcador raw
  y request->commit de RViz2 llega a `20.283/27.951 s`;
- evidencia negativa o ausente: la segunda ejecucion no produjo una tarea
  fiducial de error alto, por lo que esa prioridad se apoya en la prueba 75 y
  el test determinista;
- conclusion revisada: `PARCIAL`. Worker unico, prioridad y no interrupcion
  quedan conseguidos; independencia temporal, gating pre-anchor, snapshots y
  drenaje quedan pendientes;
- siguiente paso recomendado: corregir esos cuatro limites y repetir la misma
  prueba antes de usar `3V/3W` como cierre.
