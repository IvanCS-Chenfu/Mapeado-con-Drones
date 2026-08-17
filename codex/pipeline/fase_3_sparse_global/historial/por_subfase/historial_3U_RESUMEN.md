# Historial 3U - resumen

## Estado vigente

`REHACER`. Transporte y topología se conservan, pero la UI debe eliminar replay
110 ms/SSE desde cero y aplicar prioridades, flujos completos, promoción de
optimizaciones y reset live reconstruible.

## Capacidad actual

- grafo Cytoscape.js con `16` componentes y contratos de datos separados;
- delta/snapshot y cloud/KFs usan aristas distintas;
- colores por ingesta, poses, trabajo secundario, estado derivado y
  publicacion;
- cola visual de hasta `400` eventos, consumida a uno cada `110 ms`;
- una sola etapa secundaria vigente por `task_id`;
- tooltips de responsabilidad y payload;
- bridge ROS -> SSE local, acotado y descartable;
- fallo de browser/bridge sin efecto sobre mapa, RViz2 o backpressure.

## Evidencia

- captura desktop estatica no vacia;
- evento controlado `wrapper_server_delta` recibido por el bridge y expuesto en
  `/events`;
- `prueba_75/76`: bridge `READY`, escenario `success=true` y cierre limpio;
- `prueba_76`: `144` publicaciones con backlog secundario.

La captura headless se tomo antes de que Chrome procesara `EventSource`, por lo
que no demuestra el pulso activo. La validacion humana solicitada sigue
pendiente y evita marcar `3U` como conseguida.

Diagnostico del 2026-08-09:

- el navegador solo puede renderizar `9.09` eventos/s y puede acumular unos
  `44 s` de cola local;
- los pulsos duran `520 ms`, por lo que hasta unos cinco eventos no-task se
  solapan y encienden varios nodos a la vez;
- cada conexion SSE empieza con `sequence=0`; una reconexion reenvia hasta los
  `512` eventos conservados por el bridge y se comporta como replay;
- no hay telemetria de latencia backend/SSE/render en los logs actuales.

## Detalle

`historial_3U.md`.
