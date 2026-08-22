# Historial 3U - resumen

## Estado vigente

`CONSEGUIDA` por auditoria, contrato web 9/9 y cierre visual del usuario.

## Capacidad actual

- grafo Cytoscape.js con `23` nodos y `41` aristas;
- delta/snapshot y cloud/KFs usan aristas distintas;
- colores por ingesta, poses, trabajo secundario, estado derivado y
  publicacion;
- primera conexion SSE desde el presente, reconexion mediante `Last-Event-ID`
  y `state_reset` para cursores caducados;
- drenaje por `requestAnimationFrame`, sin cola temporizada de 110 ms;
- camino secundario acumulado por `flow_id` hasta lifecycle `done`;
- tooltips de responsabilidad y payload;
- bridge ROS -> SSE local, acotado y descartable;
- fallo de browser/bridge sin efecto sobre mapa, RViz2 o backpressure.

## Evidencia

- captura desktop estatica no vacia;
- evento controlado `wrapper_server_delta` recibido por el bridge y expuesto en
  `/events`;
- `prueba_75/76`: bridge `READY`, escenario `success=true` y cierre limpio;
- `prueba_76`: `144` publicaciones con backlog secundario.

Las pruebas posteriores 160/161 y 191-194 reutilizaron el grafo integrado; el
usuario confirma finalmente que es muy bueno y funciona bien.

El diagnostico del 2026-08-09 queda como antecedente ya corregido:

- el navegador solo puede renderizar `9.09` eventos/s y puede acumular unos
  `44 s` de cola local;
- los pulsos duran `520 ms`, por lo que hasta unos cinco eventos no-task se
  solapan y encienden varios nodos a la vez;
- cada conexion SSE empieza con `sequence=0`; una reconexion reenvia hasta los
  `512` eventos conservados por el bridge y se comporta como replay;
- no habia telemetria de latencia backend/SSE/render.

La politica visual actual y su buffer acotado quedan aceptados. Las pruebas de
carga/reconexion de 3V/3W son regresion global y no reabren 3U salvo impacto
funcional o deterioro visual observado.

## Detalle

`historial_3U.md`.
