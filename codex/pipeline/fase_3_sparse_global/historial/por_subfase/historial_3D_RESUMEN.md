# Historial 3D - resumen

Leer este archivo antes de `historial_3D.md` cuando haya que hablar o trabajar
sobre 3D.

## Estado vigente

`PARCIAL`. La implementación y la validación técnica están completas. Replay 89
cumple el contrato incremental y live 88 cumple los criterios automáticos, pero
el usuario confirmó que Chrome abrió la página sin dibujar el grafo. La
corrección aislada ya está confirmada visualmente; se ha autorizado repetir la
prueba live para observar mensajes reales.

## Estado activo

- `SparseGlobalBackend` posee y coordina `RawMapDatabase` y `GlobalPoseStore`.
- Raw conserva la única copia ORB-SLAM3 y produce cambios precisos de pose.
- El store empieza vacío, omite submapas sin anchor y protege poses aceptadas.
- Un anchor captura el submapa raw de forma acotada y crea las poses world en un
  único commit.
- El servidor continúa siendo orquestador ROS/cola/worker/backpressure/replay y
  no contiene lógica de poses.
- El grafo tiene siete nodos/seis aristas; raw->poses solo se activa ante KFs
  nuevos, poses modificadas o invalidaciones.
- RViz2 continúa sin publishers globales por contrato.

## Evidencia vigente

- build final de los tres paquetes: exit 0;
- tests funcionales: raw 5/5, backend/poses 4/4, cola 4/4 y web 8/8 tras la
  corrección de apertura;
- prueba 87: intento replay parcial, porque el submapa anclado no recibió KFs
  posteriores;
- prueba 89: 262 entradas raw exactas, 188 KFs, 21659 MPs, un anchor, 101 poses
  activas y 43 commits;
- prueba 88: seis goals live correctos, ramas `SKIP` y `UNANCHORED`, web
  `ready/live`, shutdown drenado y cero publishers en nube/keyframes; criterio
  visual web fallido por página sin grafo;
- launch aislado posterior: readiness antes de Chrome, assets `no-store`, URL
  `fresh`, captura técnica 7/6 y confirmación humana conseguida.

## No repetir

- no duplicar el listado de anchors en raw ni hacer llamadas raw->store;
- no crear placeholders globales antes del anchor;
- no tratar cambios exclusivos de MPs/asociaciones/covisibilidad como cambios
  de pose;
- no degradar una pose aceptada durante reconciliación raw;
- no copiar en bloque el `GlobalPoseStore` de `legacy2`;
- no considerar prueba 87 suficiente para incrementalidad; la evidencia válida
  para ese criterio es prueba 89.

## Pendiente inmediato

La prueba 90 live terminó técnicamente correcta: 6/6 goals, browser preparado,
101 ramas `SKIP`, 87 `UNANCHORED`, cero anchors/applied y cola drenada tras 175
entradas. Falta incorporar la observación del usuario sobre grafo y RViz2 a esa
misma prueba y decidir el cierre de 3D. 3E seguirá necesitando preparación.

## Detalle

`historial_3D.md`.
