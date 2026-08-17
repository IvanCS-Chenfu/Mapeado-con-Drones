# Historial 3C - resumen

Leer este archivo antes de `historial_3C.md`.

## Estado vigente

`CONSEGUIDA`. La reimplementación activa de 3C contiene `RawMapDatabase`,
`PrimaryQueue`, un único `PrimaryWorker`, journal/record/replay, backpressure
8/2 y grafo web real hasta raw. RViz2 permanece sin publicaciones globales.

## Implementacion vigente

- deltas live de ambos wrappers se encolan por `arrival_id`;
- callbacks no hacen commits raw;
- el worker único termina cada tarea tras `InsertDelta()`;
- una entrada no se libera al worker hasta emitir su telemetría de enqueue;
- replay atraviesa la misma cola/worker;
- shutdown drena y registra `pending`, procesados y concurrencia máxima;
- el grafo muestra seis nodos/cinco aristas; full snapshots quedan para 3G.

## Evidencia de cierre

- build final y tests focalizados: correctos;
- prueba 85 live: `success=true`, seis goals correctos, gate WAIT 67.956 s,
  CLEAR al llegar a pending=2, 262 entradas y `max_active=1`;
- record 85: 3 submapas, 188 KFs, 21659 MPs;
- prueba 86 replay sin Gazebo: mismas 262 entradas/estadísticas exactas;
- bridge replay: secuencia 587 y eventos SSE reales queue->worker->rawDB;
- topic audit de prueba 83: cero publishers para nube y keyframes con RViz2
  suscrito;
- render final 1440x900 y 390x844 correcto.

Interpretación revisada: el backpressure de prueba 85 ya estaba activo antes
del primer movimiento. No fueron mensajes vacíos: durante la espera se
confirmaron deltas con 0 KFs nuevos y 143-181 MPs actualizados, principalmente
compatibles con estadísticas mutables de tracking. La web 3C no muestra esa
clasificación y visualmente todos esos casos parecen el mismo flujo completo.

## Intentos preservados

- prueba 79: `PARCIAL`; ruta correcta y record, pero no se provocó gate;
- prueba 81: `PARCIAL`; backpressure llegó demasiado tarde para bloquear otro
  movimiento y el delay quedó activo al cleanup;
- prueba 83: `NO CONSEGUIDA`; timeout del segundo movimiento con backpressure
  siempre false; guardó solo 19 entradas. Sirvió para auditar RViz2 y descubrir
  textos 3B residuales en la web.

## No repetir

- activar el delay dinámico después de que hayan cesado los deltas;
- dejar el delay distinto de cero al terminar;
- permitir que el worker despierte antes de emitir el enqueue visual;
- asumir que una captura SSE tardía reproduce eventos: la primera conexión
  empieza en el presente;
- interpretar cada pulso como un KF o geometría nuevos; puede transportar solo
  cambios estadísticos raw de MPs;
- usar los publishers espaciales antes de 3F.

## Siguiente paso

Preparar 3D y extender la misma `PrimaryTask` con `GlobalPoseStore`, sin crear
otra cola/worker principal.

Detalle cronológico: `historial_3C.md`.
