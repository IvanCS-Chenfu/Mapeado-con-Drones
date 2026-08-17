# Historial 3M - resumen

## Estado vigente

`CONSEGUIDA` el 2026-08-15. `CovisibilityDatabase` y la
`DatabaseUpdateTask` MEDIA estan integradas en el unico worker secundario.

## Estado actual

- Una tarea MEDIA por `ChangeSet` prepara fuera de lock y compromete un patch
  canonico, versionado e idempotente.
- Al terminar encola una `LoopTask` BAJA por KF elegible; si no cambia
  covisibilidad, las BAJAS se crean directamente.
- La base conserva fuente, soporte, pose relativa medida/current y no participa
  aun en la optimizacion fiducial.
- Snapshots sin cambio material no crean ruido.

## Evidencia vigente

- build final 3/3; regresion 53/53 C++ y 9/9 web;
- replay 152 `PARCIAL`: backlog del consumidor loop, no de la base;
- replay 153 `CONSEGUIDA`: 806 secundarias, `pending=0`, cero hard;
- live 154: 371 principales concurrentes y cierre secundario limpio.

## No repetir

No reintroducir importacion sin patches ni commits largos bajo lock. La futura
fuente geometrica server solo se compromete tras la fusión de 3P.

Detalle: `historial_3M.md`.
