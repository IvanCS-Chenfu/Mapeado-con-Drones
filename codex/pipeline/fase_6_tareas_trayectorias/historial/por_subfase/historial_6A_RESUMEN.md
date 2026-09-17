# Resumen historial 6A

La entrada de 2026-09-07 conserva evidencia de la arquitectura legacy. Para
el contrato nuevo de 6A, estado agregado: `PARCIAL` el 2026-09-16.

`WorkflowScheduler` implementa FIFO, deduplicacion y generaciones STOP con
CTest `8/8` de `task_server`. Aun no recibe el flujo real: los workers y la
retirada de `RunFacadeWorker` pertenecen a la continuacion de la migracion.
