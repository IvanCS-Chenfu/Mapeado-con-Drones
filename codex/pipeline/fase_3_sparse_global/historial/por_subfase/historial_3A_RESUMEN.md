# Historial 3A - resumen

Leer este archivo antes de `historial_3A.md` cuando haya que hablar o trabajar
sobre `3A`.

## Estado vigente

`3A` esta `CONSEGUIDA`. Fue baseline del servidor monolitico antes de iniciar
la migracion controlada.

## Que se hizo

- Se creo una trayectoria multi-dron de referencia con ida/vuelta al fiducial 2.
- No se modifico codigo de paquetes.
- Se documento el comportamiento legacy que debia preservarse al crear el
  servidor nuevo de `3B`.

## Evidencia

- Build seleccionado: `BUILD-EXIT-CODE 0`.
- `prueba_1`: `SIM-DONE prueba=1 success=true`, `SIM-EXIT-CODE 0`.
- Recepcion de deltas `OrbMap` de `dron_1` y `dron_2`.
- Submapas identificados por `(drone_id, map_epoch)`.
- Fiduciales anclados y `/global_sparse_cloud` publicada por la ruta legacy.

## Aprendizajes

- El servidor monolitico mezclaba ROS, fiduciales, fused map, loops y
  optimizacion; no usarlo como arquitectura nueva.
- La optimizacion local legacy no demostro apply util en esta baseline.
- `gazebo exit code 255` tras `SIM-DONE success=true` queda como cleanup no
  bloqueante si no hay otro error grave.

## Detalle

`historial_3A.md`.
