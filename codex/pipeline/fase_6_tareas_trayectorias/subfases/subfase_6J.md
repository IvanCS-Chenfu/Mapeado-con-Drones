# Subfase 6J - Monitor de trayectoria, reservas y GUI

## Estado

`PENDIENTE DE MIGRACION`.

La reserva y el monitor son el destino de los planes de 6I. Esta subfase no
puede conservar una semantica de trayectoria distinta de la nueva inspeccion
por secciones U.

## Objetivo

Desacoplar la ejecucion fisica de la planificacion y proteger un corredor ya
aceptado sin retener al worker que calculo D*.

## Contrato funcional

Al aceptar un `MOVE_AND_CAPTURE` o un prefijo `FREE`,
`ActiveTrajectoryMonitor` reserva antes de enviarlo al dron el volumen de este
durante toda la polilinea. Las reservas no
son `OCCUPIED` reales ni se confunden con evidencia del mapa: permiten ocupar
celdas `FREE` durante el vuelo y restauran su estado base al liberar la
reserva.

El monitor posee la reserva tras el `accepted` del dron. Cuando recibe
`started`, publica en GUI solo la trayectoria realmente activa. Se suscribe a
deltas de `VoxelMapBuilder` y compara exclusivamente IDs modificados con los
IDs del corredor reservado. Si aparece `OCCUPIED` o una reserva ajena en ese
corredor, solicita STOP. No se replantea ni escanea el mapa completo dentro
del monitor.

Al resultado terminal normal, aborto o STOP limpia trayectoria GUI y reservas.
Dos causas concurrentes de STOP comparten una generacion, acumulan causas y no
envian dos ordenes contradictorias. El resultado del movimiento se convierte
en un trabajo para `DepthIntegrationQueue` o para la continuacion acordada.

## Cambios requeridos

- Convertir la gestion actual de trayectoria activa en un worker por workflow
  y no en estado retenido por `TrajectoryPlanningWorker`.
- Mantener indices `workflow -> reserved_ids` y `voxel_id -> workflows`.
- Mostrar `RESERVED` como capa propia en GUI, diferenciada de `FREE` y
  `OCCUPIED`, y restaurarla al liberar.

## Exclusiones

- No decide destinos, no ejecuta D* y no integra evidence.
- No cancela una trayectoria sin ordenar STOP al dron.
- No decide coverage, no sustituye `MOVE_AND_CAPTURE` por una mirada y no
  altera la U ni la evidencia depth.

## Validacion y exito

Dos drones pueden ejecutar corredores distintos en paralelo. La ocupacion
nueva de una celda del corredor provoca un unico STOP, limpieza completa y
ninguna reserva residual. La GUI nunca sustituye una polilinea activa por una
trayectoria solo planificada.
