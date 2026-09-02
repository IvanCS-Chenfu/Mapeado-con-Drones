# Subfase 6J - Reservas espaciales y colisiones multi-dron

## Estado

```text
sin hacer
```

## Dependencia

6C, 6D, 6G y 6I.

## Objetivo tecnico

Garantizar que cada trayectoria ejecutada tenga una reserva global coherente
con su volumen fisico y que dos commits no compitan.

## Modelo cerrado

Baseline espacial, no espacio-temporal. `ReservationWorker` procesa una
solicitud cada vez. La reserva committed existente gana: el candidato busca
otra ruta o espera seguro.

La validacion reconstruye `TrajectoryPlan`, samplea la curva real, orienta la
bounding box por yaw e infla con el margen del servidor. D* usa inflacion XYZ
barata; la decision final usa swept volume.

Cada segmento conserva corredor voxel, revisiones de mapa/reserva y resultado.
Un cambio posterior solo revalida segmentos cuyo corredor se ve afectado.

Para sustituir el plan de un dron, la reserva vieja permanece mientras se
valida la nueva y el commit hace reemplazo atomico. Tras STOP se libera futuro
y se mantiene `HOLD_RESERVATION` para el volumen parado.

## Cambios requeridos

1. Implementar cola/worker serial y registro autoritativo de reservas.
2. Construir swept volume desde samples y footprint registrado + margen.
3. Detectar conflicto con drones en movimiento, HOLD y hard flight volume.
4. Cachear validacion por segmento/corredor/revisiones.
5. Revalidar cambios de mapa posteriores antes del commit sin bloquear voxel worker.
6. Implementar atomic replace y lifecycle complete/cancel/fail/replace.
7. Mantener dron incomunicado como presencia fisica; timeout no libera sin evidencia.
8. Publicar eventos/metricas agregadas de queue, validation, conflict y commit.

## Limites

No introducir time-space reservations ni paralelismo antes de medir. No reservar
solo waypoints ni hacer release-then-propose. No comprobar paredes mediante
geometria ad hoc fuera del voxel map.

## Pruebas

Cruce de dos planes, solicitudes simultaneas, tamaños distintos, yaw, HOLD,
atomic replace, mapa cambiado dentro/fuera del corredor, conflicto sin ruta y
watchdog. Medir queue/sampling/check/commit con GUI+Gazebo+grafo.

## Criterio de exito

No existen carreras, huecos de reserva ni trayectorias sin volumen validado; un
conflicto conserva la reserva previa y produce alternativa o espera segura.
