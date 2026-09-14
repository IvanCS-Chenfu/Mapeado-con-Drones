# Subfase 6P - Integracion y validacion final multidron

## Estado

PENDIENTE. Es el cierre de 6A-6O tras la migracion a barrido de fachada.

## Flujo final

```text
drone listo -> asignacion por intervalo pendiente -> inspeccion depth
            -> prefijo FREE -> D* -> trayectoria -> reserva -> ACTIVE
            -> terminal/STOP -> coverage lineal -> reencolado
```

## Escenarios obligatorios

1. D1/GT al fiducial 2 y barrido de una `MAP_SECTION` hasta una cara lateral.
2. Prefijo parcialmente FREE: ejecutar solo el tramo confirmado y reinspeccionar.
3. Dos drones: reservas y movimientos concurrentes sin serializacion global.
4. `TRACKING_RISK` durante mirada al objetivo y restauracion de fachada.
5. Fiducial nuevo/visto por `(drone_id,map_epoch)`.
6. Revision de pose KF: retirar/reintegrar FREE depth sin fantasmas.
7. Tres inspecciones fallidas: `TO_FINISH`, desasignacion y nueva tarea.
8. GUI: subROI, linea de coverage y trayectoria ACTIVE independientes.

## Metricas

Latencia de inspeccion y D*, puntos depth aceptados/rechazados, voxeles DDA,
longitud FREE confirmada, coverage, STOP causal, reservas, tiempos de action,
trafico y memoria de colas/buffers.

## Limites

No incluye nube densa global, ramas runtime, coverage volumetrico ni correccion
del optimizador fiducial de Fase 3.

## Criterio de exito

Los escenarios completan con GUI F7 y Gazebo, sin RViz2, sin destinos UNKNOWN,
sin depth automatico por KF, sin reservas o acciones huerfanas y con
documentacion/historial coherentes.
