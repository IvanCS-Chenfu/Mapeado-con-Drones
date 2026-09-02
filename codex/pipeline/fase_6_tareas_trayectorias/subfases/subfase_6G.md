# Subfase 6G - Planificador global D* Lite y waypoints XYZ

## Estado

```text
sin hacer
```

## Dependencia

6D, 6E y dimensiones registradas por 6C.

## Objetivo tecnico

Implementar en `task_lib` D* Lite 3D reproducible e incremental sobre el mapa
voxel y simplificar su ruta a waypoints geometricos.

## Contrato

D* responde como llegar a un objetivo XYZ; no elige coverage, yaw, pitch ni la
trayectoria fisica. Usa 26-connectivity con longitudes reales y validacion de
diagonales para no cortar esquinas ocupadas.

```text
FREE normal | UNKNOWN transitable penalizado | OCCUPIED bloqueado
```

El footprint se aproxima mediante inflacion conservadora XYZ. La bounding box
orientada/swept volume se valida posteriormente. A mayor incertidumbre se usan
planes mas cortos y menor velocidad segun distancia visible, reaccion y frenado.

## Cambios requeridos

1. Implementar D* Lite puro con costes actualizables y estado incremental.
2. Consumir `MapChangeEvent` por revision/chunks, no reconstruir mapa completo.
3. Reparar solo cambios relevantes y medir plan inicial frente a repair.
4. Aplicar occupancy, clearance, UNKNOWN y costes configurables sin fijar pesos.
5. Simplificar ruta mediante LOS/shortcutting medido, sin confundirla con trayectoria.
6. Exponer corredor/chunks de la ruta para 6J/6K.
7. Actualizar grafo con queue, reason, current drone y latencias.

## Limites

No integrar yaw/pitch en el estado D*, no validar dinamica con la polilinea y no
interrumpir el planner desde otro thread. Al acabar se comparan revisiones.

## Pruebas

Mapa vacio, pared, pasillo, laberinto, UNKNOWN, obstaculo añadido/eliminado,
diagonales, cambio irrelevante/relevante y optimizacion. Medir determinismo,
tiempo inicial/repair y memoria. Integracion visible con GUI+Gazebo+grafo.

## Criterio de exito

Rutas XYZ seguras segun el modelo voxel, repairs incrementales correctos,
UNKNOWN navegable con cautela y metricas suficientes para ajustar parametros.
