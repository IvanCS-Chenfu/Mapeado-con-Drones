# Subfase 6O - Integracion y validacion final multidron

## Estado

```text
sin hacer
```

## Dependencia

6A-6N conseguidas y GUI F7 integrada hasta las vistas necesarias.

## Objetivo tecnico

Demostrar Fase 6 completa como sistema autonomo multi-dron y cerrar parametros
experimentales mediante evidencia, no por conveniencia.

## Escenarios obligatorios

1. Exterior/interior con varios drones, cuatro subROIs por nivel y coverage.
2. Rama 3D con entrada unica; puerta cerrada sin busqueda infinita.
3. Dos entradas a misma region: merge, pasada ligera y salida conveniente.
4. MP/KF/depth movido: retirar/reintegrar voxel sin fantasmas.
5. Obstaculo global futuro: repair incremental y handover sin parada si hay margen.
6. Depth inesperado: STOP, hover, HOLD, map update y replan.
7. Start-state mismatch: reparar solo prefijo e insertar enlaces.
8. TRACKING_RISK antes de LOST, VISUAL_RETREAT y alternativa.
9. LOST real integrado con F5 y ANCHOR_SUBMAP con PAUSED.
10. Yaw/pitch y observacion lateral sobre superficies utiles.
11. Tamaños distintos y conflictos/reservas seriales.
12. GO_TO pendiente sin preemptar RUNNING y fiducial oportunista.
13. Cambios de mapa que afectan/no afectan corredor y queue coalesced.

## Metricas

D* inicial/repair, coverage target, lib_tray, sampling, collision check, queue,
commit, plans/replans, segments reused, STOP/braking, risk/retreat/lost,
distancia a superficies, occupancy/coverage, conflictos, memoria y trafico
`mission_msgs`. El grafo web debe mostrar workers/colas/revisiones en vivo.

## Pruebas

Todas las integraciones se realizan con Gazebo y GUI F7 abiertos; RViz solo si
se acuerda como debug auxiliar. El scenario runner prepara condiciones, no
precalcula la autonomia. Logs completos se reducen antes de leer y cada intento
se conserva cronologicamente.

## Criterio de exito

La mision termina cubriendo lo accesible sin colisiones, reservas inconsistentes,
voxel fantasma, paradas por waypoint, divergencia servidor/dron, busquedas
infinitas, GT funcional ni dependencia de Fase 8. GUI y grafo son observadores:
cerrarlos no afecta mision/control.

## Criterio de fallo

Cualquier tarea cerrada falsamente, region accesible abandonada, reserva
huerfana, STOP tardio, trayectoria no reproducible o dependencia de GT impide
cerrar. El resultado final distingue `IMPLEMENTADO`, `PROBADO` y `CONSEGUIDO`.
