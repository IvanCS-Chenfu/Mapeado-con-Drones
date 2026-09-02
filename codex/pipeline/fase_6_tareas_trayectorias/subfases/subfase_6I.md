# Subfase 6I - Trayectorias multi-waypoint reproducibles

## Estado

```text
sin hacer
```

## Dependencia

6C, 6F, 6G y contratos finales de control/lib_tray tras 1J/F5.

## Objetivo tecnico

Extender `lib_tray`/`TrayAction` sin romper modos legacy y hacer que servidor y
dron reproduzcan exactamente una trayectoria continua desde `TrajectoryPlan`.

## Contrato

Un waypoint interno es estado dinamico: posicion, velocidad, aceleracion, yaw,
pitch y derivadas necesarias. Objetivo inicial: continuidad C2 en posicion y
jerk acotado/medido. Los planes son cortos por distancia/duracion y no envian
miles de samples.

`task_server` genera/valida en W con la misma implementacion de `lib_tray`. El
dron comprueba version/revision, transforma W->O una sola vez y reproduce la
misma curva. `camera_pitch` es referencia articular, no pose world.

Ante cambio de estado inicial, solo se regenera el prefijo hasta el primer
waypoint estable identico; se insertan estados intermedios si el enlace no es
viable. No se modifica el sufijo sin necesidad.

## Cambios requeridos

1. Crear generador multi-waypoint compatible con pol3/veltrap/elipse legacy.
2. Versionar perfil, limites, timings y estados necesarios en `mission_msgs`.
3. Implementar W->O para posicion, derivadas lineales y yaw; preservar tiempos/pitch.
4. Detectar mismatch de alineamiento/start state/generador antes de ejecutar.
5. Samplear en servidor para occupancy, footprint, corredor y reservas.
6. Implementar prefijo reparable e insercion de enlaces.
7. Probar igualdad muestra a muestra entre builds Server/Dron.

## Limites

No encadenar actions legacy con parada por waypoint, no transmitir samples como
plan y no convertir con una revision W/O distinta a la validada.

## Pruebas

Recta, curvas, C2/jerk, distintos estados iniciales, enlaces, limites,
compatibilidad legacy, sampling/corredor, W->O y paridad estricta servidor-dron.
Prueba fisica con GUI+Gazebo sin RViz y continuidad entre planes.

## Criterio de exito

La trayectoria reservada y ejecutada coincide dentro de tolerancia, no se para
en cada waypoint, conserva continuidad y rechaza contextos incompatibles.
