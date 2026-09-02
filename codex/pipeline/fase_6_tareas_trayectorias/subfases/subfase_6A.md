# Subfase 6A - Arquitectura, paquetes y configuracion de mision

## Estado

```text
sin hacer
```

## Dependencia

Fases 1J y 5 cerradas. Primera subfase de Fase 6.

## Objetivo tecnico

Crear la arquitectura compilable de F6 y validar una configuracion de mision,
sin implementar todavia voxel, planning ni movimiento autonomo.

## Contrato vigente

Paquetes nuevos fijados: `task_server`, `task_lib`, `task_manager`,
`task_manager_lib` y `mission_msgs`. `task_server` se comunica por ROS con
`orbslam3_server`; no enlaza `orbslam3_multi` ni se integra dentro de
`global_map_server.cpp`.

El YAML contiene `mission_id`, drones participantes, `mapping_roi` en `world`,
`mapping_hysteresis=[hx,hy,hz]` en metros no negativos y `level_height`. La
histeresis expande minimo y maximo en ambos sentidos de cada eje. No contiene
`flight_bounds`, `tasks_per_level`, waypoints ni pesos algoritmicos.

```text
hard_flight_volume = expand(mapping_roi, mapping_hysteresis)
```

Este volumen expandido es el limite de vuelo. El ROI sigue siendo el objetivo
de mapeo. El margen de seguridad es configuracion de servidor; dimensiones del
dron llegan por handshake en 6C.

Desde esta subfase se crea `mission_flow`, grafo web incremental dedicado a F6
con nodos de los cuatro workers. Es telemetria desactivable y nunca autoridad
funcional. `system_architecture` se amplia tambien con los paquetes y enlaces
reales, pero no sustituye el flujo interno de mision.

## Cambios requeridos

1. Auditar el workspace posterior a 1J/F5 y crear esqueletos de los cinco paquetes.
2. Fijar dependencias y ownership Server/Dron conforme a Fase 2.
3. Crear parser/validador puro de mision en `task_lib`.
4. Rechazar IDs duplicados, ROI no finito/nulo/invertido, histeresis negativa y `level_height<=0`.
5. Derivar `hard_flight_volume`; no aceptar un tercer volumen configurable.
6. Declarar arquitectura `TaskWorker`, `VoxelMapWorker`, `PlanningWorker`, `ReservationWorker` y writer logico por estado.
7. Crear `mission_flow` base y eventos `MISSION_CONFIG_LOADED/INVALID`
   agregados; ampliar la topologia estatica de `system_architecture`.

## Limites

No crear tareas, voxel map, D* Lite, contratos completos ni mover drones. No
usar GT. Los parametros numericos experimentales quedan `A MEDIR` en perfiles
de navegacion, no en el YAML de mision.

## Pruebas

- Unitarios de YAML valido y casos invalidos.
- Builds aislados de paquetes nuevos y guarda de dependencias.
- Integracion estacionaria con dos drones, Gazebo + GUI F7 + `mission_flow`,
  sin RViz: carga valida y rechazo controlado de una configuracion invalida.
- La prueba puede usar `phase5_navigation_source=gt` para control/estabilidad y
  visualizacion actuales; ningun paquete F6 recibe GT ni decide con el.
- Pausa manual para revisar GUI y ambos grafos antes del cierre automatico.

## Criterio de exito

Arquitectura separada y compilable, YAML instalado/validado, volumen derivado
correctamente y ambos grafos observables sin alterar runtime. No quedan
referencias funcionales a `flight_bounds` ni `tasks_per_level`, ni dependencia
de GT dentro de F6.

## Riesgos

Duplicar paquetes existentes, mezclar SLAM/mision o convertir telemetria en
dependencia. Ante nombres/interfaz F5 dudosos, detener y auditar antes de fijar.
