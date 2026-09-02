# Resumen - Fase 6: misiones y navegacion autonoma multi-dron

## Estado

```text
sin hacer
Preparacion documental: reconciliada con ambos ZIP el 2026-09-02
Autorizacion funcional: pendiente
Historial: vacio; no hay ejecuciones de Fase 6
```

## Autoridad documental

Este pipeline procede de:

1. `Fase_1J_y_Fase_6_reestructurada_detallada.zip`;
2. `Fase_6_complemento_post_zip_detallado.zip`.

El complemento es posterior y prevalece ante contradicciones. Las antiguas
subfases basadas en `tasks_per_level`, puntos A-B-C, `orbslam3_msgs` como
contrato de mision o implementacion dentro de `orbslam3_server` no son vigentes.

## Arquitectura cerrada

```text
Servidor: task_server -> task_lib
                         ^
                         | interfaces ROS publicas de mapa/pose
              orbslam3_server

Interfaces: mission_msgs

Dron: task_manager -> task_manager_lib -> dron_individual -> lib_tray
```

- `task_server` y `orbslam3_server` son paquetes/nodos independientes.
- `task_lib` no accede a `orbslam3_multi` ni a memoria interna del mapa.
- `mission_msgs` no se mezcla con `orbslam3_msgs`.
- Servidor y Dron usan la misma implementacion/version de `lib_tray`.
- Los nodos gestionan ROS; las librerias concentran logica testeable.

Workers iniciales de `task_server`:

```text
TaskWorker | VoxelMapWorker | PlanningWorker | ReservationWorker
```

Cada estado tiene un writer logico. Hay un `PlanningWorker` y un
`ReservationWorker`, ambos seriales internamente y paralelos entre subsistemas.

## Mision y volumen

- `mapping_roi`: volumen `world` que debe mapearse.
- `mapping_hysteresis`: extension de maniobra/observacion.
- `hard_flight_volume = expand(mapping_roi, mapping_hysteresis)`.
- No existe el parametro ni un tercer volumen `flight_bounds`.
- Se conserva `level_height`; el resto vertical se suma al ultimo nivel.
- Se elimina `tasks_per_level`.
- Cada nivel crea cuatro subROIs solapadas asociadas a los lados AB/BC/CD/DA.
- Una subROI es responsabilidad inicial, no ruta ni limite de movimiento.

Las ramas descubiertas por frontiers tienen ownership 3D y pueden cruzar
subROIs y niveles. Una segunda entrada a la misma region no repite coverage
detallada: puede realizar una pasada simple para loops/covisibilidad y salir por
el acceso mas conveniente.

## Navegacion

```text
coverage/frontiers -> objetivo XYZ
D* Lite 3D          -> ruta XYZ
view planner        -> yaw/pitch
lib_tray            -> trayectoria fisica
ReservationWorker   -> validacion y commit
task_manager        -> W->O una vez, reproduccion y ejecucion
```

- D* Lite usa 26-connectivity, `FREE` normal, `UNKNOWN` transitable penalizado
  y `OCCUPIED` bloqueado.
- Los planes son cortos por distancia/duracion, no por numero de waypoints.
- `TrajectoryPlan` se expresa en W y transporta todos los datos deterministas.
- El dron valida revision/alineamiento, convierte a O una sola vez y congela la
  ejecucion local ante optimizaciones globales.
- Los waypoints internos son estados dinamicos; objetivo inicial C2 y jerk
  acotado/medido.

## Seguridad, mapa y coordinacion

- Voxel map global incremental con `occupancy/free` separado de `coverage`.
- Evidencia reversible por procedencia: MP debil, depth endpoint ocupado,
  depth ray libre, trayectoria estimada realmente recorrida libre y KF como
  referencia/coverage, nunca obstaculo por si solo.
- Depth se conserva relativo al KF; mover `W_T_KF` retira y reintegra.
- Depth local es autoridad inmediata y puede ordenar `STOP` sin permiso.
- `TRACKING_RISK` preventivo usa `VISUAL_RETREAT`; `STOP` tiene prioridad.
- Reservas espaciales, swept volume con bounding box orientada y margen del
  servidor; commit serial, reserva existente gana y reemplazo atomico.
- Tras STOP se mantiene `HOLD_RESERVATION` hasta reemplazo seguro.

## Telemetria web

Desde 6A se crea un grafo web incremental inspirado en Fase 3, pero con la
topologia real de Fase 6. Muestra workers, colas, revisiones e IDs correlables;
crece al implementar cada subfase. Es opcional, no bloqueante y nunca controla
la mision.

## Secuencia vigente

```text
6A  arquitectura, paquetes y configuracion
6B  geometria de subROIs y ownership 3D
6C  mission_msgs, registro y lifecycle
6D  mapa voxel reversible
6E  gestor y asignador de tareas
6F  base autonoma de task_manager
6G  D* Lite y waypoints XYZ
6H  frontiers, coverage y ramas
6I  trayectorias multi-waypoint reproducibles
6J  reservas y colisiones multi-dron
6K  replanning incremental y handover
6L  TRACKING_RISK, STOP y VISUAL_RETREAT
6M  observacion yaw/pitch/distancia
6N  GO_TO, ANCHOR_SUBMAP y fiduciales oportunistas
6O  integracion y cierre
```

## Parametros a medir

No fijar sin pruebas: voxel size, pesos/umbrales de evidencia y coverage,
frontier clustering, distancia preferida, coste/velocidad en UNKNOWN, longitud
de planes, lead time, safety margin, sampling swept-volume, dinamica STOP,
limites VISUAL_RETREAT y thresholds de riesgo visual.

## Prueba final

Mision con N drones, GUI Fase 7 y Gazebo, sin RViz2 como vista normal. Debe
demostrar coverage accesible, ramas 3D, voxel reversible, D* incremental,
reproduccion W/O, reservas, STOP/HOLD, riesgo visual, behaviors especiales y
cierre sin colisiones, GT funcional, bucles infinitos ni dependencia de Fase 8.
