# Resumen - Fase 6: misiones y navegacion autonoma multi-dron

## Estado

```text
PARCIAL: 6A-6C, 6E-6G y 6L-6M CONSEGUIDAS; 6D y 6H-6K PARCIALES; 6N, 6O y 6P pendientes
Preparacion documental: reconciliada con ambos ZIP el 2026-09-02
Ultima validacion: prueba 722, 2026-09-13
Historial: 6A-6M con builds, CTest, GUI F7, Gazebo y grafos web
```

La prueba 603 mostro la configuracion real, 12 subROIs sin asignar, el
registro de dos drones y ambos grafos web con GUI F7 y Gazebo, sin RViz2. La
revision visual humana fue correcta. El cierre posterior por interrupcion
controlada tuvo `SIM-EXIT-CODE 0`; no sustituye el veredicto de la prueba.

El bloque de mapa navegable, D* y FREE se da por cerrado conversacionalmente.
La entrega visual de FREE puede llegar despues de OCCUPIED y no formar una ruta
legible; se mantiene como limitacion conocida para reabrir solo si un bloque
posterior lo necesita. No altera el estado parcial de 6I, cuya trayectoria
continua reproducible permanece pendiente.

La prueba 678 valida dos ajustes posteriores: `OCCUPIED` sparse exige cuatro
MapPoints distintos con score `>= 0.2`, de forma reversible, y las rutas
ejecutables usan un mínimo de `8 s` por tramo con empalme C1 de `3 s`. D1/GT
completó fiducial 2 y coverage con `SIM-DONE success=true`; los STOP por
`occupied_or_inflated` permanecen como comportamiento pendiente de decisión,
no como resultado de estos ajustes.

Las pruebas 682/683 inauguran 6J/6K: `RESERVED` es una capa dinamica por
propietario, invisible como `OCCUPIED` en GUI. 682 encontró un interbloqueo
HOLD/FIFO real; 683 lo corrige reanudando localmente al dueño del HOLD antes de
la espera ajena. Se validaron commit, espera, replace, release y dos rutas
ACTIVE concurrentes. El corredor actual reserva la polilínea D* inflada; el
sampler curvo común de `lib_tray` sigue pendiente.

Las pruebas 717 y 721 validan 6L con franjas ORB solapadas de 75 % y tres
frames: la primera activa yaw horizontal y la segunda un sector vertical con
pitch local `-25 grados`, sin ruta/reserva de reorientacion en servidor. La
722 confirma visualmente el movimiento fisico del joint de camara. 6L queda
CONSEGUIDA; `VISUAL_RETREAT`, calibracion geometrica amplia y depth local
pertenecen a trabajo posterior, ahora identificado como 6N.

6H valida en la casa exterior vigente el filtro sparse, el indicador provisional
de frontera resuelta y la publicación de candidatos D* en GUI F7. El interior
`UNKNOWN` tras una pared `OCCUPIED` se considera inaccesible, no una rama. El
lifecycle de ramas queda explícitamente aplazado: se retomará cuando haya una
topología que presente una expansión conectada real, sin fabricar otra
simulación ni asumir edificios cerrados como único caso de uso.

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
- `VoxelMapWorker` mantiene por perfil una guia de macro-voxeles local; D* usa
  un corredor fino ampliable, pero la guia no puede aceptar ni rechazar rutas.
- La cola registra pops y obsoletos; estos ultimos no consumen expansiones. La
  telemetria detallada se conserva en logs, no en el grafo web ni GUI F7.
- Los planes son cortos por distancia/duracion, no por numero de waypoints.
- `TrajectoryPlan` se expresa en W y transporta todos los datos deterministas.
- El dron valida revision/alineamiento, convierte a O una sola vez y congela la
  ejecucion local ante optimizaciones globales.
- Los waypoints internos son estados dinamicos; objetivo inicial C2 y jerk
  acotado/medido.

## Seguridad, mapa y coordinacion

- Voxel map global incremental con `occupancy/free` separado de `coverage`.
- Evidencia reversible por procedencia: MP debil, depth endpoint ocupado,
  depth ray libre, trayectoria estimada realmente recorrida libre en su volumen
  fisico sin margen y KF como referencia/coverage, nunca obstaculo por si solo.
- La trayectoria se conserva relativa a su KF y se retira/reintegra cuando
  cambia `W_T_KF`; FREE de paso real gana a ocupacion sparse debil en la misma
  celda. El margen pertenece exclusivamente a D*.
- Depth se conserva relativo al KF; mover `W_T_KF` retira y reintegra.
- Depth local es autoridad inmediata y puede ordenar `STOP` sin permiso.
- `TRACKING_RISK` preventivo usa STOP y una reorientacion que evita el primer
  sector pobre; `VISUAL_RETREAT` es una posible mejora futura.
- Reservas espaciales, swept volume con bounding box orientada y margen del
  servidor; commit serial, reserva existente gana y reemplazo atomico. La capa
  `RESERVED` es dinamica, tiene owner, no altera `OCCUPIED` ni se dibuja como
  tal; cada dron evita reservas ajenas e ignora la suya.
- La curva comun de `lib_tray` se sampleara para reserva cada
  `reservation_sweep_sample_step_voxels=0.5` voxeles iniciales, parametro a
  medir. No se ampliara `TrajectoryPlan` con estado dinamico inicial mientras
  el siguiente despacho espere el terminal normal y conserve la guarda stale.
- Un conflicto intenta alternativa y, sin ella, deja `WAITING` con
  `waiting_reservation` para reintentar por release/cambio relevante, sin STOP.
  Tras STOP se mantiene `HOLD_RESERVATION` hasta reemplazo seguro; no hay
  release por timeout ante perdida de comunicacion.

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
6L  TRACKING_RISK, STOP y reorientacion preventiva
6M  observacion yaw/pitch/distancia
6N  depth local, evidencia voxel reversible y STOP inmediato
6O  GO_TO, ANCHOR_SUBMAP y fiduciales oportunistas
6P  integracion y cierre
```

Estado de 6G: `task_lib` contiene D* Lite 3D incremental y `task_server`
publica un `TrajectoryPlan` previsto por solicitud XYZ. La prueba 615 valido
un plan y un replan cortos en Gazebo con GUI F7; 6I aun debe convertirlos en
trayectoria fisica ejecutable. Desde la 625, `VoxelMapWorker` precalcula por
perfil fisico transitabilidad, conexiones 26 y costes base: 24 cambios raw se
actualizaron en 110.169 ms frente a 16.060 ms de la implementacion inicial.
La telemetria adicional del PlanningWorker paso
build/CTest y su verificacion live queda pendiente de una nueva solicitud ROS.

Estado del bloque 4: 6D incorpora evidencia voxel `OCCUPIED` reversible desde
sparse y `FREE` reversible por volumen fisico, ligado a KF; depth real queda
para Fase 8. 6I habilita ese FREE pero aun no genera la trayectoria continua.
La prueba 620 confirmo reintegracion de FREE por revisiones de KF; D1 rechazo
el objetivo `(-9,-4,1.0)` por estar ocupado o inflado. La 625 confirmo que la
misma clase de objetivo cercano ahora devuelve rechazo seguro inmediato
(`goal_occupied_or_inflated`, cero expansiones), pero aun falta la demostracion
visual de un desvio D* hacia un punto con clearance en mapa denso. 6E crea y
asigna 12 tareas regionales con autoridad GT/ORB explicita.
6F confirma localmente, sin ejecutar ni completar.

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
