# Pipeline Fase 6 - Misiones y navegacion autonoma multi-dron

## Contrato documental

Este documento consolida los dos ZIP de Fase 6. El complemento post-ZIP
prevalece ante contradicciones. Es una especificacion de trabajo, no evidencia
de implementacion ni autorizacion para ejecutar una subfase.

Conceptos retirados:

```text
flight_bounds como parametro o volumen independiente
tasks_per_level 4/8
tareas por puntos A-B-C
logica F6 dentro de orbslam3_server/orbslam3_multi
interfaces de mision dentro de orbslam3_msgs
secuencia artificial 6A-6T
```

## Objetivo

Ejecutar misiones multidron configuradas, repartir responsabilidades regionales
de mapeo, construir navegacion voxel incremental y mover drones de forma
continua, segura y coordinada en entornos parcialmente desconocidos. Fase 6 no
genera la nube densa global de Fase 8.

## Paquetes

```text
servidor/task_server       nodo ROS y workers de F6
servidor/task_lib          logica pura de mision, voxel, planning y reservas
servidor/mission_msgs      copia canonica de interfaces F6
dron/task_manager          nodo ROS por dron
dron/task_manager_lib      estado local, safety y decisiones tacticas
dron/mission_msgs          replica exacta controlada por Fase 2
dron/lib_tray              generacion fisica compartida/reproducible
```

`task_server` consume interfaces ROS publicas de `orbslam3_server`; no enlaza
`orbslam3_multi`. Los nodos son fronteras de comunicacion y las librerias no son
canales ocultos entre grupos.

## Configuracion de mision

El YAML de mision contiene semantica del objetivo, no waypoints ni pesos de
algoritmo:

```yaml
mission_id: mission_house
drones: [1, 2]
mapping_roi:
  frame_id: world
  min: [x_min, y_min, z_min]
  max: [x_max, y_max, z_max]
mapping_hysteresis: [hx, hy, hz]  # forma final validada por 6A
level_height: 2.0
```

Regla cerrada:

```text
hard_flight_volume = expand(mapping_roi, mapping_hysteresis)
```

No existe `flight_bounds`. El ROI es objetivo de coverage; el volumen expandido
es el limite fisico. La histeresis permite maniobra y observacion, pero no
expande indefinidamente la mision.

Los parametros experimentales viven en perfiles de navegacion del servidor:
`voxel_size`, pesos, thresholds, distancia de observacion, limites de plan,
margen de seguridad, sampling, STOP, riesgo visual y coverage.

## Geometria y ownership

`level_height` crea bandas verticales; el resto se incorpora a la ultima. No
hay barrera de ejecucion entre niveles.

Cada nivel crea cuatro responsabilidades regionales solapadas, basadas en los
lados AB, BC, CD y DA y extendidas hacia el interior. No son cuatro puntos ni
rutas perimetrales. Una `MAP_SECTION` impulsa coverage de una subROI base y de
las ramas accesibles que descubre.

Una rama se reclama solo tras cruzar un frontier y confirmar nuevo FREE. El
ownership es 3D, puede cruzar nivel/subROI y se conserva durante `PAUSED`. Si
dos entradas conectan la misma region se fusionan linajes: primer owner para
coverage detallada, segunda visita ligera para loops/covisibilidad. Si el owner
falla de forma prolongada, `TaskWorker` puede reasignar.

## Mapa voxel

`VoxelMapWorker` es el unico writer. Mantiene separados:

```text
occupancy/free evidence
coverage evidence
```

Evidencia:

- MP: superficie/ocupacion debil ponderable por score;
- endpoint depth: ocupacion fuerte;
- rayo depth: FREE;
- trayectoria estimada realmente recorrida: FREE fuerte;
- KF: referencia/coverage, nunca obstaculo por si solo.

Toda contribucion movible conserva procedencia y admite add/remove/move. Depth
se acumula como `LocalVoxelSubmap` relativo al KF; mover `W_T_KF` retira la
rasterizacion anterior y reintegra sin recalcular profundidad. Cada commit
emite `map_revision` y chunks/voxels/AABB afectados.

## Workers y concurrencia

```text
TaskWorker          lifecycle, asignacion, ownership y drones
VoxelMapWorker      mapa voxel reversible y MapChangeEvent
PlanningWorker      coverage, frontiers, D* Lite, vista y candidato
ReservationWorker  validacion final, reservas y reemplazo atomico
```

`PlanningWorker` y `ReservationWorker` empiezan con una instancia y cola
serial. Los cuatro subsistemas pueden trabajar en paralelo, pero cada estado
tiene un writer logico y las lecturas usan snapshots/revisiones o locks cortos.
No se usa un mutex global.

La `PlanningQueue` prioriza emergencia, plan proximo a agotarse y trabajo
normal; aplica anti-starvation y coalescing por dron. Un cambio de mapa nunca
modifica D* desde otro thread: al acabar el calculo se comparan revisiones y se
repara antes de reservar si afecta al corredor.

## Flujo normal

```text
TaskWorker asigna MAP_SECTION
  -> coverage/frontier selecciona target XYZ
  -> D* Lite genera/repara ruta XYZ
  -> view planner asigna yaw/pitch
  -> lib_tray genera trayectoria fisica
  -> validacion occupancy + swept volume
  -> ReservationWorker hace commit
  -> task_server envia TrajectoryPlan en W
  -> task_manager valida revision y transforma W->O una vez
  -> misma lib_tray reproduce la referencia
  -> dron_individual ejecuta
```

El servidor piensa globalmente; `task_manager` reacciona localmente; el
controlador solo ejecuta referencias y maniobras fisicas.

## D* Lite y coverage

D* Lite 3D usa 26-connectivity y no decide el objetivo perceptivo:

```text
FREE      coste normal
UNKNOWN   transitable con penalizacion
OCCUPIED  bloqueado
```

La inflacion conservadora aproxima el volumen del dron en XYZ. La validacion
final usa bounding box orientada por yaw y swept volume. Los pesos, resolucion,
distancia preferida y velocidad en incertidumbre se miden, no se inventan.

El planner de coverage agrupa frontiers alcanzables y usa un score sencillo de
ganancia, superficie, distancia, continuidad, riesgo y coste. `frontier
lineage` nace al confirmar expansion FREE, se hereda por la rama y se fusiona
cuando dos accesos conectan la misma componente.

Una `MAP_SECTION` termina cuando no quedan frontiers utiles alcanzables, las
superficies accesibles tienen coverage suficiente y sus ramas estan cerradas o
justificadamente no alcanzables. UNKNOWN detras de paredes no impide terminar;
una conectividad nueva puede reabrir region mientras la mision siga activa.

## TrajectoryPlan y frames

Los planes son cortos segun `max_trajectory_distance` y
`max_trajectory_duration`. No se transmiten miles de samples. Cada waypoint
interno es un estado dinamico reproducible: posicion, velocidad, aceleracion,
yaw/pitch y derivadas requeridas por el generador.

`TrajectoryPlan` viaja en W con IDs/revisiones de tarea, dron, plan,
trayectoria, epoch, mapa y alineamiento, ademas de waypoints, timings, limites y
version del generador. El dron comprueba el contexto W/O; ante incompatibilidad
reporta `PLAN_ALIGNMENT_MISMATCH` o equivalente y no convierte con otra
revision.

Servidor y dron reconstruyen exactamente la misma trayectoria. La conversion
W->O ocurre una vez para congelar continuidad fisica. Se exige C2 en posicion y
jerk acotado/medido, preservando modos legacy de `lib_tray`.

## Reservas, replanning y seguridad

Las reservas iniciales son espaciales, conservadoras y serializadas. La
reserva committed existente gana. Un nuevo plan conflictivo busca alternativa
o espera. Para un mismo dron, la reserva vieja sigue vigente mientras se valida
la nueva y el reemplazo es atomico.

Cada segmento conserva corredor voxel y revisiones. Un cambio solo revalida los
segmentos afectados; se intenta conservar prefix/suffix. Un start-state nuevo
regenera hasta el primer estado antiguo identico e inserta waypoints de enlace
si hace falta. Un handover normal no obliga a parar.

Depth local tiene prioridad inmediata:

```text
riesgo fisico -> cancelar plan -> STOP dinamico -> hover
              -> HOLD_RESERVATION -> actualizar mapa -> replan
```

`task_manager` no espera permiso del servidor. `dron_individual` frena desde el
estado actual sin salto instantaneo de referencia.

`TRACKING_RISK` precede a LOST y combina cantidad/distribucion de soporte,
movimiento/yaw/pitch previstos y tendencia. Activa `VISUAL_RETREAT` sobre la
trayectoria realmente recorrida hasta el ultimo estado visual estable. Si surge
riesgo depth durante retreat, `STOP > VISUAL_RETREAT`. LOST reutiliza recovery
de Fase 5; Fase 6 no crea otro mecanismo paralelo.

## Observacion y comportamientos

Tras D* Lite se decide yaw/pitch para observar superficies, mantener tracking y
refrescar laterales. `camera_pitch` forma parte de `TrajectoryPlan`; la ejecucion
del joint pertenece a `dron_individual`. La distancia preferida es coste suave,
no restriccion rigida.

`GO_TO`, `ANCHOR_SUBMAP` y fiduciales oportunistas usan el mismo pipeline de
planning, trayectoria y reservas. `GO_TO` tiene prioridad alta pendiente, pero
no preempta una tarea `RUNNING`. `ANCHOR_SUBMAP` pausa `MAP_SECTION`, conserva
ownership y la reanuda tras recuperar anclaje. Sin pose global fiable queda una
limitacion inicial de reserva durante recovery; no se inventa global lock.

## Grafo web y telemetria

Desde 6A existe un grafo web de Fase 6, desactivable y no funcionalmente
necesario. Crece con las subfases y muestra inicialmente:

```text
TaskWorker -> PlanningWorker -> ReservationWorker -> task_manager
     ^              ^                 |
     |         VoxelMapWorker <-------+
```

Publica eventos agregados correlacionables por `task_id`, `drone_id`,
`map_epoch`, `map_revision`, `plan_id`, `trajectory_id`, `reservation_id` y
`lineage_id`, ademas de tamaños de cola y latencias. No controla mision ni
justifica paralelizar workers sin medidas.

## Secuencia oficial

| ID | Salida principal |
|---|---|
| 6A | Paquetes, workers, configuracion y grafo web base |
| 6B | Cuatro subROIs solapadas y ownership regional/3D |
| 6C | `mission_msgs`, registro de drones y lifecycle |
| 6D | Mapa voxel reversible, occupancy/coverage y revisiones |
| 6E | Gestor/asignador global de tareas regionales |
| 6F | `task_manager`/`task_manager_lib` base por dron |
| 6G | D* Lite 3D incremental y waypoints geometricos |
| 6H | Frontiers, lineage, coverage y completion |
| 6I | Multi-waypoint reproducible y `TrajectoryPlan` W->O |
| 6J | Reservas espaciales, conflicto y HOLD |
| 6K | Replanning incremental, prefix/suffix y handover |
| 6L | TRACKING_RISK, STOP y VISUAL_RETREAT |
| 6M | Yaw/pitch/distancia y observacion lateral |
| 6N | GO_TO, ANCHOR_SUBMAP y fiducial oportunista |
| 6O | Integracion final multidron |

## Parametros no cerrados

Quedan `A MEDIR`: voxel size, pesos/thresholds de evidencia y coverage,
clustering, coste UNKNOWN, velocidad, distancia de superficie, horizonte de
plan, margen, sampling, dinamica STOP, retreat y riesgo visual. Antes de cerrar
interfaces se auditan topics/servicios finales de Fase 5, W/O, revisiones,
reference KF, culling, epoch y TF/pitch de 1J.

## Prueba final

Con Gazebo y GUI F7 abiertos, sin RViz como vista normal: varios drones deben
completar coverage accesible, cruzar ramas 3D, reparar cambios de mapa, evitar
conflictos, reproducir trayectorias servidor/dron, reaccionar a depth/riesgo
visual y ejecutar especiales sin GT funcional ni dependencia de Fase 8.
