# Launches de `simulacion_dron`

`multi_dron.launch.py` expone `phase5_pose_metrics_enabled` y
`phase5_pose_metrics_output_dir`. Al activarlos lanza `phase5_pose_metrics` con
`use_sim_time=true`, numero/namespaces de drones y salida de artefactos propia.

## `multi_dron.launch.py`

Arranca Gazebo, el numero de drones definido en `config/sim_dron.yaml`,
wrappers y `global_map_server`. RViz2, bridge web, navegador y telemetria de
terminal son opcionales mediante `config/debug.yaml`. Pasa
`config/global_map/` al launch del servidor y solo sobrescribe identidad del
despliegue y opciones explicitas de record/log.

Desde 4B lanza tambien `fiducial_spawner.py` cuando
`spawn_fiducials:=true` (default). El nodo carga
`config/fiducial_objects.yaml` y `config/fiducial_rendering.yaml`, espera
`/spawn_entity` y publica `/fiducial_spawn_ready` solo tras crear los tres
objetos. El escenario puede esperar este topic antes de mover drones.

Desde 4D pasa `config/fiducial_objects.yaml` al `fiducial_config_server` y
propaga `debug_fiducial_visualization` y
`debug_fiducial_display_seconds` a todos los wrappers. Sus defaults son
`false` y `5.0`; el debug no altera la deteccion ni la publicacion del SLAM.

Desde 4F, `config/global_map/runtime.yaml` incluye
`fiducial_pending_capacity_per_drone=10`. Con
`debug_system_architecture_web=true` y
`debug_architecture_telemetry=true`, la arista de batches wrapper→Servidor se
activa en live; si telemetry es false el grafo permanece estatico.

Referencia:

```text
simulacion_dron/launch/multi_dron.launch.py
rg -n "fiducial_spawner|spawn_fiducials|global_map_config_dir|pipeline_flow" \
  simulacion_dron/launch/multi_dron.launch.py
```

RViz2 usa `sparse_global_debug.rviz`, recibe `use_sim_time=true` y un entorno
sin rutas Snap/VS Code para evitar bibliotecas GUI incompatibles. El bridge
sirve la topología 3Q vigente en el puerto 8765.

## Configuración global

`config/global_map/` replica los seis YAML del servidor por decisión de
despliegue: ejecutar simulación usa estos valores; ejecutar directamente el
servidor usa su propia copia. En la etapa actual ambas copias deben ser
idénticas y `test_global_map_config.py` lo comprueba automáticamente.

`loop_fusion.yaml` incluye los cuatro parametros
`loop_recent_loss_single_recovery_*`. El perfil de simulacion coincide con el
canonico: recuperacion 1/1 habilitada, `0.50 m`, `0.15 rad` y recorrido maximo
`2.0 m`; fuera de esa continuidad se aplica el apoyo adaptativo 2/4/6.

Perfil `config/debug.yaml` y argumentos homonimos:

```text
debug_fase_1=false
fase3_rviz2=false
fase3_grafo_web=false
fase3_abrir_navegador_web=false
fase3_logs_terminal=false
debug_fiducial_visualization=false
debug_fiducial_display_seconds=5.0
debug_orb_control_state=false
```

`debug_fase_1` llega al launch de cada dron, al generador URDF y al Xacro.
Apagado fija nivel `warn` en nodos de vuelo y bloquea `INFO` en plugins de
motores, GT y pitch; no afecta al runner ni a Fases 3-5.

`debug_orb_control_state` se propaga a cada `generar_dron.launch.py` y activa
los marcadores estructurados `[F5H-ORB-MEASUREMENT]`, `[F5H-ORB-PUBLISH]` y
`[F5H-CONTROL-DIAG]` sin cambiar fuente, trayectoria ni ganancias.
`orb_qualification_samples=20` tambien se propaga al mux; la etapa 1 de 5H lo
eleva para observar ORB mientras GT conserva el control.

Con `fase3_logs_terminal=false`, el servidor usa nivel ROS `ERROR`: se
suprimen los diagnosticos `[F3*]`, pero no los errores reales. El navegador
solo arranca si tambien esta activo `fase3_grafo_web`.

Otros argumentos de rendimiento/operacion:

```text
launch_gazebo_gui=true
launch_mission_gui=true
spawn_fiducials=true
drone_start_stagger_sec=8.0
orb_vocabulary_path=<ORBvoc.txt completo>
dron_spawn_override_enabled=false
dron_spawn_y=-10.8
dron_spawn_yaw_deg=90.0
phase6_execution_timing_factor=2.0
phase6_trajectory_waypoint_min_separation_m=1.0
phase6_trajectory_min_segment_duration_sec=8.0
phase6_waypoint_blend_sec=3.0
phase6_min_occupied_mappoints_per_voxel=4
phase6_extra_obstacle_clearance_voxels=2
phase6_voxel_worker_coalesce_ms=100
phase6_execute_facade_sweeps=false
phase6_facade_preferred_wall_distance_m=2.5
phase6_facade_preferred_displacement_m=2.0
phase6_facade_wall_distance_weight=1.0
phase6_facade_displacement_weight=1.0
phase6_facade_height_weight=1.0
phase6_facade_completion_ratio=0.99
phase6_facade_candidate_step_m=0.25
phase6_facade_min_free_prefix_m=1.0
phase6_facade_orientation_tolerance_deg=25.0
phase6_facade_max_inspection_failures=3
phase6_facade_worker_period_ms=250
phase6_reservation_sweep_sample_step_voxels=0.5
phase6_depth_inspection_enabled=false
phase6_depth_evidence_enabled=false
```

Cuando se habilita la ejecucion 6I, el launch entrega
`phase6_execution_timing_factor`,
`phase6_trajectory_waypoint_min_separation_m` y
`phase6_trajectory_min_segment_duration_sec`, y
`phase6_min_occupied_mappoints_per_voxel` a `task_server`, y
`phase6_waypoint_blend_sec` a cada `gen_tray`. Los parametros de ruta
depuran de forma segura una cadena D* y acotan inferiormente la duración de
cada tramo antes de construir `Pol3Waypoints`; el ultimo define la ventana de
empalme C1 a cada lado de una guía interior. La velocidad nominal es
`phase6_execution_nominal_velocity_mps=0.8` y define el tiempo de cada arista
como `factor * distancia / vel_max`. El ejecutor retemporiza solo una
desviacion inicial tolerable; si el inicio ya es obsoleto, rechaza y el servidor
replanifica.

Los argumentos `phase6_facade_*` controlan el barrido exterior: tres preferencias
bilaterales, paso de candidatos, prefijo FREE minimo, tolerancia de orientacion,
umbral de coverage lineal, reintentos y periodo del worker. No existe una meta
UNKNOWN ni un analizador de completion volumetrico. La inspeccion depth se
activa por separado y solo bajo demanda con
`phase6_depth_inspection_enabled`; `phase6_depth_evidence_enabled` permite al
servidor integrar su producto exclusivamente como FREE reversible.

`phase6_extra_obstacle_clearance_voxels` se suma al radio fisico ya voxelizado
del dron y sustituye al clearance metrico anterior.
`phase6_voxel_worker_coalesce_ms` agrupa cambios antes de un commit incremental.
`phase6_reservation_sweep_sample_step_voxels` controla la separación máxima de
las muestras que convierten la polilínea D* inflada en la reserva espacial;
actualmente no representa aún el sampler curvo final de `lib_tray`.

El override de spawn de 5B está desactivado por defecto. Cuando se habilita,
`multi_dron.launch.py` coloca X en `-1/+1` según dron y pasa Y/yaw al
`generador_URDF`; se limita a escenarios dirigidos.

En la prueba 201 se mantuvieron Gazebo GUI y RViz2, mientras
`pipeline_flow`, `system_architecture`, navegadores y telemetria arquitectonica
quedaron desactivados mediante sus argumentos de launch.

La prueba 210 completo la trayectoria tipica con ambos grafos y 68/68 matches.
La prueba 211 repitio solo el primer tramo con telemetria arquitectonica activa:
ambos grafos `mode=live` y 18/18 matches.

El escenario tipico de Fase 4 recorre las aristas del cuadrado ±10 y contiene
paradas en los cuatro puntos medios `(0,±10)` y `(±10,0)`. Su copia auxiliar
de ejecucion debe permanecer identica al YAML instalado.

Las seis transiciones que entran, permanecen o salen de `±180°` usan yaw
relativo: dron 2 `+90/0/+90` y dron 1 `-90/0/-90`. Posicion y resto de yaw
siguen absolutos. Esto evita que el controlador elija la vuelta equivalente de
270/360 grados al cruzar la discontinuidad angular.

Con `launch_gazebo_gui=false`, el launch inicia `gzserver` directamente. Cada
grupo de dron posterior al primero se envuelve en un `TimerAction` con retardo
`(indice-1)*drone_start_stagger_sec`; esto evita materializar varios
vocabularios y modelos en el mismo pico. El vocabulario completo es el default;
L5 solo se usa mediante un override explicito. El launch carga
`physical_dron.yaml`, `simulated_sensors.yaml` y la replica parcial
`actuators_dron.yaml`, nunca `hardware.yaml` de Dron.

Perfiles validados:

```text
2 drones / ejecucion normal:
  launch con Gazebo GUI y GUI de mision; observabilidad 3S desactivada

3+ drones / escala o futuras fases dense:
  launch_gazebo_gui:=false
  launch_mission_gui:=false
  RViz2 y web solo al habilitar sus booleanos de debug
```

La prueba 137 confirmo seis goals, tres anchors, 141 KFs activos y 7981 puntos
con tres drones y stagger 0/8/16 s. `config/sim_dron.yaml` se restauro despues
a dos drones y la prueba visual 138 verifico el estado normal.

Para diagnóstico puntual de 6I, `multi_dron.launch.py` expone
`phase6_debug_trajectory_diagnostics=false`. Al activarlo lo propaga a
`task_server` como `debug_trajectory_diagnostics` y a cada `gen_tray` y
`control_calcular_fuerzas` como `debug_f6i_trajectory`; no activa RViz, otra
GUI ni cambia las rutas D*.

## `f3f_replay.launch.py`

Para el laboratorio 321, `multi_dron.launch.py` propaga
`f5h_orb_control_override` a cada `generar_dron.launch.py`. Su default
`normal` no altera ejecuciones ordinarias; los valores GT son diagnosticos
temporales de Fase 5.

Inicia bridge, helper de navegador, RViz2 y servidor en replay, sin Gazebo,
wrappers ni GT live. Carga los YAML normales de simulación y añade
`replay_debug.yaml`. Argumentos:

```text
rawdb_replay_path                 obligatorio
rawdb_replay_entry_delay_ms      100 por defecto
pipeline_flow_port               8768
open_pipeline_flow_browser       true
launch_sparse_global_rviz        true
```

El launch sanea el entorno de RViz2 igual que `multi_dron.launch.py`. Se ejecuta
con `run_simulation.sh --without-gazebo`, que conserva healthcheck y reintentos
de Gazebo para las simulaciones normales y los omite explicitamente en replay.

## `f3e_replay.launch.py`

Reinyecta records v1/v2/v3 con observaciones fiduciales, sin Gazebo ni GT live.
Expone los tres umbrales de error para replay normal o forzado sin modificar
codigo. La prueba 146 reprodujo el record v3 de la live 145 con 496 entradas,
44 tareas, 30 commits y 14 `STALE`.

## Escenarios 3F

```text
tray_prueba_92.yaml -> espera 30 s mientras termina replay
tray_prueba_93.yaml -> tracking; ambos a fiducial 2; espera anchors;
                      ambos a x=-8; observacion final 30 s
```

## Escenarios 3G

```text
tray_prueba_98.yaml -> tracking; ambos a fiducial 2; espera snapshots/anchors;
                      ambos a x=-8; observacion visual final
tray_prueba_99.yaml -> espera para replay delta-only sin Gazebo
tray_prueba_137.yaml -> tres drones al fiducial 2 y avance paralelo headless
tray_prueba_139.yaml -> dos drones: fiducial 2, x=-8 y regreso; observacion
                       prolongada de RViz2 y grafo web
```

## `pipeline_flow_only.launch.py`

Ruta de diagnostico visual aislada: bridge y helper de navegador, sin Gazebo,
drones, servidor global ni RViz2.
## Parametros 6L/6M

`multi_dron.launch.py` expone `phase6_visual_risk_enabled`,
`phase6_debug_visual_risk_display`,
`phase6_visual_risk_empty_region_fraction=0.75`,
`phase6_visual_risk_persistence_frames` y
`phase6_visual_risk_reorientation_grace_sec=6.0`, junto a
`phase6_visual_risk_reorientation_step_deg=25.0`. La fraccion crea las cuatro
franjas direccionales solapadas de la evidencia ORB; el riesgo exige cero
inliers en la franja hacia la que se mueve el dron.

## Calidad depth 6N

`multi_dron.launch.py` expone
`phase6_depth_max_disparity_gradient_px_per_pixel=2.0`,
`phase6_depth_min_texture_gradient=8.0` y
`phase6_depth_texture_window_radius_px=2` y los reenvia a cada worker
estereo. El primero limita discontinuidades de disparidad; los dos ultimos
exigen textura local suficiente en la imagen izquierda rectificada. Se aplican
solo cuando `CaptureDepth` procesa una peticion: no existe calculo automatico
por KF. Una muestra descartada no forma parte de `DenseKFObservation`, de los
rayos FREE derivados ni del calculo de proximidad de esa captura. Depth no
genera endpoints OCCUPIED. Son umbrales experimentales independientes de
`phase6_depth_min_confidence`, que el servidor aplica solo a observaciones ya
filtradas.
