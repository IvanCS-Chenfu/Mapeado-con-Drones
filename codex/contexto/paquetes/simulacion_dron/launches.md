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
fase3_rviz2=false
fase3_grafo_web=false
fase3_abrir_navegador_web=false
fase3_logs_terminal=false
debug_fiducial_visualization=false
debug_fiducial_display_seconds=5.0
debug_orb_control_state=false
```

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
```

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

## `f3f_replay.launch.py`

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
