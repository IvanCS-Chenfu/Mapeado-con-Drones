# Launches de `simulacion_dron`

## `multi_dron.launch.py`

Arranca Gazebo, el numero de drones definido en `config/sim_dron.yaml`,
wrappers, `global_map_server`, RViz2, bridge web y el helper que abre una
pestaña del grafo. Reenvia colas principal/secundaria, record, snapshots,
inyeccion drop, fiduciales, umbrales y parametros de grafo/refinamiento.

Referencia:

```text
simulacion_dron/launch/multi_dron.launch.py
rg -n "primary_queue|rawdb_|full_snapshot|debug_drop|fiducial_|pipeline_flow|sparse_global_rviz" \
  simulacion_dron/launch/multi_dron.launch.py
```

RViz2 usa `sparse_global_debug.rviz`, recibe `use_sim_time=true` y un entorno
sin rutas Snap/VS Code para evitar bibliotecas GUI incompatibles. El bridge
sirve la topologia 3O de 22 nodos/34 aristas en 8765.

Argumentos de rendimiento/operacion:

```text
launch_gazebo_gui=true
launch_mission_gui=true
launch_sparse_global_rviz=true
launch_pipeline_flow_visualizer=true
open_pipeline_flow_browser=true
drone_start_stagger_sec=8.0
orb_vocabulary_path=<ORBvoc_L5.txt>
```

Con `launch_gazebo_gui=false`, el launch inicia `gzserver` directamente. Cada
grupo de dron posterior al primero se envuelve en un `TimerAction` con retardo
`(indice-1)*drone_start_stagger_sec`; esto evita materializar varios
vocabularios y modelos en el mismo pico. El vocabulario L5 es el default de
este launch, pero puede sustituirse por el L6 completo mediante el argumento.

Perfiles validados:

```text
2 drones / desarrollo:
  launch ordinario con Gazebo GUI, RViz2, web y GUI de mision

3+ drones / escala o futuras fases dense:
  launch_gazebo_gui:=false
  launch_mission_gui:=false
  RViz2 y web solo durante el diagnostico que los necesite
```

La prueba 137 confirmo seis goals, tres anchors, 141 KFs activos y 7981 puntos
con tres drones y stagger 0/8/16 s. `config/sim_dron.yaml` se restauro despues
a dos drones y la prueba visual 138 verifico el estado normal.

## `f3f_replay.launch.py`

Inicia bridge, helper de navegador, RViz2 y servidor en replay, sin Gazebo,
wrappers ni GT live. Argumentos:

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
