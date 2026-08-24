# Launches - `orbslam3_server`

## Launch activo

```text
orbslam3_server/launch/global_orb_map_server.launch.py
  -> generate_launch_description / _launch_setup
  -> rg -n "CONFIG_FILES|config_dir|replay_debug_config|parameter_overrides" \
       orbslam3_server/launch/global_orb_map_server.launch.py
```

El launch carga por orden cinco YAML operativos desde `config/global_map/`:

```text
runtime.yaml
fiducials.yaml
optimization.yaml
loop_fusion.yaml
scoring.yaml
```

`replay_debug.yaml` es opt-in mediante `replay_debug_config`; no se carga en
live. `config_dir` permite seleccionar el perfil del despliegue: el launch
directo usa la copia de `orbslam3_server`, mientras la simulación pasa la copia
equivalente de `simulacion_dron`.

Los argumentos `use_sim_time`, `drone_count` y `drone_namespace_base` son la
autoridad dinámica del despliegue y no se duplican en `runtime.yaml`.
Record/replay, inyección de fallo,
anchor sintético y umbrales fiduciales admiten overrides opcionales con el
sentinel `__from_yaml__`; si no se proporcionan prevalece el YAML.

`log_level` controla el nivel ROS del nodo y vale `info` por defecto. El launch
de simulacion lo establece en `error` cuando `fase3_logs_terminal=false`, de
modo que desaparece la telemetria `[F3*]` pero siguen visibles los errores
reales.

Con `rawdb_replay_path` no vacío, el nodo no crea subscriptions wrapper/GT,
clientes ni timers snapshot. Reinyecta deltas y observaciones normalizadas por
la misma cola y backend. El drop one-shot y el anchor sintético son únicamente
herramientas explícitas de prueba y permanecen desactivados en el perfil
normal.

La publicación no tiene parámetros de topic o periodo: los topics son fijos y
solo `PrimaryWorker` construye/publica al terminar una entrada principal. El
launch configura el pipeline completo de covisibilidad, loops, fusión,
optimización y scoring vigente.

## Instalación y contrato

`orbslam3_server/CMakeLists.txt` instala `config/` y `launch/`. El test
`simulacion_dron/test/test_global_map_config.py` exige que cada parámetro tenga
un único YAML propietario, que todos los `declare_parameter` estén cubiertos y
que las copias servidor/simulación sean idénticas mientras se trabaje solo con
simulación.

`debug_pipeline_flow_events=false` y `debug_architecture_telemetry=false` son
defaults standalone. El primero evita crear el publisher o construir payloads
de `pipeline_flow`; el segundo gobierna el canal ligero e independiente
`/system_architecture/activity`. Simulacion activa cada productor solo cuando
su visualizador master correspondiente esta habilitado.
