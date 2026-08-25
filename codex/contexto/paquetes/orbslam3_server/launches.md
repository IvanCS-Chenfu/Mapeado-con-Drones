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

Tambien inicia el nodo independiente `fiducial_config_server.py`, que carga
`fiducial_objects_config` y ofrece `/global_mapping/get_fiducial_config`. El
parser puro `scripts/fiducial_config.py` valida familia `APRILTAG_36H11`,
`SUBPIX`, `IPPE_SQUARE`, umbral positivo e IDs/tamanos sin duplicados; una
lista vacia es valida y desactiva funcionalmente el detector.

`replay_debug.yaml` es opt-in mediante `replay_debug_config`; no se carga en
live. `config_dir` permite seleccionar el perfil del despliegue: el launch
directo usa la copia de `orbslam3_server`, mientras la simulación pasa la copia
equivalente de `simulacion_dron`.

Los argumentos `use_sim_time`, `drone_count` y `drone_namespace_base` son la
autoridad dinámica del despliegue y no se duplican en `runtime.yaml`.
Record/replay, inyección de fallo,
anchor sintético y umbrales fiduciales visuales admiten overrides opcionales con el
sentinel `__from_yaml__`; si no se proporcionan prevalece el YAML.

`runtime.yaml` declara `fiducial_pending_capacity_per_drone=10`. Limita por
dron el FIFO de batches visuales que aun no tienen KF raw; puede ajustarse sin
recompilar y las copias Servidor/Simulacion deben permanecer identicas.

`log_level` controla el nivel ROS del nodo y vale `info` por defecto. El launch
de simulacion lo establece en `error` cuando `fase3_logs_terminal=false`, de
modo que desaparece la telemetria `[F3*]` pero siguen visibles los errores
reales.

Con `rawdb_replay_path` no vacío, el nodo no crea subscriptions wrapper,
clientes ni timers snapshot. Reinyecta deltas y observaciones normalizadas por
la misma cola y backend; solo admite observaciones fiduciales visuales. El drop
one-shot y el anchor sintético son únicamente
herramientas explícitas de prueba y permanecen desactivados en el perfil
normal.

La publicación no tiene parámetros de topic o periodo: los topics son fijos y
solo `PrimaryWorker` construye/publica al terminar una entrada principal. El
launch configura el pipeline completo de covisibilidad, loops, fusión,
optimización y scoring vigente.

## Instalación y contrato

`orbslam3_server/CMakeLists.txt` instala `config/`, `launch/` y los scripts. El test
`simulacion_dron/test/test_global_map_config.py` exige que cada parámetro tenga
un único YAML propietario, que todos los `declare_parameter` estén cubiertos y
que las copias servidor/simulación sean idénticas mientras se trabaje solo con
simulación.

`debug_pipeline_flow_events=false` y `debug_architecture_telemetry=false` son
defaults standalone. El primero evita crear el publisher o construir payloads
de `pipeline_flow`; el segundo gobierna el canal ligero e independiente
`/system_architecture/activity`. Simulacion activa cada productor solo cuando
su visualizador master correspondiente esta habilitado.
