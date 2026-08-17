# Launches - orbslam3_server

## Launch activo

```text
orbslam3_server/launch/global_orb_map_server.launch.py
  -> generate_launch_description
  -> rg -n "DeclareLaunchArgument|rawdb_|fiducial_|body_camera" orbslam3_server/launch
```

Parámetros principales:

| Parámetro | Default |
|---|---:|
| `use_sim_time` | `true` |
| `drone_count` | `2` |
| `drone_namespace_base` | `dron` |
| `primary_queue_high_watermark` | `8` |
| `primary_queue_low_watermark` | `2` |
| `secondary_queue_high_watermark` | `64` |
| `secondary_queue_low_watermark` | `16` |
| `primary_worker_debug_delay_ms` | `0` |
| `rawdb_record_enabled` | `false` |
| `rawdb_record_path` | `/tmp/f3c_raw.record` |
| `rawdb_replay_path` | vacío |
| `rawdb_replay_entry_delay_ms` | `0` |
| `full_snapshot_enabled` | `true` |
| `full_snapshot_startup_delay_sec` | `35.0` |
| `full_snapshot_period_sec` | `35.0` |
| `debug_drop_one_delta_for_snapshot_test` | `false` |
| `debug_drop_delta_drone_id` | `1` |
| `fiducial_sim_enabled` | `true` |
| `fiducial_gt_max_dt_sec` | `1.0` |
| `fiducial_translation_threshold_m` | `0.35` |
| `fiducial_rotation_threshold_rad` | `0.35` |
| `fiducial_yaw_threshold_rad` | `0.25` |
| `pose_graph_control_vertex_ratio` | `0.30` |
| `pose_graph_endpoint_neighborhood_ratio` | `0.20` |
| `fiducial_max_correction_fraction_per_pass` | `1.0` |
| `fiducial_max_refinement_passes` | `4` |

Configuración fija expuesta por parámetros del nodo:

```text
fid1=(0, 9, 1), radio=2 m
fid2=(0,-9, 1), radio=2 m
body_T_camera translation=(0.10,0.03,0.03)
RPY=(0,-90,90), optical=true
```

Con `rawdb_replay_path` no vacío, el nodo no crea subscriptions wrapper ni GT;
no crea clientes ni timers snapshot y reinyecta deltas y observaciones
normalizadas por la misma cola y backend. El
retardo opcional entre entradas permite observar el flujo sin introducir un
retardo artificial en uso normal. El
anchor sintético heredado de 3D permanece disponible solo como opción explícita
de test y está desactivado en 3E.

El drop one-shot es exclusivamente una inyección de fallo de prueba: omite un
delta live del dron indicado y solicita inmediatamente un snapshot fresco. Está
desactivado por defecto y nunca se usa en replay.

La publicacion global no necesita argumentos de topic o periodo: los topics son
fijos y la construccion ocurre al final de una tarea delta. El launch configura
el worker secundario fiducial; loops y `DatabaseUpdateTask` aun no son
funcionales. Los launches previos estan congelados en `legacy2`.
