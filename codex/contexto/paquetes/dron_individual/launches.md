# Launches de `dron_individual`

## `launch/generar_dron.launch.py`

Lanza los nodos principales de un dron dentro del namespace que le ponga el launch superior.

Nodos lanzados:

| Ejecutable | Nombre | Rol |
|---|---|---|
| `gen_tray` | `gen_tray` | Action server de trayectoria |
| `control_calcular_fuerzas` | `control_calcular_fuerzas` | Control PD a fuerza/torque |
| `aplicar_fuerzas_dron` | `aplicar_fuerzas_dron` | Mezcla fuerza/torque a motores |

También incluye `orbslam_use.launch.py` si `activar_orbslam=true`.

Argumentos:

- `activar_orbslam`;
- `drone_id`;
- `drone_name`;
- `local_map_frame`;
- `orb_vocabulary_path`;
- `use_sim_time`.

Carga por nodo `config/trajectory.yaml`, `config/physical.yaml`,
`config/control.yaml` y `config/actuators.yaml`. Los booleanos operativos de
ORB se leen desde `config/vision.yaml` como booleanos YAML reales.

## `launch/orbslam_use.launch.py`

Lanza ORB-SLAM3 mono o estéreo y, con debug fiducial activo, un visualizador
ROS independiente.

Argumentos:

- `vocab`;
- `yaml_mono`;
- `yaml_stereo`;
- `rectify`;
- `use_sim_time`;
- `usar_estereo`;
- `drone_id`;
- `drone_name`;
- `local_map_frame`.
- `debug_fiducial_visualization`;
- `debug_fiducial_display_seconds`.

Los nodos mono, estereo y visualizador reciben el entorno saneado. Los nodos
ORB reciben `MALLOC_ARENA_MAX=2`. El entorno elimina
entradas `/snap/` y `/snapd/` de las rutas de bibliotecas/plugins y limpia las
variables Snap/VS Code que pueden contaminar HighGUI. El argumento `vocab`
es obligatorio en este launch y normalmente llega desde
`generar_dron.launch.py`.

Nodo estéreo actual:

```text
package='orbslam3'
executable='stereo'
name='orbslam3_stereo'
```

Con `debug_fiducial_visualization=true` tambien se lanza:

```text
package='orbslam3'
executable='fiducial_visualizer'
name='fiducial_visualizer'
```

El wrapper publica `orbslam/fiducial_debug/image` dentro del namespace del
dron y el visualizador lo consume con cola latest-only. El parametro
`debug_fiducial_display_seconds` se entrega solo al visualizador. Por tanto,
cerrar o matar su ventana no termina `orbslam3_stereo` ni detiene sus deltas.

Las configuraciones mono y estéreo fijan:

```yaml
loopClosing: 0
Camera.width: 480
Camera.height: 360
Camera.fps: 20.0
ORBextractor.nFeatures: 900
```

El estereo usa `fx=fy=286.02185016085167`, centro `(240.5,180.5)` y
`bf=16.303245459168547`, equivalentes a baseline 0.057 m. Estos valores deben
mantenerse sincronizados con `simulacion_dron/config/simulated_sensors.yaml` y
la camara Xacro.

`generar_dron.launch.py` y `simulacion_dron/multi_dron.launch.py` usan por
defecto el vocabulario completo instalado `ORBvoc.txt`. La ruta sigue siendo
configurable y L5 queda reservado a overrides deliberados:

```text
orb_vocabulary_path:=/ruta/a/ORBvoc.txt
```

El wrapper carga por defecto las calibraciones de este paquete. Con esa política
ORB-SLAM3 indexa los KFs en `KeyFrameDatabase` sin ejecutar detección,
corrección o merge de loop; `Tracking` y `LocalMapping` permanecen activos.

Remappings:

```text
camera/left  -> sensor/camara_izq/image_raw
camera/right -> sensor/camara_der/image_raw
```

Parámetros enviados:

- `use_sim_time`;
- `drone_id`;
- `drone_name`;
- `local_map_frame`;
- `use_corrected_keyframes=true`;
- `max_nearest_kf_distance_m=2.0`.

## Riesgos

- El paquete ejecutable del wrapper se llama `orbslam3`, no `orbslam3_ros2` en este launch.
- La calibración usada por este launch está en `dron_individual/config/orbslam/`, no en `simulacion_dron/config/orbslam/`.
- Si se cambian nombres de topics de cámara, actualizar remappings.
- El vocabulario completo se prepara fuera de `src/` mediante
  `codex/herramientas/bootstrap_orbvoc.sh` y se instala con el paquete.
