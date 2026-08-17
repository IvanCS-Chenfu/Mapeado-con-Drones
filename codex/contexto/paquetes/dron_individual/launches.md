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

- `usar_veltrap`;
- `activar_orbslam`;
- `drone_id`;
- `drone_name`;
- `local_map_frame`;
- `orb_vocabulary_path`;
- `use_sim_time`.

Lee defaults desde:

- `config/tray_dron.yaml`;
- `config/vision.yaml`.

## `launch/orbslam_use.launch.py`

Lanza ORB-SLAM3 mono o estéreo.

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

Los nodos mono y estereo reciben `MALLOC_ARENA_MAX=2`. El argumento `vocab`
es obligatorio en este launch y normalmente llega desde
`generar_dron.launch.py`.

Nodo estéreo actual:

```text
package='orbslam3'
executable='stereo'
name='orbslam3_stereo'
```

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
mantenerse sincronizados con `config/hardware.yaml` y la camara Xacro.

`generar_dron.launch.py` conserva por defecto el vocabulario completo
`ORBvoc.txt`. `simulacion_dron/multi_dron.launch.py` reemplaza ese argumento por
`ORBvoc_L5.txt` para el perfil multi-dron. La ruta sigue siendo configurable:

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
- El vocabulario L5 reduce mucho la memoria, pero no sustituye una comparacion
  L6 en pruebas especificas de relocalizacion y loops.
