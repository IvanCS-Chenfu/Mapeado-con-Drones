# Configuración de las cámaras en el modelo

## Fuentes

- `simulacion/simulacion_dron/urdf/dron_plugins.xacro`
- `simulacion/simulacion_dron/config/simulated_sensors.yaml`
- `simulacion/simulacion_dron/config/calibration_dron.yaml`

## Valores efectivos

El `.xacro` declara estos valores por defecto para los sensores de cámara:

| Parámetro | Valor por defecto |
|---|---:|
| `sensores_camara_porcentaje_estereo` | `0.3` |
| `sensores_camara_publish_rate` | `30.0 Hz` |
| `sensores_camara_width` | `640 px` |
| `sensores_camara_height` | `480 px` |
| `horizontal_fov` | `1.3962634 rad`, aproximadamente `80 grados` |
| `near` / `far` | `0.05 m` / `50.0 m` |

Durante la prueba, `simulated_sensors.yaml` sobrescribe los valores relevantes
con la configuración efectiva:

| Parámetro | Valor efectivo |
|---|---:|
| `sensores.camara.porcentaje_estereo` | `0.3` |
| `sensores.camara.publish_rate` | `20.0 Hz` |
| `sensores.camara.width` | `480 px` |
| `sensores.camara.height` | `360 px` |
| `sensores.camara.mostrar_gazebo` | `false` |

Por tanto, la resolución y la frecuencia observadas en ROS son `480x360` y
aproximadamente `20 Hz`, no los valores por defecto `640x480` y `30 Hz` del
`.xacro`.

## Geometría estéreo

- El `stereo_rig` se sitúa en `x=0.10 m`, `y=0`, `z=0.03 m` respecto a
  `cuerpo`.
- La cámara izquierda se coloca en `y=+0.0285 m`.
- La cámara derecha se coloca en `y=-0.0285 m`.
- La línea base resultante es `0.057 m`.
- La prueba se ejecutó con `camera_pitch_enabled=false`, por lo que el soporte
  estéreo quedó unido mediante una articulación fija.

El plugin publica, para cada cámara, los tópicos `image_raw` y `camera_info`.
El `.xacro` declara como `frameName` el frame óptico de cada cámara. En la
muestra real de `CameraInfo`, sin embargo, `header.frame_id` aparece como
`cuerpo`; se conserva este dato tal como fue medido y no se modifica el modelo.

## Transformación de calibración

`calibration_dron.yaml` declara:

```yaml
body_T_camera_x: 0.10
body_T_camera_y: 0.03
body_T_camera_z: 0.03
body_T_camera_roll_deg: -90.0
body_T_camera_pitch_deg: 0.0
body_T_camera_yaw_deg: -90.0
use_camera_optical_frame_convention: true
```

La configuración de `dron_individual/config/calibration.yaml` es equivalente.
