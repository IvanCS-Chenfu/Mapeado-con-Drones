# Comparativa de la configuración

## Intrínsecas de las dos cámaras

| Magnitud | `camara_izq` | `camara_der` | Resultado |
|---|---:|---:|---|
| Resolución | `480x360` | `480x360` | Coinciden |
| Modelo de distorsión | `plumb_bob` | `plumb_bob` | Coinciden |
| `fx` | `286.02185016085167` | `286.02185016085167` | Coinciden |
| `fy` | `286.02185016085167` | `286.02185016085167` | Coinciden |
| `cx` | `240.5` | `240.5` | Coinciden |
| `cy` | `180.5` | `180.5` | Coinciden |
| Distorsión `D` | cinco ceros | cinco ceros | Coinciden |
| Matriz `R` | identidad | identidad | Coinciden |
| Frecuencia de imagen | aproximadamente `20 Hz` | aproximadamente `20 Hz` | Coinciden nominalmente |

Los mensajes se obtuvieron de:

- `/dron_1/sensor/camara_izq/camera_info`
- `/dron_1/sensor/camara_der/camera_info`

Cada tópico tenía un único publicador `camera_controller_izq` o
`camera_controller_der`, con tipo `sensor_msgs/msg/CameraInfo`.

## Coherencia entre simulación y ORB-SLAM3

| Parámetro | `CameraInfo` real | `.xacro` + sensores | ORB-SLAM3 efectivo |
|---|---:|---:|---:|
| Anchura | `480` | `480` | `480` |
| Altura | `360` | `360` | `360` |
| `fx` | `286.02185016085167` | derivada de FOV | `286.02185016085167` |
| `fy` | `286.02185016085167` | derivada de FOV | `286.02185016085167` |
| `cx` | `240.5` | centro de imagen | `240.5` |
| `cy` | `180.5` | centro de imagen | `180.5` |
| Distorsión | cero | no se introduce | cero |
| Línea base | implícita en el par estéreo | `0.057 m` | `bf=16.303245459168547` |
| Frecuencia | aproximadamente `20 Hz` | `20 Hz` | `20 Hz` |

La igualdad `bf = fx * 0.057` se cumple dentro de la precisión decimal del
YAML. La configuración es, por tanto, internamente consistente para el par
estéreo utilizado en la simulación.

## Observación de frames

El `.xacro` declara `frameName` como
`${drone_name}/camera_${name}_optical_frame`, mientras que las dos muestras de
`CameraInfo` llevan `header.frame_id: cuerpo`. Esta diferencia queda registrada
como observación de la prueba. No se ha corregido porque esta actividad es
documental y no se ha solicitado modificar el proyecto.
