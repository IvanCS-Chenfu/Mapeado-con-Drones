# Subfase 1E — Especificación de GT y cámaras

Este archivo complementa `subfase_1E.md`.

## Estado de la subfase

```text
realizado
```

## YAML obligatorio

Ruta:

```text
src/dron_individual/config/hardware.yaml
```

### Ground Truth

```yaml
sensores.ground_truth.publish_rate: 50.0   # Hz
```

Validaciones:

- número finito;
- `> 0` para frecuencia limitada;
- si se permite `<= 0` como “cada tick”, debe documentarse expresamente;
- la frecuencia observada se mide con tiempo de simulación.

### Cámara física

```yaml
fisico.camara.dim: [x, y, z]              # m
fisico.camara.color: "Yellow"
fisico.camara.masa: 0.0                   # kg; justificar si es cero
fisico.camara.matriz_inercia: [ixx, iyy, izz, ixy, ixz, iyz]
```

### Cámara simulada

```yaml
sensores.camara.porcentaje_estereo: 0.3
sensores.camara.publish_rate: 30.0         # Hz
sensores.camara.mostrar_gazebo: "false"   # string en la implementación actual
```

Semántica de `porcentaje_estereo`:

- `0.0`: una cámara `mono` centrada;
- distinto de `0.0`: dos cámaras `izq` y `der`;
- la separación lateral se calcula como una fracción del ancho utilizable del cuerpo;
- no es una calibración real ni un baseline medido.

Debe validarse un rango acordado, recomendado `[0.0, 1.0]`, y documentar el baseline geométrico resultante.

## Modelo GT

Tag Xacro esperado:

```xml
<plugin name="plugin_sensor_groundtrurh" filename="libplugin_sensor_groundtrurh.so">
  <link_name>cuerpo</link_name>
  <topic_pose>sensor/GT/pose</topic_pose>
  <topic_vel>sensor/GT/vel</topic_vel>
  <topic_acc>sensor/GT/acc</topic_acc>
  <publish_rate>...</publish_rate>
  <frame_id>world</frame_id>
  <frame_propio>false</frame_propio>
</plugin>
```

Interfaces por namespace:

| Topic | Tipo | Frame esperado | Fuente |
|---|---|---|---|
| `sensor/GT/pose` | `geometry_msgs/msg/PoseStamped` | `world` | `link_->WorldPose()` |
| `sensor/GT/vel` | `geometry_msgs/msg/TwistStamped` | `world` o link según `frame_propio` | velocidades Gazebo |
| `sensor/GT/acc` | `geometry_msgs/msg/AccelStamped` | igual que velocidad | derivada numérica |

Timestamps: `info.simTime`, convertidos a `builtin_interfaces/msg/Time`.

### Reglas del plugin GT

1. Resolver `link_name` al cargar.
2. No publicar si el link o publishers no son válidos.
3. Respetar `publish_rate` con tiempo simulado.
4. No calcular aceleración en la primera muestra.
5. Evitar división por `dt` casi cero.
6. Si `frame_propio=true`, rotar velocidad y aceleración a frame del link.
7. Mantener coherencia entre `header.frame_id` y valores publicados.
8. Emitir logs de carga con link, topics y frecuencia.

## Modelo de cámara

Cada cámara se representa como un link unido de forma fija al `cuerpo` y un sensor Gazebo `type="camera"`.

Configuración de referencia:

```text
horizontal_fov: 1.3962634 rad (~80°)
width: 640 px
height: 480 px
format: R8G8B8
near: 0.05 m
far: 50.0 m
```

Estos valores también son de simulación. Si se modifican, deben pasar a YAML o quedar documentados con su finalidad.

Plugin:

```xml
<plugin name="camera_controller_<name>" filename="libgazebo_ros_camera.so">
  <cameraName>camara_<name></cameraName>
  <imageTopicName>sensor/camara_<name>/image_raw</imageTopicName>
  <cameraInfoTopicName>sensor/camara_<name>/camera_info</cameraInfoTopicName>
  <frameName>camara_<name></frameName>
</plugin>
```

Topics esperados:

### Mono

```text
sensor/camara_mono/image_raw
sensor/camara_mono/camera_info
```

### Estéreo

```text
sensor/camara_izq/image_raw
sensor/camara_izq/camera_info
sensor/camara_der/image_raw
sensor/camara_der/camera_info
```

Todos quedan bajo el namespace del dron.

## Macros Xacro a localizar

```text
link_camara
joint_camara_mono
joint_camara_estereo
sensores_camara_porcentaje_estereo == 0.0
libgazebo_ros_camera.so
plugin_sensor_groundtrurh
```

## Cambios requeridos

### Ground Truth

1. Compilar e instalar `libplugin_sensor_groundtrurh.so`.
2. Integrarlo sobre `cuerpo`.
3. Publicar los tres topics relativos.
4. Usar tiempo Gazebo en headers.
5. Documentar que `gen_tray` y `control_calcular_fuerzas` consumirán pose/vel en `1G`.
6. Añadir una advertencia permanente: GT no puede permanecer como entrada funcional después de Fase 5.

### Cámaras

1. Crear links visuales/inerciales según YAML.
2. Posicionar mono en el centro o estéreo de forma simétrica.
3. Integrar `libgazebo_ros_camera.so`.
4. Publicar `image_raw` y `camera_info` relativos.
5. Mantener `frameName` único por cámara.
6. Asegurar que la frecuencia viene del YAML.
7. Verificar que `mostrar_gazebo` se interpreta correctamente pese a ser string en el baseline.
8. Documentar baseline geométrico y orientación de las cámaras.

## Riesgos y limitaciones

- aceleración ruidosa por derivación numérica;
- frame incorrecto al rotar solo parte de las variables;
- masa de cámara cero o inercia incoherente;
- estéreo sin calibración intrínseca/extrínseca real;
- `robotNamespace` absoluto que rompa el aislamiento;
- frecuencia solicitada distinta de la simulada;
- carga de RViz2 que reduzca rendimiento;
- dependencia temporal de GT que debe eliminarse en Fase 5.
