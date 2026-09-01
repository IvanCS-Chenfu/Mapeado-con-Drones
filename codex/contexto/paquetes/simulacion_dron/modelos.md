# Modelos, URDF/Xacro y plugins de `simulacion_dron`

## URDF/Xacro

### `urdf/dron.xacro`

Modelo base del dron:

- cuerpo central;
- brazos;
- motores;
- materiales;
- macros para 4, 6 u 8 brazos.

En el perfil multi-dron vigente, `physical_dron.yaml` aporta masas e inercias
por enlace y `actuators_dron.yaml` fija brazos de `0.25 m` a 45 grados. El
modelo de cuatro brazos suma 1.4 kg: cuerpo de 1.0 kg, cuatro brazos de 0.05 kg
y cuatro motores de 0.05 kg. Los centros de los brazos quedan a radio 0.125 m
y los motores a radio 0.25 m y z 0.015 m. Componiendo los tensores declarados
con ejes paralelos respecto al centro de masas se obtiene aproximadamente
`J_body=diag(0.00803107,0.00803107,0.015805) kg*m^2`; los productos de inercia
se cancelan por simetria. No confundir esta J compuesta con la J de cada enlace
ni con el antiguo `fisico.total.matriz_inercia=diag(1e-4)` del controlador.

### `urdf/dron_plugins.xacro`

Modelo extendido con:

- plugins de motores;
- plugin de ground truth;
- cámara mono/estéreo;
- parámetros de sensores.

Usa parámetros procedentes de:

- `dron_individual/config/hardware.yaml`;
- `simulacion_dron/config/sim_dron.yaml`.

Las camaras estereo se fijan a `stereo_rig`, y este se une a `cuerpo` mediante
`stereo_pitch_joint`; permanecen rigidas entre si y desplazadas a ambos lados
en Y. Con `camera_pitch_enabled=true` el joint es revolute y se carga el servo;
con `false` es fixed y el plugin no se carga. En neutral miran hacia el frente del body. Las
imagenes procesadas por OpenCV/ORB usan frame optico (X derecha, Y abajo, Z
frente). La extrinseca optica `B_T_C` correspondiente tiene rotacion
`RPY=(-90,0,-90)` bajo `Rz*Ry*Rx`, que es la configuracion vigente en las tres
copias `calibration*.yaml`. La rotacion historica `RPY=(0,-90,90)` era su
inversa; el flag `use_camera_optical_frame_convention=true` no cambia por si
solo el wrapper.

Auditoria previa 1J: el modelo no contiene `stereo_rig` ni joint movil. El
generador entrega el URDF directamente a `/spawn_entity`; el despliegue actual
no lanza `robot_state_publisher`, no publica `robot_description` para ese nodo
y no se ha localizado un productor de `joint_states`. Por tanto, añadir solo
un joint al Xacro no proporciona una TF dinamica consumible por F5.

Implementacion 1J en validacion: `dron_plugins.xacro` contiene
`stereo_rig` y `stereo_pitch_joint`; las camaras conservan 0.057 m de baseline
y el plugin `plugin_camera_pitch.cpp` recibe `camera_pitch/command`, publica
`camera_pitch/joint_states` y la TF body-camara con stamp de simulacion. El
runtime 361 produjo NaN en neutral. El ajuste fisico posterior elimina esos
valores no finitos en 362 y permite alcanzar `+30 deg` con unos `0.40 deg` de
error, pero queda una oscilacion de velocidad aproximada entre `-0.052` y
`+0.126 rad/s`; el modelo aun no satisface el criterio de reposo.

En el intento 363 se redujo la friccion del joint a `0.00005 Nm`, se aumento
el damping a `0.01 Nms/rad` y `kd` a `0.004`. El joint sigue finito y alcanza
la tolerancia angular, pero persisten impulsos alternos de velocidad aun cuando
la posicion tardia apenas cambia. El siguiente cambio candidato es filtrar la
velocidad usada por el controlador y por el criterio de asentamiento.

Implementacion vigente tras 364: filtro paso bajo configurable
`velocity_filter_tau_sec=0.05`; D y `JointState.velocity` usan la velocidad
filtrada, y `[1J-PITCH-STATE]` conserva tambien `velocity_raw`. La puerta
aislada supera `+30 deg` y retorno a cero dentro de `1 deg` y `0.03 rad/s`.
La bateria 365 valida `+30`, `-30`, saturacion `+90 -> +70` y retorno; 366
valida movimiento del dron con pitch no neutral bajo GT_FORCED.

Los plugins de camara usan frame IDs namespaced
`${drone_name}/camera_{izq,der}_optical_frame`. El link fisico mira por `+X` y
debe distinguirse del frame optico OpenCV que mira por `+Z`.

La posicion actual sale de dimensiones del modelo y queda aproximadamente en
`(0.0975, +/-0.0285, 0.03) m`; la calibracion canonica usa
`(0.10, 0.03, 0.03) m` para la izquierda y baseline 0.057 m. Este desfase
pequeno queda resuelto en la TF dinamica autoritaria, que usa
`(0.10,0.0285,0.03) m` para la izquierda. `physical_dron.yaml` fija cada
camara en `1e-5 kg`; `stereo_rig` reutiliza esa masa e inercia configurables.
El conjunto añade `3e-5 kg`, masa positiva para el joint pero despreciable
frente a los `1.4 kg` del dron. La configuracion anterior sumaba `0.04 kg` a
`0.10 m` del cuerpo y generaba `~0.0392 Nm` no modelados por el control; 370
reprodujo la deriva y 370R2 valido la correccion.

Referencia:

```text
simulacion/simulacion_dron/urdf/dron_plugins.xacro
  -> joint_camara_mono / joint_camara_estereo
  -> rg -n "joint_camara_(mono|estereo)|origin xyz"
```

Para masa e inercia:

```text
simulacion/simulacion_dron/urdf/dron.xacro
  -> brazo_total / joint_cuerpo_brazo / joint_brazo_motor
simulacion/simulacion_dron/config/physical_dron.yaml
simulacion/simulacion_dron/config/actuators_dron.yaml
```

## Generación/spawn

### `src/fiducials/fiducial_spawner.py`

Genera en runtime 15 texturas AprilTag, verifica cada una mediante OpenCV,
construye un SDF por objeto y usa `/spawn_entity`. Los tres modelos baseline
son cajas estaticas y colisionables de 0.40 m, con tags de 0.30 m en cinco
caras y poses `(0,+8.5,1)`, `(0,-8.5,1)` y `(+8.5,0,1)`.

Referencia:

```text
simulacion/simulacion_dron/src/fiducials/fiducial_spawner.py
rg -n "validate_contract|generate_assets|build_model_sdf|spawn_main" simulacion/simulacion_dron/src/fiducials/fiducial_spawner.py
```

`spawn_main` conserva vivo el publicador transient-local tras readiness y
trata `KeyboardInterrupt`/`ExternalShutdownException` como cierre normal.

### `src/generar_URDF/generador_URDF.cpp`

Nodo:

```text
generador_URDF
```

Rol:
- lee parámetros físicos y de simulación;
- declara y lee `sensores.camara.width/height` con fallback 640x480;
- reenvia frecuencia, ancho y alto de camara como argumentos Xacro;
- ejecuta/genera descripción URDF/Xacro;
- llama al servicio `/spawn_entity` de Gazebo;
- spawnea el dron en el mundo.

Para pruebas dirigidas declara `dron.spawn_override_enabled` (default false),
`dron.spawn_x`, `dron.spawn_y` y `dron.spawn_yaw_deg`. Si el override está
desactivado conserva el spawn aleatorio del `spawn_box`; 5B lo usa para
inicializar ambos drones frente al fiducial y provocar después una pérdida por
giro real, sin blackout ni cambios en tracking.

## Plugins Gazebo

### `plugin_actuar_motores.cpp`

Plugin de Gazebo que se engancha al modelo y aplica fuerzas a los enlaces de motor.

Su mensaje informativo de suscripcion solo se emite con `debug_fase_1=true`.

Entradas ROS:

- `motor/arr_iz`
- `motor/ab_iz`
- `motor/ab_der`
- `motor/arr_der`
- o variantes para 6/8 motores según Xacro.

Uso:
- recibe fuerzas calculadas por `aplicar_fuerzas_dron`;
- aplica fuerzas y torques en Gazebo.

En cold start, `last_forces_` y `last_torques_` se inicializan
explícitamente a cero. `OnUpdate` reaplica esos valores en cada tick y
`OnWrench` los sustituye al recibir una orden: la semantica física es ZOH y
el cero previo al primer comando esta demostrado.

### `plugin_sensor_groundtrurh.cpp`

Plugin de Gazebo que publica estado real simulado del cuerpo.

Sus mensajes informativos de topics/rate solo se emiten con
`debug_fase_1=true`; la publicacion no depende del flag.

Salidas:

| Topic | Tipo |
|---|---|
| `sensor/GT/pose` | `geometry_msgs/msg/PoseStamped` |
| `sensor/GT/vel` | `geometry_msgs/msg/TwistStamped` |
| `sensor/GT/acc` | `geometry_msgs/msg/AccelStamped` |

Uso permitido:
- control simulado;
- fiducial simulado;
- debug.

Uso prohibido:
- pose final sin GT;
- construcción de mapa final;
- fused score;
- dense mapping final.

## Cámara

Parámetros en `hardware.yaml`:

- `sensores.camara.porcentaje_estereo`;
- `sensores.camara.publish_rate`;
- `sensores.camara.width`;
- `sensores.camara.height`;
- `sensores.camara.mostrar_gazebo`.

Perfil 3G vigente en `dron_individual/config/hardware.yaml`:

```text
publish_rate=20.0
width=480
height=360
```

`dron_plugins.xacro` usa esos valores en `<update_rate>` y `<image>`. Sus
defaults 30 Hz/640x480 son solo fallback cuando el YAML no proporciona los
parametros.

Ojo:
- La calibración ORB-SLAM3 usada por el launch actual está en `dron_individual/config/orbslam/orbslam_stereo.yaml`.
- Debe ser coherente con la cámara generada por Xacro/Gazebo.
