# Control del dron en `dron_individual`

## Archivos cubiertos

- `src/control_tray/gen_tray.cpp`
- `src/control_tray/navigation_state_mux.cpp`
- `include/dron_individual/navigation_state_mux.hpp`
- `src/control_tray/control_calcular_fuerzas.cpp`
- `src/control_tray/aplicar_fuerzas_dron.cpp`
- `src/vision/control_dron.cpp` como control experimental de visión

## Flujo de control actual

```text
Cliente action / GUI / script
  ↓ goal TrayAction
/dron_X/AccionTrayectoria   (`gen_tray`)
  ↓ feedback: trayectoria deseada
control_calcular_fuerzas
  ↓ fuerza + torque
aplicar_fuerzas_dron
  ↓ fuerzas por motor
plugin_actuar_motores en Gazebo
  ↓ fuerzas físicas en enlaces motor
Gazebo
```

## `gen_tray.cpp`

Nodo:

```text
gen_tray
```

Action server:

```text
AccionTrayectoria
```

Entradas:

| Entrada | Tipo | Uso |
|---|---|---|
| `orbslam/navigation_state` | `orbslam3_msgs/msg/NavigationState` | Pose/velocidad comun, gate y snapshot |
| `AccionTrayectoria` | `dron_individual/action/TrayAction` | Objetivo de trayectoria |

Salida:

- feedback de action con arrays `x`, `y`, `z`, `yaw`.

Funciones internas:

- `pose_actual_callback`: guarda una muestra de pose cuando no está bloqueada.
- `vel_actual_callback`: guarda una muestra de velocidad.
- `proyeccion`: reparte una velocidad máxima lineal entre X/Y/Z según dirección del movimiento.
- `pose2yaw`: extrae yaw de un `PoseStamped`.
- `normalizar_angulo`: normaliza ángulos a `[-pi, pi]`.
- `array_to_msg`: convierte `std::array<double,5>` a `Float64MultiArray`.
- `handle_goal`: valida tipo, frescura y semántica local/global del goal.
- `handle_cancel`: acepta cancelaciones.
- `handle_accepted`: aborta goal anterior si existe y lanza `execute` en hilo separado.
- `execute`: función principal. Espera pose/vel inicial, crea trayectoria según tipo y publica feedback hasta terminar/cancelar.

`NavigationGoalPolicy` exige muestra fresca, local, continua y con velocidad.
Un absoluto requiere global valida, `GT_FALLBACK` o C_T_W cacheada del mismo
epoch. El goal congela epoch, muestra y transformacion antes de ejecutar.

Cada hilo `execute` coordina `control/set_trajectory_active` con
`navigation_state_mux`. La fuente queda retenida al terminar y durante waits.
Al empezar el siguiente goal se abre la frontera, se espera una muestra
consumible, se vuelve a bloquear y se congelan conjuntamente pose, velocidad y
frame absoluto. El primer feedback usa `t=0` y esa misma condición inicial.
Los marcadores son `[F5H-SOURCE-RETAINED-BETWEEN-GOALS]` y
`[F5H-ATOMIC-GOAL-START]`.

Para diagnosticar el contrato de frame sin alterar el control, cada goal
absoluto iniciado con ORB emite dos líneas
`[F5H-ABSOLUTE-FRAME-DIAG]`: `part=poses` conserva `O_T_B/W_T_B` y autoridad;
`part=target_axes` conserva `C_T_W`, los ejes world expresados en control y el
target antes/despues de transformarlo. Ambas comparten `epoch/sample` y permiten
distinguir direccion de composicion de una convencion body/camera incompatible.
La prueba 251 confirma que la formula de `C_T_W` refleja fielmente sus entradas:
X world aparece casi como Z control porque el `W_T_B` recibido ya lleva la
extrinseca optica invertida. No se debe compensar este defecto en `gen_tray`.

El modo legacy queda solo para pruebas antiguas. El camino Fase 5 operativo no
suscribe `gen_tray` ni control directamente a `sensor/GT/pose` o
`sensor/GT/vel`.

Referencia:

```text
dron/dron_individual/src/control_tray/gen_tray.cpp
  -> Clase_Servicio_Accion::{handle_goal,handle_accepted,execute}
  -> rg "handle_goal|bloquear_callback = false|pose_ready"
dron/dron_individual/include/dron_individual/navigation_goal_policy.hpp
  -> NavigationGoalPolicy::Evaluate
  -> rg "reject_global_invalid|accept_relative"
```

Tipos de trayectoria:

1. `tipo_trayectoria=0`: `GenTrayPol3`.
2. `tipo_trayectoria=1`: `GenTrayVelTrap`.
3. `tipo_trayectoria=2`: `GenTrayElipse`.

Para `tipo_trayectoria=2`, `target_pose.position` define el centro, `tx/ty`
son los radios, `tz` rota la elipse y `tyaw` fija el tiempo de una vuelta. El
goal completo se genera al aceptarse. Con centro X/Y relativo, este se expresa
respecto al yaw actual del dron. El generador vigente recorre siempre una vuelta
en el sentido positivo; no existe todavia un parametro de sentido.

Notas:
- Para objetivos relativos en X/Y se transforma el desplazamiento usando yaw actual.
- Para yaw relativo puede haber normalización si se sale de `[-pi, pi]`.
- Solo debe haber un goal activo; uno nuevo aborta el anterior.

## `navigation_state_mux`

El mux recibe desde `orbslam3_ros2` el estado ORB ya filtrado y propagado a
50 Hz. No estima, filtra ni predice pose o velocidad. Selecciona ORB o fallback,
aplica la transformacion rigida al frame O continuo y publica
`orbslam/navigation_state`. `[F5H-SOURCE-CONTINUITY]` mide el salto SE(3)
exacto al cambiar fuente.

Al pasar `ORB -> GT_FALLBACK`, `ContinuousSourcePose::Update` calcula una
alineacion fija de GT contra el ultimo O y conserva exactamente el frame del
goal activo. La pose world GT viaja temporalmente en `w_t_body` con
`global_valid=false`; solo `gen_tray` la usa para componer `O_T_W` de fallback.
GT queda retenido hasta la frontera y ORB solo puede volver en el siguiente
goal si cumple tracking, anchor y cualificacion.

El mux recibe temporalmente `sensor/GT/pose` y `sensor/GT/vel`. En fallback
reenvia ambas medidas exactas, expresadas mediante la misma rotacion rigida en
el O continuo; no pasan por filtros ni predictores. Suscripcion, transporte,
lock, hold y alineacion GT llevan `TODO FASE 6` porque desaparecen junto con el
fallback.

Referencia:

```text
dron/dron_individual/src/control_tray/navigation_state_mux.cpp
  -> NavigationStateMuxNode::{OnOrbState,OnGtPose,OnGtVelocity}
  -> rg "OnOrbState|OnGtPose|OnGtVelocity"
dron/dron_individual/include/dron_individual/navigation_state_mux.hpp
  -> ContinuousSourcePose
  -> rg "class ContinuousSourcePose|RotateVectorFromSource"
```

## `control_calcular_fuerzas.cpp`

Nodo:

```text
control_calcular_fuerzas
```

Rol:
- recibe feedback de trayectoria y `orbslam/navigation_state`;
- calcula fuerza total y torque deseado con control PD;
- publica comandos agregados.

Con `debug_orb_control_state=true`, el callback registra estados no consumibles
mediante `[F5H-CONTROL-STATE-INVALID]` y el lazo emite a 10 Hz
`[F5H-CONTROL-DIAG]`: timestamp/edad y metadata de `NavigationState`, normas
`ep/ev/er/ew`, fuerza, torque, RPY y omega corporal. Las formulas y ganancias
del controlador no cambian; el flag queda desactivado por defecto.

Al cambiar `NavigationState.pose_source`, el callback sustituye cualquier
consigna retenida del frame anterior por un hold en la nueva `o_t_body`:
posición, velocidad, yaw y yaw rate actuales, con aceleración, jerk y yaw
acceleration a cero. El marcador `[F5H-CONTROL-SOURCE-HOLD]` registra la transición. Esto evita
que el intervalo entre dos action goals compare una consigna GT antigua con una
pose ORB nueva; el siguiente feedback reemplaza normalmente el hold.

En `GT_FALLBACK -> ORB`, el primer feedback activa ademas un handoff angular
temporal de Fase 5. Captura la orientacion completa `R_act` y la velocidad
angular corporal `w_b`; el primer ciclo usa `R_des=R_act` y
`Omega_des=w_b`, haciendo cero `er/ew`, y durante
`control.source_handoff_duration_sec` (default `0.5 s`) interpola en SO(3) con
`slerp` hacia la referencia nominal. No modifica `ORB -> GT`. El marcador
`[F5H-ANGULAR-HANDOFF]` registra inicio, mitad y final con normas de `er/ew` y
comandos de fuerza/torque. Este mecanismo se debe retirar junto al fallback en
Fase 6.

Entradas típicas:

| Topic | Tipo | Uso |
|---|---|---|
| `AccionTrayectoria/_action/feedback` | feedback action | Trayectoria deseada |
| `orbslam/navigation_state` | `NavigationState` | `o_t_body` y velocidad exactas de control |

Salidas típicas:

| Topic | Tipo | Uso |
|---|---|---|
| `control/fuerza_total` | `std_msgs/msg/Float64` | Fuerza vertical total |
| `control/torque` | `geometry_msgs/msg/Vector3Stamped` | Torque deseado |

Funciones/conceptos:

- convierte pose a yaw;
- calcula error de posición/velocidad;
- aplica ganancias `kp`, `kv`, `kr`, `kw` de `control.yaml`;
- usa masa, inercia, gravedad y geometría del dron;
- calcula comandos agregados para `aplicar_fuerzas_dron`.

## `aplicar_fuerzas_dron.cpp`

Nodo:

```text
aplicar_fuerzas_dron
```

Rol:
- recibe fuerza total y torque;
- resuelve una matriz de mezcla para obtener fuerza en cada motor;
- publica fuerzas individuales.

Entradas:

| Topic | Tipo | Uso |
|---|---|---|
| `control/fuerza_total` | `std_msgs/msg/Float64` | Fuerza total |
| `control/torque` | `geometry_msgs/msg/Vector3Stamped` | Torque XYZ |

Salidas:

| Topic | Tipo |
|---|---|
| `motor/arr_iz` | `std_msgs/msg/Float64` |
| `motor/ab_iz` | `std_msgs/msg/Float64` |
| `motor/ab_der` | `std_msgs/msg/Float64` |
| `motor/arr_der` | `std_msgs/msg/Float64` |

Funciones/conceptos:

- construye matriz `A` de reparto fuerza/torque;
- si `A` es casi singular usa pseudoinversa;
- publica fuerzas por motor periódicamente.

## `control_dron.cpp`

Estado: experimental/visión.

Rol:
- cliente de `AccionTrayectoria`;
- recibe puntos de visión en `vision/keypoint_cercano`;
- publica byte de control en `vision/byte_control`;
- implementa comportamientos como encontrarse, colocarse frente a pared y obtener nube.

No es parte central del pipeline actual de sparse global multi-dron.

## Riesgos

- El fallback Fase 5 depende de GT, visible y desactivado por defecto.
- GT no puede alimentar estimacion, mapa, anchors ni pose global.
- Si se automatizan pruebas de Codex, el script debe llamar a `AccionTrayectoria` respetando namespaces.
