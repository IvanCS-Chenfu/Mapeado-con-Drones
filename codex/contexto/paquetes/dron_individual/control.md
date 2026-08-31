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

Para el diagnostico 348, `[F5H-FALLBACK-CAUSE-TRACE]` se emite en cada cambio
de fuente/reason y conserva todos los predicados del mensaje raw. La decision
base ORB exige `tracking_state==OK && local_valid && local_continuity_valid` y
epoch anclado; `velocity_valid` y `reference_keyframe_valid` se registran como
predicados no pertenecientes al source gate. La traza distingue ademas
`ORB_QUALIFYING` y `TRAJECTORY_SOURCE_LOCKED`, estado del lock, edad, epoch,
Kref y samples de entrada/salida. No cambia la politica del mux.

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

El predictor angular raw/bias no vive en este nodo. Sus parametros se alojan
en `config/navigation_state.yaml`, pero los declara y consume
`orbslam3_ros2::StereoSlamNode`; control recibe directamente pose y velocidades
ya preparadas. La configuracion vigente añade deadband/confirmacion del bias,
supresion por movimiento y decay raw, pero todo ello se ejecuta en
`orbslam3_ros2`. La prueba 262 confirma que ganancias, formulas y camino GT
permanecen intactos mientras se diagnostica la oscilacion de `omega_motion`.

La instrumentacion 263 añade `[F5H-PHASE-CONTROL]` sin throttle cuando el debug
está activo. Conserva los vectores exactos usados en el tick: `R_act`, `R_des`,
omega recibida O y transformada a body, `Omega_des`, `er`, `ew`, términos de
torque separados, torque total, `Kr/Kw`, sample y timestamps. La suma de
términos es algebraicamente la misma ecuación previa y no altera el control.

Para el laboratorio dinamico 292-295, el mensaje existente
`control/tray/torque` incluye ahora el `header.stamp` del tick. El vector sigue
siendo exactamente `tau_total` en body: no cambian ecuacion, ganancias ni
valor publicado. `orbslam_use.launch.py` pasa `physical.yaml` al nodo
diagnostico para compartir `fisico.total.matriz_inercia`. Tras la primera 292,
el valor compartido se corrige a la inercia compuesta
`diag(0.00803107,0.00803107,0.015805) kg*m^2`; esto cambia los terminos
dinamicos que dependen de J, pero no ganancias ni estructura del controlador.

Para el laboratorio translacional 309-312, `control_calcular_fuerzas.cpp` ->
`Clase_Publisher::enviar_fuerzas` (`rg -n "control/tray/thrust|control_stamp"`)
publica ademas `control/tray/thrust` como `Vector3Stamped` en frame `cuerpo`.
Su vector es exactamente `[0,0,F_des.z()]` y comparte stamp con el torque del
mismo tick. El topic legacy `control/tray/fuerza` permanece sin cambios para el
mixer. Este es lineal, no aplica clipping/saturacion y sus cuatro salidas suman
el thrust total que Gazebo aplica sobre `+Z` relativo de los motores.

Auditoria causal post-318: antes del primer estado/feedback,
`aplicar_fuerzas_dron` conserva `fuerza_total=0` y `torque=0`, y su timer
publica esa mezcla a 50 Hz. Esto demuestra que el seed de actuación cero en el
consumidor dinamico representa el estado físico inicial, no un fallback
inventado.

La prueba 264 demuestra que el termino proporcional es decisivo: tras el
handoff, `tau_er` realiza trabajo positivo respecto a la omega GT en `80.9 %`
de los ciclos e inyecta `+0.005173 J` al reprocesar con el analizador de 265.
`tau_ew` es anti-amortiguante de forma
intermitente, pero netamente disipativo. Las ganancias y ecuaciones no se
modifican todavia; esta evidencia exige corregir primero la fase de la pose
angular que recibe el controlador.

En 266 el controlador permanece idéntico mientras `orbslam3_ros2` reancla la
pose visual. En la ventana comun, `tau_er` baja a `+0.002067 J` y el torque
total resulta disipativo (`-0.001945 J`), confirmando que la mejora procede del
estado de entrada y no de gains. El fallo tardio moderate conserva los gains
fuera del siguiente ajuste.

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
| `control/tray/thrust` | `geometry_msgs/msg/Vector3Stamped` | Copia sellada del thrust body para diagnostico F5H |

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

Laboratorio F5H: `navigation_state_mux.cpp` declara
`f5h_diagnostic_force_source`; `rg -n "f5h_diagnostic_force_source"` localiza
la selección forzada `gt|orb`. Solo se usa en 269-272, después de calcular la
decisión normal, para eliminar conmutaciones de la comparación. El default
`normal` conserva intacta la política runtime y el bloque debe retirarse.

Para el diagnostico post-320R, `f5h_diagnostic_force_source=shadow_gt`
mantiene GT autoritativo mientras el mismo ORB dinamico productivo sigue
publicando en sombra. `OrbShadowActivationGate` exige tracking, anchor, estado
consumible y `1.5 s` continuos bajo `0.15 m/s` y `0.15 rad/s`; el servicio
`control/activate_orb_shadow` habilita una sola frontera posterior. Los
marcadores `[F5H-ORB-SHADOW]`, `[F5H-ORB-ACTIVATION-READY]` y
`[F5H-ORB-ACTIVATED]` registran estado y saltos. Todo el bloque es temporal de
Fase 5 y debe retirarse junto a `GT_FALLBACK`.

La bateria 321 añade el topic transient-local
`control/orb_authority_confirmed`: solo publica `true` despues de que el mux
haya publicado efectivamente un `NavigationState` con source ORB. El marcador
`[F5H-ORB-AUTHORITY-CONFIRMED]` precede asi al nuevo goal. El parametro temporal
`f5h_orb_control_override` admite `normal`, `position_gt`, `velocity_gt` y
`position_velocity_gt`; sustituye exclusivamente posicion y/o velocidad lineal
en la salida comun, nunca en estimadores o buffers ORB. GT se alinea al O
continuo en el handoff; orientacion y omega permanecen ORB.

```text
dron/dron_individual/include/dron_individual/navigation_state_mux.hpp
  -> OrbShadowActivationGate/DiagnosticGtControlAlignment
  -> rg "class OrbShadowActivationGate|class DiagnosticGtControlAlignment"
dron/dron_individual/src/control_tray/navigation_state_mux.cpp
  -> OnShadowActivation/OnOrbState
  -> rg "activate_orb_shadow|F5H-ORB-AUTHORITY-CONFIRMED|f5h_orb_control_override"
```

- El fallback Fase 5 depende de GT, visible y desactivado por defecto.
- GT no puede alimentar estimacion, mapa, anchors ni pose global.
- Si se automatizan pruebas de Codex, el script debe llamar a `AccionTrayectoria` respetando namespaces.

Validacion de `F5H-FALLBACK-CAUSE-TRACE` en 348: el primer cambio ORB->GT se
produce con tracking 2 porque `local_valid` y `local_continuity_valid` son
falsos. `velocity_valid` y `reference_keyframe_valid` tambien caen, pero la
traza los marca como `NON_SOURCE_GATE`. La recuperacion posterior se separa de
`TRAJECTORY_SOURCE_LOCKED`; la instrumentacion no altera la fuente elegida.

Diagnostico 349: `DiagnosticGtStateBuffer` conserva hasta 200 muestras de pose
y twist GT, interpola ambas al `control_stamp` o propaga causalmente la ultima
pose si su soporte no excede `f5h_diagnostic_gt_max_skew_sec=0.03`. Los modos
`orb_pv_gt_angular` y `gt_pv_orb_angular` sustituyen bloques complementarios
despues de la decision del mux y antes de publicar al controlador.
`[F5H-CHANNEL-OVERRIDE]` registra stamps, skew, validez y fuente efectiva de
cada componente. El default `normal`, el source gate y el source lock no
cambian.

El buffer sella GT con tiempo ROS local de recepcion: los headers GT de Gazebo
pertenecen a otro dominio y no se comparan directamente con `control_stamp`.
La telemetria conserva el soporte/skew local que determina la validez causal.
La alineacion se captura una sola vez en la primera muestra ORB con soporte GT
valido, sin depender de que ese soporte coincida exactamente con el tick del
handoff.

La validacion 349AR3 amplia exclusivamente ese soporte de `20 ms` a `30 ms`.
No es una latencia añadida: la pose causal se propaga hasta `control_stamp`.
La traza incluye `navigation_source` y `override_requested` para que el ratio
excluya aproximacion, shadow y fallback.

Resultados: 349AR3 aplica R/omega GT en `5474/5475` publicaciones ORB y 349B
aplica p/v GT en `3778/3778`. Ambos experimentos fallan funcionalmente, lo que
clasifica defectos independientes en los dos bloques ORB. Esta infraestructura
es temporal de Fase 5 y no debe convertirse en ruta productiva.
