# Script `StereoSlamNode` del wrapper ORB-SLAM3 estéreo

## Rol

`StereoSlamNode` es el nodo que conecta cámaras estéreo ROS 2 con ORB-SLAM3. Además de ejecutar tracking local, exporta al servidor global la información de mapa necesaria para construir un mapa sparse multi-dron.

## Flujo interno

```text
camera/left + camera/right
  ↓ sincronización ApproximateTime
GrabStereo
  ↓ cv_bridge + rectificación opcional
ORB_SLAM3::System::TrackStereo + StereoTrackingReceipt
  ├─ callback: pose local + mapa ORB
  └─ solo KF y config READY: cola acotada -> FiducialWorker
       ├─ tags validos -> FiducialKeyFrameObservations -> Servidor
       └─ debug opcional -> Image ROS -> fiducial_visualizer separado
  ↓
pose local Twc + mapa ORB interno
  ↓
PublishLocalPose / PublishOrbMapDelta / get_full_map
```

## Frontera camara-body y auditoria 1J

`TrackStereo()` y los `OrbKeyFrame` trabajan en pose de camara. Esa semantica
alimenta tambien el mapa sparse y F4 y debe conservarse.

Para `NavigationState`, el constructor carga una unica `body_t_camera_` desde
los parametros `body_T_camera_*`. `PublishNavigationState()` aplica hoy:

```text
pose_result.o_t_camera * body_t_camera_.inverse() -> raw_o_t_body
pose_result.w_t_camera * body_t_camera_.inverse() -> raw_w_t_body
```

La Fase 1J debera sustituirla por la extrinseca correspondiente al stamp de la
imagen cuando el rig tenga pitch. El predictor lineal/angular debe seguir
recibiendo pose body ya compensada; aplicar el estado mas reciente sin
sincronizacion produciria velocidad ficticia durante el movimiento del joint.
1J incorpora los parametros `body_camera_transform_mode=static|tf` y
`camera_frame`. En modo `tf`, `ResolveBodyTCamera()` exige la extrinseca al
stamp de imagen; si falta, publica salida body invalida y reinicia el predictor
sin alterar pose de camara, KFs o fiduciales. Compila y pasa los 117 tests del
estimador. Las pruebas 364-366 validan el joint y control bajo GT_FORCED, pero
no validan aun la salida body ORB ni la dinamica F5 durante pitch. Pendientes:
tests de composicion variable y pruebas ORB en sombra/productivas.

`orbslam_use.launch.py` publica `dron_X/base_link` como `body_frame`, aunque el
link fisico principal del modelo se llama `cuerpo`. La introduccion de TF debe
resolver esa relacion sin cambiar accidentalmente el frame publico de
`NavigationState`. El modo `static` conserva despliegues sin rig movil y el
modo `tf` no reutiliza silenciosamente el ultimo pitch disponible.

F4 publica `camera_t_tag` del mismo KF y el servidor obtiene
`world_T_camera`; no usa esta `body_t_camera_`. No debe añadirse una segunda
compensacion por pitch en esa ruta.

Auditoria post-366: el predictor de thrust conserva masa `1.4 kg` y el de
torque una inercia body fija/torque de control. Gazebo incorpora `0.04 kg` de
rig/camaras y torque de reaccion del servo. Ese residual debe medirse y su
politica cerrarse antes de validar F5 ORB con camara movil.

## Constructor `StereoSlamNode(...)`

Responsabilidades:

1. Declara parámetros ROS:
   - `drone_id`;
   - `drone_name`;
   - `local_map_frame`;
   - `delta_publish_period_frames`;
   - `fiducial_queue_capacity=4`;
   - `debug_fiducial_visualization=false`.

2. Carga parámetros de cámara desde el YAML ORB-SLAM3:
   - `Camera.fx`, `Camera.fy`, `Camera.cx`, `Camera.cy`, `Camera.bf`;
   - `Camera.width`, `Camera.height`;
   - fallback a `LEFT.K`, `LEFT.width`, `LEFT.height` si hace falta.

   Estos campos siguen alimentando `OrbMap`; para PnP el wrapper usa
   exclusivamente el modelo efectivo del recibo de `TrackStereo()`.

3. Crea publishers:
   - `orbslam/orb_map_delta`;
   - `orbslam/pose_local`;
   - `orbslam/navigation_state`;
   - `orbslam/fiducial_keyframe_observations`, reliable/volatile
     KeepLast(32);
   - `orbslam/fiducial_debug/image`, solo con debug activo, con
     best-effort/KeepLast(1).

4. Crea servicio:
   - `orbslam/get_full_map`.

5. Configura rectificación si `rectify=true`.

6. Crea subscribers sincronizados:
   - `camera/left`;
   - `camera/right`.

7. Crea cliente absoluto `/global_mapping/get_fiducial_config`, timer de
   retry/timeout y un worker fiducial. El wrapper no crea ventanas ni ejecuta
   HighGUI.

## `GrabStereo(msgLeft, msgRight)`

Callback principal.

Hace:

1. Convierte imágenes ROS a `cv::Mat` con `cv_bridge`.
2. Rectifica si `doRectify` está activo.
3. Ejecuta:

```cpp
m_SLAM->TrackStereo(...)
```

4. Actualiza epoch/mapa activo con `UpdateMapEpochFromCurrentMap()`.
5. Si tracking está OK, publica pose local con `PublishLocalPose()`.
6. Incrementa `frame_counter_`.
7. Publica delta cada `delta_publish_period_frames_`.
8. Si detecta nuevo epoch, fuerza publicación inmediata.

### Recibo 4C y trabajo fiducial 4D

`GrabStereo()` pasa un `StereoTrackingReceipt` a la misma llamada de
`TrackStereo()`. Si `keyframe_event.created=true`, verifica timestamp y
geometria; solo con configuracion `READY` mueve la copia efectiva a una cola de
capacidad cuatro. Si se llena elimina el trabajo mas antiguo. No hay sondeo de
`GetLastKeyFrameInfo`, buffer pre-READY ni deteccion en el callback.

Desde 5B, `StereoTrackingReceipt` incluye `tracking_state`, reference KF real y
`Tcr` del mismo frame. `NavigationStateEstimator` fija `O_T_Kref` y compone
`O_T_B` mediante la extrínseca `body_T_camera`; un cambio de reference KF
reancla su relación sin salto discreto. Un epoch nuevo reinicia la continuidad
y `RECENTLY_LOST`/`LOST` publican local/continuidad inválidas, sin prolongar la
última pose como si siguiera siendo válida. Global y velocidad quedan
explícitamente inválidas hasta subfases posteriores.

Desde 5D-5E, el wrapper solicita la pose global de la reference KF activa al
servicio compartido y escucha el push dirigido reliable. Un cambio de reference
KF reemplaza la solicitud; respuestas tardias, epoch distinto y revisiones no
crecientes se descartan. `NavigationStateEstimator` conserva `O_T_B` intacta,
publica W provisional al cambiar referencia y solo marca
`AUTHORITATIVE/global_valid=true` al aceptar autoridad del backend. La
composicion es `W_T_C = W_T_Kref * inverse(Tcr)` y
`W_T_B = W_T_C * inverse(body_T_camera)`.

Tras la regresion 253, el suavizado pertenece exclusivamente a este wrapper.
`OrbPosePredictor` recibe `O_T_B` de cada frame ORB, aplica alpha-beta a
traslacion y propaga el estado a 50 Hz desde un timer de `StereoSlamNode`.
En rotacion calcula `DeltaR_raw`, `omega_raw` y `alpha_raw` respecto a la medida
raw anterior. Su calidad temporal y plausibilidad alimentan `omega_motion` con
un filtro separado. El residual absoluto medida-prediccion mantiene la
probation temporal y alimenta una `omega_bias` acotada; la pose publicada se
integra desde `omega_total=omega_motion+omega_bias`. SMALL es fijo.
`omega_bias` usa `BiasCorrectionState` (`OFF/PENDING/ACTIVE/DECAY`), deadband
con histeresis y confirmacion temporal; el movimiento raw significativo
suprime el bias. Raw `SUSPICIOUS/REJECTED` lleva `omega_motion` gradualmente a
cero. Una innovacion excesiva acumula rechazo y tres rechazos invalidan ORB. Tracking
invalido reinicia el predictor y publica estado invalido a 50 Hz para que el
mux pueda activar fallback sin perder cadencia.

`NavigationStateEstimator` separa reference KF reportada, candidata y activa.
La probation acumula una cadena de incrementos plausibles aunque cambie el ID
candidato. Dentro del mismo KF usa `Tcr`; al cambiar de ID, el incremento de
`local_t_camera` solo enlaza la cadena si es fisicamente plausible y un salto BA
se trata como incremento cero. Tres frames geometricos confirman la referencia.
Durante probation se publica prediccion, se conserva metadata/autoridad activa
y no se solicita W para la candidata. Volver a la activa cancela probation si
la cadena es coherente; una inconsistencia persistente agota seis frames y
fuerza invalidez. Marcador: `[F5H-REFERENCE-GATE]`.

Parametros: `orb_state_publish_rate_hz`; `orb_state_filter.position_alpha`,
`orientation_alpha`, `max_position_innovation_m`,
`max_rotation_innovation_rad`, `max_linear_speed_mps`,
`max_angular_speed_radps`, `max_linear_acceleration_mps2`,
`max_angular_acceleration_radps2`, `max_consecutive_angular_rejections` y
`max_extrapolation_sec`; `small_rotation_innovation_rad`,
`moderate_confirmation_frames`,
`moderate_post_reference_confirmation_frames`,
`moderate_max_pending_frames`, `moderate_direction_consistency`,
`moderate_magnitude_ratio`, `moderate_timeout_sec` y
`post_reference_switch_frames`; `orb_reference_gate.confirmation_frames`,
`max_pending_frames`, `max_step_translation_m` y `max_step_rotation_rad`.
La configuracion de despliegue vive en
`dron/dron_individual/config/navigation_state.yaml`.

El canal raw añade `raw_dt_max_good_sec`, `raw_dt_max_degraded_sec`,
`max_raw_rotation_step_rad`, `max_raw_angular_speed_radps`,
`max_raw_angular_acceleration_radps2` y `raw_motion_filter_alpha`. El bias usa
`max_orientation_bias_correction_rate_radps` y
`max_orientation_bias_correction_acceleration_radps2`. La politica adicional
usa `bias_deadband_enter_rad=0.005`, `bias_deadband_exit_rad=0.002`,
confirmacion `3/4`, supresion por movimiento `0.10/0.05 rad/s` y decay raw
`4 rad/s2`. Los limites raw calibrados en 260 permanecen en `0.075/0.20 s`,
`0.12 rad`, `1.0 rad/s`, `10 rad/s2` y alpha `0.35`.

Con `debug_orb_control_state=true`, `[F5H-ORB-MEASUREMENT]` conserva contexto,
raw step, innovacion vectorial, clasificacion, id/contadores pending,
consistencia, correccion aplicada, velocidades limitadas y salud.
`[F5H-ORB-PUBLISH]` mide aparte el paso timer-a-timer realmente publicado,
pose, omega, timestamp de imagen, instante ROS de recepcion y edad calculada
solo entre instantes del mismo reloj. Los eventos moderados/duros abren una
ventana detallada de dos segundos; fuera de ella la publicacion se muestrea.
Marcadores generales: `[F5H-ORB-STATE-PREDICTOR]` y
`[F5H-ORB-STATE-FILTER]`.

Para análisis de fase, `[F5H-PHASE-MEASUREMENT]` emite cada medida con
orientación raw y vectores `omega_raw`, target/motion, bias y total;
`[F5H-PHASE-PUBLISH]` emite cada tick de 50 Hz con sample, pose y omega. El
input usa reloj Gazebo y receive/publish reloj ROS, por lo que se declaran como
dominios distintos y la edad visual se reconstruye offline mediante GT
dual-clock.

El diagnostico lineal post-321 añade, solo con
`debug_orb_control_state=true`, `[F5H-LINEAR-MEASUREMENT]`: stamps de imagen y
recepcion, `p_k2/p_k1/p_k`, velocidades midpoint, `a_hat`, `v_hat_tk`, modo y
calidad temporal, reference KF y `correction_class`. Es observabilidad pura:
no altera `CausalLinearVelocityEstimator`, su salida ni la rama productiva.

El laboratorio temporal F5H vive en
`src/stereo/gt-timing-diagnostic-node.cpp` -> `GtTimingDiagnosticNode`
(`rg -n "F5H-GT-TIMING|TracePeriods|DeliveryDelay"`). Alimenta el mismo
`OrbPosePredictor` con pose GT perfecta, conserva el timestamp fisico y publica
`NavigationState` a 50 Hz con telemetria compatible. Sus modos son `gt_50`,
`gt_20`, `gt_20_delay` y `gt_orb_timing`; esta apagado por defecto y debe
retirarse tras el diagnostico.

El laboratorio translacional vive en los mismos archivos:
`navigation-state-estimator.hpp/.cpp` -> `BodyThrustDynamicPredictor`
(`rg -n "BodyThrustDynamicPredictor|F5H-TRANSLATIONAL"`). Conserva un buffer
acotado de thrust body sellado, usa la masa `fisico.total.masa`, gravedad world
y `R_dynamic(t)` del `BodyTorqueDynamicPredictor`. Integra cada intervalo con
dt real, aceleracion en el punto medio, velocidad semiimplicita y posicion
trapezoidal. Los modos 309-312 son exclusivamente diagnosticos y no conectan
esta clase a `StereoSlamNode` productivo.

En la ruta productiva post-323, la gravedad fisica parte de
`g_W=(0,0,-9.81)` y `EpochGravityState` la expresa como
`g_O=O_R_W*g_W`. `StereoSlamNode` obtiene `O_T_W` de la primera pose global
`AUTHORITATIVE`, congela `g_O` durante el `map_epoch` y la invalida al cambiar
de epoch. Hasta disponer de ella, la base translacional dynamic no es
consumible. Las revisiones globales posteriores no sobrescriben la gravedad.
Marcadores debug: `[F5H-GRAVITY-O-INIT]`, `[F5H-GRAVITY-O-WAIT]` y
`[F5H-DYNAMIC-TRANSLATION]`.

`CausalLinearVelocityEstimator` conserva THREE_SAMPLE para telemetria y
comparacion. La ruta productiva post-327 usa `PredictMidpointDynamicVelocity`
en `navigation-state-estimator.hpp/.cpp` (`rg -n
"PredictMidpointDynamicVelocity|F5H-MIDPOINT-DYNAMIC"`). A partir de dos poses
visuales aceptadas calcula `v_mid`, interpola R en SO(3), estima omega espacial
causal y traduce el midpoint de reloj imagen a reloj ROS. Los predictores de
torque y thrust propagan conjuntamente hasta `t_k` usando gravedad O. Solo
cobertura `FULL` y resultado valido sustituyen la base lineal productiva; un
rechazo conserva la ultima base aceptada y el cambio de epoch reinicia el
historial. `[F5H-PRODUCTIVE-MEASUREMENT]` declara
`linear_source=MIDPOINT_DYNAMIC|UNAVAILABLE`; THREE_SAMPLE no gobierna.
326-329 validan cobertura y precision, y 330/331 completan hover ORB real sin
fallback ni perdida de tracking.

E/F/G añaden los modos `gt_20_exact_omega`,
`gt_20_exact_omega_hold` y `gt_20_exact_omega_extrapolate`. La omega GT world/O
se sincroniza con la ultima muestra cuyo timestamp no es futuro. E llama a
`OrbPosePredictor::OverrideAngularVelocityForDiagnostics`; F/G sustituyen solo
la rama angular y conservan la traslacion del predictor. La API y estos modos
son instrumentacion temporal marcada para retirar, no arquitectura final.

Estimacion causal vigente desde 276:

- `OrbPosePredictor::UpdateMeasurement` calcula velocidad espacial world/O con
  `Log(R_k R_{k-1}^{-1})/dt`, coherente con propagacion izquierda;
- con tres poses GOOD estima aceleracion entre midpoints y proyecta la ultima
  velocidad media solo hasta `t_k`;
- con dos poses o dt degradado usa el ultimo intervalo; entre callbacks hace
  hold de omega, sin extrapolarla con aceleracion hasta `now`;
- una medida rechazada no avanza el historial, el decay permanece y un cambio
  de `map_epoch` reinicia el historial angular;
- `raw_reversal_noise_step_rad=0.005` anula solo inversiones microscopicas;
  `raw_motion_filter_alpha<=0` queda como kill switch de tests, no filtro;
- localizar con
  `rg -n "ThreeSamplePredicted|omega_hat_at_measurement|microscopic_reversal"`.

276-277 con pose GT 20 Hz completan y son disipativas. Esta evidencia valida
el estimador sin delay, pero no aun ORB real.

Diagnostico 282/284:

- `gt-timing-diagnostic-node.cpp` construye solo su predictor con horizonte
  `0.18 s`; el wrapper productivo conserva `0.10 s`;
- `OrbPosePredictorConfig::predict_angular_acceleration=false` mantiene el
  comportamiento productivo. El laboratorio lo activa y `Predict` integra
  coherentemente pose/omega al mismo target mediante aceleracion causal;
- `PredictedOrbPoseState` informa alpha, delta angular y limites. Alpha se
  invalida con dt degradado, rechazo, epoch, override o reset;
- cinco GTests cubren alpha cero, aceleracion conocida, cambio de signo, clamp
  y coherencia pose/omega. 282 y 284 fallan; la rama no se habilita en ORB real.

La prueba 262 valida el nuevo deadband, supresion y decay con 37/37 GTests y en
lazo cerrado: `omega_bias=0` durante ORB y raw rechazado decae sin corte. Aun
asi el hover falla porque `omega_motion` empieza a oscilar mientras raw todavia
es plausible y alcanza unos `0.617 rad/s`. ORB dura unos 5.92 s y el fallback
precede en ~0.54 s a tracking 2->3. El siguiente diagnostico debe correlacionar
timestamps y fase de `omega_raw -> omega_motion -> control omega -> ew ->
torque`; no corresponde tocar W, GT, mux ni ganancias.

La prueba 264 completa ese diagnostico con 323 ciclos ORB sincronizados. Raw
sigue al GT en x/y con correlacion `0.984/0.982` y lag `~0.08 s`, mientras el
estado angular usado por control queda fuera de fase y acaba en rechazo/fallback
con tracking aun en 2. La siguiente modificacion debe tratar la coherencia de
fase de la orientacion publicada; no se justifica aumentar filtros o retocar
umbrales raw por latencia de transporte.

Semantica temporal comprobada tras 264:

- `PublishNavigationState` llama a `UpdateMeasurement(raw_o_t_body,
  RosTimeToSeconds(image.header.stamp), ...)`; por tanto `stamp_sec_` usa el
  tiempo de imagen, no el instante de finalizacion de ORB;
- al terminar una actualizacion, `pose_` y `angular_velocity_` representan el
  estado integrado al stamp de imagen. `pose_` no se sustituye por la
  orientacion raw: avanza desde el estado anterior con
  `omega_motion + omega_bias`;
- el timer llama `Predict(now.seconds())`; en la captura 264 ese `now` esta en
  reloj ROS epoch y `stamp_sec_` en Gazebo, de modo que el horizonte se satura
  siempre en `max_extrapolation_sec=0.10` en vez de usar la edad visual real;
- `Predict` no muta `pose_`, por lo que no acumula dos propagaciones, pero su
  unica extrapolacion usa actualmente un horizonte incorrecto;
- `latest_orb_measurement_stamp_sec_` se toma despues de `TrackStereo`, no al
  entrar la imagen al callback. El puente Gazebo/ROS de la prueba 264 vive en
  metricas offline y usa GT; no es una entrada valida para el predictor;
- `orientation_alpha` se declara y carga, pero la ruta angular vigente no lo
  consume. Durante movimiento con bias suprimido, la orientacion depende de la
  integracion de `omega_motion` y no recibe una fusion directa con la
  orientacion raw.

Correccion temporal para 265:

- `GrabStereo` captura `callback_arrival_stamp_sec` antes de conversion,
  rectificacion y `TrackStereo`, y lo pasa explicitamente a
  `PublishNavigationState`;
- `ComputeOrbPredictionTiming` calcula la edad local no negativa entre ese
  ingreso y el tick actual, y la suma al stamp visual. Asi `Predict` recibe un
  target del mismo dominio que `stamp_sec_`, sin GT ni resta Gazebo/ROS;
- `Predict` sigue siendo `const` y realiza una unica propagacion desde `pose_`;
  informa `prediction_horizon_sec` y `prediction_clamped`, reutilizando
  `max_extrapolation_sec=0.10`;
- la publicacion diagnostica separa `visual_q`, `base_q` y `predicted_q`, junto
  con ingreso, final de procesamiento, edad local, horizonte y clamp. No hay
  fusion nueva con la orientacion raw.

Resultado 265: la correccion usa edad local media `51.5 ms`, horizonte medio
`43.2 ms` y clamp `10.4 %`, por lo que elimina la saturacion fija y conserva
una unica propagacion. Sin embargo, `visual_q -> base_q` llega a `0.339 rad`,
mientras `base_q -> predicted_q` solo aporta `0.0093 rad` medio tras 4 s. El
hover empeora y `tau_er` inyecta `+0.160266 J`; el problema dominante esta en
la orientacion base integrada de `pose_`, no en el horizonte del timer. No se
ha añadido fusion raw.

Reanclaje angular vigente tras 267:

- `pose_` representa el mejor estado base en el timestamp visual actual;
- SMALL con raw plausible adopta directamente la orientacion `O_T_B` visual
  continua entregada por `NavigationStateEstimator`;
- MODERATE_CONFIRMED solo con raw plausible adopta tambien por completo la
  orientacion visual; una confirmacion con raw rechazado queda PREDICT_ONLY;
- MODERATE_PENDING, MODERATE_DISCARDED y REJECTED conservan PREDICT_ONLY;
- `omega_motion` sigue describiendo el movimiento entre observaciones y
  `Predict` conserva la unica extrapolacion desde `t_visual`;
- diagnostics distinguen predicted-before/base-after, tipo de update,
  correccion y error visual-base before/after.

Resultado 266: 118/157 medidas ORB son `SMALL_ANCHOR` y dejan error after
exactamente cero. En ventana comun `tau_er` baja de `+0.153559` a
`+0.002067 J`; ORB dura `8.06 s`. Desde `+5.90 s`, moderate confirmado deja
`0.040 rad` medio tras corregir, alterna con PREDICT_ONLY y raw se rechaza a
`+7.50 s`; fallback llega a `+8.08 s`. El anclaje SMALL se conserva y la
recuperacion moderate sigue abierta.

Resultado 267: cuatro `MODERATE_CONFIRMED_ANCHOR` dejan error after cero, pero
ORB dura `5.56 s`; desde el primer anclaje hasta fallback `tau_er` acumula
`+0.029136 J` y el total `+0.018489 J` en `1.22 s`, frente a `2.06 s` para una
energia total casi identica en 266. Un anclaje se aplico con raw rechazado;
despues de la prueba se añade el gate raw y una regresion, alcanzando 46/46
GTests. Esta version final no se ha simulado y no se ejecuta 268 sin acuerdo.

Resultado 268: el gate raw final queda validado en simulacion. El unico
`MODERATE_CONFIRMED_ANCHOR` es PLAUSIBLE, corrige `0.057317 rad` y deja error
after cero; aun asi, desde ese instante hasta fallback se acumulan
`tau_er=+0.043934 J` y total `+0.046416 J` en `0.88 s`. No hay 269. El
anclaje completo no debe repetirse; `Delta_target` gradual a `0.30 rad/s` esta
acordado conceptualmente, pero no implementado ni autorizado todavia.

Referencias: `src/stereo/stereo-slam-node.cpp` -> `PublishNavigationState` y
`PublishPredictedNavigationState` -> `rg "UpdateMeasurement|Predict\(now"`;
`src/stereo/navigation-state-estimator.cpp` -> `OrbPosePredictor::UpdateMeasurement`,
`Predict` y `Propagate` -> `rg "stamp_sec_|angular_prediction|Propagate"`.

El marcador diagnostico temporal `[F5H-WRAPPER-FRAME-DIAG]`, limitado a pose
global autoritativa y con throttle de un segundo, separa `part=inputs`
(`O_T_C`, `W_T_C`, `B_T_C`) y `part=outputs` (`O_T_B`, `W_T_B`, `Tcr`). Ambas
lineas comparten dron, epoch, reference KF y raw sample. Permite comprobar si
la autoridad ya llega con una extrinseca adicional o si el error aparece al
convertir camara a cuerpo, sin alterar ningun mensaje publicado.

El diagnostico 251 localizo que la antigua rotacion `RPY=(0,-90,90)` era
`C_T_B` y se invertia otra vez. La calibracion vigente ya carga un `B_T_C`
completo: `RPY=(-90,0,-90)` en la misma convencion `yaw*pitch*roll` y la
traslacion de la camara expresada en body. Por tanto la composicion existente
`W_T_C * inverse(body_T_camera)` convierte de camara optica a cuerpo sin
permutar los ejes. El parametro YAML `use_camera_optical_frame_convention` no
se declara ni consume en este nodo; la convencion queda materializada en el
propio SE(3).

```text
dron/orbslam3_ros2/src/stereo/navigation-state-estimator.cpp
  -> Update / ApplyAuthoritativeGlobalPose / InvalidateGlobalPose
dron/orbslam3_ros2/src/stereo/stereo-slam-node.cpp
  -> RequestGlobalPose / ApplyGlobalPoseMessage / PublishNavigationState
rg "F5D-KF-REQUEST|F5E-GLOBAL-AUTHORITY|F5E-POSE-STATE"
```

Referencias de este limite vigente:

```text
dron/ORB_SLAM3/include/System.h
  -> ORB_SLAM3::System::StereoTrackingReceipt
  -> rg "struct StereoTrackingReceipt"
dron/ORB_SLAM3/src/System.cc
  -> ORB_SLAM3::System::TrackStereo
  -> rg "System::TrackStereo|ConsumeLastCreatedKeyFrameEvent"
dron/ORB_SLAM3/src/Tracking.cc
  -> almacenamiento de Tcr por frame
  -> rg "Tcr_|mlRelativeFramePoses"
dron/orbslam3_ros2/src/stereo/navigation-state-estimator.cpp
  -> NavigationStateEstimator::Update
  -> rg "reference_keyframe_id|continuity_valid|Reset"
```

`NavigationStateEstimator::Update` calcula `step_translation_m` y
`step_rotation_rad` entre estados consecutivos procesados. El log
`[F5B-O-CONTINUITY]` esta limitado a una linea cada dos segundos, pero los
valores que muestra corresponden al ultimo paso entre frames, no al intervalo
completo entre dos lineas de log.

`ManageFiducialConfig()` reintenta cada segundo y declara timeout a los dos
segundos. Config vacia deja el componente `DISABLED`. `FiducialWorkerLoop()`
ejecuta `FiducialDetector` con APRILTAG_36H11, SUBPIX e IPPE_SQUARE; conserva
todos los tags decodificados, rechaza unknown/geometria/reproyeccion y calcula
el score lineal acordado. `PublishFiducialObservations()` publica exactamente
una vez cada resultado no vacio, solo con observaciones validas y ordenadas.
Reutiliza `TimestampToRosTime()` para que batch y `OrbKeyFrame` tengan el mismo
timestamp, convierte Rodrigues a quaternion normalizado y conserva el frame
optico efectivo. No repite PnP ni consulta un epoch mutable al publicar.

Con debug activo, `PublishFiducialDebugImage()` dibuja aceptados en verde y
rechazados en rojo, y publica la imagen anotada. El ejecutable separado
`fiducial_visualizer` recibe solo la imagen mas reciente, reinicia un timeout
de cinco segundos por defecto y es el unico propietario de `namedWindow`,
`imshow`, `waitKey` y `destroyWindow`. Sin `DISPLAY` ni `WAYLAND_DISPLAY` se
desactiva a si mismo. Un cierre normal o un `SIGKILL` del visualizador no puede
terminar el proceso `stereo`; este sigue procesando y publicando deltas.
Para evitar carreras de creacion de HighGUI, solo clasifica `user_close`
despues de haber observado la ventana visible al menos una vez; antes de eso
un valor transitorio cero de `WND_PROP_VISIBLE` no la destruye.

El launch de Dron elimina rutas `/snap/` del entorno de ambos ejecutables para
evitar cargar bibliotecas glibc incompatibles al abrir HighGUI.

Referencias:

```text
dron/orbslam3_ros2/src/stereo/stereo-slam-node.cpp
  -> StereoSlamNode::GrabStereo
  -> rg "StereoTrackingReceipt|navigation_state|F5B-TRACKING|FiducialWorkerLoop"
  -> aproximadamente lineas 250-720
dron/orbslam3_ros2/src/stereo/fiducial-detector.cpp
  -> FiducialDetector::Configure / FiducialDetector::Detect
  -> rg "solvePnPGeneric|SOLVEPNP_IPPE_SQUARE|reprojection_error"
dron/orbslam3_ros2/src/stereo/fiducial-visualizer-node.cpp
  -> FiducialVisualizerNode
  -> rg "FID-VISUALIZER|namedWindow|display_seconds"
```

## `PublishLocalPose(stamp, Tcw)`

Convierte `Tcw` a `Twc` y publica `geometry_msgs/PoseStamped` en `orbslam/pose_local`.

Convención:
- `Tcw`: cámara respecto al mundo local ORB.
- `Twc`: pose de cámara en mapa local ORB.

El `frame_id` publicado es `local_map_frame_`, normalmente algo como `dron_1_orb_map`.

## `PublishOrbMapDelta()`

Construye un `OrbMap` incremental con:

```cpp
BuildOrbMap(false, true)
```

Si no hay cambios en KFs ni MPs, no publica. Si hay cambios, publica en
`orbslam/orb_map_delta`. El delta no representa una secuencia cronologica de
KFs exclusivamente nuevos: tambien vuelve a incluir KFs existentes cuando
cambia su hash por pose, asociaciones u otro estado exportado. Por ello un KF
con ID menor puede aparecer en un delta posterior a otro con ID mayor.

Un dron quieto también puede producir deltas no vacíos. `HashMapPoint()` incluye
estadísticas mutables de tracking como `found_ratio`, observaciones y normal;
ORB-SLAM3 puede actualizarlas al seguir viendo el mismo mapa aunque no cree KFs
ni MPs. El intento periódico sin entidades se descarta antes del publisher, por
lo que un flujo wrapper-servidor visible siempre contiene al menos una entidad,
pero no implica geometría nueva. Esta distinción debe conservarse al extender
el flujo principal: cambios estadísticos raw no deben activar automáticamente
todos los consumidores derivados.

## `GetFullMapServiceCallback(...)`

Devuelve un snapshot completo:

```cpp
BuildOrbMap(true, true)
```

Uso principal:
- el servidor puede recuperar el mapa completo si perdió deltas o si entra un epoch nuevo.
- la reconciliacion puede incorporar en bloque KFs antiguos que no llegaron en
  deltas previos; el consumidor no debe interpretar el orden de llegada como
  orden temporal completo.

## `BuildOrbMap(full_snapshot, update_cache)`

Función central de exportación.

Hace:

1. Asegura que el epoch/mapa activo está actualizado.
2. Crea `orbslam3_msgs/msg/OrbMap`.
3. Rellena metadatos:
   - `drone_id`;
   - `drone_name`;
   - `map_frame`;
   - `map_sequence`;
   - `map_epoch`;
   - cámara estéreo.
4. Recorre `m_SLAM->GetAllMapPoints()`.
5. Filtra puntos que no pertenecen al mapa activo.
6. Para snapshot completo envía todos los puntos válidos.
7. Para delta envía solo puntos nuevos/cambiados o marcados bad.
8. Recorre `m_SLAM->GetAllKeyFrames()` con la misma lógica.
9. Actualiza caches de hashes si `update_cache=true`.

## `FillMapPointMsg(pMP, mp_msg)`

Exporta un `ORB_SLAM3::MapPoint` a `OrbMapPoint`.

Campos rellenados:
- ID local;
- posición local ORB;
- descriptor representativo;
- `is_bad=false`;
- número de observaciones;
- `found_ratio`;
- normal;
- distancias invariantes;
- KF de referencia;
- lista de observaciones KF-feature.

Importancia:
- El servidor usa estos datos para fused landmarks, subcloud matching, score y covisibilidad.

## `FillKeyFrameMsg(pKF, kf_msg)`

Exporta un `ORB_SLAM3::KeyFrame` a `OrbKeyFrame`.

Campos rellenados:
- ID local;
- pose `Twc`;
- timestamp;
- keypoints;
- descriptores;
- `u_right`;
- depth;
- IDs de MapPoints asociados por feature;
- covisibilidad;
- BoW vector;
- FeatureVector;
- parent/children;
- loop edges locales.

Importancia:
- Es la base para `GlobalKeyFrameDatabase`, `GlobalORBMatcher`, loops y optimización global/local.

## `HashMapPoint(pMP)`

Calcula un hash de estado del MapPoint.

Incluye:
- ID;
- posición cuantizada;
- descriptor;
- observaciones;
- found ratio;
- normal;
- distancias invariantes;
- KF de referencia;
- lista de observaciones.

Uso:
- decidir si el MapPoint debe publicarse en un delta.

## `HashKeyFrame(pKF)`

Calcula un hash de estado del KeyFrame.

Incluye:
- ID;
- pose cuantizada;
- asociaciones a MapPoints;
- covisibilidad;
- BoW;
- FeatureVector;
- stereo info;
- parent/children;
- loop edges locales.

Uso:
- decidir si el KeyFrame cambió suficientemente para ser reenviado.

## `UpdateMapEpochFromCurrentMap()`

Detecta si ORB-SLAM3 cambió de mapa/submapa.

Mecanismos:
- compara puntero del mapa activo;
- calcula firma por keyframes: número, mínimo ID, máximo ID;
- detecta resets donde el puntero no cambia pero el primer KF salta.

Si detecta cambio:
- incrementa `map_epoch_`;
- limpia caches de hashes;
- resetea `map_sequence_`;
- fuerza publicación inmediata desde `GrabStereo()`.

## `GetCurrentMapPointerFromKeyFrames()`

Busca el KF válido más reciente y devuelve su `GetMap()`.

Uso:
- inferir mapa activo cuando no hay API directa suficiente.

## `KeyFrameBelongsToMap(...)` y `MapPointBelongsToMap(...)`

Filtran KFs/MPs para no mezclar datos de mapas internos distintos de ORB-SLAM3.

Esto es fundamental para no contaminar un epoch con datos de otro.

## `LoadCameraInfoFromSettings(...)`

Carga intrínsecos y baseline desde YAML.

Primero intenta formato ORB-SLAM3:

```text
Camera.fx
Camera.fy
Camera.cx
Camera.cy
Camera.bf
Camera.width
Camera.height
```

Luego fallback a `LEFT.K`, `LEFT.width`, `LEFT.height`.

## Riesgos al modificar

- No romper identidad `(drone_id, map_epoch, local_id)`.
- No cambiar convención de pose sin actualizar servidor y corrector.
- No quitar BoW/FeatureVector: son necesarios para matching global.
- No publicar `MapPoints` de mapas inactivos.
- No usar score global aquí.

## Perfil de recursos 3G

El wrapper no cambia su contrato de deltas/snapshots, pero el launch multi-dron
lo alimenta con un perfil medido:

```text
imagen estereo: 480x360 a 20 Hz
features: 900
vocabulario: ORBvoc_L5.txt en multi-dron
allocator: MALLOC_ARENA_MAX=2
```

La calibracion se valida al arrancar con `[CALIB0-WRAPPER-INIT]` y debe mostrar
`camera_valid=true`, ancho/alto 480x360 y baseline estimada 0.057 m. El
vocabulario L5 no cambia el mensaje ROS, pero reduce la memoria DBoW2 privada
por proceso. El vocabulario L6 completo sigue disponible mediante
`orb_vocabulary_path` y debe utilizarse como referencia en futuras pruebas de
relocalizacion/loop de maxima fidelidad.

## Build recomendado si se modifica

El fuente completo esta disponible en `orbslam3_ros2/`, aunque el nombre ROS
declarado en `package.xml` es `orbslam3`. Tras modificarlo, compilar con:

```bash
./codex/herramientas/build_selected_packages.sh orbslam3
```

Compilar tambien los consumidores si cambia algun contrato de mensajes.

La recuperacion de 2026-08-05 se valido con build aislado de `orbslam3` con
codigo 0 y comprobando que el binario `stereo` contiene `BuildOrbMap`,
`PublishOrbMapDelta`, `GetFullMapServiceCallback`, `HashMapPoint`,
`HashKeyFrame` y `UpdateMapEpochFromCurrentMap`.

## Diagnostico cruzado F5H

El cruce ampliado añade `DiagnosticControlState` y los modos
`gt_20_delay_pvgt_rpred_omegapred`, `..._omegagt`,
`gt_20_delay_pvgt_rgt_omegapred` y `..._omegagt`. P/v/R/omega se seleccionan
por canal y `[F5H-PHASE-PUBLISH]` registra `p_pred/p_gt_now/p_used`,
`v_pred/v_gt_now/v_used`, fuentes, edades y skew. Localizacion:
`navigation-state-estimator.hpp` -> `SelectDiagnosticControlState` y
`gt-timing-diagnostic-node.cpp` -> `UsesGtPositionNow`. Las pruebas 288-291
demuestran que solo las ramas con omega predicha fallan bajo delay; el bloque
sigue siendo instrumentacion temporal, no control productivo.

`GtTimingDiagnosticNode` conserva pose y omega GT recientes antes del
downsample visual. Los modos `gt_20_delay_rpred_omegagt`,
`gt_20_delay_rgt_omegapred` y `gt_20_delay_rgt_omegagt` seleccionan R/omega de
forma independiente con `SelectDiagnosticAngularState`; traslacion y velocidad
lineal permanecen en el predictor. `[F5H-PHASE-PUBLISH]` registra fuentes,
estados predicho/GT/usado, edades locales y skew fisico.

Referencia: `src/stereo/gt-timing-diagnostic-node.cpp` -> buscar
`UsesGtOrientationNow` y `UsesGtAngularVelocityNow`; selector en
`src/stereo/navigation-state-estimator.hpp` -> buscar
`SelectDiagnosticAngularState`. Todo este bloque es laboratorio temporal F5H.

## Predictor dinamico temporal F5H

`navigation-state-estimator.hpp/.cpp` -> `BodyTorqueDynamicPredictor` conserva
un buffer de torque corporal y propaga `R,omega` con timestamps reales mediante
`J*omega_dot=tau-omega x (J*omega)`. `gt-timing-diagnostic-node.cpp` consume
`control/tray/torque`, carga `fisico.total.matriz_inercia` y expone los modos
`dynamic_292` a `dynamic_295`; `[F5H-DYNAMIC-PREDICT]` registra horizonte,
torque, omega dinamica y truth GT. Es instrumentacion temporal.

La prueba 292 usa `J=diag(1e-4)` y falla en decimas: el modelo predice cientos
de rad/s ante milinewton-metro mientras la planta mide alrededor de 1 rad/s.
Con la J compuesta `diag(0.00803107,0.00803107,0.015805)`, las pruebas 296,
297, 293, 298, 294 y 295 completan entre `54.62` y `55.12 s`, con RMSE omega
`0.00255-0.00557 rad/s`, mismatch maximo `0.777 %` y energia total negativa.
El predictor angular queda validado con delay fijo hasta 294. En 295,
`UsesCrossDiagnostic()` es falso y `DeliveryDelay()` devuelve cero: sus 31 ms
medios validan estado completo sin delay añadido, no bajo los ~110 ms de
296-294. Quedan pendientes timing/jitter medido y ORB real.

`dynamic_299` reutiliza el estado completo dinamico de 295, pero selecciona
`TracePeriods()` y `TraceDelays()` de 268. No activa overrides GT actuales:
p/v proceden del predictor y R/omega de `BodyTorqueDynamicPredictor`; GT queda
solo como truth. El GTest `CompositeInertiaHandlesIrregularOrbTimingTrace`
cubre J compuesta, periodos irregulares y continuidad del buffer de torque.

La prueba 299 no valida esta rama con jitter: mantiene cobertura completa del
buffer (`missing=false`) y no pierde fuente, pero los periodos de hasta
`0.12 s` generan 48 estados `DEGRADED_DT`; con edad visual maxima `0.20 s`, el
lazo acumula `+0.0141 J` y falla tras `16.94 s`. Los rechazos visuales aparecen
despues del crecimiento. `dynamic_299` queda como laboratorio diagnostico; no
se ha conectado el predictor a `StereoSlamNode` productivo.

Los modos de laboratorio `dynamic_303` a `dynamic_306` reutilizan la traza de
299 para cruzar p/v y estado angular. 303 usa p/v GT actuales con angular
dinamico; 304 inicializa R/omega desde GT interpolado en `t_k` y propaga hasta
now solo con torque/J; 305 usa p/v predichas con angular GT actual; 306 usa GT
actual completo como sanity. `GtTimingDiagnosticNode` conserva buffers GT
acotados de pose y twist; la interpolacion 304 es retrospectiva, no espera
muestras y rechaza instantes sin bracket. Buscar `UsesTraceTiming`,
`InterpolateGtAt` y `[F5H-GT-TK-INVALID]`. Es instrumentacion retirable F5H y
no modifica `StereoSlamNode` productivo.

Resultado 303-306: 303 y 306 completan; 304 y 305 fallan. El cruce identifica
p/v predichas como causa principal bajo jitter: angular dinamica funciona con
p/v GT, pero angular GT(now) no salva p/v predichas. La interpolacion 304 fue
valida y el torque estuvo cubierto, aunque GT(t_k)+dinamica no reprodujo la
omega actual; esa rama queda diagnostica y no validada. Ninguno de estos modos
entra en la ruta productiva.

## Ruta dinamica productiva temporal

`src/stereo/stereo-slam-node.cpp` -> `StereoSlamNode`, buscar
`navigation_prediction_mode`, `ResetDynamicNavigationState`,
`[F5H-PRODUCTIVE-MEASUREMENT]` y `[F5H-PRODUCTIVE-PREDICT]`. En modo
`dynamic`, la medida O aceptada inicializa p/R,
`CausalLinearVelocityEstimator` aporta v en `t_k`, `OrbPosePredictor`
aporta omega causal y los predictores de torque/thrust propagan un estado comun
hasta el tick. Los callbacks conservan los stamps de las ordenes; un intervalo
no cubierto emite `[F5H-PRODUCTIVE-MISSING]` e invalida la fuente local.

El modo `legacy` permanece por defecto. La prueba 318 equivalente encuentra
un arranque con buffer no vacio cuya unica orden es posterior a la base; no
existe cobertura causal y el bootstrap de buffer vacio no aplica. La ruta no
se ha validado todavia con ORB real.

`navigation-state-estimator.hpp/.cpp` -> `ActuationCoverage`,
`CoverInterval` y `ActuationCoverageStatusName`: torque y thrust distinguen
`EMPTY`, `MISSING_PREFIX` y `FULL`; una orden conocida se mantiene por ZOH
sin fingir huecos internos. `StereoSlamNode` crea
`[F5H-ACTUATION-SEED]` cero al cold start demostrado y
`ResetDynamicNavigationState` conserva los buffers físicos.

Limitacion descubierta en 318R: `AddTorque/AddThrust` podan actualmente todas
las muestras anteriores a `max_history_sec`. Tras una espera larga, eso puede
eliminar el unico predecesor ZOH justo al llegar la primera orden. Debe
conservarse una muestra anterior al corte antes de validar la ruta.

La implementacion vigente poda con `while (size > 1 && sample[1].stamp <=
cutoff) pop_front()`: conserva una predecesora y todas las recientes.
318R2/319R validan cobertura sin crecimiento indefinido. 320R confirma
`mode=dynamic` y cero missing productivo, pero no valida el comportamiento de
control: ORB gobierna antes de la frontera acordada y el dron no alcanza la
pose de aproximacion.

Auditoria de `reference_kf` post-342R:

- `stereo-slam-node.cpp` -> `PublishNavigationState` -> localizar con
  `rg "raw_o_t_body|F5H-REF-SWITCH-TRACE"`;
- `raw_o_t_body` se deriva de `pose_result.o_t_camera` y vive en O, aunque la
  metadata registre la KF activa;
- `navigation-state-estimator.cpp` -> `OrbPosePredictor::UpdateMeasurement`
  -> localizar con `rg "last_raw_measurement_|raw_history_advanced"`;
- el baseline raw solo avanza con movimiento plausible; un rechazo lo retiene;
- `F5H-REF-SWITCH-TRACE`, solo con `debug_orb_control_state`, registra durante
  la ventana post-KF el stamp previo, accion `ADVANCE|KEEP`, raw/O steps,
  innovaciones y consumidor angular. No cambia gates ni estado productivo;
- `linear_source=MIDPOINT_DYNAMIC` identifica la velocidad productiva;
  `linear_mode=THREE_SAMPLE_PREDICTED` es diagnostico y no implica una
  regresion productiva.

Resultado 344: `CROSS_REFERENCE_RAW_HISTORY` descartado y
`STALE_RAW_HISTORY` confirmado con GT gobernando. La recuperacion vigente en
`OrbPosePredictor::UpdateMeasurement` detecta `raw_dt` mayor que
`raw_dt_max_degraded_sec`, conserva el rechazo del delta actual, rebasa solo
`last_raw_measurement_/last_raw_stamp_sec_` e invalida la derivada angular raw
anterior. El siguiente frame puede volver a producir un delta comparable. La
telemetria usa `REBASE`; pose O, predictor dinamico y buffers no se reinician.

Validacion: 345 confirma en GT+shadow raw_dt maximo `0.201 s`, cinco `KEEP`,
dos `REBASE` y solo dos `SUSPICIOUS`. 346 mantiene esa higiene con ORB real
(`KEEP=0`, `REBASE=1`), pero el segundo tramo activa fallback con tracking aun
OK y el tracking se pierde mas tarde. El rebase stale queda validado; la
inestabilidad residual en dos fachadas pertenece a otro mecanismo.

Diagnostico post-346: `[F5H-ORB-STATE-REJECTED]` diferencia
`PREDICTION_INVALID` y `PREDICTOR_UNHEALTHY` en la medida. En la publicacion a
50 Hz, `[F5H-NAV-VALIDITY-TRACE]` explica una prediccion no consumible mediante
base dinamica, gravedad, intento y validez angular/translacional. Ambas trazas
son observacionales y no modifican flags, pose, velocidad ni resets vigentes.

En 348, el primer fallback precede en unos 19.54 s a la perdida real de
tracking y no coincide con `PREDICTOR_UNHEALTHY`: es un pulso transitorio de
validez local/continuidad/reference. La divergencia lineal y angular ya habia
crecido durante unos 40 s. El historial raw permanece sano, con `raw_dt`
siempre positivo y acciones `ADVANCE/KEEP/REBASE` coherentes.

## Evidencia visual ORB de 5H

La telemetría opcional se activa con `debug_orb_visual_evidence` y escribe en
`orb_visual_evidence_output_dir`. `StereoSlamNode::WriteVisualEvidence`
consume exclusivamente el `StereoTrackingReceipt` del callback actual y
genera `{drone_name}_orb_visual_evidence.csv` con:

- stamps de imagen/llegada, frame, tracking y reference KF;
- features, candidatos, inliers y ratio real;
- depth/disparidad y cobertura espacial 4x3 de inliers;
- `Tcr` cruda del mismo frame.

Está desactivada por defecto, no publica estado alternativo y no interviene en
gates, predictor, mux ni control. Las métricas geométricas viven en
`visual-evidence-metrics.cpp`; localizar con
`rg "WriteVisualEvidence|debug_orb_visual_evidence|ComputeVisualEvidenceMetrics"`.
