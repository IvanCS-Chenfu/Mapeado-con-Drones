# 00_summary - orbslam3_ros2

Resumen: Wrapper ROS 2 para ORB-SLAM3; publica `pose_local`,
`navigation_state`, `orb_map_delta` y ofrece `GetOrbMap`.

Estado del fuente: el arbol completo vuelve a estar disponible en
`orbslam3_ros2/`. Se recupero el snapshot base del commit
`00c54335ccc010d74c1e24e336aa817604124947` de `zang09/ORB_SLAM3_ROS2` y se
superpusieron las ultimas versiones locales conservadas por VS Code Local
History de los nueve archivos personalizados del workspace. El paquete ROS
declarado sigue llamandose `orbslam3`.

Interfaces:
- Publishes: `orbslam/pose_local`, `orbslam/orb_map_delta` y, solo con debug,
  `orbslam/fiducial_debug/image`; desde 4E publica tambien
  `orbslam/fiducial_keyframe_observations`.
- Services: `orbslam/get_full_map`.
- Clients: `/global_mapping/get_fiducial_config`.

Ejecutables/nodos: `StereoSlamNode` y `fiducial_visualizer`.

Parámetros relevantes: `drone_id`, `local_map_frame`, `delta_publish_period_frames`, `use_sim_time`.

Fase 4D consume el recibo exacto de 4C y procesa cada KF en una cola de cuatro
elementos con worker unico. `FiducialDetector` usa AprilTag 36h11, SUBPIX e
IPPE_SQUARE y conserva pose, reproyeccion, area, ambiguedad y calidad. Las
observaciones validas se publican en un unico batch no vacio por KF, ordenadas
por `tag_id`, con QoS reliable/volatile KeepLast(32).

Con `debug_fiducial_visualization=true`, el wrapper anota todos los tags
decodificados y publica la ultima imagen con QoS best-effort/KeepLast(1). El
ejecutable ROS independiente `fiducial_visualizer` es el unico propietario de
HighGUI y mantiene la ventana durante `debug_fiducial_display_seconds=5.0`.
Cerrar o matar ese proceso no termina `stereo`; el entorno launch elimina
rutas Snap de ambos procesos.

Fase 2 añade `debug_architecture_telemetry=false`. Cuando Simulacion activa el
master de `system_architecture`, el wrapper emite eventos ligeros y muestreados
en `/system_architecture/activity` al consumir el par estereo; no publica ni
serializa esa telemetria con el debug apagado.

Fase 5B añade `NavigationStateEstimator`: compone `O_T_B` intra-epoch con
reference KF real/`Tcr`, reancla cambios de referencia sin salto y marca
local/continuidad inválidas al perder tracking. 5D-5E añaden cliente de
`GetGlobalKeyFramePose`, push dirigido y W
`INVALID/PROVISIONAL/AUTHORITATIVE` con revisión; stale y epoch mismatch se
descartan y W nunca mueve O. `NavigationStateEstimator` confirma una cadena
geometrica aunque cambie el ID del reference KF; `local_t_camera` solo enlaza
cambios plausibles y Tcr conserva la autoridad local principal.
`OrbPosePredictor`, dentro del mismo wrapper, publica a 50 Hz un estado SE(3).
En rotacion separa la orientacion absoluta visual aceptada del movimiento entre
medidas (`omega_motion`): SMALL/plausible reancla `pose_` en `t_visual`,
MODERATE_CONFIRMED con raw plausible reancla tambien por completo y el resto
avanza como PREDICT_ONLY. La correccion de pose no se convierte en velocidad
fisica; `omega_motion + omega_bias` queda para velocidad y una unica
propagacion temporal. El dt raw se
clasifica `GOOD/DEGRADED/INVALID`, el movimiento
`PLAUSIBLE/DEGRADED_DT/SUSPICIOUS/REJECTED` y el residual absoluto conserva
`SMALL/MODERATE_PENDING/MODERATE_CONFIRMED/MODERATE_DISCARDED/REJECTED_EXCESSIVE`.
SMALL es fijo. La correccion de bias usa deadband con histeresis, confirmacion
temporal y estados `OFF/PENDING/ACTIVE/DECAY`; durante movimiento raw
significativo se lleva hacia cero. Si raw pasa a `SUSPICIOUS/REJECTED`,
`omega_motion` decae de forma continua en vez de conservarse o cortarse.

El parametro `debug_orb_control_state=false` habilita, solo para diagnostico,
`[F5H-ORB-MEASUREMENT]` y `[F5H-ORB-PUBLISH]`. Separan medida raw, innovacion,
correccion aplicada, paso realmente publicado, omega y edad del estado. Los
umbrales se cargan desde `dron_individual/config/navigation_state.yaml`; GT
permanece exacto fuera de este predictor.

El mismo flag añade los marcadores sin throttle
`[F5H-PHASE-MEASUREMENT]` y `[F5H-PHASE-PUBLISH]`: orientación raw,
`omega_raw/motion/bias/total`, input Gazebo, recepción ROS, publicación y
sample. Los dominios de reloj se etiquetan explícitamente; el wrapper no resta
input Gazebo de tiempo ROS.

Validacion vigente: builds de los tres consumidores correctos, 46/46 GTests y
7/7 tests del analizador.
La prueba 262 repite el hover ORB: `omega_bias` permanece en cero y el decay raw
actua, pero `omega_motion` oscila hasta unos `0.617 rad/s`; ORB gobierna unos
5.92 s y el estimador fuerza fallback unos 0.54 s antes de tracking 2->3. La
prueba 263 descubre el doble reloj y queda sin diagnostico. La 264 usa el
puente corregido: raw sigue al GT con unos 80 ms, pero la pose/control angular
queda fuera de fase y el termino proporcional de orientacion inyecta energia.
La correccion aplicada en 265 usa la edad local desde el ingreso del
callback para construir un target en reloj visual y extrapolar una sola vez;
reutiliza el clamp de 0.10 s y no fusiona directamente la orientacion raw.
Cumple su contrato temporal, pero no estabiliza el hover: `visual_q -> base_q`
crece hasta `0.339 rad`, muy por encima del paso de prediccion, y `tau_er`
inyecta `+0.160266 J`.
La prueba 266 valida el reanclaje visual SMALL: reduce `tau_er` un `98.7 %` en
ventana comun y extiende ORB a `8.06 s`. Aun falla porque la correccion moderate
de `0.015 rad` deja residual y termina en predict-only/rechazo/fallback. La
prueba 267 ensaya el anclaje moderate completo: el error after queda a cero,
pero ORB vuelve a `5.56 s` y la energia dañina se acumula mas deprisa tras el
primer anclaje. Un caso confirmado con raw rechazado revelo que ambos gates
eran independientes; el codigo vigente exige ahora raw plausible y pasa
46/46 GTests, pero esa reparacion posterior aun no tiene simulacion.
La prueba 268 valida ya ese gate: no hay confirmed anchors con raw rechazado,
pero el unico anclaje plausible de `0.057317 rad` precede `+0.046416 J` en
`0.88 s` y fallback. El anclaje completo queda descartado como politica
moderate; el siguiente diseño acordable es un residual SO(3) persistente y
gradual, aun no implementado.
Las pruebas 269-272 añaden temporalmente el ejecutable de laboratorio
`gt_timing_diagnostic`: reutiliza el `OrbPosePredictor` con pose GT perfecta y
modos 50 Hz, 20 Hz, 20 Hz +80 ms y traza determinista de 268. A es estable;
B ya falla sin error geometrico ORB y C/D lo agravan. Esto localiza la causa
principal en el pipeline temporal/predictor. El modo queda `off` por defecto y
marcado para retirar.
La bateria E/F/G 273-275 extiende ese laboratorio con omega GT sincronizada.
Las tres variantes completan y son disipativas: predictor actual con omega GT,
hold angular y extrapolacion SO(3) directa. Queda aislada como causa principal
la derivacion/filtrado de `omega_motion`; no fallan el hold ni la formula de
propagacion cuando pose y omega son coherentes.
No ejecutar recorridos largos mientras falle el hover.

Relación: alimentado por `simulacion_dron` (cámaras), usa `ORB_SLAM3`, define mensajes en `orbslam3_msgs` y es consumido por `orbslam3_server`.

Semantica relevante: `orb_map_delta` contiene entidades nuevas o modificadas,
no una lista cronologica de KFs nuevos. `get_full_map` permite reconciliar
ausencias y puede devolver en bloque KFs anteriores.

Detalles en `stereo_slam_node.md` y demás MDs del directorio.
