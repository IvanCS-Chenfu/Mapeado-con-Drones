# 00_summary - orbslam3_ros2

Resumen: Wrapper ROS 2 para ORB-SLAM3; publica `pose_local`,
`navigation_state`, `orb_map_delta` y ofrece `GetOrbMap`.

Con `debug_orb_visual_evidence=true`, escribe por dron un CSV del mismo frame
con tracking/reference KF, candidatos e inliers, depth/disparidad, cobertura
4x3 y `Tcr` cruda. Está desactivado por defecto y no modifica la salida de
navegación. `analyze_orb_visual_evidence.py` resume todos los frames y separa
las estadísticas con tracking `OK`.

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
El laboratorio F5H permite seleccionar de forma independiente p/v/R/omega
predichas o GT actuales mediante `DiagnosticControlState`; esta instrumentacion
esta apagada por defecto y marcada para retirar. Las pruebas 288-291 aislan el
fallo con delay en `omega_pred(now)`: las dos ramas que la usan fallan y las dos
con omega GT completan, incluido el sanity GT total. No es una ruta productiva.
El laboratorio añade tambien `BodyTorqueDynamicPredictor`: integra en body la
ecuacion rigida con buffer temporal de `control/tray/torque` y J configurable.
La prueba 292 con la J nominal `diag(1e-4)` falla de inmediato porque predice
cientos de rad/s frente a valores GT del orden de uno. Tras sustituirla por la
J compuesta `diag(0.00803107,0.00803107,0.015805)`, 296-298 y 293-295 completan
el hover con RMSE `0.00255-0.00557 rad/s`, mismatch menor de `0.78 %` y energia
total negativa. La clase angular queda validada en laboratorio con entrada GT y
delay fijo hasta 294. `dynamic_295` no activa el delay cruzado y valida estado
completo con edad media de 31 ms, no con los 80 ms añadidos. Sigue siendo una
ruta diagnostica, no productiva.
El modo `dynamic_299` añade timing/delay determinista de ORB al estado completo
dinamico, sin overrides GT actuales, para validar jitter antes de conectar la
ruta ORB real. La prueba 299 falla tras `16.94 s`: no hay fallback ni huecos
de torque, pero 48 intervalos `DEGRADED_DT`, edad maxima `0.20 s` y energia
positiva invalidan el estado completo bajo jitter. La ruta ORB no se integra.
Los modos `dynamic_303` a `dynamic_306` separan p/v de R/omega bajo esa misma
traza. 304 interpola GT retrospectivo en `t_k` desde buffers acotados, sin
espera ni uso de GT(now) como salida angular; todo el bloque sigue siendo
diagnostico y retirable.
303/306 completan y 304/305 fallan: el cruce localiza el problema principal en
p/v predichas bajo jitter. La propagacion desde omega GT(t_k) de 304 tampoco
queda validada y requiere revisar marco/alineacion antes de reutilizarla.

Los cruces 307/308 separan posicion y velocidad bajo angular GT actual. El
laboratorio incorpora `BodyThrustDynamicPredictor`, alimentado por el nuevo
topic sellado `control/tray/thrust`: integra p/v con masa compartida, gravedad,
dt reales y la orientacion angular dinamica durante cada intervalo. Los modos
309-312 y su telemetria son retirables y no forman parte aun de la ruta ORB.

`CausalLinearVelocityEstimator` conserva THREE_SAMPLE como diagnostico.
La ruta productiva usa `PredictMidpointDynamicVelocity`: deriva velocidad entre
dos posiciones visuales aceptadas en su midpoint, interpola R en SO(3), alinea
el reloj ROS y propaga causalmente hasta `t_k` con torque, thrust y gravedad O.
Solo una cobertura `FULL` actualiza la base productiva. 326-329 validan su
precision y 330/331 validan hover ORB real reproducible.

La integracion post-317 conecta esas clases a `StereoSlamNode` mediante
`navigation_prediction_mode=legacy|dynamic` (`legacy` por defecto). La rama
dinamica forma una base O comun, consume `control/tray/torque` y
`control/tray/thrust` sellados y publica p/v/R/omega propagados al mismo tick;
si falta cobertura invalida el estado para habilitar fallback. Compila y pasa
94/94 GTests, pero 318 detecta un hueco inicial de torque con muestra posterior
a la base. La rama no esta validada y ORB real no se ejecuto.

Post-318 se formaliza cobertura de actuacion
`EMPTY/MISSING_PREFIX/FULL`, seed cold-start cero, ZOH y persistencia de
buffers ante resets visuales. 98/98 GTests pasan. 318R descubre que la poda por
`max_history_sec` elimina el seed antes de la primera orden tras una espera
larga; queda un prefijo desconocido de 70 ms. La correccion pendiente debe
retener una muestra predecesora al horizonte.

La poda corregida conserva exactamente una predecesora ZOH y todas las
muestras recientes. Pasa 102/102 GTests y 318R2/319R sin missing. En 320R la
ruta `StereoSlamNode dynamic` se activa sin huecos, pero el dron no sigue la
aproximacion y termina cerca del suelo; la integracion ORB sigue no validada.

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
Desde 276, `omega_motion` usa dos incrementos espaciales SO(3) de tres poses
aceptadas, estima aceleracion entre midpoints y proyecta hasta `t_k` sin
pasa-bajos; la omega se mantiene entre medidas. Rechazos no contaminan el
historial y el epoch lo reinicia. 276-277 completan el hover de laboratorio con
pose GT 20 Hz y energia negativa; falta validar delay/jitter y ORB real.
La prueba 278 conserva el estimador y añade 80 ms: falla con edad visual media
`0.1129 s`, clamp `72.2 %`, RMSE `1.447 rad/s` y energia positiva. El siguiente
problema aislado es la compensacion desde `t_k` hasta `now`; 279-281 no se
ejecutan.
Las pruebas 282/284 descartan dos soluciones simples: `0.18 s` elimina el clamp
pero empeora, y aceleracion constante mejora RMSE/energia pero falla antes. El
flag `predict_angular_acceleration` queda `false` por defecto y solo el nodo GT
diagnostico lo activa; ORB productivo conserva propagacion constante.
Las pruebas cruzadas 285-287 seleccionan independientemente orientacion y omega
GT actuales y registran edad/skew y estados predicho/GT/usado. 286 con omega
predicha falla mucho antes que 285/287, pero 287 tambien falla porque p/v
lineales permanecen retrasadas; el aislamiento queda parcial.
No ejecutar recorridos largos mientras falle el hover.

Relación: alimentado por `simulacion_dron` (cámaras), usa `ORB_SLAM3`, define mensajes en `orbslam3_msgs` y es consumido por `orbslam3_server`.

Semantica relevante: `orb_map_delta` contiene entidades nuevas o modificadas,
no una lista cronologica de KFs nuevos. `get_full_map` permite reconciliar
ausencias y puede devolver en bloque KFs anteriores.

Detalles en `stereo_slam_node.md` y demás MDs del directorio.

Auditoria post-342R: la entrada denominada raw del predictor es
`raw_o_t_body`, ya expresada en O por `NavigationStateEstimator`; no se ha
encontrado un historico de pose raw almacenado en el frame de una KF. El
baseline `last_raw_measurement_/last_raw_stamp_sec_` solo avanza cuando
`raw_motion_plausible`; tras un rechazo puede quedar antiguo y hacer crecer
los siguientes `raw_dt/raw_step`. La telemetria diagnostica
`F5H-REF-SWITCH-TRACE` expone stamp previo, `ADVANCE|KEEP`, old/new ref y
efecto angular sin modificar la estimacion.

La prueba 344 con GT gobernando confirma que no existe mezcla geometrica de
KFs, pero si retencion indefinida del baseline raw en O: hasta `20.609 s` y
`2.741 m` raw frente a `0.025 m` en O. El codigo vigente rebasa solo el
historico raw cuando `raw_dt` supera el maximo degradado; ese delta permanece
rechazado/PREDICT_ONLY y la siguiente muestra empieza una comparacion nueva.
La accion se registra como `REBASE` y no resetea estado fisico ni dinamico.
345 valida este comportamiento en shadow: raw_dt maximo baja a `0.201 s`,
`KEEP 125->5` y `SUSPICIOUS 168->2`. En 346 ORB el historial sigue acotado,
pero el control entra en fallback antes de la perdida visual; por tanto la
correccion stale es vigente y correcta, aunque no resuelve por si sola el
movimiento ORB largo.
