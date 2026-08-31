# 00_summary — dron_individual

Resumen: Lógica por dron: control, generación de trayectorias (`TrayAction`), launches del wrapper ORB-SLAM3 y configuración física/vision.

Interfaces/Entradas: recibe GT y sensores simulados; action `TrayAction`.

Ejecutables principales: `gen_tray`, `control_calcular_fuerzas`, `aplicar_fuerzas_dron`, `control_dron`.

Config/Launch: `config/*.yaml`, `launch/` con `orbslam_use.launch.py`.

Relación: consume `lib_tray`, es lanzado por `simulacion_dron`.

Fase 5H hace que `gen_tray` y `control_calcular_fuerzas` consuman exclusivamente
`orbslam/navigation_state`. `navigation_state_mux` selecciona ORB o
`GT_FALLBACK` y publica el estado comun sin filtrar ni predecir. La estimacion
ORB corregida a 50 Hz llega ya preparada desde `orbslam3_ros2`; GT usa pose y
velocidad exactas. Tambien ofrece el servicio namespaced
`control/set_trajectory_active` para congelar la fuente durante cada goal.
`GT -> ORB` solo ocurre en frontera; una perdida permite `ORB -> GT` inmediata y
retiene GT hasta terminar sin cambiar el frame O activo. La cualificacion usa
tracking+anchor consecutivos, no errores frente a GT.

La prueba 253 demostro que un predictor uniforme en el mux desestabiliza GT; esa
responsabilidad ya no pertenece a este paquete.

`config/navigation_state.yaml` centraliza la frecuencia, limites SE(3),
probation angular moderada, calidad/plausibilidad raw, limites del bias y gate
de reference KF que consume el wrapper. Incluye tambien deadband/confirmacion
del bias, supresion por movimiento raw y aceleracion de decay de
`omega_motion`; el controlador no implementa esos filtros. El
master `debug_fase_5=false` bloquea toda la telemetria extensa de esta fase.
Con el master activo, `debug_orb_control_state` habilita telemetria causal en
wrapper y controlador sin modificar el control.

Con ese flag, `[F5H-PHASE-CONTROL]` registra cada tick de 50 Hz: sample y
timestamps de recepción, `R_act/R_des`, omega O/body, `Omega_des`, `er/ew` y
los vectores `tau_er/tau_ew/tau_feedforward/tau_gyro/tau_total`. Es
observabilidad únicamente; publicaciones, ganancias y ecuaciones permanecen.
El ensayo dinamico 292 solo añade timestamp al torque corporal ya publicado y
pasa `physical.yaml` al nodo diagnostico; no modifica su valor. Tras el fallo
con la J nominal, `fisico.total.matriz_inercia` contiene la inercia compuesta
del modelo de 1.4 kg:
`diag(0.00803107,0.00803107,0.015805) kg*m^2`. Controlador y predictor consumen
el mismo tensor; los tensores por enlace de Gazebo permanecen intactos.
La prueba 264 muestra que `tau_er` inyecta energia en `80.9 %` del tramo
post-handoff y domina el fallo, mientras `tau_ew` conserva damping neto. En
265 el controlador permanece intacto y consume el `NavigationState`
extrapolado con la edad visual local corregida en `orbslam3_ros2`; el torque
total inyecta `+0.145081 J`, confirmando que el consumidor no debe compensar
la fase y que la correccion pendiente pertenece al estimador ORB.
Para 266 el controlador sigue intacto: consume la pose visual reanclada y la
omega de movimiento publicadas por `orbslam3_ros2`, sin filtros compensatorios.
La ventana comun 266 deja torque total disipativo (`-0.001945 J`), aunque el
ciclo tardio moderate vuelve a crecer y fuerza fallback; no se cambian gains.

Los goals absolutos pueden usar global valida, un frame C_T_W cacheado del mismo
epoch o el fallback temporal Fase 5. El perfil general conserva fallback
desactivado; `multi_dron.launch.py` lo activa explicitamente para la validacion.

Para pruebas dirigidas existe
`use_legacy_gt_goal_policy_for_simulation=false`. Al activarlo se omite por
completo el gate de `NavigationState` y se aceptan goals relativos/absolutos
mediante el control GT legacy. El default conserva la política 5B y los
absolutos deshabilitados.

Las calibraciones `config/orbslam/orbslam_mono.yaml` y
`config/orbslam/orbslam_stereo.yaml` fijan `loopClosing: 0`. ORB-SLAM3 conserva
tracking, `LocalMapping`, fusión local, BoW y covisibilidad, mientras el
servidor mantiene la autoridad sobre loops y optimizaciones globales.

Perfil multi-dron: camaras 480x360 a 20 Hz, calibracion coherente con baseline
0.057 m y 900 features. `generar_dron.launch.py` acepta
`orb_vocabulary_path`; standalone y Simulacion usan por defecto el
`ORBvoc.txt` completo instalado desde el tarball versionado. L5 solo puede
seleccionarse mediante override explicito.

La configuracion vigente separa `physical.yaml`, `control.yaml`,
`trajectory.yaml`, `actuators.yaml`, `vision.yaml` y `calibration.yaml`.
`hardware.yaml`, `tray_dron.yaml` y `usar_veltrap` ya no forman parte del
runtime. Standalone usa `use_sim_time=false`; Simulacion lo sobrescribe a
`true`. Los procesos ORB limitan arenas glibc con `MALLOC_ARENA_MAX=2`.

`calibration.yaml` expresa un SE(3) `B_T_C` completo. Para la camara optica
frontal usa traslacion `(0.10,0.03,0.03) m` en body y
`RPY=(-90,0,-90)` bajo `Rz*Ry*Rx`; el wrapper puede aplicar
`W_T_C * inverse(B_T_C)` sin permutar los ejes world/control.

Detalles en `launches.md`, `control.md` y archivos de config del paquete.

`orbslam_use.launch.py` y `generar_dron.launch.py` propagan
`orb_navigation_prediction_mode` al parametro productivo
`navigation_prediction_mode` de `StereoSlamNode` y le entregan
`physical.yaml`. El default productivo es `dynamic`; `legacy` solo permanece
como opcion explicita de regresion.

`orbslam_use.launch.py` carga ahora los YAML base antes de `stereo_params`,
de modo que el override explicito del launch prevalezca sobre el YAML.

5J retiro `f5h_gt_timing_mode`, forcing de fuente, shadow manual,
`control/activate_orb_shadow` y overrides parciales p/v/R/omega. El topic
`control/orb_authority_confirmed` representa autoridad ORB real del mux. GT
normal no se remuestrea ni filtra y solo permanece en `GT_FALLBACK`.
