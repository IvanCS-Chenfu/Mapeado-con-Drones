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
