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
`OrbPosePredictor`, dentro del mismo wrapper, publica a 50 Hz un estado SE(3)
corregido gradualmente con limites de innovacion, velocidad y aceleracion.
Pose y velocidades proceden exactamente del mismo estado corregido.

La validacion dinamica 256 demuestra una limitacion vigente: el gate duro
angular de `0.35 rad` permite que una innovacion aislada de `0.125261 rad` se
convierta en un paso publicado de `0.119002 rad`. El handoff GT->ORB fue
continuo, pero drone2 perdio tracking `0.793 s` despues. La salida actual no es
aun apta para control sostenido; falta acordar confirmacion temporal separada
para innovaciones angulares moderadas.

Relación: alimentado por `simulacion_dron` (cámaras), usa `ORB_SLAM3`, define mensajes en `orbslam3_msgs` y es consumido por `orbslam3_server`.

Semantica relevante: `orb_map_delta` contiene entidades nuevas o modificadas,
no una lista cronologica de KFs nuevos. `get_full_map` permite reconciliar
ausencias y puede devolver en bloque KFs anteriores.

Detalles en `stereo_slam_node.md` y demás MDs del directorio.
