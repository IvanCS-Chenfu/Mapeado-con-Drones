# 00_summary — orbslam3_ros2

Resumen: Wrapper ROS 2 para ORB-SLAM3; publica `pose_local`, `orb_map_delta` y ofrece `GetOrbMap`.

Estado del fuente: el arbol completo vuelve a estar disponible en
`orbslam3_ros2/`. Se recupero el snapshot base del commit
`00c54335ccc010d74c1e24e336aa817604124947` de `zang09/ORB_SLAM3_ROS2` y se
superpusieron las ultimas versiones locales conservadas por VS Code Local
History de los nueve archivos personalizados del workspace. El paquete ROS
declarado sigue llamandose `orbslam3`.

Interfaces:
- Publishes: `orbslam/pose_local`, `orbslam/orb_map_delta`.
- Services: `orbslam/get_full_map`.

Ejecutables/nodos: `StereoSlamNode`.

Parámetros relevantes: `drone_id`, `local_map_frame`, `delta_publish_period_frames`, `use_sim_time`.

Relación: alimentado por `simulacion_dron` (cámaras), usa `ORB_SLAM3`, define mensajes en `orbslam3_msgs` y es consumido por `orbslam3_server`.

Semantica relevante: `orb_map_delta` contiene entidades nuevas o modificadas,
no una lista cronologica de KFs nuevos. `get_full_map` permite reconciliar
ausencias y puede devolver en bloque KFs anteriores.

Detalles en `stereo_slam_node.md` y demás MDs del directorio.
