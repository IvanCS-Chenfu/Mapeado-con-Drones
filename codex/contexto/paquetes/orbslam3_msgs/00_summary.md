# 00_summary — orbslam3_msgs

Resumen: Paquete de mensajes ROS 2 que define `OrbMap`, `OrbMapPoint`, `OrbKeyFrame` y mensajes auxiliares; contrato entre wrapper, servidor y corrector.

Interfaces definidas (ejemplos): `OrbMap`, `OrbMapPoint`, `OrbKeyFrame`,
`MapCorrection`, `CorrectedKeyFrameArray`, `FiducialTagConfig` y
`GetFiducialConfig`. Fase 4E añade `FiducialTagObservation` y
`FiducialKeyFrameObservations` para transportar un batch visual por KF.

Uso: publicadas por `orbslam3_ros2`, consumidas por `orbslam3_server` y `orbslam3_multi`.

Fase 4D usa `FiducialTagConfig` para `tag_id/size_m` y
`GetFiducialConfig` para familia, refinamiento, solver, umbral de reproyeccion
y lista de tags. Las copias Dron/Servidor deben ser byte a byte identicas.

`FiducialKeyFrameObservations` conserva `drone_id`, `drone_name`,
`map_epoch`, `local_keyframe_id`, `source_frame_id`, timestamp/frame optico y
un vector ordenado de tags. Cada tag lleva `camera_T_tag`, quality y metricas
de reproyeccion, area y ambiguedad; no contiene semantica de objeto ni GT.

Reglas: no cambiar nombres/campos sin actualizar wrapper, servidor y corrector.

Detalles en los MDs del paquete (definiciones de mensajes y servicios).
