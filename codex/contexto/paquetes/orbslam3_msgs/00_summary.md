# 00_summary — orbslam3_msgs

Resumen: Paquete de mensajes ROS 2 que define `OrbMap`, `OrbMapPoint`, `OrbKeyFrame` y mensajes auxiliares; contrato entre wrapper, servidor y corrector.

Interfaces definidas (ejemplos): `OrbMap`, `OrbMapPoint`, `OrbKeyFrame`,
`MapCorrection`, `CorrectedKeyFrameArray`, `FiducialTagConfig` y
`GetFiducialConfig`. Fase 4E añade `FiducialTagObservation` y
`FiducialKeyFrameObservations` para transportar un batch visual por KF.

Fase 5B añade `NavigationState`: una muestra por frame con tracking, source,
epoch/sample, reference KF, `Tcr`, `O_T_B`, campos futuros `W_T_B`/velocidad y
bits de validez explícitos. En 5B solo local/continuidad pueden ser válidas;
global y velocidad permanecen inválidas. Las copias Dron/Servidor incluyen el
mensaje en CMake y deben seguir siendo byte a byte idénticas.

5D añade `GlobalKeyFramePose` y `GetGlobalKeyFramePose` con identidad completa,
status, `pose_revision` y `W_T_KF`. 5E amplía `NavigationState` con estado
global `INVALID/PROVISIONAL/AUTHORITATIVE` y revisión. En 5C-5F solo
`AUTHORITATIVE` activa `global_valid`; los goals absolutos siguen deshabilitados.

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
