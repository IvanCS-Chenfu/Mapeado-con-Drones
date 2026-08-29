# multidron_gui_lib

Librería C++ de Fase 7 para la GUI multi-dron. Está preparada contra el `main` auditado en el commit `206e56892d5c852f9fab27459c59632bf237670b`.

## Entradas reales conectadas ahora

- `/global_sparse_cloud` (`sensor_msgs/msg/PointCloud2`), QoS reliable + transient local. Se consumen `x`, `y`, `z`, `score`, `drone_id`, `map_epoch_low`, `map_epoch_high`. El `rgb` actual del servidor se ignora deliberadamente: el color por score se calcula en la GUI.
- `/global_keyframes` (`visualization_msgs/msg/MarkerArray`), conservando pose, geometría y color publicados por el backend.
- `/<namespace>_<id>/orbslam/navigation_state` (`orbslam3_msgs/msg/NavigationState`) para pose/estado por dron. La GUI usa `w_t_body` solo cuando la pose global es válida y la fuente no es el fallback Ground Truth.
- `fiducial_config_path`: consume el YAML canónico de Fase 4 para visualizar los objetos fiduciales sin alterar `orbslam3_server`.

## Slots preparados, sin contrato ROS inventado

`GuiDataModel` incluye `TrajectoryVisual`, `VoxelVisual` y `TaskVisual`, pero `RosDataBridge` **no** crea topics ficticios de Fase 6. Cuando Fase 6 exista, Codex debe adaptar los contratos reales de `task_server`/`mission_msgs` a estas estructuras o ajustar las estructuras si el contrato final exige otros campos.

La trayectoria guardada es únicamente la trayectoria futura vigente por dron; un nuevo plan reemplaza al anterior. Los vóxeles conservan `unknown/free/occupied` y score. El renderer muestra la rejilla de todos los vóxeles y solo rellena los ocupados; free/unknown tienen relleno completamente transparente.

## Threading

ROS 2 escribe únicamente `GuiDataModel`. La UI toma snapshots inmutables `shared_ptr`. Ningún callback ROS toca widgets. La nube sparse no se copia a 30 Hz: el renderer detecta cambios por identidad de snapshot y actualiza el VBO solo cuando cambia esa capa o su estilo.

## Navegación 3D

- arrastre izquierdo: órbita;
- arrastre derecho/medio: pan;
- rueda: zoom;
- doble click izquierdo: reset de cámara;
- click: picking genérico e inspector.

## Build

Dependencias de sistema típicas en Ubuntu/Qt5:

```bash
sudo apt install qtbase5-dev libqt5opengl5-dev
```

Después, desde el workspace:

```bash
colcon build --packages-select multidron_gui_lib
```

Este ZIP se genera fuera de un entorno ROS 2/Qt completo, por lo que la compilación real debe ejecutarla Codex en el workspace objetivo antes de marcar ninguna subfase como conseguida.
