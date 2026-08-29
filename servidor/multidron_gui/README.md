# multidron_gui

Ejecutable mínimo de Fase 7. El proceso mantiene Qt en el thread principal y ejecuta ROS 2 en un `MultiThreadedExecutor` independiente. Cerrar la ventana cancela únicamente el executor de la GUI y ejecuta `rclcpp::shutdown()` para este proceso; no existe ninguna llamada de parada al backend.

## Arranque

```bash
ros2 launch multidron_gui multidron_gui.launch.py \
  drone_count:=2 \
  use_sim_time:=true
```

El launch resuelve por defecto el YAML canónico:

```text
orbslam3_server/config/fiducial_objects.yaml
```

y usa las interfaces actuales auditadas:

```text
/global_sparse_cloud
/global_keyframes
/dron_1/orbslam/navigation_state
/dron_2/orbslam/navigation_state
...
```

## Integración posterior en simulación

Codex puede añadir `multidron_gui` al launch multi-dron y retirar RViz2 como frontend habitual. Este paquete no requiere RViz2 ni web.

No hay endpoints inventados para Fase 6: GO_TO, CAPTURE_SPARSE, CAPTURE_DENSE, task state, `TrajectoryPlan` y el mapa voxel ROS se conectarán cuando existan sus contratos reales. La UI/librería ya contiene las estructuras y layers para trayectoria, tareas y vóxeles.

## Build conjunto

```bash
colcon build --packages-select multidron_gui_lib multidron_gui
```

Después:

```bash
source install/setup.bash
ros2 launch multidron_gui multidron_gui.launch.py use_sim_time:=true
```

La compilación y las pruebas ROS/Qt reales deben ejecutarse en el workspace del proyecto antes de actualizar el estado documental de Fase 7.
