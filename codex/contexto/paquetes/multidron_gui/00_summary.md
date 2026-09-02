# 00_summary - multidron_gui

Proceso Qt/ROS 2 independiente de Fase 7. `src/main.cpp` mantiene el event loop
Qt en el thread principal y un `MultiThreadedExecutor` ROS en otro thread; el
poll de `rclcpp::ok()` y el shutdown final permiten cierre ROS->Qt y Qt->ROS.

`launch/multidron_gui.launch.py` expone `drone_count`, namespace, topics sparse
y KFs, fichero fiducial y `drone_stale_timeout_sec=1.0`. El ejecutable no es
requisito del backend y puede arrancar sin publishers.

`simulacion_dron/multi_dron.launch.py` lo inicia por defecto con
`launch_multidron_gui=true`; `launch_rviz=false` mantiene RViz fuera del flujo
normal. El entorno del proceso elimina variables/rutas Snap incompatibles con
Qt del sistema.

Referencias:

```text
src/main.cpp -> main -> GUI-BOOT/GUI-SHUTDOWN
launch/multidron_gui.launch.py -> generate_launch_description
```
