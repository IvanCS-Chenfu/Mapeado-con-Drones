# Historial 7A

## 2026-09-02 - Bloque 1

Se valido la separacion `multidron_gui_lib`/`multidron_gui`, Qt 5.15.3,
OpenGL Mesa 4.6, event loop Qt principal y executor ROS separado. El launch
global activa la GUI por defecto y permite apagarla sin afectar backend.

Pruebas: standalone sin publishers abre y cierra limpio; 376R valida Gazebo y
backend sin GUI; 377 valida GUI+Gazebo y ausencia de RViz. La reapertura
standalone posterior vuelve a cerrar con `[GUI-SHUTDOWN]`.

Intento 376: `NO CONSEGUIDA` por usar el nombre YAML inexistente
`mode: parallel`; el runner rechazo antes de enviar goals. Se conservo el
intento y 376R corrigio exclusivamente a `mode: simultaneous`.

Conclusion: `CONSEGUIDA`.
