# GUI y gráficas en `simulacion_dron`

## `src/control_tray/gui_tray_multi.py`

GUI Tkinter para enviar goals `TrayAction` a varios drones.

Rol:
- seleccionar dron;
- configurar tipo de trayectoria;
- enviar objetivo a `/dron_X/AccionTrayectoria`.

Limitación:
- es interactiva; para pruebas automáticas conviene crear un script no interactivo que use la misma action.

## `src/graficar/graficar.py`

Nodo Python de visualización de arrays numéricos publicados en:

- `/numeric_array`
- `/labels_array`

## Ejecutables C++ de gráficas

- `graficar_GT.cpp`: publica valores GT para graficar.
- `graficar_tray.cpp`: publica referencias de trayectoria desde feedback de action.
- `graficar_GTvsTray.cpp`: publica errores entre GT y trayectoria.

## Estado

Útiles para debug de control, no para validar directamente el mapa global.

Para las fases de Codex, la validación principal debe venir de logs del servidor, nubes en RViz/GUI futura y métricas automáticas.

## `pose_metrics_node.py` (5F)

Recolector namespaced de `NavigationState` y `sensor/GT/pose`. Empareja por
timestamp con skew maximo configurable, fija una sola alineacion GT->O por
epoch y escribe CSV, `summary.json` y un PNG O/W/GT por dron. Reporta RMSE,
MAE, p95 y maximo de posicion/orientacion/yaw, frecuencia, inter-arrival,
jitter, latencia relativa al primer sample del epoch, cambios de reference KF
y revisiones. GT es exclusivamente metrica externa.

```text
src/graficar/pose_metrics_node.py -> PoseMetricsNode / build_summary
test/test_pose_metrics.py -> rg "alignment|temporal|shutdown"
launch/multi_dron.launch.py -> phase5_pose_metrics_enabled/output_dir
```

La prueba 230 confirma que la instrumentacion funciona. Sus agregados incluyen
todas las muestras autoritativas y varias revisiones, por lo que no representan
por si solos el estado convergido final. El analisis siguiente debe segmentar
por `pose_revision` y ventanas posteriores a optimizacion. El error angular
global cercano a 2 rad tambien exige validar la convencion body/camera/GT antes
de atribuirlo al optimizador.

## `global_drone_pose_visualizer.py` (5H)

Consume `/dron_X/orbslam/navigation_state` y publica `/global_drone_poses`.
Construye ejes XYZ desde `o_t_body`, no desde `w_t_body`, y etiqueta cada pose
como `[ORB]` o `[GT]`. Esa es exactamente la pose que recibe
`control_calcular_fuerzas`. La visibilidad no depende de autoridad global y una
muestra sin velocidad no borra la ultima pose consumida, evitando parpadeo.

```text
src/visualizer/global_drone_pose_visualizer.py
  -> pose_is_visible / build_pose_markers / GlobalDronePoseVisualizer
  -> rg "F5H-RVIZ-CONTROL-POSE|o_t_body|pose_source_name"
test/test_global_drone_pose_visualizer.py
  -> rg "fallback_pose|source_label|provisional"
```
