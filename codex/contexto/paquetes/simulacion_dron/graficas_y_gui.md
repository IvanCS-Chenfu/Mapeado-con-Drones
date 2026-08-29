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

En diagnóstico 5H, `on_gt_velocity` escribe también
`drone_N_gt_angular.csv` con omega world/body y dos timestamps:
`gt_stamp` del tiempo físico Gazebo y `gt_receive_stamp` del reloj ROS. No deben
restarse directamente stamps de dominios distintos; la herramienta
`codex/herramientas/analyze_f5h_angular_phase.py` usa el par para sincronizar
medida visual, publicación y tick de control y genera CSV/JSON/PNG derivados.
La prueba 264 valida esta reconstruccion: produce timeline para 323 ticks ORB
y metricas finitas de correlacion, potencia y torque ideal. El dron 2 no
participa en el YAML de hover y queda correctamente como datos insuficientes.
Desde 265 el analizador integra por separado trabajo y energia de `tau_er`,
`tau_ew` y torque total, y mide edad local, horizonte/clamp y distancias
`visual_q -> base_q -> predicted_q`. Esta captura localiza el desfase dominante
en la pose base integrada, no en la extrapolacion del timer.
Para 266 registra tambien error visual-base antes/despues del update, conteos
de `SMALL_ANCHOR`/pending/confirmed/rejected y una ventana comun configurable
para comparar energia sin sesgo por distinta duracion ORB.
Los conteos se deduplican por `measurement_receive_stamp`, evitando ponderar
una observacion varias veces por los ticks de control de 50 Hz.

```text
src/graficar/pose_metrics_node.py -> PoseMetricsNode / on_gt_velocity / build_summary
test/test_pose_metrics.py -> rg "alignment|angular_velocity|temporal|shutdown"
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

En 269-272, `pose_metrics_node.py` registra el GT angular de laboratorio y
`analyze_f5h_angular_phase.py` consume los marcadores compatibles emitidos por
`gt_timing_diagnostic`. Los resultados separan A-D en directorios
`metricas/prueba_269` a `metricas/prueba_272`.
