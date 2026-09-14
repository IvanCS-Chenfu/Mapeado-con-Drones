# 00_summary - multidron_gui_lib

Biblioteca de modelo, bridge ROS, widgets y renderer OpenGL de Fase 7.

- `GuiDataModel` intercambia snapshots inmutables entre callbacks ROS y Qt,
  rechaza `DroneState` anteriores por epoch/secuencia/revision y marca stale
  conservando la ultima pose valida.
- `RosDataBridge` consume `/global_sparse_cloud`, `/global_keyframes`,
  `/dron_N/orbslam/navigation_state`, `/mission/geometry`,
  `/mission/task_states`, `/mission/voxel_map` y `/mission/planned_routes`;
  muestra la pose canonica
  publicada por el mux, tanto ORB anclada como `GT_FORCED` de simulacion.
- `MainWindow` contiene escena central, tarjetas de drones desplazables,
  inspector y formulario F6 fijo `drone/type/x/y/z/yaw`; el envio permanece
  deshabilitado hasta existir el contrato de Fase 6. El boton `Regiones` de la
  barra superior agrupa las subROIs por nivel y permite su visibilidad multiple.
  El dock `Drones` toma una sola vez al aparecer la primera tarea real el ancho
  natural de su tarjeta; las tarjetas no tienen ancho rígido y el usuario puede
  redimensionar el dock libremente después.
- `Scene3DWidget` usa OpenGL/VBO y capas para sparse, KFs, drones, fiduciales,
  trayectorias, voxeles y regiones de mision. No renderiza retícula voxel 2D
  ni 3D: muestra solo las celdas `OCCUPIED`/`FREE` habilitadas para preservar la
  interacción. Las regiones se renderizan con
  color estable, relleno, borde y etiqueta; su click solo actualiza inspector.
  `7E`, `7F` y `7H` estan validadas: score visual, labels D/F y picking con
  identidad estable.
- `visualization_policy` fija el gradiente rojo-amarillo-verde, filtro visual y
  desempate de picking por distancia en pantalla y profundidad. El inspector
  invalida selecciones desaparecidas sin conservar punteros a snapshots.
- Los KFs recibidos solo contienen marker ID/namespace, pose, color y
  geometria. No se muestra `drone_id/map_epoch` porque el contrato actual
  `/global_keyframes` no entrega esa metadata.
- Las tarjetas muestran tarea/estado reales y seleccionan su subROI mediante
  identidad estable. Los toggles `Ocupados`, `Libres` y `Reservados` son
  independientes; `UNKNOWN` no se dibuja. `RESERVED` procede de la proyeccion
  transitoria publicada por `task_server`, no modifica el estado raw y se
  superpone a `FREE`/`UNKNOWN`; `OCCUPIED` raw conserva prioridad visual. La
  fuente de pose muestra `GT` para
  `POSE_SOURCE_GT_FORCED`, no `INVALID`.
- Desde 6I, la capa principal muestra exclusivamente el `TrajectoryPlan`
  `ACTIVE` consumido por el dron. `PLANNED` es una intención pendiente y el
  bridge la ignora para no sustituir la polilínea física vigente. Un terminal
  solo retira la capa cuando su `trajectory_id` coincide con el almacenado para
  ese dron. La geometría de la capa es la de
  `Pol3Waypoints`, no una quintica paralela de `gen_tray`. El bridge registra
  `GUI-TRAJECTORY-UPDATE`, `GUI-TRAJECTORY-IGNORE` y elimina la capa con
  `GUI-TRAJECTORY-CLEAR` condicionado por identidad. No reconstruye la
  trayectoria desde el rastro de poses. STOP no dibuja una polilínea degenerada:
  al comenzar retira la ruta visible y la siguiente capa aparece únicamente al
  entrar su ruta nueva en `ACTIVE`.
- Las tarjetas traducen todos los lifecycle publicados, incluido `TO_FINISH`,
  y muestran una barra `Coverage` solo cuando `progress_known` llega por
  `/mission/task_states`; no calculan ni suavizan el porcentaje. Al seleccionar
  una tarea de barrido, la escena dibuja simultaneamente el subROI y la union de
  `coverage_intervals` como una linea sobre la fachada a altura media, usando el
  mismo color estable de la region. La linea desaparece al quitar la seleccion
  y no sustituye a la trayectoria activa.
- El lifecycle textual de reserva en tarjetas sigue pendiente, pero la escena
  ya consume `VoxelMap.reserved_voxels`: al commit/reemplazo aparece la huella
  `MOVING` o `HOLD`; al release la siguiente publicacion restituye el estado
  raw anterior. No se dibujan cajas de seguridad adicionales.

Tests: modelo/reordenacion/stale, loader fiducial, invalidacion `RenderLayer`,
politica de score/picking con 100k candidatos, layout con 20 drones y smoke
visual sintetico solo bajo `BUILD_TESTING`.

Referencias:

```text
include/multidron_gui_lib/gui_data_model.hpp -> GuiDataModel
src/ros_data_bridge.cpp -> RosDataBridge / OnNavigationState / CheckStaleDrones
src/main_window.cpp -> BuildDroneDock / UpdateDroneCards
src/main_window.cpp -> UpdateMissionRegionMenu / ApplyMissionRegionVisibility
src/ros_data_bridge.cpp -> OnPlannedRoute / GUI-PLANNED-ROUTE-UPDATE
src/gui_data_model.cpp -> ReplaceTrajectory
src/gui_data_model.cpp -> coverage_intervals / selected task dirty state
include/multidron_gui_lib/render_layer.hpp -> RenderLayer
include/multidron_gui_lib/visualization_policy.hpp -> ScoreColor / SparsePointVisible / SelectBestCandidate
src/scene3d_widget.cpp -> SetVisibleMissionRegions / SynchronizeGpuData / paintGL
```

Validacion vigente: build correcto y CTest `9/9`.
