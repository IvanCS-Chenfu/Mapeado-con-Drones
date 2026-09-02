# 00_summary - multidron_gui_lib

Biblioteca de modelo, bridge ROS, widgets y renderer OpenGL de Fase 7.

- `GuiDataModel` intercambia snapshots inmutables entre callbacks ROS y Qt,
  rechaza `DroneState` anteriores por epoch/secuencia/revision y marca stale
  conservando la ultima pose valida.
- `RosDataBridge` consume `/global_sparse_cloud`, `/global_keyframes` y
  `/dron_N/orbslam/navigation_state`; carga fiduciales YAML canonicos y nunca
  usa GT como pose funcional.
- `MainWindow` contiene escena central, tarjetas de drones desplazables,
  inspector y formulario F6 fijo `drone/type/x/y/z/yaw`; el envio permanece
  deshabilitado hasta existir el contrato de Fase 6.
- `Scene3DWidget` usa OpenGL/VBO y seis instancias `RenderLayer` para sparse,
  KFs, drones, fiduciales, trayectorias y voxeles. `7E`, `7F` y `7H` estan
  validadas: score visual, labels D/F y picking con identidad estable.
- `visualization_policy` fija el gradiente rojo-amarillo-verde, filtro visual y
  desempate de picking por distancia en pantalla y profundidad. El inspector
  invalida selecciones desaparecidas sin conservar punteros a snapshots.
- Los KFs recibidos solo contienen marker ID/namespace, pose, color y
  geometria. No se muestra `drone_id/map_epoch` porque el contrato actual
  `/global_keyframes` no entrega esa metadata.

Tests: modelo/reordenacion/stale, loader fiducial, invalidacion `RenderLayer`,
politica de score/picking con 100k candidatos, layout con 20 drones y smoke
visual sintetico solo bajo `BUILD_TESTING`.

Referencias:

```text
include/multidron_gui_lib/gui_data_model.hpp -> GuiDataModel
src/ros_data_bridge.cpp -> RosDataBridge / OnNavigationState / CheckStaleDrones
src/main_window.cpp -> BuildDroneDock / UpdateDroneCards
include/multidron_gui_lib/render_layer.hpp -> RenderLayer
include/multidron_gui_lib/visualization_policy.hpp -> ScoreColor / SparsePointVisible / SelectBestCandidate
src/scene3d_widget.cpp -> SynchronizeGpuData / paintGL
```
