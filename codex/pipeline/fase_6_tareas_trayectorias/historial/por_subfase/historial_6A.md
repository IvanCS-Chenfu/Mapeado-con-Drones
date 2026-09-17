# Historial 6A

## 2026-09-07 - Arquitectura y configuracion de mision

- objetivo intentado: separar paquetes, validar YAML y derivar el volumen duro.
- archivos modificados: `mission_msgs`, `task_lib`, `task_server`, launch y
  telemetria web de F6.
- paquetes compilados: `mission_msgs`, `task_lib`, `task_server`.
- resultado de build: correcto; CTest `task_lib` 5/5 y `task_server` 7/7.
- prueba Gazebo: 603 con dos drones estacionarios, GUI F7 y grafos web; sin RViz2.
- evidencia positiva: `F6A-MISSION-CONFIG`, volumen duro derivado y grafos live.
- evidencia negativa o ausente: no se implementaron voxel, tareas ni movimiento.
- conclusion: `CONSEGUIDA`.
- siguiente paso recomendado: 6D, voxel reversible.

## 2026-09-16 - Inicio de la migracion por workflows desacoplados

- objetivo intentado: implantar el armazon de 6A sin alterar el despacho,
  profundidad, D* ni vuelo vigentes.
- archivos modificados: `workflow_scheduler.hpp/.cpp`,
  `test_workflow_scheduler.cpp`, `CMakeLists.txt` y `task_server_node.cpp`.
- paquetes compilados: `task_server`.
- resultado de build: correcto; solo avisos heredados por la API ROS de QoS
  deprecada en servicios existentes.
- pruebas Gazebo/replay: no aplican todavia; el scheduler no recibe ni emite
  ordenes ROS en este bloque.
- pruebas unitarias: CTest `8/8`, incluidos FIFO/reencolado, deduplicacion y
  STOP generacional.
- evidencia positiva: la nueva biblioteca mantiene cinco colas tipadas,
  rechaza duplicados y no permite dos ordenes normales por dron.
- evidencia negativa o ausente: `RunFacadeWorker` sigue activo y aun no hay
  comandos/resultados desacoplados ni workers reales conectados.
- conclusion: `PARCIAL` para el contrato nuevo de 6A.
- siguiente paso recomendado: migrar 6B/6C sobre esta infraestructura.
