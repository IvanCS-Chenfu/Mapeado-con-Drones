# Historial - 6E

## 2026-09-07 - Implementacion y validacion

- `task_server` crea 12 tareas `PENDING`, publica `/mission/task_states` y
  acepta reports de los managers.
- La elegibilidad se gobierna por `phase5_navigation_source`: GT solo en la
  rama simulada `gt`; ORB world anclado en `orb`.
- CTest final de `task_server`: 7/7 correctos.
- Prueba 605: asignacion determinista de AB a dron 1 y BC a dron 2, ambas
  aceptadas localmente; ninguna tarea se completo automaticamente.

Conclusion: CONSEGUIDA para allocator base; coste de navegacion y lifecycle de
ejecucion quedan pendientes.

## 2026-09-16 - Bloque 3 - Evidencia KF desacoplada

- objetivo intentado: separar el calculo de evidencia sparse por KF de la
  materializacion del mapa voxel mundial.
- archivos modificados: `evidence_pipeline.hpp/.cpp`, `task_server_node.cpp`,
  `CMakeLists.txt` y prueba `test_evidence_pipeline.cpp`.
- resultado de build: `task_server` correcto, exit 0.
- pruebas: se anadieron casos de alta atomica, reproyeccion y tombstone. El
  primer CTest dio 8/9 por formato; tras correccion mecanica, la repeticion dio
  9/9, incluidos los tres gtest y seis linters.
- conclusion: PARCIAL. La frontera nueva compila y la ruta RANSAC directa fue
  retirada del nodo; falta conectar depth en 6F.
