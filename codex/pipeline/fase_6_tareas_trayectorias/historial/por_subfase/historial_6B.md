# Historial 6B

## 2026-09-07 - Geometria y visualizacion de subROIs

- objetivo intentado: cuatro mitades solapadas por nivel y snapshot de geometria.
- archivos modificados: `task_lib`, `task_server`, `multidron_gui_lib` y grafos F6.
- paquetes compilados: `task_lib`, `task_server`, `multidron_gui_lib`.
- resultado de build: correcto; CTest GUI 9/9 y smoke Qt/OpenGL correcto.
- prueba Gazebo: 603 con geometria runtime y puerta de revision manual.
- evidencia positiva: 3 niveles, 12 regiones; menu individual/multiple,
  mostrar/ocultar y prismas con relleno, borde y etiqueta confirmados por usuario.
- evidencia negativa o ausente: no existen asignaciones runtime antes de 6E.
- conclusion: `CONSEGUIDA`.
- siguiente paso recomendado: enlazar regiones a tareas reales en 6E/7I.

## 2026-09-16 - Ingreso de disponibilidad por workflow

- objetivo intentado: iniciar la asignacion nueva sin retirar aun el
  dispatcher legacy.
- archivos modificados: `task_server_node.cpp` y scheduler de 6A.
- resultado de build: `mission_msgs` servidor/dron, `task_server` y
  `task_manager` correctos.
- evidencia positiva: disponibilidad entra por `TASK_ASSIGNMENT`; la asignacion
  por proximidad crea un workflow `POINT_SELECTION` con IDs y revision de mapa.
- evidencia pendiente: no se pudo ejecutar CTest ni smoke ROS por limite
  externo de la plataforma; `POINT_SELECTION` no tiene aun consumer nuevo.
- conclusion: `PARCIAL`.
