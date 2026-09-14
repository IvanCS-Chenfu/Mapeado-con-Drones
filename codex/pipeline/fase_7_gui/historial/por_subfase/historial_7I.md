# Historial - 7I

## 2026-09-07 - Tramo habilitado

- `RosDataBridge` consume `/mission/task_states`; las tarjetas muestran estado,
  tarea y progreso solo si es conocido.
- Clicar la tarea resalta su subROI sin alterar el menu manual `Regiones`.
- La prueba 604 mostro 12 tareas en GUI y fue revisada visualmente como correcta
  por el usuario. La 605 confirmo la actualizacion de dos asignaciones reales.

Conclusion: PARCIAL; no se inventa progreso hasta disponer de coverage/planes.

## 2026-09-12 - Capa RESERVED

`RosDataBridge` consume `VoxelMap.reserved_voxels` como overlay transitorio:
superpone RESERVED sobre FREE/UNKNOWN, deja OCCUPIED raw con prioridad y vuelve
al snapshot raw al release. La barra incorpora el toggle `Reservados`. La
prueba 687 de Gazebo+GUI F7 registro 310 reservas al commit de D1 y cero tras
su release; tambien observo HOLD local y reservas simultaneas.

Conclusion: PARCIAL; la visualizacion de reservas esta conseguida, mientras el
lifecycle textual de reserva en tarjetas sigue pendiente.
