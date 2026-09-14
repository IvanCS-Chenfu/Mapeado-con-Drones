# task_manager - resumen vigente

## Responsabilidad

Nodo por dron que registra su perfil, confirma tareas y ejecuta localmente las
ordenes del servidor. Es mediador entre `task_server`, el wrapper visual y
`gen_tray`; no transforma `NavigationState` ni crea un sistema de coordenadas
alternativo.

Archivo: `dron/task_manager/src/task_manager_node.cpp` -> clase
`TaskManagerNode`.

## Trayectorias y STOP

La action relativa `execute_trajectory` valida que el `TrajectoryPlan`
pertenece al dron y a una tarea asignada/running. Reenvia exactamente sus
destinos, yaw/pitch y tiempos acumulados a `TrayAction`/`Pol3Waypoints`, sin
insertar un origen artificial ni reconstruir otra geometria.

STOP no posee lifecycle especial. Al recibir `stop_at_current_pose`, el dron
captura su pose canonica, sustituye localmente la action fisica por una
trayectoria al mismo punto y deja que `gen_tray` produzca el terminal normal.
Nunca se cancela el controlador sin una orden que lo deje estable.

Una geometria con inicio obsoleto responde `stale_start_pose`; el servidor debe
replanificar desde la pose actual, no reutilizar waypoints antiguos.

## Riesgo visual

Consume `orbslam/visual_tracking_evidence` y cuenta persistencia direccional
cuando la region configurada no contiene inliers y el movimiento apunta hacia
ella. El primer sector persistente activa STOP local y una correccion local de
yaw o pitch. El servidor coordina el lifecycle, pero no crea una ruta D* ni
reserva para ese giro.

Cuando termina la reorientacion, publica el terminal normal y conserva la
orientacion corregida. La imagen de debug pertenece a Fase 6 y usa el frame
exacto indicado por `VisualRiskEvent`.

## InspectFacade

Expone el servicio relativo `inspect_facade`. Su secuencia es:

```text
CaptureDepth de la orientacion actual de fachada
-> giro local hacia el objetivo solicitado
-> CaptureDepth del ultimo frame o del frame exacto de TRACKING_RISK
-> restaurar yaw/pitch de fachada
-> responder al servidor con las capturas y orientaciones
```

El servicio usa el cliente local `orbslam/capture_depth`. Si aparece riesgo
visual durante la mirada temporal, ejecuta el mismo STOP local, conserva el
frame que activo la persistencia y solicita ese frame exacto como segunda
captura. No calcula el mapa voxel ni decide si el corredor es navegable.

`MultiThreadedExecutor(4)` permite progresar callbacks de servicio, action,
captura depth y evidencia visual sin bloqueo mutuo.

El nodo tambien escucha `control/trajectory_active`, publicado por `gen_tray`
con QoS transient-local. `InspectFacade` responde `drone_busy` antes de mover
la camara si existe cualquier trayectoria fisica activa, incluida una enviada
por un cliente externo al servidor. Asi una inspeccion nunca sustituye el goal
que esta llevando al dron hasta el fiducial.

Los motivos terminales de `InspectFacade` se conservan por etapa
(`target_orientation_failed`, `target_capture_failed` o
`facade_restore_failed`); completar la restauracion no borra un fallo previo.

## Referencias

```text
src/task_manager_node.cpp -> HandleTaskStates / TaskReport
src/task_manager_node.cpp -> HandleExecuteGoal / ExecuteTrajectoryPlan
src/task_manager_node.cpp -> HandleInspectFacade / BeginFacadeInspection
src/task_manager_node.cpp -> HandleVisualEvidence / StartLocalVisualStop
src/task_manager_node.cpp -> PhysicalTrajectoryActive / HandleInspectFacade
```

Build vigente: correcto.
