# task_manager - resumen vigente

## Responsabilidad

Nodo por dron que registra su perfil, confirma tareas y ejecuta localmente las
ordenes del servidor. Es mediador entre `task_server`, el wrapper visual y
`gen_tray`; no transforma `NavigationState` ni crea un sistema de coordenadas
alternativo.

Archivo: `dron/task_manager/src/task_manager_node.cpp` -> clase
`TaskManagerNode`.

## Comandos autonomos Fase 6

`autonomous_command` implementa la recepcion breve de
`SubmitAutonomousCommand`. Valida el dron y los IDs, deduplica por
`command_id`, conserva una unica orden normal pendiente y responde
`accepted_queued` sin ejecutar dentro del callback. Un timer extrae la orden y
la ejecuta en un hilo local mediante el action server propio
`execute_trajectory`, por lo que comparte `TrayAction`, `gen_tray`, STOP y los
marcos de la trayectoria ordinaria.

Al terminar, el dron llama una sola vez a `/mission/report_autonomous_result`
con la pose final, depth utilizable y el mismo `command_id`. Acepta
`MOVE_AND_CAPTURE`, `LOOK_AND_CAPTURE` y `MOVE_AND_WATCH_FIDUCIAL`. El ultimo
solo observa durante el movimiento `/mission/fiducial_primary_observations` de
Fase 4 y devuelve `fiducial_seen`; no activa ni interpreta AprilTags. Si
`TRACKING_RISK` interrumpe la orden, STOP y la reorientacion siguen siendo
locales, el terminal queda `RESULT_ABORTED` y se intenta capturar depth antes
de notificarlo. Este estado es diagnostico: no descarta una observacion valida.

Esta cola no sustituye aun `execute_trajectory` ni `inspect_facade`; ambos
contratos legacy siguen vigentes durante la transicion.

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
-> CaptureDepth newest-first sobre candidatos cualificados de esta inspeccion
-> si hubo riesgo: esperar STOP y ejecutar una unica correccion local
-> responder al servidor con las capturas y orientaciones
```

El servidor lo invoca asincronamente, pero el handler local del servicio es
sincrono: conserva la respuesta hasta terminar las capturas y los giros de
inspeccion. El `MultiThreadedExecutor(4)` permite que las actions de
trayectoria y los callbacks visuales progresen mientras tanto, aunque solo
puede haber una inspeccion activa por dron. Una arquitectura futura basada en
colas puede convertir esta operacion larga en una orden aceptada de inmediato
y un evento de resultado correlacionado.

El servicio usa el cliente local `orbslam/capture_depth`. Si aparece riesgo
visual durante la mirada temporal, ejecuta el mismo STOP local y solicita el
depth de hasta 16 candidatos anteriores con al menos 20 inliers, separados 0.5
s o 2.5 grados, posteriores al inicio de la inspeccion, confianza minima 0.25 y
al menos 20 puntos depth. Despues ejecuta
una unica correccion de 25 grados alejandose del primer sector pobre. No encadena
restauracion, segundo STOP ni otra correccion. No calcula el mapa voxel ni
decide si el corredor es navegable.

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

`FacadeOrientation` trata `n` y `-n` como el mismo plano: una normal relativa
de 0 o 180 grados equivale a correccion yaw cero. Solo acepta la direccion con
soporte minimo 40, confianza minima 0.8 y correccion no superior a 25 grados;
si falla el gate conserva yaw/pitch actual. Todos los giros usan el arco minimo
y una duracion de al menos 5 s; el despliegue vigente limita el pico real de
yaw/pitch a 5 grados/s. Como el Pol3 reposo-a-reposo alcanza 1.5 veces su velocidad media,
`ExecuteInspectionOrientation` incorpora ese factor al calcular la duracion.

## Referencias

```text
src/task_manager_node.cpp -> HandleTaskStates / TaskReport
src/task_manager_node.cpp -> HandleAutonomousCommand / RunAutonomousCommandWorker
src/task_manager_node.cpp -> ExecuteAutonomousCommand / ReportAutonomousTerminal
src/task_manager_node.cpp -> HandleExecuteGoal / ExecuteTrajectoryPlan
src/task_manager_node.cpp -> HandleInspectFacade / FacadeOrientation
src/task_manager_node.cpp -> BeginInspectionVisualPhase / InspectionRiskMask
src/task_manager_node.cpp -> HandleVisualEvidence / StartLocalVisualStop
src/task_manager_node.cpp -> PhysicalTrajectoryActive / HandleInspectFacade
```

Build vigente: correcto.

`TRACKING_RISK` considera pobre un sector con
`<= visual_risk_directional_max_inliers` (valor inicial `3`) durante tres
frames. El debug visual del wrapper recibe el mismo umbral para sombrear la
region que realmente podria activar STOP.
