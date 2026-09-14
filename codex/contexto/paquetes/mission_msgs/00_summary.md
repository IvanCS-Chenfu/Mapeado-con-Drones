# mission_msgs - resumen vigente

Contrato ROS de mision replicado exactamente en `servidor/mission_msgs` y
`dron/mission_msgs`. Las dos copias deben permanecer identicas antes de
compilar cualquier productor o consumidor.

## Tareas, mapa y rutas

- `TaskState` es el estado autoritativo de una tarea. Incluye
  `state_revision`, progreso conocido, estado `TO_FINISH` e intervalos 1D de
  fachada mediante `FacadeCoverageInterval`.
- `TaskReport` confirma desde el dron los cambios locales de tarea.
- `VoxelMap` publica snapshot voxel y la proyeccion temporal de reservas.
- `GlobalSparseMapDelta` transporta snapshot/upserts/deletes con identidad
  estable para que `VoxelMapWorker` actualice solo cambios.
- `PlanRoute` solicita un objetivo world XYZ; `dispatch_execution` permite
  despachar el mismo plan por el runtime productivo.
- `TrajectoryPlan` conserva geometria, tiempos, identidad y lifecycle
  `PLANNED/ACTIVE/COMPLETED/CANCELED/REJECTED`.
- `ExecuteTrajectory` entrega el plan a `task_manager`. Un STOP usa
  `stop_at_current_pose`; el dron captura su pose y ejecuta una trayectoria
  normal de frenado, sin terminal paralelo `stop_completed`.

## Riesgo visual e inspeccion depth

`VisualRiskEvent` identifica el frame exacto y la region visual pobre que
activa el protocolo local. No transporta la imagen de debug.

`DenseKFObservation` es el producto compacto reutilizado por una captura depth:
incluye identidad de dron/epoch/frame/reference KF, calibracion, `K_T_C`, nube
de endpoints, normal estimada, calidad y motivo de fallo. En el flujo vigente
no se publica automaticamente por cada KF.

Servicios nuevos:

- `CaptureDepth`: selecciona el ultimo frame estereo o exige un `frame_id`
  exacto del buffer local y devuelve una observacion depth.
- `InspectFacade`: compone captura de fachada, giro temporal hacia un objetivo,
  segunda captura o frame exacto de tracking risk y restauracion de orientacion.

`FiducialPrimaryObservation` avisa al coordinador de una interpretacion primary
valida. Es un evento ligero de lifecycle; el mapa y el optimizador siguen
usando sus contratos de Fase 3.

## Referencias

```text
msg/TaskState.msg -> estado autoritativo e intervalos de coverage
msg/FacadeCoverageInterval.msg -> intervalo 1D normalizado de fachada
msg/DenseKFObservation.msg -> producto depth compacto bajo demanda
msg/FiducialPrimaryObservation.msg -> interrupcion fiducial de barrido
msg/TrajectoryPlan.msg -> plan previsto/activo y lifecycle
msg/VisualRiskEvent.msg -> riesgo visual asociado a frame exacto
srv/CaptureDepth.srv -> captura local por frame
srv/InspectFacade.srv -> inspeccion compuesta del corredor
srv/PlanRoute.srv -> planificacion XYZ
```
