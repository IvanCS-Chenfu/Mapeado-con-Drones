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
- `KeyframeSparseEvidenceDelta` transporta incrementalmente los MapPoints
  observados por cada KF, sus revisiones y deletes. Es la frontera F3->F6 para
  reconstruir fuentes FREE locales y reversibles; no usa la asociacion unica de
  publicacion global de un MapPoint.
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

`DenseKFObservation` separa `points_k` cercanos, filtrados estrictamente y
aptos para normal, de `far_free_points_k` stereo-validos de 5..10 m. Estos
ultimos solo producen rayos FREE truncados por el servidor y nunca OCCUPIED ni
normales. Incluye identidad de dron/epoch/frame/reference KF, calibracion,
`K_T_C`, calidad y motivo de fallo; no se publica automaticamente por cada KF.

Servicios nuevos:

- `CaptureDepth`: selecciona el ultimo frame estereo, exige un `frame_id`
  exacto o solicita fallback newest-first sobre candidatos cualificados. La
  peticion transporta `minimum_candidate_frame_id`, `minimum_confidence` y
  `minimum_support_points` para que productor y consumidor apliquen la misma
  puerta.
- `InspectFacade`: compone captura de fachada, giro temporal hacia un objetivo y
  segunda captura. Ante tracking risk usa candidatos anteriores cualificados,
  ejecuta STOP y una unica correccion local.
- `SubmitAutonomousCommand`: servidor a dron. Transporta identidad
  correlacionada, tipo `MOVE_AND_CAPTURE`/`LOOK_AND_CAPTURE`/`LOOK_FIDUCIAL`,
  objetivo visual, plan opcional y politica de captura. La respuesta confirma
  aceptacion inmediata o duplicado; nunca conserva abierta una maniobra.
- `ReportAutonomousResult`: dron a servidor. Transporta la misma identidad,
  estado terminal, pose final y observaciones depth. La respuesta solo confirma
  que el servidor lo ha encolado o que ya era duplicado.

`FiducialPrimaryObservation` avisa al coordinador de una interpretacion primary
valida. Es un evento ligero de lifecycle; el mapa y el optimizador siguen
usando sus contratos de Fase 3.

## Referencias

```text
msg/TaskState.msg -> estado autoritativo e intervalos de coverage
msg/FacadeCoverageInterval.msg -> intervalo 1D normalizado de fachada
msg/DenseKFObservation.msg -> producto depth compacto bajo demanda
msg/KeyframeSparseEvidenceDelta.msg -> evidencia sparse incremental por KF
msg/FiducialPrimaryObservation.msg -> interrupcion fiducial de barrido
msg/TrajectoryPlan.msg -> plan previsto/activo y lifecycle
msg/VisualRiskEvent.msg -> riesgo visual asociado a frame exacto
srv/CaptureDepth.srv -> captura local por frame
srv/InspectFacade.srv -> inspeccion compuesta del corredor
srv/SubmitAutonomousCommand.srv -> aceptacion inmediata servidor a dron
srv/ReportAutonomousResult.srv -> resultado correlacionado dron a servidor
srv/PlanRoute.srv -> planificacion XYZ
```
