# `SparseGlobalBackend`

## Rol

Coordinador de dominio de los flujos principal, fiducial y loop. El servidor conserva
la orquestacion, pero no implementa decisiones geometricas ni commits de mapa.

```text
principal: InsertDelta/InsertFullSnapshot -> raw/pose/score -> builder dirty
fiducial: ProcessFiducialObservation -> anchor | task
secundario: Revalidate -> Build graph -> Optimize -> Validate -> Commit
database: PlanSecondaryWork -> ProcessDatabaseUpdate -> CreateLoopTasks
loop: ProcessLoopTask -> LoopPipeline -> CommitLoopAnchorBatch si procede
fusion: LoopPipeline -> prepare privado -> commit tracks/covis/score -> dirty
opt loop: OptimizationEvidence -> graph/solve/validate -> commit -> fusion
publicacion: BuildGlobalMap, solo desde PrimaryWorker
```

## Operaciones 3H-3L

- `ConfigureFiducialOptimization()` aplica los mismos umbrales/ratios a manager,
  builder, solver y validador.
- `RevalidateFiducialTask()` relee target y ultimo control; devuelve `STALE` si
  otra correccion ya dejo el KF dentro de umbral.
- `BuildFiducialPoseGraph()` captura raw y poses bajo una seccion breve.
- `OptimizeFiducialPoseGraph()` y `ValidateFiducialProposal()` trabajan sobre
  datos privados sin locks live.
- `CommitFiducialProposal()` reconsulta el estado, comprueba revisiones de los
  KFs consumidos, incorpora ventana tardia compatible y el tail que ya existe,
  y llama a un unico `CommitAcceptedPoses()`.
- Tras un commit completo promueve el target a control. Un parcial conserva el
  control anterior.
- El commit completo actualiza conjuntamente la continuidad de
  `GlobalPoseStore`; los KFs que entren en deltas posteriores al target se
  derivan desde ese control sin alterar la base raw.
- Solo `PoseChangeSet.updated_ids` llega a
  `GlobalMapBuilder::MarkPoseChanges()`; no se ejecuta ni publica desde el
  worker secundario.

Los intermedios inactivos se omiten. Si el flujo principal cambia un dato
consumido durante el solve, el commit devuelve conflicto y el worker puede
revalidar/reconstruir de forma acotada.

Una tarea fiducial pendiente cuyo target ya no es posterior al control vigente
termina `Stale` en `RevalidateFiducialTask()`. No llega al builder ni se
convierte en fallo duro cuando otro commit adelanta el control.

## Operaciones 3M-3O

- `PlanSecondaryWork()` deriva una `DatabaseUpdateTask` por `ChangeSet` cuando
  cambia covisibilidad; si no, produce `LoopTask` directas.
- `ProcessDatabaseUpdate()` prepara y aplica el patch de covisibilidad.
- `CreateLoopTasks()` captura revisiones de apariencia, geometria y anchor para
  scheduling causal, y una revision geometrica exacta separada para stale y
  commit.
- `ProcessLoopTask()` delega BoW, regiones, geometria e hipotesis en
  `LoopPipeline`; solo un anchor confirmado llega a `CommitLoopAnchorBatch()`.
- Tras el anchor marca los KFs dirty y devuelve todos los KFs del submapa para
  reencolarlos bajo la nueva revision. No registra el KF loop como control
  fiducial. Si llega el primer fiducial directo de ese hijo, recompone todas
  sus poses con `replacement_world_T_local`, lo hace hard y corta la
  dependencia. No construye ni publica el mapa desde el secundario.

## Referencias

```text
include/orbslam3_multi/sparse_global_backend.hpp
  -> SparseGlobalBackend / PrimaryBackendResult
  -> rg -n "PlanSecondaryWork|ProcessDatabaseUpdate|ProcessLoopTask|ProcessLoopOptimization|CommitFiducial"
src/sparse_global_backend.cpp
  -> rg -n "SparseGlobalBackend::(PlanSecondaryWork|ProcessDatabaseUpdate|ProcessLoopTask|ProcessLoopOptimization|CommitLoopProposal|CommitFiducial)"
```

El backend no crea threads, no conoce GT y no publica ROS. La seccion de estado
serializa la decision de visita y el commit breve, pero grafo/solver/validacion
siguen fuera de locks live.

## Operaciones 3P

- `ProcessLoopTask()` conserva la decision en `LoopPipeline`. Si es fusion,
  `FusedLandmarkManager::PrepareFusion()` calcula tracks, representante,
  evidencia y visibilidad fuera del lock coordinador.
- Antes del commit relee revisiones raw, pose, fusion, score y covisibilidad.
  Un cambio devuelve stale sin aplicar o activa rollback explicito si otra base
  cambia durante el lote.
- El commit aplica tracks, aristas server y score como una unidad logica breve;
  despues entrega IDs exactos a `GlobalMapBuilder`, sin ejecutarlo ni publicar.
- Un track retirado durante otra union del mismo patch se elimina tambien de
  `touched_tracks`; las referencias locales se validan para impedir accesos a
  IDs ya absorbidos.
- `BuildGlobalMap()` toma una revision coherente de pose/score/fusion y el
  siguiente principal publica la vista materializada.
- El backend devuelve `stale`/`rolled_back` y la telemetria completa de
  visibilidad; el servidor es quien crea despues una `LoopTask` fresca. No
  existe retry interno ni recursion dentro de `ProcessLoopTask()`.

## Operaciones 3Q

- `ProcessLoopOptimization()` reutiliza `PoseGraphBuilder`,
  `OptimizationManager` y `OptimizationValidator` sobre un problema
  `LoopRelative`; conserva la computacion RANSAC de la misma tarea.
- El grafo puede abarcar varios submapas mediante hard fiduciales, tramos
  temporales, dependencias blandas, constraints previas y covisibilidad nativa.
- `CommitLoopProposal()` aplica correcciones sobre el estado vigente y llama a
  un commit multi-submapa atomico. Revalida controles activos, drift raw local,
  hard e invariantes; un conflicto incompatible devuelve stale sin escritura.
  Snapshot y poses actuales se releen bajo `state_commit_mutex_`; el batch loop
  no repite una expectativa de `pose_revision` ya cubierta por esa
  serializacion, pero conserva revision raw y tolerancia de drift obligatorias.
  Durante ese rebase puede omitir controles intermedios que hayan quedado
  ausentes o cuyo raw haya cambiado demasiado durante el solve. Un control
  culled que aun existe y conserva su raw puede seguir como apoyo virtual de la
  correccion, incluidos extremos loop y fixed, pero nunca se reactiva ni recibe
  escritura. Se exigen al menos dos controles activos actuales por submapa
  antes de interpolar y comprometer.
- `AcceptedPoseBatchResult::detail` distingue el precondicionante exacto de un
  commit fallido (snapshot/control, drift raw, continuidad, hard o store); la
  decision sigue siendo stale/error segun `PoseCommitStatus`, pero el log deja
  de colapsar causas diferentes bajo `revision_conflict`.
- Los KFs posteriores a la ventana y los hijos soft se propagan con la
  correccion de su ultimo control. Solo IDs movidos se notifican dirty.
- Tras commit aceptado llama directamente a `CommitLoopFusion()`; una fusion
  stale no revierte las poses correctas y origina el retry fresco del servidor.
- `LoopOptimizationSummary` separa `graph_ms`, `solve_ms`, `validation_ms` y
  `commit_ms` para localizar coste sin instrumentacion externa.
- Antes de construir el grafo, `EvaluateProtectedLoopRegions()` compara la
  relacion world vigente con la medida RANSAC. Una region es protegida si el KF
  o vecinos temporales/covisibles acotados pertenecen a un hard fiducial,
  corredor hard-hard o solucion fiducial aceptada/optimizada. Si ambos lados
  son fiables y la incompatibilidad supera 5 m o 20 grados, la region se
  descarta como `protected_region_far_repeated_loop`; con un solo lado fiable
  se conserva la optimizacion asimetrica.
- `LoopRejectionLedger` recuerda por regiones temporales, transformacion
  cuantizada y revisiones de anchor los rechazos protegidos o estructurales.
  Los KFs vecinos equivalentes terminan como
  `regional_rejection_ledger_hit` sin builder/solver. El ledger se invalida al
  cambiar anchors, fiduciales, loops o fusiones que afectan a esos submapas; no
  crea una blacklist permanente.
- `CreateFusionRefreshTasks()` agrupa los KFs movidos por buckets temporales y
  crea un representante `FusionRefresh` por region. El filtrado espacial fino
  se realiza despues en `LoopPipeline`, de modo que un commit grande no genera
  una busqueda global por cada KF movido.

## Operaciones 3S

- Tras cada raw commit, `ApplyRawChanges()` actualiza base ORB solo para IDs
  afectados y `RefreshGeometryScores()` construye entradas world desde KFs
  observadores con pose activa.
- Deltas raw, anchors, commits loop y commits fiduciales entregan KFs/MPs
  exactos. Poses movidas expanden solo sus asociaciones mediante
  `GetBuilderSnapshot()`; no se recorre toda la base.
- `RefreshScoresAfterPoseChanges()` recalcula factores, propaga raw modificados
  a sus fused tracks y marca pose/score dirty bajo un lock breve del builder.
- El baseline se consulta una vez por submapa/lote. Inputs geometricos
  identicos no reindexan vecindad, evitando el backlog observado en 192.
- `ConfigureLandmarkScores()` y `ConfigureFusedLandmarks()` centralizan
  parametros; el backend no crea threads, publica ni conoce GT.

Referencias:

```text
src/sparse_global_backend.cpp
  -> RefreshGeometryScores / RefreshScoresAfterPoseChanges / RefreshFusedScores
  -> rg -n "RefreshGeometryScores|RefreshScoresAfterPoseChanges|RefreshFusedScores"
```
