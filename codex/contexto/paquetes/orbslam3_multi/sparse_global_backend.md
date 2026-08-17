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
  -> rg -n "PlanSecondaryWork|ProcessDatabaseUpdate|ProcessLoopTask|CommitFiducial"
src/sparse_global_backend.cpp
  -> rg -n "SparseGlobalBackend::(PlanSecondaryWork|ProcessDatabaseUpdate|ProcessLoopTask|CommitFiducial)"
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
