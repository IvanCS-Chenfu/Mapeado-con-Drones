# `GlobalPoseStore`

## Rol

Autoridad unica y versionada de anchors y poses world. Mantiene separado
`raw_world_pose`, `correction_pose` y `world_pose`, con linaje de commits.

## Referencias

```text
include/orbslam3_multi/global_pose_types.hpp
  -> GlobalPoseRecord / AcceptedPoseUpdate / PoseChangeSet
  -> rg -n "GlobalPoseRecord|AcceptedPoseUpdate|PoseChangeSet"
include/orbslam3_multi/global_pose_store.hpp
  -> commits fiducial, loop anchor y optimizacion multi-submapa
  -> rg -n "CommitAnchor|ApplyRawPoseChanges|CommitAcceptedPoses|CommitLoopAnchorBatch|CommitLoopOptimizedPoses"
src/global_pose_store.cpp
  -> commits atomicos y validacion de revisiones/hard
  -> rg -n "GlobalPoseStore::(CommitAnchor|ApplyRawPoseChanges|CommitAcceptedPoses|CommitLoopAnchorBatch)"
```

## Contrato activo

- `CommitAnchor()` hace visible un submapa completo de forma atomica y marca un
  unico KF `FiducialAccepted`/hard.
- `ApplyRawPoseChanges()` crea o recompone poses bajo el anchor inicial hasta
  el ultimo control y bajo la transformacion de continuidad para IDs
  posteriores al control. `raw_world_pose` conserva siempre la expresion raw
  del anchor inicial; `world_pose` y `correction_pose` reflejan el control
  aceptado. Una pose ya
  aceptada/optimizada conserva `world_pose` ante un rebase raw; una
  invalidacion conserva linaje y pose, pero deja el registro inactivo.
- `GetSubmapPoses()` entrega la copia versionada usada por el grafo.
- `CommitAcceptedPoses()` valida submapa, revision esperada y que ningun hard se
  mueva; si todo es compatible aplica todo el batch bajo un lock y una sola
  revision. Un commit full puede indicar el nuevo control y actualizar en la
  misma transaccion su `ContinuationRecord`. Si recibe
  `replacement_world_T_local`, exige una dependencia loop viva, reemplaza el
  anchor completo y corta esa dependencia al crear el primer hard del hijo.
- El target completo puede quedar hard sin mover anchors previos. Los cambios
  exponen solo IDs realmente movidos y nuevos hard.
- Un conflicto de revision o hard no deja cambios parciales.
- `CommitLoopAnchorBatch()` revalida el lote contra el backend raw, incluye los
  KFs tardios y crea poses `LoopAnchorDerived` en una sola revision.
- `CommitLoopOptimizedPoses()` valida y aplica en una sola revision las poses
  corregidas de todos los submapas de un grafo 3Q. Ningun hard puede moverse;
  un fallo de revision/invariante deja el store intacto.
- `AcceptedPoseBatchResult` devuelve `detail` con el precondicionante exacto y
  los conteos `rebased_skipped_controls`/`rebased_inactive_controls`. Los
  apoyos virtuales se usan para interpolar fuera del store: no se reactivan ni
  se escriben en el batch.
- El commit 3Q actualiza continuidades por submapa y propaga rigidamente tails,
  KFs llegados durante el solve y dependencias soft afectadas. El changeset
  diferencia IDs optimizados de `control_propagated_ids`.
- Un anchor loop conserva una dependencia padre-hijo blanda. Los cambios
  aceptados del KF de apoyo del padre propagan rigidamente todas las poses, el
  anchor y la continuidad del hijo bajo el mismo lock; los IDs movidos aparecen
  tambien en `control_propagated_ids`. Cuando el hijo recibe su primer
  fiducial hard, todo el submapa se recompone desde el nuevo anchor absoluto,
  la dependencia se elimina y deja de seguir al padre.

Las poses inactivas no se reactivan para completar una ventana. Un hard
inactivo puede seguir siendo frontera fija del grafo, mientras el builder sigue
excluyendolo de la vista publica por su estado raw/world.

`ContinuationRecord` guarda control, `world_T_local`, revision y linaje del
commit por submapa. Un commit parcial no lo desplaza. El cambio es atomico con
las poses aceptadas y se valida contra revision antes de aplicar nuevos raw.

Los tests de `test_sparse_global_backend.cpp` y `test_loop_pipeline.cpp` cubren
anchor, rebase, invalidacion, preservacion, commit atomico, tail, primer KF
llegado despues del commit, aislamiento entre submapas, anchor loop,
propagacion blanda, reanchor fiducial hard y corte de dependencia.
