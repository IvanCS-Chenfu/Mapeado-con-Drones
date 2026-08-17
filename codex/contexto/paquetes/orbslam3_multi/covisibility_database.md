# `CovisibilityDatabase`

## Rol

Base derivada, canonica y versionada de relaciones de covisibilidad. No guarda
candidatos BoW ni modifica `RawMapDatabase`.

## Contrato activo

- `PrepareOrbslam3Patch()` calcula fuera de lock altas, cambios y bajas para un
  submapa/revision a partir de las asociaciones raw vigentes.
- `ApplyPatch()` valida revision, aplica el lote de forma atomica e idempotente
  y conserva `relative_pose_measured` separada de `relative_pose_current`.
- `PrepareServerGeometricPatch()`, `ApplyTransactionalPatch()` y
  `RollbackPatch()` permiten a 3P comprometer o revertir aristas geometricas
  dentro del commit logico multi-base.
- Las claves de pares son canonicas; una misma pareja puede conservar a la vez
  las fuentes ORB nativa y server. `GetEdge()` prioriza la medida geometrica
  server sin borrar la procedencia nativa.
- 3M no usa estas aristas dentro del grafo fiducial. 3P compromete fuente
  `ServerLoopGeometric`, pero las optimizaciones todavia no consumen
  covisibilidad hasta la fase acordada posterior.

## Referencias

```text
include/orbslam3_multi/covisibility_database.hpp
  -> CovisibilityEdge / CovisibilityPatch / CovisibilityDatabase
  -> rg -n "class CovisibilityDatabase|PrepareServerGeometricPatch|ApplyTransactionalPatch|RollbackPatch"
src/covisibility_database.cpp
  -> prepare, commit y queries
  -> rg -n "CovisibilityDatabase::(PrepareOrbslam3Patch|PrepareServerGeometricPatch|ApplyTransactionalPatch|RollbackPatch|GetNeighbors)"
test/test_loop_pipeline.cpp
  -> patch canonico, revision e idempotencia
```

Telemetria de integracion: `[F3M-DATABASE-ENQUEUE]`,
`[F3M-DATABASE-UPDATE]` y los campos `covis=(+N,~N)` de `[F3P-FUSION]`.
