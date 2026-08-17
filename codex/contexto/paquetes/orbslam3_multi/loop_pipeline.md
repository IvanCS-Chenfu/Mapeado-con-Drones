# `LoopPipeline`

## Rol

Implementa la busqueda y decision 3N-3P dentro de una sola `LoopTask`: indice
BoW derivado, regiones, rama rapida world, subnubes, matching ORB, RANSAC y
seleccion de fusion/evidencia 3Q.

## Flujo

```text
revalidate -> UpsertBow/SearchBow -> GroupRegions
           -> BuildCloud(max 320) -> VerifyRegion/RANSAC
           -> fusion | evidence 3Q | deferred | anchor batch | reject
```

- El indice BoW no se guarda en covisibilidad ni altera el BoW raw.
- Se verifican hasta tres regiones; cada region usa una seed y vecinos de
  covisibilidad acotados.
- Matching usa Hamming, cross-check y ratio; RANSAC 3D-3D es determinista y usa
  por defecto 80 iteraciones.
- La cache negativa se identifica por par canonico y revisiones. Una fusion
  valida domina; 3P consume directamente sus regiones y no repite RANSAC.
- `MatchEvidence` conserva residual, inlier/hard-outlier e IDs raw de ambos
  extremos. La evidencia agrupada mantiene procedencia query/candidate para
  fusion, covisibilidad y evaluacion sparse simetrica.
- Cada `LoopGeometryResult` conserva los inliers de su region y el par de KFs
  que los origino. `LoopTaskComputation::fusion_pairs` ofrece ademas la union
  canonica aplanada para diagnostico, pero 3P debera preparar covisibilidad y
  fusion desde los resultados agrupados para no perder procedencia ni medida.
- La cola coalesce por apariencia, geometria semantica y anchor. El dequeue y
  el commit revalidan ademas `validation_revision`, que incluye la geometria
  exacta; reducir reevaluaciones no permite comprometer una nube antigua.
- Anchors, evidencia de optimizacion y constraints no ancladas requieren dos
  queries independientes compatibles. `BuildAnchorCascade()` puede resolver
  atomicamente un componente conectado cuando uno de sus submapas obtiene
  world.

## Referencias

```text
include/orbslam3_multi/loop_task.hpp
  -> LoopTaskRevision / LoopTask / DatabaseUpdateTask
include/orbslam3_multi/loop_pipeline.hpp
  -> LoopPipelineConfig / LoopTaskComputation / LoopPipeline
src/loop_pipeline.cpp
  -> rg -n "LoopPipeline::(UpsertBow|SearchBow|GroupRegions|VerifyRegion|AddHypothesisEvidence|BuildAnchorCascade|Process)"
```

Tests: `test_loop_pipeline` valida anclaje tras dos queries, dependencia blanda
hasta fiducial hard y coalescencia semantica frente a refinamiento exacto.
Runtime: `[F3N-LOOP-ENQUEUE]` expone raw/appearance/geometry/validation/anchor,
ademas de `[F3O-RANSAC]`, `[F3O-LOOP-DONE]` y `[F3P-FUSION]` cuando la decision
de error bajo llega al commit.
