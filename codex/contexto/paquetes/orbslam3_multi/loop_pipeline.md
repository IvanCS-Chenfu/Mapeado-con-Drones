# `LoopPipeline`

## Rol

Implementa la busqueda y decision 3N-3Q dentro de una sola `LoopTask`: indice
BoW derivado, regiones, rama rapida world, subnubes, matching ORB, RANSAC y
seleccion de fusion u optimizacion relativa.

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
- La cache negativa se identifica por par canonico y revisiones. Una region
  robusta de error alto con apoyo independiente domina sobre regiones
  fusionables de la misma tarea; se seleccionan hasta tres regiones coherentes
  para 3Q. Solo cuando no existe esa evidencia domina 3P, que consume las
  regiones fusionables sin repetir RANSAC.
- `MatchEvidence` conserva residual, inlier/hard-outlier e IDs raw de ambos
  extremos. La evidencia agrupada mantiene procedencia query/candidate para
  fusion, covisibilidad y evaluacion sparse simetrica.
- El error de loop compara la relacion world actual entre los dos KFs con la
  relacion local medida por RANSAC. No compara anchors estaticos, evitando
  optimizaciones falsas despues de un commit previo.
- Cada `LoopGeometryResult` conserva los inliers de su region y el par de KFs
  que los origino. `LoopTaskComputation::fusion_pairs` ofrece ademas la union
  canonica aplanada para diagnostico, pero 3P debera preparar covisibilidad y
  fusion desde los resultados agrupados para no perder procedencia ni medida.
- La cola coalesce por apariencia, geometria semantica y anchor. El dequeue y
  el commit revalidan ademas `validation_revision`, que incluye la geometria
  exacta; reducir reevaluaciones no permite comprometer una nube antigua.
- Anchors y constraints no ancladas requieren dos queries independientes.
  La evidencia de optimizacion usa apoyo adaptativo 2/4/6 segun
  perdida/asimetria, ambiguedad y correccion grande, conservando progresion
  coherente de query y candidate. `BuildAnchorCascade()` puede resolver
  atomicamente un componente conectado cuando uno de sus submapas obtiene
  world.
- `RecentLossRecoveryContext` habilita excepcionalmente apoyo 1/1 si el nuevo
  submapa permanece dentro de los limites configurados respecto al ultimo
  control perdido. La constraint resultante es provisional: no entra en
  fusion, scaffold ni cascada hasta recibir un segundo apoyo independiente o
  un fiducial. Al superar el recorrido maximo se conserva el apoyo 2/4/6.
- `BuildAnchorCascade()` tambien se invoca explicitamente cuando aparece nueva
  autoridad world por fiducial; recorre solo constraints activas no
  provisionales de la componente.
- La independencia evita falsos positivos repetitivos. Los motivos de espera,
  competencia y ambiguedad quedan expuestos para distinguir demora legitima de
  falta de evidencia.
- `OptimizationEvidence` conserva regiones e inliers en la computacion; el
  backend construye/optimiza/valida/compromete y puede llamar directamente a
  fusion 3P sin crear otra tarea.
- `LoopTask::intent` separa tareas `Full` de reruns `FusionRefresh`. Estos
  ultimos conservan BoW, RANSAC, fusion y score, pero convierten evidencia de
  optimizacion en un final no recursivo; una nueva revision raw normal vuelve a
  crear una tarea `Full`.
- `FusionRefresh` no repite una busqueda BoW global. Calcula el AABB world de la
  subnube directa del query y solo conserva candidatos cuyas subnubes solapan o
  quedan dentro de `fusion_refresh_spatial_margin_m` (1 m por defecto). Si no
  queda ninguno termina como `fusion_refresh_no_spatial_candidates` antes de
  RANSAC; `Full` mantiene la busqueda global normal.
- La telemetria `refresh_spatial=(accepted,rejected)` de `[F3O-LOOP-DONE]`
  permite medir el filtro sin alterar las decisiones de una tarea normal.
- La seleccion de optimizacion vigente toma hasta tres regiones RANSAC
  compatibles con la hipotesis soportada y no fusionables. No exige que cada
  region seleccionada tenga error world alto; la prueba 220 demuestra que una
  constraint alta aislada entre candidatos vecinos con constraints ya
  satisfechas puede ampliar indebidamente el solve. Esto no identifica por si
  solo que pose del par es incorrecta. Es el punto de reentrada actual de 3Q.

## Referencias

```text
include/orbslam3_multi/loop_task.hpp
  -> LoopTaskRevision / LoopTask / DatabaseUpdateTask
include/orbslam3_multi/loop_pipeline.hpp
  -> LoopPipelineConfig / LoopTaskComputation / LoopPipeline
src/loop_pipeline.cpp
  -> rg -n "LoopPipeline::(UpsertBow|SearchBow|GroupRegions|VerifyRegion|AddHypothesisEvidence|BuildAnchorCascade|Process)"
```

Tests: `test_loop_pipeline` valida anclaje tras dos queries, dependencia blanda,
coalescencia semantica, optimizacion+fusion dentro de la misma tarea y rechazo
espacial temprano de `FusionRefresh` lejano. Tambien cubre apoyo adaptativo,
cascada al aparecer world, recuperacion reciente 1/1 estricta y fallback al
superar el recorrido maximo.
Runtime: `[F3N-LOOP-ENQUEUE]` expone raw/appearance/geometry/validation/anchor,
ademas de `[F3O-RANSAC]`, `[F3O-LOOP-DONE]` y `[F3P-FUSION]` cuando la decision
de error bajo llega al commit.
