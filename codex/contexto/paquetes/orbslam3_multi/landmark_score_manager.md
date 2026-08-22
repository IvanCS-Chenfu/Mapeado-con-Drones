# `LandmarkScoreManager`

## Rol

Autoridad numerica unica de score raw y fused. Conserva base ORB, factores
geometricos, evidencias idempotentes, score final y revisiones; ninguna clase
externa modifica valores directamente.

## Politica raw 3S

```text
base_score_orb = clamp(
  0.55 * min(observations_count / 8, 1)
  + 0.35 * found_ratio
  + 0.10 * descriptor_valid,
  0, 1)

score_raw = clamp(
  base_score_orb * distance_factor * isolation_factor
  + positive_adjustment + negative_adjustment,
  0, 1)
```

En el runtime 3S `positive_adjustment` recibe `+0.04` por inlier de fusion
confirmado. `negative_adjustment` se conserva en el modelo/rollback, pero 3S no
genera penalizaciones sparse: la oclusion queda para Fase 8 con nube densa.

- `ApplyRawChanges()` consume solo MPs nuevos o con inputs ORB modificados.
- Un raw no anclado usa factores neutros `1`.
- `ApplyGeometryChanges()` recibe posicion world, distancia al KF observador y
  baseline estereo; puede penalizar y recuperar el score.
- La cercania sospechosa usa umbral metrico fijo `1.0 m` y
  `max(0.05, (distance/near_limit)^2)`; representa plausibilidad fisica y no
  escala con baseline.
- La banda neutra termina en
  `far_limit=max(near_limit,83.333333*baseline)`, o fallback `5.0 m`. Con el
  baseline actual `0.06 m` abarca 1-5 m. Despues usa
  `max(0.25,(far_limit/distance)^2)`.
- El aislamiento se activa solo tras madurez ORB minima. Cuenta ocupacion en
  los 27 voxels vecinos y reduce de forma progresiva si no alcanza soporte.
- Altas, movimientos y bajas agrupan primero voxels afectados; cada punto se
  reevalua en coste constante respecto al tamaño global. Una geometria
  identica solo reevalua el propio MP y no reindexa vecinos.

`ScoreChangeSet` distingue altas, outputs modificados, invalidaciones, cambios
solo de input y fused tracks. `score_revision`/dirty avanzan solo ante salida
material. Evidencia repetida por `evidence_id` es no-op y `RollbackPatch()`
restaura exactamente el estado anterior.

## Configuracion y stats

`LandmarkScoreConfig` contiene radio/minimo/madurez/factor de aislamiento,
umbral/factor de cercania y multiplicador/fallback/factor minimo lejano. Sus
defaults de distancia son `1.0`, `0.05`, `83.333333`, `5.0` y `0.25`.
`GetStats()` expone tracked/bad/anchored/isolated/near/far y min/media/max.

## Referencias

```text
orbslam3_multi/include/orbslam3_multi/landmark_score_manager.hpp
  -> LandmarkScoreConfig / LandmarkScoreGeometryInput / LandmarkScoreManager
  -> rg -n "LandmarkScoreConfig|ApplyGeometryChanges|LandmarkScoreStats"

orbslam3_multi/src/landmark_score_manager.cpp
  -> ApplyRawChanges / ApplyGeometryChanges / DistanceFactor / IsolationFactor
  -> rg -n "ApplyGeometryChanges|DistanceFactor|IsolationFactor|RecomputeOutput"

orbslam3_multi/test/test_landmark_score_manager.cpp
  -> base ORB, distancia recuperable, aislamiento recuperable e inlier posterior
```

El builder publica todos los puntos independientemente del score.

La prueba 194 valida los defaults recalibrados con 24.977 puntos anclados:
`near=99`, `far=11.433` y media `0.2596`, frente a `1`, `24.195` y `0.1502` en
193. Las colas terminan vacias y no aparecen penalizaciones sparse.
