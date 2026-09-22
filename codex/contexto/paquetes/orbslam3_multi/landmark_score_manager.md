# `LandmarkScoreManager`

## Rol

Autoridad numerica unica de score raw y fused. Conserva base ORB, factores
geometricos, evidencias idempotentes, score final y revisiones; ninguna clase
externa modifica valores directamente.

## Politica raw 3R

```text
base_score_orb = clamp(
  0.55 * min(observations_count / 8, 1)
  + 0.35 * found_ratio
  + 0.10 * descriptor_valid,
  0, 1)

score_raw = clamp(
  base_score_orb * distance_factor * isolation_factor * body_factor
  + positive_adjustment + negative_adjustment,
  0, 1)
```

Si un punto ya maduro no tiene ningun vecino valido dentro del radio de
aislamiento y `isolation_min_factor=0`, el resultado es un cero duro: no lo
puede elevar un ajuste positivo posterior. Al aparecer un vecino, el factor se
recupera y el score vuelve a calcularse desde la evidencia conservada.

`body_factor` es tambien un corte duro y reversible. Vale cero cuando la
posicion global del MapPoint cae dentro de la esfera fisica de cualquier
KeyFrame global activo cuyo dron tenga dimensiones registradas en
`/mission/registry`. La esfera se centra en `base_link`, usa como radio la
semidiagonal de `dimensions_m` y no anade margen de seguridad. No depende de
ownership ni de la relacion observador-observado del MapPoint.
`LandmarkScoreConfig::drone_body_mask_enabled=false` neutraliza este factor a
`1`, rechaza las actualizaciones de esferas antes de indexarlas y no altera
distancia, aislamiento, base ORB ni evidencia de fusion.

En el runtime 3R `positive_adjustment` recibe `+0.04` por inlier de fusion
confirmado. `negative_adjustment` se conserva en el modelo/rollback, pero 3R no
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
- El aislamiento se activa solo tras madurez ORB minima. El indice consulta las
  27 celdas candidatas, pero cuenta solo MapPoints validos cuya distancia
  euclidea exacta es menor o igual al radio. Con la configuracion activa son
  `0.30 m`, dos vecinos y tres observaciones; un aislado publica score `0`.
- Altas, movimientos y bajas agrupan primero voxels afectados; cada punto se
  reevalua en coste constante respecto al tamaño global. Una geometria
  identica solo reevalua el propio MP y no reindexa vecinos.
- En el flujo principal vigente, `SparseGlobalBackend::InsertDelta()` incluye
  tambien `score_input_changed_mappoint_ids` en `RefreshGeometryScores()`. Por
  ello cambios estadisticos ORB como `found_ratio`, aun sin movimiento, vuelven
  a construir la entrada geometrica y `ApplyGeometryChanges()` reevalua el MP
  identico. La prueba GT de dos drones del 2026-09-21 confirma que este trabajo
  se combina con cientos o miles de MPs por delta y amplifica el coste previo a
  publicar. Esta ruta ya existia antes de `cd50829`, pero entonces
  `NeighborCount()` sumaba ocupacion de 27 celdas; ese commit hizo exacto el
  radio y paso a recorrer, consultar y medir cada candidato de las 27 celdas.
  La combinacion frecuencia antigua + coste nuevo explica la regresion. Retirar
  la reevaluacion geometrica de cambios solo estadisticos exige conservar
  `ApplyRawChanges()` como autoridad de los cambios de score base.
- `IsolationFactor()` solo distingue si hay menos de
  `isolation_min_neighbors`, pero `NeighborCount()` recorre hoy todos los
  candidatos incluso despues de alcanzar ese minimo. Un corte temprano al
  alcanzar el umbral conserva exactamente el resultado publicado y reduce el
  coste en zonas densas.
- `UpdateDroneBodySpheres()` mantiene un segundo indice de esferas por KF. Al
  mover, invalidar o registrar una esfera solo consulta las celdas que
  intersectan sus volumenes antiguo/nuevo; las altas de MapPoint resuelven su
  mascara contra las esferas de su celda. Nunca se recorre la nube completa.

`ScoreChangeSet` distingue altas, outputs modificados, invalidaciones, cambios
solo de input y fused tracks. `score_revision`/dirty avanzan solo ante salida
material. Evidencia repetida por `evidence_id` es no-op y `RollbackPatch()`
restaura exactamente el estado anterior.

## Configuracion y stats

`LandmarkScoreConfig` contiene radio/minimo/madurez/factor de aislamiento,
flag de mascara fisica, umbral/factor de cercania y
multiplicador/fallback/factor minimo lejano. Sus
defaults de distancia son `1.0`, `0.05`, `83.333333`, `5.0` y `0.25`.
`GetStats()` expone tracked/bad/anchored/isolated/body_masked/near/far y
min/media/max.

## Referencias

```text
orbslam3_multi/include/orbslam3_multi/landmark_score_manager.hpp
  -> LandmarkScoreConfig / LandmarkScoreGeometryInput / LandmarkScoreManager
  -> rg -n "LandmarkScoreConfig|ApplyGeometryChanges|LandmarkScoreStats"

orbslam3_multi/src/landmark_score_manager.cpp
  -> ApplyRawChanges / ApplyGeometryChanges / DistanceFactor / IsolationFactor
  -> UpdateDroneBodySpheres / RecomputeOutput
  -> rg -n "ApplyGeometryChanges|UpdateDroneBodySpheres|BodyFactor|RecomputeOutput"

orbslam3_multi/test/test_landmark_score_manager.cpp
  -> base ORB, distancia recuperable, corte duro/recovery exacto de aislamiento
     e inlier posterior

orbslam3_multi/test/test_fused_landmark_manager.cpp
  -> track formado solo por miembros aislados publica score fused cero
```

El builder publica todos los puntos independientemente del score.

La prueba 194 valida los defaults recalibrados con 24.977 puntos anclados:
`near=99`, `far=11.433` y media `0.2596`, frente a `1`, `24.195` y `0.1502` en
193. Las colas terminan vacias y no aparecen penalizaciones sparse.
