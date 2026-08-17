# `LandmarkScoreManager`

## Rol

Base autoritativa del score inicial de cada `RawMapPointId`. No duplica la
geometria raw: conserva score, revision, observaciones, `found_ratio`, validez
del descriptor y estado `is_bad`.

```text
score = clamp(
  0.55 * min(observations_count / 8, 1)
  + 0.35 * clamp(found_ratio, 0, 1)
  + 0.10 * descriptor_valid,
  0, 1)
is_bad => score 0
```

## Flujo

- `ApplyRawChanges()` consume MPs nuevos y `score_input_changed_mappoint_ids`;
  no recalcula score por un cambio exclusivamente geometrico.
- Los inputs candidatos se leen en un batch ligero de `RawMapDatabase`; no se
  copia un `OrbMapPoint` completo ni se toma un lock raw por candidato.
- `invalidated_mappoint_ids` fuerza score cero y conserva el registro para
  trazabilidad.
- Un punto que ORB marca `is_bad` permanece trazable con score cero mientras
  exista en `RawMapDatabase`.
- Un registro incrementa su `record_revision` cuando cambia cualquiera de sus
  inputs ORB. `input_updated_ids` permite actualizar esa trazabilidad sin
  afirmar que el score visible ha cambiado.
- `score_revision`, `updated_ids` y el dirty del builder solo avanzan cuando
  cambia el score calculado o `is_bad`. Por ejemplo, pasar de 8 a 12
  observaciones mantiene el termino saturado y no recalcula la vista.
- `ScoreChangeSet` separa creados, score actualizado, invalidaciones e inputs
  internos; el builder recibe exclusivamente IDs con salida material.
- `GetStats()` devuelve revision, puntos seguidos/bad y min/media/max.
- `PrepareScorePatch()` y `ApplyScorePatch()` aplican evidencia idempotente por
  `evidence_id`; `RollbackScorePatch()` restaura exactamente el estado previo
  si falla otra base del commit 3P.
- Los inliers confirmados suman `+0.04` por endpoint. Los hard outliers aislados
  son diagnostico; solo una expectativa sparse simetrica fiable aplica
  `-0.01` por miss esperado o `-0.03` por contradiccion foreground.
- Cada fused track conserva score proyectado, procedencia y revisiones. Su base
  provisional usa el maximo de sus miembros y bonos acotados por miembros,
  drones y submapas; los deltas de evidencia se mantienen separados.

## Referencias

```text
orbslam3_multi/include/orbslam3_multi/landmark_score_manager.hpp
  -> LandmarkScoreRecord / ScoreChangeSet / LandmarkScoreManager
  -> rg -n "LandmarkScoreRecord|ScorePatch|FusedLandmarkScoreRecord|class LandmarkScoreManager"

orbslam3_multi/src/landmark_score_manager.cpp
  -> ComputeOrbScore / ApplyRawChanges / PrepareScorePatch / rollback
  -> rg -n "ComputeOrbScore|ApplyRawChanges|PrepareScorePatch|ApplyScorePatch|RollbackScorePatch"

orbslam3_multi/test/test_landmark_score_manager.cpp
  -> formula, no-op, actualizacion focal, is_bad=0 y saturacion sin dirty
```

3P implementa el score geometrico minimo necesario para fusion. 3S completara
la politica general y el GUI filtrara por score en fase 7; el builder sigue
publicando todos los puntos.
