# `FusedLandmarkManager`

## Rol

Autoridad derivada de identidad fusionada. Une `RawMapPointId` sin modificar
`RawMapDatabase`, mantiene tracks transitivos con ID estable y prepara patches
privados que el backend coordina con covisibilidad y score.

## Contrato

- `PrepareFusion()` reutiliza regiones RANSAC compatibles de la misma
  `LoopTaskComputation`.
- Dos tracks se unen bajo ID estable; altas, extensiones, merges y retiradas
  producen `FusionChangeSet` exacto.
- Cada track conserva miembros, KFs observadores, procedencia, descriptor
  medoid, representante y revisiones esperadas.
- El guard de dispersion por defecto es `0.50 m`; una union incompatible no
  altera estado.
- `ApplyPatch()` devuelve rollback para que el backend revierta si falla otra
  autoridad del commit logico.

## Score 3R

```text
score_fused = clamp(media(score_raw de todos los miembros) + 0.04 * N, 0, 1)
```

Cada penalizacion geometrica permanece en el score raw del miembro afectado.
La media permite que miembros posteriores con observacion valida diluyan una
penalizacion cercana antigua; el track no conserva cap ni castigo permanente.

`N` es el numero total de miembros, por lo que el primer track de dos raw suma
`0.08`. La misma fusion añade ademas `+0.04` raw a cada endpoint como evidencia
idempotente. `BuildScoreUpdatesForMembers()` localiza y recalcula solo tracks
que contienen raw scores modificados posteriormente.

La visibilidad sparse simetrica conserva depth buffers compartidos y contadores
de proyecciones/contradicciones, pero no emite deltas negativos en 3R. Una
tarea rejected/stale conserva `dirty=0` y ninguna evidencia se vuelve visible.

## Referencias

```text
orbslam3_multi/include/orbslam3_multi/fused_landmark_types.hpp
  -> FusionPatch / visibility_diagnostic_events / FusionChangeSet

orbslam3_multi/include/orbslam3_multi/fused_landmark_manager.hpp
  -> FusedLandmarkConfig / FusedLandmarkManager

orbslam3_multi/src/fused_landmark_manager.cpp
  -> FusedScore / PrepareFusion / BuildScoreUpdatesForMembers
  -> rg -n "FusedScore|visibility_diagnostic_events|BuildScoreUpdatesForMembers"

orbslam3_multi/test/test_fused_landmark_manager.cpp
  -> union transitiva, media+bonus y visibilidad diagnostica sin negativos
```

## Telemetria

`[F3P-FUSION]` conserva el resumen estructural y
`[F3R-FUSED-SCORE-COMMIT]` separa committed, positivos, negativos,
diagnosticos y raw dirty. La prueba 193 observa 166 intentos muestreados, 77
commits, 12.672 positivos, 6.319 diagnosticos, 4.528 raw dirty y cero eventos
negativos.
