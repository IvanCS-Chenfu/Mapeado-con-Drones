# `FusedLandmarkManager`

## Rol

Autoridad derivada 3P de identidad fusionada. Une `RawMapPointId` sin modificar
`RawMapDatabase`, mantiene tracks transitivos con ID estable y prepara patches
privados que el backend compromete junto con covisibilidad y score.

## Contrato

- `PrepareFusion()` consume todas las regiones RANSAC compatibles de una
  `LoopTaskComputation`; no repite BoW, matching ni RANSAC.
- `member_to_track_` resuelve raw a track en O(1). Dos tracks se unen bajo el ID
  menor; un miembro nuevo se incorpora al track existente.
- Cada track conserva miembros, evidencias idempotentes, KFs observadores,
  drones/submapas de procedencia, descriptor medoid y miembro representante.
- La posicion publicable se calcula de forma ponderada desde miembros world. El
  guard inicial de dispersion es `0.50 m`; una union incompatible se rechaza
  sin alterar estado.
- `FusionPatch` incluye revisiones esperadas, tracks finales, bajas,
  asignaciones, score raw/fused y diagnostico de visibilidad.
- `ApplyPatch()` devuelve `FusionChangeSet` exacto y un rollback. El backend
  puede llamar a `RollbackPatch()` si falla score o covisibilidad.
- La evaluacion sparse simetrica usa calibracion pinhole y recorre toda la
  evidencia negativa elegible de cada region compatible. No existe corte por
  reloj; el trabajo queda acotado estructuralmente por regiones y subnubes.
- Un track absorbido se elimina de `touched_tracks` en el mismo prepare. Las
  referencias locales se comprueban antes de acceder para que una cadena
  transitoria no pueda lanzar `map::at`.

## Referencias

```text
orbslam3_multi/include/orbslam3_multi/fused_landmark_types.hpp
  -> FusedLandmarkTrack / FusionPatch / FusionChangeSet / FusionRollbackPatch
  -> rg -n "FusedLandmarkTrack|FusionPatch|FusionChangeSet"

orbslam3_multi/include/orbslam3_multi/fused_landmark_manager.hpp
  -> FusedLandmarkManager
  -> rg -n "class FusedLandmarkManager|PrepareFusion|ApplyPatch|RollbackPatch"

orbslam3_multi/src/fused_landmark_manager.cpp
  -> preparacion, union transitiva, visibilidad y commit
  -> rg -n "PrepareFusion|touched_tracks|ApplyPatch|RollbackPatch"

orbslam3_multi/test/test_fused_landmark_manager.cpp
  -> union transitiva, guard de dispersion, regresion touch+merge+retire y
     recorrido completo de contradicciones de visibilidad
```

## Telemetria

`[F3P-FUSION]` expone prepare/commit/stale/rollback, pares, tracks
creados/actualizados/retirados, miembros ocultos, score, covisibilidad,
regiones iniciadas/completadas, proyecciones y tiempos. El tiempo de commit es
telemetria, no umbral de aceptacion ni warning. La prueba 161 recorre `56/56`
regiones y drena todos los retries sin excepcion ni fallo duro.
