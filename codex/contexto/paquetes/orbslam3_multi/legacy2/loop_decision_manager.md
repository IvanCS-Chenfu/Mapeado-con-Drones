# `LoopDecisionManager`

## Rol

`LoopDecisionManager` consume en `3P` las decisiones geométricas positivas de
3O sin ejecutar optimización.

## Archivos

```text
orbslam3_multi/include/orbslam3_multi/loop_decision_manager.hpp
orbslam3_multi/src/loop_decision_manager.cpp
```

## Comportamiento

Para `FUSION_CANDIDATE`:

1. obtiene las poses globales actuales de query y candidate;
2. deriva de la corrección RANSAC la relación relativa medida entre KFs;
3. calcula por separado la relación relativa current desde `GlobalPoseStore`;
4. inserta `SERVER_LOOP_GEOMETRIC` en `CovisibilityDatabase`;
5. conserva soporte y cobertura 2D/3D de la verificación;
6. entrega a `FusedLandmarkManager` solo pares inlier de IDs distintos.

Para `LOOP_OPTIMIZATION_CANDIDATE` registra igualmente la covisibilidad
confirmada y devuelve `optimization_pending_recorded`, sin fusionar, crear
grafos, modificar poses ni iniciar optimización. `REJECT`, `HOLD` y
`ALREADY_CONFIRMED_COVISIBILITY` quedan fuera del apply de 3P.

`relative_pose_measured` y `relative_pose_current` no se copian ciegamente:

```text
current = world_T_query^-1 * world_T_candidate
measured = (ransac_correction * world_T_query)^-1 * world_T_candidate
```

## Salida

`LoopDecisionResult` indica si la covisibilidad fue creada o reforzada y conserva
el resultado detallado de creación, ampliación, refuerzo, merge o rechazo de
cada pareja.
