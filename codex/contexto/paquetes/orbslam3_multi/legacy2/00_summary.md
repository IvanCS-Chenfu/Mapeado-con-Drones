# 00_summary - orbslam3_multi

Backend algoritmico de Fase 1. Contiene las bases raw/poses/covisibilidad,
anchors fiduciales, BoW, verificacion geometrica, fusion, grafo, optimizacion y
construccion de la vista global.

## Autoridades

- `RawMapDatabase`: estado ORB-SLAM3 crudo; fusion/optimizacion no lo modifica.
- `GlobalPoseStore`: anchors, poses world, accepted/optimized/propagated y
  autoridad de KFs futuros.
- `CovisibilityDatabase`: aristas ORB nativas y geometricas del servidor.
- `FusedLandmarkManager`: tracks transitivos de `RawMapPointId` distintos.
- `LandmarkScoreManager`: scores raw/fused consumidos por publicacion.

## Flujo fiducial

`FiducialAnchorManager` crea el primer anchor. Una revisit de error alto produce
`FiducialOptimizationTask`. `PoseGraphBuilder::BuildForFiducialTask` construye
el tramo entre hard fiducials; `OptimizationManager` resuelve/valida sobre una
copia y el servidor intercambia el lote aceptado en `GlobalPoseStore`.

## Flujo loop

`LoopDetector` propone candidatos BoW. En el runtime actual:

- una query atrasada no compara con KFs posteriores del mismo submapa;
- `near_same_submap` sigue disponible para fusion/evidencia, pero una decision
  de error alto cercana no puede optimizar poses;
- `SubcloudLoopVerifier` produce medida relativa e inliers;
- `LoopDecisionManager` decide fusion u optimizacion;
- `PoseGraphBuilder::BuildForLoopTask` crea una arista `LOOP_RELATIVE`;
- el lado candidato es referencia fija y no existe prior absoluto falso;
- `OptimizationManager` evalua before/after sobre esa restriccion relativa;
- un accept puede actualizar poses y fusionar inliers dentro del mismo task.

La ruta de loop requiere soporte geometrico previo antes de construir el
grafo. Hard fiducials permanecen inmoviles.

## Publicacion

`GlobalMapBuilder` consume snapshots de raw, poses, fused tracks y scores.
Proyecta cada MapPoint desde un KF con pose world utilizable; no usa fallback
rigido para puntos sin KF valido. Devuelve una vista coherente para nube/KFs.

## APIs/archivos clave

```text
include/orbslam3_multi/raw_map_database.hpp
include/orbslam3_multi/global_pose_store.hpp
include/orbslam3_multi/loop_candidate.hpp
include/orbslam3_multi/loop_optimization_task.hpp
include/orbslam3_multi/pose_graph_problem.hpp
src/loop_detector.cpp
src/subcloud_loop_verifier.cpp
src/loop_decision_manager.cpp
src/pose_graph_builder.cpp
src/optimization_manager.cpp
src/global_map_builder.cpp
```

## Tests relevantes

- `test_loop_optimization_task`: arista relativa, ausencia de prior absoluto,
  dry-run sin mutacion y filtro causal de candidatos.
- `test_global_pose_store_tail_anchor`: autoridad accepted/tail y rollback.
- `test_covisibility_database`: importacion y fuerza de aristas.
- `test_fused_landmark_manager`: tracks transitivos y publicacion sin
  duplicados raw.
- `test_loop_pair_attempt_database`: cache canonica por revisiones.

Todos los tests anteriores pasan tras la integracion de `prueba_76`.

## Estado

- `3I-3L` fiducial: cerradas para su contrato historico y reutilizadas.
- `3M-3O`: integradas dentro de `LoopTask`.
- `3P`: `PARCIAL`; fusion activa, pendiente cierre algoritmico global.
- `3Q`: `PARCIAL`; arquitectura relativa/causal validada, pendiente ampliar
  accepts live validos tras la ultima guarda.

Detalles en los MDs de cada componente de este directorio.
