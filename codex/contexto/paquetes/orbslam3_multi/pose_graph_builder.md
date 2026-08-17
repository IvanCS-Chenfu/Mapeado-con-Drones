# `PoseGraphBuilder`

## Rol

Construye un `PoseGraphProblem` privado y determinista desde el ultimo control
aceptado hasta el KF fiducial target del mismo submapa.

## Comportamiento

- exige control temporalmente anterior, existente y hard;
- exige target raw/world activo;
- conserva el hard control aunque haya quedado inactivo tras un snapshot;
- omite KFs intermedios raw o world inactivos sin reactivarlos;
- selecciona `max(2, ceil(0.30 * window))` controles;
- combina cobertura por camino 3D, densidad temporal y preferencia por esquinas
  SE(3);
- protege vecindades de extremos con ratio 0.20;
- crea aristas temporales SE(3) entre controles y un plan de propagacion para
  no controles.

La covisibilidad y las ventanas multi-submapa no forman parte de 3I.

## Referencias

```text
include/orbslam3_multi/pose_graph_problem.hpp
  -> PoseGraphProblem / PoseGraphKeyFrame / PoseGraphPropagationEntry
include/orbslam3_multi/pose_graph_builder.hpp
  -> PoseGraphBuilder::Build
src/pose_graph_builder.cpp
  -> rg -n "PoseGraphBuilder::Build|CornerStrength|control_vertex_ratio"
test/test_fiducial_optimization.cpp
  -> cobertura 30 %, extremos e intermedios inactivos
```
