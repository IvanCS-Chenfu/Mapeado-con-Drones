# `PoseGraphBuilder`

## Rol

Construye problemas SE(3) privados y deterministas para fiducial absoluto y
loop relativo. Ambos reutilizan controles, aristas temporales, covisibilidad
nativa confirmada, constraints server, dependencias soft y planes de
propagacion multi-submapa.

## Comportamiento

- fiducial exige control temporalmente anterior, existente y hard;
- exige target raw/world activo;
- conserva el hard control aunque haya quedado inactivo tras un snapshot;
- omite KFs intermedios raw o world inactivos sin reactivarlos;
- selecciona `max(2, ceil(0.30 * window))` controles;
- combina cobertura por camino 3D, densidad temporal y preferencia por esquinas
  SE(3);
- protege vecindades de extremos con ratio 0.20;
- crea aristas temporales SE(3) entre controles y un plan de propagacion para
  no controles;
- `Build()` fiducial parte del snapshot target y expande la componente solo por
  loops/fusiones `ServerLoopGeometric` y dependencias soft confirmadas; la
  covisibilidad ORB nativa refuerza submapas ya incluidos, no los descubre;
- `BuildLoop()` obtiene el subgrafo conectado de query/candidate hasta
  autoridades hard, siguiendo tramos temporales, todas las fusiones server
  transitivas, dependencias blandas y constraints previas;
- un hijo soft conectado a un tramo delimitado por hard se incluye en la
  ventana conjunta;
- los hard fiduciales son vertices fijos; una arista `CurrentLoop` conserva la
  medida relativa RANSAC y no fija unilateralmente query ni candidate;
- `BuildLoop()` incorpora entre una y tres regiones coherentes de error alto
  como aristas `CurrentLoop`, deduplicadas y con cobertura final obligatoria;
- el problema conserva aristas estructurales temporales, covisibles, de fusion
  previa y referencias de corredor hard-hard para validacion before/after;
- la densidad base del 30 % se amplia con endpoints de constraints. La
  covisibilidad nativa se mantiene sparse: maximo seis aristas fuertes por
  control para evitar grafos densos cuadraticos.

## Referencias

```text
include/orbslam3_multi/pose_graph_problem.hpp
  -> PoseGraphProblem / PoseGraphKeyFrame / PoseGraphPropagationEntry
include/orbslam3_multi/pose_graph_builder.hpp
  -> PoseGraphBuilder::Build / BuildLoop
src/pose_graph_builder.cpp
  -> rg -n "PoseGraphBuilder::(Build|BuildLoop)|AppendNativeCovisibilityEdges|control_vertex_ratio"
test/test_fiducial_optimization.cpp
  -> cobertura, hard, covisibilidad sparse, loop relativo y dependencia soft
```

Los tests de 3Q cubren expansion fiducial multi-submapa, cierre transitivo,
varias `CurrentLoop`, componente soft y corredores hard-hard.
