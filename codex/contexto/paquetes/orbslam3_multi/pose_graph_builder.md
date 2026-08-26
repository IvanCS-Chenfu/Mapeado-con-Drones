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
- `BuildSegmented()` es la seleccion comun: `BuildLoop()` aporta endpoints
  RANSAC y `BuildExpandedFiducial()` el target fiducial real, sin loop sintetico;
- cada endpoint selecciona intervalos delimitados por hard anterior/posterior;
  intervalos separados no crean temporal ni propagacion a traves del hueco;
- otro intervalo entra solo por `ServerLoopGeometric` o dependencia soft
  incidente; la covisibilidad ORB nativa refuerza, pero no descubre submapas;
- un hijo soft conectado a un tramo delimitado por hard se incluye en la
  ventana conjunta;
- los hard fiduciales son vertices fijos; una arista `CurrentLoop` conserva la
  medida relativa RANSAC y no fija unilateralmente query ni candidate;
- `BuildLoop()` incorpora entre una y tres regiones coherentes de error alto
  como aristas `CurrentLoop`, deduplicadas y con cobertura final obligatoria;
- el problema conserva estructura temporal, covisible y de fusion previa, y
  marca poses `LoopOptimized`/`FiducialOptimized` como revisitadas;
- al menos tres segmentos soporte, excluido query, con cobertura server minima
  60 % quedan `consensus_fixed` solo durante ese solve, nunca hard persistentes;
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
  -> rg -n "PoseGraphBuilder::(Build|BuildLoop|BuildExpandedFiducial|BuildSegmented)|consensus_submaps|control_vertex_ratio"
test/test_fiducial_optimization.cpp
  -> cobertura, hard, covisibilidad sparse, loop relativo y dependencia soft
```

Los tests 3Q cubren segmentacion loop/fiducial, incidencia server, varias
`CurrentLoop`, consenso temporal, pesos y origen optimizado.
