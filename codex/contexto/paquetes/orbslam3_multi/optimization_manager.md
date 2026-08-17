# `OptimizationManager`

## Rol

Resuelve el grafo fiducial sobre una copia privada y devuelve una
`OptimizationProposal`. No escribe bases, no publica y no toma locks live.

## Algoritmo activo

Convierte la correccion absoluta del target en SE(3), fija exactamente el
primer control y distribuye una correccion suave entre controles. Las
vecindades protegidas inicial/final se mueven rigidamente con su extremo y el
target alcanza la observacion absoluta con la configuracion normal
`max_correction_fraction_per_pass=1.0`.

Con una fraccion menor puede devolver `MaxIterations` y un candidato parcial
finito para que 3L lo valide y 3K lo comprometa/refine sin ceder el worker.

## Referencias

```text
include/orbslam3_multi/optimization_manager.hpp
  -> OptimizationProposal / OptimizationSolverStatus / OptimizationManager
include/orbslam3_multi/pose_geometry.hpp
  -> conversion e interpolacion SE(3), errores y finitud
src/optimization_manager.cpp
  -> rg -n "OptimizationManager::Optimize|correction_fraction|protected_neighborhood"
```

La formulacion toma de `legacy2` sus propiedades estables: SE(3), control fijo,
target absoluto y vecindades rigidas. No porta su infraestructura, dumps ni
variantes experimentales.
