# `OptimizationManager`

## Rol

Resuelve sobre una copia privada el grafo comun fiducial/loop y devuelve una
`OptimizationProposal`. No escribe bases, no publica y no toma locks live.

## Algoritmo activo

En `FiducialAbsolute` fija los hard, lleva el target a su observacion absoluta
y distribuye la correccion mediante aristas temporales/covisibles. En
`LoopRelative` minimiza conjuntamente aristas temporales,
`CovisibilityNative`, constraints `PriorLoop` y la medida `CurrentLoop`; query
y candidate pueden moverse y ningun prior world artificial fija un lado.

El solver iterativo conserva el limite de 160 iteraciones, pero puede terminar
desde la 24 cuando el error relativo cae por debajo de un cuarto de los
umbrales, el coste no empeora y se alcanza convergencia practica. El validador
sigue siendo la autoridad de accept: el corte temprano no compromete poses.

Con una fraccion menor puede devolver `MaxIterations` y un candidato parcial
finito para que 3L lo valide y 3K lo comprometa/refine sin ceder el worker.

## Referencias

```text
include/orbslam3_multi/optimization_manager.hpp
  -> OptimizationProposal / OptimizationSolverStatus / OptimizationManager
include/orbslam3_multi/pose_geometry.hpp
  -> conversion e interpolacion SE(3), errores y finitud
src/optimization_manager.cpp
  -> rg -n "OptimizationManager::Optimize|FiducialAbsolute|LoopRelative|practical_convergence"
```

La formulación vigente conserva SE(3), control fijo, target absoluto y
vecindades rígidas, recuperados durante la reconstrucción. La infraestructura,
dumps y variantes experimentales anteriores fueron retirados en 3T.
