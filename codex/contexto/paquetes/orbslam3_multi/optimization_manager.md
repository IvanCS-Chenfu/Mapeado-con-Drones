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
desde la 24 al alcanzar el target independiente `0.05 m/0.03 rad` sin empeorar
el coste. Fusion y commit no controlan la parada. `information_weight` afecta
la relajacion normalizado por familia; el validador conserva la decision final.

En `LoopRelative`, todas las `CurrentLoop` participan y la condicion de parada
exige conjuntamente que cada una alcance `0.05 m/0.03 rad`. Sin embargo, si se
agotan las 160 iteraciones, la implementacion vigente devuelve igualmente
`Converged`; no emite `MaxIterations` ni conserva si se alcanzo el target. Esta
limitacion quedo expuesta por la prueba 220.

La rama `FiducialAbsolute` si conserva su semantica de propuesta parcial y
refinamiento. No debe extrapolarse ese contrato al solve `LoopRelative`.

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
