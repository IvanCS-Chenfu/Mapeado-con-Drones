# `OptimizationValidator`

## Rol

Puerta privada entre propuesta y commit. Clasifica cada pasada como
`AcceptFull`, `AcceptPartialRetry` o `HardFailure`.

## Comprobaciones

- estado y poses finitos;
- propuesta con todos los controles;
- todos los vertices fixed/hard exactamente inmoviles;
- target dentro de los tres umbrales para `AcceptFull`;
- progreso positivo y seguro para aceptar un parcial;
- fallo duro ante solver invalido, control movido, propuesta incompleta o falta
  de progreso.

Para `LoopRelative` exige ademas:

- error inicial/final para cada arista `CurrentLoop` y todas dentro del umbral;
- coste no creciente y mejora de al menos una constraint actual;
- incremento acotado en aristas temporales, `CovisibilityNative` y
  `PriorLoop`;
- pose propuesta para cada KF de corredor hard-hard;
- exceso de corredor final no mayor que el exceso inicial. Un exceso heredado
  puede conservarse o mejorar, nunca empeorar.

Los defaults declarados por el servidor son: incremento temporal maximo
`2.0 m/0.70 rad`, covisible `1.0 m/0.50 rad` y prior loop
`0.50 m/0.35 rad`; corredor hard-hard `5.0 m/0.349066 rad`. Son limites por
tipo de arista, no un criterio visual global sobre toda la ventana.

`ValidationResult` expone numero de aristas/KFs comprobados, incrementos
estructurales maximos y excesos de corredor before/after. El backend traduce
los rechazos estructurales de loop en final sin commit ni fallo bloqueante.

Las revisiones live y la cobertura de late-window/tail se vuelven a comprobar
en el commit atomico del backend. Un conflicto de revision no aplica cambios y
el worker revalida/reconstruye de forma acotada.

## Referencias

```text
include/orbslam3_multi/optimization_validator.hpp
  -> ValidationDecision / ValidationResult / OptimizationValidator
src/optimization_validator.cpp
  -> rg -n "OptimizationValidator::Validate|structural_edges_checked|hard_corridor"
test/test_fiducial_optimization.cpp
  -> accept full/partial, cobertura loop, estructura y corredor
```
