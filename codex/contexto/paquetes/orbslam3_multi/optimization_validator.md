# `OptimizationValidator`

## Rol

Puerta privada entre propuesta y commit. Clasifica cada pasada como
`AcceptFull`, `AcceptPartialRetry` o `HardFailure`.

## Comprobaciones

- estado y poses finitos;
- propuesta con todos los controles;
- control inicial exactamente inmovil;
- target dentro de los tres umbrales para `AcceptFull`;
- progreso positivo y seguro para aceptar un parcial;
- fallo duro ante solver invalido, control movido, propuesta incompleta o falta
  de progreso.

Las revisiones live y la cobertura de late-window/tail se vuelven a comprobar
en el commit atomico del backend. Un conflicto de revision no aplica cambios y
el worker revalida/reconstruye de forma acotada.

## Referencias

```text
include/orbslam3_multi/optimization_validator.hpp
  -> ValidationDecision / ValidationResult / OptimizationValidator
src/optimization_validator.cpp
  -> rg -n "OptimizationValidator::Validate|AcceptPartialRetry|HardFailure"
test/test_fiducial_optimization.cpp
  -> accept full y partial retry
```
