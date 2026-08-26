# `OptimizationValidator`

## Rol

Puerta privada entre propuesta y commit. Clasifica cada pasada como
`AcceptFull`, `AcceptPartialRetry` o `HardFailure`.

## Comprobaciones

- estado y poses finitos;
- propuesta con todos los controles;
- todos los vertices `fixed` exactamente inmoviles, incluidos hard permanentes
  y soportes de consenso temporales;
- target dentro de los tres umbrales para `AcceptFull`;
- progreso positivo y seguro para aceptar un parcial;
- fallo duro ante solver invalido, control movido, propuesta incompleta o falta
  de progreso.

Para `LoopRelative` exige ademas:

- error inicial/final para cada arista `CurrentLoop` y todas dentro del umbral
  de commit seguro `0.25 m/0.15 rad`, no del target de convergencia;
- coste no creciente y mejora de al menos una constraint actual mediante OR;
  no exige hoy mejora o no degradacion individual de cada loop;
- incremento acotado en aristas temporales, `CovisibilityNative` y
  `PriorLoop`;
- pose propuesta para cada KF `previously_optimized`;
- cambio maximo de cada revisitado respecto a su pose optimizada vigente:
  5 m y 0.349066 rad. Un KF nunca optimizado no recibe este limite.

Los defaults declarados por el servidor son: incremento temporal maximo
`2.0 m/0.70 rad`, covisible `1.0 m/0.50 rad` y prior loop
`0.50 m/0.35 rad`; revisitados `5.0 m/0.349066 rad`. Los tres primeros son
limites por familia; el ultimo compara cada pose ya optimizada.

`ValidationResult` expone numero de aristas/KFs comprobados, incrementos
estructurales maximos y cambio maximo de revisitados. El backend traduce
los rechazos estructurales de loop en final sin commit ni fallo bloqueante.

Consecuencia vigente: una `CurrentLoop` inicialmente satisfecha puede empeorar
mientras otra mejora, siempre que todas terminen bajo el commit seguro y el
coste total no crezca. La prueba 220 identifica esta diferencia entre objetivo
del solver y contrato efectivo de aceptacion como punto de reentrada 3Q.

Las revisiones live y la cobertura de late-window/tail se vuelven a comprobar
en el commit atomico del backend. Un conflicto de revision no aplica cambios y
el worker revalida/reconstruye de forma acotada.

## Referencias

```text
include/orbslam3_multi/optimization_validator.hpp
  -> ValidationDecision / ValidationResult / OptimizationValidator
src/optimization_validator.cpp
  -> rg -n "OptimizationValidator::Validate|structural_edges_checked|previously_optimized"
test/test_fiducial_optimization.cpp
  -> accept full/partial, cobertura loop, estructura, consenso y revisitados
```
