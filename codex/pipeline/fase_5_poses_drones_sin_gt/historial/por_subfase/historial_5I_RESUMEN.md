# Historial 5I - Resumen

## Estado agregado

`CONSEGUIDA` para movimiento ORB dentro del dominio de evidencia visual
suficiente.

## Que se hizo

- se elevo el estado ORB de 20 a 50 Hz en el wrapper;
- se corrigieron extrinseca, fase angular, omega causal y propagacion dinamica;
- se aislaron p/v y R/omega con baterias GT estrictamente diagnosticas;
- se corrigieron `A_HAT_AMPLIFICATION`, gravedad en O y velocidad mediante
  `MIDPOINT_DYNAMIC`;
- se hicieron causales los buffers de thrust/torque con predecesora ZOH;
- se valido hover, X, Y, Z, yaw y una ruta corta junto a fachada;
- se incorporo evidencia visual sellada del mismo frame ORB.

## Que fallo y no debe ocultarse

Las pruebas intermedias 253-349 contienen colapsos, falsos handoffs, errores de
fase, velocidad residual, `STALE_RAW_HISTORY` y fallos por degradacion visual.
No se reinterpretan como exitos. El historial largo conserva cada intento.

## Evidencia vigente

- 351: la degradacion visual precede `RECENTLY_LOST` en la fachada pobre.
- 352: ruta favorable shadow con 1510 frames `OK` consecutivos tras anchor.
- 353-355: tres rutas ORB consecutivas completadas, sin fallback posterior ni
  tracking no `OK`.

## Limite

La vuelta larga por fachadas pobres no queda prometida como ORB-only. La
planificacion por evidencia y la retirada de `GT_FALLBACK` pasan a Fase 6.
