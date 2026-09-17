# Subfase 6I - D* estricto y despacho de inspeccion

## Estado

`PARCIAL`. El despacho estricto, las reservas y el monitor ya estan migrados;
queda validar su comportamiento fisico sostenido y reducir STOPs que procedan
de cambios navegables no resueltos.

Este contrato consume el contexto de seccion y objetivo creado por 6H. Sustituye
el flujo legacy que trataba la seleccion `0.2..0.6` como objetivo de D*.

## Objetivo

Obtener una polilinea segura exclusivamente por voxeles navegables y despachar
al dron sin retener el worker. D* elige como llegar; no elige la fachada, la
seccion de coverage ni el yaw de observacion.

## Contrato de navegacion

D* usa una instantanea coherente de `VoxelMapBuilder` para el dron concreto:

- `OCCUPIED` real y `RESERVED` ajeno no son transitables;
- `UNKNOWN` no se atraviesa;
- la inflacion de ocupados se aplica como coste alto `100`, no como veto
  infinito, para tolerar huecos aislados que dejan los rayos FREE. Su margen
  adicional es `extra_obstacle_clearance_voxels=1`, sumado a la semidimension
  registrada del dron; para D1 de `0.6x0.6x0.3 m` y voxel `0.25 m` resulta
  `(3,3,2)` celdas de inflacion, mientras la reserva usa solo `(2,2,1)`;
- la burbuja de salida conserva `OCCUPIED`, `UNKNOWN` y `RESERVED` como vetos,
  pero no deja que la inflacion bloquee las celdas raw `FREE` dentro de
  `start_escape_radius_voxels=4` alrededor de la pose actual.

La meta de D* es la pose de inspeccion de 6H, nunca el voxel de fachada ni un
voxel unknown. Antes de planificar se vuelve a comprobar que la pose y su
corredor siguen siendo navegables con la revision de mapa actual.

## Resultados

- `FULL_FREE`: se genera la polilinea, se planifican waypoints y se encola
  `MOVE_AND_CAPTURE`. La orientacion permanece mirando a la fachada durante
  la trayectoria y el dron toma el depth de `vista_pared` al terminar.
- `FREE_PREFIX`: existe un primer `UNKNOWN`, o una `vista_unknown` no despejo
  la pose original y 6H encontro una pose FREE de respaldo antes del primer
  `UNKNOWN`. Se ejecuta solo hasta el ultimo waypoint `FREE` alcanzable. Al
  terminar realiza `VIEW_ADVANCE`, una captura
  depth mirando al objetivo visual original. La evidencia aporta FREE y puede
  aportar OCCUPIED directo solo si la normal de superficie es frontal y fiable.
  Si produce `OCCUPIED=1`, 6G deriva los claims espaciales y vecinos de U en la
  misma materializacion. Tras `sources_applied`, el dron vuelve a 6H para
  elegir un objetivo desde la nueva pose y la evidencia materializada.
- `UNKNOWN_FALLBACK`: es la variante de `FREE_PREFIX` que aparece tras una
  mirada sin despeje. Comparte `VIEW_ADVANCE`; no puede avanzar sin capturar
  ni se confunde con `vista_pared`.
- `BLOCKED`: `OCCUPIED` o una reserva impiden planificar. La entrada vuelve al
  final de la FIFO con el mismo contexto; no bloquea a otros drones.
- `NO_PROGRESS`: no hay tramo FREE ejecutable. Se descarta el contexto visual
  y vuelve a 6H, que selecciona otro objetivo/seccion.

No se vuelve a mirar inmediatamente el mismo unknown tras un `FREE_PREFIX`.
El avance hasta el ultimo FREE obtiene una nueva pose y evidencia depth/KF; la
proxima seleccion decide de nuevo solo cuando esa evidencia ya esta en el mapa.

## Despacho no bloqueante

`TrajectoryPlanningWorker` construye el plan y lo encola a
`ActiveTrajectoryMonitor`; nunca espera la ejecucion ni el depth. El monitor:

1. reserva el corredor barrido por el volumen fisico del dron;
2. entrega la misma polilinea al GUI y al dron una vez aceptada;
3. muestra solo la trayectoria activa y los voxeles `RESERVED`;
4. vigila cambios que intersecten el corredor y solicita STOP si aparece
   `OCCUPIED` relevante;
5. al terminal, libera la reserva, retira GUI y encola la continuacion de
   depth o de seleccion.

Una reserva no modifica el estado base `FREE`/`UNKNOWN`; es una capa temporal
que desaparece al terminar o parar. El planificador no cancela nunca una orden
dejando al dron sin referencia: todo reemplazo usa STOP local.

Si STOP reemplaza una orden que ya capturo depth, el resultado terminal puede
ser `REJECTED`; esto no invalida sus observaciones. 6F las materializa y la
continuacion vuelve a seleccion, sin reanudar el plan que STOP sustituyo.

## Exclusiones

No integra depth, no activa coverage, no cambia la orientacion por una normal
estimada y no permite que la U de coverage altere D*. La comprobacion de
coverage y `TO_FINISH` pertenecen a 6H.

## Validacion y exito

En corredor FREE, la salida es `MOVE_AND_CAPTURE` y llega una captura de pared.
Un unknown produce como maximo un prefijo FREE sin atravesarlo. Dos drones con
reservas distintas pueden planificar en paralelo; una reserva ajena reencola
el objetivo sin dejar un worker esperando.
