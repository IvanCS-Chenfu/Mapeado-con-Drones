# Subfase 6O - GO_TO, ANCHOR_SUBMAP y fiduciales oportunistas

## Estado

PENDIENTE, con integracion fiducial minima incluida en la migracion de fachada.

## Objetivo

Conservar los objetivos operativos de prioridad alta y permitir que una
inspeccion de fachada pause su tarea cuando aparece un fiducial nuevo para el
submapa actual.

## Fiducial oportunista acordado

- La identidad es `(drone_id,map_epoch,fiducial_id)`.
- `visto` significa observacion primaria valida interpretada por
  `orbslam3_server`, aunque una optimizacion posterior falle.
- Si el fiducial no fue visto por ese submapa, se ordena STOP, se libera la
  reserva movil y se usa el pipeline fiducial normal.
- Si ya fue visto, se ignora y continua el barrido.
- No se modifica el optimizador de Fase 3 ni se usa su exito para validar 6O.
- Mientras la interrupcion permanezca pendiente, el dron no puede recibir una
  nueva `MAP_SECTION`. Primero termina la inspeccion/action anterior, se limpia
  su runtime y su entrada `ready`, y solo despues vuelve a la cola general.
  Esto impide que el cierre de una tarea antigua retire la subtarea de la nueva.

## GO_TO y ANCHOR_SUBMAP

`GO_TO` mantiene prioridad alta entre tareas pendientes, pero no preempta una
action normal salvo peligro. `ANCHOR_SUBMAP` exige tracking valido y usa la
autoridad habitual de Fases 4-5. Ambos respetan D*, reservas, STOP y volumen de
vuelo.

## Pruebas

Fiducial nuevo/visto en el mismo epoch, reinicio de epoch, STOP sin lifecycle
huerfano y reentrada posterior en asignacion. El fallo conocido de optimizacion
fiducial de Fase 3 no bloquea esta validacion.

## Criterio de exito

Un fiducial nuevo pausa la tarea exactamente una vez por submapa y uno ya visto
no rompe el barrido.
