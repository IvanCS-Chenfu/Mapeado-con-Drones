# Subfase 6B - Registro y asignacion de subROI

## Estado

`PARCIAL`.

La disponibilidad ya entra por `TASK_ASSIGNMENT` y la asignacion por
proximidad crea una continuacion `POINT_SELECTION`. El selector real aun no
consume esa cola, por lo que el dispatcher legacy permanece transitoriamente
activo.

## Objetivo

Asignar trabajo regional sin retener el worker de asignacion. Un dron anclado,
registrado y sin orden normal activa entra en `TaskAssignmentQueue`; el worker
elige una tarea pendiente cercana, la asigna y encola el dron en
`PointSelectionQueue`.

## Contrato funcional

- La distancia de asignacion se calcula desde la pose navegable actual hasta
  el subROI o intervalo pendiente, no desde un orden fijo de tarjetas.
- Una tarea parcial conserva sus intervalos cubiertos y estado `TO_FINISH`.
- `COMPLETED` solo se usa cuando alcanza el coverage acordado.
- Al completar o liberar una tarea, el dron vuelve a la cola de asignacion.
- Llegar a una cara compartida con otro subROI puede dejar el actual en
  `TO_FINISH`; nunca se marca completo solo por proximidad.

## Cambios requeridos

- Migrar registro, disponibilidad y ownership a trabajos identificados.
- El worker termina despues de asignar o de no encontrar tarea compatible.
- Conservar una sola tarea regional asignada por dron.
- Publicar snapshots de tarea para GUI sin usar esos snapshots como cola.

## Exclusiones

- No se ejecuta D*, depth ni reserva durante la asignacion.
- No se decide aqui la geometria detallada de fachada.

## Validacion y exito

Dos drones anclados en una interseccion reciben tareas por proximidad. Un dron
que termina y otro que deja `TO_FINISH` vuelven a competir sin duplicar
ownership ni bloquear el worker.
