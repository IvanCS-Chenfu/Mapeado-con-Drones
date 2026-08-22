# Anexo 3H-3K - Cola y worker secundarios

## Estado y propiedad

```text
Preparacion: CERRADA
Autorizacion funcional: CONCEDIDA
Ejecucion: IMPLEMENTADA Y VALIDADA TECNICAMENTE; CIERRE VISUAL PENDIENTE
Propietaria de creacion: 3H
Extension transaccional: 3K
```

Este anexo define el contrato transversal de `SecondaryTaskQueue` y
`SecondaryWorker`. Sustituye la interpretacion antigua que retrasaba su creacion
hasta 3K.

## Arquitectura

Existe una sola cola secundaria global y un solo worker persistente para todo el
servidor:

```text
producers -> SecondaryTaskQueue -> SecondaryWorker -> complete task flow
```

En 3H-3L solo `FiducialOptimizationTask` tiene implementacion funcional. Las
interfaces de los otros tipos se reservan para que las fases posteriores no
tengan que cambiar la semantica del scheduler.

| Prioridad | Tarea | Flujo indivisible |
|---|---|---|
| `MAX` | `FiducialOptimizationTask` | revalidate -> graph -> solve -> validate -> commit/refine |
| `HIGH` | `DatabaseUpdateTask` | se implementara en su fase |
| `NORMAL` | `LoopTask` | se implementara en su fase |

## Orden

- se selecciona el carril de mayor prioridad no vacio;
- dentro de cada carril se conserva FIFO por `enqueue_sequence`;
- la prioridad ordena tareas pendientes, nunca interrumpe la tarea activa;
- una futura `LoopTask` activa termina todas sus subtareas antes de una MAX;
- tras terminar la activa, todas las MAX pendientes se ejecutan en FIFO antes
  de HIGH/NORMAL;
- graph, solve, validation y commit no son nuevas entradas de cola.

La ausencia de preemption evita observar medio flujo o dejar locks/estado
intermedio. Una tarea costosa debe calcular sobre copias privadas, no resolverse
manteniendo bloqueado el pipeline.

## Identidad y deduplicacion

Cada tarea tiene `task_id` unico y un `logical_key` especifico de su tipo. Para
fiduciales:

```text
logical_key = (submap_id, keyframe_id, fiducial_id)
```

Una repeticion exacta del mismo KF puede unirse o rechazarse como duplicada. Dos
KFs diferentes, aunque pertenezcan a la misma visita o fiducial, son trabajos
distintos y se conservan.

No se usa una politica “ya hay optimizacion de este submapa” para eliminar
tareas. La revalidacion al dequeue es el mecanismo correcto para descubrir que
una optimizacion anterior ya corrigio el error.

## Ciclo de vida

Estados minimos observables:

```text
PENDING
DEQUEUED
REVALIDATING
STALE
BUILDING_GRAPH
SOLVING
VALIDATING
COMMITTING
REFINING
SUCCEEDED
HARD_FAILED
CANCELLED_SHUTDOWN
```

Una tarea fiducial pasa a `STALE` si, al desencolarse, su KF ya esta dentro de
los tres umbrales respecto del objetivo guardado. Ese final es correcto y no
crea grafo.

Un commit parcial valido usa `REFINING` y otra pasada interna. Conserva
`task_id`, ownership y flag de optimizacion activa. No vuelve a `PENDING` ni
cede el worker.

## Concurrencia

- los productores mantienen el mutex de cola solo para enqueue/dedup breve;
- el worker extrae una tarea y libera el mutex antes de ejecutar;
- las capturas de bases son breves y versionadas;
- grafo, solver y validacion operan fuera de locks live;
- el commit usa una seccion critica breve y atomica;
- el flujo principal puede ingerir, construir y publicar durante el calculo;
- no hay un segundo worker secundario.

El shutdown deja de aceptar tareas, permite cancelar de forma estructurada el
trabajo privado y hace `join` del worker. No deja commits parciales visibles.

## Capacidad y backpressure

La cola no debe descartar silenciosamente trabajo unico. Puede usar una
capacidad fisica grande y monitorizada, pero la proteccion operativa es:

```text
secondary_high = 64 pending tasks
secondary_low  = 16 pending tasks
```

El mission gate se activa si la cola alcanza high o si hay una optimizacion
fiducial/loop activa. Se libera solo sin optimizacion activa y cuando las colas
principal y secundaria estan por debajo de sus low watermarks.

El goal actual no se cancela. No se envia el siguiente goal mientras el flag
este activo. Los snapshots/deltas ya en vuelo se procesan y no se pierde el
estado necesario para propagar KFs posteriores.

La implementación no puede conservar el comportamiento antiguo que dejaba
backpressure permanentemente a false o descartaba trabajo
al llenar una cola pequeña.

## Resultados y fallos

Cada tarea produce un resultado estructurado con:

```text
task_id, type, priority, state
enqueue/start/end timestamps
initial/final fiducial errors when applicable
passes, revisions, moved_kf_count
failure_reason
```

Los fallos duros incluyen NaN/Inf, fallo numerico, hard movido, revision
incompatible y batch incoherente. No hacen commit y mantienen la parada de
drones en estado bloqueante hasta tratamiento explicito; no se ocultan como
`STALE` o exito.

## Integracion visual

El grafo web debe distinguir:

- `enqueue` y prioridad;
- `dequeue/start`;
- estado de la tarea activa;
- `STALE` por revalidacion;
- commit parcial y nueva pasada;
- commit completo;
- fallo duro;
- transiciones del flag de parada.

La telemetria es acotada y no bloqueante. Perder un evento visual no altera la
cola ni el resultado funcional.

## Pruebas

- prioridad MAX>HIGH>NORMAL con FIFO interno;
- tarea activa no interrumpible;
- varias MAX pendientes se ejecutan en su orden;
- dedup exacto y preservacion de KFs distintos;
- revalidacion convierte trabajo obsoleto en `STALE`;
- una optimizacion previa que mueve otro submapa puede volver `STALE` una tarea
  futura cuando se active covisibilidad;
- el flujo principal avanza con worker ocupado;
- high/low watermarks activan y liberan mision con histeresis;
- capacidad monitorizada sin drop silencioso;
- commit parcial conserva worker y `task_id`;
- shutdown limpio en pending, solving y antes de commit;
- telemetria refleja el orden real.

## Fuera de alcance 3H-3L

- preemption de tareas activas;
- varios workers secundarios;
- `DatabaseUpdateTask` y `LoopTask` funcionales;
- fairness que permita saltarse MAX pendientes;
- publicacion ROS o espera de `GlobalMapBuilder`/RViz2/web.
