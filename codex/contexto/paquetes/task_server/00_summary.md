# task_server - resumen vigente

## Responsabilidad

`task_server` es el coordinador global de tareas y movimientos. Mantiene el
registro de drones, asigna `MAP_SECTION`, encola subtareas de movimiento,
solicita inspecciones al dron, planifica con D*, reserva el corredor, publica
el plan y procesa su terminal. No controla motores ni calcula depth local.

Archivo principal:
`servidor/task_server/src/task_server_node.cpp` -> clase `TaskServerNode`.

## Asignacion y lifecycle

Los drones anclados y disponibles compiten por la tarea pendiente mas cercana
a su pose. Una tarea parcial conserva sus intervalos recorridos y puede usar el
estado `TO_FINISH`; vuelve a competir por proximidad al intervalo que falta.
Al terminar o liberar una tarea, el dron retorna a la cola general.

Cada dron posee runtime de ejecucion independiente. La secuencia por subtarea
es:

```text
seleccionar destino de fachada
-> InspectFacade
-> integrar FREE depth
-> comprobar corredor o prefijo FREE
-> D*
-> reservar
-> publicar/ejecutar trayectoria
-> procesar terminal
-> actualizar coverage
-> volver a encolar
```

## Inspeccion y evidencia

El servidor llama al servicio relativo `inspect_facade` de cada
`task_manager`. Integra las capturas aceptadas exclusivamente como evidencia
depth FREE reversible, usando la identidad de captura como `source_id`.

Si el dron responde `drone_busy` porque aun ejecuta una trayectoria fisica, el
worker aplaza la inspeccion conservando la misma tarea y candidato. Este estado
no incrementa el contador de capturas fallidas ni provoca `TO_FINISH`.

Una interrupcion fiducial pendiente bloquea temporalmente nuevas asignaciones
para ese dron. El cierre de la tarea antigua limpia primero su runtime y solo
entonces reencola al dron, evitando mezclar entradas `ready` de dos tareas.

El recorrido de rayos usa voxelizacion supercover. Solo se rellena un hueco de
una celda cuando hay evidencia depth coherente a ambos lados en un eje. Depth
no crea OCCUPIED ni puede borrar ocupacion sparse. Los MapPoints ORB
cualificados conservan la autoridad OCCUPIED y el paso fisico del dron marca su
volumen como FREE atravesado.

## Planificacion de fachada

`FacadeTaskRuntime` conserva orientacion, lado de barrido, fallos de inspeccion
e intervalos recorridos. El candidato usa las preferencias configurables de
distancia a pared, desplazamiento y altura media. El corredor completo debe
ser FREE; si no, se permite el mayor prefijo conectado FREE que alcance el
minimo configurado.

D* se ejecuta con `require_known_free=true`. Las reservas de otro dron se
tratan como celdas bloqueadas sin convertirlas en OCCUPIED persistente. La
polilinea, sus waypoints y la reserva pertenecen al mismo plan.

Al terminar una trayectoria, el intervalo solo aumenta si la orientacion real
esta dentro de `facade_orientation_tolerance_deg`. Con ratio `>=0.99` la tarea
termina. Si llega a una cara lateral sin completar, pasa a `TO_FINISH`. Tres
inspecciones fallidas tambien producen `TO_FINISH` y liberan al dron.

La seleccion volumetrica de metas UNKNOWN, el analizador periodico de coverage
y los portales de rama ya no forman parte del runtime.

## STOP, reservas y replanning

Un cambio relevante a OCCUPIED o RESERVED en el corredor activo puede pedir el
STOP local ya existente. Tras el terminal, se libera la reserva anterior y se
replanifica desde la pose actual. Cambios FREE fuera del corredor no cancelan
el movimiento.

La GUI recibe solo la trayectoria actualmente ejecutada; una ruta preparada no
sustituye visualmente a la activa antes de su despacho.

## Interrupcion fiducial

El servidor escucha `/mission/fiducial_primary_observations`. Deduplica por
`(drone_id, map_epoch, object_id)`. Una primary nueva deja la tarea de fachada
en `TO_FINISH`; si el dron vuela solicita STOP y, si esta inspeccionando, espera
su terminal. Despues libera runtime, reserva y dron para que el pipeline
fiducial normal continue.

Esta frontera no modifica anchors, epochs, backend ni optimizacion de Fase 3.

## Topics y servicios principales

- clientes `/<drone_id>/task_manager/inspect_facade`;
- action clients de trayectoria por dron;
- `/mission/fiducial_primary_observations`;
- snapshots/deltas voxel y estado de tareas para GUI;
- planes y terminales de ejecucion;
- eventos de arquitectura del grafo web.

## Pruebas

Validacion vigente: build correcto y CTest `7/7`. Los contratos web dirigidos
verifican las conexiones `InspectFacade`, `CaptureDepth` y primary fiducial.
Quedan pendientes las dos pruebas integradas Gazebo+GUI F7 del barrido y del
tracking risk acordadas para el cierre de la migracion.
