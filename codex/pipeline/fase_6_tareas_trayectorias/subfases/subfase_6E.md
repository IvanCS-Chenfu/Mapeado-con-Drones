# Subfase 6E - Gestor y asignador global de tareas

## Estado

```text
PARCIAL: tareas base, snapshot y confirmacion estan conseguidos. La asignacion
actual por orden de creacion regional se sustituira por disponibilidad tras
anclaje y proximidad espacial al subROI.
```

## Dependencia

6B, 6C y estado/pose global publica posterior a Fase 5. Puede empezar con mapa
pobre; mejora su coste cuando 6D/6G esten disponibles.

## Objetivo tecnico

Crear en `task_lib` el nucleo autoritativo de mision y en `task_server` el
`TaskWorker`: tareas regionales, lifecycle, drones, prioridades y asignacion.

## Comportamiento esperado

Al iniciar se crean cuatro `MAP_SECTION` por nivel. La unidad asignada es una
responsabilidad regional; no contiene A-B-C ni sentido de entrada fijo.

Un dron registrado no entra aun en la asignacion por el solo hecho de publicar
una pose: debe alcanzar el estado global anclado, valido y autoritativo. Esta
regla es igual para ORB y para GT de simulacion; GT solo sustituye la fuente de
pose, no adelanta el lifecycle de anclaje.

El servidor mantiene una cola de drones anclados sin tarea principal. Para cada
dron disponible evalua exclusivamente `MAP_SECTION` en `PENDING` y escoge la de
menor distancia euclidea desde su pose anclada al volumen `AxisAlignedBox` del
subROI, no a su centro. Estar dentro de un subROI cuesta cero. Los solapes son
intencionados: si dos regiones pendientes contienen la pose, se desempata de
forma estable por `region_id`; una vez asignada una, el siguiente dron escoge
la pendiente mas cercana restante. El orden de construccion `AB -> BC -> CD ->
DA` no es una politica de asignacion.

Este coste inicial es barato y determinista mientras el mapa sea pobre. Tras
6G puede sustituirse por coste de ruta navegable, junto con cambio de altura,
carga, continuidad regional, accesibilidad y calidad de acceso. Los niveles no
forman barreras.

Estados y ownership tienen una unica autoridad. `PAUSED` conserva tarea y ramas.
Un intento fallido no completa ni cierra automaticamente una region; se evita
repetir indefinidamente el mismo intento y se permite reasignacion.

`GO_TO` puede existir como pendiente de alta prioridad, pero no preempta una
tarea `RUNNING`; su ejecucion completa pertenece a 6O.

## Cambios requeridos

1. Implementar registro por `task_id`, transiciones atomicas y validacion de dron.
2. Crear todas las tareas base antes de asignar y conservar resultados.
3. Implementar asignador determinista regional, sin puntos A/C.
4. Gestionar `REGISTERED -> ANCHORED_READY -> ASSIGNED -> RUNNING`, idle/busy,
   prioridades, fallo, pausa, reanudacion y reasignacion. Al completar una
   `MAP_SECTION`, liberar su ownership y reencolar el dron anclado para obtener
   la siguiente tarea principal; no confundir este retorno con las subtareas de
   movimiento de 6K.
5. Separar tarea regional del target interno que elegira `PlanningWorker`.
6. Definir completion de mision sin convertir FAILED en COMPLETED.
7. Exponer snapshot transient-local para GUI 7I y eventos agregados del grafo.

## Limites

No ejecutar planes, no implementar D* ni frontiers y no usar GT. No declarar
inaccesible una region por un unico intento.

## Pruebas

- Un/multiples drones y tareas; mas drones que tareas y viceversa.
- Dos drones anclados en la interseccion de subROIs solapados: el primero toma
  el empate estable y el segundo la region pendiente mas cercana, sin saltar a
  otra por orden de creacion. Repetir tras `COMPLETED` para comprobar el
  reingreso del dron a la cola de tareas principales.
- Transiciones duplicadas/reordenadas, dron no registrado/no anclado.
- Asignacion determinista entre niveles y continuidad sin FIFO rigido.
- PAUSED conserva ownership; fallo permite alternativa sin retry infinito.
- GO_TO pendiente no preempta RUNNING.
- Integracion GUI+Gazebo+grafo con lifecycle sintetico, sin movimiento fisico.

## Criterio de exito

No hay dobles asignaciones ni estados imposibles; el coste es testeable y
regional; mission/task snapshots permiten a GUI mostrar estado real.

## Ejecucion vigente

`task_server` crea las 12 `MAP_SECTION` en `PENDING` y publica
`/mission/task_states` transient-local. Un dron registrado entra en la cola
deduplicada de disponibilidad solo si su `NavigationState` es global, valido y
`AUTHORITATIVE`; en simulacion GT, la pose de control sigue siendo GT, pero la
autoridad procede del anclaje ORB en sombra. El primer dron de la cola recibe
la `MAP_SECTION` pendiente a menor distancia de su `AxisAlignedBox`; estar
dentro cuesta cero y los empates usan `region_id`. Al cerrar una tarea regional
terminal, el dron anclado vuelve a esa cola para obtener otra tarea principal.
La FIFO de 6K permanece separada y solo genera el siguiente movimiento de una
tarea `RUNNING`.
