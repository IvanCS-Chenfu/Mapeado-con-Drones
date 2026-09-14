# Subfase 6J - Reservas espaciales y colisiones multi-dron

## Estado

```text
PARCIAL: overlay, commit/replace/release por propietario y espera por conflicto
integrados; falta muestrear la curva real compartida de `lib_tray` y validar la
prueba dual completa.
```

## Dependencia

6C, 6D, 6G y 6I.

## Objetivo tecnico

Garantizar que cada trayectoria ejecutada tenga una reserva global coherente
con su volumen fisico y que dos commits no compitan.

## Modelo cerrado

El baseline es espacial, no espacio-temporal. `RESERVED` no modifica la
evidencia raw ni convierte una celda en `OCCUPIED`: es una capa dinamica
derivada con `reservation_id`, `drone_id` propietario, revision y tipo
`MOVING` o `HOLD`. No se dibuja como voxel ocupado en GUI F7. Para el dron
propietario sus propias celdas no bloquean D*; para cualquier otro son
equivalentes a `OCCUPIED` **sin** sumar clearance entre drones. Cada reserva
contiene ya el volumen fisico voxelizado del dron durante toda la trayectoria,
incluido el footprint, pero no un margen adicional. El clearance se aplica
solo frente a evidencia raw `OCCUPIED` del mapa. Asi D* y el mecanismo de
seguridad reaccionan a una reserva ajena sin convertirla en evidencia raw, y
el mapa fisico y sus cambios reversibles siguen teniendo una sola autoridad:
`VoxelMapWorker`.

`ReservationWorker` procesa una solicitud por vez. La cola es FIFO; dos
solicitudes con la misma frontera se desempatan por `drone_id`. Una reserva ya
committed gana: el candidato vuelve a planificar contra la capa dinamica. Si no
hay alternativa segura, la tarea queda `WAITING` con detalle
`waiting_reservation`, sin enviar STOP ni liberar el dron; se reintenta cuando
se libera/actualiza una reserva o llega un cambio navegable del mapa.

La secuencia objetivo de una ruta normal es:

```text
objetivo -> D* sobre mapa + reservas ajenas -> polilinea/waypoints
         -> trayectoria comun de lib_tray -> swept volume
         -> commit atomico RESERVED -> ACTIVE a dron y GUI F7
         -> terminal normal -> release -> siguiente objetivo
```

No se anade pose, velocidad ni aceleracion inicial a `TrajectoryPlan`: el
siguiente despacho espera el terminal de la action, de modo que ambos parten
de una pose detenida casi identica; la guarda existente de inicio stale rechaza
una desviacion relevante. La entrega actual reserva la polilinea D* muestreada
cada `reservation_sweep_sample_step_voxels=0.5` voxeles iniciales y la ocupa
con el footprint fisico, sin clearance inter-dron. D* y `Commit` deben usar la
misma funcion de swept volume; un commit conflictivo solo puede ser una carrera
con una reserva nueva y desencadena un replan inmediato antes de declarar
`WAITING`. Es una aproximacion conservadora por segmentos, no el sampler de
curva de `lib_tray`: ese sampler comun sigue siendo obligatorio antes de
declarar el swept volume final conseguido.

Cada segmento conserva corredor voxel, revisiones de mapa/reserva y resultado.
Un cambio posterior solo revalida los segmentos cuyo corredor o reserva se vea
afectado. Para sustituir un plan del mismo dron, la reserva vieja permanece
mientras se valida la nueva y el commit la reemplaza atomica y totalmente.
Tras STOP se elimina por completo la reserva MOVING, tanto prefijo como futuro,
y se crea un `HOLD_RESERVATION` con solo el footprint fisico en la pose
canonico-confirmada donde termino el dron. Ese HOLD persiste hasta que un
replace atomico lo sustituya por una nueva reserva segura. Una perdida de
comunicacion tampoco libera por timeout: conserva la presencia fisica hasta
evidencia canonica de que ya no aplica.

Un STOP se conserva si una celda raw pasa a `OCCUPIED` y alcanza cualquier
parte del corredor completo de la trayectoria, ya recorrida o futura, o entra
en su clearance estatico frente a obstaculos. No se recorta el prefijo como
optimizacion funcional. Cada STOP debe publicar telemetria causal: voxel raw y
estado anterior/posterior, celda navegable afectada, causa directa o por
inflacion de obstaculo, y distancia al segmento de la ruta.

## Cambios requeridos

1. Implementar cola/worker serial y registro autoritativo de reservas con owner.
2. Exponer una capa dinamica por perfil para que D* ignore la reserva propia y
   bloquee las ajenas sin tocar `OCCUPIED` raw.
3. Construir swept volume desde el sampler comun de `lib_tray`, con
   `reservation_sweep_sample_step_voxels` configurable y footprint fisico
   registrado, sin margen entre drones.
4. Detectar conflicto con drones en movimiento, HOLD y hard flight volume.
5. Cachear validacion por segmento/corredor/revisiones y revalidar solo cambios
   relevantes antes del commit, sin bloquear `VoxelMapWorker`.
6. Implementar atomic replace y lifecycle complete/cancel/fail/replace; ante
   conflicto de commit, replanificar antes y publicar espera solo si ya no hay
   alternativa, nunca una parada repetitiva.
7. Mantener dron incomunicado como presencia fisica; timeout no libera sin evidencia.
8. Publicar eventos/metricas agregadas de queue, validation, conflict, wait y commit.

## Limites

No introducir time-space reservations ni paralelismo antes de medir. No reservar
solo waypoints ni hacer release-then-propose. No comprobar paredes mediante
geometria ad hoc fuera del voxel map, ni duplicar la curva de `lib_tray` en el
servidor. No publicar cajas 3D de reserva en GUI F7 ni cambiar el enum raw de
voxeles para representarlas.

La autoridad de reservas dinamicas sigue siendo `ReservationWorker`. Sus deltas
de bloqueo espacial llegan a la capa navegable que sirve `VoxelMapWorker` al
planner, con filtro por propietario. No se activa esa capa dinamica durante la
optimizacion inicial de 6D/6G ni se mezcla con las revisiones de evidencia raw.

## Entrega actual

`ReservationOverlay` vive en `task_lib`, conserva `MOVING`/`HOLD`, hace
`Commit`/`Replace`/`Release` atomicos y proyecta solo reservas ajenas en el
snapshot de navegacion. Una reserva MOVING se voxeliza con la huella fisica del
propietario; el planificador solicitante expande esa reserva solo por su propia
huella para impedir interseccion de cuerpos, sin introducir clearance extra
entre drones. Ante conflicto de commit se intenta una alternativa con el
overlay actual antes de `WAITING`. Tras STOP se reemplaza toda MOVING por HOLD
local en la pose canonica. Las reservas no se publican como voxeles raw ni como
cajas GUI. La prueba 686 valido dos ejecuciones ACTIVE simultaneas, replan
previo a espera y causas STOP deduplicadas; queda pendiente muestrear la curva
ejecutable comun de `lib_tray` en vez de la polilinea D*.

## Pruebas

Cruce de dos planes, solicitudes simultaneas, tamaños distintos, yaw, HOLD
local, atomic replace, mapa cambiado dentro/fuera del corredor, conflicto sin
ruta y watchdog. La integrada usa dos drones GT que alcanzan fiducial 2 y
comprueba que D2 planifica una alternativa contra el volumen exacto reservado
por D1 y que ambos vuelos compatibles permanecen activos en paralelo. Tambien
comprueba STOP por ocupado directo y por clearance estatico, con su causa
telemetrica, y que el HOLD no retiene ruta futura. Medir
queue/sampling/check/commit/retry y comprobar que GUI solo muestra lifecycle de
reserva en tarjetas y la ruta `ACTIVE`.

## Criterio de exito

No existen carreras, huecos de reserva ni trayectorias sin volumen validado; un
conflicto conserva la reserva previa y produce alternativa o espera segura. Un
STOP conserva HOLD, un terminal normal libera su reserva y un cambio
`UNKNOWN -> FREE` irrelevante no genera STOP ni replanning fisico.

## Proyeccion visual de reservas

`RESERVED` no es un cuarto estado de evidencia raw ni entra en el calculo de
coverage. `task_server` publica, junto al snapshot raw de `/mission/voxel_map`,
una coleccion efimera de las celdas presentes en `ReservationOverlay`. Cada
celda reservada puede cubrir visualmente una celda `FREE` o `UNKNOWN`, pero no
reescribe su estado almacenado en `ReversibleVoxelMap`. Al completar, liberar o
reemplazar una reserva, la siguiente publicacion elimina esas celdas de la
capa dinamica y la GUI vuelve a mostrar el estado raw subyacente. Un STOP deja
visible solamente el `HOLD` local mientras siga vigente. `OCCUPIED` raw tiene
prioridad visual sobre una reserva superpuesta.

La GUI F7 ofrece un toggle `Reservados`, independiente de `Ocupados` y
`Libres`. Las reservas no se presentan como `OCCUPIED`, no alteran el mapa ni
la telemetria de cobertura y no se dibujan cajas de seguridad adicionales.

## Limitaciones observadas y diferidas

En el arranque, la decision y el commit de subtareas se serializan. Si los dos
drones comienzan cerca del mismo fiducial y sus primeras rutas se solapan, uno
puede quedar temporalmente en `WAITING` hasta que exista una alternativa o se
libere la reserva inicial; una vez sus corredores son compatibles, sus actions
pueden y deben ejecutarse en paralelo. Esta contencion inicial se conserva por
ahora como comportamiento seguro. Una futura optimizacion podra preparar mas
de una alternativa antes del commit sin relajar la atomicidad.

La politica de STOP sigue siendo deliberadamente conservadora: cualquier
`OCCUPIED` raw nuevo o su clearance estatico que alcance el corredor completo
activa STOP, aunque algunas paradas aisladas puedan resultar evitables tras
analizar la evolucion fina del mapa. Se mantiene asi hasta una prueba futura
que mida causalidad, distancia al corredor y coste de falsos positivos sin
degradar la seguridad.
