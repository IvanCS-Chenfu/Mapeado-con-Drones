# Subfase 6K - Replanning incremental y handover continuo

## Estado

```text
PARCIAL: cola FIFO de drones listos y runtime aislado por dron integrados;
quedan prioridades, reparacion de prefijo/sufijo y validacion dual en Gazebo.
```

## Dependencia

6D, 6G, 6I y 6J.

## Objetivo tecnico

Reparar planes en preparacion/ejecucion conservando segmentos validos y evitando
paradas salvo seguridad o ausencia de siguiente plan.

## Comportamiento esperado

Una `MAP_SECTION` `RUNNING` conserva su identidad como tarea principal. Cada
movimiento fisico es una subtarea interna de esa tarea: al inicio y tras el
terminal ordinario de una trayectoria, el dron entra en una cola FIFO de
`drones_listos`. `TaskWorker` desencola un dron, obtiene su siguiente objetivo,
solicita D*, reserva, despacha y registra la subtarea activa en
`ExecutionRuntime[drone_id]`. El vuelo de otra subtarea ya confirmada puede
continuar en paralelo; la seleccion, planificacion y commit permanecen
seriales. `ExecutionRuntime` no es un worker ni crea un thread por dron: solo
evita mezclar trayectoria, reserva, STOP y terminales de drones distintos.

La cola `drones_listos` de 6K no asigna subROIs: solo entrega la siguiente
subtarea de movimiento de una `MAP_SECTION` ya `RUNNING`. Cuando el servidor
declara `COMPLETED`, `BLOCKED` terminal o cancela esa tarea principal, el dron
sale de esta cola y, si conserva anclaje global valido/autoritativo, vuelve a
la cola de disponibilidad de 6E para recibir otra `MAP_SECTION` pendiente. No
se reencola una tarea principal tras cada trayectoria ordinaria; en ese caso
solo vuelve a `drones_listos` para continuar el mismo subROI.

Causas HARD: ocupacion raw directa, entrada de ocupacion raw en el clearance
estatico, conflicto de reserva exacta o geometria que invalida. Las reservas
ajenas no reciben clearance adicional: ya representan el volumen fisico del
otro dron. Causas SOFT: coverage, distancia de observacion, orientacion o ruta
mejor.

Ante `S0'` distinto, conservar W1... y regenerar solo hasta el primer estado
antiguo identico. Si no conecta, insertar Wa/Wb y volver a leer el estado actual
antes del enlace final. Ante obstaculo medio, D* repara el tramo y conserva
prefix/suffix cuyos corredores y estados siguen validos.

La primera entrega usa una cola FIFO simple de `drones_listos`, deduplicada por
`drone_id`: el frente es el unico que puede crear una subtarea, pero las
subtareas ya confirmadas de otros drones siguen ejecutandose en paralelo. La
unica excepcion es el propietario de un HOLD tras STOP: se reencola al frente
para reemplazar su propia reserva y no bloquear a una espera ajena. P1/P2,
anti-starvation y coalescing por motivo/revision quedan como ampliacion
posterior, no se simulan como si ya existieran.

Un cambio de mapa no interrumpe D* desde otro thread. Tras calcular se comparan
revisiones; si afecta al corredor, repair antes de reservar. Tambien es una
causa HARD que una reserva ajena invada el volumen fisico reservado. Los
cambios `UNKNOWN -> FREE`, de coste o de reserva propia siguen actualizando D*
pero no interrumpen la ruta fisica. Un STOP por mapa se solicita si cualquier
parte del corredor completo, ya recorrida o futura, pasa a estar ocupada o deja
de respetar el clearance frente a una ocupacion raw; su evento causal identifica
el voxel raw, la inflacion navegable y el segmento afectado. El reemplazo de
reserva es atomico y el handover conserva la ruta vigente hasta que la sucesora
este validada.

La ruta se crea y reserva antes de enviarse; servidor y dron samplean la misma
curva nominal mediante `lib_tray`. `TrajectoryPlan` no transporta dinamica
inicial adicional: no se despacha una sucesora hasta el terminal ordinario de
la action actual, por lo que el arranque es detenido y la guarda de start stale
resuelve discrepancias materiales. Si una ruta activa se degrada por ocupacion
o reserva ajena, se solicita STOP; este solo ordena al dron generar una
trayectoria normal hacia su pose actual. Al terminal normal de STOP, el servidor
retira toda la reserva MOVING y mantiene solo un `HOLD_RESERVATION` local del
footprint del dron en la pose donde se detuvo; despues vuelve a planificar y
entrega la siguiente ruta.

Un conflicto de reserva sin alternativa no es una causa de STOP: deja la tarea
en `WAITING`/`waiting_reservation` y coalesce un reintento por drone al recibir
release/cambio relevante. El lifecycle de reserva se refleja en la tarjeta de
GUI F7; la escena sigue mostrando exclusivamente la geometria `ACTIVE`, sin
cajas de reserva ni una polilinea candidata que sustituya a la fisica.

## Cambios requeridos

1. Implementar cola priorizada/coalesced y motivos HARD/SOFT, incluyendo
   reserva ajena relevante y espera reintentable.
2. Indexar cambios por corredores de segmento.
3. Reparar prefijo, tramo medio y sufijo reutilizable.
4. Insertar waypoints de enlace dinamicamente viables.
5. Regenerar/validar solo partes nuevas con el sampler comun de `lib_tray`.
6. Sustituir reserva y enviar `plan_revision` sin salto de referencia; la
   muestra espacial del swept volume usa el parametro
   `reservation_sweep_sample_step_voxels` (0.5 inicial).
7. Adaptar longitud/velocidad del plan a incertidumbre.
8. Medir segments reused, repair time, coalescing y handover.

## Limites

No replanificar por conteo bruto de MPs, no recalcular todo por defecto y no
parar ante cada cambio SOFT o `UNKNOWN -> FREE`. No permitir que otro thread
mute D* activo, liberar una reserva antes de confirmar su reemplazo ni tratar
una espera por reserva como fallo de vuelo.

## Entrega actual

`ExecutionRuntime[drone_id]` contiene exclusivamente la tarea, trayectoria,
reserva, objetivo y corredor de ese dron. Ya no existe estado global de una
sola ejecucion ni un parametro que fuerce D1. Al aceptar una tarea, y tras un
terminal ordinario, el dron entra en FIFO; el dispatch consume su entrada solo
si el commit de reserva tiene exito. Un release despierta antes a las tareas en
`waiting_reservation` y despues reencola al dron que acaba de terminar. STOP
transforma MOVING en HOLD y reencola a su propietario con prioridad local
cuando su action normal termina; esta excepcion evita que un WAITING ajeno
impida reemplazar el HOLD. El HOLD ya es local: se libera el corredor MOVING
completo y se retiene solo la huella fisica en la pose final de STOP. La prueba
686 tambien confirma que la cola serializa decision/commit, no ejecucion: dos
runtimes confirmados llegaron a ACTIVE en paralelo.

## Pruebas

Start mismatch, enlace imposible/directo, obstaculo medio, cambio irrelevante,
conservacion de suffix, requests coalesced, anti-starvation, handover continuo,
espera/release de reserva y fallback a hover. Validacion visual con GUI+Gazebo+
grafo: dos drones GT desde fiducial 2 deben resolver el conflicto como ruta
alternativa o espera, sin STOP si el corredor activo no se degrada.

## Criterio de exito

Cambios locales producen reparaciones locales reproducibles; no hay huecos de
reserva ni discontinuidades y las paradas quedan justificadas exclusivamente
por degradacion HARD del corredor. La espera por reserva se reintenta sin
desbloquear fisicamente al dron ni producir un bucle STOP.
