# Resumen 6N

Estado agregado: PARCIAL.

## Rendimiento F6N pendiente

La prueba GT de dos drones del 2026-09-21 descubrio que el productor
`KeyframeSparseEvidenceDelta` seguia ejecutandose con Fase 6 apagada y emitio
200 deltas F6N dentro de la ruta sincrona anterior a publicar KFs. Se corrige
solo la puerta de despliegue: `phase6_enabled=false` impide crear y calcular
F6N. La prueba integrada `c5_5_3_two_drones_gates_off_v2` valida el gate con
dos drones GT: cero deltas F6N, escenario completo y cola primaria maxima 1
frente a 35 en la ejecucion anterior. Quedan documentadas para las pruebas
especificas de Fase 6, sin cambio interno ahora, la invalidacion por revisiones raw meramente estadisticas, la
expansion repetida de KFs observadores, las consultas por MP y la construccion
sincrona de evidencia equivalente. No se consideraran cerradas sin una prueba
autonoma con F6N habilitado y temporizacion propia.

La prueba 769 valida la correccion de prioridad depth con dos drones. D1 y D2
llegaron por GT al fiducial 2 en posiciones verticalmente separadas y con yaw
`+90 deg`; el coverage empezo solo entonces. Una pareja de D2
`2:0:23/2:0:35` espero detras de una pareja activa de D1, pero al terminar esta
la cola pidio explicitamente las dos poses de D2, las integro y completo la
subtarea `DA`. Por tanto, D2 ya no queda detenido por una fuente global antigua
de D1. Las parejas creadas cerca del cierre de la ventana quedaron pendientes
por fin de escenario; los reintentos repetidos de pose son una mejora de
rendimiento pendiente, no una mezcla de identidades.

La prueba 758 valida por primera vez el ciclo de avance incremental acordado:
D1/GT integra las dos observaciones depth, D* acepta `UNKNOWN` con coste mayor,
avanza hasta el primer hueco, termina la trayectoria correctamente, inspecciona
ese hueco y vuelve al selector. No reaparece el antiguo rechazo por una recta
no completamente FREE. Tras ese retorno surge un bucle nuevo de solicitudes de
inspeccion cada unos 250 ms sin consolidar la siguiente accion. El avance
incremental esta conseguido; el lifecycle posterior aun requiere diagnostico.

La prueba 759 aplica el umbral visual `0.60`. El detector se activa y solicita
STOP antes de que ORB pase a LOST, pero lo hace mientras `InspectFacade` ya
ejecutaba una orientacion extrema de `108 deg` yaw y `46 deg` pitch. No se ve
STOP completado ni la retencion `F6I-STOP-HOVER` antes de la perdida ocurrida
unos 2.2 s despues. La prioridad ahora es hacer atomico ese relevo local y no
atribuir el fallo a que la fachada carezca de puntos.

La prueba 751 compila y ejecuta la nueva maquina de estados de integracion, con
un unico D1/GT, GUI F7 y Gazebo limpios. No valido aun la transicion completa:
el candidato `(-0.5,-14,1.2)` quedaba aproximadamente detras de la pose del
dron orientada a yaw 90 grados, por lo que `TargetOrientation` ordeno una
mirada cercana a 180 grados. La normal depth no la origino: fue rechazada y
conservo yaw 90. ORB paso de tracking `2` a `3` antes de responder la
inspeccion; no hubo marcador `F6L` porque el protocolo actual requiere tres
frames consecutivos con cero inliers en el semiplano esperado. El cambio de
epoch posterior invalido la inspeccion. La evidencia confirma que hace falta
debatir la politica de gran giro hacia objetivo y la condicion preventiva de
riesgo, sin atribuirlo a la espera de integracion depth.

La migracion al barrido lateral de fachada esta implementada y compila. Los
tests dirigidos vigentes son correctos. La prueba 737 valido que una inspeccion
automatica ya no sustituye una trayectoria fisica externa activa. Tambien se
corrigieron la barrera de asignacion durante una interrupcion fiducial y la
propagacion del motivo real de fallo de `InspectFacade`.

La prueba 738, interrumpida por el usuario, descubrio el bloqueo por frente de
la cola global y la perdida prematura del frame exacto. Ambas correcciones ya
compilan y sus tests dirigidos pasan; la prueba 739 consiguio aislar D1.

La 739 no alcanzo D*, reservas, movimiento de fachada ni coverage. Una
inspeccion obtuvo depth en ambas orientaciones, pero no genero el prefijo FREE
minimo; al perder ORB, el servidor entro en reasignaciones inmediatas de tareas
`TO_FINISH` con capturas vacias. Tampoco hubo `TRACKING_RISK`: ORB paso a
`RECENTLY_LOST` durante la restauracion angular, actualmente fuera de la
ventana vigilada por `inspection_target_look_active_`.

El criterio de validacion queda corregido: se mantiene el barrido lateral
completo, pero la inspeccion inicial no puede producir giros yaw erraticos ni
perder ORB antes de D*. Antes de continuar hay que estabilizar orientacion y
direccion de barrido, vigilar el riesgo durante toda la maniobra y evitar el
bucle de reasignacion sin nueva evidencia visual.

Las pruebas 741--743 acotan el fallo angular. El arco largo de 742 se debia a
normalizar `+181 grados` como `-179 grados` y ya esta corregido en `gen_tray`.
En 743 el arco corto se respeta, pero tras el primer STOP `InspectFacade` lanza
una restauracion, esa restauracion activa un segundo STOP y despues se ejecuta
una correccion local de 25 grados. El giro posterior al STOP es por tanto una
orden nueva del protocolo, no la trayectoria cancelada que continue. Sigue
pendiente simplificar y validar esta recuperacion antes del barrido D*.

La prueba 744 valida que `5 deg/s` evita la perdida, pero descubre la causa del
bucle de inspeccion: las segundas capturas contienen 1/0/1 puntos y confianza
inferior a 0.25. El dron puede responder `success=true`, mientras el servidor
las rechaza despues de haber encolado ya la primera captura. Asi se reintegra
FREE de fachada, no FREE hacia el destino, y se consumen tres intentos. Queda
pendiente hacer atomica la recepcion y unificar el criterio de validez depth.

La implementacion posterior hace atomica la pareja y añade fallback newest-first
sobre tres frames con al menos 20 inliers. 745 no avanza: una razon de confianza
1.0 sobre un unico punto demuestra que falta soporte absoluto, y la perdida ORB
invalida el epoch de las capturas.

746 adelanta correctamente `TRACKING_RISK` al 65 % y conserva tracking, pero
localiza otro bloqueo: los tres candidatos ofrecen confianza
0.049/0.121/0.167 y despues el buffer reutiliza 485 veces el frame 1367 con
cuatro puntos y confianza 0.059. El recibo solo transporta imagen/camara valida
al crear un KF, por lo que `StoreStereoFrame` descarta los frames intermedios;
el servidor libera y reasigna la
misma tarea cada tres fallos, reintentando cada unos 250 ms sin imagen nueva.
No hubo integracion FREE ni D*. Estado agregado: PARCIAL; atomicidad y fallback
mecanico estan implementados, pero frescura/soporte depth y backoff siguen
pendientes de acuerdo. La ruta propuesta es guardar desde el wrapper el par
rectificado de cada frame cualificado, sin modificar ORB-SLAM3.
