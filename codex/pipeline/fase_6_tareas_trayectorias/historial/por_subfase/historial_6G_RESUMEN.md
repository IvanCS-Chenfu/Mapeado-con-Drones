# Resumen - 6G

Estado: CONSEGUIDA. D* Lite 3D incremental genera rutas XYZ previstas desde
`task_server`; la prueba 615 valido plan y replan con GUI F7. 6I conserva la
responsabilidad de convertir esa polilinea en trayectoria ejecutable. La 616
solo valido infraestructura porque no pudo enviar las solicitudes ROS.

Reapertura 617/618: sin limite de distancia, los MapPoints produjeron voxeles
reales, pero el inicio estaba ocupado o inflado. La salida segura necesita la
evidencia `FREE` de 6I o una nueva regla funcional acordada.

Reapertura 619/620: se implemento esa evidencia `FREE` real y el escape local
solo por `FREE`, sin relajar el margen. La 620 confirma la reintegracion por
KF, pero D1 rechazo el objetivo exacto por `goal_occupied_or_inflated` y D2 no
respondio dentro de la ventana de prueba. El comportamiento de rechazo es
correcto; falta una demostracion visual de desvio D* con un objetivo seguro y
una respuesta acotada en el mapa denso.

La 621 intento un candidato cercano `(-8.5,-4,1.0)`, pero el cliente no obtuvo
respuesta y el log no contiene entrada `PlanRoute`. Queda pendiente repetir con
comprobacion de descubrimiento ROS, sin concluir aun que D* sea la causa.

La 622 verifico el servicio y el envio de la peticion, pero tampoco obtuvo una
respuesta en 35 s ni un evento de resultado del servidor. Queda pendiente
acotar el presupuesto del planner antes de explorar mas candidatos cercanos.

La 623 anadio telemetria de entrada y confirmo que la callback llega a D*; no
termina en unos 20 s con `map_revision=263`. El acuerdo posterior no introdujo
un presupuesto temporal: se optimizo primero la capa navegable de
`VoxelMapWorker`, manteniendo iguales distancia y seguridad.

La 624 identifico el cuello previo a D*: 24 cambios raw/1.056 celdas tardaron
16.060 ms. La 625 valida el recálculo local en dos pasadas y el consumo por
snapshot/delta: 24 cambios/1.122 celdas tardaron 110.169 ms; el lote mayor
observado, 988 cambios/34.703 celdas, tardo 2.633 s. La llamada posterior a
`(-8.5,-4,1.0)` respondio al cliente en 2.35 s y el servidor la rechazo en la
entrada con `goal_occupied_or_inflated` y cero expansiones. Es una respuesta
correcta y acotada, pero no una demostracion de desvio: sigue pendiente elegir
un objetivo con clearance para observar una ruta en GUI F7.

La 626 encontro finalmente una ruta segura a `(-2,-8,4)`, con cinco waypoints,
50.120 expansiones y 5.294 s, pero se publico durante el cierre y no pudo
revisarse en GUI. La 627 priorizo el destino pedido `(-10,5,2)`: no quedo
ocupado/inflado, pero agoto el limite existente de 250.001 expansiones en
28.238 s. No se modifico el limite sin acuerdo; sigue pendiente decidir si se
amplia solo para esa prueba remota o se usa un objetivo con coste asumible.

Optimizacion acordada e implementada, pendiente de la prueba 628: el planner
cuenta pops de cola, descarta los obsoletos por generacion antes de restar
presupuesto y usa heuristica ponderada `epsilon=1.2`. Sigue una guia de
macro-voxeles por perfil (`4x4x4` finos) dentro de un corredor de 8 celdas que
se amplia de 4 en 4 y finalmente puede usar la ventana completa. La guia no
puede aceptar ni rechazar una ruta; la malla fina conserva la validacion de
OCCUPIED, inflacion y diagonales. GUI F7 muestra solo la ruta final y el grafo
web conserva eventos esenciales; las metricas detalladas quedan en log.

La 628 fallo antes de ejecutar el escenario por una ruta YAML relativa. La 629
repitio limpiamente con ruta absoluta, GT, Gazebo y GUI F7: D1 acepto el plan a
`(-10,5,2)` con 10 waypoints, 8.569 expansiones reales y 11.812 s. El corredor
de 8 celdas no requirio ampliacion; se registraron 90.011 pops, 81.442
obsoletos, que no consumieron el presupuesto. Gazebo y GUI permanecieron 64 s
desde el inicio de D*, sin residuos tras cerrar. La demostracion visual queda
CONSEGUIDA. La siguiente mejora opcional es una cola indexada/decrease-key para
reducir pops obsoletos; se implemento y valido en la entrada posterior.

La cola indexada ya se implemento y valido: build/CTest 7/7 de `task_lib` y
`task_server`; la 631 dio `stale_queue_pops=0` y una ruta GUI de seis waypoints
hacia `(-10,5,2)`, manteniendo la simulacion mas de 50 s. Su 17.237 s no se
compara directamente con 629 (11.812 s), porque 631 tenia mas expansiones
reales (11.480 frente a 8.569) y un corredor distinto (88 frente a 71). La
eliminacion de duplicados queda CONSEGUIDA; una comparativa temporal estricta
requiere mapa/replay determinista.

Estado de migracion nueva: PARCIAL. El bloque 3 introduce `VoxelMapBuilder`,
que consume transacciones de `EvidenceDatabase` y retira/reproyecta solo las
fuentes sparse del KF modificado o eliminado. La evidencia depth y ocupacion
sparse legacy siguen transitoriamente fuera de esta capa hasta 6F. Build
correcto; CTest dirigido correcto (9/9).

Actualizacion 786: `VoxelMapBuilder` informa fuentes OCCUPIED materializadas y
retiradas; `task_server` deriva claims U por `source_id` y KF, incluidos dos
vecinos continuos y las rebanadas espaciales de cada endpoint. D1/GT/depth
materializo dos fuentes `VIEW_ADVANCE` con 14 y 13 claims y 18 secciones
activas. Falta una prueba Gazebo que mueva o elimine una de esas fuentes para
observar su desactivacion, por lo que 6G global sigue PARCIAL.
