# Historial - 6I

## 2026-09-16 - Diagnostico de rechazo D* estricto, prueba 773

- Se anadio `debug_facade_dstar_failure=false` a `task_server` y al launch de
  simulacion. Al habilitarlo, el rechazo FREE estricto registra los voxeles de
  inicio/meta, sus estados raw y navegables, y una frontera acotada de la
  componente FREE. El diagnostico no cambia costes, mapa, colas ni despacho.
- `task_server` y `simulacion_dron` compilaron correctamente; CTest de
  `task_server` paso 9/9.
- La prueba 773 ejecuto solo D1, con GT al fiducial 2 y depth activo. El primer
  arranque de Gazebo se reintento automaticamente; el segundo termino con
  `SIM-DONE success=true` y `SIM-EXIT-CODE 0`.
- En las revisiones 78, 89 y 103, el inicio era raw `FREE` pero
  `occupied_inflated`; la meta `8:-40:4` seguia raw/navegablemente `UNKNOWN`.
  El fallo fue `goal_occupied_or_inflated` y `used_safe_escape=false`: D* no
  llego a evaluar salida segura porque rechazo antes una meta que no era FREE.
  La frontera FREE quedo vacia porque el inicio navegable estaba inflado, no
  porque se demostrase que no hubiera FREE general alrededor.

Conclusion: PARCIAL. La instrumentacion confirma dos problemas de flujo: la
continuacion tras `vista_unknown` planifica sin revalidar que la meta se haya
vuelto FREE, y la futura salida desde una celda inflada debe conservar la regla
de no ejecutar UNKNOWN. No se ha modificado todavia la politica de movimiento.

Acuerdo posterior: se mantiene el clearance, pero el inicio puede recorrer raw
FREE dentro de una burbuja de `start_escape_radius_voxels=4`; OCCUPIED real,
UNKNOWN y RESERVED permanecen vetados. Si la meta inspeccionada sigue UNKNOWN,
6H buscara antes un fallback FREE de menor coste dentro de
`unknown_fallback_free_radius_voxels=8`; sin alternativa, se descarta el
candidato. Implementacion y prueba pendientes.

## 2026-09-16 - Migracion FIFO de D* y prueba 772

- `TrajectoryPlanningWorker` consume la continuacion de depth y ejecuta D*
  con perfil FREE. Emitio varios prefijos FREE mediante el runtime autonomo,
  sin cruzar UNKNOWN ni bloquear la FIFO.
- La prueba 772 confirmo `F6I-WORKFLOW-PLAN`, ejecucion local de trayectorias y
  retorno a seleccion tras el prefijo. Una captura `vista_pared` aparecio solo
  tras `TRACKING_RISK`; no se observo todavia una ruta completa FREE que haga
  captura normal de pared.

Conclusion: PARCIAL. La cadena D* -> orden autonoma funciona para prefijos;
falta demostrar el caso FULL_FREE y su depth de fachada ordinario.

## 2026-09-08 - Habilitacion de FREE de trayectoria real

- Sin iniciar aun la generacion de trayectoria continua, se implemento la
  parte previa de 6I: cada pose de navegacion canonica elegible se conserva
  como volumen fisico `FREE` relativo a su keyframe de referencia.
- Un KF con pose global disponible libera su propio volumen de inmediato. Las
  muestras sin pose world disponible esperan al KF; una revision de `W_T_KF`
  elimina e inserta de nuevo todos sus volumenes, sin dejar evidencia fija
  obsoleta.
- La politica acordada mantiene el margen de seguridad fuera de la evidencia
  FREE. Esta evidencia vence ocupacion sparse debil coincidente, pero no
  convierte espacio desconocido en libre.
- Validacion: builds de `task_lib` y `task_server` correctos; CTest 7/7 en cada
  paquete. La prueba 620 con GT, Gazebo y GUI F7 confirmo eventos
  `F6I-FREE-KF` con revisiones push y service reales.

Conclusion: PARCIAL. FREE reversible de volumen recorrido esta conseguido;
faltan perfil multi-waypoint, paridad Server/Dron, conversion W->O,
reproduccion continua y prueba de ejecucion.

## 2026-09-09 - FREE discreto tras ancla y reversible por voxel

- Se retiro el backlog de muestras `pending_world_samples`: no se publica ni
  conserva FREE de barrido mientras falte `W_T_KF`.
- El ancla libera solo la huella fisica del KF, centrada en el voxel que lo
  contiene. Tras el ancla, se materializa una huella adicional solo en la
  primera entrada a cada voxel local nuevo, ligada al KF de referencia.
- Las revisiones de KF reintegran tanto su huella como esos centros locales.
  Se anadio `F6I-FREE-VOXEL` para distinguir una alta discreta de la
  reintegracion `F6I-FREE-KF`.
- Validacion: build de `task_server` correcto y CTest 7/7. Prueba 632 con GT,
  Gazebo y GUI F7 termino con `SIM-DONE success=true`: varios anclajes iniciales
  registraron `traversed_voxels=0`; despues hubo cuatro altas discretas; la
  revision 3 del KF `1:0:31` reintegro exactamente un voxel asociado.
- Revision visual posterior del usuario: los bloques FREE aparecieron despues
  de los OCCUPIED y el desplazamiento no formo una trayectoria perceptible. La
  prueba conserva la validez funcional de la politica, pero no valida aun su
  latencia visual ni que una ruta FREE sea legible en GUI. No quedaron procesos
  residuales.

Conclusion: PARCIAL. La politica FREE discreta, sin backlog preancla y
reversible por KF esta validada funcionalmente; su respuesta visual inmediata y
la legibilidad de la ruta FREE quedan pendientes. Tambien permanecen pendientes
el perfil multi-waypoint, la paridad Server/Dron, la conversion W->O y la
reproduccion continua.

## 2026-09-10 - Primera entrega de trayectoria segmentada

- `TrajectoryPlan` incorpora lifecycle y `hold_initial_yaw`; `TrayAction`
  incorpora el modo multi-waypoint sin modificar los goals legacy.
- `task_server` limita los candidatos D* por 2 m o 5 s y puede despachar solo
  D1. `task_manager` media `ExecuteTrajectory`; `gen_tray` conserva la unica
  conversión W->O y evalua quinticas por tramos continuos.
- Builds correctos de las dos replicas `mission_msgs`, `dron_individual`,
  `task_manager`, `task_server`, GUI y simulacion. CTest: `task_server` 7/7 y
  GUI 9/9 con el overlay completo.
- Pruebas 637, 638 y 639: 637 arranco con un timeout de `fiducial_spawner` y
  Gazebo salio 255. La revision posterior de 638 corrige su clasificacion:
  `task_server` despacho `dstar_1_1` solo a D1, `task_manager` lo reenvio y la
  GUI lo recibio como `ACTIVE`; al mismo tiempo se aborto el goal legacy de D1.
  El goal anterior siguio vivo en `gen_tray` y realizo una segunda finalizacion,
  que lanzo `rclcpp_action::exceptions::UnknownGoalHandleError`. El
  `scenario_runner` recibio el goal legacy cancelado, termino con codigo 250 y
  el helper envio SIGINT/SIGTERM a todo el launch. La caida visual de ambos
  drones es por la retirada conjunta de los controladores, no por GT/ORB ni por
  un despacho 6I a D2. 639 sigue sin evidencia por cierre temprano externo.

Conclusion: PARCIAL. El contrato y la implementacion compilan, pero hay que
corregir primero la preempcion/doble finalizacion de un goal legacy antes de
una simulacion persistente que confirme anclaje, acceptance, curva activa,
finalizacion y encadenamiento de D1.

## 2026-09-10 - Relevo corregido y regresion de reposo terminal

- La prueba 640 corrigio el lifecycle previo: anclaje GT completado para ambos,
  solo D1 recibio ejecucion 6I y no aparecio `UnknownGoalHandleError`. Sin
  embargo D1 cayo; por ello no se acepta `SIM-DONE success=true` como criterio
  fisico suficiente.
- Se acordo que `task_server`, no `gen_tray`, conservara la propiedad de los
  tiempos. Se anadio `execution_timing_factor=2.0`, de modo que cada arista
  usa `factor * distancia / execution_nominal_velocity_mps`. El launch lo
  expone y `F6I-SEGMENT` registra duracion, velocidad nominal y factor.
- Build de `task_server` y `simulacion_dron` correctos; CTest `task_server`
  7/7 y contratos de simulacion 2/2 correctos. La prueba 641 completo el
  escenario y el helper cerro normalmente. D1 ejecuto tres segmentos finales
  de 1.414, 0.894 y 1.162 s, sin excepcion de lifecycle, pero termino con
  tracking/pose ORB invalidos; D2 se mantuvo sano. El epoch nuevo de ORB es un
  sintoma posterior de la caida, no su causa: el control de esta prueba usa GT.
- Revision posterior del diagnostico: se descarta que falte la muestra final.
  El bucle multi-waypoint, igual que el legacy, evalua una vez mas tras superar
  el total y publica el endpoint clampado. La diferencia con la GUI es el
  perfil: un pol3 unico con 16 s manuales frente a varias quinticas D* de
  1.414, 0.894 y 1.162 s, encadenadas sin pausa y con tangentes internas. Aun
  con factor 2, los picos de aceleracion/jerk pueden ser incompatibles con el
  controlador fisico. Falta instrumentar los picos reales y la pose GT antes
  de escoger el siguiente ajuste.

Conclusion: PARCIAL. El relevo de lifecycle esta corregido y el temporizado
del productor funciona, pero 6I no puede declararse conseguida hasta medir y
ajustar el perfil multi-waypoint para que D1 permanezca estable despues del
tramo.

## 2026-09-10 - Contraste lento de temporizado y tormenta de replan

- La prueba 642 repitio el escenario de 641 sin cambiar codigo, con GT,
  fiducial 2, D1 como unico ejecutor y D2 en hover. Se redujo
  `execution_nominal_velocity_mps` de `0.8` a `0.3 m/s`, manteniendo
  `execution_timing_factor=2.0`.
- El contrato temporal se aplico: `F6I-SEGMENT` registro `vel_max_mps=0.300`,
  factor `2.000` y duraciones de hasta 5 s. Durante toda la ventana observada
  D1 siguio publicando pose ORB sana (`epoch=0`, pose local/global valida),
  sin la perdida posterior vista en 641. Es evidencia prometedora de que el
  perfil mas lento evita la inestabilidad fisica, pero no es aun una
  validacion limpia de vuelo.
- La ejecucion automatica genero muchas rutas para la misma tarea y despacho
  reemplazos mientras aun habia una action activa. Se observan pares como
  `dstar_1_207 -> dstar_1_208 -> dstar_1_209` separados por decimas de
  segundo, finales `success=false` del tramo sustituido y
  `F6I-EXECUTION-PREEMPT reason=replan_same_target`. Una revision de mapa por
  si sola esta siendo tratada como motivo suficiente de preempcion, aunque no
  hay evidencia de que invalide el corredor pendiente.
- En el apagado, `gen_tray` termino por intentar publicar el resultado de un
  goal ya inexistente (`Asked to publish result for goal that does not exist`).
  El helper registro `SIM-DONE success=true` antes de esa condicion de cierre,
  por lo que ese codigo no convierte la prueba en correcta.

Conclusion: PARCIAL. La reduccion a `0.3 m/s` es el primer indicio de
estabilidad fisica, pero 642 no valida 6I: hay que serializar la reserva de
dispatch, reemplazar una ruta activa solo cuando la actualizacion afecte su
corredor pendiente y hacer idempotente el terminal durante cancelacion/apagado.

## 2026-09-10 - Meta de exploracion estable y filtrado navegable

- Se anadio una meta de coverage comprometida por tarea, con tolerancia de
  llegada, permanencia minima, ganancia minima y separacion minima como
  parametros. El candidato no se sustituye por cada revision de mapa.
- El segmento activo conserva su corredor voxelizado. Solo un cambio que
  empeora seguridad (pierde transitabilidad o vecindad, o aumenta coste) puede
  solicitar un relevo; la evidencia FREE del propio avance no debe dispararlo.
- La seleccion descarta metas ya alcanzadas y, si D* rechaza un voxel FREE por
  estar ocupado o inflado para el perfil del dron, intenta los siguientes
  candidatos elegibles en esa misma ronda.
- `gen_tray` centraliza el terminal de action y omite un handle ya inactivo,
  para que una cancelacion o apagado no lance un segundo resultado.
- Build de `task_server` y `dron_individual` correctos; CTest `task_server`
  7/7, regresiones dirigidas de dron 2/2 y contratos de simulacion 2/2.
- La 643 elimino la tormenta previa (dos dispatches, ninguna preempcion), pero
  su escenario no alcanzo cierre normal. La 644 confirmo el fallback de
  candidatos y los markers `F6H-CANDIDATE-HOLD`, pero revelo que una mejora
  FREE se trataba erróneamente como cambio de corredor; se corrigio despues.
  La 644 tambien fue interrumpida antes de `SIM-DONE`, por lo que no valida la
  correccion final ni el movimiento visual sostenido.

Conclusion: PARCIAL. El contrato de exploracion estable y sus protecciones
estan implementados y pasan build/CTest. Falta una ejecucion Gazebo completa
posterior a la clasificacion direccional del delta para validar el avance visual
sostenido, ausencia de replans por FREE y cierre normal del escenario.

## 2026-09-10 - Metas de observacion lejanas y rafagas voxel coalescidas

- Se incorporaron `coverage_wall_standoff_m=2.5` y
  `coverage_candidate_min_goal_distance_m=5.0`. El selector conserva solo
  candidatos FREE a distancia aproximada de una celda OCCUPIED y descarta
  movimientos cortos; yaw/pitch siguen fuera de este tramo y pertenecen a 6M.
- `FlushVoxelChanges` ya no recalcula coverage por cada delta: programa una
  pasada coalescida a 500 ms. Esto evita repetir analisis/planning dentro del
  callback voxel y reduce la posibilidad de bloquear servicios.
- Build correcto de `task_lib`, `task_server` y `simulacion_dron`. CTest:
  `task_lib` 9/9, `task_server` 7/7 y contratos de simulacion 2/2. El test
  nuevo valida que el analizador conserva solo el candidato que cumple
  distancia a pared y desplazamiento minimo.
- Prueba 647: ambos goals GT al fiducial 2 terminaron correctamente; el
  servicio de habilitacion, que en 646 habia agotado 20 s, respondio en unos
  9.9 s y el escenario finalizo con `SIM-DONE success=true`. No hubo
  preempciones, microtrayectorias ni errores de terminal. Con el mapa sparse
  actual no hubo candidatos que cumplieran a la vez standoff 2.5 m y distancia
  minima 5 m (`candidates=0`), por lo que D1 permanecio en espera; no se puede
  usar esta ejecucion para validar movimiento sostenido hacia una pared.

Conclusion: PARCIAL. La nueva politica y la respuesta del servicio estan
validadas, pero el entorno de prueba no contiene aun evidencia de pared que
permita una meta de observacion con los parametros acordados. Falta acordar si
la primera exploracion sin pared OCCUPIED debe esperar como ahora o usar una
politica de descubrimiento inicial distinta antes de repetir movimiento.

## 2026-09-10 - Descubrimiento por portal con preferencias suaves

- La interpretacion de la prueba 647 se corrigio sin alterar su evidencia:
  `coverage_wall_standoff_m=2.5` y
  `coverage_candidate_min_goal_distance_m=5.0` dejan de filtrar candidatos.
  Son preferencias del selector; la seguridad continua siendo responsabilidad
  de `OCCUPIED`, inflacion, D* y la tolerancia de llegada.
- `SurfaceCoverageAnalyzer` vuelve a emitir todos los portales alcanzables y
  trata `UNKNOWN` explicito como frontier. `task_server` convierte cada portal
  en una meta FREE y una meta UNKNOWN; esta ultima solo existe como extremo de
  ese portal, no como una muestra arbitraria del volumen.
- La prioridad es ganancia de coverage, diversidad frente a las ultimas ocho
  metas, error de viaje respecto a 5 m y error de standoff respecto a 2.5 m.
  Si ninguna opcion satisface la preferencia, se conserva la alternativa
  factible; un rechazo D* elimina esa celda de la ronda y prueba la siguiente.
- Build de `task_lib` y `task_server` correcto. CTest final: `task_lib` 9/9 y
  `task_server` 7/7, incluido el GTest de portales sin filtro de politica.
- Prueba 648: ambos drones se anclaron por GT en fiducial 2 `(0,-10,Z)` y el
  gate de D1 respondio. El primer mapa genero 58 metas; D* rechazo las
  infladas/no navegables y acepto el portal UNKNOWN `-4:-42:5`. D1 publico,
  activo y completo `dstar_1_1` (4 waypoints, 5 s, 29.697 ms), despues encadeno
  segmentos posteriores sin preemptar una ruta activa. El escenario termino
  con `SIM-DONE success=true` y la GUI F7 recibio poses y trayectorias.

Conclusion: PARCIAL. La politica de descubrimiento inicial ya no queda en
espera infinita y tiene evidencia integrada de D* + ejecucion GT. La
trayectoria continua completa, la evidencia visual humana de este pase y la
optimizacion para descartar antes los numerosos portales inflados siguen
pendientes.

## 2026-09-10 - Ejecucion de metas UNKNOWN y reparacion de corredor

- cambio ejecutado: el ejecutor parcial de 6I recibe ahora solo metas UNKNOWN
  de portal. D* mantiene UNKNOWN penalizado, OCCUPIED/inflacion como veto y
  repara el corredor al mismo destino antes de que 6H pueda sustituirlo.
- build y CTest: `task_server` compilo correctamente; CTest final 7/7.
- prueba 650: GT, anclaje en fiducial 2, D1 como unico ejecutor y D2 en hover.
  El intento recuperado cerro con `SIM-DONE success=true`.
- evidencia: D1 despacho varios prefijos de cinco segundos y mantuvo
  `F6H-CANDIDATE-HOLD reason=active_route_valid` durante rutas activas. Los
  `F6I-CORRIDOR-CHANGE` se asociaron a cambios de corredor; no hubo despacho
  de una meta FREE. D* encontro destinos UNKNOWN tras rechazar los inflados.
- limitacion: la prueba no completa coverage volumetrico y el numero de
  reintentos contra destinos inflados sigue alto. La GUI se abrio, pero no se
  obtuvo una revision visual humana nueva antes de la limpieza del helper.
- conclusion: PARCIAL. La ejecucion conserva la semantica UNKNOWN y cierra el
  escenario correctamente; faltan trayectoria continua global, depth y
  prefiltrado mas barato de candidatos no navegables.

## 2026-09-11 - Ruta completa y bloqueo de la compuerta

- cambio: se retiraron los prefijos artificiales; una ruta D* aceptada conserva
  todos sus waypoints y se temporiza por MRU con velocidad nominal y factor.
- pruebas: las suites dirigidas de servidor, GUI y simulacion pasaron. En
  656/657 el anclaje GT termina, pero la llamada que habilita ejecucion vence
  por backlog serial de mapa; no hay evidencia valida de despacho 6I posterior.
- conclusion: PARCIAL. El contrato esta implementado; la demo integrada espera
  una solucion explicita de concurrencia para el servicio de compuerta.

## 2026-09-11 - Validacion de la compuerta concurrente

- 659 valida la solucion: el servicio responde en unos 4 ms aun cuando el mapa
  tarda 34.141 s; el worker aplica la intencion en una frontera segura.
- no hubo despacho 6I porque la primera meta UNKNOWN recibio `no_safe_escape`
  en 0.286 ms, rechazo esperado por clearance y FREE actual.
- conclusion: PARCIAL. La compuerta deja de bloquear control; falta una meta
  con salida segura para demostrar la ejecucion completa de la ruta.

## 2026-09-11 - Espera de salida segura sin cancelar tarea

- cambio: 6I consume la política de 6H por la que `no_safe_escape` no cancela
  una ejecución inexistente, no libera D1 y no genera trayectoria parcial.
- prueba 660: D1 se mantuvo en la tarea mientras 6H reintentó la misma meta
  UNKNOWN; el escenario cerró `success=true` y no hubo `BLOCKED`.
- limitacion: no hubo ruta aceptada porque el mapa seguía sin salida FREE
  confirmada; la ejecución física completa de 6I permanece pendiente de una
  meta con escape seguro.
- conclusion: PARCIAL, con semántica de espera ya validada.

## 2026-09-11 - Sin trayectoria ejecutable por ausencia de escape local

- prueba 661: D2 fue el único ejecutor, pero su candidato D* quedó en
  `no_safe_escape` durante toda la ventana. No hubo `F6I-ROUTE` para D2 ni
  `ExecuteTrajectory` automático.
- evidencia adicional: el vuelo GT de anclaje sí materializó FREE discreto y
  el escenario terminó correctamente; no basta aún para la inflación física
  de 1 m exigida para escapar.
- conclusión: NO CONSEGUIDA para demostración de ruta física automática. No se
  modifica la política de seguridad hasta acordar cómo obtener su precondición.

## 2026-09-11 - Intencion pendiente y reparacion selectiva de corredor

- cambio: los planes pendientes no guardan una polilinea calculada desde una
  pose antigua; conservan tarea, dron, destino y revision, y D* los recalcula
  con la pose canonica vigente al despacharlos. El corredor activo cubre el
  volumen inflado de la ruta futura y solo se marca para reparacion si un delta
  degrada ese volumen.
- prueba 664: D1 completo su goal GT en fiducial 2, encadeno y finalizo rutas
  `dstar_1_1`, `dstar_1_5`, `dstar_1_8`, `dstar_1_11` y posteriores. Se observa
  una intencion pendiente que se replantea a la revision vigente y un
  `F6I-CORRIDOR-CHANGE` asociado a un voxel del corredor, sin preempcion por
  cambios ajenos. El runner termino `success=true`.
- limite: la trayectoria entregada sigue siendo la ruta temporizada existente;
  no hay aun generador puro compartido de curva continua ni validacion server
  del volumen barrido, limites dinamicos y colision antes de la action. El
  aviso terminal durante SIGINT (`goal ... does not exist`) permanece solo como
  ruido de apagado a corregir en el lifecycle.

Conclusion: PARCIAL. El despacho desde intencion fresca y la reparacion
selectiva se demostraron; la cadena completa curva-validacion-ejecucion queda
pendiente.

## 2026-09-11 - Migracion Pol3Waypoints y prueba integrada 671

- cambio: se retiró `ExecuteMultiWaypoint`. `lib_tray` incorpora
  `GenTrayPol3Waypoints` y `GenTrayVelTrapWaypoints`; `TrayAction` transporta
  `waypoint_targets` y tiempos acumulados, y `task_manager` omite el primer
  punto visual de D* para que `gen_tray` capture el origen canónico.
- lifecycle: `task_server` publica el plan `PLANNED` con el mismo
  `trajectory_id` antes de enviarlo a `ExecuteTrajectory`, de modo que GUI F7
  y el dron reciben la misma geometría.
- verificación mecánica: réplicas de `mission_msgs` idénticas; `lib_tray` 5/5,
  `dron_individual` 8/8, `task_server` 7/7 y `multidron_gui_lib` 9/9. Los
  binarios instalados resuelven a los builds actuales y su interfaz ROS expone
  `waypoint_targets`/`waypoint_times`.
- prueba 671: D1 llegó por GT al fiducial 2. El primer plan D* fue
  `dstar_1_1`, seis waypoints y 15.797 s; 1.1 s después de quedar `ACTIVE`, un
  cambio de corredor duro solicitó cancelación, STOP de 5 s y replan. El patrón
  volvió a ocurrir con `dstar_1_6`. El usuario observó la caída de D1 y detuvo
  la simulación.
- diagnóstico: el corredor futuro, compuesto también por UNKNOWN mutable,
  puede invalidarse antes de estabilizar el tramo. Además no aparecen
  `F6I-POL3-WAYPOINTS-START/FINAL`, aunque las acciones se aceptan y los
  binarios contienen esos marcadores. Se debe instrumentar de forma causal el
  salto `task_manager` -> `gen_tray` y rechazar un goal 6I sin destinos, para
  eliminar cualquier fallback legacy silencioso antes de modificar la política
  de relevo.

Conclusión: NO CONSEGUIDA para estabilidad física integrada. No se atribuye a
ORB porque la prueba usó `GT_FORCED`; tampoco es un plan D* simple erróneo, sino
la interacción entre su corredor mutable y la cadena de actions/STOP.

## 2026-09-11 - Replanteo seguro y perfil UNKNOWN

- cambio: las metas UNKNOWN se temporizan a 0.4 m/s y las FREE a 0.8 m/s.
  `gen_tray` retorna `stale_start_pose` si la primera pose ya está a más de
  0.25 m, y para desfases menores retemporiza desde la pose actual. El servidor
  conserva intención, nunca una ruta obsoleta.
- corredor: una degradación blanda prueba reparar D*; una dura cancela la
  action anidada mediante `task_manager`, espera `canceled` y replantea desde
  la pose vigente.
- prueba 665 interrumpida: `dstar_1_1` fue despachada con 5 waypoints,
  16.223 s y 0.4 m/s. Una degradación dura causó cancelación propagada a
  `gen_tray` y replan posterior. No demuestra aún suavidad visual ni
  estabilidad física prolongada.
- revisión visual posterior: el usuario observó caída de D2. El log de 665
  confirma que D2 solo tuvo candidatos de coverage; no recibió
  `F6I-EXECUTION-DISPATCH`, `F6I-TASK-EXECUTE` ni una action física. Todos los
  goals y dispatch fueron de D1. La caída no se atribuye a 6I/D*: queda como
  inestabilidad independiente de control/simulación que debe aislarse antes de
  una prueba visual multi-dron.
- corrección de interpretación: el usuario aclara que la caída era de D1. La
  secuencia de D1 contiene múltiples `F6I-EXECUTION-CANCEL` y dispatch/replan
  inmediatos. El controlador usa el feedback de `AccionTrayectoria`; al
  cancelar el modo multi-waypoint, `gen_tray` deja de publicarlo sin transición
  de frenado/hold. Por tanto conserva el último setpoint con posible velocidad
  y aceleración, y recibe el siguiente plan antes de estabilizarse. Es la causa
  probable de la caída y salida del ROI; requiere relevo dinámicamente seguro,
  no atribuirlo a D2 ni a una meta D* fuera de la subROI.

## 2026-09-11 - STOP normal, sin cancelacion ROS

- cambio: `ExecuteTrajectory` incorpora `stop_at_current_pose` en ambas
  réplicas. `task_server` reemplaza `async_cancel_goal` por una orden STOP sin
  waypoints y borra la polilínea de la ruta retirada. `task_manager` serializa
  las actions, reemplaza la ruta local por `TrayAction.stop_at_current_pose` y
  expone el terminal normal; `gen_tray` ejecuta el hover Pol3 local de 5 s.
- pruebas de código: builds correctos de `mission_msgs` servidor/dron,
  `dron_individual`, `task_manager` y `task_server`; CTest
  `dron_individual` 8/8 y `task_server` 7/7.
- prueba 674: escenario GT de D1 al fiducial 2, D2 sin ruta física, GUI F7 y
  Gazebo. El helper y el escenario terminaron con código 0. Se observaron nueve
  secuencias con `STOP-REQUEST`, reemplazo de la ruta por
  `replaced_by_stop`, `TASK-STOP-FINAL success=true` aproximadamente 5 s más
  tarde, `EXECUTION-STOP-FINAL success=true` y replan posterior. No hubo
  `async_cancel_goal`, rechazo STOP, fallo terminal ni proceso residual.
- salvedad: el primer terminal STOP llegó al servidor con demora frente al
  terminal de `task_manager`; el orden causal se conservó y las secuencias
  posteriores se resolvieron normalmente. Los marcadores internos de
  `gen_tray` no llegaron al log aunque el binario instalado los contiene; la
  cadena de actions acredita el resultado, pero la instrumentación debe
  revisarse antes de usar esos marcadores para cronometraje fino.

Conclusión: CONSEGUIDA para el protocolo STOP. 6I global permanece PARCIAL.

## 2026-09-12 - Spline C2, depuración de ruta y STOP estricto

- se retiró la reparación física por cambios blandos: solo un cambio
  `transitable -> no transitable` por `OCCUPIED` o inflación dentro del corredor
  activo solicita STOP. `UNKNOWN -> FREE`, coste y vecindad actualizan D* sin
  cancelar ni retirar la ruta;
- `task_server` depura la cadena D* antes de publicarla y despacharla. El
  parámetro `trajectory_waypoint_min_separation_m=0.5` omite puntos cercanos
  solo cuando el atajo conserva la transitabilidad inflada. Tras ello aplica
  `trajectory_min_segment_duration_sec=1.0` a cada tramo;
- `GenTrayPol3Waypoints` resuelve ahora una spline cúbica global C2 por eje,
  con velocidad inicial capturada, velocidad final nula y aceleración continua
  en los vértices. La prueba de tiempos no uniformes quedó añadida a `lib_tray`;
- validación de código: builds correctos de `lib_tray`, `task_server`,
  `dron_individual`, `task_manager` y `simulacion_dron`; CTest
  `lib_tray` 5/5, `task_server` 7/7 y `dron_individual` 8/8;
- prueba 675: D1 llegó con GT al fiducial 2, activó coverage y ejecutó rutas
  D* de 2--8 waypoints. Las cuatro solicitudes STOP observadas fueron siempre
  precedidas por `F6I-CORRIDOR-CHANGE severity=occupied_or_inflated`; no hubo
  marcador de reparación blanda, cancelación ROS ni STOP asociado a
  `UNKNOWN -> FREE`. Tres STOP cerraron con éxito normal y replanteo posterior.
  En el primer relevo la ruta sustituida notificó `success=false` mientras el
  STOP local ya había terminado con éxito; el replan posterior continuó, pero
  se conserva como salvedad de orden/telemetría a vigilar;
- escenario y helper terminaron con código 0, y no quedaron procesos ROS,
  Gazebo ni GUI. El `exit 255` de Gazebo se produjo durante el SIGINT de
  limpieza tras `SIM-DONE`, no durante la ejecución.

Conclusión: PARCIAL. Se valida automáticamente la nueva política de corredor,
la entrega de rutas y la continuidad C2 unitaria. Queda pendiente una revisión
visual prolongada que confirme suavidad física y una instrumentación más fina
del relevo en el primer terminal sustituido.

## 2026-09-12 - Empalmes Pol3 C1 y temporización conservadora

- cambio: se retiró la spline C2. Con varios destinos,
  `GenTrayPol3Waypoints` crea cada arista nominal, la recorta `1.0 s` antes y
  después de cada guía interior, e intercala un Pol3 de `2.0 s` que conserva
  pose y velocidad. No exige aceleración/jerk continuos ni que el vehículo
  cruce el vértice de guía. Un destino sigue siendo idéntico al `GenTrayPol3`
  legacy; los calendarios que solaparían las ventanas se rechazan.
- parámetros: `trajectory_waypoint_min_separation_m` sube a `1.0 m` y
  `trajectory_min_segment_duration_sec` a `3.0 s`. El último destino no se
  elimina por cercanía. `phase6_waypoint_blend_sec=1.0` llega desde
  `multi_dron.launch.py` a cada `gen_tray`.
- validación de código: builds correctos de `lib_tray`, `task_server`,
  `dron_individual`, `task_manager` y `simulacion_dron`; CTest `lib_tray` 5/5,
  `task_server` 7/7, `dron_individual` 8/8 y contratos de simulación 2/2.
  `task_manager` no declara una suite CTest propia.
- prueba 676: D1/GT alcanzó el fiducial 2, habilitó coverage y el escenario
  terminó `SIM-DONE success=true`. Se despacharon rutas D* D1 de 2--4
  waypoints y 10.001--15.601 s; las cuatro solicitudes STOP observadas fueron
  de nuevo exclusivamente `occupied_or_inflated`. No se registró simplificación
  de waypoint en esta geometría concreta. El marcador
  `F6I-POL3-WAYPOINTS-START` es `INFO` y `gen_tray` se ejecutó con nivel
  `warn`, por lo que esta ejecución no aporta su marca interna ni una medida
  visual de suavidad. El `exit 255` de Gazebo ocurrió únicamente tras el
  SIGINT de limpieza posterior a `SIM-DONE`; no quedaron procesos ROS, GUI o
  Gazebo.

Conclusión: PARCIAL. La composición C1, los defaults y su integración de
compilación están validados; 676 confirma el flujo de plan/action/STOP sin
regresión de seguridad. Sigue pendiente una revisión visual prolongada de la
suavidad física y una ejecución con telemetría INFO si se necesita atribuir
cada acción en `gen_tray` de forma causal.

### Reinterpretación visual posterior de 676

El usuario observa brusquedad en los cambios de waypoint y no percibe
continuidad de velocidad. Por tanto 676 no acredita suavidad física, aunque el
GTest acredita igualdad de pose/velocidad en las fronteras matemáticas del
empalme C1. La hipótesis principal no es todavía una conclusión: puede ser el
salto de aceleración permitido por C1, la discretización de feedback o una
discontinuidad posterior entre `gen_tray` y control. Hay que medir referencias
de posición/velocidad inmediatamente antes y después de cada frontera antes de
escoger un perfil de mayor orden.

El usuario también percibe varios STOP aparentemente innecesarios. El reducido
solo demuestra que los cuatro STOP de D1 tuvieron motivo interno
`occupied_or_inflated`; no demuestra que esa ocupación o inflación estuviera
en una zona físicamente peligrosa para el dron. Debe contrastarse cada voxel
causal contra la polilínea, su inflación y la capa mostrada en GUI antes de
alterar la política de STOP.

## 2026-09-12 - Diagnóstico de fronteras C1 y STOP (prueba 677)

- instrumentación: `TrayAction` propaga `trajectory_id` e índice de pieza
  diagnóstica; `gen_tray` registra ambos lados de cada frontera Pol3 y el
  controlador registra la referencia recibida junto a estado GT. El parámetro
  temporal `phase6_debug_trajectory_diagnostics` no modifica perfiles, control,
  D* ni STOP. `F6I-STOP-CAUSAL` añade estado raw, causa de inflación, segmento
  más cercano y distancia a polilínea.
- validación previa: builds de `lib_tray`, `dron_individual`, `task_manager`,
  `task_server` y `simulacion_dron` correctos; CTest 5/5, 8/8, 7/7 y contratos
  de launch 2/2.
- prueba 677: D1/GT alcanzó el fiducial 2, D2 permaneció en hover y la ventana
  de coverage terminó con `SIM-DONE success=true`. En las fronteras medidas de
  `dstar_1_1`, `dstar_1_11` y `dstar_1_12`, pose y velocidad de referencia
  fueron idénticas antes/después al redondeo mostrado; el controlador recibió
  la pieza nueva en su siguiente feedback (normalmente submilisegundos y, en
  algunas muestras, hasta unos 30 ms). No hay evidencia de salto de velocidad
  entre `GenTrayPol3Waypoints` y el controlador.
- STOP: hubo cuatro solicitudes reales, para `dstar_1_1`, `dstar_1_5`,
  `dstar_1_8` y `dstar_1_11`. Todas fueron por una celda raw `UNKNOWN` que se
  volvió no transitable por inflación de una ocupación vecina; las distancias
  registradas a la polilínea fueron `1.250`, `1.097`, `1.215` y `0.436 m`.
  Dos trazas causales adicionales con distancia infinita llegaron mientras el
  STOP ya estaba solicitado y la ruta retirada: no emitieron una segunda orden
  STOP y son ruido diagnóstico a suprimir si se mantiene esta sonda.

Conclusión: PARCIAL. La brusquedad observada no procede de una discontinuidad
de velocidad de referencia ni de su entrega inicial a control. Sigue siendo
compatible con el salto de aceleración permitido por C1, con la respuesta
dinámica del controlador o con los cuatro STOP por inflación. La política STOP
no debe cambiarse aún: hace falta acordar si se prioriza suavidad C2/mayor
ventana de empalme, o una revisión funcional del criterio de inflación.

## 2026-09-12 - Temporización 8/3 y ocupación sparse filtrada (prueba 678)

- `trajectory_min_segment_duration_sec` subió a `8.0 s` y
  `waypoint_blend_sec` a `3.0 s` en servidor, librería, configuración y launch.
  Un tramo interior conserva al menos `2 s` nominales entre sus recortes de 3 s.
- Las unidades quedaron correctas: `task_lib` 9/9, `lib_tray` 5/5,
  `task_server` 7/7, `dron_individual` 8/8 y contratos de simulación 2/2.
  La regresión de `lib_tray` comprueba el calendario C1 8/3.
- Prueba 678: D1/GT alcanzó el fiducial 2, D2 no recibió dispatch físico y el
  escenario terminó `SIM-DONE success=true`. Las rutas arrancaron con
  `blend_sec=3.000`; una ruta corta conservó `duration_sec=8.000`, y las
  fronteras registraron pose/velocidad coincidentes antes y después.
- Persistieron STOP por `occupied_or_inflated`; esa política no se alteró y la
  prueba no certifica por sí sola la suavidad física prolongada.

Conclusión: la configuración 8/3 y su recorrido plan/action quedan validados;
6I sigue PARCIAL por suavidad física, barrido/límites y política de inflación.

### Revisión visual posterior de 678 - polilínea pendiente sustituye a la activa

El usuario observó que la GUI F7 borraba una polilínea que D1 seguía físicamente
y mostraba otra. No es una desviación de D*, de `gen_tray` ni de la elección
física del destino: `QueueOrDispatchCoveragePlan` conserva la action activa
ante un objetivo distinto, pero publica el nuevo plan con estado `PLANNED` y
marca `F6I-EXECUTION-PENDING`. El bridge GUI recibe cualquier plan no terminal
por `/mission/planned_routes` y ejecuta `ReplaceTrajectory` por `drone_id`, por
lo que dibuja esa intención pendiente encima de la ruta `ACTIVE` que sigue
consumiendo el dron. La 678 contiene esa secuencia para `dstar_1_11` y
`dstar_1_12`.

Hay además una fragilidad independiente: al recibir un terminal, el bridge hace
`ClearTrajectory(drone_id)` sin comprobar que el `trajectory_id` terminal sea
el que está dibujando. La corrección deberá preservar en la vista principal
solo la ruta `ACTIVE` (o, como mucho, la recién despachada mientras llega el
acuse), ignorar `PLANNED` pendiente y condicionar cada clear a identidad. Una
vista secundaria de intención pendiente, si se desea, debe tener capa y estilo
distintos; no puede reemplazar la polilínea ejecutada.

## 2026-09-12 - Lifecycle visual de ruta activa (prueba 679)

- `RosDataBridge` ignora todo `TrajectoryPlan` no terminal que no sea `ACTIVE`.
  `GuiDataModel::ClearTrajectory` recibe además el `trajectory_id` y solo borra
  si coincide con la geometría visible del dron.
- Build de `multidron_gui_lib` correcto y CTest 9/9, incluida la regresión en
  que el terminal de `traj_A` no borra `traj_B`.
- Prueba 679: D1/GT completó fiducial 2 y coverage con `SIM-DONE success=true`;
  D2 permaneció sin dispatch físico. Mientras `dstar_1_2` era activa,
  `dstar_1_3` se publicó como `F6I-EXECUTION-PENDING` y GUI registró solo
  `GUI-TRAJECTORY-IGNORE`. La capa no cambió hasta el estado `ACTIVE`; los
  terminales repetidos tras STOP registraron `cleared=false` al no coincidir.

Conclusión: CONSEGUIDA para la corrección de lifecycle visual. No altera D*,
coverage, STOP ni el estado agregado PARCIAL de 6I.

## 2026-09-16 - Monitor FIFO, reserva y terminal correlacionado (prueba 775)

- objetivo intentado: integrar los planes FIFO en `ACTIVE_TRAJECTORY_MONITOR`
  antes de despacharlos, para que tengan el mismo ciclo de reserva, GUI y STOP
  que los planes de ejecución ya activos.
- implementación: `MOVE_AND_CAPTURE` se encola al monitor. Tras aceptar la
  orden normal, este compromete el corredor reservado, publica la polilínea
  `ACTIVE` y conserva la correlación `(drone_id, workflow_id, command_id)`.
  El terminal autónomo libera la reserva y retira la ruta sin reencolar por la
  senda legacy.
- build y tests: `task_server` compiló correctamente; CTest completo 9/9,
  incluidos tres GTests y todos los linters.
- prueba Gazebo: 775, D1/GT, fiducial 2, 180 s autónomos, depth activo. El
  runner informó `SIM-DONE success=true` y `SIM-EXIT-CODE 0`.
- evidencia: se observaron compromisos `F6J-AUTONOMOUS-RESERVATION-COMMIT`
  de 102 a 259 celdas para `dstar_1_1` a `dstar_1_10`. Varios terminales
  `segmento completado por gen_tray` liberaron su monitor, y los STOP
  terminaron como `replaced_by_stop`, también liberando la reserva de forma
  correlacionada. No se reprodujo el bloqueo de rutas FIFO sin `RESERVED`.
- limitación: tras el final correcto del escenario, durante el SIGINT/SIGTERM
  forzado del runner apareció `UnawareGoalHandleError` de ROS al publicar el
  resultado de una action ya retirada. Es un defecto de lifecycle de apagado,
  no evidencia de fallo de la ventana autónoma, y queda pendiente separado.
- conclusión: CONSEGUIDA para la integración monitor/reserva/terminal del
  flujo FIFO. 6I global permanece PARCIAL por los pendientes de política,
  profundidad de pared y validación física prolongada.

## 2026-09-17 - Captura obligatoria al avanzar por corredor FREE (779)

- la prueba 778 reveló que `fallback_free_advance` se publicaba con
  `capture_after_command=false`: el dron llegaba al último FREE, devolvía
  `depth=0` y el servidor iniciaba otra mirada UNKNOWN.
- en 779 se unificó con `FREE_PREFIX` como `VIEW_ADVANCE`. D1 completó esos
  movimientos con `depth=1`; el servidor escribió fuentes `view_advance` y no
  liberó la siguiente selección hasta `F6F-DEPTH-SOURCES-APPLIED`.
- hubo STOPs independientes que siguen reportando `result_without_usable_depth`;
  no se modificó la política de monitor ni de inflación en esta corrección.

Conclusión: CONSEGUIDA para el contrato de captura del avance FREE; 6I global
continúa PARCIAL por STOP y validación física del recorrido de fachada.

## 2026-09-17 - Inflacion uno y causal de STOP (prueba 786)

- `extra_obstacle_clearance_voxels` paso de 2 a 1; D1 navega con inflacion
  `(3,3,2)` y reserva fisica `(2,2,1)`.
- 786 cerro limpia y GUI publico reservas de 102 a 568 celdas. El diagnostico
  causal encontro dos STOP por `unresolved_navigation_change` sobre UNKNOWN y
  uno por `raw_occupied_static_clearance`; no hubo STOP sin marcador causal.
- conclusion: CONSEGUIDA para el ajuste de margen y observabilidad causal. 6I
  sigue PARCIAL porque los cambios navegables no resueltos requieren politica
  futura, distinta de la seguridad de esta correccion.
