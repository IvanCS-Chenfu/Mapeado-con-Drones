# Resumen - 6I

Estado: PARCIAL. La prueba 773 anadio diagnostico opcional de rechazo D*
estricto y confirma que el origen de D1 era raw `FREE` pero navegablemente
`occupied_inflated`, mientras la meta seguia raw/navegablemente `UNKNOWN`
despues de aplicar el depth. D* rechazo primero la meta
`goal_occupied_or_inflated`, por lo que su salida segura no llego a evaluarse.
La continuacion `vista_unknown -> TRAJECTORY_PLANNING` debe revalidar el
objetivo ya materializado y no puede ejecutar UNKNOWN en una futura salida.
El acuerdo correctivo mantiene la inflacion global, pero permite al plan salir
por raw FREE dentro de una burbuja de 4 voxeles alrededor de la pose actual.
Si la meta sigue UNKNOWN, 6H buscara un fallback FREE de menor coste dentro de
8 voxeles; sin alternativa, descarta el candidato visual.

La prueba 772 incorpora `TrajectoryPlanningWorker` FIFO:
tras depth UNKNOWN, D* genero y ejecuto prefijos FREE por el runtime 6D, sin
cruzar UNKNOWN. El unico depth `vista_pared` observado fue tras
`TRACKING_RISK`; falta una llegada FULL_FREE que capture una fachada de forma
normal. 632 valido FREE fisico discreto y reversible por KF. La entrega
de trayectoria continua compila: lifecycle en `TrajectoryPlan`, segmentos D*,
mediacion `ExecuteTrajectory` y conversion W->O unica en `gen_tray`, con
quinticas multi-waypoint. 640 corrigio la doble finalizacion legacy. 641 con
`vel_max=0.8` siguio siendo `NO CONSEGUIDA` fisicamente. En contraste, 642 con
`vel_max=0.3` y factor 2.0 mantuvo D1 con pose valida durante la observacion,
por lo que el temporizado lento es prometedor. No es una prueba de exito: el
servidor lanzo y preempto repetidamente planes del mismo objetivo ante revisiones
ordinarias, y `gen_tray` intento publicar el resultado de un goal ya inexistente
durante apagado. Falta serializar dispatch, exigir invalidacion real del corredor
para relevar y volver idempotente el terminal. FREE sigue sin validar latencia
visual.

Actualización 671: `ExecuteMultiWaypoint` fue sustituido por
`GenTrayPol3Waypoints`/`GenTrayVelTrapWaypoints`; builds de las dos réplicas,
`lib_tray` 5/5, `dron_individual` 8/8, `task_server` 7/7 y GUI 9/9 correctos.
La simulación GT con D1 al fiducial 2 fue interrumpida al caer D1. El log muestra
`hard_corridor_degradation` seguido de STOP de 5 s y replans repetidos, incluso
antes de que la ruta se estabilice; no aparecen los marcadores
`F6I-POL3-WAYPOINTS-*`. Por tanto la migración compila y sus unidades pasan,
pero la ejecución física queda NO CONSEGUIDA y requiere instrumentar/serializar
el relevo antes de otra prueba.

Actualización 674: el relevo simplificado ya está implementado. STOP es una
orden de `task_server` sin waypoints; el dron genera localmente el hover Pol3
de 5 s y lo termina como una action normal. Se validaron nueve ciclos
`STOP -> terminal normal -> replan`, sin `async_cancel_goal`, sin
`stop_completed` y sin procesos residuales. La validación del protocolo STOP
es CONSEGUIDA; el estado agregado de 6I permanece PARCIAL por sus objetivos de
validación de curva, barrido y estabilidad física global aún pendientes.

Actualización 675: STOP queda exclusivamente ante `OCCUPIED` o inflación del
corredor; la simulación GT de D1 al fiducial 2 ejecutó rutas de 2--8 waypoints
y sus cuatro paradas se debieron solo a ocupación/inflación, sin STOP blando ni
cancelación ROS. Un primer terminal de ruta sustituida llegó con `success=false`
aunque su STOP local terminó bien y hubo replan, por lo que 6I sigue PARCIAL.

Actualización 676: la spline C2 se sustituyó por aristas Pol3 nominales y
empalmes cúbicos C1 de 2 s alrededor de cada guía interior (`t_waypoint=1 s`).
La librería conserva paridad exacta con Pol3 legacy para un único destino,
rechaza ventanas solapadas y pasó 5/5. Los defaults son separación segura
`1 m` y mínimo `3 s`; el último destino sigue protegido. Builds de los cinco
paquetes, CTest `task_server` 7/7, `dron_individual` 8/8 y contratos de launch
2/2 pasaron. La 676 cerró `SIM-DONE success=true`, con D1/GT, rutas D* de 2--4
waypoints y STOP solo por ocupación/inflación. No hubo simplificación en esa
geometría ni el marcador INFO interno de `gen_tray`, por lo que falta revisión
visual prolongada de suavidad y telemetría causal fina; el estado permanece
PARCIAL.

Revisión visual posterior: el usuario observa brusquedad en cambios de waypoint
y varios STOP que parecen innecesarios. La continuidad C1 unitaria no valida la
suavidad física; falta medir la referencia antes/después de cada frontera. Los
STOP conservan motivo interno `occupied_or_inflated`, pero falta verificar su
intersección real con polilínea inflada y GUI antes de cambiar la política.

La correccion posterior incorpora una meta de coverage comprometida, filtrado
de candidatos ya alcanzados/no navegables y relevo solo ante degradacion real
del corredor. Compila y pasa CTest, pero las pruebas 643/644 fueron
interrumpidas antes de su cierre normal: no demuestran aun movimiento visual
sostenido ni cierran 6I.

La prueba 647 conserva la evidencia de servicio/anclaje, pero su interpretacion
se corrigio: `standoff=2.5 m` y distancia deseada `5 m` no deben ser filtros
duros. La nueva politica usa portales `FREE/UNKNOWN` alcanzables, prioriza
ganancia, diversidad espacial y ambas preferencias, y deja a D* vetar
ocupacion/inflacion. `task_lib` 9/9 y `task_server` 7/7 pasan tras el cambio.
La prueba 648 con GT en fiducial 2 termino con `SIM-DONE success=true`: D1
rechazo los portales inflados, acepto un portal UNKNOWN, ejecuto y completo
`dstar_1_1`, y continuo segmentos sin preempcion de una ruta activa. La
politica deja de estar bloqueada en el mapa sparse; queda optimizar el numero
de candidatos que D* debe descartar y completar la trayectoria continua global.

La correccion 650 restringe la ejecucion a metas UNKNOWN y conserva la ruta
activa salvo degradacion del corredor. `task_server` paso 7/7 y 650 cerro con
`SIM-DONE success=true`; D1 despacho varios prefijos y no hubo meta FREE. La
evidencia sigue siendo parcial: no hay depth, coverage volumetrico ni revision
visual humana nueva, y persisten reintentos hacia portales inflados.

Actualizacion: 6I recibe ahora rutas completas temporizadas sin truncado. Las
pruebas 656/657 muestran que el bloqueo de la demo es la compuerta retenida por
callbacks de VoxelMapWorker, no D*. Su validacion integrada sigue pendiente.

La causa de compuerta se resolvio en 659 con grupos mapa/control. El flujo
ejecutable sigue pendiente solo de una meta que admita salida segura.

La 660 verifica que un `no_safe_escape` no cancela ni bloquea esa tarea: D1
conserva la meta UNKNOWN, la reintenta tras el deadline y no recibe trayectoria
parcial. Sigue pendiente demostrar una ruta física aceptada con salida FREE.

La 661 confirma el pendiente: incluso con anclaje GT y FREE discreto, D2 no
recibió una ruta ejecutable porque su escape local no tenía FREE suficiente
para el clearance. La ejecución física automática continúa sin demostración.

La 664 incorpora el despacho desde intencion fresca: una ruta pendiente se
recalcula desde la pose canonica justo antes de ejecutarse y el corredor se
repara solo al degradarse dentro de su volumen inflado. D1 encadeno varias
rutas D* y el escenario termino correctamente. Sigue PARCIAL: el servidor aun
no valida una curva continua, su barrido ni sus limites dinamicos antes de la
action; el aviso terminal durante SIGINT es un pendiente de lifecycle.

Actualización 665: rutas con meta UNKNOWN usan 0.4 m/s y las FREE 0.8 m/s; el
primer dispatch observado fue `dstar_1_1` con 5 waypoints y 16.223 s. Una
degradación dura canceló explícitamente `dstar_1_1`, `task_manager` propagó la
cancelación a `gen_tray` y el servidor replanteó desde la intención. La
ejecución fue detenida manualmente, por lo que no valida aún suavidad visual ni
estabilidad física prolongada.

Actualización 677: la telemetría activable correlaciona `trajectory_id`, piezas
Pol3, referencia de control y STOP causal. Builds de cinco paquetes correctos;
CTest 5/5, 8/8, 7/7 y contratos 2/2. D1/GT completó la prueba y las fronteras
medidas conservan exactamente pose y velocidad de referencia; el controlador
recibe la pieza nueva en el siguiente feedback. Cuatro STOP reales fueron por
inflación de ocupación vecina sobre celdas raw UNKNOWN, a `0.436--1.250 m` de
la polilínea; trazas posteriores durante STOP no son órdenes adicionales.
Conclusión agregada: 6I sigue PARCIAL. La brusquedad no se atribuye a un salto
de velocidad de referencia; quedan aceleración C1, dinámica y política de
inflación como hipótesis separadas.

Actualización 678: el mínimo por tramo pasó a 8 s y el empalme C1 a 3 s. Los
tests dirigidos fueron `task_lib` 9/9, `lib_tray` 5/5, `task_server` 7/7,
`dron_individual` 8/8 y contratos de simulación 2/2. D1/GT completó fiducial 2
y coverage con `SIM-DONE success=true`; las rutas registraron
`blend_sec=3.000`, una duración mínima de 8 s y fronteras C1 de pose/velocidad.
Persistieron STOP por `occupied_or_inflated`, que quedan fuera de esta
modificación. 6I mantiene estado PARCIAL por validación física prolongada,
barrido/límites y política de inflación.

Revisión visual posterior: la GUI sustituye indebidamente una ruta `ACTIVE` por
cualquier `PLANNED` publicado como intención pendiente, aunque el dron siga la
primera. El servidor conserva correctamente la action activa; el defecto está
en el contrato visual compartido y en `RosDataBridge`, que además limpia por
dron sin comprobar `trajectory_id`. Pendiente de decisión: mostrar solo activa
o añadir una capa diferenciada para intención pendiente.

Actualización 679: la GUI ignora `PLANNED`, dibuja solo `ACTIVE` y condiciona
el clear por `trajectory_id`. `multidron_gui_lib` compiló y pasó 9/9. En
simulación GT, `dstar_1_3` quedó pendiente mientras D1 seguía `dstar_1_2` y la
GUI registró `IGNORE`, sin reemplazar la línea; cada actualización ocurrió al
entrar en `ACTIVE`. La corrección visual queda CONSEGUIDA; 6I global sigue
PARCIAL por sus pendientes físicos y de política de inflación.

Actualización 775: el despacho FIFO `MOVE_AND_CAPTURE` pasa por
`ACTIVE_TRAJECTORY_MONITOR`. D1/GT comprometió corredores de 102--259 celdas,
publicó la ruta activa y liberó cada reserva en el terminal normal o tras STOP,
con `SIM-DONE success=true`. El `UnawareGoalHandleError` apareció solo durante
el apagado forzado posterior del runner y queda como pendiente de lifecycle.
La integración monitor/reserva queda CONSEGUIDA; 6I global continúa PARCIAL.
# Actualización 777

La validación integrada de la U no pasa: el monitor cancela muchas rutas con
`occupied_or_inflated_corridor`, incluso antes de una captura wall, y el STOP
falla con frecuencia. Se conservan reservas y planes publicados, pero la
secuencia no llega de forma fiable a `MOVE_AND_CAPTURE`; queda pendiente aislar
qué celda del corredor dispara cada STOP.

Actualización 778/779: el avance FREE de respaldo después de una mirada
UNKNOWN ya no se ejecuta sin captura. Se unifica con `FREE_PREFIX` como
`VIEW_ADVANCE`, devuelve depth, espera su materialización y solo entonces
reselecciona. La 779 valida el ciclo; 778 se conserva como fallo que detectó
el anterior `depth=0`. Persisten interrupciones por STOP y la cobertura de
fachada normal, así que el estado agregado sigue PARCIAL.

Actualizacion 786: la inflacion adicional se reduce a un voxel y D1 navega con
`(3,3,2)`, frente a una reserva fisica `(2,2,1)`. Las reservas fueron visibles
en GUI. El diagnostico causal explica los tres STOP observados: dos cambios
navegables UNKNOWN no resueltos y un ocupado real dentro del clearance. El
ajuste y su trazabilidad quedan validados; 6I global sigue PARCIAL.
