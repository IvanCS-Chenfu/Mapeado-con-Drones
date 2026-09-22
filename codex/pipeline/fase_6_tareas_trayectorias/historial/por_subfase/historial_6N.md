# Historial 6N

## 2026-09-14 - Migracion al barrido de fachada

- objetivo intentado: sustituir la exploracion reiterada de destinos `UNKNOWN`
  por inspeccion depth bajo demanda y barrido lateral conocido `FREE`.
- archivos modificados: contratos `mission_msgs`, procesamiento depth de
  `orbslam3_ros2`, `InspectFacade` en `task_manager`, runtime de fachada en
  `task_server`, utilidades de `task_lib`, GUI F7, launch y documentacion.
- resultado de build: correcto en los paquetes dirigidos; CTest vigente de
  `task_server` 7/7, `orbslam3_server` 13/13, `task_lib` 9/9 y
  `multidron_gui_lib` 9/9.
- evidencia positiva: contratos, tests unitarios y cadena estatica compilan.
- evidencia ausente: faltaba una ejecucion integrada completa del barrido.
- conclusion: PARCIAL.

## 2026-09-14 - Prueba 734

- objetivo intentado: D1/GT hasta fiducial 2 y barrido automatico con una GUI.
- prueba Gazebo: no alcanzo ningun paso porque el runner recibio una ruta YAML
  relativa invalida desde la raiz del workspace.
- conclusion: NO CONSEGUIDA por error mecanico del escenario, sin evidencia
  funcional sobre 6N.

## 2026-09-14 - Pruebas 735 y 736

- objetivo intentado: repetir la validacion integrada con YAML absoluto.
- evidencia negativa: al anclarse D1, `InspectFacade` envio una reorientacion al
  mismo `gen_tray` unos instantes antes de terminar la llegada externa y
  sustituyo ese goal. Aumentar el periodo del worker no elimino la carrera.
- correccion: `gen_tray` publica `control/trajectory_active`; `task_manager`
  aplaza la inspeccion con `drone_busy`, y `task_server` no consume intentos por
  ese aplazamiento.
- builds/tests: `dron_individual`, `task_manager` y `task_server` correctos;
  CTest `task_server` 7/7.
- conclusion: NO CONSEGUIDAS como pruebas integradas; diagnostico y correccion
  implementados.

## 2026-09-14 - Prueba 737

- prueba Gazebo/F7: ejecucion terminal correcta tras 431 s, con una sola GUI y
  sin activar la guarda de recursos.
- evidencia positiva: la llegada externa termino `success=true`; la asignacion
  automatica solo comenzo despues. Queda validada la frontera comun de
  trayectoria fisica.
- evidencia negativa: una interrupcion fiducial permitio asignar DA antes de
  cerrar AB y el cierre anterior retiro la entrada `ready` nueva. Ademas, una
  captura depth vacia termino `success=false` con motivo incorrecto `ok`.
- correccion: la interrupcion fiducial bloquea nuevas asignaciones hasta su
  cierre y `InspectFacade` conserva el motivo real de cada etapa.
- conclusion: PARCIAL; guarda validada, barrido completo no validado.

## 2026-09-14 - Prueba 738

- prueba Gazebo/F7: interrumpida voluntariamente por el usuario tras 213 s; el
  helper cerro todos los procesos y no hubo presion de memoria.
- evidencia positiva: el diagnostico corregido aparecio como
  `target_capture_failed:exact_frame_not_buffered`.
- evidencia negativa: D2 entro en la cola antes que D1 aunque el escenario solo
  desplazaba D1. `ready_drones_` se consume solo por su frente, por lo que D2
  bloqueo el progreso de D1. No aparecieron plan D*, trayectoria de fachada ni
  incremento de coverage antes de la parada.
- conclusion: PARCIAL por interrupcion y bloqueo de planificacion entre drones.
- siguiente paso recomendado: arbitraje independiente por dron, retencion del
  frame exacto hasta `CaptureDepth`, reduccion del ruido
  `invalid_stereo_receipt` y nueva prueba D1 realmente aislada hasta validar
  inspeccion, FREE, D*, reserva, movimiento, coverage y `TO_FINISH`.

## 2026-09-14 - Prueba 739

- objetivo ejecutado: aislar D1, llegar con GT al fiducial 2 y dejar activo el
  runtime automatico de fachada. La prueba fue interrumpida voluntariamente por
  el usuario y la limpieza cerro todos los procesos.
- correccion de criterio conversada: el usuario si quiere validar el barrido
  lateral completo. La confusion procedia del nombre: interpreto como barrido
  el giro yaw observado al anclarse. Ese giro no era D*, sino la ida y
  restauracion angular de `InspectFacade`, y no debe comportarse de forma
  erratica ni hacer perder ORB antes de comenzar el desplazamiento lateral.
- evidencia objetiva: no hubo plan D*, reserva, trayectoria de fachada ni
  coverage. La primera inspeccion termino sin depth en la mirada al objetivo;
  la segunda obtuvo 13 puntos en fachada y 41 en el objetivo, pero el prefijo
  FREE resulto menor que el minimo. Despues, las capturas devolvieron cero
  puntos sobre el ultimo frame util y el servidor reasigno inmediatamente las
  tareas `TO_FINISH`, creando un bucle de inspecciones fallidas.
- riesgo visual: no aparecio ningun marcador `F6L-FIRST-EMPTY-REGION`,
  `F6L-TRACKING-RISK` ni `F6L-LOCAL-REORIENT`. ORB paso de estado `OK` a
  `RECENTLY_LOST` durante la restauracion de la segunda inspeccion. El runtime
  desactiva actualmente `inspection_target_look_active_` antes de restaurar la
  orientacion, por lo que esa parte de la maniobra queda fuera de la vigilancia
  de `TRACKING_RISK`.
- conclusion corregida: NO CONSEGUIDA para la validacion funcional pedida. El
  movimiento observado fue la ida y restauracion angular de `InspectFacade`,
  no un barrido D*; quedan por acordar la estabilizacion de yaw/direccion, la
  vigilancia durante toda la maniobra y la politica ante fallos reiterados.

## 2026-09-14 - Pruebas 741--743

- objetivo: estabilizar la inspeccion angular previa al barrido y validar el
  protocolo de riesgo visual;
- 742 identifico un defecto mecanico de wrap yaw en `gen_tray`; el objetivo
  corto de unos `+91 grados` se convertia en un arco largo de unos
  `-269 grados`. La correccion `NearestEquivalentYaw` compila y pasa CTest 8/8;
- 743 confirma que el arco largo desaparece, pero no valida la recuperacion:
  riesgo cancela el target, STOP termina, se lanza automaticamente `restore`,
  aparece otro riesgo y STOP, y luego se ejecuta `restore_local_reorient`;
- evidencia temporal 743: primer riesgo a `1789413598.478`,
  `RECENTLY_LOST` a `1789413599.680`, restore a `1789413603.544`, segundo
  riesgo a `1789413603.645` y correccion local de 25 grados a
  `1789413608.757`;
- conclusion: NO CONSEGUIDA. La causa del giro posterior al STOP es la maquina
  de estados de restauracion, no la reanudacion de la trayectoria cancelada.
  El siguiente cambio funcional queda pendiente de acuerdo.

## 2026-09-15 - Prueba 744: bucle por segunda captura depth

- objetivo: repetir la inspeccion con `5 deg/s` y observar captura, integracion
  FREE y progreso hacia el corredor;
- resultado terminal: `success=true`; el usuario confirma vuelo estable, pero
  observa reintentos y solo FREE procedente de la primera captura;
- evidencia: en tres inspecciones AB, las primeras capturas tienen 128 puntos y
  confianza suficiente; las segundas tienen 1, 0 y 1 puntos, con confianza
  0.018, 0 y 0.016. La mayoria de candidatos se rechaza por textura;
- causa de integracion parcial: `task_server` encola la primera observacion antes
  de validar la segunda. La segunda incumple `depth_min_confidence=0.25`, pero
  la primera permanece y se integra. Dos respuestas del dron eran
  `success=true, reason=ok`, por lo que el log incongruente muestra
  `INSPECTION-FAILED reason=ok`;
- efecto: no se libera el corredor hacia el destino, se repite la inspeccion y
  tras tres fallos la tarea pasa a `TO_FINISH`. Los riesgos posteriores pueden
  fallar tambien por `exact_frame_not_buffered`;
- conclusion: NO CONSEGUIDA para integración depth y avance. Se requiere validar
  ambas capturas antes de encolar cualquiera y compartir el mismo criterio de
  calidad entre productor y consumidor.

## 2026-09-15 - Implementacion de fallback y prueba 745

- cambios: `CaptureDepth` acepta fallback sobre los tres candidatos mas
  recientes con al menos 20 inliers y confianza minima 0.25; `task_server`
  valida las dos observaciones de forma atomica; tras riesgo queda una sola
  correccion local, sin restauracion encadenada;
- build: correctos ambas replicas `mission_msgs`, `orbslam3`, `task_manager`,
  `task_server` y `simulacion_dron`. CTest `orbslam3` 4/4 y `task_server` 7/7;
- evidencia 745: la captura inicial de fachada fue valida. El fallback termino
  seleccionando una captura de un solo punto con confianza 1.0, porque la
  confianza era una razon relativa y no imponia soporte absoluto. ORB entro en
  `RECENTLY_LOST`, cambio de epoch y no pudo aplicarse la pareja;
- conclusion: NO CONSEGUIDA. La atomicidad evita la integracion parcial de 744,
  pero la calidad del fallback y la perdida de tracking impiden el corredor.

## 2026-09-15 - Prueba 746: confianza repetida sobre un frame congelado

- objetivo: repetir 745 cambiando exclusivamente la fraccion de riesgo visual
  de 0.75 a 0.65;
- la captura inicial de fachada fue valida: frame 906, 128 puntos y confianza
  0.600. Tras el riesgo, los candidatos frames 1328/1293/1261 obtuvieron
  12/30/41 puntos y confianza 0.049/0.121/0.167, todos rechazados;
- despues se reutilizo 485 veces el frame 1367: 68 candidatos, 64 descartes por
  textura, cuatro puntos aceptados y confianza 0.059;
- `System::TrackStereo` solo incluye imagen efectiva y camara valida en el
  recibo cuando crea un KF. Por eso `StoreStereoFrame` informo periodicamente
  `invalid_stereo_receipt` y no incorporo frames intermedios aunque tracking se
  mantuvo en estado 2;
- el worker reintento aproximadamente cada 250 ms. Tras tres fallos puso la
  tarea en `TO_FINISH`, la libero y la reasigno inmediatamente, repitiendo la
  captura sin evidencia nueva;
- no hubo pareja atomica aceptada, integracion FREE, D* ni movimiento de
  fachada. La prueba fue detenida por el usuario;
- conclusion: NO CONSEGUIDA para 6N. No debe resolverse aceptando sin mas una
  captura de cuatro puntos. La solucion propuesta es almacenar en el wrapper
  el estereo rectificado de cada frame cualificado, sin exigir que sea KF ni
  tocar ORB-SLAM3, y acordar frescura/soporte y cooldown de reintento.

## 2026-09-15 - Prueba 751: lifecycle de profundidad y perdida durante la mirada

- objetivo: validar con D1/GT al fiducial 2 que una pareja depth conserva el
  mismo candidato hasta integrar ambas fuentes antes de recalcular el corredor;
- preflight y ejecucion: sin ROS, Gazebo ni GUI heredados; una GUI F7, Gazebo y
  D1 solamente. `task_server` compilo y CTest paso 7/7 antes de la prueba;
- evidencia: el primer candidato fue `(-0.50,-14.00,1.20)` con ratio 0.479.
  La puerta de normal depth rechazo la normal por confianza 0.591 y mantuvo yaw
  de fachada 90.002 grados. El giro observado no salio de esa normal, sino de
  `TargetOrientation`, que apunta temporalmente al candidato situado detras de
  la pose del dron;
- riesgo: no aparecieron `F6L-FIRST-EMPTY-REGION` ni STOP/reorientacion. ORB
  paso de tracking 2 a 3 y despues reinicio epoch. El detector vigente exige
  tres frames consecutivos con cero inliers en el semiplano esperado; perder
  tracking no es actualmente una activacion alternativa;
- lifecycle: no hubo respuesta `InspectFacade` valida ni pareja encolada antes
  de la perdida, por lo que tampoco pudo observarse
  `WAITING_DEPTH_INTEGRATION -> RECHECKING_SAME_TARGET`. La prueba no refuta la
  maquina de estados, pero no la valida;
- conclusion: PARCIAL. La espera de mismo objetivo esta compilada y testeada
  unitariamente por CTest general, pero la politica de mirar un destino trasero
  y el criterio preventivo de `TRACKING_RISK` requieren acuerdo antes de otra
  prueba integrada.

## 2026-09-15 - Prueba 758: avance incremental hasta el primer UNKNOWN

- ejecucion: D1/GT al fiducial 2, una GUI F7 y Gazebo, sin D2 ni RViz. La
  ejecucion termino con `SCENARIO-RUNNER-DONE success=true` y
  `SIM-EXIT-CODE=0`, sin procesos residuales;
- evidencia inicial: el servidor eligio el voxel visual `(1.38,-6.62,1.12)`
  con score `0.462` y la pose fisica `(1.75,-9.18,1.20)`. Espero las dos
  capturas, integro depth y reevalua el mismo objetivo;
- avance: tras la reevaluacion, D* identifico el primer hueco
  `UNKNOWN=(0.62,-9.88,0.88)`, activo el plan
  `facade_incremental_unknown`, ejecuto `dstar_1_1` y lo termino con exito.
  A continuacion inspecciono ese hueco como `probe=true`, integro depth y
  emitio `F6N-UNKNOWN-PROBE-RESOLVED`, retornando al selector de objetivo;
- hallazgo: despues de resolver el hueco, el selector produjo solicitudes
  `INSPECTING probe=false` sucesivas cada unos 250 ms sin llegar a consolidar
  una nueva inspeccion o trayectoria antes de terminar el escenario. Es un
  bucle posterior independiente del veto antiguo de prefijo FREE;
- conclusion: PARCIAL. Queda validada la eliminacion del veto de recta FREE,
  la navegacion por UNKNOWN con coste y el ciclo aproximar-inspeccionar-volver
  a seleccionar. Falta diagnosticar y estabilizar el lifecycle de la siguiente
  seleccion para evitar solicitudes repetidas.

## 2026-09-15 - Prueba 759: riesgo visual al 60 %

- ejecucion: D1/GT, una GUI F7 y Gazebo; el usuario detuvo la observacion al
  perder ORB. El launch recibio explicitamente
  `phase6_visual_risk_empty_region_fraction:=0.60`;
- evidencia: durante `InspectFacade` se emitio
  `F6N-INSPECTION-ORIENTATION` con un giro solicitado de `108.039 deg` yaw y
  `46.190 deg` pitch, duracion `32.412 s`. A los pocos segundos el detector
  marco `F6L-FIRST-POOR-REGION` con `fraction=0.60`, `max_inliers=3`, mostro
  el debug visual y solicito `F6N-INSPECTION-RISK-STOP`;
- perdida: unos 2.2 s despues ORB paso a `tracking_state=3`. No existe antes
  de ese instante un marcador de STOP completado ni `F6I-STOP-HOVER`, aunque
  la configuracion del controlador establece `stop_duration_sec=5.0`;
- causa angular concreta: el `probe=true` pide inspeccionar el centro del
  primer voxel UNKNOWN despues de que D* ya haya llevado al dron a ese mismo
  centro. `TargetOrientation` calcula yaw/pitch con el vector
  `target_world - pose_actual`; al ser casi nulo, diferencias de centimetros
  producen valores arbitrarios. En esta prueba se convirtieron en `108.039 deg`
  de yaw y `46.190 deg` de pitch. No proceden de la normal depth ni de la
  fachada;
- conclusion: NO CONSEGUIDA como prueba de recuperacion. El umbral 0.60 si
  detecta y solicita el protocolo antes de LOST, pero la inspeccion ya habia
  ordenado apartar la camara de la fachada con un giro grande y no hay evidencia
  de que el STOP de retencion sustituyera fisicamente esa orden antes de perder
  tracking. Debe revisarse el relevo atomico orientacion -> STOP y evitar que
  una inspeccion solicite una orientacion extrema sin proteccion efectiva.

## 2026-09-16 - Prueba 769: prioridad depth por subtarea con dos drones

- objetivo: comprobar que una pareja depth pendiente de D2 no queda detenida
  por el recorrido de fuentes antiguas de D1, manteniendo la evidencia voxel
  global y reversible;
- ejecucion valida: preflight limpio, una GUI F7 y Gazebo. D1 llego por GT a
  `(0,-10,1)` y D2 a `(0,-10,1.3)`, ambos con yaw `+90 deg`. El coverage se
  habilito solo despues de que ambos alcanzasen el fiducial 2. El escenario
  termino con `SIM-SCENARIO-EXIT-CODE=0` y `SIM-EXIT-CODE=0`;
- evidencia de separacion: D1 encolo la pareja `1:0:33/1:0:34`, que se
  integro y completo. Mas tarde, tras completar otra pareja de D1, D2 encolo
  `2:0:23/2:0:35`; la cola solicito esas dos poses identificadas como
  `drone=2`, integro ambas y emitio
  `F6N-DEPTH-TASK-COMPLETE task=map_section_level_0_DA`. Ya no se bloquea
  reintentando `depth_keyframe_evidence_.begin()` de D1;
- alcance temporal: D1 volvio a encolar una pareja final y D2 otra al cierre
  de los 180 s; no completaron porque el escenario termino durante sus
  solicitudes de pose, no porque se mezclasen identidades. Los reintentos de
  pose de una pareja pendiente siguen siendo numerosos y son una oportunidad
  de optimizacion, no un fallo de separacion funcional;
- conclusion: CONSEGUIDA para la prioridad FIFO por pareja
  `(task_id,drone_id,facade,target)`, la solicitud de KFs propios y la
  liberacion de D2 tras completar su pareja. 6N permanece PARCIAL por sus
  criterios de depth y cobertura mas amplios.
  La correccion funcional a debatir es que un objetivo visual demasiado cercano
  no active `TargetOrientation`: debe conservar la fachada o inspeccionarse
  desde una pose vecina al UNKNOWN, no desde su propio centro.

## 2026-09-21 - Gate efectivo de evidencia sparse F6N

- objetivo: garantizar que `mission_mode=trajectory` o
  `launch_phase6=false` detengan por completo la construccion y publicacion de
  `KeyframeSparseEvidenceDelta`, sin cambiar aun su implementacion interna;
- implementacion: `multi_dron.launch.py` propaga la Fase 6 efectiva al servidor
  global; este no crea el publisher F6N ni llama
  `BuildKeyframeSparseEvidenceDelta()` cuando el gate esta apagado. El gate de
  mascara fisica y el protocolo de perdida ORB quedan independientes;
- build y tests: los cuatro paquetes afectados compilan. El contrato del
  servidor F6N pasa dentro de CTest 13/13 y el contrato de propagacion del
  launch pasa en `mission_flow_contract`;
- prueba integrada valida: `c5_5_3_two_drones_gates_off_v2`, dos drones GT,
  modo trayectoria, Gazebo y GUI global. Escenario y seis goals terminan con
  exito;
- evidencia: `[GLOBAL-FEATURE-GATES] phase6_sparse_evidence=false
  body_mask=false`; cero deltas F6N y cero registros de mascara. La cola
  primaria alcanza solo un pending, frente a 35 cuando F6N se construia de
  forma sincrona aun estando apagada;
- conclusion: CONSEGUIDA para el gate de despliegue. 6N permanece PARCIAL
  porque la invalidacion por revisiones estadisticas, expansion repetida de
  observadores, consultas por MP y construccion sincrona con F6N habilitado no
  se han modificado ni validado;
- siguiente paso recomendado: medir y corregir esos costes solo en una prueba
  autonoma de Fase 6 con F6N habilitado y temporizacion propia.
