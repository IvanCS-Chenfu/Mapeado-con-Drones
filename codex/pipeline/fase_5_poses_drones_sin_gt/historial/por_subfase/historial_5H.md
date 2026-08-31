# Historial 5H - Integracion final de control

## 2026-08-27 - Intentos 237 a 241

- 237 completo con fallback dominante, pero no validaba la conmutacion real;
- 238 y 239: `NO CONSEGUIDA`, los drones divergieron al entrar ORB durante un
  goal; se corrigieron primero el frame fallback y despues la cualificacion;
- 240: recorrio hasta el ultimo tramo, pero un hueco `global_valid=false` del
  mismo epoch rechazo un absoluto; se autorizo usar C_T_W cacheada del epoch;
- 241: interrumpida tras observacion humana; RViz2 mejoro, Gazebo se bloqueo y
  persistieron movimientos raros. El analisis confirmo conmutaciones dentro de
  goals y que el visualizador usaba `w_t_body`, no la entrada del controlador.

## 2026-08-27 - Source lock y pruebas 242/243

- objetivo: congelar fuente por goal y mostrar `o_t_body` exacta en RViz2;
- cambios: `GoalSourceLock`, cualificacion tracking+anchor sin error GT, servicio
  y handshake de frontera, frame absoluto por epoch, visualizador `[ORB]/[GT]`;
- builds: `dron_individual` y `simulacion_dron` correctos;
- tests: navegación 2/2 y visualizador 1/1 correctos con overlays;
- prueba 242: `NO CONSEGUIDA`; lock correcto, pero el paso 6 recibio la primera
  muestra ORB con velocidad invalida y rechazo el goal;
- correccion: esperar una muestra nueva local, continua y con velocidad valida;
- prueba 243: `success=true`, exit 0, 17/17 pasos, 22/22 goals, 44/44 handshakes,
  11 locks/unlocks por dron y cero `GT -> ORB` dentro de lock GT. Se observaron
  12 `ORB -> GT` por perdida dentro de goals, comportamiento permitido;
- recursos: guarda inactiva, minimo MemAvailable 5572.5 MiB;
- evidencia ajena: dos hard failures F3L del optimizador, sin corregir aqui;
- conclusion: `PARCIAL`, tecnica conseguida y pendiente de revision visual de
  Gazebo/RViz2 por el usuario.

## 2026-08-27 - Prueba visual 246

- 244/245 no iniciaron la trayectoria por una ruta YAML relativa incorrecta;
- 246 se ejecuto con Gazebo/RViz2 y fue interrumpida por el usuario al observar
  que el dron se vuelve loco en `GT -> ORB`;
- el log confirma que el mux libera la fuente al terminar un goal individual y
  conmuta a ORB antes del siguiente goal;
- `control_calcular_fuerzas` conserva el ultimo `x_des` y mantiene
  `feedback_activado=true`, por lo que durante la frontera compara una consigna
  expresada en GT con una pose `O_T_B` de ORB;
- ambos drones pierden tracking aproximadamente 1.5 s despues de la primera
  conmutacion observada y regresan a fallback;
- conclusion: `NO CONSEGUIDA` visualmente. El lock debe abarcar la mision YAML
  completa, no cada waypoint/action goal.

## 2026-08-27 - Reset sincronizado y prueba 247

- cambio temporal: al variar `pose_source`, el controlador sustituye la
  consigna retenida por hold en la nueva pose y limpia feedforward/derivadas;
- build `dron_individual` correcto; GTest seleccionados 2/2. La suite global
  conserva tres fallos legacy de lint ajenos y el archivo tocado pasa estilo;
- prueba 247 interrumpida por el usuario: persisten saltos malos;
- el marcador confirma que el hold ocurre tras cada cambio de fuente;
- primera transicion: sucede durante una espera sin goal nuevo; ORB pasa a
  cambios de `0.032 m / 0.145 rad` y ambos drones pierden tracking en 1.2-1.6 s;
- otra transicion produce `0.048 m / 0.063 rad` y perdida en unos 1.2 s;
- interpretacion revisada con el usuario: no atribuir el fallo a ORB ni a las
  ganancias. El reset solo hizo cero el error de posicion; fijo
  `x_dot_des=0`, de modo que el primer error de velocidad ORB no era cero.
  Ademas la fuente cambio durante la espera, separada del siguiente goal;
- conclusion: `NO CONSEGUIDA`. La conmutacion debe ser atomica con el inicio de
  la nueva trayectoria: mantener GT entre goals, capturar pose y velocidad ORB,
  generar `x0/v0` con esa misma muestra y activar ORB junto al primer setpoint.

## 2026-08-28 - Conmutacion atomica y prueba 248

- cambios: la fuente se retiene entre goals; el siguiente goal abre la frontera,
  espera una muestra consumible, vuelve a bloquear y genera el primer setpoint
  con pose, velocidad, yaw y yaw rate de esa misma muestra. El hold copia tambien
  velocidad y yaw rate;
- build: `dron_individual` 1/1 correcto; GTest seleccionados 2/2 correctos;
- prueba: trayectoria tipica con Gazebo y RViz2, interrumpida por el usuario al
  observar un nuevo fallo de movimiento;
- evidencia positiva: antes del cambio ambos drones tienen autoridad global del
  epoch 0 (`global_valid=true`, revision 2). Los arranques ORB usan muestras
  coherentes y velocidades pequenas: dron 1 `0.0064 m/s`, dron 2 `0.0049 m/s`;
- evidencia negativa: ambos pierden tracking 1.56-1.70 s despues de entrar ORB.
  En el siguiente goal sus poses GT ya son fisicamente anomalas, incluido dron 2
  a `z=11.37 m`. El fallo tardio del paso 16 procede de la interrupcion manual;
- interpretacion revisada tras 249: se descartan una `C_T_W` identity heredada,
  la falta de autoridad y un error inicial `ep/ev`, pero 248 no permitia validar
  la rotacion interna de `C_T_W`. El handoff angular posterior descarta tambien
  un impulso inicial de `er/ew`; la causa mas probable pasa a ser una semantica
  de orientacion incompatible entre `O_T_B` y `W_T_B` al convertir el objetivo
  absoluto world a control;
- conclusion: `NO CONSEGUIDA` visualmente; 5H permanece `PARCIAL`. Antes de otra
  correccion se necesita una observacion temporal minima de `er`, `ew`, fuerza y
  torque alrededor del primer `GT -> ORB`.

## 2026-08-28 - Handoff angular y prueba 249

- cambio: primer ciclo `GT_FALLBACK -> ORB` con `R_des=R_act` y
  `Omega_des=w_b`, seguido de transicion SO(3) de `0.5 s`;
- build: `dron_individual` 1/1 correcto; GTest 2/2 y estilo del archivo tocado
  correctos;
- prueba: trayectoria tipica con Gazebo/RViz2, interrumpida manualmente por el
  usuario tras 274 s; cleanup correcto y guarda de recursos inactiva;
- evidencia: el handoff comienza con `er=ew=0`, fuerza de hover
  `13.68-13.79 N` y torque cero. A mitad/final conserva errores menores de
  `0.035` y torque menor de `0.0033 Nm`; no existe impulso angular brusco;
- ambos drones pierden tracking 1.7-2.2 s despues. Al terminar ese goal, dron 2
  aparece a `z=11.37 m` tras partir de `z=1.31 m`, mientras el objetivo world
  solo cambiaba X de 0 a +10 m. Dron 1, cuyo cambio X era -10 m, termina contra
  el suelo. Es la firma de una rotacion world->control que proyecta X sobre Z;
- conclusion revisada: `NO CONSEGUIDA` para el movimiento. El problema actual
  mas probable no es el handoff ni las ganancias, sino la rotacion de `C_T_W`:
  `O_T_B` y `W_T_B` no parecen compartir la misma convencion de frame corporal
  u orientacion. Falta registrar el objetivo absoluto ya transformado y ambas
  orientaciones para confirmar cual de las dos interfaces viola el contrato.

## 2026-08-28 - Diagnostico de frames, pruebas 250/251

- instrumentacion pura: `gen_tray` registra `O_T_B`, `W_T_B`, `C_T_W`, target
  world/control y ejes; el wrapper registra `O_T_C`, `W_T_C`, `B_T_C` y las
  salidas `O_T_B/W_T_B`. No se modificaron formulas ni control;
- builds: `dron_individual` 1/1 y `orbslam3` 1/1 correctos; GTest de navegacion
  2/2 y `test_navigation_state_estimator` 1/1 correctos;
- 250 y 251 terminan con exit 1 esperado porque el YAML diagnostico cancela el
  primer goal ORB al segundo. Ambas duran 91 s, cierran limpiamente y no son
  validaciones funcionales de la vuelta;
- evidencia downstream 251: X world se expresa como
  `(0.0047,-0.0050,0.99998)` y `(0.0029,-0.0066,0.99997)` en control. Por ello
  `-10/+10 m` en X se convierten en `-8.85/+11.41 m` de Z;
- evidencia upstream: con cuerpo en yaw 90 grados, `W_T_C` vale aproximadamente
  `q=(-0.707,0,0,0.707)`, orientacion correcta para la camara optica frontal.
  El YAML entrega como `B_T_C` `q=(0.5,-0.5,0.5,0.5)`, que es la rotacion
  inversa `C_T_B`. El wrapper vuelve a invertirla en
  `W_T_B=W_T_C*inverse(B_T_C)` y obtiene `q=(0.707,0,0.707,0)`, permutando X
  world sobre Z control;
- la traslacion del YAML sigue siendo la posicion de camara expresada en body,
  por lo que el SE(3) configurado tampoco es un `C_T_B` coherente completo. El
  parametro `use_camera_optical_frame_convention=true` no tiene consumidor;
- la ruta fiducial no introduce la rotacion: PnP, `FaceTransform` y
  `world_T_camera` respetan el convenio optico. La pose local aparentemente
  correcta en RViz2 se explica porque el mux alinea el `O_T_B` de ORB con GT al
  cambiar fuente, ocultando la extrinseca erronea; `W_T_B` conserva el error y
  contamina `C_T_W`;
- conclusion: diagnostico `CONSEGUIDO`, movimiento 5H aun `NO CONSEGUIDO` y
  estado agregado `PARCIAL`. La correccion minima pendiente es hacer que la
  calibracion sea un `B_T_C` real, con rotacion optica
  `RPY=(-90,0,-90)` / `q=(-0.5,0.5,-0.5,0.5)`, manteniendo la formula del
  wrapper. Requiere nueva autorizacion funcional y regresion completa.

## 2026-08-28 - Correccion extrinseca y prueba 252

- objetivo: aplicar el `B_T_C` optico correcto y repetir completa la trayectoria
  tipica con Gazebo y RViz2 visibles;
- cambios: las tres copias de calibracion usan `RPY=(-90,0,-90)`; formulas,
  ganancias, optimizador y politica de fuentes permanecen intactos;
- builds: `dron_individual`, `orbslam3_server` y `simulacion_dron` correctos;
  contrato `global_map_config_contract` 1/1 correcto;
- prueba: 252 termina `success=true`, exit 0, 17/17 pasos y 22/22 goals en
  505 s; guarda de recursos inactiva. El exit 255 de Gazebo es posterior a
  `SIM-DONE` y no afecta la ejecucion;
- revision visual: el fallo de ejes/suelo/subida desaparece y el movimiento es
  mucho mejor, pero ambos drones avanzan a tirones y presentan maniobras
  puntuales alocadas, sobre todo cerca de giros;
- causa de los tirones: estado ORB a 20 Hz, control a 50 Hz y velocidad lineal
  y angular por diferencia finita sin filtro ni rechazo de outliers. Se miden
  pasos ORB de hasta `0.207806 m / 0.042847 rad` en drone1 y rotaciones de hasta
  `0.064863 rad`; el PD amplifica los picos de velocidad derivados;
- causa de los episodios alocados: 8/14 entradas `RECENTLY_LOST` y 3/3 `LOST`
  para drone1/drone2, varias durante los pasos 6, 8 y 14. El cambio
  `ORB -> GT_FALLBACK` adopta GT mediante `ResetToSource` dentro del goal; el
  hold se aplica, pero el siguiente feedback de 30 Hz restaura la trayectoria
  congelada en el marco ORB anterior;
- evidencia adicional: algunos goals arrancan copiando derivadas ORB espurias,
  como drone2 `v0=(0.178,0.495,-0.983) m/s` en el paso 6 y drone1
  `v0=(-0.648,-0.794,0.465) m/s` en el paso 14. En el handoff del paso 6 el
  drone2 alcanza `er_norm=0.721` y `ew_norm=0.499` a los 0.5 s;
- fuentes: en el registro completo GT fallback representa el 68.2 % de muestras
  de drone1 y el 76.1 % de drone2; ORB se reactiva solo 4/5 veces. El
  `success=true` del action confirma fin temporal, no llegada fisica al target;
- pose/KFs: los ejes muestran `o_t_body` exacta de control, frecuentemente
  `[GT]`, mientras los KFs se publican en W global y pueden moverse al
  optimizar. Su separacion ocasional es esperable y no demuestra por si sola un
  KF mal colocado;
- conclusion: `PARCIAL`. La extrinseca queda corregida y la vuelta completa,
  pero la suavidad ORB y la transicion de marco durante perdida no alcanzan la
  calidad visual exigida. No se atribuye este movimiento a las optimizaciones
  de Fase 3.

## 2026-08-28 - Predictor 50 Hz y prueba 253 interrumpida

- objetivo intentado: suavizar el estado de control con un predictor alpha-beta
  SE(3) a 50 Hz y mantener continuidad al pasar de ORB a GT fallback;
- cambios ya compilados: predictor de pose/velocidad, innovaciones y velocidades
  limitadas, publicacion a 50 Hz y alineacion fija de GT contra el O activo;
- validacion previa: build `dron_individual` correcto; GTests focales 2/2,
  uncrustify focal 5/5 y `git diff --check` correcto;
- prueba: 253, misma trayectoria tipica, Gazebo y RViz2 visibles. Se interrumpe
  por peticion del usuario a los 103 s durante el paso 5; cleanup correcto,
  runner exit 130 por Ctrl-C y launch `[SIM-EXIT-CODE] 0`;
- evidencia de fuente: ambos drones permanecieron en `GT_FALLBACK` desde el
  arranque. No existe ningun evento `F5H-SOURCE-CONTINUITY`, por lo que el fallo
  observado no coincide con una conmutacion ORB/GT;
- evidencia dinamica: al comenzar el primer goal las velocidades eran casi
  cero, pero el filtro empezo a limitar innovaciones angulares de hasta unos
  3 rad. Antes de la interrupcion habia limitado 1096/1612 medidas de drone1 y
  863/1244 de drone2. En drone1 la innovacion lineal crecio hasta 40.9 m y el
  siguiente goal capturo `x0=(20.739,-47.572,0.774)`;
- diagnostico: el predictor se aplica tambien a la actitud GT exacta. El limite
  angular de 0.08 rad y la correccion alpha introducen retraso de actitud justo
  cuando el dron inclina para acelerar. El lazo de actitud consume esa pose
  retrasada, aplica torque incorrecto y realimenta la divergencia;
- conclusion: `NO CONSEGUIDA`. La prueba 253 invalida esta aplicacion uniforme
  del predictor. No tocar alineacion de marcos, ganancias ni optimizador para
  ocultarlo. La correccion funcional queda pendiente de nuevo acuerdo.

## 2026-08-28 - Predictor solo ORB y prueba 254 interrumpida

- objetivo intentado: validar el predictor a 50 Hz trasladado a
  `orbslam3_ros2`, con GT exacto sin filtro y el mux limitado a seleccionar y
  alinear fuentes;
- cambios: `OrbPosePredictor` filtra traslacion y velocidades ORB, acepta cada
  orientacion medida y la propaga entre frames; `navigation_state_mux` recibe
  pose y velocidad GT exactas. El timer del wrapper es el unico publicador a
  50 Hz;
- validacion previa: builds finales de `orbslam3` y `dron_individual`
  correctos; CTest del estimador 1/1 y CTests del mux/policy 2/2 correctos;
  estilo focal y `git diff --check` limpios;
- prueba: 254, trayectoria tipica con Gazebo y RViz2 visibles. El usuario la
  interrumpe al observar que el inicio es correcto pero los drones divergen al
  girar bajo ORB. Runner exit 130 por Ctrl-C, launch exit 0, guarda de recursos
  inactiva y 13/17 pasos terminados; el paso 14 se cancela por la interrupcion;
- continuidad: los cambios `GT -> ORB` empiezan con salto exactamente cero en
  pose y el handoff angular con `er=ew=0`, por lo que el cambio de frame no es
  la causa primaria;
- evidencia angular: durante los giros aparecen pasos ORB de hasta
  aproximadamente `0.279 rad` en drone1 y `0.273 rad` en drone2 por frame. La
  orientacion medida sustituye completa a la predicha, pero la velocidad
  angular derivada queda limitada a `1.5 rad/s`; pose y velocidad dejan de ser
  dinamicamente coherentes;
- relacion con reference KF: los nueve avisos observados por encima de
  `0.08 rad/frame` no coinciden con el callback de reanclaje. Cada
  `[F5B-REFERENCE-KF]` conserva `step_rotation_rad=0`; los picos aparecen entre
  `0.042` y `0.250 s` despues, ya dentro del reference KF activo. Se concentran
  en periodos de conmutacion rapida de referencias, pero no son el salto
  geometrico directo del cambio de KF;
- consecuencia: el error angular discontinuo excita el controlador antes de
  las perdidas de tracking. Drone2 pierde tracking unos 7.6 s despues de
  entrar en ORB y el siguiente goal GT arranca desde su posicion fisica ya
  divergida `(30.68,-35.84,32.11)`;
- errores: no hay fallo grave de proceso; los unicos `ERROR` reducidos son la
  cancelacion controlada al cerrar;
- conclusion: `NO CONSEGUIDA`. GT queda correctamente aislado, pero la salida
  angular ORB a 50 Hz no es apta para control. Pendiente acordar un limite o
  suavizado de innovacion angular que produzca pose y velocidad coherentes sin
  tocar GT, ganancias, optimizador ni YAML.

## 2026-08-28 - Gate de referencias y prueba 255 interrumpida

- objetivo: poner referencias nuevas en probation y rechazar outliers
  angulares ORB sin introducir retardo en las medidas buenas;
- implementacion: tres frames plausibles confirman candidato, seis frames de
  churn producen timeout, el predictor rechaza innovacion mayor de `0.08 rad`
  o velocidad implicita mayor de `1.5 rad/s`, y tres rechazos consecutivos
  invalidan ORB. GT, controlador, ganancias, ORB-SLAM3 core y YAML no cambian;
- validacion previa: dos builds `orbslam3` correctos; primer CTest 12/13 por un
  flag mecanico de timeout, corregido; segundo CTest 13/13 correcto, estilo
  focal y `git diff --check` limpios;
- prueba: 255 con trayectoria tipica, Gazebo y RViz2. Interrumpida por el
  usuario a los 211 s; runner exit 130, launch exit 0 y guarda de recursos
  inactiva. Completa 7/17 pasos y 8 goals; el paso 8 se cancela al cerrar;
- gate: 108 aceptaciones, cero rechazos por incremento candidato y 10 timeouts
  por churn. Hay siete medidas limitadas y tres invalidaciones por rechazo
  angular consecutivo;
- dinamica: en los pasos 5, 6 y 7 las entradas `GT -> ORB` tienen salto cero.
  Innovaciones de `0.081-0.214 rad` se rechazan antes de publicarse y los
  timeouts/rechazos fuerzan `ORB -> GT` tambien con salto cero. No reaparece la
  pareja incoherente de unos `0.27 rad/frame` publicada en 254;
- limitacion: ORB gobierna pocos segundos por tramo y el fallback es frecuente.
  No se valida una vuelta completa ni calidad ORB sostenida;
- revision visual posterior: el usuario observa que cada entrada en ORB empieza
  bien y despues los drones hacen movimientos alocados, se estrellan o pierden;
- causalidad revisada: no la provoca la optimizacion global. La primera
  degradacion coincide con una optimizacion activa entre `4754.197-4755.710`,
  pero la segunda empieza en `4792.419` y la tercera en `4831.111`, cuando la
  ultima optimizacion habia terminado en `4783.929`. La autoridad global solo
  actualiza W/revision y no modifica la O consumida por el controlador;
- secuencia comun: ORB gobierna entre aproximadamente `1.35-7.10 s`; despues
  aparecen cambios rapidos de reference KF, timeout de probation o innovacion
  angular de `0.081-0.214 rad`, y finalmente perdida real de tracking. Los 10
  timeouts son evidencia directa de churn. Los outliers angulares ocurren con
  `reference_pending=false`, por lo que el churn es precursor fuerte pero no
  explica por si solo toda la degradacion dentro de una referencia activa;
- conclusion revisada: `PARCIAL`. El gate evita el salto grande de 254 y el
  cambio de fuente conserva continuidad, pero la O local ORB se degrada durante
  el control: referencias inestables y medidas angulares anormales preceden la
  perdida. No ajustar el optimizador global para corregir este fallo.

## 2026-08-28 - Estimador SE(3), probation multi-KF y prueba 256

- objetivo intentado: sostener ORB ante churn normal mediante una cadena
  geometrica independiente del ID de reference KF y corregir gradualmente toda
  medida SE(3), con pose y velocidades derivadas del mismo estado;
- cambios: `OrbPosePredictor` limita innovacion, velocidad y aceleracion lineal
  y angular; la pose publicada se integra desde esas velocidades. La probation
  confirma tres incrementos geometricos aunque cambie el ID y usa
  `local_t_camera` solo para enlazar cambios plausibles;
- build y tests: dos builds finales de `orbslam3` correctos. El primer CTest
  quedo 14/15 por redondeo en un limite exacto; tras un epsilon mecanico, el
  CTest final paso 15/15 GTests. Estilo focal y `git diff --check` correctos;
- prueba: 256 con la trayectoria tipica y Gazebo/RViz2 visibles. El scenario
  termino con exit 1: completo los pasos 1-5 y fallo al iniciar el 6 porque el
  goal de drone2 fue rechazado tras perder tracking y abrir epoch 1. El usuario
  observo que el dron colapso al cambiar a ORB;
- handoff: ambos `GT_FALLBACK -> ORB` tienen salto de pose y orientacion
  exactamente cero. El control empieza con `er=ew=0`, fuerza de hover y torque
  cero; al acabar el handoff los errores siguen pequenos. No hay una
  discontinuidad inicial del mux ni una optimizacion global en el intervalo;
- secuencia critica de drone2: entra en ORB en `1787932610.482`, acepta los KF
  25 y 26, y en `1787932614.295` recibe una innovacion angular de
  `0.125261 rad`. El filtro no la rechaza y registra
  `rotation_step_rad=0.119002`, que mide la orientacion raw contra el estado
  filtrado previo, no el salto publicado. Despues acepta rapidamente los KF 28
  y 31 y pierde tracking en
  `1787932615.088`, solo `0.793 s` despues del salto;
- interpretacion revisada: la cronologia demuestra una innovacion moderada no
  rechazada antes de la perdida, pero la telemetria 256 no separa correccion
  aplicada, paso de pose publicado ni omega publicada. Por tanto no demuestra
  que el controlador recibiera `0.119002 rad` ni que esa medida causara el
  colapso. La hipotesis vigente es que el gate instantaneo de `0.35 rad` puede
  aceptar jitter visual sin confirmacion temporal; debe comprobarse midiendo la
  cadena innovacion -> estado publicado -> error/torque -> tracking;
- evidencia secundaria: el goal absoluto se transforma con el `W_T_O` vigente
  sin salto inicial. Sus cotas distintas en O corresponden a la inclinacion de
  ese marco y no demuestran otra causa del colapso. El `W_T_B` de drone1 acaba
  corrompido mas tarde, cuando la ejecucion ya habia fallado fisicamente;
- conclusion: `NO CONSEGUIDA`. La implementacion compila y pasa unitarios, pero
  no es apta para control ORB sostenido. 5H permanece `PARCIAL`;
- siguiente paso recomendado: acordar una probation angular temporal separada:
  innovaciones moderadas se mantienen en cuarentena mientras la salida predice
  brevemente, y solo se incorporan si varias medidas confirman una evolucion
  fisicamente coherente. No tocar GT, mux, ganancias ni optimizador W.

## 2026-08-28 - Probation temporal, pruebas 257 y 258 / etapa 1

- implementacion: innovaciones angulares moderadas quedan pending hasta tres
  confirmaciones coherentes, o cuatro tras un cambio de reference KF; aisladas
  se descartan y persistentes se absorben gradualmente. La telemetria separa
  raw, innovacion, correccion y paso publicado, y el control registra errores y
  torque sin cambiar ganancias;
- build/tests: `orbslam3` y `dron_individual` correctos. El primer CTest paso
  19/21; dos giros exactos en el limite `0.50` fallaron por redondeo float. Tras
  epsilon mecanico, 21/21 GTests correctos;
- prueba 257: intento de infraestructura `NO EJECUTADO`; exit 1 porque la ruta
  relativa del YAML no existia desde el cwd del runner. No hubo maniobras ni
  evidencia funcional;
- prueba 258 / etapa 1: `success=true`, 11/11 pasos y 7/7 goals. GT goberno toda
  la ejecucion (`orb_samples=0`, `fallback_ratio=1.0`) mientras se observaron
  hover, X/Y/Z, yaw lento, yaw rapido y 180 grados;
- tracking: ORB permanecio OK hasta yaw rapido, entro en estado 3 durante ese
  giro y recupero estado 2 al final en epoch 1. El reducido dirigido no contiene
  clasificaciones moderadas ni excesivas, por lo que la perdida no se atribuye
  a la nueva probation. No se ajustan umbrales con una sola ejecucion;
- observabilidad: se detecto que la edad ORB mezclaba timestamp de imagen
  relativo y reloj ROS absoluto. Se separan `measurement_input_stamp` y
  `measurement_receive_stamp`; la edad y ventana detallada usan un mismo reloj.
  Esta reparacion no cambia la salida de control;
- conclusion etapa 1: `CONSEGUIDA` como caracterizacion. El error Gazebo 255
  ocurre unicamente en cleanup posterior a `SIM-DONE`.

## 2026-08-28 - Prueba 259 / etapa 2 hover ORB

- prueba: drone1 llega y se ancla con GT, cambia a ORB con salto SE(3) cero y
  debe mantener hover 25 s. El scenario formal completa 6/6 pasos y 2/2 goals,
  pero el criterio dinamico falla;
- fuente: ORB gobierna solo 227 muestras, unos 4.5 s. Despues conmuta a GT con
  salto cero y tracking entra en estado 3. Las etapas 3-8 no se ejecutan por el
  criterio de parada acordado;
- antes de probation: la oscilacion ya crece con medidas clasificadas como
  pequenas/no relevantes. A `1787947686.450` se publica un paso angular de
  `0.058777 rad`; el control ya registra `er=0.0845`, `ew=0.471` y torque
  `0.0552`, antes del primer pending;
- probation: primer pending en `.657`, descarte y nueva cadena. Se confirma en
  `.805` una innovacion `0.176529 rad`, aplica fraccion 0.70 y llega a pasos
  publicados limitados de `0.075 rad`. Despues aparecen rechazos excesivos de
  `0.359/0.760 rad`, fallback en `.310` y tracking 3 en el segundo siguiente;
- diagnostico: la confirmacion moderada amplifica una inestabilidad previa, no
  la origina por si sola. El limite SMALL crece con `0.5*a*dt^2`, por lo que un
  gap puede convertir una correccion muy superior a `0.015 rad` en pequena.
  Ademas, la direccion persistente del residual puede ser realimentacion, no
  evidencia independiente de movimiento fisico;
- telemetria: la reparacion de relojes queda validada; edades normales
  `0-0.07 s` y ventana detallada activa;
- conclusion: `NO CONSEGUIDA`. Builds y 21/21 GTests no predicen estabilidad de
  hover real. 5H permanece `PARCIAL`; una siguiente iteracion debe debatir
  SMALL fijo y confirmacion basada en incrementos raw/correccion de gauge antes
  de modificar otra vez.

## 2026-08-28 - Prueba 260 / calibracion de movimiento angular raw

- objetivo: instrumentar `dt_raw`, `DeltaR_raw`, `omega_raw` y `alpha_raw` sin
  cambiar la salida, y repetir las maniobras de etapa 1 con GT gobernando;
- validacion previa: build `orbslam3` correcto y 22/22 GTests;
- ejecucion: `success=true`, 11/11 pasos y 7/7 goals; recursos sanos. ORB se
  observa sin gobernar el control;
- drone1: maximos step/omega/alpha de `0.00448/0.0878/0.823` en hover,
  `0.02976/0.2247/4.496` en yaw lento, `0.04693/0.4386/7.728` en yaw rapido y
  `0.09243/0.6206/6.708` en 180 grados. Drone2 quieto llega a
  `0.00221/0.04334/0.65684`;
- tracking: drone1 pierde 2 durante yaw rapido y recupera 2 en epoch 1; no se
  atribuye esa perdida al predictor porque GT gobierna;
- decision calibrada: raw step `0.12 rad`, speed `1.0 rad/s`, alpha
  `10 rad/s2`; dt GOOD `0.075 s`, DEGRADED `0.20 s`;
- conclusion: `CONSEGUIDA` como calibracion. No valida control ORB.

## 2026-08-28 - Redisenio raw/bias y prueba 261 / hover ORB

- implementacion: SMALL fijo; `omega_motion` filtrada desde movimiento raw
  plausible y `omega_bias` desde residual absoluto con limites propios;
  `omega_total` se limita e integra para conservar pose/velocidad coherentes;
- validacion: builds `orbslam3`, `dron_individual` y `simulacion_dron`
  correctos; 27/27 GTests, incluidos hover, offset persistente, raw estable,
  gap de dt y yaw acelerado;
- ejecucion 261: runner exit 0, scenario formal completo, recursos sanos. El
  criterio funcional falla y por acuerdo no se ejecuta etapa 3;
- handoff: `GT_FALLBACK -> ORB` con salto SE(3) y primer error de control cero;
- cronologia: ORB gobierna desde `1787952753.236` hasta `1787952757.053`, unos
  `3.82 s`. El primer pending aparece a los `2.56 s`, el primer confirmed a los
  `2.66 s` y el primer excesivo a los `3.71 s`;
- fallback: tres `REJECTED_EXCESSIVE` hacen no saludable al estimador mientras
  tracking sigue en 2. La perdida 2->3 llega unos `10.25 s` despues, por lo
  que no causa el fallback inicial;
- magnitudes: maximos aproximados `omega_motion=0.629 rad/s`,
  `omega_bias=0.080 rad/s`, `omega_total=0.669 rad/s`, residual `0.699 rad` y
  error angular de control `0.391 rad`;
- diagnostico: el bias actua tambien en SMALL y empieza a oscilar antes de la
  probation. El movimiento fisico que genera el lazo se observa como raw
  plausible y pasa a dominar la salida. Cuando llegan los outliers, retener el
  ultimo `omega_motion` alto mantiene unos `0.56 rad/s` durante los rechazos;
- conclusion: `NO CONSEGUIDA`. La separacion conceptual y los unitarios no
  bastan para estabilidad en lazo cerrado. 5H queda `PARCIAL`; antes de otra
  simulacion debe acordarse como amortiguar raw rechazado y evitar que el bias
  SMALL excite movimiento realimentado, sin tocar GT, mux, ganancias ni W.

## 2026-08-29 - Deadband/decay y prueba 262 / hover ORB

- implementacion: `BiasCorrectionState` `OFF/PENDING/ACTIVE/DECAY`, deadband
  `0.005/0.002 rad`, confirmacion 3 frames y 4 post-reference, supresion del
  bias por movimiento `0.10/0.05 rad/s` y decay raw continuo a `4 rad/s2`;
- validacion: builds `orbslam3`, `dron_individual` y `simulacion_dron`
  correctos; 37/37 GTests, conservando los 27 anteriores y añadiendo diez
  contratos dirigidos;
- ejecucion 262: runner exit 0, scenario formal completo y recursos sanos. Se
  repite exactamente el hover 261 y no se avanza a etapa 3;
- handoff: `GT_FALLBACK -> ORB` con salto de traslacion y rotacion cero; primer
  diagnostico de control `ep=0.001086`, `ev=0.001355`, `er=0.000041` y
  `ew=0.000083`;
- cronologia: ORB gobierna unos `5.92 s`; fallback ocurre por estimador no sano
  y tracking 2->3 llega unos `0.54 s` despues;
- evidencia angular durante ORB: `omega_bias` permanece exactamente en cero
  (`OFF` en 67/69 muestras y `PENDING` en dos), por lo que deadband,
  confirmacion y supresion cumplen su objetivo. Raw rechazado reduce
  `omega_motion` gradualmente hasta cero con el decay configurado;
- magnitudes: maximos aproximados `omega_motion=0.617 rad/s`, residual
  `0.517 rad`, paso publicado `0.0309 rad`, `er=0.267 rad`, `ew=0.604 rad/s` y
  torque `0.0656`. Mejoran respecto a 261, pero no satisfacen hover estable;
- diagnostico: la oscilacion nace en `omega_motion` mientras las medidas raw
  aun son dinamicamente plausibles. El gate no puede descartarlas porque el
  dron ya se mueve fisicamente; al final llegan tres residuos excesivos y se
  activa fallback;
- conclusion: `NO CONSEGUIDA`. Los dos defectos concretos de 261 quedan
  corregidos, pero 5H sigue `PARCIAL`. La siguiente investigacion debe medir
  latencia/fase de `omega_raw -> omega_motion -> control omega -> ew -> torque`
  antes de proponer otro filtro o cambiar umbrales.

## 2026-08-29 - Instrumentacion temporal y prueba 263

- objetivo: confirmar o descartar anti-damping por latencia/fase sin cambiar
  predictor, filtros, thresholds, ganancias, mux, GT ni misión;
- instrumentacion: marcadores por medida/publicación/control con vectores y
  timestamps; GT angular world/body externo; analizador reproducible para
  timeline, correlación, frecuencia/fase, potencia y torque ideal;
- validación: builds `orbslam3`, `dron_individual` y `simulacion_dron`
  correctos; 37/37 GTests, analizador 3/3 y pose metrics 8/8;
- ejecución 263: mismo `f5h_etapa_2_hover_orb.yaml`, runner exit 0,
  `success=true`, 91 s, guard de recursos inactivo y mínimo 4694.6 MiB;
- captura: drone1 genera 443 medidas ORB, 3922 publicaciones y 2740 ticks de
  control, 355 con source ORB. Drone2 no participa en este YAML;
- evidencia ausente: el CSV original guardó GT con stamp Gazebo (`0.x...`),
  mientras receive/publish/control usan reloj ROS epoch (`178799...`). El
  analizador obtuvo cero filas sincronizables y no calculó lags ni potencias;
- conclusión de prueba: `DATOS_INSUFICIENTES`. 263 no confirma ni descarta la
  hipótesis y no debe reinterpretarse como fallo funcional del estimador;
- corrección posterior sin nueva ejecución: GT guarda también
  `gt_receive_stamp`; el analizador usa el par Gazebo/ROS para mapear input y el
  wrapper etiqueta ambos dominios. Rebuilds y tests focales correctos;
- siguiente paso: acordar una nueva ejecución del mismo hover con la captura
  dual-clock. No ejecutar etapa 3 ni modificar comportamiento.

## 2026-08-29 - Prueba 264 / diagnostico dual-clock de fase angular

- objetivo: repetir exactamente el hover de 262/263 con el puente dual-clock
  ya validado y confirmar o descartar latencia/fase y anti-damping;
- cambios funcionales: ninguno. Se conserva predictor, filtros, umbrales,
  ganancias, mux, GT, W y mision; no se ejecuta etapa 3;
- ejecucion: runner exit 0, `success=true`, 92 s, guard de recursos inactivo y
  minimo 4671.4 MiB disponibles;
- captura: 323 ciclos sincronizados con source ORB durante `6.44 s`; tracking
  permanece en 2. Raw pasa a rechazado a `+6.16 s` y fallback llega a
  `+6.46 s`, sin perdida previa de tracking;
- transporte visual: raw frente a GT alcanza correlacion `0.984/0.982` en
  x/y con lag aproximado de `0.08 s`; la edad visual habitual es
  `0.04-0.07 s`. No aparece una latencia de transporte cercana a medio segundo;
- fase de control: en x, el canal angular consumido por control presenta una
  correlacion extrema `-0.836` a `+0.50 s`. `tau_ew` realiza trabajo
  anti-amortiguante en `35.9 %` de los ciclos y se opone al torque ideal en
  `34.0 %`, aunque su energia neta post-handoff es disipativa (`-0.005913 J`);
- termino dominante: reprocesando 264 con el analizador de 265, `tau_er`
  inyecta `+0.005173 J` y realiza trabajo positivo
  en `80.9 %` de los ciclos post-handoff. El error de orientacion supera `0.05`
  a `+4.50 s` y llega a `0.161`, casi anulando el damping de velocidad;
- artefactos: reducido 264 y
  `metricas/prueba_264/angular_phase/{summary.json,drone_1/timeline.csv,drone_1/*.png}`.
  El log completo se conserva y no se lee directamente;
- conclusion diagnostica: `CONSEGUIDA`. Se confirma un desfase angular del
  estado usado por control: el problema dominante es la pose angular estimada
  y su termino proporcional `tau_er`, acompañado por damping intermitentemente
  contrario. No lo explican tracking loss, GT, mux ni una simple latencia de
  publicacion;
- conclusion funcional: `NO CONSEGUIDA`; 5H permanece `PARCIAL`. Antes de otra
  modificacion debe acordarse como corregir la fase de orientacion sin ocultar
  movimiento real ni depender de GT.

## 2026-08-29 - Correccion del horizonte temporal y prueba 265

- objetivo intentado: sustituir el horizonte fijo saturado a `0.10 s` por una
  unica propagacion con la edad visual local real, aislando ese defecto sin
  fusionar directamente la orientacion raw;
- archivos modificados: predictor y wrapper de `orbslam3_ros2`, sus tests y
  `codex/herramientas/analyze_f5h_angular_phase.py` con tests focales;
- implementacion: el ingreso local se captura antes de `TrackStereo`, se
  construye un target en el dominio visual y `Predict` informa horizonte y
  clamp. La telemetria separa `visual_q`, `base_q` y `predicted_q`;
- build: el primer intento de `orbslam3` detecta un argumento fuera de scope y
  se corrige mecanicamente. Builds finales de `orbslam3`, `dron_individual` y
  `simulacion_dron` correctos;
- tests: primer pase 39/40 por una tolerancia de double de 76 ns; tras ajustar
  solo la tolerancia, 40/40 GTests y 5/5 tests del analizador correctos;
- prueba Gazebo: 265 usa el mismo `f5h_etapa_2_hover_orb.yaml`; runner exit 0,
  `success=true`, 93 s y recursos sanos. No se ejecuta etapa 3 ni se dispone de
  una validacion visual humana adicional;
- patrones de reduccion: `SCENARIO|F5H-PHASE|F5H-ORB|F5H-CONTROL|F5B-TRACKING|SOURCE|SIM-DONE|ERROR|FATAL`;
- evidencia positiva: la correccion temporal cumple su contrato. Edad local
  media `51.5 ms`, horizonte medio `43.2 ms`, clamp `10.4 %` y una sola
  extrapolacion; el paso `base_q -> predicted_q` queda acotado;
- evidencia negativa: ORB dura `5.56 s`, raw se rechaza desde `+4.30 s` y
  fallback llega a `+5.58 s` con tracking todavia en 2. Maximos:
  `omega_motion/ew=0.727 rad/s`, `er=0.666 rad` y GT fisico `6.24 rad/s`;
- comparacion causal: `tau_er` pasa de `+0.005173 J` en 264 a
  `+0.160266 J` en 265 y realiza trabajo positivo en `89.16 %`; `tau_ew`
  conserva energia disipativa (`-0.015185 J`), pero el total pasa a
  `+0.145081 J`;
- diagnostico: `visual_q -> base_q` crece de `0.0159 rad` de media antes de
  4 s a `0.1680 rad` despues, con maximo `0.339 rad`. En cambio
  `base_q -> predicted_q` aporta solo `0.0093 rad` medio despues de 4 s. El
  desfase dominante vive en `pose_` base integrada; el antiguo `0.10 s` fijo
  lo compensaba parcialmente de forma accidental;
- conclusion: iteracion funcional `NO CONSEGUIDA`; la correccion temporal se
  conserva porque arregla la semantica del reloj, pero no estabiliza el hover.
  Fase 5H permanece `PARCIAL` y las etapas 3-8 siguen detenidas;
- siguiente paso recomendado: debatir una fusion o anclaje causal de `pose_`
  con la orientacion visual raw, manteniendo una unica propagacion temporal y
  sin GT. No aplicar ese cambio sin un nuevo acuerdo.

## 2026-08-29 - Reanclaje visual de pose base y prueba 266

- objetivo intentado: usar la orientacion visual `O_T_B` aceptada como ancla
  absoluta en `t_visual`, separando correccion de pose y movimiento fisico;
- implementacion: SMALL con raw plausible aplica `SMALL_ANCHOR` completo;
  MODERATE_CONFIRMED corrige geodesicamente hasta `0.015 rad`; pending,
  discarded y rejected avanzan como PREDICT_ONLY. `omega_motion` conserva
  velocidad y extrapolacion, sin recibir la correccion de pose;
- telemetria/analizador: predicted-before, base-after, tipo de update, error
  visual-base before/after, conteos deduplicados, energia por segundo y ventana
  comun configurable;
- validacion: builds `orbslam3`, `dron_individual` y `simulacion_dron`
  correctos; 44/44 GTests y 7/7 tests del analizador. El primer pase GTest fue
  36/44 por precondiciones legacy incompatibles con la nueva separacion; se
  aislaron mecanicamente los contratos de bias/decay sin cambiar produccion;
- prueba: 266 repite exactamente `f5h_etapa_2_hover_orb.yaml`; runner exit 0,
  `success=true`, 92 s y recursos sanos. Gazebo exit 255 ocurre tras el SIGINT
  de cleanup. No se ejecuta etapa 3 ni hay validacion visual humana adicional;
- reduccion: `SCENARIO|F5H-PHASE|F5H-ORB|F5H-CONTROL|F5B-TRACKING|SOURCE|SIM-DONE|ERROR|FATAL`;
- duracion: ORB gobierna `8.06 s` frente a `5.56 s` en 265 (`+45 %`);
  fallback llega a `+8.08 s` y tracking 2->3 a `+8.68 s`, por lo que el
  estimador vuelve a invalidarse antes de perder tracking;
- ventana comun 5.56 s: `tau_er=+0.002067 J` frente a `+0.153559 J`
  (`-98.7 %`); `tau_ew=-0.004013 J`; torque total `-0.001945 J` frente a
  `+0.138374 J`, pasando a disipativo;
- tramo ORB completo 266: `tau_er=+0.036697 J`, `tau_ew=-0.020684 J` y total
  `+0.016013 J`. La energia por segundo de `tau_er` baja ~`84.8 %` y la total
  ~`92.7 %` respecto a 265;
- maximos: `er=0.264 rad` frente a `0.666`, GT omega `2.65 rad/s` frente a
  `6.24`; control/ew queda parecido, `0.739 rad/s` frente a `0.727`;
- anclas deduplicadas: 157 medidas ORB, 118 SMALL_ANCHOR, 15
  MODERATE_CONFIRMED, seis pending, 15 PREDICT_ONLY y tres rejected; update
  aplicado en `84.7 %`. SMALL deja error after exactamente cero;
- fallo tardio: desde `+5.90 s` aparecen ciclos moderate/discarded. La
  correccion confirmada resta `0.015 rad`, pero deja `0.040 rad` medio. El
  residual crece durante PREDICT_ONLY, raw se rechaza a `+7.50 s` y termina en
  fallback;
- conclusion: `NO CONSEGUIDA` funcionalmente, con mejora causal sustancial. El
  reanclaje SMALL valida la hipotesis y se conserva, pero no completa el hover.
  Fase 5H sigue `PARCIAL` y etapa 3 permanece detenida;
- siguiente paso recomendado: debatir la politica moderada para recuperar fase
  mas deprisa (reanclaje completo o correccion mayor/ligera fusion), sin tocar
  gains, GT, mux ni omega hasta medir esa decision.

## 2026-08-29 - Anclaje moderate completo y prueba 267

- objetivo intentado: hacer que una medida `MODERATE_CONFIRMED` fiable reanclase
  completamente `R_base` a la orientacion visual, sin convertir la correccion
  de pose en omega;
- implementacion ejecutada: SMALL conserva anclaje completo; pending,
  discarded y rejected quedan predict-only; confirmed adopta la medida y la
  telemetria informa `MODERATE_CONFIRMED_ANCHOR`;
- validacion previa: builds `orbslam3`, `dron_individual` y
  `simulacion_dron` correctos; 45/45 GTests y 7/7 tests del analizador;
- prueba: 267 repite `f5h_etapa_2_hover_orb.yaml`; runner exit 0,
  `success=true`, 93 s, recursos sanos, minimo 4503.1 MiB y sin observacion
  visual humana adicional;
- resultado funcional: `NO CONSEGUIDA`. ORB dura `5.560105 s`, fallback llega
  a `+5.580026 s` y tracking no OK a `+6.120019 s`; por el criterio acordado
  no se ejecuta 268 ni etapa 3;
- anclas: 106 medidas ORB unicas, 89 SMALL_ANCHOR, cuatro
  MODERATE_CONFIRMED_ANCHOR, seis pending, cinco predict-only y dos rejected.
  Las cuatro confirmadas dejan error visual-base after cero;
- energia en ventana comun: `tau_er=+0.030448 J`,
  `tau_ew=-0.014826 J` y total `+0.015622 J`. Es mejor que 265, pero peor que
  266 (`+0.002067/-0.004013/-0.001945 J`);
- cronologia causal: antes del primer anclaje moderate, 267 acumula solo
  `tau_er=+0.002268 J` y total `-0.001488 J`. Desde ese anclaje hasta fallback
  acumula `+0.029136/+0.018489 J` en `1.22 s`; 266 tarda `2.06 s` en alcanzar
  casi la misma energia total. El anclaje completo acelera el episodio, aunque
  una sola ejecucion no aisla por completo la variabilidad;
- defecto contractual descubierto: una de las cuatro confirmaciones ancla con
  `raw_class=REJECTED`. Confirmacion angular y gate raw eran independientes;
- reparacion posterior no simulada: el anclaje confirmado exige tambien
  `raw_motion_plausible`; se adapta el test de omega y se añade
  `ConfirmedModerateWithRejectedRawRemainsPredictOnly`. Build final correcto y
  46/46 GTests;
- conclusion: 5H permanece `PARCIAL`. 267 no valida el anclaje moderate
  completo como solucion; la version final con gate raw esta comprobada solo
  unitariamente. No repetir ni elegir fusion/ganancia sin nuevo acuerdo.

## 2026-08-29 - Validacion del gate raw final y prueba 268

- objetivo: repetir el hover sin otros cambios y determinar si el anclaje con
  raw rechazado de 267 explicaba el fallo;
- codigo: version final posterior a 267, donde MODERATE_CONFIRMED solo ancla
  con `raw_motion_plausible`; build previo correcto y 46/46 GTests;
- ejecucion: mismo `f5h_etapa_2_hover_orb.yaml`, runner exit 0,
  `success=true`, 92 s, recursos sanos, minimo 4372.7 MiB y sin revision visual
  humana adicional;
- resultado funcional: `NO CONSEGUIDA`. ORB dura `5.720050 s`, fallback llega
  a `+5.740012 s` y tracking no OK a `+5.920002 s`; no se ejecutan 269 ni
  etapa 3;
- updates: 107 medidas ORB unicas, 95 SMALL_ANCHOR, un
  MODERATE_CONFIRMED_ANCHOR, tres pending, seis predict-only y dos rejected;
- gate validado: el unico anclaje moderate usa raw plausible, corrige
  `0.057317 rad` y deja error visual-base after cero. No hay anclajes confirmed
  con raw rechazado;
- ventana comun: `tau_er=+0.039126 J`, `tau_ew=-0.001529 J` y total
  `+0.037597 J`, peores que 267 y 266;
- cronologia causal: durante los `4.84 s` anteriores al anclaje,
  `tau_er=+0.002147 J` y total `-0.001356 J`. Desde el anclaje hasta fallback,
  en solo `0.88 s`, se acumulan `+0.043934/+0.046416 J`; raw empieza a
  rechazarse unos `0.24 s` despues;
- conclusion: el bug raw de 267 no era la causa principal. Un anclaje completo
  inmediato y plausible basta para disparar el crecimiento angular. Esto
  refuerza el diseño condicional de residual `Delta_target` persistente y
  gradual, pero no lo autoriza ni implementa automaticamente. 5H sigue
  `PARCIAL` y la ejecucion se detiene como se acordo.

## 2026-08-29 - Bateria diagnostica GT A/B/C/D, pruebas 269-272

- objetivo: aislar si el fallo angular procede de la geometria ORB o de la
  frecuencia, latencia y jitter del pipeline visual;
- cambios: nodo `gt_timing_diagnostic` que introduce GT perfecto en el
  `OrbPosePredictor` real; fuente fija de laboratorio en el mux; launch
  `f5h_gt_timing_mode`; cuatro YAML equivalentes. Defaults apagados y marcados
  para retirar. No se tocaron gains, thresholds, GT normal ni `Delta_target`;
- builds: `orbslam3`, `dron_individual` y `simulacion_dron`, todos codigo 0;
- 269/A, GT normal 50 Hz: runner 0, escenario completo, GT 100 %. Ultimos 25 s:
  omega GT media `0.000063`, p95 `0.000244`, maxima `0.000370 rad/s`. Estable;
- 270/B, GT perfecto 20 Hz por predictor/publicacion 50 Hz: runner 0 y sin
  fallback, pero queda girando a ~`0.1059 rad/s`. 399 medidas, 1197 estados en
  23.94 s; `tau_er=+0.03893 J`, `tau_ew=-0.02805 J`, total `+0.01088 J`;
- 271/C, B +80 ms: runner 1; completa llegada, pero rechaza el segundo goal.
  Edad media/max `0.111/0.140 s`, clamp `74.5 %`; total `+0.00874 J` en 1.58 s;
- 272/D, traza determinista 268: dos reintentos por muerte temprana de Gazebo;
  el intento 2 completa llegada y rechaza el segundo goal. Edad media/max
  `0.115/0.200 s`, clamp `68.8 %`, `er` max `0.586 rad`, omega control max
  `0.673 rad/s`; total `+0.02532 J` en 1.62 s;
- conclusion: `PARCIAL` para 5H y diagnostico causal `CONSEGUIDO`. GT perfecto
  reproduce el fallo ya en B; latencia y jitter lo agravan. La causa principal
  esta en la semantica 20->50 Hz y la dinamica del predictor/publicacion, no en
  la calidad geometrica ORB. No implementar aun la solucion sin nuevo acuerdo.

## 2026-08-29 - Bateria E/F/G, pruebas 273-275

- objetivo: separar derivacion/filtrado de `omega_motion`, hold angular 20 Hz
  y extrapolacion SO(3), siempre con pose y omega GT perfectas;
- implementacion diagnostica: E usa el predictor actual y sustituye solo
  `omega_motion`; F conserva la ultima orientacion GT; G propaga
  `exp(omega_GT*age)*R_GT`. F/G conservan la rama lineal del predictor y la
  sincronizacion usa la ultima omega no futura. No se tocaron gains, thresholds,
  `Delta_target`, estimator retardado ni etapa 3;
- validacion: build `orbslam3` codigo 0 y CTest 2/2;

| Metrica | E / 273 | F / 274 | G / 275 |
|---|---:|---:|---:|
| Scenario success | si | si | si |
| max `er` rad | 0.0784 | 0.0974 | 0.0960 |
| max `ew` rad/s | 0.1087 | 0.1016 | 0.1226 |
| max `omega_control` rad/s | 0.1087 | 0.0964 | 0.1226 |
| energia `tau_er` J | +0.002225 | +0.003074 | +0.003336 |
| energia `tau_ew` J | -0.002291 | -0.003150 | -0.003430 |
| energia total J | -0.000066 | -0.000076 | -0.000093 |
| mismatch direccion GT/control | 0.41 % | 0.29 % | 0.094 % |
| horizonte medio s | 0.0332 | 0 | 0.0293 |
| oscilacion creciente | no | no | no |

- conclusion: diagnostico `CONSEGUIDO`, opcion A. Sustituir solo la omega
  derivada por omega GT estabiliza E; F descarta que el hold a 20 Hz sea
  insuficiente y G valida la extrapolacion SO(3) con pose/omega coherentes. La
  causa principal es la derivacion/filtrado de `omega_motion`. 5H sigue
  `PARCIAL`; falta diseñar la correccion con medidas ORB reales.

## 2026-08-29 - Estimador causal de omega, pruebas 276-277

- objetivo: sustituir el pasa-bajos de `omega_motion` por estimacion causal de
  extremo y validarla primero con pose GT perfecta a 20 Hz;
- cambios: dos velocidades espaciales
  `Log(R_k R_{k-1}^{-1})/dt`, aceleracion entre midpoints y proyeccion hasta
  `t_k`; hold entre medidas, timestamps reales, historial solo con medidas
  aceptadas, reset por epoch y supresion de inversiones microscopicas. No se
  tocaron gains, anclajes, mux, W, KF policy, GT normal ni `Delta_target`;
- validacion: build `orbslam3` correcto; CTest 2/2, estimador 55/55 GTests y
  analizador 8/8 tests. El intento 0 de 276 fallo antes de goals por YAML
  relativo y se conserva aparte;
- 276: escenario completo, 906 medidas/2717 publicaciones, sin fallback;
  `er` max `0.01368 rad`, RMSE `0.00374 rad/s`, mismatch `4.26 %` sobre 94
  muestras y energia total `-0.00000442 J`;
- 277: repeticion completa, 914 medidas, sin fallback; `er` max `0.01778 rad`,
  RMSE `0.00304 rad/s`, mismatch `0.64 %` sobre 157 muestras y energia total
  `-0.00000510 J`;
- comparacion: 270/B tenia RMSE `0.43338 rad/s` y `+0.010879 J`; 273/E con
  omega GT tenia RMSE `0.00570 rad/s` y `-0.0000659 J`. Las amplitudes GT no
  son identicas, por lo que la energia absoluta no es un ranking directo;
- conclusion: 276 y 277 `CONSEGUIDAS`; la correccion es estable y reproducible
  con pose perfecta a 20 Hz. 5H sigue `PARCIAL` hasta validar delay/jitter y,
  despues, ORB real bajo un nuevo acuerdo.

## 2026-08-29 - Validacion de delay fijo, prueba 278

- objetivo: validar el estimador causal congelado con pose GT perfecta a
  20 Hz, delay fijo de 80 ms, timestamp fisico original y control a 50 Hz;
- cambios: solo escenarios 278-281; la politica acordada obliga a detenerse en
  el primer fallo. No se modifico `OrbPosePredictor`, gains, anclajes ni gates;
- validacion: build `orbslam3` codigo 0, CTest 2/2 y analizador 8/8;
- ejecucion: runner y escenario codigo 1, guard inactiva y minimo 5379 MiB. La
  fuente diagnostica goberno 3.00 s; no hubo fallback ni tracking no-OK;
- evidencia: 50 medidas/151 publicaciones, edad visual media/maxima
  `0.11285/0.14003 s`, horizonte medio `0.09709 s`, clamp `72.19 %`, `er` max
  `0.36829 rad` y RMSE/MAE omega `1.44719/0.99097 rad/s`;
- energia: `tau_er=+0.022108 J`, `tau_ew=+0.006732 J` y total
  `+0.028840 J` en 2.38 s;
- conclusion: 278 `NO CONSEGUIDA`; 279-281 no ejecutadas. Debe debatirse la
  compensacion causal desde `t_k` hasta `now`.

## 2026-08-30 - Clamp y extrapolacion alpha, pruebas 282 y 284

- objetivo: separar el limite de `0.10 s` de una propagacion dinamica
  insuficiente entre `t_k` y `now`;
- 282/278B: solo el nodo GT de laboratorio usa horizonte `0.18 s`; clamp cae
  de `72.19 %` a cero, pero el hover falla. RMSE/MAE `2.46780/1.34977 rad/s`,
  energia total `+0.068966 J` y gobierno `2.82 s`; peor que 278;
- cambio 278C: flag `predict_angular_acceleration=false` por defecto. Solo el
  laboratorio publica pose y omega en el mismo target con velocidad espacial
  world/O y multiplicacion izquierda:
  `omega=omega_k+alpha*dt`,
  `R=Exp(omega_k*dt+0.5*alpha*dt^2)R_k`. Usa la misma dt clamped y limita
  aceleracion/velocidad; alpha se borra tras degradacion, rechazo o reset;
- validacion: build `orbslam3` correcto; CTest 2/2 con cinco GTests nuevos
  (60/60 agregados) y analizador 8/8;
- 284/278C: hover fallido, gobierno `1.86 s`, clamp cero, RMSE/MAE
  `1.13627/0.95386 rad/s`, mismatch `38.46 %` sobre 39 muestras y energia total
  `+0.020524 J` en 1.26 s. Ninguna publicacion alcanzo limites alpha/omega;
- interpretacion: 278C mejora RMSE y energia instantanea frente a 278/282,
  pero falla antes y raw empieza a rechazarse a `0.84 s`. Ni ampliar horizonte
  ni aceleracion constante resuelven el delay; no repetir ni avanzar a 279;
- conclusion: diagnostico `CONSEGUIDO`, solucion funcional `NO CONSEGUIDA`, 5H
  `PARCIAL`. La rama alpha permanece solo diagnostica y apagada en produccion.

## 2026-08-30 - Cruces R/omega con GT actual, pruebas 285-287

- objetivo: separar `R_pred(now)` y `omega_pred(now)` con entrada visual GT a
  20 Hz, delay 80 ms, predictor 284 y salida a 50 Hz;
- cambios: seleccion diagnostica independiente de R/omega; registro de fuentes,
  estados predicho/GT/usado, edad local y skew. Produccion intacta y 279-281
  detenidas;
- validacion: build `orbslam3` codigo 0; GTest directo 63/63, incluidos tres
  tests cruzados; analizador codigo 0 y YAMLs validos. CTest no pudo escribir
  `LastTest.log` por sandbox y el escalado fue rechazado por limite de uso;
- 285, `R_pred+omega_GT`: escenario 1, gobierno `13.1199 s`, RMSE
  `0.6433 rad/s`, mismatch `0.78 %`, energia total `-0.013794 J`;
- 286, `R_GT+omega_pred`: escenario 1, gobierno `2.9800 s`, RMSE
  `1.6436 rad/s`, mismatch `49.02 %`, energia total `+0.044590 J`;
- 287, `R_GT+omega_GT`: escenario 1, gobierno `13.0600 s`, RMSE
  `1.0963 rad/s`, mismatch `0.47 %`, energia total `-0.163509 J`;
- integridad GT: skew pose/omega cero y edad ordinaria `10-20 ms`; una muestra
  inicial de 287 mezcla el salto de reloj ROS, fuera de la ventana analizada;
- interpretacion: 285/287 casi coinciden y son disipativas, mientras 286 falla
  pronto y es inestable. Señala con fuerza a `omega_pred`, pero no lo demuestra:
  el sanity 287 tambien falla porque p/v lineales siguen en el predictor
  retrasado y no constituyen GT completo;
- conclusion: bateria completa, diagnostico `PARCIAL`, solucion funcional
  `NO CONSEGUIDA`. No ejecutar 279-281. El siguiente sanity debe fijar tambien
  `p(now)` y `v(now)` GT en las tres ramas.

## 2026-08-30 - Cruce completo p/v/R/omega, pruebas 288-291

- objetivo: cerrar la ambiguedad de 285-287 fijando `p(now)` y `v(now)` GT en
  las cuatro ramas, con entrada visual a 20 Hz, delay 80 ms y control a 50 Hz;
- cambios: selector diagnostico independiente para p/v/R/omega, telemetria
  predicha/GT/usada, edad y skew; cuatro GTests y YAMLs 288-291. Predictor,
  control, gains, gates y rutas productivas permanecen intactos;
- validacion: build `orbslam3` codigo 0; GTest directo 67/67, analizador
  correcto, YAMLs validos y `git diff --check` limpio;
- 288, `p/v GT + R/omega pred`: escenario 1, gobierno `2.5399 s`, RMSE omega
  `1.8111 rad/s`, mismatch `50.0 %` y energia total `+0.079788 J`;
- 289, `p/v/omega GT + R pred`: escenario completo, gobierno `54.4600 s`,
  RMSE omega `0.003829 rad/s`, mismatch `0.114 %` y energia total
  `-0.0000487 J`;
- 290, `p/v/R GT + omega pred`: escenario 1, gobierno `5.1800 s`, RMSE omega
  `2.0236 rad/s`, mismatch `47.06 %` y energia total `+0.088871 J`;
- 291, estado GT actual completo: escenario completo, gobierno `55.2000 s`,
  RMSE omega `0.001255 rad/s`, mismatch cero y energia total `-0.0000660 J`;
- integridad: las fuentes coinciden al 100 % con cada rama; skew pose/omega
  nulo salvo una muestra de `0.02 s` en 289 y edades GT ordinarias de
  `8-14 ms`;
- conclusion: diagnostico `CONSEGUIDO`. El sanity valida el montaje y ambos
  cruces demuestran que sustituir solo `omega_pred` por `omega_GT` cambia fallo
  por hover completo. La causa inmediata esta en la omega predicha bajo delay,
  no en R ni p/v. 5H sigue `PARCIAL`: falta corregirla sin GT y validar ORB
  real. 279-281 permanecen detenidas.

## 2026-08-30 - Predictor dinamico por torque, prueba 292

- objetivo: sustituir la extrapolacion cinematica de omega por integracion
  causal de dinamica rigida desde `t_k`, usando el torque corporal ordenado y
  la misma `fisico.total.matriz_inercia=diag(1e-4)` del controlador;
- cambios: `BodyTorqueDynamicPredictor` con buffer acotado, timestamps reales,
  Euler semiimplicito y ecuacion
  `J*omega_dot=tau-omega x (J*omega)`; modos diagnosticos 292-295, timestamp en
  `control/tray/torque`, ocho GTests y cuatro YAMLs. Gains, mixer, gates, mux,
  KF y rutas productivas no cambian;
- auditoria: `control/tray/torque` esta expresado en body; el mixer 4x4 no
  satura y el plugin aplica directamente fuerzas y pares, por lo que el torque
  reconstruido coincide algebraicamente con el deseado;
- validacion previa: builds `orbslam3` y `dron_individual` correctos, GTest
  directo 75/75 y analizador correcto;
- prueba 292: runner/escenario codigo 1 tras 63 s, guard inactivo y minimo
  5068.7 MiB. El primer tick de control ordena
  `tau=(0.003779,-0.005317,-0.000006) Nm`; con `J=1e-4`, el predictor alcanza
  `omega=(336.3,296.5,7.0) rad/s` mientras GT mide aproximadamente
  `(-0.96,-0.82,-0.004) rad/s`. La realimentacion genera despues torques de
  decenas y cientos de Nm, omega de miles y finalmente `NaN` en unas decimas;
- interpretacion: la integracion implementa la ecuacion acordada, pero la J
  nominal compartida con control no representa la respuesta fisica compuesta
  de Gazebo. Las metricas globales posteriores al colapso no son una medida
  util del estimador;
- conclusion: 292 `NO CONSEGUIDA`; modelo dinamico `NO VALIDADO`. Por el STOP
  acordado no se repite 292 ni se ejecutan 293-295. 279-281 siguen detenidas y
  5H permanece `PARCIAL` a la espera de revisar J efectiva/torque aplicado.

## 2026-08-30 - J compuesta y validacion dinamica, pruebas 296-298 y 293-295

- cambio: `fisico.total.matriz_inercia` pasa a la inercia compuesta del modelo
  Gazebo de 1.4 kg:
  `diag(0.00803107,0.00803107,0.015805) kg*m^2`. Se calcula con cuerpo, cuatro
  brazos de 0.25 m y cuatro motores respecto al CoM; controlador y predictor
  consumen la misma J y los tensores por enlace no cambian;
- verificacion: calculo independiente reproduce
  `(0.0080310714,0.0080310714,0.015805)`; builds `dron_individual` y `orbslam3`
  codigo 0, GTest 75/75 y analizador correcto;

| Prueba | Rama | Edad visual media s | Gobierno s | RMSE omega rad/s | Mismatch | Energia total J | Resultado |
|---|---|---:|---:|---:|---:|---:|---|
| 296 | 292 con J compuesta | 0.1102 | 54.64 | 0.002554 | 0.129 % | -0.00004094 | CONSEGUIDA |
| 297 | confirmacion 296 | 0.1103 | 55.12 | 0.005565 | 0.777 % | -0.00004833 | CONSEGUIDA |
| 293 | omega causal + dinamica; p/v/R GT | 0.1104 | 54.62 | 0.002565 | 0 % | -0.00006332 | CONSEGUIDA |
| 298 | confirmacion 293 | 0.1100 | 54.72 | 0.003608 | 0 % | -0.00007662 | CONSEGUIDA |
| 294 | p/v GT; R/omega dinamicas | 0.1099 | 54.70 | 0.003385 | 0 % | -0.00005771 | CONSEGUIDA |
| 295 | p/v predichas; R/omega dinamicas | 0.0311 | 54.90 | 0.003588 | 0 % | -0.00022465 | CONSEGUIDA |

- todas completan el escenario sin fallback, tracking no-OK, clamp ni guard de
  recursos. El intento 0 de 293 tuvo muerte temprana de Gazebo y fue reintentado
  automaticamente; el intento 1 es la ejecucion funcional medida;
- correccion conversada posterior: `dynamic_295` no activa
  `UsesCrossDiagnostic()`, por lo que su `DeliveryDelay()` es cero. Sus 31 ms
  medios proceden de muestreo/publicacion, no de los 80 ms artificiales. 294
  valida R/omega dinamicas con unos 110 ms de edad; 295 valida estado completo
  a 20 Hz sin delay añadido;
- conclusion: la J nominal, no la integracion rigida, causaba el colapso de
  292. El predictor angular queda `VALIDADO EN LABORATORIO` con delay fijo y
  el estado completo queda validado sin delay añadido. 5H permanece `PARCIAL`
  porque faltan timing/jitter medido, estado completo bajo ese timing y ORB
  real; 279-281 siguen detenidas hasta un nuevo acuerdo.

## 2026-08-30 - Timing/jitter determinista, prueba 299

- objetivo: validar el estado completo sin GT actual usando la traza
  determinista de periodos y delays medida en 268, antes de integrar el
  predictor en la ruta ORB productiva;
- cambios: modo `dynamic_299`, que combina `TracePeriods/TraceDelays`, p/v
  predichas y R/omega del predictor dinamico; YAML 299 y GTest
  `CompositeInertiaHandlesIrregularOrbTimingTrace`;
- validacion previa: build `orbslam3` correcto tras corregir mecanicamente el
  namespace del nuevo test; GTest 76/76 y suite del analizador correctos;
- ejecucion: runner/escenario codigo 1 tras 62 s; gobierno ORB diagnostico
  `16.94 s`, sin fallback ni tracking no-OK y sin guard de recursos;
- timing: edad visual media `0.11533 s`, maxima `0.20001 s`, horizonte medio
  `0.11500 s` y clamp `4.13 %`; la traza produce 48 intervalos
  `DEGRADED_DT`, principalmente por periodos de `0.12 s`;
- integridad dinamica: no aparece `F5H-DYNAMIC-MISSING`; las 889 predicciones
  observadas declaran `missing=false`, por lo que el buffer de torque cubre la
  ventana irregular;
- resultado angular: RMSE/MAE control-GT `0.34965/0.08101 rad/s`, `er` maximo
  `0.68453 rad`; `tau_er=+0.19480 J`, `tau_ew=-0.18072 J` y energia total
  `+0.01411 J` durante `16.34 s`;
- causalidad: el primer rechazo raw llega a `+15.84 s`, despues del crecimiento
  fisico y energetico. Los diez `REJECTED` finales y los grandes huecos de
  timestamp son consecuencia del colapso, no su disparador inicial;
- conclusion: 299 `NO CONSEGUIDA`. La dinamica y la cobertura de torque siguen
  validadas, pero el estado completo no es estable bajo la combinacion realista
  de periodos largos y delay variable. La evidencia apunta a la coherencia
  temporal durante `DEGRADED_DT`/reanclaje de la base y no justifica tocar J,
  gains, gates o mixer;
- STOP aplicado: 300 no ejecutada; no se integra la ruta ORB productiva y
  301-302 no se ejecutan. Fase 5H permanece `PARCIAL`.

## 2026-08-30 - Cruce jitter p/v y angular, pruebas 303-306

- objetivo: separar si el fallo 299 nace en p/v predichas, en el estado
  angular inicial durante `DEGRADED_DT` o en ambos, sin recalibrar arquitectura;
- montaje: las cuatro pruebas reutilizan `TracePeriods/TraceDelays` de 299.
  303 usa p/v GT(now)+angular dinamica; 304 p/v GT(now)+R/omega GT interpoladas
  en `t_k` y dinamica hasta now; 305 p/v predichas+angular GT(now); 306 GT(now)
  completo. Build correcto, GTest 77/77 y analizador correcto;

| Prueba | p/v | Angular usada | Gobierno | RMSE omega | Energia total | Resultado |
|---|---|---|---:|---:|---:|---|
| 299 | pred | causal+dinamica | 16.94 s | 0.34965 | +0.01411 J | falla |
| 303 | GT(now) | causal+dinamica | 54.54 s | 0.00304 | -0.0000402 J | pasa |
| 304 | GT(now) | GT(t_k)+dinamica | 2.52 s | 3.13892 | -0.11277 J | falla |
| 305 | pred | GT(now) | 15.06 s | 1.86968 | -0.06038 J | falla |
| 306 | GT(now) | GT(now) | 54.60 s | 0.00405 | -0.0000501 J | pasa |

- 303: escenario completo, 758 medidas, 49 `DEGRADED_DT`, edad media/maxima
  `0.1147/0.2000 s`, sin fallback, tracking no-OK ni raw rechazado;
- 304: interpolacion valida con bracket tipico `0.02 s`, residual R inicial
  respecto a GT(t_k) cero y 896 predicciones con torque cubierto. Aun asi,
  colapsa pronto; la omega GT instantanea en `t_k` presenta variacion rapida y
  la propagacion torque/J no reproduce GT(now). Esto invalida 304 como prueba
  positiva del predictor bajo jitter, pero no contradice 303;
- 305: con angular GT(now), p/v predichas bastan para fallar; primer rechazo
  raw a `+13.86 s`, despues de la divergencia. No hubo fallback ni tracking
  no-OK antes del fallo;
- 306: sanity completo correcto, 759 medidas, edad media/maxima
  `0.1144/0.2000 s`, sin fallback, tracking no-OK ni raw rechazado;
- conclusion principal: `PV PRINCIPAL`. Sustituir solo p/v convierte el fallo
  299 en hover completo, mientras conservar p/v predichas hace fallar incluso
  con angular GT actual. El angular causal+dinamico es suficiente bajo jitter
  cuando p/v son correctas;
- conclusion secundaria: `GT(t_k)+dinamica` no queda validado con jitter. Antes
  de usarlo como referencia deben revisarse marco y alineacion temporal de la
  omega GT instantanea y la suficiencia del torque modelado;
- no se ejecutan 300-302 ni se modifica la ruta productiva. 5H sigue `PARCIAL`.

## 2026-08-30 - Predictor translacional, pruebas 307-313

- objetivo: separar p/v y validar propagacion translacional causal con la traza 299;
- 307 (`p_GT+v_pred`) y 308 (`p_pred+v_GT`), ambas con angular GT(now),
  fallan antes del primer goal. Diagnostico `P Y V`;
- auditoria: thrust total `+Z_body`, mixer lineal sin clipping, fuerzas
  relativas en Gazebo y masa compartida de `1.4 kg`;
- cambios: topic paralelo sellado `control/tray/thrust`, conservando el legacy;
  `BodyThrustDynamicPredictor` con gravedad, dt reales y `R_dynamic(t)`;
- validacion: builds correctos, GTest 86/86 y analizador correcto;
- 309 y su repeticion 313 completan en 91 s sin huecos de fuerza usando p/v
  GT(t_k) solo como estado inicial. RMSE p `0.0882/0.0990 m` y RMSE v
  `1.2020/1.2357 m/s`;
- 310 usa p/v del estimador causal vigente y falla antes del primer goal. RMSE
  p/v `0.1184/1.2888`; no hubo huecos de fuerza;
- conclusion: predictor translacional basico `VALIDADO EN LABORATORIO`; la
  estimacion de `v_hat(t_k)` sigue `NO VALIDADA` y bloquea el estado completo;
- STOP: 311/312 y 300-302 no ejecutadas. Fase 5H `PARCIAL`.

## 2026-08-30 - Velocidad lineal causal, pruebas 314-317

- objetivo: sustituir en laboratorio la `v_hat(t_k)` mezclada con correcciones
  de pose por una estimacion causal basada solo en tres posiciones visuales
  aceptadas, dejando la propagacion `t_k -> now` al predictor dinamico;
- cambios: `CausalLinearVelocityEstimator` reutilizable, historial crudo
  separado de clamps/correcciones, dt real, velocidad entre midpoints,
  aceleracion y proyeccion solo hasta `t_k`; rechazos no entran y epoch reinicia.
  La ruta productiva de `StereoSlamNode` no consume aun esta clase;
- validacion previa: build `orbslam3` codigo 0, GTest 94/94, suite del analizador
  correcta y `git diff --check` sin errores;
- prueba 314: completa llegada y hover sin fallback, tracking loss ni huecos de
  thrust. RMSE p/v `0.0901 m / 1.1061 m/s`; RMSE de `v_hat(t_k)` `0.7329 m/s`;
- intento inicial 315: invalido por omitir en el comando
  `f5h_gt_timing_mode:=dynamic_315`; completo usando `gt_fallback`. Sus
  artefactos se conservan como `prueba_315_invalida_sin_modo.*` y no cuentan;
- prueba 315 valida: reproduce 314, sin fallback ni huecos. RMSE p/v
  `0.0822 m / 1.0071 m/s`; RMSE de v_hat `0.5785 m/s`;
- prueba 316, estado completo dinamico: completa sin fallback. RMSE p/v
  `0.1026 m / 1.1842 m/s`, RMSE angular `0.1077 rad/s`, `er` max
  `0.7942 rad` y trabajo angular total `-0.00223 J`;
- prueba 317 confirma 316: RMSE p/v `0.0956 m / 1.0933 m/s`, RMSE angular
  `0.1149 rad/s`, `er` max `0.8047 rad` y trabajo total `-0.00409 J`;
- conclusion: bateria 314-317 `CONSEGUIDA`. La nueva `v_hat(t_k)` y el estado
  completo dinamico quedan `VALIDADOS EN LABORATORIO` bajo la traza 299. Fase
  5H sigue `PARCIAL`: falta acordar la conexion productiva y validar ORB real.

## 2026-08-30 - Integracion productiva y prueba 318

- objetivo: conectar en `StereoSlamNode` los estimadores causales y predictores
  dinamicos validados, comprobar paridad en 318 y despues validar ORB real en
  319/320 con parada en el primer fallo;
- cambios: selector temporal `navigation_prediction_mode=legacy|dynamic`,
  consumo sellado de torque/thrust, base comun p/v/R/omega por medida O y
  publicacion coherente propagada hasta `now`; un hueco invalida el estado y
  permite el fallback de Fase 5. Launches propagan el selector y parametros
  fisicos compartidos. Se crean los escenarios 318-320;
- build/tests: `orbslam3`, `dron_individual` y `simulacion_dron` codigo 0;
  GTest 94/94, analizador 8/8, Python/YAML y `git diff --check` correctos;
- prueba 318: runner codigo 0, `success=true`, 92 s, guard inactivo y minimo
  4847.5 MiB; no hubo fallback ni hueco translacional;
- evidencia negativa: un `F5H-DYNAMIC-MISSING` al inicio de un movimiento. El
  buffer contenia una muestra de torque, pero ninguna cubria el timestamp de la
  base dinamica. El bootstrap solo inserta cero cuando el buffer esta vacio;
- conclusion: 318 `NO CONSEGUIDA` por el criterio estricto de ausencia de
  huecos. Se aplica STOP y 319/320 no se ejecutan. La integracion compila, pero
  no queda validada productivamente ni existe evidencia ORB real;
- siguiente paso: acordar una politica de arranque con cobertura causal.

## 2026-08-30 - Politica causal y prueba 318R

- auditoria: `aplicar_fuerzas_dron` inicializa fuerza/torque a cero y publica
  a 50 Hz; `plugin_actuar_motores` inicializa todos los wrenches a cero y los
  reaplica por ZOH. El cold start cero queda demostrado fisicamente;
- cambios: cobertura `EMPTY/MISSING_PREFIX/FULL` para torque/thrust, seed cero
  sellado y trazable, stamps reales, buffers preservados ante reset visual y
  telemetria oldest/newest/prefijo. Las ecuaciones dinamicas quedan intactas;
- validacion: builds de los tres paquetes codigo 0, GTest 98/98, analizador 8/8,
  Python/YAML y `git diff --check` correctos;
- prueba 318R: escenario completo, codigo 0, 92 s, sin fallback y recursos
  sanos, pero un `F5H-DYNAMIC-MISSING` con `MISSING_PREFIX=0.070007563 s`;
- causa: el seed se creo correctamente, pero al llegar el primer comando tras
  unos 32 s, el recorte de 0.5 s elimino todas las muestras anteriores. La
  unica muestra restante quedo 70 ms despues de la base;
- conclusion: 318R `NO CONSEGUIDA`; cobertura de actuacion e integracion
  productiva no validadas. STOP: 319R/320/321 no ejecutadas;
- siguiente paso: conservar durante el pruning una unica muestra predecesora
  al horizonte como autoridad ZOH, sin extender ilimitadamente el historial.

## 2026-08-30 - Poda ZOH, paridad y ORB productivo 318R2-320R

- cambio: la poda conserva la ultima muestra `<= cutoff` y todas las
  posteriores, tanto para torque como thrust. La ventana sigue en 0.5 s y el
  coste adicional maximo es una predecesora;
- validacion: build `orbslam3` correcto, GTest 102/102 y analizador 8/8;
- 318R2: `CONSEGUIDA`, 92 s, dos goals correctos, cero missing, fallback,
  tracking loss o NaN/FATAL;
- 319R: `CONSEGUIDA`, 90 s y mismos criterios. Poda ZOH, cobertura de
  actuacion y paridad dinamica quedan `VALIDADAS`;
- 320: intento `INVALIDO`. El escenario completo, pero
  `navigation_state.yaml` sobrescribio el override y ambos stereo usaron
  `mode=legacy`. Se conserva como evidencia, no como validacion;
- correccion mecanica: `orbslam_use.launch.py` carga `stereo_params` al
  final; build `dron_individual` correcto;
- 320R: ambos stereo usan `mode=dynamic` y no hay missing de actuacion, pero
  ORB gobierna ya la aproximacion. Tras ese goal, el estado sigue en
  `(-0.030,-8.996,0.078) m` con velocidad
  `(0.064,-0.626,0.931) m/s`, lejos del objetivo `(0,-10,1) m`;
- durante el hover, tracking pasa 2->3 unos 20.3 s despues, conmuta a fallback
  sin salto y recupera 59 ms despues, pero queda bloqueado en GT por contrato;
- conclusion: 320R `NO CONSEGUIDA`. El action termina por tiempo y no
  demuestra seguimiento. La integracion productiva ORB no esta validada y 321
  no se ejecuta por STOP.

## 2026-08-30 - ORB shadow y handoff limpio 320R2/320R2R

- objetivo: separar la activacion prematura de 320R del comportamiento del
  ORB productivo al tomar autoridad desde hover estacionario;
- cambios: modo temporal `shadow_gt`, gate de tracking+anchor+estado valido y
  `1.5 s` de estacionariedad, servicio `control/activate_orb_shadow`, paso
  generico `call_set_bool` y YAMLs 320R2/321;
- exclusiones respetadas: sin cambios en estimadores, predictores, J, masa,
  gravedad, buffers, gains, gates visuales, referencia KF, W o control;
- builds: `dron_individual` correcto; primer build `simulacion_dron` fallo por
  `find_package(std_srvs)` omitido y paso tras la correccion mecanica;
- tests: mux 11/11, estimador productivo 102/102 y analizador 8/8;
- 320R2: `INVALIDA`; el runner recibio una ruta YAML relativa y termino antes
  del primer paso funcional. Se conserva como intento independiente;
- 320R2R: escenario `success=true`, 91 s, recursos sanos. Predictor `dynamic`,
  aproximacion integra con GT, ORB activo en sombra y handoff solo en la
  frontera del goal nuevo tras anchor y asentamiento;
- handoff: pose y rotacion `0`, velocidad `0.247078 m/s`, omega
  `0.003635 rad/s`. Goal ORB iniciado en
  `(0.011726,-10.002154,1.006141) m` con velocidad
  `(0.029463,-0.155310,0.194482) m/s`;
- evidencia posterior: sin missing, tracking permanece `2` y no hay fallback,
  pero el error de posicion llega a `~1.63 m`, velocidad a `~1.75 m/s` y error
  angular a `~0.52 rad`;
- conclusion: 320R2R `NO CONSEGUIDA`. La activacion prematura explica por que
  320R no era una prueba limpia, pero no es causa suficiente del fallo: el
  hover diverge con handoff controlado y tracking sano. 321 no ejecutada.

## 2026-08-30 - Bateria p/v ORB real 321A-D

- objetivo: eliminar la carrera authority/goal y aislar posicion y velocidad
  lineales sin modificar el estimador productivo;
- cambios: topic transient-local `control/orb_authority_confirmed`, marcador
  ordenado antes del goal, overrides de salida `position_gt|velocity_gt|both`
  alineados al O continuo y YAMLs 321A-D. Orientacion/omega siempre ORB;
- build/tests: `dron_individual` y `simulacion_dron` codigo 0; mux 13/13,
  Python/YAML y `git diff --check` correctos;
- 321A: `INVALIDA` antes del hover porque el lock retenido impedia confirmar
  ORB. Se abre mecanicamente la frontera y se repite como 321AR;
- validez comun: 321AR/B/C/D confirman ORB antes del segundo goal, usan
  predictor `dynamic`, cobertura FULL y cero missing. Los cuatro runners
  terminan `success=true`; recursos sanos;
- shadow RMSE p/v/R/omega: 321AR `0.0084/0.4556/0.0017/0.0240`, 321B
  `0.0129/0.6102/0.0017/0.0170`, 321C
  `0.0104/0.5459/0.0010/0.0338`, 321D
  `0.0105/0.3954/0.0021/0.0237` en m, m/s, rad y rad/s;
- 321AR `pGT+vORB`: max hover `ep=1.604 m`, `v=2.428 m/s`,
  `er=0.931 rad`; tracking 2 y sin fallback. `NO CONSEGUIDA`;
- 321B `pORB+vGT`: max hover `ep=0.159 m`, `v=0.046 m/s`,
  `er=0.027 rad`; tracking 2 y sin fallback. `CONSEGUIDA`;
- 321C `p/vGT+angularORB`: ORB cae a fallback antes de terminar el segundo
  goal por estado local no consumible, aunque el campo tracking siga en 2;
  max previo `ep=0.322 m`, `v=0.693 m/s`, `er=0.667 rad`. `NO CONSEGUIDA` y
  no valida aisladamente el canal angular;
- 321D ORB completo: max hover `ep=1.761 m`, `v=2.799 m/s`,
  `er=0.965 rad`; tracking 2 y sin fallback. `NO CONSEGUIDA`;
- conclusion agregada: diagnostico `CONSEGUIDO`, hover productivo
  `NO CONSEGUIDO`. 321B estable y el contraste A/D demuestran que la velocidad
  lineal ORB es la causa principal. La siguiente correccion debe actuar en
  `v_hat` sin GT y repetirse con ORB completo.

## 2026-08-30 - Diagnostico lineal ORB 322/323

- objetivo: localizar en que etapa se degrada `v_ORB` sin permitir que mueva
  el dron y contamine la medida;
- cambios: telemetria `[F5H-LINEAR-MEASUREMENT]`, velocidad GT lineal en el
  CSV, analizador dual-clock y YAML 322; la salida productiva no cambia;
- build/tests: tres paquetes codigo 0, GTest 102/102, analizador 4/4, YAML y
  diff correctos;
- prueba 322: `success=true`, autoridad GT durante toda la mision, predictor
  `dynamic`, tracking drone1=2 y ventana settled de unos 43 s; 907 medidas
  validas, sin missing ni perdida visual;
- RMSE/MAE/p95/max en m/s: `v_mid` `0.01984/0.01475/0.04061/0.10766`;
  `v_hat_tk` `0.03457/0.02557/0.07174/0.20456`; `v_dynamic_now`
  `0.43308/0.39375/0.69850/1.82854`;
- 323 paralelo sobre las mismas muestras: TWO_SAMPLE en `t_k` da RMSE
  `0.01988` frente a THREE_SAMPLE `0.03457`; `gain_hat` mediano `1.705`;
- 905/907 medidas usan THREE_SAMPLE y solo dos DEGRADED_DT; todas las medidas
  asentadas son `correction_class=SMALL`. Los cambios de referencia empeoran
  localmente, pero el tramo estable tambien conserva la amplificacion;
- conclusion: diagnostico `CONSEGUIDO`, causa `MULTICAUSAL` con
  `A_HAT_AMPLIFICATION` demostrada y una degradacion dominante adicional en
  `DYNAMIC_PROPAGATION`. No corresponde atribuirla a DEGRADED_DT ni solo a KF.
  STOP antes de modificar el estimador o predictor.

## 2026-08-30 - Gravedad expresada en O, pruebas 324/325

- auditoria: el predictor recibia p/v/R en O y rotaba correctamente thrust
  B->O, pero sumaba `(0,0,-9.81)` como si O estuviera alineado con W;
- cambio: `EpochGravityState` calcula `g_O=O_R_W*g_W` desde el primer
  `O_T_W` autoritativo, la congela por epoch y la invalida en epoch nuevo. Sin
  gravedad valida, la base dynamic no es consumible. THREE_SAMPLE queda igual;
- telemetria: init/wait, componentes thrust/gravity/aceleracion y residual de
  hover. El analizador filtra ahora por namespace para no mezclar drones;
- validacion: tres builds codigo 0, GTest 108/108, analizador 6/6, YAML/Python
  y diff correctos;
- 324: `success=true`, 875 medidas; `g_O=(0.5448,9.7923,0.2254)`, norma
  `9.8100`. RMSE mid/two/three/dynamic
  `0.02119/0.02122/0.03613/0.03583 m/s`; gain dynamic `0.992`. Residual de
  aceleracion mediano/p95 `0.0129/0.1330 m/s2`;
- 325: `success=true`, 906 medidas; `g_O=(0.3205,9.8014,0.2568)`, norma
  `9.8100`. RMSE mid/two/three/dynamic
  `0.02162/0.02166/0.03695/0.03707 m/s`; gain `1.003`. Residual mediano/p95
  `0.1064/0.1491 m/s2`;
- comparacion 322: dynamic baja de `0.43308` a `0.03583/0.03707 m/s` y el
  gain de `12.53` a aproximadamente `1`. Desaparecen bias Y/Z de
  `-0.2788/+0.2765 m/s`; 325 queda en `+0.00223/-0.00007 m/s`;
- conclusion: `GRAVITY_FRAME CONFIRMADO` y `DYNAMIC_PROPAGATION CORREGIDA` de
  forma reproducible. Fase 5H sigue `PARCIAL` porque
  `A_HAT_AMPLIFICATION/THREE_SAMPLE` permanece sin corregir. STOP respetado.

## 2026-08-31 - MIDPOINT_DYNAMIC y hover ORB real, pruebas 326-331

- cambio: `PredictMidpointDynamicVelocity` usa dos posiciones aceptadas,
  velocidad en el midpoint, interpolacion SO(3), omega espacial causal y los
  predictores de torque/thrust para propagar hasta `t_k`; exige cobertura
  `FULL`. THREE_SAMPLE queda solo como diagnostico;
- validacion estatica: tres paquetes compilan, GTest 116/116, analizador 7/7 y
  `git diff --check` correcto;
- 326 shadow hover: 930 muestras comunes y cobertura 100 %. RMSE TWO/THREE/
  MIDPOINT `0.023052/0.039098/0.023144 m/s`;
- 327 shadow X suave: 814 muestras; RMSE global
  `0.074311/0.112426/0.074276 m/s` y en movimiento
  `0.097242/0.136433/0.097166 m/s`;
- 328/329 shadow productivo: cobertura 100 % y RMSE MIDPOINT
  `0.021125/0.024599 m/s`. El intento 0 de 329 muere antes del escenario por
  Gazebo; el reintento 1 completa y fundamenta la conclusion;
- 330 ORB real: `34.78 s`, tracking OK, cero fallback/clamp, max
  `er=0.0674 rad`, max omega control `0.0722 rad/s` y energia angular total
  `-0.000441 J`;
- 331 repeticion: `35.30 s`, tracking OK, cero fallback/clamp, max
  `er=0.0631 rad`, max omega control `0.0774 rad/s` y energia angular total
  `-0.000348 J`;
- conclusion: `A_HAT_AMPLIFICATION CORREGIDA`,
  `LINEAR_VELOCITY_ESTIMATOR VALIDADO` y `HOVER ORB REAL VALIDADO` de forma
  reproducible. 5H permanece `PARCIAL` hasta validar movimiento y trayectoria.

## 2026-08-31 - Validacion progresiva de movimiento, pruebas 332-334

- contrato: arquitectura congelada, fallback cero en maniobras elementales,
  repeticion obligatoria y STOP en el primer fallo;
- validacion previa: tres builds codigo 0, GTest 116/116, analizador 7/7, YAML
  y diff correctos;
- 332 X 2 m: `CONSEGUIDA`; ORB `29.56 s`, sin fallback, tracking loss, missing
  ni clamp. RMSE/max ep `0.0360/0.0676 m`; ep/ev final
  `0.0439 m / 0.0283 m/s`; energia total `-0.001766 J`;
- 333 repeticion X: `CONSEGUIDA`; ORB `30.28 s`, sin fallback ni tracking
  loss. RMSE/max ep `0.0477/0.1125 m`; ep/ev final
  `0.0507 m / 0.0294 m/s`; energia `-0.002225 J`;
- conclusion X: `MOVIMIENTO X ORB VALIDADO` y reproducible;
- 334 Y 2 m: `INVALIDA POR COLISION`. ORB gobierna `6.62 s`; tracking pasa `2->3` y
  despues `3->0->1`, activando `gt_fallback reason=tracking_lost`. Antes de la
  perdida, RMSE/max ep `0.0295/0.0648 m` y RMSE/max ev
  `0.0567/0.1660 m/s`; no existe divergencia de control previa;
- interpretacion revisada tras observacion del usuario: el goal avanza desde
  `[0,-10,1]` hasta `[0,-8,1]` y atraviesa el fiducial 2 situado en
  `[0,-8.5,1]`. La colision explica la posterior perdida de tracking; no hay
  evidencia contra el control Y;
- 335-343 no ejecutadas. Fase 5H permanece `PARCIAL`; repetir como 334R hacia
  `-Y` sin obstaculo y con arquitectura intacta.

## 2026-08-31 - Repeticion Y sin obstaculo, prueba 334R

- escenario: `[0,-10,1] -> [0,-12,1]`, alejandose del fiducial 2; estimador,
  control y criterios congelados;
- resultado de infraestructura: runner `success=true`, ORB `30.26 s`, tracking
  2, cero fallback, missing o clamp y cobertura MIDPOINT 100 %;
- posicion/angular: RMSE/max ep `0.0327/0.0628 m`, max er `0.1372 rad` y
  energia angular total `-0.002717 J`;
- velocidad: RMSE/max ev `0.1630/0.4118 m/s`; ev final `0.1873 m/s`. En los
  ultimos 3 s, RMSE/max ev ORB `0.1739/0.3342 m/s`, mientras GT da
  `0.0491/0.0745 m/s` y termina en `0.0561 m/s`;
- conclusion: `NO CONSEGUIDA`. El movimiento conserva posicion y tracking,
  pero no vuelve limpiamente a `v_ORB≈0`; existe velocidad estimada residual y
  oscilante en el frenado/hover final;
- STOP aplicado: 335-343 no ejecutadas. No se modifica el estimador.

## 2026-08-31 - Repeticion visual Y, prueba 334R2

- se repite exactamente 334R con Gazebo GUI visible y artefactos separados;
- runner `success=true`, ORB `33.30 s`, tracking 2, cero fallback/missing y
  cobertura MIDPOINT 100 %;
- max ep/ev `0.4929 m / 2.1876 m/s`; cierre ep/ev
  `0.2128 m / 0.5506 m/s`; RMSE/max ev en los ultimos 3 s
  `1.0076/2.1876 m/s`; RMSE lineal productivo `0.4398 m/s`;
- conclusion: `NO CONSEGUIDA`; reproduce y agrava la inestabilidad lateral y
  de frenado con tracking sano. 335-343 permanecen sin ejecutar.

## 2026-08-31 - Proximidad visual, Y y Z, pruebas 334R3-337R

- 334R3: `INVALIDA DE INFRAESTRUCTURA`; timeout de
  `/fiducial_spawn_ready`, sin goals ni evidencia funcional;
- 334R3R/335R: X seguido de +Y a `x=2 m`, evitando el fiducial y acercando la
  camara a la pared. Ambas completan con tracking 2 y cero fallback, missing o
  clamp. Max ep en Y/hover `0.071/0.059 m` y `0.088/0.092 m`; velocidad final
  `0.108/0.111 m/s`;
- conclusion Y cercana: `CONSEGUIDA` con residual de velocidad documentada.
  El contraste con 334R/334R2 apoya la hipotesis de peor cobertura visual al
  alejarse de la pared;
- 336 Z +0.5 m: `CONSEGUIDA`; max ep `0.051 m`, ev final `0.015 m/s`, tracking
  continuo, cero fallback y energia angular `-0.000400 J`;
- primer intento 337: `INVALIDA DE INFRAESTRUCTURA` por ruta YAML relativa;
- 337R: `CONSEGUIDA`; reproduce Z con max ep `0.046 m`, ev final `0.018 m/s`,
  tracking continuo, cero fallback y energia `-0.000477 J`;
- conclusion: `MOVIMIENTO Y CON BUENA COBERTURA VISUAL VALIDADO` y
  `MOVIMIENTO Z ORB VALIDADO`.

## 2026-08-31 - Giro yaw, prueba 338

- escenario: giro lento `90 -> 0 deg` con ORB real, arquitectura congelada;
- runner y goals completan, pero ORB gobierna solo `11.18 s` antes de que
  tracking pase `2->3`; el mux activa `gt_fallback reason=tracking_lost`;
- antes/durante la perdida: max er `0.995 rad`, max ew `0.709 rad/s`, RMSE
  omega control-GT `0.409 rad/s` y RMSE lineal productivo `0.530 m/s`;
- la energia angular neta es negativa (`-0.000785 J`), pero no compensa la
  degradacion de pose/velocidad ni la perdida visual;
- conclusion: `NO CONSEGUIDA`. El giro yaw no queda validado. STOP aplicado:
  339-343 no ejecutadas; no se modifica el estimador ni el control.
