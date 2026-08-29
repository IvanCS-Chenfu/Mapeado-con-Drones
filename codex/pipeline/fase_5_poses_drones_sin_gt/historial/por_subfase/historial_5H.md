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
