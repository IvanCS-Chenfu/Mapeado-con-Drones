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
  `0.125261 rad`. El filtro no la rechaza y publica un paso de `0.119002 rad`.
  Despues acepta rapidamente los KF 28 y 31 y pierde tracking en
  `1787932615.088`, solo `0.793 s` despues del salto;
- diagnostico: el umbral duro de `0.35 rad` es demasiado permisivo para actitud
  de control. La correccion gradual convierte un outlier moderado en un cambio
  de aproximadamente `6.82 grados` antes de comprobar persistencia temporal;
  suavizarlo no lo vuelve verdadero. El churn de referencias es el contexto
  local que lo precede, pero la inyeccion angular aceptada es el primer evento
  dinamico anormal demostrado;
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
