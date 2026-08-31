# Estado actual - resumen

## Situacion

```text
Fase 2: CONSEGUIDA
Fase 3: cierre previo conseguido; reabierta únicamente en 3Q
Fase 4: CONSEGUIDA Y CERRADA con alcance 4A-4H
Fase 5: funcionalmente CONSEGUIDA; 5H cerrada por evidencia visual y ORB 3/3
4A: CONSEGUIDA
4B: CONSEGUIDA
4C: CONSEGUIDA
4D: CONSEGUIDA
4E: CONSEGUIDA
4F: CONSEGUIDA
4G: CONSEGUIDA
4H: CONSEGUIDA
4I: APLAZADA; regresion opcional futura
Subfase actual: 5H CONSEGUIDA; limitación de observabilidad transferida a Fase 6
Preparacion 5H: CERRADA y autorización consumida
Siguiente punto de entrada: preparar Fase 6 y retirada progresiva de GT fallback
Trabajo funcional activo: ninguno; 279-281 siguen detenidas
Punto de entrada siguiente: preparar Fase 6 y retirar progresivamente GT fallback
Revision visual de prueba 200: confirmada correcta por el usuario
Pendiente de Fase 2: ninguno
Autorizacion 4A+4B: concedida y consumida
```

## Bloque 4A+4B

- tres objetos fiduciales baseline a ±8.5 m, 15 tags y rango `[1,5] m`;
- spawner Gazebo con validacion offline, SDF dinamico y readiness transient-local;
- trayectoria de dos drones a ±10 m: 10/10 goals correctos en prueba 201;
- prueba 202 confirma cierre limpio del spawner;
- Gazebo y RViz2 activos; ambos grafos web desactivados;
- revision visual: confirmada perfecta por el usuario;
- trayectoria tipica: cuadrado ±10 con paradas cardinales, usada parcialmente en 205.

## Bloque 4C+4D

- evento one-shot y recibo exacto de KF, imagen y calibracion implementados;
- servicio con 15 tags, cliente, cola 4/drop-oldest, worker y detector validados;
- prueba 205: tag 202 detectado por ambos drones con error menor de `0.26 px`;
- pruebas 205/206 revelaron contaminacion Snap y que un cierre forzado de
  HighGUI dentro de `stereo` mata el wrapper;
- el wrapper publica ahora imagen anotada latest-only y un proceso ROS separado
  `fiducial_visualizer` posee HighGUI;
- prueba 207: escenario completo y wrappers estables, pero una carrera cerraba
  las ventanas en 3-4 ms antes de hacerlas visibles;
- prueba 208: escenario completo, 79 SHOW, 17 timeouts, cero cierres falsos y
  deltas de ambos wrappers 57 s despues del ultimo cierre;
- el usuario acepta la 208 y da 4C+4D por concluidas.

## Bloque 4E+4F

- topic reliable/volatile KeepLast(32), batch no vacio y solo tags validos;
- identidad y timestamp de KF exactos, con `camera_T_tag` y metricas finitas;
- sidecar pending O(1), FIFO por dron, capacidad configurable 10 y sin TTL;
- digest consumido sustituye el flag `fiducial_batch_consumed`;
- pruebas unitarias, builds y CTests completos correctos;
- prueba 210: trayectoria tipica, 68/68 matches, pico pending 7/10 y cero
  expulsiones/conflictos/rechazos;
- prueba 211: ambos grafos live y 18/18 matches adicionales.

## Bloque 4G+4H

- interpretador en Servidor con `yaml-cpp`, rango por tag, fusion robusta,
  primary unico, visitas por intervalos y FIFO 50;
- todos los KFs primary llegan al `FiducialAnchorManager` existente;
- ruta GT fiducial eliminada de codigo, configuracion, replay y grafos;
- CTest: Servidor 150 y Simulacion 85 tests sin fallos;
- prueba 216: trayectoria completa sin GT, 52/52 primary y tres objetos;
- smoke 217: ambos grafos live; guardas 15/15.

Repeticion visual 212: el YAML incorpora seis yaw relativos y pasa contratos,
pero un loop incompatible con una hard constraint activo un fallo bloqueante
antes del paso 5. Los giros nuevos no se ejecutaron; la repeticion esta
suspendida hasta decidir como tratar el mission gate.

El usuario decide eliminar el latch persistente de fallo. Tras el cambio,
`orbslam3_server` compila y pasa 12/12 targets. La prueba 213 completa 17/17
pasos y 22/22 goals, libera el backpressure tras cada optimizacion y produce
74/74 PUB/SHOW fiduciales. El usuario da 4A-4F por concluidas, pero no acepta
esta ejecucion como validacion de 3Q: hubo derivas visibles y nueve propuestas
loop tardias rechazadas. La prueba 213 queda como reentrada obligatoria de 3Q.

## Correccion 3Q y pruebas 219-220

- ventana segmentada comun para loop/fiducial, sin loop sintetico ni cierre por
  submapa completo;
- solo hard inmovil; KFs internos sin deadband 2 cm y revisitados 5 m/20 grados;
- apoyo 2/4/6, pesos efectivos, consenso temporal 3/60 y umbrales separados;
- builds correctos; CTest `orbslam3_multi` 9/9, Servidor 12/12 y Simulacion 10/10;
- 219 completa 17/17 y 22/22; 22/30 solves hacen commit y cinco fusionan;
- ocho descartes explicables, cero hard/revisit/fatal y cero anchors por loop;
- visualmente una esquina queda corregida por completo y la derecha multi-epoch
  solo parcialmente; `pending` llega a 51 y queda un solve activo al apagar.
- mejora posterior: cascada automatica al aparecer autoridad world y
  recuperacion reciente 1/1 provisional dentro de `0.50 m/0.15 rad/2 m`, con
  fallback 2/4/6 fuera de esa banda;
- 220 completa 17/17 y 22/22, valida la cascada y no activa incorrectamente 1/1;
  el usuario califica el resultado general como excelente;
- defecto residual: `task=1000000005590` encontro constraints consecutivas con
  error world casi `0/1.012/0 m`, selecciono las tres regiones y movio 277 KFs
  de una ventana de 296. Esto no identifica por si solo al candidate central
  como pose incorrecta. El validator admitio 0.289 m de degradacion estructural
  por quedar dentro de sus limites amplios;
- punto de entrada: conservar las tres como `CurrentLoop`, pero no aceptar
  `MaxIterations` como convergencia ni permitir que loops ya satisfechos
  empeoren por la mejora OR; añadir guarda estructural local, sin tocar cascada.

## Fase 5A

- arquitectura reconciliada con `O_T_B` continuo y `W_T_B` corregible;
- reference KF real + `Tcr`; consulta inicial asíncrona y revisiones por push;
- absolutos sin global rechazados y goals activos congelados en `O`;
- `GT_FALLBACK` temporal visible hasta recovery real en Fase 6;
- smoothing no obligatorio y antigua 5I absorbida en 5H;
- sin cambios funcionales, builds, tests o simulaciones.

## Fase 5B

- `StereoTrackingReceipt` coherente con tracking, ref-KF real y `Tcr`;
- `NavigationState` y `O_T_B` continuos dentro del epoch, sin continuidad falsa
  durante pérdida;
- absolutos sin global rechazados; relativos frescos aceptados con snapshot;
- builds seleccionados correctos y tests nuevos funcionales correctos;
- prueba 225: dos anchors hard, cambios ref-KF con paso cero, 7/7 pasos y
  pérdida 2->3->0->1 de ambos drones tras giro de 180 grados;
- conclusión: CONSEGUIDA; siguiente bloque 5C+5D+5E+5F.

## Bloque 5C+5D+5E+5F

- 5C-5E CONSEGUIDAS y 5F PARCIAL tras la prueba 230;
- autoridad existente en `GlobalPoseStore`, sin base global paralela;
- servicio asíncrono, pending de referencia activa y push dirigido por dron;
- W provisional observable pero inválida; solo autoritativa habilita global;
- goals absolutos deshabilitados hasta la conversión world->O de 5H;
- 5F entrega métricas numéricas y gráficas O/W/GT; el error frente a GT no
  gobierna la selección de fuente por la deriva acumulada.

## Bloque 5G+5H

- mux ORB/`GT_FALLBACK`, velocidad común y goals absolutos integrados;
- fuente congelada por trayectoria: no existe `GT -> ORB` dentro de un goal;
- una pérdida ORB permite `ORB -> GT` inmediato y mantiene GT hasta la frontera;
- el error GT-pose estimada es solo una métrica externa, nunca una guarda;
- RViz2 representa la `O_T_B` exacta consumida por el controlador y etiqueta
  cada dron con `[ORB]` o `[GT]`;
- prueba 242 fallida por handshake no consumible, conservada en historial;
- prueba 243: 17/17 pasos, 22/22 goals, 44/44 handshakes, cero `GT -> ORB`
  dentro de goals y ejecución `success=true`;
- prueba 246: revisión visual NO CONSEGUIDA; `GT -> ORB` entre goals deja la
  última consigna GT activa frente a una pose ORB;
- prueba 247: el reset hizo `ep=0`, pero no `ev=0` y cambió fuente durante una
  espera; no demuestra un defecto de ORB ni de ganancias;
- prueba 248: la conmutacion atomica arranca con pose/velocidad y frame global
  coherentes, pero ambos drones pierden tracking en menos de 1.7 s y divergen;
- prueba 249: handoff angular inicia con `er=ew=0`, hover y torque cero; no es la
  causa del movimiento;
- pruebas 250/251: X world se proyecta casi exactamente sobre Z control. La
  autoridad `W_T_C` es correcta y el YAML denominado `body_T_camera` contiene
  la rotacion inversa `camera_T_body`; el wrapper la invierte otra vez;
- prueba 252: `B_T_C` corregido, 17/17 pasos, 22/22 goals y exit 0. Desaparece
  el fallo de ejes, pero persisten tirones y maniobras alocadas;
- causa vigente: velocidad ORB por diferencia finita sin filtro a 20 Hz frente
  a control 50 Hz, mas `ORB -> GT` dentro del goal mediante `ResetToSource`
  mientras el feedback conserva la trayectoria del O anterior;
- prueba 253: intento de predictor SE(3) a 50 Hz interrumpido a los 103 s. No
  hubo cambio de fuente; ambos drones seguian en GT. El filtro angular retrasó
  la actitud consumida por el control y desestabilizó el lazo de torque;
- los ejes muestran `o_t_body` de control y los KFs viven en W optimizado; una
  discrepancia aislada entre ambos no demuestra un KF incorrecto;
- conclusión: PARCIAL; extrinseca conseguida, calidad dinamica pendiente.
- prueba 254: predictor trasladado al wrapper, GT exacto y 3/3 tests focales
  correctos; simulacion interrumpida tras 13/17 pasos por divergencia en giros
  ORB. El handoff tiene salto cero, pero orientacion y velocidad angular quedan
  incoherentes ante pasos de hasta unos `0.28 rad/frame`;
- siguiente correccion: limitar/suavizar solo innovacion angular ORB y derivar
  de ella pose y velocidad coherentes. No tocar GT, ganancias ni optimizador.
- prueba 255: builds y 13/13 GTests finales correctos; gate de referencias y
  outliers evita publicar las innovaciones angulares graves y conmuta a GT con
  salto cero. Interrumpida tras 7/17 pasos, con 10 timeouts y ORB poco
  sostenido. La revision visual y los timestamps descartan optimizacion global:
  churn de reference KF/outliers locales preceden los tres episodios y dos
  ocurren despues de terminar todas las optimizaciones.
- prueba 256: probation geometrica multi-KF y estimador SE(3), con build y 15/15
  GTests correctos. El handoff GT->ORB tiene salto cero, pero drone2 acepta una
  innovacion angular de 0.125 rad, registra un `rotation_step` de 0.119 rad y
  pierde tracking 0.793 s despues. Ese campo no es el salto publicado; queda
  pendiente confirmar temporalmente correcciones moderadas e instrumentar el
  estado realmente consumido, torque y tracking antes de fijar causalidad.
- nueva iteracion: probation angular temporal, telemetria causal, builds
  correctos y 21/21 GTests. La etapa 1 de prueba 258 completa 11/11 pasos bajo
  GT observado y localiza una perdida ORB durante yaw rapido;
- prueba 259: el hover entra en ORB con salto cero, pero solo dura 227 muestras.
  Una salida tratada como pequena publica `0.058777 rad` y el control oscila
  antes del primer pending; despues se confirma un residual creciente, llegan
  pasos de `0.075 rad`, dos outliers, fallback y tracking 3. Etapas 3-8
  detenidas. Limitaciones: SMALL aumenta con `dt` y la persistencia del residual
  no es evidencia independiente de movimiento fisico.
- redisenio raw/bias: SMALL fijo, canales `omega_motion`/`omega_bias`, builds
  correctos y 27/27 GTests;
- prueba 260: 11/11 pasos y 7/7 goals bajo GT; calibracion raw conseguida;
- prueba 261: handoff y primer error cero, pero ORB dura unos 3.82 s. El bias
  actua desde SMALL, el movimiento realimentado se acepta como raw plausible y
  domina la salida. Tres rechazos invalidan el estimador antes de tracking 3;
- correccion 262: bias con deadband/confirmacion/supresion y decay raw; builds
  correctos y 37/37 GTests;
- prueba 262: `omega_bias=0` y decay correcto, pero `omega_motion` alcanza
  ~`0.617 rad/s`; ORB dura ~`5.92 s` y fallback precede ~`0.54 s` a tracking
  2->3;
- conclusion vigente: `PARCIAL`; no ejecutar etapa 3. El siguiente diagnostico
  debe medir latencia/fase del canal angular visual y el lazo de control.
- prueba 263: ejecución técnica correcta, pero `DATOS_INSUFICIENTES` porque GT
  y control quedaron en relojes Gazebo/ROS sin puente en esa captura;
- estado preparado: `gt_receive_stamp`, telemetría vectorial y analizador
  dual-clock pasan builds y tests;
- prueba 264: 323 ciclos ORB sincronizados durante `6.44 s`; raw sigue al GT
  con `~0.08 s`, pero la pose/control queda fuera de fase. `tau_er` hace trabajo
  positivo en `80.9 %` del tramo post-handoff e inyecta `+0.005173 J`, casi
  anulando el damping neto de `tau_ew`;
- conclusion vigente: diagnostico temporal `CONSEGUIDO`, hover
  `NO CONSEGUIDO`, 5H `PARCIAL`. No ejecutar etapa 3.
- prueba 265: corrige el horizonte con edad local real y una unica propagacion;
  40/40 GTests y 5/5 tests del analizador pasan. ORB dura `5.56 s`, `tau_er`
  inyecta `+0.160266 J` y el torque total `+0.145081 J`;
- diagnostico 265: `visual_q -> base_q` llega a `0.339 rad`, muy por encima de
  `base_q -> predicted_q`; el desfase dominante vive en `pose_` base integrada
  y el antiguo horizonte fijo lo compensaba parcialmente;
- conclusion vigente: 265 `NO CONSEGUIDA`, 5H `PARCIAL`; etapa 3 detenida y
  fusion/anclaje visual pendiente de nuevo acuerdo.
- prueba 266: SMALL/plausible reancla la pose y moderate corrige hasta
  `0.015 rad`; tres builds, 44/44 GTests y 7/7 tests del analizador correctos;
- comparacion comun: ORB `8.06 s` frente a `5.56`; `tau_er=+0.002067 J` frente
  a `+0.153559 J`; torque total `-0.001945 J` frente a `+0.138374 J`;
- fallo 266: desde `+5.90 s` moderate deja residual, alterna PREDICT_ONLY y raw
  se rechaza a `+7.50 s`; fallback `+8.08 s`, tracking 3 `+8.68 s`;
- conclusion vigente: principio SMALL validado, prueba funcional
  `NO CONSEGUIDA`, 5H `PARCIAL`; no etapa 3, politica moderate por acordar.
- prueba 267: cuatro anclas moderate completas dejan error after cero, pero ORB
  cae a `5.56 s`; `tau_er=+0.030448 J` y total `+0.015622 J` en ventana comun;
- diagnostico 267: antes del primer anclaje el total era disipativo; despues la
  energia de fallo se acumula en `1.22 s`, mas deprisa que en 266;
- correccion posterior: confirmed solo ancla con raw plausible; build y 46/46
  GTests correctos, sin nueva simulacion. No ejecutar 268 ni etapa 3.
- prueba 268: el unico anclaje moderate es raw plausible, corrige
  `0.057317 rad` y deja error after cero; gate raw validado;
- fallo 268: antes del anclaje el total es `-0.001356 J`; despues acumula
  `+0.046416 J` en `0.88 s`, ORB dura `5.72 s` y cae a fallback;
- conclusion: anclaje completo contraindicado, no 269 ni etapa 3;
  `Delta_target` gradual a `0.30 rad/s` queda pendiente de autorizacion.

## Entrega de Fase 2

- grupos fisicos `dron`, `servidor` y `simulacion`;
- builds y prefijos separados por grupo;
- interfaces duplicadas de forma controlada;
- configuracion por dominio y despliegue segun ADR 0009;
- ORBvoc completo instalado desde un bootstrap fuera de `src`;
- observabilidad lazy-gated segun ADR 0010;
- `system_architecture` estatico/live separado de `pipeline_flow`;
- guardas automaticas de arquitectura y documentacion.

## Validacion

- build: 9/9 paquetes, un paquete por invocacion;
- CTest: `lib_tray` 4/4, `orbslam3_multi` 9/9,
  `orbslam3_server` 10/10 y `simulacion_dron` 9/9;
- prueba 199: debug-off, 5/5 pasos y 4/4 goals;
- prueba 200: debug completo, 14/14 pasos y 20/20 goals;
- RViz2 y ambos web activos; guarda de recursos no disparada;
- `system_architecture_bridge` cierra sin el `ValueError` de prueba 198.
- layout final: CTest 9/9, guarda 15/15 y dos viewports inspeccionados.

## Limitaciones

`dron_individual` conserva deuda legacy global de linters, aunque todos los
archivos tocados pasan comprobaciones focales y rebuild. La prueba 200 presenta
un traceback de cleanup de `gui_tray_multi` y el exit 255 conocido de Gazebo,
ambos posteriores a `SIM-DONE`.

Las pruebas 205-207 preservan fallos ya corregidos. La 208 valida el aislamiento
y fue aceptada por el usuario como cierre de 4D.

## Referencias

```text
codex/pipeline/fase_2_separacion_paquetes/RESULTADO_FINAL_FASE_2.md
codex/pipeline/fase_2_separacion_paquetes/historial/INDEX.md
codex/pipeline/fase_4_fiducial_real/historial/INDEX.md
codex/contexto/decisiones/ADR_0009_configuracion_por_dominio_y_despliegue.md
codex/contexto/decisiones/ADR_0010_observabilidad_web_debug_coste_cero.md
```
