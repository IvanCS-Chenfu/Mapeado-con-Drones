# Historial de preparacion y ejecucion 1J

## Acuerdo de la entrega previa

Antes de crear el joint de pitch se acordo implementar
`phase5_navigation_source=gt|orb`. Las dos pruebas debian usar la misma escena,
con el dron frente al fiducial. En `gt`, GT debia gobernar perfectamente y ORB
seguir en sombra; en `orb`, ORB debia gobernar realmente con
`gt_fallback_enabled=false`, sin exigir estabilidad al control de Fase 5.

Quedaron fuera de esta entrega el joint, pitch, PID, TF dinamica, extrinseca,
estimador ORB, gains, fiduciales y cualquier reparacion del control F5.

## Cambios y validacion estatica

Se incorporo `POSE_SOURCE_GT_FORCED=4` a las dos replicas de
`NavigationState`, el parser estricto del selector, la publicacion GT
independiente de ORB, su tratamiento en goals, launches, visualizacion,
metricas y tests. Las replicas Dron/Server quedaron identicas.

Compilaron `orbslam3_msgs` Dron, `dron_individual`, `simulacion_dron` y
`orbslam3_msgs` Server. Pasaron 2/2 tests funcionales C++ y 24/24 tests Python
dirigidos. El CTest global solo fallo por 1540 avisos `flake8` heredados en 26
scripts experimentales no modificados.

## 359 - intento inicial GT

Resultado: **INVALIDA DE INFRAESTRUCTURA**.

El launch termino antes del escenario porque `pose_metrics_node` cargo la
replica Server antigua de `orbslam3_msgs`, sin `POSE_SOURCE_GT_FORCED`. La
memoria minima fue 5515.5 MiB y el guard no actuo. Se mantuvo este intento en
el historial, se sincronizaron ambas replicas, se recompilo Server y se
verifico que el overlay runtime exponia el valor 4.

## 359R - GT forzado

Resultado: **CONSEGUIDA**.

El runner completo el escenario en 79 s (`success=true`), con minimo 5279.8
MiB y guard inactivo. El goal absoluto fue aceptado con `source=gt_forced` y
termino correctamente. Las 3479 muestras de salida fueron `GT_FORCED`, sin ORB
ni fallback; el error O-GT fue esencialmente cero. ORB siguio en sombra con
3201 muestras de tracking OK y 12 referencias, sin activar autoridad.

## 360 - ORB estricto

Resultado: **CONSEGUIDA PARA EL SELECTOR**.

El runner completo el escenario en 79 s (`success=true`), con minimo 5402.8
MiB y guard inactivo. La autoridad cambio de `invalid` a `orb` antes del goal,
con tracking, pose local, continuidad, velocidad, pose global, anchor y
referencia validos. El goal fue aceptado con `source=orb` y termino. Se
observaron 3074 muestras ORB, cero fallback y cero GT forzado.

La pose/velocidad ORB inicial difirio de la fisica, como ya permite el alcance
aceptado de Fase 5. Ademas, `pose_metrics_node` no emparejo timestamps ORB y GT
por usar dominios de reloj distintos; esto no invalida la fuente, demostrada
por mux, escenario y logs reducidos. La salida del servidor de configuracion
fiducial con codigo 1 ocurrio durante SIGINT, despues de `SIM-DONE`.

## Conclusion agregada

El selector previo de navegacion queda **CONSEGUIDO** y permite desarrollar 1J
primero bajo GT y repetir despues bajo ORB. La subfase 1J completa permanece
pendiente: aun no existe el joint de pitch ni su control o extrinseca dinamica.

## 361 - primera integracion completa 1J con GT forzado

Resultado: **NO CONSEGUIDA; STOP EN BLOQUE FISICO**.

Se implementaron `stereo_rig`, `stereo_pitch_joint`, plugin fisico con
`JointTrajectory`/`JointState`, perfil limitado, TF body-camera sincronizada,
paso `pitch` del scenario runner y modo `static|tf` del wrapper. Compilaron
`simulacion_dron` y `orbslam3`; pasaron 117 tests de navegacion, 3 fiduciales,
3 de evidencia visual y 24 contratos Python.

La prueba 361 mantuvo recursos sanos y completo el hover GT, pero el joint
publico `velocity=-nan` y `effort=-nan` desde el primer segundo aun con target
neutral. La consigna `+30 deg` fue recibida correctamente, la posicion quedo
en cero y el paso expiro a 15 s. La contaminacion fisica alcanzo despues
velocidad/control del dron con valores NaN. No se ejecutaron `-30 deg`,
saturacion, retorno neutral ni validacion dinamica F5. Se aplica el STOP
acordado antes de retocar masa, inercia, damping, esfuerzo o gains.

La ejecucion arranco dos drones por el valor vigente de `sim_dron.yaml`, aunque
el escenario solo dirigio `dron_1`; esto debe corregirse o declararse en la
repeticion para aislar el caso.

## 362 - estabilizacion fisica y consigna aislada

Resultado: **NO CONSEGUIDA; MOVIMIENTO FINITO PERO SIN REPOSO**.

Tras aumentar masa e inercia a valores pequenos pero numericamente estables,
reducir esfuerzo y gains, y añadir damping, friccion y guardas finitas, se
recompilo `simulacion_dron` correctamente. La prueba aislada mantuvo todos los
estados del joint finitos: no reaparecieron los NaN de 361.

Ante `+30 deg`, el joint alcanzo aproximadamente `0.5166 rad` para una consigna
de `0.5236 rad`, un error estacionario de unos `0.40 deg`, dentro de la
tolerancia angular de `1 deg`. Sin embargo, la velocidad alterno
aproximadamente entre `-0.052` y `+0.126 rad/s`, por encima del criterio de
reposo `0.03 rad/s`. El paso expiro a 15 s y no se ejecuto el retorno a cero.

Los recursos permanecieron sanos: 38 muestras en 45 s, minimo 5408.7 MiB,
guard inactivo y sin PSI de memoria. Se aplica STOP antes de retocar de nuevo
damping, friccion, gains o criterio de asentamiento. La prueba sigue arrancando
dos drones por la configuracion vigente, aunque solo `dron_1` recibe consigna.

## 363 - reduccion de friccion y aumento de amortiguamiento

Resultado: **NO CONSEGUIDA; OSCILACION DE VELOCIDAD PERSISTENTE**.

Con autorizacion del usuario se mantuvieron masa, inercia, `kp=0.01`, `ki=0`,
torque maximo `0.002 Nm` y limites. Se redujo la friccion Coulomb de `0.001` a
`0.00005 Nm`, se aumento el damping viscoso de `0.005` a
`0.01 Nms/rad` y `kd` de `0.003` a `0.004`. El objetivo era eliminar un posible
ciclo de adherencia/liberacion sin introducir accion integral. El paquete
`simulacion_dron` compilo correctamente en 16.1 s.

La prueba 363 repitio exactamente neutral, `+30 deg` y retorno con tolerancia
`1 deg` y timeout `15 s`. No aparecieron NaN. El joint alcanzo
aproximadamente `0.5119 rad` ante `0.5236 rad`, error cercano a `0.67 deg`,
pero la velocidad continuo alternando aproximadamente entre `-0.053` y
`+0.127 rad/s`. El paso `+30 deg` expiro y no se ejecuto el retorno a cero.

La posicion entre muestras tardias varia muy poco mientras la velocidad
instantanea reportada conserva impulsos alternos. El cambio de friccion y
damping no elimina el fenomeno, por lo que la hipotesis de stick-slip queda
debilitada; la siguiente hipotesis es ruido/impulso numerico en
`Joint::GetVelocity(0)` realimentado por el termino derivativo. Probar un filtro
de velocidad cambia el controlador y requiere nuevo acuerdo.

Recursos: 38 muestras en 45 s, minimo 5473.4 MiB, guard inactivo, sin PSI de
memoria y Gazebo termino limpiamente durante el cierre. Se aplica STOP sin
realizar otra modificacion ni repetir.

## 364 - filtro de velocidad y puerta aislada

Resultado: **CONSEGUIDA**.

Se incorporo el parametro configurable
`fisico.camera_pitch.velocity_filter_tau_sec=0.05`. El plugin aplica un filtro
paso bajo de primer orden a `Joint::GetVelocity(0)`: la velocidad filtrada se
usa en el termino D y en `JointState.velocity`, mientras el log conserva tanto
`velocity_filtered` como `velocity_raw`. Las guardas siguen validando la medida
cruda y reinician el filtro si aparece un valor no finito. Tambien se corrigio
el default mecanico de `kd` del generador de `0.003` a `0.004`, alineandolo con
YAML y Xacro. `simulacion_dron` compilo correctamente en 20.6 s.

La prueba 364 mantuvo los criterios de 362-363. `+30 deg` completo en
`29.006 deg` con velocidad filtrada `0.011370 rad/s`; el retorno completo en
`0.988 deg` y `-0.013038 rad/s`. Tras la observacion final la posicion continuo
convergiendo cerca de cero. La medida cruda conservo impulsos aproximados entre
`-0.05` y `+0.12 rad/s`, ahora separados del estado operativo y sin excitar D.
No aparecieron NaN y el escenario termino `success=true`.

Recursos: 43 muestras en 51 s, minimo 5464.4 MiB, guard inactivo y sin PSI de
memoria. La puerta aislada neutral/`+30 deg`/retorno queda conseguida; procede
la bateria completa GT_FORCED.

## 365 - bateria completa de consignas GT_FORCED

Resultado: **CONSEGUIDA**.

Se repitio la bateria original de 361 con el controlador filtrado: espera de
tracking, hover frente al fiducial bajo GT forzado, `+30 deg`, `-30 deg`,
solicitud `+90 deg` saturada al limite `+70 deg` y retorno neutral. Todas las
puertas completaron:

- `+30 deg`: `29.006 deg`, `0.012866 rad/s`;
- `-30 deg`: `-29.034 deg`, `-0.012398 rad/s`;
- `+90 deg`: comando aceptado como `1.221730 rad`, `saturated=true`, y estado
  final `69.011 deg`, `0.012728 rad/s`;
- retorno: `0.976 deg`, `-0.012554 rad/s`.

No hubo timeout, NaN ni estado no finito. El hover GT tambien completo antes
de mover el joint. El escenario termino `success=true` en 119 s; 97 muestras
de recursos, minimo 5383.7 MiB, guard inactivo y sin PSI de memoria. Queda por
validar especificamente movimiento del dron con pitch no neutral y estabilidad
de Fase 5 antes del cierre agregado.

## 366 - movimiento del dron con pitch no neutral

Resultado: **CONSEGUIDA**.

Bajo `GT_FORCED`, el joint alcanzo primero `+30 deg` (`29.010 deg`,
`0.012401 rad/s`) y el dron completo un goal absoluto. Despues alcanzo
`-30 deg` (`-29.035 deg`, `-0.012260 rad/s`) y completo un segundo goal
absoluto. Ambos goals fueron aceptados con `source=gt_forced` y el mux activo
conservo el bloqueo de fuente `gt_forced`. El retorno neutral termino en
`-0.995 deg`, `0.013862 rad/s`. No aparecieron NaN, `NONFINITE`, timeout ni
fallo de paso.

El escenario termino `success=true` en 106 s. Se registraron 87 muestras de
recursos, minimo 5388.9 MiB, guard inactivo y sin PSI de memoria. Esto valida
que el pequeño torque del servo no impide el control de traslacion del dron y
que Fase 5 con autoridad GT mantiene los goals durante pitch no neutral.

## Regresiones finales

Despues del cambio final pasan:

- estimador/predictores F5: `117/117`;
- detector fiducial: `3/3`;
- metricas de evidencia visual: `3/3`;
- contratos Python dirigidos de simulacion/arquitectura/metricas: `35/35`.

La suite Python completa dio `52/54`. Los dos fallos son heredados y ajenos a
1J: divergencia previa entre dos copias del escenario de rodeo y un test de
perfil debug que no incluye tres claves F5 ya existentes. No se modificaron
para ocultar el resultado.

## Conclusion agregada 1J

Fase 1J queda **CONSEGUIDA BAJO GT_FORCED**. Existe un unico
`stereo_pitch_joint`, el control actua exclusivamente por torque, respeta
limites configurables y satura consignas fuera de rango. El filtro configurable
de velocidad evita realimentar impulsos numericos sin ocultar la medida cruda.
La extrinseca body-camera se publica con el angulo y timestamp simulados, y las
pruebas validan consignas, limites, retorno y movimiento del dron con pitch no
neutral. La repeticion con autoridad ORB pertenece a la validacion posterior
prevista tras estabilizar/retomar Fase 5.

## Auditoria transversal Fases 1-5 posterior al cierre fisico

La revision del usuario detecta correctamente que el cierre anterior solo
cubria modelo/control y navegacion GT. Estado corregido de 1J: **PARCIAL** hasta
completar el bloque transversal.

### Fase 1 - sensores estereo

La pareja izquierda/derecha permanece rigida dentro de `stereo_rig`; baseline,
intrinsecos, rectificacion y sincronizacion no dependen del pitch y no requieren
conversion body. La TF dinamica publicada representa body->camara optica
izquierda, que es la camara de referencia ORB. Pendiente: contrato/test de
frames y stamps para ambas camaras, y declarar incompatible o fallar pronto si
se solicita `camera_pitch_enabled=true` con el camino mono, que no crea el rig.

### Fase 2 - configuracion y despliegue

El launch conmuta `body_camera_transform_mode=tf` cuando activa pitch y usa
frames namespaced. Las tres copias `calibration*.yaml` siguen siendo una
extrinseca frontal estatica valida solo como fallback con pitch desactivado;
deben documentarse y protegerse para que ningun despliegue movil las consuma
por error. La documentacion previa sobre frame IDs y masa cero estaba obsoleta.

### Fase 3 - mapa sparse global

ORB, `OrbKeyFrame.pose`, MapPoints, covisibilidad, loops, optimizacion y fusion
trabajan en pose de camara. Esa semantica sigue siendo correcta con una camara
movil: no se debe convertir el mapa ni los KFs a body. `orbslam/pose_local`
tambien es pose de camara y no tiene consumidores externos localizados.
Pendiente solo regresion live con pitch para confirmar tracking/mapa/loops; no
se identifica cambio de codigo obligatorio.

### Fase 4 - fiduciales

`camera_t_tag`, pose del KF y `world_T_camera` comparten frame de camara. El
servidor no usa `body_T_camera`, por lo que añadir otra compensacion seria un
error. Pendiente una prueba live con fiduciales visibles a pitch no neutral que
valide deteccion, batch KF, anchor/revisita y ausencia de GT. No se identifica
cambio geometrico obligatorio.

### Fase 5 - pose y dinamica del cuerpo

`StereoSlamNode::ResolveBodyTCamera()` consulta body->camara al stamp exacto de
imagen; `PublishNavigationState()` compone `O_T_B` y `W_T_B` antes de alimentar
predictor/control. Si falta TF, falla cerrado e invalida body sin alterar mapa,
KFs o fiduciales. Esta arquitectura es correcta, pero carece de test dirigido
con extrinseca variable y las pruebas 364-366 usaron autoridad GT, de modo que
no validan la salida ORB body.

Hay dos desajustes dinamicos reales:

1. Gazebo añade `0.04 kg` moviles (`0.02 kg` de rig y dos camaras de `0.01 kg`),
   mientras control y `BodyThrustDynamicPredictor` conservan masa `1.4 kg`.
2. `BodyTorqueDynamicPredictor` solo consume el torque del controlador y una
   inercia body fija. No observa el torque de reaccion del servo ni la inercia
   compuesta dependiente del angulo del rig.

El error de masa es aproximadamente `2.86 %`; el efecto angular debe medirse
antes de decidir entre modelarlo como perturbacion pequena, publicar torque del
servo e incluirlo, o reducir masa/inercia movil manteniendo estabilidad
numerica. Esta decision afecta directamente la estimacion dinamica de F5.

### Trabajo necesario para cerrar el bloque

- tests unitarios/contractuales de composicion body-camera para `q=0`, `+/-30`
  y limites, con stamp exacto y fallo cerrado;
- prueba ORB en sombra comparada con body GT mientras solo se mueve el pitch;
- prueba ORB productiva de trayectoria con pitch fijo y cambio de pitch;
- prueba F4 con fiducial visible a pitch no neutral;
- medir el residual de masa/inercia/torque del servo y cerrar la politica F5;
- regresion con pitch desactivado para demostrar equivalencia con extrinseca
  estatica.

No se autoriza aun modificar runtime: primero debe acordarse la politica
dinamica de F5 y el criterio cuantitativo de las pruebas ORB/body.

### Correccion tras rastreo exhaustivo de matrices rigidas

Ante la duda del usuario se amplio la busqueda a nombres alternativos y scripts
de vision. La afirmacion correcta no es que F1-F4 carezcan por completo de
matrices rigidas, sino que el **pipeline productivo actual** no las consume
fuera de `StereoSlamNode`.

Las tres copias `calibration*.yaml` solo son cargadas por
`orbslam_use.launch.py` para el wrapper; Servidor y Simulacion mantienen
replicas contractuales comprobadas por tests, pero no componen con ellas la
ruta F3/F4. ORB estereo no inercial trabaja directamente en camara; los `Tbc`
de configuraciones inerciales de ejemplo no pertenecen al launch activo.

Sin embargo, numerosos scripts de `dron_individual/src/vision/` contienen
`camara2cuerpo=[0.1,0.03,0.03]` y matrices fijas, entre ellos nubes, planos,
ICP, TSDF y prototipos de mapeado. No se instalan como ejecutables Python, no
se lanzan y la documentacion los clasifica como experimentales/legacy; por
eso no afectan F1-F4 productivas. Si F6 rescata cualquiera de ellos, la matriz
fija debe eliminarse y sustituirse por body->camara al stamp de cada imagen o
nube. Esta deuda entra explicitamente en la preparacion F6.

## 367 y 367R - handoff ORB previo al barrido de limites

`367` es **INVALIDA DE INFRAESTRUCTURA**: el runner no pudo abrir la ruta YAML
relativa y no ejecuto pasos. El mux si habia arrancado en `GT_FALLBACK`.

`367R` repitio mecanicamente con ruta absoluta y es **NO CONSEGUIDA ANTES DEL
BARRIDO**. La llegada se bloqueo correctamente con `source=3` (`GT_FALLBACK`)
y completo. Tras ocho segundos, el segundo goal abrio la frontera, pero volvio
a bloquear `source=3`; `orb_authority_confirmed` expiro a 30 s. No se envio
ninguna consigna de pitch.

ORB conservaba tracking `2` y referencias locales validas, pero no obtuvo
anchor global. La reduccion fiducial posterior corrige el diagnostico inicial:
el wrapper proceso decenas de KFs con candidatos `undecoded`, pero todas dieron
`decoded=0 valid=0`; no publico batch fiducial ni el servidor creo hard anchor.
Todas las consultas globales respondieron `STATUS_PENDING=1`, nunca aparecio
`F5E-GLOBAL-AUTHORITY accepted=true` ni `F5H-GRAVITY-O-INIT`.

Torque y thrust **si se recibieron**: los buffers alcanzaron aproximadamente 26
muestras cada uno. `torque_coverage=EMPTY` y `thrust_coverage=EMPTY` eran valores
por defecto porque, sin gravedad O autoritativa, el calculo midpoint no llegaba
a ejecutarse. La salida body quedo invalidada por `DYNAMIC_BASE_NOT_READY` con
`gravity_valid=false`. Al abrir el segundo goal el estado GT era
aproximadamente `(-0.018,-6.963,0.805)`, lejos del target `[0,-10,1]`, aunque
el action anterior habia finalizado. La prueba no permite atribuir nada al
torque del servo: el joint nunca se movio.

Runtime codigo 1 en 90 s, 73 muestras, minimo 4857.3 MiB, guard inactivo y sin
PSI. Se aplica STOP antes de corregir Fase 5 o repetir el barrido.

## 368 - aproximacion determinista a `(0,-10,1,90 deg)`

Resultado: **NO CONSEGUIDA ANTES DEL ANCHOR**.

Se fijo el spawn de `dron_1` en `(-1,-10,0.025), yaw=90 deg` y se envio
explicitamente el goal world `(0,-10,1), yaw=90 deg`, bloqueado en
`GT_FALLBACK`. El action termino `success=true` al agotar sus 12 s, pero ese
resultado solo certifica el final temporal de la trayectoria. Al abrir el
siguiente goal, el estado O/GT continuo era aproximadamente
`(-0.003397,-6.973328,0.811563)`: X convergio y Z subio, pero el dron derivo
unos 3 m en Y y no llego al punto de observacion solicitado.

El detector proceso KFs durante y despues de la aproximacion, siempre con
`decoded=0 valid=0`; no hubo batch, hard anchor, autoridad global ni handoff a
ORB. El segundo goal tambien bloqueo GT_FALLBACK y el runner expiro esperando
`orb_authority_confirmed`. No se envio ninguna consigna pitch.

Runtime codigo 1 en 91 s, 73 muestras, minimo 4924.7 MiB, guard inactivo y sin
PSI. Hace falta acordar una aproximacion GT correctiva con verificacion real de
error antes de volver a intentar anchor/handoff.

## 369 - fuente por goal y barrido con fallback

Resultado: **PARCIAL**.

- objetivo: llegar a `(0,-10,1), yaw=90 deg` con GT_FORCED, restaurar la
  politica global ORB mediante `None` y barrer pitch `+70/-70/+70 deg`;
- cambios: selector `navigation_source` por goal en runner/mux y escenario 367;
- builds: `dron_individual` codigo 0 en 16.9 s, `simulacion_dron` codigo 0 en
  13.8 s y recompilacion diagnostica de `dron_individual` en 17.5 s;
- runtime: codigo 0, `success=true`, 137 s, 107 muestras, minimo 4647.5 MiB,
  guard inactivo y memory PSI 0;
- positivo: primer goal `requested=gt`, arranque `source=4`; segundo
  `requested=none`, politica efectiva ORB y arranque `source=3` GT_FALLBACK por
  falta de anchor. Pitch en `69.013 deg / 0.010180 rad/s`,
  `-69.012 / -0.012902` y `69.006 / 0.012800`, sin timeout ni no finitos;
- negativo y causalidad corregida: el fallo primario fue que GT_FORCED no llevo
  el dron al punto de observacion. La pose al segundo goal fue
  `(-0.004969,-6.973856,0.808545)`; al no quedar frente al fiducial no hubo
  deteccion/anchor, y por esa causa tampoco autoridad ORB. El segundo goal uso
  GT_FALLBACK como consecuencia, no como causa del fallo de llegada;
- reduccion: lifecycle, selector, source lock, inicio atomico, autoridad,
  fiducial/gravedad y pitch;
- conclusion: selector y barrido **CONSEGUIDOS**; seguimiento de la primera
  trayectoria GT y, en consecuencia, prueba de handoff ORB **NO CONSEGUIDOS**.
  1J sigue **PARCIAL**;
- siguiente paso: diagnosticar la deriva Y bajo GT_FORCED y acordar un gate
  cuantitativo de llegada antes de repetir anchor/handoff.

Revision conversada posterior: GT si fue seleccionado y consumido
correctamente (`source=4`), con estado inicial
`(-1.000006,-9.999988,0.024995)` y target world `(0,-10,1)`. La formula vigente
de `gen_tray` usa `absolute_target` para los ejes absolutos, por lo que no hay
evidencia de que el goal se transformase a `Y=-6.97`. La telemetria de control
muestra un fallo de seguimiento: `position_error_norm` crece desde `0.028 m`
hasta aproximadamente `2.97 m`, con `attitude_error_norm` alrededor de
`0.3-0.46 rad`. Por tanto, el problema no es disponibilidad/seleccion de GT,
sino el lazo fisico que no sigue la referencia; el runner lo oculto al tratar
el final temporal de la action como llegada.

## 2026-09-01 - Subfase 1J - cierre de regresion fisica y barrido ORB

- objetivo: restaurar vuelo GT, validar llegada real con pitch fijo/movil y
  repetir GT -> ORB/fallback -> barrido;
- cambios: topologia condicional y masa del rig; gate
  `wait_for_navigation_pose`; dependencias CMake y escenarios 367/370/371;
- build: primer intento fallo por include Eigen ausente; correccion mecanica y
  builds posteriores de `simulacion_dron` codigo 0;
- 370, **NO CONSEGUIDA**: joint fixed sin servo, gate timeout en
  `(0.002421,-6.979690,0.813943)`, error `3.026036 m`, torque `~0.0392 Nm`;
- causa: `0.04 kg` a `0.10 m` producen `0.03924 Nm`, cifra coincidente. Se
  cambio a `1e-5 kg` por camara y rig, total `3e-5 kg`, sin tocar control;
- 370R, **INVALIDA DE INFRAESTRUCTURA**: ruta YAML relativa, sin goal;
- 370R2, **CONSEGUIDA**: joint fixed; error `0.015278 m`, yaw `0.000131 deg`,
  torque tardio `0.000024-0.000025 Nm`;
- 371, **CONSEGUIDA**: revolute/servo neutral; error `0.016051 m`, yaw
  `0.013594 deg`, torque `~0.000029 Nm`;
- 372, **CONSEGUIDA PARA 1J**: gate GT `0.015747 m`; segundo goal inicia con
  ORB real `source=1`, anchored; barrido `69.007/-69.013/69.035 deg`; maximos
  de posicion `0.077636 m`, actitud `0.101306 rad` y torque `0.008334 Nm`, sin
  NaN ni fallo del escenario;
- limitacion: ORB pierde tracking durante el primer giro y pasa al
  `GT_FALLBACK`; queda como problema visual de Fase 5/6, no fisico de 1J;
- reduccion: runner/gates, fuentes, inicio atomico, anchor, pitch, control y no
  finitos;
- conclusion agregada: **CONSEGUIDA**. No hay simulacion activa;
- siguiente paso: preparar el ciclo Fases 6/7 con la deuda de tracking ORB ante
  cambios grandes de pitch como entrada explicita.
