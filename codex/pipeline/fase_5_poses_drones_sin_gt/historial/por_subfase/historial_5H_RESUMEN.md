# Historial 5H - Resumen

Estado agregado: `PARCIAL`; la prueba 252 corrige la extrinseca y completa la
vuelta, pero revela falta de suavidad ORB. Las iteraciones hasta 259 aislan el
fallo angular local. El redisenio raw/bias y su politica de deadband/decay pasan
37/37 GTests. El hover 262 mejora a unos 5.92 s en ORB y elimina el bias, pero
`omega_motion` aun oscila y activa fallback. La prueba 263 queda como
`DATOS_INSUFICIENTES` por doble reloj. La 264 usa el puente corregido y consigue
el diagnostico: raw llega con unos 80 ms, pero la pose/control angular queda
fuera de fase y `tau_er` inyecta energia en la oscilacion.

Actualizacion 321: la carrera authority/goal queda eliminada mediante
confirmacion transient-local antes del goal. 321B, con `p_ORB+v_GT` y angular
ORB, es estable (`ep/v/er` max `0.159/0.046/0.027`) y no usa fallback; 321AR y
321D divergen al conservar `v_ORB` (`ep` max `1.604/1.761 m`). Esto aisla la
velocidad lineal ORB como causa principal. 321C cae a fallback y no valida por
si sola el angular. Diagnostico `CONSEGUIDO`; hover ORB completo aun
`NO CONSEGUIDO` y 5H permanece `PARCIAL`.

Actualizacion 322/323: con GT gobernando y ORB dynamic integro en shadow se
obtienen 907 medidas settled validas. `v_mid` tiene RMSE `0.01984 m/s`, la
proyeccion THREE_SAMPLE sube a `0.03457` frente a `0.01988` del TWO_SAMPLE
paralelo, y la propagacion hasta now alcanza `0.43308 m/s`. Se demuestra
`A_HAT_AMPLIFICATION` y una degradacion adicional dominante en
`DYNAMIC_PROPAGATION`; no es principalmente DEGRADED_DT ni reference KF.
Diagnostico `CONSEGUIDO`, salida productiva intacta y STOP antes de corregir.

Actualizacion 324/325: la auditoria confirma que la dinamica sumaba gravedad
`-Z` directamente en O. `g_O=O_R_W*g_W`, congelada desde la primera autoridad
global del epoch, reduce `v_dynamic_now` de `0.43308` a
`0.03583/0.03707 m/s`; gain `12.53 -> 0.992/1.003`. Las dos ejecuciones
completan con tracking y autoridad GT en shadow. `GRAVITY_FRAME CONFIRMADO` y
`DYNAMIC_PROPAGATION CORREGIDA`; THREE_SAMPLE sigue pendiente y 5H `PARCIAL`.

Estado tecnico vigente:

- `gen_tray` y `control_calcular_fuerzas` consumen el estado comun; GT no es
  entrada directa;
- handshake namespaced `control/set_trajectory_active` bloquea la fuente antes
  del primer setpoint y abre la frontera solo al terminar;
- un frame absoluto cacheado solo vale en su `map_epoch`;
- RViz2 dibuja `o_t_body`, la pose exacta consumida por el controlador, con
  etiqueta `[ORB]/[GT]`; no depende de `global_valid` y conserva la ultima pose
  consumible ante muestras transitorias invalidas;
- prueba 243: scenario `success=true`, 17/17 pasos, 22/22 goals, 44/44
  handshakes, cero fallos de handshake y cero entradas ORB dentro de lock GT.

La prueba 246 demuestra que el alcance del lock es insuficiente: al terminar
cada action goal se permite `GT -> ORB`, mientras el controlador conserva la
ultima consigna GT. La politica acordada debe aplicarse a toda la mision YAML:
fuente inicial retenida hasta el final, salvo `ORB -> GT` por perdida, que queda
retenido durante el resto de la mision.

La alternativa minima de reset fallo en 247 y la conmutacion atomica en 248. El
handoff angular de 249 demuestra `er=ew=0`, fuerza de hover y torque cero al
inicio. 250/251 confirman que X world llega casi exactamente como Z control. La
causa queda localizada: `W_T_C` fiducial es correcto, pero la rotacion cargada
como `body_T_camera` (`RPY=0,-90,90`) es en realidad `camera_T_body`; el wrapper
la vuelve a invertir al formar `W_T_B`. El resultado permuta los ejes y explica
por completo suelo/subida. `use_camera_optical_frame_convention=true` no se
consume y la alineacion del mux con GT ocultaba el mismo defecto en la pose O
mostrada por RViz2.

La correccion `B_T_C` real (`RPY=-90,0,-90`) pasa los tres builds, el contrato
de replicas y la prueba 252: 17/17 pasos, 22/22 goals y exit 0. La revision
visual confirma una mejora grande y elimina la proyeccion X world sobre Z.

El movimiento restante tiene dos causas demostradas. El mux recibe ORB a 20 Hz
y deriva velocidad por diferencias finitas sin filtro; el control PD corre a
50 Hz y amplifica saltos de pose de hasta 0.208 m. Ademas hubo 8/14 estados
`RECENTLY_LOST` y 3/3 `LOST`: `ORB -> GT_FALLBACK` hace `ResetToSource` dentro
del goal, pero el feedback siguiente conserva la trayectoria del frame ORB
anterior. Algunos arranques copiaron `v0` espurias cercanas a 1.1 m/s.

Los ejes RViz2 y los KFs no comparten necesariamente marco: los primeros son
`o_t_body` de control y pueden estar etiquetados `[GT]`; los segundos viven en W
global optimizado. La discrepancia visual aislada no demuestra un error de KF.
No atribuir los tirones a la optimizacion de Fase 3.

Persisten dos `[F3L-HARD-FAILURE]` del optimizador, fuera del alcance de Fase 5.
Las metricas GT-estimada se conservan solo como observacion externa de deriva.

La prueba 253 queda `NO CONSEGUIDA` e interrumpida a los 103 s. No hubo cambio
de fuente: ambos drones usaron `GT_FALLBACK` desde el arranque. El predictor
alpha-beta aplicado tambien a orientacion GT limito innovaciones angulares de
hasta unos 3 rad y acumulo 1096/1612 medidas limitadas en drone1 y 863/1244 en
drone2. El retraso de actitud entra en el lazo de torque y provoca realimentacion
inestable; drone1 termino con innovaciones lineales de hasta 40.9 m. No repetir
253 con esa configuracion ni atribuir el fallo a la alineacion ORB/GT.

La prueba 254 traslada predictor/filtros a `orbslam3_ros2` y deja GT exacto.
Builds y 3/3 tests focales pasan, pero la simulacion se interrumpe tras 13/17
pasos por divergencia en giros bajo ORB. Los handoffs empiezan con salto cero;
el defecto aparece despues porque la orientacion medida acepta pasos de hasta
`0.279/0.273 rad` por frame mientras la velocidad angular queda limitada a
`1.5 rad/s`. La correccion siguiente debe hacer coherentes pose y velocidad
angular ORB; no debe tocar GT, ganancias, optimizador ni YAML. Los picos no
ocurren en el reanclaje de reference KF, que registra paso cero, sino
`0.042-0.250 s` despues dentro de la referencia ya activa; se agrupan durante
cambios rapidos de KF, sin demostrar que el reanclaje sea la causa directa.

La prueba 255 añade probation de tres frames, timeout de seis y gate angular
coherente. Builds y 13/13 GTests finales pasan. Antes de la interrupcion completa
7/17 pasos: el gate rechaza innovaciones de `0.081-0.214 rad`, registra 10
timeouts y conmuta a GT con salto cero, evitando publicar el outlier de 254.
La revision visual confirma que ORB funciona brevemente y despues el movimiento
se vuelve inestable. La cronologia descarta la optimizacion global como causa:
solo el primer episodio coincide con un solve; los siguientes comienzan 8.5 y
47 s despues del ultimo commit. Los tres estan precedidos por churn de reference
KF, timeout del gate o innovaciones angulares dentro de la referencia activa, y
despues llega la perdida real de tracking. Queda `PARCIAL`: el gate protege el
control de los outliers grandes, pero no consigue una O local ORB sostenida.

La prueba 256 generaliza la probation a una cadena geometrica multi-KF y usa un
estimador SE(3) coherente; los builds y 15/15 GTests finales pasan. La
simulacion queda `NO CONSEGUIDA`: ambos handoffs entran con salto cero y
`er=ew=0`, pero drone2 acepta una innovacion angular de `0.125261 rad` y registra
`rotation_step_rad=0.119002`; pierde tracking `0.793 s` despues. No hubo
optimizacion global en ese intervalo. Revision posterior del codigo: ese
`rotation_step` compara la medida raw con la pose filtrada previa y no es el
salto publicado. La causalidad sigue siendo una hipotesis hasta instrumentar
correccion aplicada, pose/omega publicadas, error/torque y tracking. El siguiente
acuerdo debe introducir confirmacion temporal de innovaciones moderadas y esa
telemetria, sin tocar GT, mux, ganancias ni W.

La iteracion posterior implementa esa probation y pasa 21/21 GTests. La prueba
257 no arranca por una ruta YAML relativa y se conserva como fallo de
infraestructura. La 258 consigue la etapa 1: 11/11 pasos, 7/7 goals y control
100% GT mientras observa ORB. ORB mantiene tracking en hover/X/Y/Z/yaw lento,
pero lo pierde durante yaw rapido y lo recupera en epoch 1; no hubo evento
moderado/excesivo que vincule la probation con esa perdida. La telemetria de
edad se corrige para no mezclar reloj de imagen y reloj ROS. La etapa 2 se
ejecuta despues como prueba 259 y queda documentada a continuacion.

La prueba 259 deja la iteracion `NO CONSEGUIDA`: el handoff de hover entra con
salto cero, pero ORB gobierna solo 227 muestras antes de fallback y tracking 3.
La oscilacion y el torque crecen antes del primer pending; una medida tratada
como pequena publica `0.058777 rad`. La probation confirma despues un residual
creciente, publica hasta `0.075 rad/paso` y precede dos rechazos excesivos. El
umbral SMALL dinamico dependiente de `dt` y la consistencia basada en residual
son las limitaciones vigentes. Por acuerdo se detienen etapas 3-8.

La prueba 260 repite la etapa 1 con GT gobernando y queda `CONSEGUIDA` como
calibracion raw: 11/11 pasos y 7/7 goals. En drone1, los maximos
step/omega/alpha son `0.00448/0.0878/0.823` en hover,
`0.02976/0.2247/4.496` en yaw lento, `0.04693/0.4386/7.728` en yaw rapido y
`0.09243/0.6206/6.708` en 180 grados. De ahi se fijan `0.12 rad`, `1 rad/s` y
`10 rad/s2`, con dt GOOD hasta `0.075 s` y DEGRADED hasta `0.20 s`.

El redisenio separa `omega_motion` y `omega_bias`, fija SMALL y pasa 27/27
GTests. Los builds de `orbslam3`, `dron_individual` y `simulacion_dron` son
correctos. La prueba 261 ejecuta el mismo hover ORB de 259: handoff y primer
error cero, pero ORB gobierna solo unos `3.82 s`. Aparecen 15 SMALL, siete
pending, siete confirmed y tres `REJECTED_EXCESSIVE`; estos invalidan el
estimador y causan fallback mientras tracking aun es 2. La perdida 2->3 llega
unos `10.25 s` despues, por lo que no explica el fallback inicial.

Diagnostico 261: `omega_bias` responde ya a residuos SMALL y puede sembrar la
oscilacion; el lazo mueve fisicamente el dron y ese movimiento se acepta como
raw plausible. `omega_motion` alcanza aproximadamente `0.629 rad/s`, frente al
bias acotado a `0.080 rad/s`, y domina `omega_total`. Tras rechazar raw, retener
el ultimo movimiento alto mantiene alrededor de `0.56 rad/s` durante las tres
muestras de rechazo. El redisenio conceptual separa canales, pero no rompe la
realimentacion. La etapa 3 no se ejecuta y 5H permanece `PARCIAL`.

Correccion y prueba 262: el bias usa deadband `0.005/0.002 rad`, confirmacion
3/4, supresion por movimiento `0.10/0.05 rad/s` y raw rechazado decae a
`4 rad/s2`. Los tres builds y 37/37 GTests son correctos. En el hover,
`omega_bias` permanece exactamente a cero y el decay funciona, validando ambos
cambios. ORB gobierna unos `5.92 s`, pero `omega_motion` alcanza
aproximadamente `0.617 rad/s`; tres residuos excesivos fuerzan fallback y
tracking 2->3 ocurre ~`0.54 s` despues. La conclusion sigue `NO CONSEGUIDA` y
5H `PARCIAL`: el siguiente diagnostico es latencia/fase del canal angular
visual y el lazo de control, no una nueva recalibracion sin evidencia.

Prueba 263: builds y tests correctos; mismo hover, runner `success=true` y
recursos sanos. Drone1 dejó 443 medidas, 3922 publicaciones y 2740 ticks de
control, pero ninguna fila pudo sincronizarse honestamente con GT porque su
header está en tiempo Gazebo y receive/control en tiempo ROS. Conclusión:
`DATOS INSUFICIENTES`; no confirma ni descarta anti-damping. La captura ahora
añade `gt_receive_stamp` y el analizador mapea ambos relojes.

Prueba 264: repeticion sin cambios funcionales, runner `success=true`, recursos
sanos y 323 ciclos ORB sincronizados durante `6.44 s`. Raw sigue al GT en x/y
con correlacion `0.984/0.982` y lag `~0.08 s`; tracking permanece en 2 hasta el
fallback. El damping `tau_ew` es contrario al ideal en `34.0 %` de los ciclos,
pero netamente disipativo. La evidencia dominante es `tau_er`: trabajo positivo
en `80.9 %` del tramo post-handoff e inyeccion de `+0.005173 J` al reprocesar
con el analizador de 265, casi anulando el damping. Diagnostico `CONSEGUIDO`,
hover `NO CONSEGUIDO`; 5H sigue `PARCIAL`.
No ejecutar etapa 3. El siguiente diseño debe corregir la fase de la pose
angular usada por control, no tocar GT/mux ni recalibrar raw sin más evidencia.

Prueba 265: corrige exclusivamente el horizonte temporal; edad local media
`51.5 ms`, horizonte `43.2 ms` y clamp `10.4 %`, con 40/40 GTests y 5/5 tests
del analizador. La semantica queda arreglada, pero el hover empeora: ORB dura
`5.56 s`, `er` llega a `0.666 rad` y `tau_er` inyecta `+0.160266 J`; el torque
total aporta `+0.145081 J`. La separacion `visual_q -> base_q` crece hasta
`0.339 rad`, mientras `base_q -> predicted_q` es pequeña. Esto localiza el
desfase dominante en `pose_` base integrada y revela que el antiguo horizonte
fijo de `0.10 s` compensaba parcialmente el retraso. Resultado 265
`NO CONSEGUIDO`; 5H permanece `PARCIAL`, sin etapa 3. El siguiente cambio debe
debatir fusion/anclaje visual de la orientacion base, sin GT y sin implementarlo
bajo la autorizacion ya consumida.

Prueba 266: el reanclaje visual SMALL y la separacion pose/omega pasan tres
builds, 44/44 GTests y 7/7 tests del analizador. ORB mejora de `5.56` a
`8.06 s`; en la ventana comun, `tau_er` cae de `+0.153559` a `+0.002067 J` y
el torque total pasa de `+0.138374` a `-0.001945 J`. SMALL deja error
visual-base after exactamente cero. Sin embargo, desde `+5.90 s` la correccion
MODERATE_CONFIRMED de `0.015 rad` deja residual, alterna con PREDICT_ONLY y raw
se rechaza a `+7.50 s`; fallback llega a `+8.08 s` y tracking 3 despues, a
`+8.68 s`. Resultado funcional `NO CONSEGUIDO`, con hipotesis de anclaje
claramente validada; 5H sigue `PARCIAL` y etapa 3 detenida. Conservar SMALL y
debatir solo la politica moderada antes de otra ejecucion.

Prueba 267: el anclaje completo de MODERATE_CONFIRMED deja error after cero,
pero el hover vuelve a fallar a `5.56 s`; `tau_er=+0.030448 J` y energia total
`+0.015622 J` en ventana comun, peores que 266. Antes del primer anclaje el
total aun era disipativo; despues se alcanza `+0.018489 J` en `1.22 s`, frente
a `2.06 s` en 266. La ejecucion descubre ademas un confirmed anchor con raw
rechazado. El codigo final exige raw plausible y pasa build y 46/46 GTests,
pero esta reparacion no se ha simulado. Resultado 267 `NO CONSEGUIDA`; no se
ejecutan 268 ni etapa 3. 5H permanece `PARCIAL` y el siguiente comportamiento
requiere nuevo acuerdo.

Prueba 268: valida en simulacion el gate raw final, pero no el hover. El unico
MODERATE_CONFIRMED_ANCHOR usa raw plausible, corrige `0.057317 rad` y deja
error after cero. Antes de ese instante el total es disipativo
(`-0.001356 J`); en los `0.88 s` posteriores hasta fallback acumula
`+0.046416 J`, y raw empieza a rechazarse unos `0.24 s` despues. ORB dura
`5.72 s`, fallback `+5.74 s` y tracking no OK `+5.92 s`. Resultado
`NO CONSEGUIDA`; no se ejecutan 269 ni etapa 3. El bug raw de 267 queda
descartado como causa principal y el anclaje completo inmediato queda
contraindicado; el residual gradual `Delta_target` requiere nueva autorizacion.

Pruebas 269-272: diagnostico causal conseguido. A/GT 50 Hz mantiene hover
estable. B/GT perfecto 20 Hz por el predictor y publicacion 50 Hz completa el
escenario, pero queda girando a `0.1059 rad/s` y acumula energia total positiva
`+0.01088 J`. C/+80 ms y D/timing 268 rechazan el segundo goal; D alcanza
`er=0.586 rad` y `+0.02532 J` en 1.62 s. El fallo se reproduce sin error
geometrico ORB: la causa principal esta en 20->50 Hz y la coherencia temporal
pose/omega del predictor, agravada por delay/jitter. 5H sigue `PARCIAL`.

Pruebas 273-275: E/F/G completan el escenario y tienen energia total
ligeramente disipativa. E estabiliza el predictor sustituyendo solo
`omega_motion` por omega GT; F valida hold angular 20 Hz y G valida
`exp(omega*dt)*R` con pose/omega coherentes. El mismatch direccional baja a
`0.41/0.29/0.094 %`. Diagnostico `CONSEGUIDO`, opcion A: la causa principal es
la derivacion/filtrado de `omega_motion`, no el hold ni la extrapolacion. 5H
permanece `PARCIAL` hasta corregir la ruta ORB real.

Pruebas 276-277: `omega_motion` usa ya derivacion causal de tres poses,
aceleracion entre midpoints y proyeccion hasta el timestamp visual sin
pasa-bajos. Ambas ejecuciones con pose GT perfecta a 20 Hz completan el hover,
sin fallback y con energia total negativa. RMSE `0.00374/0.00304 rad/s`, frente
a `0.43338` en 270/B. La solucion queda validada y reproducible en laboratorio;
5H sigue `PARCIAL` porque faltan delay/jitter y ORB real.

Prueba 278: el mismo estimador con 80 ms de delay y timestamp fisico original
falla. Edad visual media `0.1129 s`, clamp `72.2 %`, RMSE `1.447 rad/s` y
energia total `+0.02884 J`; sin fallback ni tracking loss. Se detiene la
bateria y 279-281 no se ejecutan. Falta compensar `t_k -> now`.

Pruebas 282/284: subir el horizonte diagnostico a `0.18 s` elimina el clamp,
pero 282 empeora a RMSE `2.468 rad/s` y `+0.06897 J`. Propagar pose y omega con
`alpha_hat` en 284 reduce RMSE a `1.136 rad/s` y energia a `+0.02052 J`, pero
falla antes (`1.86 s`) y raw se rechaza desde `0.84 s`; los limites no se
activan. Diagnostico conseguido: clamp y aceleracion constante no son solucion
suficiente. 279-281 siguen detenidas y 5H `PARCIAL`.

Pruebas 285-287: bateria cruzada completa. `R_pred+omega_GT` y
`R_GT+omega_GT` gobiernan `13.12/13.06 s` y son disipativas; `R_GT+omega_pred`
solo `2.98 s`, mismatch `49.0 %` y `+0.04459 J`. Esto señala fuertemente a
`omega_pred`, pero 287 tambien falla y evita causalidad definitiva. El montaje
solo sustituyo R/omega; p/v lineales siguen retrasadas. Siguiente diagnostico:
fijar `p(now),v(now)` GT comunes. 279-281 siguen detenidas.

Pruebas 288-291: el cruce con p/v GT actuales cierra la causalidad pendiente.
288 y 290, ambas con omega predicha, fallan tras `2.54/5.18 s`, con RMSE
`1.811/2.024 rad/s` y energia total `+0.07979/+0.08887 J`. 289 conserva R
predicha pero usa omega GT y completa `54.46 s`; 291, estado GT completo,
completa `55.20 s`. Ambas son disipativas y su RMSE es
`0.003829/0.001255 rad/s`. El sanity pasa: la causa inmediata queda aislada en
`omega_pred(now)` bajo delay, no en R ni p/v. Diagnostico `CONSEGUIDO`; 5H
sigue `PARCIAL` hasta corregir omega sin GT y validar ORB real. 279-281 siguen
detenidas.

Prueba 292: se implementa un predictor rigido causal con torque body,
timestamps reales y `J=diag(1e-4)` compartida con el controlador; builds
correctos y 75/75 GTests. El primer torque no nulo, de solo unos
`(0.0038,-0.0053,0) Nm`, produce cientos de rad/s predichos mientras GT sigue
en torno a `1 rad/s`. El lazo escala a miles y `NaN` en decimas. La ecuacion e
integracion pasan sus tests, pero la J nominal no representa la planta
compuesta de Gazebo. 292 queda `NO CONSEGUIDA`; por STOP no se repite ni se
ejecutan 293-295. El modelo dinamico no esta validado, 279-281 permanecen
detenidas y 5H sigue `PARCIAL`.

Pruebas 314-317: el nuevo `CausalLinearVelocityEstimator` usa tres posiciones
visuales aceptadas y estima `v_hat(t_k)` sin realimentar correcciones ni
extrapolar hasta now. 314/315 con angular GT y 316/317 con estado completo
dinamico completan llegada y hover sin fallback, tracking loss ni huecos de
thrust. En 316/317 RMSE p queda `0.103/0.096 m`, RMSE v
`1.184/1.093 m/s`, RMSE angular `0.108/0.115 rad/s` y el trabajo angular total
es negativo. Un primer intento 315 sin modo diagnostico se conserva como
invalido y no cuenta. Bateria `CONSEGUIDA`; 5H sigue `PARCIAL` porque la clase
aun no alimenta `StereoSlamNode` productivo ni se ha probado con ORB real.

Pruebas 307-313: 307 y 308 fallan por separado con `v_pred` y `p_pred`, por lo
que el diagnostico es `P Y V`. La auditoria confirma thrust total `+Z_body`,
mixer sin saturacion y masa compartida de `1.4 kg`. El predictor translacional
con thrust sellado, gravedad, dt reales y `R_dynamic(t)` pasa 86/86 GTests.
309 y 313 completan usando p/v GT(t_k) solo como estado inicial, sin huecos de
fuerza. 310 falla al usar el p/v causal estimado vigente. Por STOP no se
ejecutan 311/312 ni 300-302. Pendiente: corregir y validar `v_hat(t_k)`.

Correccion post-292: la J compartida se sustituye por la inercia compuesta
`diag(0.00803107,0.00803107,0.015805) kg*m^2`. Con ella, 296 y su confirmacion
297 completan `54.64/55.12 s`; 293 y su confirmacion 298 completan
`54.62/54.72 s`; 294 y 295 completan `54.70/54.90 s`. RMSE omega queda entre
`0.00255` y `0.00557 rad/s`, mismatch entre cero y `0.777 %`, y las seis
ejecuciones tienen energia total negativa, sin fallback ni tracking no-OK.
Diagnostico definitivo: la J nominal causaba el colapso. Predictor dinamico
angular `VALIDADO EN LABORATORIO` con delay fijo. Correccion posterior: 295 no
tenia delay añadido (`31 ms` medios frente a `~110 ms` en 296-294), de modo que
el estado completo solo queda validado sin esos 80 ms. 5H sigue `PARCIAL` hasta
timing/jitter, estado completo bajo ese timing y ORB real.

La prueba 299 aplica al estado completo la traza determinista de periodos y
delays de 268. Falla tras `16.94 s` de gobierno, sin fallback, tracking no-OK
ni huecos de torque: edad media/maxima `0.115/0.200 s`, 48 intervalos
`DEGRADED_DT`, RMSE omega `0.3497 rad/s` y energia total `+0.0141 J`. El primer
rechazo raw ocurre a `+15.84 s`, cuando la divergencia fisica ya existe. Por el
STOP acordado no se ejecuta 300, no se integra ORB productivo y no se ejecutan
301-302. El estado completo bajo jitter queda `NO VALIDADO`; 5H sigue
`PARCIAL`.

La bateria 303-306 aisla el fallo restante. 303, con p/v GT y angular dinamica,
completa `54.54 s` con RMSE omega `0.00304 rad/s` y energia negativa. 305, con
p/v predichas y angular GT actual, falla tras `15.06 s`; 306 con GT completo
completa `54.60 s`. Conclusion: `PV PRINCIPAL`; el canal angular de 299 es
suficiente si p/v son correctas. 304 falla en `2.52 s` pese a GT interpolado en
`t_k`, bracket valido y torque cubierto, por lo que esa propagacion de omega GT
instantanea queda como limitacion secundaria no resuelta. 300-302 permanecen
detenidas y 5H sigue `PARCIAL`.

Integracion post-317: `StereoSlamNode` ya dispone de una ruta temporal
`dynamic` que reutiliza los estimadores causales, torque/thrust sellados y la
masa/J compartidas; `legacy` sigue siendo el default. Los tres paquetes
compilan, GTest 94/94 y analizador 8/8. La prueba 318 completa el escenario sin
fallback, pero registra un `F5H-DYNAMIC-MISSING` al arrancar un movimiento:
la unica muestra de torque era posterior a la base. Por el criterio acordado
318 queda `NO CONSEGUIDA` y se detienen 319/320. La ruta productiva no esta
validada y ORB real sigue sin ejecutarse.

Politica causal post-318: la cadena Gazebo demuestra cold start cero. Se añaden
seed explicito, cobertura `EMPTY/MISSING_PREFIX/FULL`, ZOH observable y
persistencia ante resets visuales; pasan 98/98 GTests y todos los builds. 318R
completa sin fallback, pero vuelve a registrar un missing: el recorte de 0.5 s
elimina el seed durante la espera larga y la primera orden queda 70 ms despues
de la base. Resultado `NO CONSEGUIDA`; 319R/320/321 detenidas. Falta conservar
una muestra predecesora ZOH durante la poda.

Poda ZOH post-318R: conservar una predecesora y las muestras recientes pasa
102/102 GTests. 318R2 y 319R completan sin missing, fallback ni tracking loss;
poda, cobertura y paridad quedan validadas. El primer 320 es invalido porque
el YAML sobrescribia el modo dynamic con legacy; corregida mecanicamente la
precedencia launch. 320R usa ya ORB dinamico real y no presenta missing, pero
ORB gobierna indebidamente la aproximacion y no sigue el objetivo: al terminar
queda cerca del suelo y con velocidad alta. Tracking se pierde brevemente
durante el hover y activa fallback. Integracion productiva `NO VALIDADA`;
321 no ejecutada.

Shadow/handoff post-320R: el modo diagnostico mantiene GT en aproximacion y
observa el mismo ORB dinamico hasta tracking, anchor y `1.5 s` estacionario.
320R2 es `INVALIDA` por ruta YAML relativa. 320R2R valida la frontera: pose y
rotacion saltan cero, velocidad `0.247 m/s`, tracking permanece `2` y no hay
fallback ni missing. Aun asi, el hover diverge hasta `~1.63 m` de error de
posicion y `~0.52 rad` angular. Resultado `NO CONSEGUIDA`; activacion prematura
descartada como causa suficiente, ORB productivo no validado y 321 detenida.

Correccion post-325: MIDPOINT_DYNAMIC sustituye productivamente a THREE_SAMPLE
mediante dos poses aceptadas, orientacion interpolada, alineacion de relojes y
propagacion causal con torque/thrust/gravedad O. Pasa tres builds, 116/116
GTests y 7/7 tests del analizador. 326/327 dan cobertura 100 % y muestran que
MIDPOINT empata con TWO_SAMPLE y mejora claramente a THREE_SAMPLE; 328/329 lo
validan productivamente en shadow con RMSE `0.02113/0.02460 m/s`.

330/331 validan el hover ORB real de forma reproducible durante
`34.78/35.30 s`: tracking OK, cero fallback/clamp, max error angular
`0.0674/0.0631 rad` y energia angular total negativa. Conclusiones vigentes:
`A_HAT_AMPLIFICATION CORREGIDA`, `LINEAR_VELOCITY_ESTIMATOR VALIDADO` y
`HOVER ORB REAL VALIDADO`. 5H sigue `PARCIAL` solo hasta probar movimiento y
la trayectoria representativa.

Bateria progresiva post-331: 332/333 validan y reproducen X 2 m con ORB real,
sin fallback ni tracking loss; error maximo de posicion `0.068/0.113 m` y
velocidad final `0.028/0.029 m/s`. La interpretacion de 334 se corrige tras la
observacion del usuario: el goal +Y cruza el fiducial 2 de `[0,-8.5,1]`, por
lo que la colision causa la perdida de tracking y el fallback. 334 queda
`INVALIDA`, no `NO CONSEGUIDA`; sus errores acotados se conservan como
evidencia medida, pero Y sigue sin evaluar. 335-343 no ejecutadas; 5H `PARCIAL`.

334R repite Y hacia `[0,-12,1]` sin obstaculo. Mantiene ORB `30.26 s`, tracking
2, fallback/missing/clamp cero y ep max `0.063 m`, pero no supera el frenado:
ev final `0.187 m/s` y RMSE/max ev en los ultimos 3 s
`0.174/0.334 m/s`, frente a GT `0.049/0.074 m/s`. Y queda `NO VALIDADO` por
velocidad ORB residual/oscilante; STOP mantiene 335-343 sin ejecutar.

334R2 repite visualmente el mismo escenario y agrava el resultado pese a
tracking sano y fallback cero: ep/ev final `0.213 m / 0.551 m/s`, max ev
`2.188 m/s` y RMSE ev final de 3 s `1.008 m/s`. El fallo de velocidad lateral
y frenado queda reproducido; Y sigue no validado y 335-343 detenidas.

Diagnostico de proximidad y continuacion: 334R3R/335R ejecutan X y despues +Y
a 2 m del fiducial y mas cerca de la pared. Ambas completan con tracking 2,
cero fallback/missing/clamp y posicion acotada; Y queda funcionalmente
reproducido aunque persiste una velocidad ORB residual final de
`0.108/0.111 m/s`. La diferencia frente a 334R/334R2 apoya que la cobertura
visual lejana degradaba la estimacion lateral.

336/337R validan y reproducen Z +0.5 m: max ep `0.051/0.046 m`, velocidad
final `0.015/0.018 m/s`, tracking continuo y cero fallback. La bateria se
detiene en 338 yaw: ORB gobierna solo `11.18 s`, alcanza max er `0.995 rad`,
RMSE omega `0.409 rad/s` y RMSE lineal `0.530 m/s`; tracking pasa `2->3` y el
mux activa GT fallback durante el giro. 338 queda `NO CONSEGUIDA`; 339-343 no
se ejecutan y 5H permanece `PARCIAL`.
