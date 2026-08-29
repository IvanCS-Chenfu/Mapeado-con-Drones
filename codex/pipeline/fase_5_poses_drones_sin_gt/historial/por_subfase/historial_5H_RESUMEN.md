# Historial 5H - Resumen

Estado agregado: `PARCIAL`; la prueba 252 corrige la extrinseca y completa la
vuelta, pero revela falta de suavidad ORB. Las iteraciones hasta 259 aislan el
fallo angular local. El redisenio raw/bias y su politica de deadband/decay pasan
37/37 GTests. El hover 262 mejora a unos 5.92 s en ORB y elimina el bias, pero
`omega_motion` aun oscila y activa fallback. La prueba 263 queda como
`DATOS_INSUFICIENTES` por doble reloj. La 264 usa el puente corregido y consigue
el diagnostico: raw llega con unos 80 ms, pero la pose/control angular queda
fuera de fase y `tau_er` inyecta energia en la oscilacion.

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
