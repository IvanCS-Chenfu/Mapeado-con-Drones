# Historial 5H - Resumen

Estado agregado: `PARCIAL`; la prueba 252 corrige la extrinseca y completa la
vuelta, pero revela falta de suavidad ORB. Las pruebas 253-256 aislan el fallo
en la actitud ORB local; la ultima correccion compila, pero colapsa al aceptar
una innovacion angular moderada sin confirmacion temporal.

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
edad se corrige para no mezclar reloj de imagen y reloj ROS. Etapa 2 pendiente.

La prueba 259 deja la iteracion `NO CONSEGUIDA`: el handoff de hover entra con
salto cero, pero ORB gobierna solo 227 muestras antes de fallback y tracking 3.
La oscilacion y el torque crecen antes del primer pending; una medida tratada
como pequena publica `0.058777 rad`. La probation confirma despues un residual
creciente, publica hasta `0.075 rad/paso` y precede dos rechazos excesivos. El
umbral SMALL dinamico dependiente de `dt` y la consistencia basada en residual
son las limitaciones vigentes. Por acuerdo se detienen etapas 3-8.
