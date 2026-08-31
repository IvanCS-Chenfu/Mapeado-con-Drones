# Pipeline Fase 5 — Poses de los drones sin Ground Truth — RESUMEN

## Estado

```text
5A: CONSEGUIDA documentalmente el 2026-08-25
5B: CONSEGUIDA funcionalmente el 2026-08-26
5C: CONSEGUIDA
5D: CONSEGUIDA
5E: CONSEGUIDA tecnicamente
5F: PARCIAL; puerta humana no aceptada
5G-5H: CONSEGUIDA; evidencia visual causal y ruta favorable ORB 3/3
5I: absorbida en 5H
Fase 5 funcional: CONSEGUIDA dentro del alcance previo a Fase 6
```

## Objetivo

Separar de forma estricta:

```text
control continuo: O_T_B
global corregible: W_T_B
```

`RawMapDatabase` conserva ORB crudo, `GlobalPoseStore` conserva la autoridad
world y ORB-SLAM3 no recibe correcciones globales reinyectadas en su mapa.

## Contrato principal

La pose por frame usa el reference KF real y su relación:

```text
Tcr = C_T_Kref
O_T_B = O_T_Kref * inverse(Tcr) * C_T_B
W_T_B = W_T_Kref * inverse(Tcr) * C_T_B
```

No se usa el último KF creado, nearest-KF ni round-trip al Servidor por frame.

El Servidor resuelve `(drone_id, map_epoch, keyframe_id) -> W_T_KF + revision`.
La consulta inicial es asíncrona y las revisiones posteriores llegan por push.

## Goals

```text
relativo:
    aceptar con local válida y ejecutar en O

absoluto + W_T_O válida:
    convertir world -> O al aceptar y congelar

absoluto sin W_T_O en perfil Fase 5:
    ejecutar mediante GT_FALLBACK temporal
```

Las correcciones globales posteriores no modifican el goal activo.

## Pérdida

```text
OK             -> ORB/O
RECENTLY_LOST  -> GT_FALLBACK temporal
```

El fallback permite terminar la misión, mantiene `O` continuo y se identifica
mediante `pose_source`. GT no entra en mapa, anchors, optimización ni pose
global final. Se mantiene durante Fase 5 y Fase 6 debe eliminarlo al aportar
recovery real.

La fuente queda congelada por goal. GT no cambia a ORB durante una trayectoria
activa aunque aparezca un anchor; la frontera posterior permite ORB si tracking,
anchor y cualificacion son validos. Una perdida ORB entra inmediatamente en GT
y no vuelve a ORB hasta terminar ese goal. La deriva GT-estimada no decide la
fuente.

## Decisiones de 5A

- smoothing no obligatorio;
- estimador en `orbslam3` del Dron;
- frontera obligatoria Servidor↔`orbslam3`;
- velocidad derivada de `O_T_B`;
- 5I absorbida en 5H;
- antes de 5C/5D se comprueba el cierre final de 3Q;
- antes de 5G se reconcilia formalmente el ADR de GT.

## Secuencia

```text
5A -> 5B -> 5C -> 5D -> 5E -> 5F -> 5G -> 5H
```

5B deja operativo `NavigationState`, continuidad `O_T_B` intra-epoch y el gate
de goals. La prueba 225 valida anchors hard, cambios de reference KF, rechazo
absoluto, snapshots relativos y pérdida real de ambos drones tras el giro de
180 grados.

El bloque 5C+5D+5E reutiliza `GlobalPoseStore`, resuelve la reference KF por
servicio asincrono mas push dirigido y compone W sin mover O. La prueba 230
confirma ambos anchors, loops, pushes y revisiones naturales. 5F genera
CSV/JSON/PNG validos a ~19.15 Hz. Los agregados de todas las revisiones dan
posicion MAE 2.953/0.191 m y error angular MAE 1.907/2.104 rad, pero no aislan
la convergencia posterior a cada optimizacion. Esas diferencias frente a GT se
mantienen como metricas externas de deriva y no gobiernan el control. El bloque
5G+5H implementa el mux ORB/fallback, velocidad comun, handshake
de fuente por goal y control sin topics GT directos. RViz2 muestra un sistema
XYZ desde `o_t_body`, exactamente la pose consumida por el controlador, con
etiqueta `[ORB]/[GT]` y sin depender de `global_valid`. La prueba 243 completa
17/17 pasos, 22/22 goals y 44/44 handshakes, sin entradas ORB dentro de goals
bloqueados en GT. Las pruebas 249-251 descartan el handoff y localizan la
extrinseca inversa. La 252 aplica el `B_T_C` correcto y completa 17/17 pasos y
22/22 goals, eliminando el fallo X world -> Z control. La revision visual
conserva dos defectos: ORB se consume a 20 Hz con velocidad por diferencia
finita sin filtro mientras el PD corre a 50 Hz, y una perdida dentro del goal
hace `ResetToSource` hacia GT sin regenerar la trayectoria congelada en O. La
discrepancia ocasional entre ejes y KFs es esperable al comparar `o_t_body` de
control con KFs en W optimizado.

El predictor SE(3) posterior no queda validado: la prueba 253 se interrumpe por
divergencia durante GT continuo, sin ninguna conmutacion de fuente. Filtrar y
limitar la actitud exacta introduce retardo en el lazo de torque; no repetir esa
configuracion ni confundir el fallo con la alineacion ORB/GT.

La arquitectura corregida de 254 deja GT exacto y mueve el predictor a
`orbslam3_ros2`; builds y tests focales pasan. La simulacion confirma salto cero
al entrar en ORB, pero falla en los giros porque la pose acepta orientaciones
medidas discontinuas de hasta `0.279 rad/frame` mientras la velocidad angular
publicada esta limitada. Deben corregirse juntas pose y velocidad angular ORB.

La prueba 255 implementa esa coherencia mediante probation de reference KF y
rechazo completo de outliers. Build y 13/13 GTests finales pasan. La ejecucion
interrumpida completa 7/17 pasos: evita publicar innovaciones de
`0.081-0.214 rad` y conmuta a GT con salto cero, pero acumula 10 timeouts y ORB
gobierna pocos segundos. Revision visual pendiente antes de ajustar el gate.

La prueba 256 permite churn geometrico multi-KF y corrige gradualmente todo el
estado SE(3); build y 15/15 GTests pasan. La simulacion falla al entrar en ORB:
el handoff tiene salto cero, pero drone2 acepta `0.125261 rad` de innovacion y
registra `rotation_step_rad=0.119002` antes de perder tracking `0.793 s`
despues. No coincide con una optimizacion W. Ese campo no representa el salto
publicado, por lo que la causalidad queda abierta. Hace falta confirmar
temporalmente las innovaciones moderadas e instrumentar la cadena completa
hasta pose/omega, torque y tracking.

La iteracion siguiente implementa probation temporal y telemetria completa;
builds y 21/21 GTests pasan. La etapa 1 de prueba 258 se consigue bajo GT
observado. La etapa 2 de prueba 259 falla en hover: ORB entra con salto cero,
pero oscila antes del primer pending; una salida SMALL publica `0.058777 rad`,
la cadena moderada posterior llega a `0.075 rad/paso`, aparecen dos rechazos
excesivos, fallback y tracking 3. Se detienen las etapas 3-8. El umbral SMALL
dependiente de `dt` y la confirmacion basada en residual quedan como siguiente
problema de diseno.

La iteracion raw/bias posterior fija SMALL y separa movimiento entre medidas de
correccion absoluta. Pasa 27/27 GTests y la prueba 260 calibra correctamente
hover y yaw bajo GT. La prueba 261 entra en ORB con salto y primer error cero,
pero solo lo mantiene unos 3.82 s: el bias responde desde SMALL, el movimiento
fisico realimentado se acepta como raw plausible y termina dominando la salida.
Tres rechazos excesivos invalidan el estimador antes de que tracking se pierda.
No se ejecuta etapa 3; el siguiente acuerdo debe romper esta realimentacion sin
tocar GT, mux, ganancias ni W.

La correccion posterior añade deadband/histeresis y confirmacion al bias,
supresion durante movimiento raw y decay continuo de raw rechazado. Pasa 37/37
GTests. En 262 `omega_bias` permanece a cero y el decay funciona; ORB mejora a
unos 5.92 s, pero `omega_motion` oscila hasta unos `0.617 rad/s` y vuelve a
forzar fallback antes de tracking 2->3. La etapa 3 sigue detenida. El siguiente
diagnostico debe medir latencia/fase del canal angular visual respecto al lazo
de control.

La prueba diagnóstica 263 conserva exactamente el comportamiento de 262 y
captura los vectores previstos, pero no puede sincronizar GT: su header usa
tiempo Gazebo y receive/control usan tiempo ROS. Queda `DATOS INSUFICIENTES`,
sin confirmar ni descartar fase/latencia. Se añadió `gt_receive_stamp` y el
analizador dual-clock quedó compilado.

La prueba 264 repite el hover con ese puente: raw sigue al GT con `~0.08 s`,
pero la pose/control angular queda fuera de fase. `tau_ew` es
anti-amortiguante de forma intermitente, mientras `tau_er` realiza trabajo
positivo en `80.9 %` del tramo post-handoff e inyecta `+0.005173 J` al
reprocesar con el analizador de 265. El
diagnostico temporal queda conseguido, pero el hover sigue `NO CONSEGUIDO` y
5H `PARCIAL`. No ejecutar etapa 3; el siguiente cambio debe tratar la fase de
orientacion, no GT, mux ni una simple recalibracion raw.

La prueba 265 corrige solo la semantica temporal: el wrapper usa la edad local
desde el ingreso del callback y extrapola una unica vez con horizonte medio
`43.2 ms`, en vez de saturar siempre a `0.10 s`. Builds, 40/40 GTests y 5/5
tests del analizador pasan. Funcionalmente empeora: ORB dura `5.56 s`,
`tau_er` inyecta `+0.160266 J` y el torque total `+0.145081 J`. La distancia
`visual_q -> base_q` llega a `0.339 rad`, muy por encima del paso añadido por
la prediccion. La causa dominante queda en `pose_` base integrada; el horizonte
fijo anterior compensaba parcialmente ese retraso. 265 queda
`NO CONSEGUIDA`, 5H `PARCIAL` y etapa 3 detenida. Debe acordarse la fusion o
anclaje visual de la orientacion base antes de otro cambio funcional.

La prueba 266 reancla la pose base a cada medida SMALL/plausible y limita a
`0.015 rad` las correcciones MODERATE_CONFIRMED, sin convertirlas en omega.
Pasa tres builds, 44/44 GTests y 7/7 tests del analizador. En ventana comun de
`5.56 s`, `tau_er` baja de `+0.153559` a `+0.002067 J` y el torque total pasa
de `+0.138374` a `-0.001945 J`; ORB se extiende hasta `8.06 s`. No completa el
hover: la fase moderate deja residual, alterna con PREDICT_ONLY, raw se rechaza
a `+7.50 s` y fallback llega a `+8.08 s`, antes de tracking 3. Resultado
`NO CONSEGUIDA` con mejora sustancial; 5H sigue `PARCIAL`, etapa 3 detenida y
politica moderate pendiente de nuevo acuerdo.

La prueba 267 sustituye el limite moderate por anclaje completo. Sus cuatro
anclas dejan error after cero, pero ORB cae a `5.56 s`; en ventana comun
`tau_er=+0.030448 J` y total `+0.015622 J`, peores que 266. Tras el primer
anclaje se alcanza casi la misma energia de fallo que en 266, pero en `1.22 s`
en vez de `2.06 s`. Una confirmacion anclo con raw rechazado; el gate raw final
se corrige mecanicamente, pasa build y 46/46 GTests, pero aun no tiene
simulacion. No se ejecutan 268 ni etapa 3; 5H sigue `PARCIAL`.

La prueba 268 valida el gate raw final sin otros cambios: el unico anclaje
moderate es plausible y no hay confirmed anchors con raw rechazado. Aun asi,
tras corregir `0.057317 rad`, el torque total acumula `+0.046416 J` en
`0.88 s`; ORB dura `5.72 s` y cae a fallback. El bug raw de 267 no era la
causa principal. No se ejecutan 269 ni etapa 3; el residual gradual
`Delta_target` queda pendiente de autorizacion.

El cierre 350R-355 instrumenta evidencia visual por el mismo frame sin cambiar
comportamiento. 351 reproduce bajo GT+shadow la caída de inliers, cobertura y
depth antes de `RECENTLY_LOST` en la fachada este. La ruta manual favorable
corta/lenta y junto a pared conserva tracking; 353-355 la completan 3/3 bajo
autoridad ORB sin fallback posterior. Fase 5 queda funcionalmente conseguida:
los fallos de la vuelta larga se atribuyen a baja observabilidad y su prevención
activa pasa a Fase 6. `GT_FALLBACK` se conserva hasta que Fase 6 aporte tareas y
recovery; no se afirma que la vuelta larga sea hoy ORB-only.

La bateria 269-272 cambia el diagnostico siguiente: GT normal 50 Hz es estable,
pero GT perfecto a 20 Hz atravesando el `OrbPosePredictor` ya produce giro y
energia positiva; añadir 80 ms o el timing realista de 268 provoca fallo antes
del segundo hover. La geometria ORB deja de ser necesaria para reproducir el
problema. Antes de `Delta_target` debe corregirse la semantica temporal 20->50
Hz y la coherencia de pose/omega. El laboratorio GT queda apagado por defecto.

Las pruebas 273-275 afinan esa causa: E estabiliza la arquitectura actual al
sustituir solo `omega_motion` por omega GT; F con hold y G con extrapolacion
directa tambien son estables y disipativas. Se selecciona la opcion A del
diagnostico: derivacion/filtrado de `omega_motion`. La siguiente correccion
debe actuar sobre esa estimacion ORB, no sobre gains, hold o SO(3).

Las pruebas 276-277 sustituyen esa derivacion por un estimador causal de tres
poses. Ambas completan el hover con pose GT a 20 Hz, sin fallback y con energia
total negativa; el RMSE baja de `0.43338 rad/s` en 270/B a
`0.00374/0.00304 rad/s`. La correccion es reproducible en laboratorio, pero la
fase sigue `PARCIAL` hasta ensayar delay/jitter y ORB real.

La prueba 278 añade 80 ms sin alterar el estimador y no completa el hover:
clamp `72.2 %`, RMSE `1.447 rad/s` y energia total `+0.02884 J`. La secuencia
se detiene y 279-281 no se ejecutan. Falta acordar compensacion `t_k -> now`.

282 elimina el clamp con horizonte diagnostico `0.18 s`, pero empeora el fallo.
284 propaga pose/omega con `alpha_hat`: mejora RMSE/energia, aunque cae antes y
rechaza raw desde `0.84 s`. Ninguna variante completa el hover; 279 permanece
detenida y la extrapolacion alpha queda apagada por defecto.

La bateria 285-287 cruza R y omega predichas/GT actuales. Las ramas con omega
GT duran unos `13.1 s` y son disipativas; con omega predicha solo `2.98 s` y
energia positiva. La evidencia señala omega, pero 287 tambien falla porque p/v
lineales siguen retrasadas: el sanity no es GT completo y la causalidad queda
pendiente. 279-281 permanecen detenidas.

La bateria 288-291 fija p/v GT actuales y cierra esa ambiguedad. 288 y 290 usan
omega predicha y fallan tras `2.54/5.18 s`, con energia positiva; 289 mantiene
R predicha pero usa omega GT y completa `54.46 s`, y 291 con estado GT completo
completa `55.20 s`. El sanity pasa y ambas ramas con omega GT son disipativas.
El fallo inmediato bajo delay queda aislado en `omega_pred(now)`, no en R ni
p/v. 5H sigue `PARCIAL` hasta corregirla sin GT y validar ORB real; 279-281
siguen detenidas.

La prueba 292 integra dinamica rigida causal con torque body y la J nominal
compartida con control. Builds y 75/75 GTests pasan, pero el modelo predice
cientos de rad/s frente a valores GT del orden de uno y colapsa en decimas.
292 queda `NO CONSEGUIDA`; por STOP no se ejecutan 293-295. Debe revisarse la
inercia efectiva/torque aplicado antes de continuar hacia ORB.

La revision calcula la J compuesta del modelo y actualiza el valor compartido a
`diag(0.00803107,0.00803107,0.015805)`. Con ella, 296-298 y 293-295 completan
seis hovers consecutivos, sin fallback ni tracking no-OK, RMSE omega entre
`0.00255-0.00557 rad/s`, mismatch maximo `0.777 %` y energia total negativa.
El predictor angular queda validado con delay fijo hasta 294. Una revision
posterior confirma que 295 no tenia los 80 ms añadidos (edad media 31 ms), por
lo que el estado completo queda validado solo sin ese delay. Faltan
timing/jitter medido, estado completo bajo ese timing y ORB real para cerrar 5H.

299 ensaya el estado completo con la traza temporal determinista de 268 y
falla tras `16.94 s`, pese a mantener fuente y cobertura de torque. Los
intervalos largos generan 48 `DEGRADED_DT`, la edad alcanza `0.20 s` y la
energia angular neta pasa a `+0.0141 J`; los rechazos raw aparecen al final,
despues de la divergencia. Por el STOP no se ejecutan 300-302 ni se integra el
predictor en ORB productivo. 5H sigue `PARCIAL`.

El cruce 303-306 identifica `PV PRINCIPAL`: 303 completa con p/v GT y angular
dinamica, 305 falla conservando p/v predichas aunque R/omega sean GT actuales,
y 306 valida el montaje con GT completo. 304 no valida la propagacion desde
omega GT instantanea en `t_k`, aun con bracket y torque cubiertos; queda como
diagnostico secundario. No se reabren 300-302.

La integracion post-317 lleva los estimadores causales y predictores dinamicos
a `StereoSlamNode` con selector `legacy|dynamic`. Builds y suites pasan. La
paridad 318 completa el escenario, pero incumple el criterio por un hueco de
torque al inicio de un movimiento: la muestra disponible estaba sellada despues
de la base. Se detiene el bloque; 319/320 ORB real no se ejecutan.

La iteracion causal demuestra el cero inicial, añade seed, ZOH, cobertura
explicita y persistencia ante resets. 318R completa, pero la poda de 0.5 s
elimina el seed antes del primer comando y produce `MISSING_PREFIX=70 ms`.
La correccion conserva una predecesora ZOH: 318R2 y 319R completan sin missing
ni fallback y validan poda, cobertura y paridad. La primera 320 queda invalida
porque el YAML imponia `legacy`; corregida la precedencia del launch, 320R usa
`dynamic` realmente, pero no sigue la aproximacion bajo ORB y llega al segundo
goal cerca del suelo con velocidad alta antes de una perdida visual breve.
Integracion ORB productiva no validada; 321 queda detenida por STOP.

El diagnostico shadow fuerza GT solo durante aproximacion/asentamiento y deja
el ORB dinamico productivo activo en paralelo. 320R2 es invalida por ruta YAML;
320R2R valida un handoff en frontera con salto pose/rotacion cero. Aun con
tracking `2`, sin fallback ni missing, el hover diverge hasta `~1.63 m` y
`~0.52 rad`. La bateria 321 confirma autoridad antes del goal: 321B se mantiene
estable al sustituir solo `v_GT`, mientras 321AR y 321D divergen con `v_ORB`.
La velocidad lineal ORB queda aislada como causa principal; ORB completo sigue
sin validar.

322 mantiene GT como autoridad y ejecuta ORB `dynamic` integro en shadow. En
907 medidas settled, `v_mid` da RMSE `0.01984 m/s`, THREE_SAMPLE
`0.03457 m/s` frente a TWO_SAMPLE `0.01988 m/s`, y la propagacion hasta now
`0.43308 m/s`. Diagnostico `MULTICAUSAL`: `A_HAT_AMPLIFICATION` mas
`DYNAMIC_PROPAGATION`; no se modifica aun la salida productiva.

324/325 corrigen solo la segunda causa: la gravedad se transforma W->O y se
congela por epoch desde autoridad global. `v_dynamic_now` baja de `0.43308` a
`0.03583/0.03707 m/s`, reproducible y sin cambiar THREE_SAMPLE. La propagacion
dinamica queda corregida; la amplificacion `a_hat` permanece separada.

326/327 comparan sobre muestras comunes TWO_SAMPLE, THREE_SAMPLE y el nuevo
MIDPOINT_DYNAMIC. Este ultimo conserva cobertura 100 %, empata con TWO_SAMPLE
en hover, mejora ligeramente en movimiento y evita la amplificacion de
THREE_SAMPLE. 328/329 confirman reproduciblemente su salida productiva en
shadow con RMSE `0.02113/0.02460 m/s`.

330/331 entregan el estado ORB completo al controlador tras el handoff y
completan el nuevo goal hover durante `34.78/35.30 s`, con tracking OK, sin
fallback ni clamp y energia angular total negativa. Quedan validados
`A_HAT_AMPLIFICATION CORREGIDA`, el estimador lineal productivo y el hover ORB
real. 5H permanece parcial hasta validar X/Y/Z/yaw y la trayectoria
representativa.

332/333 validan X 2 m de forma reproducible con ORB, tracking continuo y
fallback cero. 334 mantiene errores de control acotados, pero su trayectoria
`[0,-10,1] -> [0,-8,1]` atraviesa el fiducial 2 en `[0,-8.5,1]`; la colision
causa la perdida de tracking y el fallback. 334 es invalida y no evalua Y;
335-343 no se ejecutan.

334R corrige la geometria y mantiene ORB/tracking/fallback cero, pero revela
velocidad residual en el hover final: ev final `0.187 m/s` y RMSE final de
3 s `0.174 m/s`, frente a GT `0.049 m/s`. Y queda no validado y se aplica STOP.

334R2 visual reproduce y agrava el fallo sin perder tracking: ep/ev final
`0.213 m / 0.551 m/s`, max ev `2.188 m/s` y RMSE ev final `1.008 m/s`.

334R3R/335R muestran que Y se estabiliza al acercar el dron a la pared y
evitar el fiducial: tracking continuo, fallback cero y posicion acotada, con
residual final de velocidad cercana a `0.11 m/s`. 336/337R validan Z. 338 yaw
falla con max er `0.995 rad`, RMSE omega `0.409 rad/s`, RMSE lineal
`0.530 m/s` y perdida de tracking que activa fallback. STOP: 339-343 no
ejecutadas.

Auditoria 344-346: 344 descarta mezcla de frame Kref y confirma que rechazos
encadenados conservaban un baseline raw hasta `20.609 s`. El rebase aislado de
ese historial pasa build, GTest `117/117` y 345 shadow, reduciendo SUSPICIOUS
`168->2` y raw_dt maximo a `0.201 s`. En 346 la higiene raw se mantiene, pero
el dron activa fallback con tracking todavia OK y errores ORB maximos de
`0.795 m/1.730 m/s`; la perdida visual ocurre despues. Resultado: correccion
stale validada, dos fachadas ORB no validadas y 347 detenida por STOP.

348 añade solo observabilidad y confirma que el fallback de 346 se activa por
un pulso de validez local/continuidad con tracking todavia OK. La divergencia
lineal y angular comienza mucho antes y casi simultaneamente; resultado
`MULTICAUSAL`, con ligera precedencia angular. El raw permanece saneado. No se
ejecutan 348R ni 349A/349B; hace falta acordar el siguiente aislamiento.

El aislamiento acordado se completa con 349AR3/349B y cobertura GT causal
`99.9817/100 %`. Ambas ramas fallan por separado: p/v ORB con angular GT y
R/omega ORB con p/v GT. Resultado `MULTIPLE_INDEPENDENT_ERRORS`; no se ejecutan
350A/350B y se aplica STOP antes de modificar estimadores.
