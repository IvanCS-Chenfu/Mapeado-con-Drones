# Pipeline Fase 5 — Poses de los drones sin Ground Truth — RESUMEN

## Estado

```text
5A: CONSEGUIDA documentalmente el 2026-08-25
5B: CONSEGUIDA funcionalmente el 2026-08-26
5C: CONSEGUIDA
5D: CONSEGUIDA
5E: CONSEGUIDA tecnicamente
5F: PARCIAL; puerta humana no aceptada
5G-5H: PARCIAL; 268 valida gate raw y descarta anclaje moderate completo
5I: absorbida en 5H
Fase 5 funcional: en curso
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
