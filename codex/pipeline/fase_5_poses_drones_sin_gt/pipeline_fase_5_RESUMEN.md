# Pipeline Fase 5 — Poses de los drones sin Ground Truth — RESUMEN

## Estado

```text
5A: CONSEGUIDA documentalmente el 2026-08-25
5B: CONSEGUIDA funcionalmente el 2026-08-26
5C: CONSEGUIDA
5D: CONSEGUIDA
5E: CONSEGUIDA tecnicamente
5F: PARCIAL; puerta humana no aceptada
5G-5H: PARCIAL; 256 acepta un salto angular ORB y colapsa
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
publica `0.119002 rad` antes de perder tracking `0.793 s` despues. No coincide
con una optimizacion W. Hace falta confirmar temporalmente las innovaciones
angulares moderadas antes de inyectarlas al control.
