# Instrucciones para Codex — Fase 5H tras prueba 264
## Corregir la fase temporal de la orientación `O_T_B` usada por el controlador

## 0. Estado actual

Estado de Fase 5H:

```text
PARCIAL
```

Última situación validada:

```text
Prueba 263:
  diagnóstico no concluyente por mezcla de relojes Gazebo/ROS.

Prueba 264:
  diagnóstico temporal CONSEGUIDO.
  Hover funcional NO CONSEGUIDO.
  Etapa 3 NO ejecutada.
```

Resultados principales de la prueba 264:

```text
runner success=true
92 s
recursos sanos

323 ciclos ORB sincronizados
6.44 s de tramo ORB analizable

omega_raw visual sigue a GT:
  correlación X ≈ 0.984
  correlación Y ≈ 0.982
  lag ≈ 80 ms

tau_ew:
  sentido contrario al damping ideal ≈ 34 % de ciclos
  globalmente sigue siendo disipativo

tau_er:
  trabajo positivo ≈ 80.9 % del tramo post-handoff
  energía inyectada ≈ +0.005012 J

tracking permanece válido hasta el fallback
```

Conclusión de trabajo:

> El problema principal ya no parece estar en `omega_bias`, ni en el decay de `omega_motion`, ni en el handoff, ni en W/KF. La evidencia dominante apunta a que la **orientación estimada usada como `R_act` llega temporalmente fuera de fase respecto al movimiento físico real**, haciendo que el término proporcional de actitud `tau_er = -Kr * er` inyecte energía en la oscilación.

La siguiente iteración debe corregir la **fase temporal de la pose angular `O_T_B`**, no recalibrar thresholds ni tocar ganancias.

---

# 1. Qué NO hay que tocar

No modificar en esta iteración, salvo que aparezca un bug independiente y demostrado:

```text
GT
navigation_state_mux
Kr
Kw
Kp
Kv
ORB-SLAM3 core
W / GlobalPoseStore
reference gate
extrínseca B_T_C
deadband de omega_bias
decay de omega_motion
thresholds raw calibrados
misión YAML
```

No usar GT como entrada del estimador.

GT sigue siendo únicamente:

```text
truth externa de diagnóstico
```

No ejecutar etapa 3 hasta aprobar el hover ORB.

---

# 2. Diagnóstico que debe considerarse demostrado por la prueba 264

## 2.1. El canal de velocidad angular visual tiene latencia, pero no parece ser el principal culpable

Se ha medido aproximadamente:

```text
omega_GT
   ↓
~80 ms
   ↓
omega_raw
```

con correlación alta.

Además:

```text
tau_ew = -Kw * ew
```

no es ideal en todo momento, pero globalmente sigue siendo disipativo.

Por tanto:

> No centrar ahora la siguiente modificación en `Kw`, en reducir `omega_motion`, ni en cambiar el filtro raw.

## 2.2. El término de error de orientación sí está inyectando energía

Se ha medido:

```text
tau_er = -Kr * er
```

con trabajo positivo durante aproximadamente:

```text
80.9 %
```

del tramo posterior al handoff.

Energía neta aproximada:

```text
+0.005012 J
```

Esto indica que `R_act`/`O_T_B` está suficientemente fuera de fase como para que el término proporcional de actitud empuje frecuentemente en el mismo sentido que el movimiento físico real.

Cadena sospechada:

```text
R física real cambia
        |
        v
imagen capturada
        |
        v
ORB procesa con retraso
        |
        v
R_ORB corresponde a un instante anterior
        |
        v
R_act publicada se trata como si fuera actual
        |
        v
er queda desfasado
        |
        v
tau_er
        |
        v
trabajo positivo
        |
        v
oscilación crece
```

---

# 3. Hipótesis concreta a revisar en el código

Antes de implementar la corrección, Codex debe responder exactamente:

```text
¿A qué instante físico corresponde pose_ dentro del estimador?
```

Revisar en detalle:

```text
UpdateMeasurement(...)
Predict(...)
Propagate(...)
timer de publicación 50 Hz
timestamp de imagen
timestamp de recepción
timestamp del procesamiento ORB
timestamp de publicación
timestamp recibido por control
```

Especialmente revisar si ocurre algo equivalente a:

```text
imagen física:
t = 10.000

ORB termina de procesar:
t = 10.080

UpdateMeasurement(measurement, 10.080)
```

cuando en realidad:

```text
measurement
```

describe aproximadamente:

```text
R(10.000)
```

Si se almacena como si representara:

```text
R(10.080)
```

entonces `Predict()` no puede compensar correctamente esos 80 ms.

Resultado:

```text
pose aparentemente "actual"
pero físicamente atrasada
```

Esto encaja con la evidencia de la 264.

---

# 4. Objetivo exacto de la siguiente modificación

La salida ORB usada por control debe representar:

```text
O_T_B(t_control)
```

o, como mínimo, la mejor predicción disponible al instante actual.

No basta con publicar:

```text
O_T_B(t_visual)
```

con timestamp reciente de procesamiento.

La lógica deseada es:

```text
pose visual ORB en t_visual
        |
        v
estado dinámico asociado al mismo t_visual
        |
        v
propagación temporal
        |
        v
pose predicha a t_publish / t_control
        |
        v
NavigationState
        |
        v
controlador
```

---

# 5. Regla principal de timestamps

Separar explícitamente tres tiempos:

```text
t_input
    instante físico de la imagen

t_receive
    instante ROS en que se recibe/procesa la muestra

t_publish
    instante en que se publica NavigationState
```

Y no confundirlos.

El estado estimado derivado de la imagen debe considerarse inicialmente válido en:

```text
t_input
```

o en el instante físico realmente representado por ORB.

No en:

```text
t_receive
```

por conveniencia.

---

# 6. Cómo calcular el tiempo visual en reloj ROS

La prueba 263 demostró que no se podían mezclar directamente:

```text
Gazebo simulation clock
ROS wall/steady/system clock
```

La 264 ya incorpora el puente necesario para sincronizar ambos.

Usar esa infraestructura de reloj para disponer de:

```text
t_visual_ros
```

equivalente al timestamp físico de la imagen pero expresado en el mismo dominio temporal utilizado por publicación/control.

No hardcodear:

```text
80 ms
```

como constante fija.

Los ~80 ms medidos son evidencia, no una constante de diseño.

La compensación debe usar:

```text
age =
t_target - t_visual_ros
```

por muestra.

---

# 7. Predicción angular hasta el instante actual

Una vez conocido:

```text
R_visual
omega_motion
t_visual
```

proyectar:

```text
R_pred(t_target)
=
Exp(omega * dt)
*
R_visual
```

con:

```text
dt = t_target - t_visual
```

respetando la convención actual de Sophus/frames.

No copiar esta fórmula literalmente sin revisar:

```text
left/right multiplication
frame de omega
```

La implementación debe ser coherente con cómo el proyecto representa:

```text
omega_O
omega_body
SO3
```

---

# 8. Muy importante — evitar doble predicción

Antes de añadir una nueva propagación, revisar lo que ya hacen:

```text
Predict()
Propagate()
timer 50 Hz
```

No quiero:

```text
predicción actual
+
nueva compensación
=
doble extrapolación
```

Codex debe documentar claramente:

```text
pose_ vive en timestamp X

angular_velocity_ vive en timestamp X

Predict(t) hace exactamente:
...

publicación usa:
...

compensación nueva añade:
...
```

Si el problema actual es que:

```text
pose_ está etiquetada temporalmente con t_receive
en vez de t_visual
```

la mejor solución puede ser simplemente:

```text
corregir el timestamp interno del estado
```

y dejar que `Predict()` haga su trabajo correctamente.

Priorizar corregir la semántica temporal antes que añadir capas nuevas.

---

# 9. Estado pose/omega debe seguir siendo coherente

Mantener la propiedad:

```text
R(t + dt)
≈
Exp(omega * dt)
*
R(t)
```

dentro de tolerancias.

No publicar:

```text
pose extrapolada a now
+
omega correspondiente al pasado
```

ni:

```text
pose antigua
+
omega actual
```

Pose y velocidad angular deben representar el mismo estado temporal.

---

# 10. Qué hacer con `omega_motion`

No rediseñarla en esta iteración.

Conservar:

```text
raw motion plausibility
omega_motion
omega_bias
deadband
hysteresis
bias suppression
rejected-motion decay
```

La 264 muestra que el raw visual sigue razonablemente al movimiento real.

El objetivo ahora es utilizar esa información temporal para llevar la **orientación** al instante correcto.

---

# 11. Edad máxima de extrapolación

Introducir o reutilizar un límite seguro:

```text
max_orientation_extrapolation_sec
```

Si:

```text
t_target - t_visual
```

supera dicho límite:

```text
no extrapolar indefinidamente
```

Dependiendo de la arquitectura actual:

```text
limitar extrapolación
marcar estado degradado
invalidar velocity_valid/local continuity si corresponde
```

No hacer predicciones largas sin soporte visual.

---

# 12. Calidad temporal

Añadir diagnóstico:

```text
visual_age_sec
prediction_horizon_sec
prediction_clamped
```

Clasificación conceptual:

```text
FRESH
DEGRADED
TOO_OLD
```

No es obligatorio añadir este enum si no aporta valor al código, pero sí deben quedar registrados esos conceptos.

---

# 13. Tests unitarios obligatorios

Mantener:

```text
37/37
```

y ampliar cobertura.

## Test A — medida con latencia conocida

Simular:

```text
pose física en t=0
omega constante
medida visual correspondiente a t=0
publicación en t=0.08
```

Esperado:

```text
pose publicada ≈ pose propagada 80 ms
```

## Test B — timestamp visual diferente del receive

```text
t_visual != t_receive
```

Esperado:

```text
estado interno conserva semántica temporal correcta
```

No tratar pose visual como si perteneciera a `t_receive`.

## Test C — ausencia de doble extrapolación

Crear escenario donde:

```text
Predict()
```

ya propaga desde la pose base.

Esperado:

```text
resultado = una sola propagación
```

No:

```text
2 * dt
```

## Test D — omega cero

```text
omega = 0
latencia > 0
```

Esperado:

```text
orientación no cambia
```

## Test E — giro constante

```text
omega constante
```

con varios `visual_age`.

Esperado:

```text
R_pred coincide con integración analítica
```

## Test F — límite de extrapolación

```text
visual_age > max_extrapolation
```

Esperado:

```text
clamp/degraded policy correcta
```

## Test G — coherencia pose/omega

Verificar:

```text
R(t+dt)
≈
Exp(omega_pub*dt) R(t)
```

## Test H — reference KF

Todos los tests existentes de cambios de referencia siguen pasando.

## Test I — bias y decay

Todos los tests introducidos en iteraciones 261/262 siguen pasando.

No degradar esas correcciones.

---

# 14. Telemetría obligatoria para la nueva prueba

Registrar:

```text
input_stamp
input_stamp_mapped_ros

receive_stamp
publish_stamp
control_receive_stamp
control_tick_stamp

visual_age_at_receive
visual_age_at_publish
visual_age_at_control

prediction_horizon_sec
prediction_clamped

R_visual
R_predicted_publish
R_control_used

omega_raw
omega_motion
omega_bias
omega_total

er
ew

tau_er
tau_ew
tau_total

omega_GT_body

P_er_GT = tau_er · omega_GT_body
P_ew_GT = tau_ew · omega_GT_body
P_total_GT = tau_total · omega_GT_body
```

---

# 15. Prueba siguiente

Repetir exactamente:

```text
f5h_etapa_2_hover_orb.yaml
```

No ejecutar etapa 3.

Nombre sugerido:

```text
prueba 265
```

No modificar ninguna otra cosa funcionalmente.

---

# 16. Comparación obligatoria 264 vs 265

Preparar resumen:

```text
                         264          265
------------------------------------------------
ORB govern time
visual lag raw
mean visual age
max visual age

max |omega_GT|
max |omega_motion|

max |er|
max |ew|

tau_er positive-work ratio
tau_er net energy

tau_ew positive-work ratio
tau_ew net energy

total angular work

max attitude oscillation
fallback
tracking loss
scenario success
```

---

# 17. Qué debe mejorar específicamente

La métrica principal no es sólo:

```text
más segundos en ORB
```

Quiero ver una mejora causal.

En 264:

```text
tau_er trabajo positivo ≈ 80.9 %
tau_er energía ≈ +0.005012 J
```

Tras corregir fase temporal de pose:

```text
tau_er no debe seguir inyectando energía
de forma sistemática
```

No fijar un porcentaje arbitrario como criterio absoluto.

Pero debe observarse:

```text
reducción clara de trabajo positivo
reducción de energía neta inyectada
reducción de crecimiento de oscilación
```

y finalmente:

```text
hover completo
```

---

# 18. Interpretación correcta de `tau_er`

No exigir:

```text
tau_er · omega_GT < 0
```

en todos los ciclos.

`tau_er` es un término proporcional de restauración, no damping puro.

Puede realizar trabajo positivo temporalmente.

El fallo de 264 es:

```text
trabajo positivo persistente
+
energía neta positiva
+
crecimiento de oscilación
```

Lo importante en 265 será comprobar si la corrección temporal elimina esa tendencia.

---

# 19. Si la prueba 265 funciona

Si:

```text
hover ORB completa
```

y la evidencia energética mejora claramente:

```text
Resultado de esta iteración:
CONSEGUIDO
```

Entonces se puede autorizar avanzar a:

```text
etapa 3
```

pero conservando toda la telemetría.

---

# 20. Si la prueba 265 sigue fallando

No volver a tocar thresholds por intuición.

Revisar primero:

```text
1. si R_pred realmente está temporalmente alineada con GT;
2. si el lag residual de R_act sigue siendo grande;
3. si tau_er sigue siendo la fuente principal de energía;
4. si el problema cambia de tau_er a tau_ew;
5. si hay error de transformación de orientación/omega entre O y body;
6. si R_des varía por causas translacionales.
```

La siguiente investigación dependerá de qué término siga inyectando energía.

---

# 21. Posible prueba complementaria si hace falta

Sólo si la 265 no resulta concluyente, añadir una prueba de diagnóstico con:

```text
hover
R_des prácticamente constante
```

y registrar por eje:

```text
roll
pitch
yaw
```

para determinar si la inyección energética se concentra en roll, pitch o yaw.

No hacer esta prueba antes de repetir el hover normal.

---

# 22. Restricciones arquitectónicas

Mantener:

```text
ORB state estimation local al dron

O = frame continuo de control

W = frame global corregible

servidor fuera del lazo de 50 Hz
```

No introducir:

```text
GT-assisted prediction
server-assisted attitude
```

No contaminar `O` con revisiones de `W`.

---

# 23. Documentación

Actualizar tras la iteración:

```text
historial_5H_RESUMEN.md
07_ULTIMA_SESION.md
contrato 5H si cambia semántica temporal
```

Documentar explícitamente:

```text
qué timestamp representa cada pose
qué timestamp representa omega
cómo se convierte input Gazebo -> ROS
desde qué instante hasta cuál predice Predict()
```

Esta semántica temporal debe quedar escrita como contrato, no sólo implícita en código.

---

# 24. Qué debe devolver Codex

Respuesta final esperada:

```text
Resultado:
CONSEGUIDO / PARCIAL / NO CONSEGUIDO
```

Incluyendo:

```text
- archivos modificados;
- bug temporal exacto encontrado o descartado;
- semántica anterior de timestamps;
- semántica nueva;
- cambios en UpdateMeasurement/Predict/Propagate;
- cómo se evita doble predicción;
- parámetros nuevos;
- GTests nuevos;
- total GTests;
- builds;
- resultado prueba 265;
- duración bajo ORB;
- comparación 264 vs 265;
- tau_er positive-work ratio;
- energía neta tau_er;
- tau_ew;
- energía angular total;
- fallback/tracking;
- decisión explícita:
    se autoriza etapa 3
    o
    NO se autoriza etapa 3.
```

---

# 25. Resumen ejecutivo

La prueba 264 demuestra que:

```text
omega_raw visual sigue razonablemente al movimiento físico
```

pero:

```text
R_act / O_T_B usada por control está fuera de fase
```

y esto provoca `tau_er` con trabajo positivo durante gran parte del tramo y energía neta positiva.

El siguiente cambio no es:

```text
hacer ORB más suave
```

ni:

```text
reducir omega
```

ni:

```text
cambiar ganancias
```

El siguiente cambio es:

> **hacer que la orientación publicada en `NavigationState` represente correctamente el instante físico actual del control, usando el timestamp real de la medida visual y una única predicción temporal coherente.**

La meta inmediata sigue siendo:

```text
hover ORB completamente estable
```

antes de ejecutar etapa 3.
