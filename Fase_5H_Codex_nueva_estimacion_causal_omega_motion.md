# Fase 5H — Sustituir la estimación actual de `omega_motion` por una estimación causal y con menor desfase

## 0. Punto de partida

Las pruebas diagnósticas 273-275 han aislado el problema principal:

```text
E / 273:
pose GT a 20 Hz + omega GT exacta
-> escenario completo
-> energía total ligeramente disipativa

F / 274:
pose GT a 20 Hz con hold + omega GT exacta
-> escenario completo
-> estable

G / 275:
pose GT a 20 Hz + omega GT exacta + extrapolación SO(3)
-> escenario completo
-> estable
```

Conclusión aceptada:

> El problema principal NO está en el controlador a 50 Hz, ni en recibir pose a 20 Hz, ni en el hold, ni en la extrapolación SO(3). El fallo está principalmente en cómo se deriva y filtra `omega_motion` a partir de las poses.

No tocar ahora:

```text
SMALL / MODERATE
anclajes
Delta_target
reference KF
W
GT fallback
ganancias del controlador
extrapolación SO(3)
```

El siguiente trabajo debe centrarse exclusivamente en:

```text
poses ORB
    ->
estimación angular causal
    ->
omega_motion
```

---

# 1. Problema actual

La velocidad angular derivada entre dos poses:

```text
R(k-1)
R(k)
```

mediante algo equivalente a:

```text
omega_raw =
Log(R(k) * inverse(R(k-1))) / dt
```

representa aproximadamente la velocidad media ocurrida durante:

```text
[t(k-1), t(k)]
```

Por tanto, temporalmente está más cerca de:

```text
omega(t_mid)
```

con:

```text
t_mid = (t(k-1) + t(k)) / 2
```

que de:

```text
omega(t_k)
```

Además, el filtrado actual añade memoria.

Esto puede producir:

```text
el dron físico ya cambia de sentido
pero omega_motion conserva durante un tiempo
el signo del intervalo anterior
```

Las pruebas previas ya observaron precisamente este tipo de incoherencia.

---

# 2. Objetivo

Construir una `omega_motion` que estime mejor:

```text
omega(t_k)
```

y, cuando sea necesario:

```text
omega(t_now)
```

en vez de publicar directamente la velocidad media del intervalo anterior o una versión filtrada con demasiado lag.

La prioridad es:

```text
bajo desfase
coherencia temporal
robustez suficiente
```

No buscar primero una señal extremadamente suave.

---

# 3. Primera solución a implementar

Empezar con una estimación causal sencilla usando los últimos tres estados visuales válidos:

```text
R(k-2), t(k-2)
R(k-1), t(k-1)
R(k),   t(k)
```

Calcular:

```text
DeltaR_1 =
R(k-1) * inverse(R(k-2))

DeltaR_2 =
R(k) * inverse(R(k-1))
```

y sus velocidades medias:

```text
omega_mid_1 =
Log(DeltaR_1) / dt_1

omega_mid_2 =
Log(DeltaR_2) / dt_2
```

donde conceptualmente:

```text
omega_mid_1 representa t_mid_1
omega_mid_2 representa t_mid_2
```

Después estimar aceleración angular:

```text
alpha_hat =
(omega_mid_2 - omega_mid_1)
/
(t_mid_2 - t_mid_1)
```

y proyectar la última velocidad media hasta `t_k`:

```text
omega_hat_k =
omega_mid_2
+
alpha_hat * (t_k - t_mid_2)
```

En intervalos aproximadamente uniformes:

```text
t_k - t_mid_2 ~= dt_2 / 2
```

por lo que conceptualmente:

```text
omega_hat_k
≈
omega_mid_2
+
alpha_hat * dt_2 / 2
```

Codex debe revisar cuidadosamente:

```text
convención SO(3)
left/right multiplication
frame en el que está expresada omega
```

antes de aplicar literalmente las fórmulas.

---

# 4. Propagación desde `t_k` hasta `now`

Si la publicación ocurre después de la última medida:

```text
t_now > t_k
```

se puede proyectar la velocidad:

```text
omega_hat_now =
omega_hat_k
+
alpha_hat * (t_now - t_k)
```

pero únicamente durante un horizonte corto y limitado.

Añadir/reutilizar:

```text
max_omega_prediction_horizon_sec
```

No extrapolar indefinidamente.

---

# 5. No introducir todavía un filtro lento

No aplicar inicialmente otro filtro pasa-bajos fuerte sobre:

```text
omega_hat
```

porque podría volver a introducir el mismo problema de fase.

Primero probar la estimación causal.

Sí mantener protecciones de seguridad:

```text
max angular speed
max angular acceleration
dt validity
finite checks
raw plausibility
```

pero no usar esas protecciones para introducir una dinámica lenta artificial.

---

# 6. Fallback de estimación cuando no hay historial suficiente

Estados posibles:

## Primera medida

```text
omega_motion = 0
velocity_valid = false
```

o comportamiento equivalente actual.

## Dos medidas válidas

Sólo existe:

```text
omega_mid_2
```

Usarla como estimación provisional:

```text
omega_motion = omega_mid_2
```

sin estimar aceleración todavía.

## Tres o más medidas

Activar:

```text
omega_hat_k
```

con compensación temporal.

---

# 7. Qué hacer con `dt` irregular

No asumir frecuencia fija de 20 Hz.

Usar timestamps reales.

Si:

```text
dt <= mínimo válido
```

o:

```text
dt > máximo fiable
```

no calcular aceleración con esa secuencia.

En ese caso:

```text
degradar a estimación de dos muestras
```

o aplicar la política actual segura.

Registrar:

```text
omega_estimator_mode =
INIT
TWO_SAMPLE
THREE_SAMPLE_PREDICTED
DEGRADED_DT
REJECTED
```

o nombres equivalentes.

---

# 8. Qué hacer con muestras rechazadas

Si una medida visual no supera los gates actuales:

```text
NO incorporarla al historial utilizado
para estimar omega
```

No contaminar:

```text
omega_mid
alpha_hat
```

con medidas rechazadas.

Mantener el decay ya validado de `omega_motion` si se pierde soporte visual.

---

# 9. No mezclar correcciones absolutas con velocidad física

Mantener la separación:

```text
corrección de orientación estimada
!=
omega física
```

Un:

```text
SMALL_ANCHOR
```

o cualquier corrección de `R_base` no debe crear artificialmente:

```text
omega_motion
```

La nueva `omega_motion` debe salir únicamente de:

```text
evolución temporal entre poses visuales aceptadas
```

---

# 10. Telemetría obligatoria

Registrar para cada actualización:

```text
t_k2
t_k1
t_k

dt_1
dt_2

omega_mid_1
omega_mid_2

t_mid_1
t_mid_2

alpha_hat

omega_hat_k
omega_hat_now

prediction_horizon

omega_motion_final

omega_estimator_mode

raw_motion_class
tracking_state
reference_kf
map_epoch
```

Durante diagnóstico con GT, registrar también:

```text
omega_GT
```

sólo como referencia externa.

---

# 11. Métricas principales

Comparar:

```text
omega_motion nueva
vs
omega_GT
```

en el mismo escenario de hover.

Medir:

```text
correlación por eje
lag por eje

mismatch direccional GT/control

RMSE omega
MAE omega

max |omega_error|

er
ew

tau_er
tau_ew
tau_total

energía tau_er
energía tau_ew
energía total
```

Especialmente:

```text
sign(omega_GT)
vs
sign(omega_motion)
```

en los cambios de sentido.

---

# 12. GTests obligatorios

Mantener todos los tests actuales.

Añadir como mínimo:

## Test A — velocidad constante

Tres poses generadas con:

```text
omega constante
```

Esperado:

```text
alpha_hat ~= 0
omega_hat_k ~= omega real
```

---

## Test B — aceleración angular constante

Generar poses con:

```text
omega(t) lineal
```

Esperado:

```text
omega_hat_k
```

más cercana a la velocidad instantánea en `t_k` que:

```text
omega_mid_2
```

---

## Test C — cambio de signo

Secuencia donde:

```text
omega positiva
-> cruza cero
-> omega negativa
```

Esperado:

```text
la nueva omega cambia de signo
antes que el estimador antiguo filtrado
```

y sin overshoot absurdo.

---

## Test D — dt irregular

Esperado:

```text
se usan timestamps reales
sin asumir 50 ms fijos
```

---

## Test E — gap grande

Esperado:

```text
degradación segura
no alpha artificial enorme
```

---

## Test F — muestra rechazada

Esperado:

```text
no entra en historial
```

---

## Test G — reinicio epoch/reference invalidation

Si el contexto visual deja de ser coherente:

```text
reset del historial angular
```

según corresponda.

---

## Test H — no omega artificial por anchor

Aplicar una corrección absoluta de `R_base`.

Esperado:

```text
omega_motion no cambia por esa corrección
```

---

# 13. Primera simulación después de implementar

Antes de volver a ORB real, repetir una prueba de laboratorio equivalente a:

```text
GT pose 20 Hz
pero omega_motion calculada por el NUEVO estimador
```

No usar `omega_GT` como salida.

Nombre sugerido:

```text
prueba 276
```

Objetivo:

> comprobar si el nuevo estimador puede reconstruir una omega suficientemente buena a partir de poses perfectas a 20 Hz.

---

# 14. Criterio de éxito de la prueba 276

Debe completar el hover y acercarse claramente al comportamiento E/F/G.

Esperado:

```text
scenario success = sí
sin oscilación creciente
energía total ~0 o negativa
mismatch direccional bajo
```

No exigir igualdad perfecta a GT, pero sí una mejora fuerte respecto a la prueba 270/B.

Comparar:

```text
270/B antiguo predictor
273/E omega GT exacta
276 nuevo estimador
```

---

# 15. Si 276 funciona

Repetir una segunda vez para comprobar reproducibilidad.

Después probar:

```text
GT pose 20 Hz
+
80 ms
+
nuevo estimador
```

para comprobar que la nueva omega sigue siendo usable bajo delay.

Después:

```text
timing/jitter realista ORB
+
GT pose perfecta
+
nuevo estimador
```

Sólo cuando esas pruebas sean razonables volver a:

```text
ORB real
```

---

# 16. Si 276 falla

No volver a tocar SMALL/MODERATE.

Analizar específicamente:

```text
omega_mid
alpha_hat
omega_hat
lag
mismatch
```

Si la estimación de tres muestras sigue siendo demasiado ruidosa o atrasada, siguiente opción:

> regresión causal sobre una ventana corta de 3-5 poses recientes en SO(3), optimizada para estimar velocidad en el extremo actual de la ventana, no en su centro.

No implementar esa regresión antes de probar la solución de tres muestras.

---

# 17. Solución de segunda línea si visión pura no basta

Si incluso con una estimación causal optimizada la velocidad angular visual no es suficientemente buena para control:

```text
modelo dinámico del dron
+
torque comandado
+
ORB como corrección
```

sería la siguiente arquitectura.

Conceptualmente:

```text
tau conocido
    ->
modelo J
    ->
predecir omega a 50 Hz
    ->
ORB corrige periódicamente pose/omega
```

Pero NO implementar todavía.

---

# 18. Restricciones

No cambiar:

```text
control gains
GT normal
mux
W
KF policy
trajectory
ORB-SLAM3 core
```

No usar:

```text
omega_GT
```

en operación real.

GT sólo se usa en las pruebas de laboratorio para validar la calidad de la nueva estimación.

---

# 19. Entrega final de Codex

Tras implementar y probar, devolver:

```text
Resultado:
CONSEGUIDO / PARCIAL / NO CONSEGUIDO
```

Incluyendo:

```text
- archivos modificados;
- fórmula exacta y convención de la nueva omega;
- semántica temporal de omega_mid / omega_hat;
- tratamiento de dt irregular;
- tratamiento de muestras rechazadas;
- GTests añadidos;
- total GTests;
- builds;
- prueba 276;
- repetición si funciona;
- comparación 270 vs 273 vs 276;
- correlación y lag;
- mismatch direccional;
- energía angular;
- hover success;
- decisión sobre siguiente prueba.
```

Actualizar:

```text
historial_5H_RESUMEN.md
07_ULTIMA_SESION.md
contrato 5H
```

sin borrar historial anterior.

---

# 20. Resumen ejecutivo

Las pruebas 273-275 demuestran:

```text
pose a 20 Hz:
válida

hold:
válido

extrapolación SO(3):
válida

omega GT exacta:
estabiliza todo
```

Por tanto:

> **el siguiente trabajo debe reemplazar la derivación/filtrado actual de `omega_motion` por una estimación causal de velocidad angular que compense que la diferencia entre dos poses representa el intervalo pasado.**

Primera propuesta:

```text
últimas 3 poses
    ->
2 velocidades de intervalo
    ->
aceleración estimada
    ->
proyectar omega desde el centro del último intervalo hasta t_k
    ->
omega_motion actual
```

No tocar otros subsistemas hasta validar esta ruta.
