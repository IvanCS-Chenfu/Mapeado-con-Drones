# Fase 5H — Diagnóstico del fallo con delay tras prueba 278
## Objetivo: separar límite de extrapolación vs propagación temporal incompleta de `omega`

## 0. Estado actual

La prueba 278 con:

```text
GT perfecto ~20 Hz
+ delay fijo ~80 ms
+ nuevo estimador causal de omega_motion
+ control/publicación 50 Hz
```

NO completó el hover.

Resultados documentados:

```text
visual age media: ~0.11285 s
predicciones clamped: ~72.19 %
RMSE angular: ~1.44719 rad/s
energía total: +0.028840 J
gobierno diagnóstico: ~3.00 s
sin fallback ni pérdida de tracking antes del fallo
```

La batería debe seguir detenida:

```text
279 / 280 / 281 NO ejecutar todavía
```

El objetivo inmediato es saber exactamente qué falla entre:

```text
estado estimado correctamente en t_k
        ->
estado que necesita el controlador en now
```

---

# 1. Qué NO se debe tocar

Mantener congelado el estimador causal ya validado en 276/277:

```text
TWO_SAMPLE
THREE_SAMPLE_PREDICTED
omega_mid
alpha_hat
omega_hat_k
```

No modificar:

```text
SMALL / MODERATE
raw gates
bias/deadband
decay
KF/reference
W
mux
control gains
trayectoria
ORB-SLAM3 core
```

No implementar `Delta_target`.

No conectar ORB real todavía.

---

# 2. Problema demostrado nº1 — el horizonte actual se queda corto

En el código actual:

```text
Predict()
```

limita:

```text
requested_dt
```

a:

```text
config_.max_extrapolation_sec
```

y en la prueba 278 el límite efectivo es aproximadamente:

```text
0.100 s
```

Sin embargo, la edad visual observada llega frecuentemente a:

```text
0.11
0.12
0.13
0.14 s
```

y gran parte de las publicaciones aparecen como:

```text
prediction_horizon=0.100000
prediction_clamped=true
```

Por tanto, antes de cambiar ninguna fórmula hay que responder:

> ¿Cuánto del fallo se debe simplemente a que estamos dejando voluntariamente el estado varios milisegundos por detrás del presente?

---

# 3. PRUEBA 278B — sólo ampliar el horizonte máximo

## Cambio

Modificar ÚNICAMENTE para el modo diagnóstico el límite de extrapolación para que cubra holgadamente la edad normal observada.

Valor inicial sugerido para laboratorio:

```text
0.18 s
```

o un valor equivalente justificado a partir del percentil alto de `visual_age`.

NO modificar ninguna otra fórmula.

En particular, mantener:

```text
omega constante durante Propagate()
```

como está ahora.

## Objetivo

Aislar:

```text
clamp temporal
```

de cualquier otro problema.

## Métricas

Comparar 278 vs 278B:

```text
visual_age mean/max
prediction_horizon mean/max
prediction_clamped %
RMSE omega
MAE omega
lag
mismatch direccional
max |er|
max |ew|
energía tau_er
energía tau_ew
energía total
hover completo sí/no
```

---

# 4. Interpretación de 278B

## Si 278B completa el hover

Conclusión:

> El límite de `0.1 s` era una causa funcional suficiente o dominante.

En ese caso:

1. repetir 278B una segunda vez;
2. comprobar reproducibilidad;
3. NO implementar todavía propagación con `alpha_hat`;
4. sólo entonces continuar con 279.

El límite final NO debe quedar elegido arbitrariamente: debe definirse a partir del timing real de ORB con margen de seguridad.

---

## Si 278B falla

No cambiar más thresholds.

Pasar a 278C.

La interpretación será:

> Quitar el clamp no basta. `omega_hat_k` es una buena estimación en `t_k`, pero el estado sigue sin llevarse correctamente desde `t_k` hasta `now`.

---

# 5. Problema probable nº2 — `omega` se congela entre `t_k` y `now`

Actualmente el nuevo estimador calcula:

```text
omega_hat_k
alpha_hat
```

pero `Predict()/Propagate()` utiliza esencialmente:

```text
R(now) =
Exp(angular_velocity * dt) * R(t_k)
```

con:

```text
angular_velocity = constante
```

y devuelve esa misma velocidad angular.

Eso es correcto si la velocidad angular apenas cambia.

Pero durante una oscilación y con:

```text
80-140 ms
```

de horizonte, puede cambiar mucho.

La hipótesis de 278C será:

> `omega_hat_k` es correcta en el timestamp de medida, pero se queda temporalmente antigua durante el intervalo retrasado.

---

# 6. PRUEBA 278C — propagar `omega` y orientación usando `alpha_hat`

Ejecutar sólo si 278B falla.

## Cambio conceptual

Conservar:

```text
omega_hat_k
alpha_hat
```

como estado causal estimado en `t_k`.

Para un target temporal:

```text
dt = t_target - t_k
```

calcular conceptualmente:

```text
omega_pred =
omega_hat_k
+
alpha_hat * dt
```

y propagar orientación de manera coherente con aceleración angular aproximadamente constante:

```text
delta_theta =
omega_hat_k * dt
+
0.5 * alpha_hat * dt^2
```

```text
R_pred =
Exp(delta_theta) * R(t_k)
```

IMPORTANTE:

Codex debe revisar antes:

```text
convención SO(3)
frame de omega
left/right multiplication
```

No copiar ciegamente la fórmula si la convención actual exige el orden contrario.

---

# 7. Coherencia obligatoria de salida en 278C

La salida debe representar el MISMO instante temporal:

```text
pose = R_pred(t_target)
omega = omega_pred(t_target)
```

No publicar:

```text
pose(now) + omega(t_k)
```

ni:

```text
pose(t_k) + omega(now)
```

Ésta es una condición arquitectónica central.

---

# 8. Límites de seguridad en 278C

Mantener/aplicar límites físicos:

```text
max angular speed
max angular acceleration
max prediction horizon
finite checks
```

Pero el horizonte máximo debe cubrir el delay normal de laboratorio.

No usar otra vez un límite que deje sistemáticamente el estado atrás.

Registrar:

```text
omega_prediction_to_now_horizon
omega_prediction_clamped
omega_hat_k
alpha_hat
omega_pred_now
delta_theta_pred
```

---

# 9. GTests para 278C

Añadir como mínimo:

## A — alpha = 0

Esperado:

```text
resultado equivalente al Propagate actual
```

## B — aceleración angular constante conocida

Esperado:

```text
omega_pred(t)
y
R_pred(t)
```

coinciden con la integración analítica dentro de tolerancia.

## C — cambio de signo durante el delay

Ejemplo:

```text
omega_hat_k > 0
alpha_hat < 0
```

y el cruce ocurre antes de `now`.

Esperado:

```text
omega_pred_now < 0
```

si físicamente corresponde.

Esto es especialmente importante.

## D — clamp temporal

Si el horizonte supera el máximo permitido:

```text
clamp correcto y telemetría explícita
```

## E — coherencia pose/omega

Verificar que ambas corresponden al mismo `t_target`.

---

# 10. PRUEBA 278C

Repetir exactamente:

```text
GT perfecto ~20 Hz
delay fijo ~80 ms
mismo hover
```

sin jitter todavía.

Comparar:

```text
278
278B
278C
```

con las mismas métricas.

---

# 11. Árbol de diagnóstico

```text
278
FALLA
 |
 v
278B
sólo quitar clamp insuficiente
 |
 +-- FUNCIONA
 |      ->
 |   causa dominante:
 |   horizonte máximo demasiado corto
 |
 +-- FALLA
        |
        v
278C
propagar omega + R con alpha_hat
        |
        +-- FUNCIONA
        |      ->
        |   causa:
        |   estado correcto en t_k,
        |   propagación t_k -> now incompleta
        |
        +-- FALLA
               ->
           analizar modelo temporal restante
           antes de tocar ORB/gates/control
```

---

# 12. Cuándo volver a 279

NO ejecutar 279 hasta conseguir:

```text
hover completo
```

con delay fijo de ~80 ms.

Preferiblemente:

```text
dos ejecuciones consecutivas correctas
```

Una vez conseguido:

```text
279 = jitter/timing realista
```

Después, sólo si 279 funciona:

```text
280 = ORB real
```

---

# 13. Qué debe devolver Codex

Al terminar este bloque:

```text
Resultado diagnóstico:
CONSEGUIDO / PARCIAL / NO CONSEGUIDO
```

Incluir:

```text
- archivos modificados;
- valor anterior y nuevo de max_extrapolation_sec;
- prueba 278B;
- si 278B funciona o falla;
- porcentaje clamped antes/después;
- RMSE/MAE/lag/mismatch;
- energía angular;
- hover success;

si 278B falla:
- implementación exacta de propagación con alpha_hat;
- convención SO(3) utilizada;
- GTests añadidos;
- prueba 278C;
- comparación 278 / 278B / 278C;
- conclusión causal explícita;

- decisión:
    continuar a 279
    o mantener STOP.
```

Actualizar:

```text
historial_5H_RESUMEN.md
07_ULTIMA_SESION.md
```

sin borrar historial anterior.

---

# 14. Resumen ejecutivo

276/277 ya demostraron:

```text
omega estimada correctamente en t_k -> estable
```

278 demuestra:

```text
añadir delay -> vuelve a fallar
```

La investigación inmediata debe separar:

```text
A) el predictor se corta demasiado pronto
```

de:

```text
B) el predictor lleva pose y omega desde t_k hasta now
   usando una dinámica demasiado pobre
```

Primero:

```text
278B = sólo ampliar horizonte
```

Si no basta:

```text
278C = usar alpha_hat para predecir omega y orientación hasta now
```

No tocar ningún otro subsistema hasta saber cuál de estas dos causas explica el fallo.
