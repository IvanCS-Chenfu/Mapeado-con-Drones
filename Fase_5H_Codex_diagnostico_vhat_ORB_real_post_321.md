# Fase 5H — Diagnóstico causal de `v_hat(t_k)` con poses ORB reales en shadow

## Objetivo

Localizar exactamente dónde nace el error de velocidad lineal ORB antes de modificar el estimador.

## Fuente de verdad

Trabajar sobre el estado ACTUAL de:

```text
https://github.com/IvanCS-Chenfu/Mapeado-con-Drones
```

Usar como base:

```text
historial_5H_RESUMEN.md
prueba_321AR.log
prueba_321B.log
prueba_321C.log
prueba_321D.log
```

Estado aceptado:

```text
321AR: p_GT + v_ORB + angular ORB -> FALLA
321B:  p_ORB + v_GT + angular ORB -> ESTABLE
321D:  ORB completo -> FALLA
```

Conclusión vigente:

```text
V_ORB PRINCIPAL
```

La velocidad lineal ORB queda aislada como causa principal de la inestabilidad actual.

---

# 1. Qué NO hacer

No modificar todavía:

```text
CausalLinearVelocityEstimator
v_hat(t_k)
BodyThrustDynamicPredictor
predictor dinámico angular
omega_hat(t_k)
J
masa
gravedad
buffers
ZOH
poda
Kp/Kv/Kr/Kw
SMALL/MODERATE
KF policy
W
mux productivo
```

Esta iteración es exclusivamente diagnóstica.

---

# 2. Pregunta que debe responder la batería

Queremos saber dónde se degrada:

```text
poses ORB reales
    ↓
v_mid
    ↓
a_hat
    ↓
v_hat(t_k)
    ↓
propagación física
    ↓
v(now)
```

Distinguir entre:

```text
A. v_mid ya nace mal por ruido/diferenciación;
B. a_hat/proyección THREE_SAMPLE amplifica ruido;
C. el error aparece con dt irregulares/DEGRADED_DT;
D. el error coincide causalmente con cambios de reference KF;
E. v_hat(t_k) es buena pero la propagación t_k->now la degrada.
```

No asumir ninguna antes de medir.

---

# 3. Prueba 322 — ORB real en shadow con hover estable

Usar el mismo esquema limpio post-321:

```text
aproximación gobernada por GT
ORB real activo en shadow
anchor válido
airborne
settled
```

No permitir que `v_ORB` gobierne.

Configuración preferida:

```text
p_control     = p_GT(now)
v_control     = v_GT(now)
R_control     = R_ORB_dynamic(now)
omega_control = omega_ORB_dynamic(now)
```

Si por infraestructura esa combinación no mantiene un hover limpio, usar GT completo únicamente para mantener el dron quieto, pero dejar intacto todo el estimador ORB real en shadow.

Objetivo:

> observar `v_hat` ORB real sin que la propia velocidad errónea mueva el dron y contamine el diagnóstico.

---

# 4. Telemetría obligatoria POR CADA medida ORB aceptada

Registrar:

```text
measurement_stamp

p_k2
p_k1
p_k

t_k2
t_k1
t_k

dt_1
dt_2

t_mid_1
t_mid_2

v_mid_1
v_mid_2

a_hat

linear_prediction_horizon_to_tk

v_hat_tk

v_GT_tk
v_hat_error_tk

linear_estimator_mode
linear_dt_quality
linear_sample_accepted

raw_translation_step
raw_linear_speed
raw_linear_acceleration

reference_kf
reference_changed

map_epoch
tracking

raw_class
correction_class

visual_age
```

No basta con registrar sólo `v_hat`.

---

# 5. GT sincronizado a `t_k`

Para este diagnóstico obtener:

```text
v_GT(t_k)
```

sin compararla con `v_GT(now)`.

Si hace falta interpolar GT entre muestras vecinas, hacerlo sólo para métricas y documentar el skew residual.

GT sigue siendo truth externa, nunca entrada productiva del estimador.

---

# 6. Registrar también `t_k -> now`

Para cada publicación dinámica:

```text
base_stamp = t_k
target_stamp = now

v_hat_tk
v_dynamic_now
v_GT_now

p_dynamic_now
p_GT_now

dynamic_horizon

thrust_samples_used
thrust_coverage

R_dynamic_used
```

Calcular:

```text
error_v_tk  = v_hat_tk - v_GT_tk
error_v_now = v_dynamic_now - v_GT_now
```

Así se separa:

```text
error de condición inicial
```

de:

```text
error añadido por propagación.
```

---

# 7. Métricas principales

Durante `AIRBORNE + SETTLED` calcular:

```text
RMSE(v_mid_2 vs v_GT(t_mid_2))
RMSE(v_hat_tk vs v_GT(t_k))
RMSE(v_dynamic_now vs v_GT(now))

MAE equivalentes

bias X/Y/Z
std X/Y/Z
max error X/Y/Z
```

Si puede hacerse de forma temporalmente correcta:

```text
comparar a_hat con GT acceleration
```

sólo como métrica externa.

---

# 8. Medir amplificación entre etapas

Definir:

```text
E_mid =
|v_mid_2 - v_GT(t_mid_2)|

E_hat =
|v_hat_tk - v_GT(t_k)|

E_dynamic =
|v_dynamic_now - v_GT(now)|
```

Calcular:

```text
gain_hat =
E_hat / max(E_mid, epsilon)

gain_dynamic =
E_dynamic / max(E_hat, epsilon)
```

Resumir:

```text
median
p90
p95
max
```

Interpretación:

```text
gain_hat >> 1
-> a_hat/proyección amplifican ruido

gain_dynamic >> 1
-> la propagación física añade el problema
```

---

# 9. Analizar por calidad temporal

Separar:

```text
GOOD_DT
DEGRADED_DT
INVALID_DT
```

y, si es útil:

```text
<0.055 s
0.055-0.075 s
0.075-0.12 s
0.12-0.20 s
```

Para cada grupo:

```text
RMSE v_mid
RMSE v_hat
RMSE v_dynamic
|a_hat| medio/max
```

No cambiar thresholds.

---

# 10. Analizar cambios de reference KF

Crear ventanas alrededor de:

```text
reference_changed=true
```

por ejemplo:

```text
[-0.5,0) s
[0,+0.1] s
(+0.1,+0.5] s
```

Comparar:

```text
v_mid error
v_hat error
|a_hat|
```

con periodos de referencia estable.

No atribuir causalidad por simple coincidencia.

---

# 11. Analizar correcciones visuales

Separar por:

```text
SMALL
MODERATE_PENDING
MODERATE_CONFIRMED
PREDICT_ONLY
REJECTED
```

y observar:

```text
position step
v_mid
a_hat
v_hat
```

Queremos saber si una corrección visual aceptada está apareciendo indirectamente como velocidad física ficticia.

---

# 12. Prueba 323 — TWO_SAMPLE vs THREE_SAMPLE en paralelo

Ejecutar sólo si 322 muestra:

```text
v_mid_2 razonable
pero v_hat_tk claramente peor
```

No modificar el productivo.

Calcular en shadow, sobre LAS MISMAS muestras ORB:

```text
v_two_sample_tk
v_three_sample_tk
v_GT_tk
```

Preferencia:

```text
una sola simulación
dos estimaciones diagnósticas en paralelo
```

Calcular:

```text
RMSE_two
RMSE_three

MAE_two
MAE_three

p95_two
p95_three
```

Objetivo:

> demostrar si `a_hat` aporta información útil o amplifica ruido ORB.

---

# 13. Árbol de diagnóstico

## Caso A — `v_mid` ya nace mal

Si:

```text
v_mid_2 tiene error grande
y v_hat no empeora mucho
```

Conclusión:

```text
V_MID_ORB
```

La causa base está en ruido/diferenciación de posición.

## Caso B — `a_hat` amplifica

Si:

```text
v_mid_2 razonable
v_hat_tk mucho peor
RMSE_three >> RMSE_two
```

Conclusión:

```text
A_HAT_AMPLIFICATION
```

Ésta es la hipótesis principal actual.

## Caso C — dt

Si el error se concentra claramente en:

```text
DEGRADED_DT / dt grandes
```

Conclusión:

```text
DEGRADED_DT
```

## Caso D — reference KF

Si los picos están sistemáticamente ligados temporalmente a:

```text
reference_changed
```

y no aparecen con referencia estable:

```text
REFERENCE_KF
```

## Caso E — propagación

Si:

```text
v_hat_tk es buena
pero v_dynamic_now es mala
```

Conclusión:

```text
DYNAMIC_PROPAGATION
```

y reabrir entonces thrust/masa/R/timestamps.

---

# 14. Resultado que debe producir Codex

No basta con:

```text
"v_hat es mala"
```

Debe concluir una de estas opciones, con datos:

```text
V_MID_ORB
A_HAT_AMPLIFICATION
DEGRADED_DT
REFERENCE_KF
DYNAMIC_PROPAGATION
MULTICAUSAL
```

y cuantificar el peso de cada efecto relevante.

---

# 15. No modificar todavía la solución

Aunque el resultado sea claro:

```text
NO cambiar v_hat
NO añadir filtro
NO bajar gains
NO añadir clamps
```

al terminar 322/323.

Documentar únicamente la solución recomendada y hacer STOP.

---

# 16. Builds y tests

Antes:

```text
build orbslam3
build dron_individual
build simulacion_dron

GTests/CTest
analizador
git diff --check
```

Añadir tests sólo para telemetría/modos shadow:

```text
1. v_GT_tk se compara con el timestamp correcto.
2. TWO_SAMPLE shadow no altera la salida productiva.
3. THREE_SAMPLE shadow no altera la salida productiva.
4. reference_changed queda etiquetado correctamente.
5. ninguna métrica GT entra en NavigationState productivo.
```

---

# 17. Criterio de validez

322 cuenta sólo si:

```text
ORB tracking permanece OK durante una ventana suficiente
el dron permanece estable por la fuente diagnóstica
ORB sigue generando medidas reales
dynamic predictor sigue activo
sin missing thrust/torque
```

Si aparece una pérdida visual real, usar sólo el tramo válido previo y documentarlo.

---

# 18. Qué debe devolver Codex

```text
Resultado diagnóstico:
CONSEGUIDO / PARCIAL / NO CONSEGUIDO
```

Incluir:

```text
- commit/estado usado;
- archivos modificados;
- confirmación de que v_hat productiva NO cambió;

- resultado 322;

- nº de muestras ORB válidas;
- duración airborne/settled;
- tracking;
- cambios de ref KF;

- RMSE/MAE/bias/std:
    v_mid
    v_hat_tk
    v_dynamic_now;

- métricas por eje;

- métricas GOOD/DEGRADED dt;

- métricas alrededor de cambios KF;

- métricas por correction_class;

- gain_hat;
- gain_dynamic;

- resultado 323 si fue necesario;

- TWO_SAMPLE vs THREE_SAMPLE;

- conclusión explícita:
    V_MID_ORB
    A_HAT_AMPLIFICATION
    DEGRADED_DT
    REFERENCE_KF
    DYNAMIC_PROPAGATION
    o MULTICAUSAL;

- siguiente solución recomendada;

- STOP antes de modificar el estimador.
```

Actualizar:

```text
historial_5H_RESUMEN.md
07_ULTIMA_SESION.md
```

sin borrar historial previo.

---

# 19. Resumen ejecutivo

321 demuestra:

```text
p_ORB + v_GT + angular ORB
-> estable

p_GT + v_ORB + angular ORB
-> falla

ORB completo
-> falla
```

Por tanto:

```text
v_ORB = causa principal actual
```

Ahora no hay que modificarla todavía.

Primero observar con ORB real y dron estable:

```text
p(k-2), p(k-1), p(k)
        ↓
v_mid_1, v_mid_2
        ↓
a_hat
        ↓
v_hat(t_k)
        ↓
dinámica
        ↓
v(now)
```

y comparar cada etapa con GT sincronizado.

Si:

```text
v_mid buena
v_hat mala
```

ejecutar además 323 comparando en shadow:

```text
TWO_SAMPLE
vs
THREE_SAMPLE
```

sobre las mismas muestras.

> El objetivo es localizar matemáticamente qué operación amplifica el error de las poses ORB reales, no aplicar todavía otro parche.
