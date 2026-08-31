# Fase 5H — Nueva estimación causal de `v_hat(t_k)` y validación del estado translacional completo
## Objetivo: corregir la condición inicial lineal que alimenta al predictor dinámico translacional antes de volver a ORB real

## 0. Fuente de verdad

Trabajar sobre el estado ACTUAL del repositorio:

```text
https://github.com/IvanCS-Chenfu/Mapeado-con-Drones
```

Tomar como referencia directa:

```text
historial_5H_RESUMEN.md
prueba_309.log
prueba_310.log
prueba_313.log
código vigente de navigation-state-estimator.cpp
predictor dinámico angular validado
BodyThrustDynamicPredictor validado
```

Mantener detenidas:

```text
300
301
302
311
312
```

No conectar todavía ORB real.

---

# 1. Estado actual demostrado

Las pruebas recientes han cerrado varias piezas.

## Predictor dinámico translacional básico

Se ha implementado:

```text
BodyThrustDynamicPredictor
```

usando:

```text
thrust sellado
masa = 1.4 kg
gravedad
timestamps reales
R_dynamic(t)
```

Resultados:

```text
309: CONSEGUIDA
313: CONSEGUIDA
```

Ambas parten de:

```text
p_GT(t_k)
v_GT(t_k)
```

como estado inicial retardado y después propagan con el modelo dinámico sin usar GT actual.

Por tanto:

> La dinámica translacional `t_k -> now` queda validada cuando el estado inicial `p(t_k), v(t_k)` es correcto.

---

## Prueba 310

La prueba 310 sustituye el estado inicial perfecto por el estado lineal estimado actual.

Resultado:

```text
310: NO CONSEGUIDA
```

Por el STOP acordado no se ejecutaron:

```text
311
312
300-302
```

Conclusión vigente:

> El bloqueo actual está en la estimación causal de `v_hat(t_k)` y posiblemente en la coherencia conjunta `p(t_k), v(t_k)`.

---

# 2. Problema del estimador lineal actual

Auditar primero el código vigente.

Actualmente `linear_velocity_` se obtiene conceptualmente a partir de:

```text
target_position =
predicted.translation()
+
position_alpha * position_innovation
```

y después:

```text
desired_linear_velocity =
(target_position - pose_previous.translation()) / dt
```

con:

```text
clamp de velocidad
clamp de aceleración
```

Por tanto, la `linear_velocity_` actual mezcla:

```text
movimiento físico
+
predicción anterior
+
corrección de posición
+
position_alpha
+
limitadores
```

No asumir que eso representa limpiamente:

```text
v(t_k)
```

que es exactamente lo que necesita el predictor dinámico translacional como condición inicial.

No modificar todavía `position_alpha` ni los límites por intuición.

---

# 3. Objetivo de esta iteración

Construir una velocidad lineal causal con la misma semántica temporal que ya funcionó para `omega_hat(t_k)`.

La idea será:

```text
últimas posiciones visuales válidas
        ↓
velocidades medias de intervalos
        ↓
aceleración lineal estimada entre midpoints
        ↓
proyección hasta t_k
        ↓
v_hat(t_k)
```

Después:

```text
p(t_k)
+
v_hat(t_k)
+
thrust
+
masa
+
gravedad
+
R_dynamic(t)
        ↓
BodyThrustDynamicPredictor
        ↓
p(now), v(now)
```

---

# 4. Nueva estimación causal de `v_hat(t_k)`

Usar las tres últimas posiciones visuales válidas:

```text
p(k-2), t(k-2)
p(k-1), t(k-1)
p(k),   t(k)
```

Calcular:

```text
dt_1 = t(k-1) - t(k-2)
dt_2 = t(k)   - t(k-1)
```

y:

```text
v_mid_1 =
[p(k-1) - p(k-2)] / dt_1

v_mid_2 =
[p(k) - p(k-1)] / dt_2
```

Estas velocidades representan aproximadamente:

```text
t_mid_1 = (t(k-2)+t(k-1))/2
t_mid_2 = (t(k-1)+t(k))/2
```

Después:

```text
a_hat =
(v_mid_2 - v_mid_1)
/
(t_mid_2 - t_mid_1)
```

y proyectar únicamente hasta el timestamp de la última medida:

```text
v_hat_k =
v_mid_2
+
a_hat * (t_k - t_mid_2)
```

Conceptualmente, con dt aproximadamente uniforme:

```text
v_hat_k
≈
v_mid_2 + a_hat * dt_2 / 2
```

---

# 5. Importante: NO extrapolar visualmente hasta `now`

La estimación causal nueva termina en:

```text
t_k
```

No usar:

```text
a_hat visual
```

para predecir todo el intervalo:

```text
t_k -> now
```

porque durante ese tiempo ya conocemos información física mejor:

```text
thrust aplicado
R_dynamic(t)
masa
gravedad
```

Por tanto:

```text
visión
-> v_hat(t_k)

dinámica
-> v(now)
```

Mantener claramente esta separación.

---

# 6. Estado inicial de posición

Para la primera implementación:

```text
p_initial = posición visual aceptada en t_k
v_initial = nueva v_hat(t_k)
```

No intentar todavía crear otra `p_hat(t_k)` distinta de la medida visual.

La posición medida ya corresponde a:

```text
t_k
```

y el problema principal inmediato es obtener una velocidad coherente con ella.

Si después las pruebas demuestran un problema específico de posición base, se estudiará por separado.

---

# 7. Tratamiento de historial insuficiente

Crear estados explícitos equivalentes al estimador angular.

## Primera posición

```text
v_hat = 0
velocity_valid = false
mode = INIT
```

## Dos posiciones válidas

```text
v_hat = v_mid
mode = TWO_SAMPLE
```

## Tres posiciones válidas con dt bueno

```text
v_hat = v_mid_2 + a_hat * horizon
mode = THREE_SAMPLE_PREDICTED
```

No inventar aceleración si no existe suficiente historial.

---

# 8. Tratamiento de `dt`

No asumir 20 Hz fijo.

Usar timestamps reales.

Definir/reutilizar categorías conceptualmente equivalentes a:

```text
GOOD_DT
DEGRADED_DT
INVALID_DT
```

Para intervalos degradados:

```text
NO usar automáticamente una aceleración de tres muestras poco fiable
```

Preferencia inicial:

```text
degradar a TWO_SAMPLE
```

y documentarlo.

No cambiar thresholds globales de raw sin justificación.

---

# 9. Muestras rechazadas

Una posición visual rechazada no debe entrar en el historial causal de velocidad.

No actualizar con ella:

```text
v_mid
a_hat
v_hat_k
```

Cuando se invalide:

```text
map_epoch
continuidad local
referencia geométrica necesaria
```

resetear el historial lineal cuando corresponda.

---

# 10. Separar corrección de posición y velocidad física

No hacer que una corrección de pose produzca artificialmente:

```text
v_hat
```

La velocidad física debe salir de:

```text
evolución temporal de posiciones visuales aceptadas
```

No de:

```text
position_alpha * innovation
```

ni de un anclaje/corrección instantánea.

Este punto es análogo a la separación ya conseguida entre:

```text
corrección angular
```

y:

```text
omega física
```

---

# 11. Telemetría obligatoria del nuevo estimador lineal

Añadir:

```text
linear_estimator_mode

p_k2
p_k1
p_k

dt_1
dt_2

v_mid_prev
v_mid_current

t_mid_prev
t_mid_current

a_hat_linear

v_hat_k

linear_prediction_horizon_to_tk

raw_translation_step
raw_linear_speed
raw_linear_acceleration

linear_dt_quality
linear_sample_accepted
```

Durante pruebas GT registrar también:

```text
v_GT_tk
```

sólo como truth externa.

---

# 12. GTests obligatorios

Mantener todos los tests actuales.

Añadir como mínimo:

## A — velocidad constante

Secuencia:

```text
p(t) = p0 + v*t
```

Esperado:

```text
a_hat ~= 0
v_hat_k ~= v real
```

---

## B — aceleración lineal constante

Generar:

```text
p(t) = p0 + v0*t + 0.5*a*t^2
```

Esperado:

```text
v_hat_k
```

más cercana a:

```text
v(t_k)
```

que:

```text
v_mid_2
```

---

## C — cambio de signo

Movimiento:

```text
v positiva
-> cruza 0
-> v negativa
```

Esperado:

```text
v_hat_k cambia de signo con bajo lag
```

sin overshoot absurdo.

---

## D — dt irregular

Usar timestamps no uniformes.

Esperado:

```text
no asumir periodo fijo
```

---

## E — DEGRADED_DT

Esperado:

```text
degradación segura
sin a_hat artificial grande
```

---

## F — muestra rechazada

Esperado:

```text
no entra en historial
```

---

## G — cambio de epoch/reset

Esperado:

```text
historial lineal reiniciado
```

---

## H — corrección de posición no crea velocidad física artificial

Aplicar una corrección instantánea de pose/base.

Esperado:

```text
v_hat no recibe un impulso ficticio
```

---

# 13. Cuidado especial con el contacto con el suelo

Durante el arranque puede existir:

```text
thrust = 0
```

mientras Gazebo mantiene físicamente el dron apoyado en el suelo.

El modelo dinámico libre:

```text
a = g
```

no incluye contacto con el suelo.

Por tanto:

> No usar ese intervalo previo al vuelo para evaluar RMSE funcional del predictor dinámico.

Separar en métricas:

```text
PRE_FLIGHT / CONTACT
```

de:

```text
AIRBORNE / CONTROL_ACTIVE
```

o usar un criterio equivalente ya disponible.

La validación importante debe hacerse durante vuelo efectivo.

No añadir un modelo de contacto al predictor de Fase 5.

---

# 14. Prueba 314 — nueva `v_hat(t_k)` + dinámica translacional, angular GT

Después de implementar y pasar GTests, ejecutar:

```text
timing/jitter equivalente a 299
```

Usar:

```text
p_initial = p_visual(t_k)
v_initial = NUEVA v_hat(t_k)

BodyThrustDynamicPredictor:
    thrust
    masa
    gravedad
    R_dynamic(t)
```

Para aislar el canal lineal:

```text
R_control     = R_GT(now)
omega_control = omega_GT(now)

p_control = p_dynamic(now)
v_control = v_dynamic(now)
```

GT actual angular se usa sólo como laboratorio.

No usar:

```text
p_GT(now)
v_GT(now)
```

en control.

---

# 15. Métricas de 314

Comparar especialmente:

```text
v_hat(t_k)
vs
v_GT(t_k)
```

y:

```text
v_dynamic(now)
vs
v_GT(now)
```

Calcular durante vuelo:

```text
RMSE v_hat_tk
MAE v_hat_tk
lag v_hat_tk

RMSE v_dynamic_now
MAE v_dynamic_now

RMSE p_dynamic_now
MAE p_dynamic_now

max |v error|
max |p error|

ep
ev
F_des

scenario success
govern time
fallback
tracking
```

Comparar contra:

```text
310
309
313
```

---

# 16. Criterio de 314

## Si completa el hover

Considerar:

```text
v_hat(t_k) causal + dinámica = VALIDADO
```

Repetir exactamente como:

```text
315
```

No pasar a estado completo con una sola ejecución.

---

## Si falla

STOP.

No ejecutar 315-317.

Analizar:

```text
v_mid
a_hat
v_hat_k
v_GT_tk
p_initial
force buffer
R_dynamic
```

y determinar si el fallo ocurre:

```text
antes de la dinámica
```

o:

```text
durante t_k -> now
```

No tocar gains.

---

# 17. Prueba 315 — repetición

Si 314 funciona:

```text
repetir misma configuración
```

Criterio:

```text
314 = CONSEGUIDA
315 = CONSEGUIDA
```

para declarar reproducible la nueva condición inicial translacional.

---

# 18. Prueba 316 — estado completo dinámico bajo jitter sin GT en control

Sólo si 314/315 funcionan.

Usar:

```text
p_control = p_dynamic(now)
v_control = v_dynamic(now)

R_control = R_dynamic(now)
omega_control = omega_dynamic(now)
```

Cadena:

```text
poses GT retardadas con timing/jitter realista
    ->
v_hat(t_k) causal
omega_hat(t_k) causal
    ->
thrust + torque + m + J + g
    ->
p/v/R/omega(now)
    ->
control
```

GT exclusivamente para métricas.

No usar GT en ninguna componente del estado de control.

---

# 19. Prueba 317 — repetición del estado completo

Si 316 completa:

```text
repetir como 317
```

Criterio:

```text
316 = CONSEGUIDA
317 = CONSEGUIDA
```

Entonces considerar:

```text
ESTADO COMPLETO BAJO TIMING/JITTER REALISTA = VALIDADO
```

---

# 20. Qué hacer después de 316/317

Sólo si ambas funcionan:

reanudar el plan de ORB real.

Secuencia sugerida:

```text
318:
ORB REAL hover

319:
repetición ORB REAL hover
```

No ejecutar trayectoria todavía.

Primero conseguir:

```text
dos hovers ORB completos
sin caída inducida por control
```

---

# 21. Si ORB real falla después

Si:

```text
316/317 funcionan
```

pero:

```text
318 falla
```

no volver automáticamente a tocar:

```text
predictor angular
predictor translacional
masa
J
buffers
timing
```

El siguiente foco deberá ser:

```text
calidad real de pose ORB
cambios de reference KF
SMALL/MODERATE
raw rejection
tracking
```

---

# 22. Qué NO hacer

No modificar:

```text
Kp
Kv
Kr
Kw
```

No tocar:

```text
J
masa
modelo dinámico angular
BodyThrustDynamicPredictor
SMALL/MODERATE
KF/reference
W
mux
```

salvo que aparezca evidencia directa.

No implementar:

```text
EKF
nuevo filtro pasa-bajos
Delta_target
modelo de contacto con suelo
```

---

# 23. Limitación secundaria 304

Mantener documentado que:

```text
304
```

falló al propagar un estado angular GT interpolado en `t_k`.

No investigar ahora esa ruta diagnóstica.

La ruta angular productiva ya quedó validada por:

```text
303
```

bajo timing/jitter con p/v correctas.

No mezclar esa deuda con el trabajo lineal.

---

# 24. Qué debe devolver Codex

Al terminar esta autorización:

```text
Resultado:
CONSEGUIDO / PARCIAL / NO CONSEGUIDO
```

Incluyendo:

```text
- estado/commit usado;
- archivos modificados;

- auditoría de linear_velocity_ antigua;
- fórmula exacta de nueva v_hat(t_k);
- semántica temporal;
- comportamiento GOOD/DEGRADED/INVALID dt;
- resets;
- tratamiento de muestras rechazadas;

- GTests añadidos;
- total GTests;
- builds;
- analyzer tests;
- git diff --check;

- prueba 314;
- métricas v_hat(t_k) vs GT(t_k);
- métricas p/v(now);
- hover success;

- prueba 315 si corresponde;

- prueba 316 si corresponde;
- prueba 317 si corresponde;

- comparación:
    309
    313
    310
    314
    315
    316
    317

- conclusión:
    V_HAT_TK VALIDADA / NO VALIDADA
    PREDICTOR TRANSLACIONAL COMPLETO VALIDADO / NO VALIDADO
    ESTADO COMPLETO JITTER VALIDADO / NO VALIDADO

- decisión:
    continuar a ORB real
    o mantener STOP.
```

Actualizar:

```text
historial_5H_RESUMEN.md
07_ULTIMA_SESION.md
contrato 5H
```

si cambia el contrato arquitectónico.

No borrar historial anterior.

---

# 25. Resumen ejecutivo

El estado actual es:

```text
omega_hat(t_k) causal                    ✅
torque + J -> omega(now)                 ✅
R_dynamic(now)                           ✅

thrust + m + g + R(t)
con p/v GT(t_k) -> p/v(now)              ✅ 309/313

estado inicial p/v estimado actual       ❌ 310
```

El siguiente trabajo debe centrarse en:

```text
p(k-2), p(k-1), p(k)
        ->
v_mid_1, v_mid_2
        ->
a_hat
        ->
v_hat(t_k)
```

y después dejar que la dinámica física ya validada haga:

```text
t_k -> now
```

Secuencia:

```text
314:
nueva v_hat(t_k) + dinámica translacional + angular GT

315:
repetición

316:
estado completo dinámico bajo jitter

317:
repetición
```

> No volver a ORB real hasta que 316/317 completen el hover sin usar GT en ninguna componente del estado de control.
