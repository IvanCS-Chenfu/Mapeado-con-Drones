# Fase 5H — Batería 303–306 para aislar el fallo restante con timing/jitter realista
## Objetivo: separar fallo angular inicial (`omega(t_k)`) vs fallo translacional (`p/v`) antes de modificar otra vez la arquitectura

## 0. Fuente de verdad

Trabajar sobre el estado ACTUAL del repositorio:

```text
https://github.com/IvanCS-Chenfu/Mapeado-con-Drones
```

Tomar como referencia el estado actual documentado de Fase 5H y la prueba 299.

Mantener detenidas:

```text
300
301
302
```

No integrar todavía ORB real.

---

# 1. Estado actual

El predictor dinámico angular con:

```text
omega(t_k)
+
torques
+
J real
```

queda validado con delay fijo.

La inercia correcta es:

```text
J = diag(
    0.00803107,
    0.00803107,
    0.015805
) kg·m²
```

Sin embargo, la prueba 299 con timing/jitter realista falla:

```text
gobierno: ~16.94 s
visual age media: ~0.115 s
visual age máxima: ~0.200 s
DEGRADED_DT: 48 intervalos
RMSE omega: ~0.3497 rad/s
energía total: +0.0141 J
fallback: no
tracking loss: no
huecos de torque: no
```

Los rechazos visuales aparecen después de que la divergencia ya haya comenzado.

Por tanto:

> No culpar todavía a tracking, fallback, huecos de torque ni al modelo rígido completo.

---

# 2. Hipótesis que hay que separar

Quedan dos sospechosos principales.

## H1 — condición inicial angular incorrecta en gaps largos

Cuando aparece:

```text
DEGRADED_DT
```

la velocidad obtenida desde dos poses puede representar la velocidad MEDIA de un intervalo largo, no necesariamente:

```text
omega(t_k)
```

al final del intervalo.

Entonces:

```text
omega(t_k) incorrecta
    ->
predictor dinámico correcto
    ->
omega(now) incorrecta
```

---

## H2 — estado translacional `p/v` retardado o incoherente

La prueba 299 usa:

```text
position_source=PREDICTED
velocity_source=PREDICTED
```

y el canal translacional bajo timing realista todavía no está validado de forma limpia.

Errores en:

```text
p
v
```

modifican:

```text
ep
ev
F_des
R_des
```

y pueden acabar excitando también el lazo angular.

---

# 3. Regla principal

NO modificar todavía:

```text
J
predictor dinámico
integrador angular
buffer de torque
control gains
SMALL/MODERATE
raw gates
KF/reference
W
mux
trayectoria
```

No implementar todavía:

```text
nuevo filtro
nuevo estimador
repropagación retardada completa
EKF
Delta_target
```

Primero ejecutar la batería 303–306.

---

# 4. Prueba 303 — `p/v GT(now)` + angular dinámico actual

Usar:

```text
p_control     = p_GT(now)
v_control     = v_GT(now)

R_control     = R_dynamic(now)
omega_control = omega_dynamic(now)
```

Mantener exactamente:

```text
timing/jitter realista de 299
estimador causal actual
DEGRADED_DT actual
predictor dinámico actual
torques reales
J real
```

## Pregunta

> ¿El bloque angular completo actual soporta el jitter si eliminamos por completo el error translacional?

## Interpretación

Si 303 completa el hover:

```text
el problema principal restante está en p/v
```

Si 303 falla:

```text
el problema angular sigue siendo relevante
```

y hay que ejecutar 304.

---

# 5. Prueba 304 — `p/v GT(now)` + `omega_GT(t_k)` como condición inicial + dinámica hasta now

Ejecutar aunque 303 funcione, para obtener una comparación limpia.

Configurar:

```text
p_control = p_GT(now)
v_control = v_GT(now)
```

Para el estado angular:

```text
R inicial = R_GT(t_k)
omega inicial = omega_GT(t_k)
```

pero desde:

```text
t_k -> now
```

usar únicamente:

```text
torques
J
predictor dinámico
```

No usar:

```text
omega_GT(now)
R_GT(now)
```

como salida de control.

## Pregunta

> Si la condición inicial angular en `t_k` es perfecta, ¿el predictor dinámico sigue funcionando bajo los mismos periodos largos/jitter de 299?

## Interpretación clave

Si:

```text
303 falla
304 funciona
```

concluir:

> El predictor dinámico `t_k -> now` funciona; el fallo está antes, en la estimación de `omega(t_k)` bajo `DEGRADED_DT`.

Éste es uno de los resultados más importantes de la batería.

---

# 6. Prueba 305 — `p/v predichas` + estado angular GT(now)

Configurar:

```text
p_control = p_pred(now)
v_control = v_pred(now)

R_control     = R_GT(now)
omega_control = omega_GT(now)
```

Mantener el mismo timing/jitter de 299.

## Pregunta

> ¿Las p/v predichas con ese timing son por sí solas suficientes para desestabilizar el hover?

## Interpretación

Si 305 falla:

```text
p/v también necesitan corrección temporal
```

Si 305 completa:

```text
p/v actuales son suficientemente buenas
y el fallo principal restante es angular
```

---

# 7. Prueba 306 — estado GT completo actual

Configurar:

```text
p_control     = p_GT(now)
v_control     = v_GT(now)
R_control     = R_GT(now)
omega_control = omega_GT(now)
```

Mantener el mismo escenario y timing diagnóstico alrededor.

Éste es el sanity check.

## Regla

Si 306 falla:

```text
STOP TOTAL
```

La batería queda invalidada y hay que revisar el montaje/pipeline/controlador.

Si 306 funciona:

```text
el diagnóstico 303–305 es válido
```

---

# 8. Orden obligatorio

Ejecutar:

```text
303
304
305
306
```

No recalibrar nada entre pruebas.

No usar el resultado de una para cambiar thresholds antes de ejecutar la siguiente.

---

# 9. Telemetría obligatoria

Registrar explícitamente:

```text
control_stamp

p_pred
p_GT_now
p_used

v_pred
v_GT_now
v_used

R_dynamic_now
R_GT_now
R_used

omega_hat_k
omega_GT_tk
omega_dynamic_now
omega_GT_now
omega_used

position_source
velocity_source
orientation_source
omega_source

measurement_stamp
arrival_stamp
visual_age
dt_visual
raw_class
omega_estimator_mode

DEGRADED_DT count

torque_samples_used
missing_torque_interval
dynamic_horizon

ep
ev
er
ew

F_des

tau_er
tau_ew
tau_total

P_total_GT

fallback
tracking
```

---

# 10. Métricas comparativas

Comparar:

```text
299
303
304
305
306
```

con:

```text
scenario success
govern time

RMSE p
RMSE v
RMSE R
RMSE omega

MAE omega
lag omega
mismatch direccional

max |ep|
max |ev|
max |er|
max |ew|

energia tau_er
energia tau_ew
energia total

DEGRADED_DT count
max visual age
mean visual age

first divergence timestamp
first raw reject timestamp

fallback
tracking loss
```

---

# 11. Tabla mínima de entrega

```text
Prueba   p       v       R/omega inicial o usada                 Resultado
------------------------------------------------------------------------------
299      pred    pred    causal + dinámica                       actual: falla
303      GT      GT      causal + dinámica                       ?
304      GT      GT      GT(t_k) + dinámica hasta now            ?
305      pred    pred    GT(now)                                  ?
306      GT      GT      GT(now)                                  ?
```

---

# 12. Árbol de diagnóstico

## Caso A

```text
303 falla
304 funciona
305 funciona
306 funciona
```

Conclusión:

> El problema principal está en `omega(t_k)` bajo `DEGRADED_DT`.

El predictor dinámico `t_k -> now` queda validado también con jitter.

Siguiente diseño:

```text
no reemplazar omega por DeltaR/dt en gaps largos;
mantener un estado dinámico continuo y usar la nueva medida visual como corrección
```

NO implementarlo todavía bajo esta autorización.

---

## Caso B

```text
303 funciona
305 falla
306 funciona
```

Conclusión:

> El bloque angular ya soporta jitter; el problema principal está en `p/v`.

El siguiente trabajo deberá estudiar la compensación temporal translacional.

---

## Caso C

```text
303 falla
304 funciona
305 falla
306 funciona
```

Conclusión:

> Hay DOS problemas:
> 1. condición inicial angular `omega(t_k)` en gaps largos;
> 2. estado translacional `p/v`.

No intentar resolver ambos a la vez sin una nueva estrategia acordada.

---

## Caso D

```text
303 falla
304 falla
306 funciona
```

Conclusión:

> Incluso partiendo de estado angular perfecto en `t_k`, el predictor dinámico no soporta el timing/jitter actual.

Revisar específicamente:

```text
buffer de torque
alineación temporal torque/estado
intervalos largos
integración
timestamps
```

No culpar a ORB.

---

## Caso E

```text
305 funciona
```

Conclusión:

```text
p/v predichas no son causa suficiente
```

y el foco vuelve al bloque angular.

---

## Caso F

```text
306 falla
```

Conclusión:

```text
SANITY CHECK INVALIDADO
```

STOP.

---

# 13. Si se confirma problema en `omega(t_k)` bajo `DEGRADED_DT`

NO implementar todavía, pero documentar como propuesta siguiente:

```text
mantener R/omega propagadas continuamente con torques a 50 Hz
```

y cuando llegue una nueva pose visual atrasada:

```text
1. localizar/reconstruir el estado correspondiente a su timestamp;
2. usar la pose visual como corrección del estado;
3. NO sustituir omega por una media DeltaR/dt de un intervalo largo;
4. repropagar con historial de torque hasta now.
```

Conceptualmente:

```text
estado dinámico continuo
        |
        +---- torque ----> predicción
        |
llega pose visual retardada
        |
        v
corrección en t_visual
        |
        v
repropagación hasta now
```

Pero esta arquitectura requiere una autorización posterior.

---

# 14. Si se confirma problema en `p/v`

No tocar todavía gains.

El siguiente diagnóstico deberá estudiar de forma equivalente:

```text
v(t_k) -> v(now)
p(t_k) -> p(now)
```

usando información física disponible:

```text
fuerza aplicada
masa
gravedad
orientación
timestamps
```

pero sólo después de cerrar 303–306.

---

# 15. Restricción sobre GT

GT se utiliza exclusivamente para diagnóstico.

La arquitectura final debe seguir siendo:

```text
ORB
+
estado interno
+
torques/fuerzas propias
```

sin GT durante operación normal.

---

# 16. Builds y tests

Antes de las pruebas:

```text
build paquetes afectados
GTests/CTest
analyzer tests
git diff --check
```

Añadir tests focales para garantizar:

```text
303:
sólo p/v sustituidas por GT

304:
p/v GT + estado inicial angular GT(t_k), pero salida t_k->now dinámica

305:
sólo R/omega sustituidas por GT(now)

306:
estado completo GT(now)
```

Verificar especialmente:

```text
GT(t_k) != GT(now)
```

en 304.

No usar por error `omega_GT(now)` como condición inicial.

---

# 17. Qué debe devolver Codex

Al terminar:

```text
Resultado diagnóstico:
CONSEGUIDO / PARCIAL / NO CONSEGUIDO
```

Incluyendo:

```text
- archivos modificados;
- modos diagnósticos añadidos;
- confirmación de que J/predictor dinámico no se recalibraron;

- resultado 303;
- resultado 304;
- resultado 305;
- resultado 306;

- tabla 299/303/304/305/306;

- RMSE p/v/R/omega;
- lag/mismatch;
- energías;
- DEGRADED_DT;
- edad visual;
- fallback/tracking;

- conclusión explícita:
    OMEGA_TK_DEGRADED PRINCIPAL
    PV PRINCIPAL
    DOS PROBLEMAS
    PREDICTOR_DINAMICO_JITTER
    o
    SANITY INVALIDADO

- siguiente solución recomendada;
- mantener 300-302 detenidas.
```

Actualizar:

```text
historial_5H_RESUMEN.md
07_ULTIMA_SESION.md
```

sin borrar historial previo.

---

# 18. Resumen ejecutivo

La prueba 299 demuestra que:

```text
delay fijo -> predictor dinámico funciona
timing/jitter realista -> estado completo falla
```

pero aún no sabemos si la causa es:

```text
A) mala omega(t_k) en DEGRADED_DT
B) p/v retardadas
C) ambas
```

Ejecutar:

```text
303:
p/v GT + angular dinámico actual

304:
p/v GT + estado angular perfecto en t_k + dinámica

305:
p/v predichas + angular GT actual

306:
estado GT completo
```

> No modificar otra vez la arquitectura hasta que esta batería determine qué componente sigue fallando.
