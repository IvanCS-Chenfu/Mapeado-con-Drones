# Fase 5H — Batería final cruzada con `p(now)` y `v(now)` GT
## Objetivo: aislar definitivamente si el fallo principal está en `omega_pred(now)`

## 0. Fuente de verdad

Trabajar sobre el estado ACTUAL del repositorio:

```text
https://github.com/IvanCS-Chenfu/Mapeado-con-Drones
```

Mantener detenidas:

```text
279
280
281
```

No conectar todavía ORB real.

---

# 1. Estado actual

Las pruebas 285-287 dejan una señal muy fuerte:

```text
285:
R_pred + omega_GT(now)
-> ~13.12 s
-> energía angular disipativa

286:
R_GT(now) + omega_pred
-> ~2.98 s
-> mismatch ~49 %
-> energía positiva

287:
R_GT(now) + omega_GT(now)
-> ~13.06 s
-> energía angular muy disipativa
-> PERO sigue fallando
```

La prueba 287 NO fue un sanity check completo porque:

```text
posición p
velocidad lineal v
```

seguían procediendo del predictor retardado.

Por tanto, todavía no se puede afirmar de forma definitiva que:

```text
omega_pred(now)
```

sea la única causa.

---

# 2. Objetivo de esta batería

Eliminar completamente el canal translacional como variable del diagnóstico.

En todas las pruebas siguientes usar:

```text
p_control = p_GT(now)
v_control = v_GT(now)
```

y variar únicamente:

```text
R_control
omega_control
```

Así podremos saber si el problema angular principal está realmente en:

```text
omega_pred(now)
```

o también en:

```text
R_pred(now)
```

---

# 3. Regla principal

NO modificar:

```text
control gains
estimador causal
alpha_hat
predictor
SMALL/MODERATE
raw gates
KF/reference
W
mux
trayectoria
delay artificial ~80 ms
frecuencia ~20 Hz de la fuente retardada
```

No implementar todavía:

```text
modelo dinámico con torque
EKF
Delta_target
nuevo filtro
nueva extrapolación
```

Sólo crear los modos diagnósticos necesarios para sustituir selectivamente:

```text
p
v
R
omega
```

---

# 4. Prueba 288 — `p_GT + v_GT + R_pred + omega_pred`

Configurar:

```text
p_control     = p_GT(now)
v_control     = v_GT(now)

R_control     = R_pred(now)
omega_control = omega_pred(now)
```

Objetivo:

> observar el predictor angular completo eliminando cualquier error translacional.

Si 288 falla rápidamente:

```text
el problema sigue estando en el bloque angular predicho
```

Si mejora mucho respecto a 284:

```text
p/v retardadas también contribuían
```

---

# 5. Prueba 289 — `p_GT + v_GT + R_pred + omega_GT`

Configurar:

```text
p_control     = p_GT(now)
v_control     = v_GT(now)

R_control     = R_pred(now)
omega_control = omega_GT(now)
```

Objetivo:

> comprobar si una `omega(now)` correcta basta incluso manteniendo orientación predicha.

Interpretación:

Si 289 funciona:

```text
R_pred(now) es suficientemente buena para hover
cuando omega(now) es correcta
```

---

# 6. Prueba 290 — `p_GT + v_GT + R_GT + omega_pred`

Configurar:

```text
p_control     = p_GT(now)
v_control     = v_GT(now)

R_control     = R_GT(now)
omega_control = omega_pred(now)
```

Objetivo:

> comprobar si `omega_pred(now)` por sí sola puede desestabilizar el hover aun con pose actual perfecta.

Ésta es la prueba más importante para confirmar la sospecha actual.

Interpretación:

Si 290 falla rápidamente mientras 289 funciona:

> `omega_pred(now)` queda prácticamente demostrada como causa angular principal.

---

# 7. Prueba 291 — estado GT actual completo

Configurar:

```text
p_control     = p_GT(now)
v_control     = v_GT(now)
R_control     = R_GT(now)
omega_control = omega_GT(now)
```

Éste sí debe ser el sanity check verdadero.

Objetivo:

> demostrar que el controlador/pipeline sigue siendo estable cuando recibe el estado completo actual perfecto.

## Regla

Si 291 falla:

```text
STOP TOTAL
```

No interpretar 288-290 como diagnóstico definitivo.

Revisar pipeline/controlador/montaje de laboratorio.

Si 291 funciona:

```text
la batería cruzada es válida
```

---

# 8. Orden obligatorio

Ejecutar:

```text
288
289
290
291
```

con el mismo escenario de hover.

No recalibrar nada entre pruebas.

---

# 9. Telemetría obligatoria

Registrar explícitamente en cada publicación/control tick:

```text
control_stamp

p_pred
p_GT_now
p_used

v_pred
v_GT_now
v_used

R_pred
R_GT_now
R_used

omega_pred
omega_GT_now
omega_used

position_source
velocity_source
orientation_source
omega_source

ep
ev
er
ew

F_des
tau_er
tau_ew
tau_total

P_er_GT
P_ew_GT
P_total_GT

visual_age
prediction_horizon

fallback
tracking
```

---

# 10. Métricas principales

Comparar:

```text
284
285
286
287
288
289
290
291
```

con:

```text
scenario success
govern time

max |ep|
max |ev|
max |er|
max |ew|

RMSE p_pred vs p_GT
RMSE v_pred vs v_GT
RMSE R_pred vs R_GT
RMSE omega_pred vs omega_GT

lag p
lag v
lag R
lag omega

omega sign/direction mismatch

energía tau_er
energía tau_ew
energía total angular

primer crecimiento inestable
primer rechazo raw

fallback
tracking loss
```

---

# 11. Tabla mínima esperada

Codex debe devolver algo equivalente a:

```text
Prueba   p       v       R        omega      hover
--------------------------------------------------
288      GT      GT      pred     pred       ?
289      GT      GT      pred     GT         ?
290      GT      GT      GT       pred       ?
291      GT      GT      GT       GT         ?
```

---

# 12. Árbol de diagnóstico

## Caso A — resultado esperado principal

```text
288 falla
289 funciona
290 falla
291 funciona
```

Conclusión:

> `omega_pred(now)` es la causa angular principal.

`R_pred(now)` es suficientemente buena para hover si la omega actual es correcta.

Siguiente paso recomendado:

```text
diseñar predicción dinámica de omega(now)
```

usando información disponible sin GT.

---

## Caso B

```text
288 falla
289 falla
290 funciona
291 funciona
```

Conclusión:

> `R_pred(now)` es la causa principal.

---

## Caso C

```text
288 falla
289 falla
290 falla
291 funciona
```

Conclusión:

> tanto `R_pred(now)` como `omega_pred(now)` son insuficientes bajo delay.

La solución futura deberá reconstruir ambos conjuntamente.

---

## Caso D

```text
288 funciona
```

Conclusión:

> el error translacional p/v retardado era una parte crítica del fallo.

En ese caso analizar p/v antes de rediseñar el bloque angular.

---

## Caso E

```text
291 falla
```

Conclusión:

> el montaje diagnóstico o el pipeline/controlador tiene un problema adicional.

STOP.

---

# 13. Si se confirma `omega_pred(now)` como causa principal

NO implementar todavía la solución dentro de esta misma batería.

Documentar como siguiente propuesta:

```text
último estado visual válido en t_k
+
omega estimada en t_k
+
torque/comandos aplicados por el propio dron
+
inercia conocida
    ->
predecir omega(now)
```

Conceptualmente:

```text
omega_dot =
J^-1 (tau - omega x J omega)
```

y después:

```text
R_dot = R [omega]x
```

o convención equivalente correcta.

La arquitectura final no usaría GT.

---

# 14. Restricción sobre GT

GT en 288-291 es exclusivamente:

```text
truth/control de laboratorio
```

La solución final de Fase 5 debe seguir siendo:

```text
ORB
+
estimador local
+
información propia del dron
```

sin GT durante operación normal.

GT sólo permanecerá como fallback temporal de Fase 5 cuando ORB pierda tracking de verdad.

---

# 15. Builds y tests

Antes de ejecutar:

```text
build de paquetes afectados
GTests/CTest
analyzer tests
git diff --check
```

Añadir tests focales para asegurar:

```text
288:
sólo p/v son GT

289:
p/v/omega son GT; R sigue predicted

290:
p/v/R son GT; omega sigue predicted

291:
p/v/R/omega son GT
```

y confirmar que los valores GT usados corresponden a:

```text
t_control / now
```

no a:

```text
t_visual
```

---

# 16. Qué debe devolver Codex

Respuesta final:

```text
Resultado diagnóstico:
CONSEGUIDO / PARCIAL / NO CONSEGUIDO
```

Incluyendo:

```text
- archivos modificados;
- modos diagnósticos añadidos;
- confirmación de que no se modificó el predictor;
- resultado 288;
- resultado 289;
- resultado 290;
- resultado 291;

- tabla comparativa 284-291;
- ep/ev/er/ew;
- RMSE p/v/R/omega;
- lag;
- mismatch angular;
- energía;
- fallback/tracking;

- conclusión explícita:
    OMEGA_NOW PRINCIPAL
    R_NOW PRINCIPAL
    AMBAS ANGULARES
    P/V TAMBIÉN CRÍTICAS
    o
    SANITY CHECK INVALIDADO

- siguiente solución recomendada;
- mantener 279-281 detenidas.
```

Actualizar:

```text
historial_5H_RESUMEN.md
07_ULTIMA_SESION.md
```

sin borrar historial anterior.

---

# 17. Resumen ejecutivo

Las pruebas 285-287 señalan fuertemente a:

```text
omega_pred(now)
```

pero 287 no era un sanity check completo porque:

```text
p
v
```

seguían retardadas.

La batería final debe fijar:

```text
p(now) = GT
v(now) = GT
```

en todas las pruebas y variar únicamente:

```text
R
omega
```

Ejecutar:

```text
288: GT p/v + R_pred + omega_pred
289: GT p/v + R_pred + omega_GT
290: GT p/v + R_GT + omega_pred
291: GT p/v + R_GT + omega_GT
```

> Esta batería debe cerrar definitivamente el diagnóstico antes de diseñar la predicción dinámica final.
