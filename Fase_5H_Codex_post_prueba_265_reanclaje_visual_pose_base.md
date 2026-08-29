# Instrucciones para Codex — Fase 5H tras prueba 265
## Reanclar `pose_base` a la orientación visual ORB aceptada y usar `omega_motion` sólo para propagación causal

## 0. Estado actual

Estado de Fase 5H:

```text
PARCIAL
```

Última iteración:

```text
Prueba 265:
  corrección temporal técnicamente correcta
  horizonte medio ≈ 43.2 ms
  una sola extrapolación
  builds correctos
  40/40 GTests
  5/5 tests del analizador

Resultado funcional:
  hover NO CONSEGUIDO
  duración ORB ≈ 5.56 s
  tau_er inyecta energía ≈ +0.160266 J
  etapa 3 NO ejecutada
```

Conclusión actual:

> La extrapolación temporal final funciona, pero la orientación base que se extrapola, `pose_base`, ya está fuera de fase porque se construye principalmente mediante integración acumulativa de `omega_motion`.

Por tanto, la siguiente iteración debe modificar **cómo se construye `pose_base`**.

No seguir intentando corregir únicamente:

```text
timer prediction
latencia de publicación
omega_bias
decay
thresholds raw
```

---

# 1. Hipótesis principal

La arquitectura angular actual puede resumirse aproximadamente como:

```text
R_raw visual
      |
      +----> DeltaR_raw
                |
                v
           omega_motion
                |
                v
          integración
                |
                v
            pose_base
                |
                v
     extrapolación temporal
                |
                v
             R_act
```

La prueba 265 indica que:

```text
la extrapolación final es correcta
```

pero:

```text
pose_base ya llega desfasada
```

Por tanto:

```text
estado base fuera de fase
+
predicción temporal correcta
=
estado final todavía fuera de fase
```

La sospecha principal es:

> Integrar `omega_motion` durante muchos frames hace que pequeños errores de velocidad/fase acumulen deriva angular y desfase en `pose_base`.

---

# 2. Nuevo principio de diseño

La orientación visual ORB aceptada debe volver a ser la **ancla absoluta del estado angular**.

`omega_motion` debe utilizarse para:

```text
- representar movimiento entre observaciones;
- propagar desde t_visual hasta t_actual;
```

pero NO debe ser la única fuente que mantenga indefinidamente la orientación absoluta mediante integración.

La arquitectura deseada es:

```text
            ORB
             |
             v
   orientación visual raw
       en t_visual
             |
       gates existentes
             |
             v
      MEDIDA ACEPTADA
             |
             v
     R_base(t_visual)
             |
             | omega_motion
             v
   propagación causal única
             |
             v
      R_control(t_now)
             |
             v
         controlador
```

---

# 3. Regla principal

Para cada orientación visual ORB **fiable y aceptada**:

```text
R_base(t_visual)
```

debe reanclarse a:

```text
R_visual_accepted(t_visual)
```

o a una fusión muy cercana a esa medida si es necesario.

No mantener como verdad absoluta:

```text
R_base =
integral acumulada de omega_motion
```

durante largos periodos.

---

# 4. Distinción conceptual obligatoria

Separar claramente:

```text
A) corrección del estado estimado
B) movimiento físico
```

Si:

```text
R_pred(t_visual) != R_visual(t_visual)
```

esa diferencia es:

```text
error del estimador
```

No implica que el dron haya ejecutado físicamente ese giro durante el último frame.

Por tanto:

```text
corrección de pose
!=
omega física
```

Esta distinción debe quedar explícita en código y documentación.

---

# 5. Qué papel conserva `omega_motion`

`omega_motion` sigue siendo útil y necesaria.

Debe representar:

```text
movimiento angular observado entre medidas ORB consecutivas
```

y servir para:

```text
1. describir dinámica física reciente;
2. extrapolar desde t_visual a t_publish/t_control;
3. alimentar la velocidad angular publicada;
```

No usar `omega_motion` para mantener indefinidamente la orientación absoluta sin nuevas correcciones visuales.

---

# 6. Primera implementación que quiero probar

No implementar todavía un EKF ni una fusión compleja.

Hacer primero la versión más simple que permita demostrar la hipótesis:

```text
si medida visual angular es aceptada:
    R_base(t_visual) = R_visual_accepted(t_visual)

después:
    R_now = Propagate(R_base, omega_motion, t_now - t_visual)
```

Mantener:

```text
una sola extrapolación temporal
```

No añadir otra capa de predicción.

---

# 7. Qué medidas pueden reanclar directamente

No todas las medidas ORB deben tratarse igual.

## 7.1. Medida pequeña y plausible

Si:

```text
tracking válido
continuidad válida
raw motion plausible
clasificación angular SMALL
sin rechazo de referencia
```

entonces:

```text
R_base <- R_visual
```

en el timestamp visual de esa muestra.

Éste debe ser el caso normal.

---

## 7.2. Medida moderada

Si la medida entra en:

```text
MODERATE_PENDING
```

NO reanclar todavía.

Mantener:

```text
R_base anterior
+
propagación breve
```

hasta que la probation decida.

Si pasa a:

```text
MODERATE_CONFIRMED
```

entonces se permite actualizar `R_base`.

Pero la política exacta debe evitar introducir un salto enorme.

Dos opciones válidas:

```text
A) reanclaje completo si la corrección confirmada es segura;

B) corrección acotada sobre R_base.
```

Codex debe justificar cuál utiliza.

---

## 7.3. Medida excesiva/rechazada

Si:

```text
REJECTED_EXCESSIVE
```

entonces:

```text
NO tocar R_base
```

Mantener la predicción actual sólo durante el horizonte permitido.

Después aplicar las políticas existentes:

```text
decay
degraded
invalidación/fallback
```

según corresponda.

---

# 8. Cambios de KF

No rediseñar la lógica de reference KF.

La orientación visual que llega a esta nueva lógica debe ser ya la salida coherente de:

```text
NavigationStateEstimator
```

tras:

```text
reference gate
continuidad O
Tcr
KF activo
```

La nueva lógica angular debe trabajar sobre:

```text
O_T_B visual aceptada
```

No sobre poses crudas inconsistentes entre referencias.

---

# 9. No volver a `omega_bias` como mecanismo principal de corrección

No implementar:

```text
error absoluto
    ->
omega_bias grande
    ->
integración prolongada
```

como solución principal.

El problema actual justamente parece venir de convertir errores de estimación en dinámica acumulada.

`omega_bias` puede mantenerse para correcciones lentas muy pequeñas si ya está en la arquitectura, pero:

```text
no debe ser el mecanismo principal de alineación angular
```

La alineación principal pasa a ser:

```text
R_visual aceptada -> R_base
```

---

# 10. Coherencia temporal

Cada `R_base` debe estar asociada explícitamente a:

```text
base_timestamp
```

que debe representar:

```text
t_visual de la observación aceptada
```

No:

```text
t_receive
t_publish
```

salvo que realmente coincidan.

Después:

```text
Predict(t_target)
```

debe propagar exactamente desde:

```text
base_timestamp
```

hasta:

```text
t_target
```

---

# 11. Evitar doble predicción

Mantener el resultado conseguido en 265:

```text
una sola extrapolación
```

Codex debe documentar:

```text
R_base vive en t_visual
omega_motion vive asociado a qué intervalo
Predict() propaga desde base_timestamp
timer publica una única predicción
```

No volver a introducir:

```text
UpdateMeasurement propaga a receive
+
Predict propaga a publish
+
otra compensación externa
```

---

# 12. Problema que se quiere evitar

Ejemplo:

```text
R_visual:
0°
1°
2°
3°
4°

omega_motion ligeramente sesgada
```

Con integración acumulativa puede ocurrir:

```text
R_base:
0°
0.8°
1.6°
2.4°
3.2°
```

El error aumenta y genera desfase.

Con reanclaje visual:

```text
cada observación SMALL/plausible:
R_base =
0°
1°
2°
3°
4°
```

y `omega_motion` sólo cubre:

```text
entre observación y presente
```

No debe existir deriva angular acumulativa de largo plazo.

---

# 13. Qué hacer si el anclaje directo introduce jitter

NO complicar la primera implementación antes de medir.

Primero probar:

```text
R_base = R_visual_accepted
```

para SMALL/plausible.

Si esto elimina la oscilación pero introduce jitter pequeño, en una iteración posterior se puede estudiar una fusión ligera:

```text
e =
Log(R_visual * R_pred^-1)

R_base =
Exp(k * e) * R_pred
```

con:

```text
k cercano a 1
```

y sólo con el objetivo de reducir ruido.

No usar un `k` pequeño que vuelva a introducir gran lag.

---

# 14. GTests obligatorios

Mantener todos los:

```text
40/40 GTests
```

actuales.

Añadir como mínimo:

## Test A — reanclaje SMALL

Secuencia visual:

```text
0°
1°
2°
3°
4°
```

con `omega_motion` ligeramente sesgada.

Esperado:

```text
R_base sigue cada medida aceptada
no acumula deriva
```

---

## Test B — error acumulativo del integrador

Construir un caso donde:

```text
integración pura divergiría
```

Esperado con nuevo diseño:

```text
reanclajes visuales eliminan deriva acumulada
```

---

## Test C — extrapolación después de ancla

```text
R_base en t_visual
omega_motion constante
t_publish > t_visual
```

Esperado:

```text
R_publish correcta
```

---

## Test D — no doble extrapolación

Verificar otra vez:

```text
una única propagación temporal
```

---

## Test E — MODERATE_PENDING no reancla

Esperado:

```text
R_base se mantiene
```

---

## Test F — MODERATE_CONFIRMED

Esperado:

```text
R_base se actualiza según política definida
```

sin salto no acotado.

---

## Test G — REJECTED_EXCESSIVE

Esperado:

```text
R_base no cambia
```

---

## Test H — reference switch

Todos los tests existentes de KF siguen pasando.

---

## Test I — bias/deadband/decay

Todos los tests de 261/262 siguen pasando.

---

## Test J — coherencia timestamp

Cada actualización de `R_base` conserva:

```text
base_timestamp = t_visual
```

---

# 15. Telemetría obligatoria

Registrar:

```text
visual_timestamp
base_timestamp
publish_timestamp
control_timestamp

R_visual
R_pred_before_measurement
R_base_after_measurement
R_publish
R_control_used

base_update_applied
base_update_type:
  SMALL_ANCHOR
  MODERATE_PENDING
  MODERATE_CONFIRMED
  REJECTED
  PREDICT_ONLY

base_rotation_correction_rad

omega_raw
omega_motion
omega_bias
omega_total

prediction_horizon_sec
prediction_clamped

er
ew

tau_er
tau_ew
tau_total

omega_GT_body

P_er_GT
P_ew_GT
P_total_GT
```

---

# 16. Métrica nueva importante

Medir:

```text
visual_base_error =
Log(R_visual * R_base_predicted^-1)
```

antes del reanclaje.

Y después:

```text
visual_base_error_after
```

Para SMALL/plausible debería observarse:

```text
error después del update ≈ 0
```

salvo tolerancias numéricas.

Esto permitirá confirmar que `pose_base` no se queda acumulativamente atrás.

---

# 17. Prueba siguiente

Repetir exactamente:

```text
f5h_etapa_2_hover_orb.yaml
```

Nombre sugerido:

```text
prueba 266
```

No ejecutar etapa 3 todavía.

No modificar otros subsistemas en la misma prueba.

---

# 18. Comparación obligatoria 265 vs 266

Preparar:

```text
                         265          266
------------------------------------------------
ORB govern time

mean visual-base error
max visual-base error

mean prediction horizon

max |er|
max |ew|

tau_er positive-work ratio
tau_er net energy

tau_ew net energy
total angular work

max |omega_GT|
max |omega_motion|

fallback
tracking loss
hover success
```

---

# 19. Qué debe mejorar

La evidencia principal de éxito debe ser:

```text
pose_base deja de acumular desfase
```

seguida de:

```text
er disminuye
tau_er deja de inyectar energía sistemáticamente
amplitud angular deja de crecer
```

En 265:

```text
tau_er ≈ +0.160266 J
hover ≈ 5.56 s
```

En 266 quiero ver:

```text
reducción muy clara de energía positiva de tau_er
```

y, sobre todo:

```text
hover completo bajo ORB
```

---

# 20. Criterio de éxito

La iteración se considera `CONSEGUIDA` si:

```text
1. builds correctos;
2. todos los GTests pasan;
3. R_base se reancla correctamente;
4. no hay deriva angular acumulativa;
5. una sola extrapolación temporal;
6. pose y omega mantienen semántica temporal clara;
7. tau_er deja de alimentar sistemáticamente la oscilación;
8. hover ORB completa;
9. no hay fallback provocado por inestabilidad;
10. tracking se mantiene válido durante el hover.
```

---

# 21. Si 266 funciona

Si se cumple:

```text
hover ORB = CONSEGUIDO
```

entonces:

```text
se puede autorizar etapa 3
```

manteniendo toda la instrumentación temporal y energética.

---

# 22. Si 266 falla pero mejora mucho

Si:

```text
tau_er baja drásticamente
pero sigue habiendo jitter/oscilación residual
```

NO volver atrás.

Eso demostraría que:

```text
el anclaje visual era correcto
```

y el siguiente paso sería ajustar una fusión ligera entre:

```text
R_visual
R_pred
```

con el mínimo lag posible.

---

# 23. Si 266 falla sin mejorar

Entonces revisar:

```text
1. si R_visual O_T_B está realmente alineada con GT;
2. si el reanclaje se hace en el frame correcto;
3. si left/right SO3 multiplication es correcta;
4. si omega_motion está expresada en el frame esperado;
5. si R_des varía por translación;
6. si el error está concentrado en roll/pitch/yaw;
7. si hay un bug en el cálculo de er.
```

No tocar gains antes de demostrarlo.

---

# 24. Restricciones arquitectónicas

Mantener:

```text
O continuo de control
W global corregible
estimador local al dron
servidor fuera del lazo rápido
```

No usar:

```text
GT como corrección
servidor como fuente de actitud
W para corregir R_base
```

---

# 25. Documentación

Actualizar:

```text
historial_5H_RESUMEN.md
07_ULTIMA_SESION.md
contrato 5H
```

Documentar explícitamente:

```text
R_visual:
  observación absoluta visual aceptada

R_base:
  última orientación absoluta aceptada en t_visual

omega_motion:
  movimiento angular entre observaciones

Predict():
  única propagación desde R_base(t_visual) hasta t_target
```

---

# 26. Qué debe devolver Codex

Respuesta final:

```text
Resultado:
CONSEGUIDO / PARCIAL / NO CONSEGUIDO
```

Incluir:

```text
- archivos modificados;
- arquitectura angular anterior;
- arquitectura angular nueva;
- dónde se actualiza R_base;
- condiciones exactas para SMALL/MODERATE/REJECTED;
- semántica temporal de R_base;
- cómo se evita doble predicción;
- GTests añadidos;
- total GTests;
- builds;
- prueba 266;
- duración ORB;
- visual-base error;
- tau_er positive-work ratio;
- tau_er net energy;
- tau_ew net energy;
- fallback;
- tracking;
- hover success;
- decisión:
    autorizar etapa 3
    o
    mantener STOP.
```

---

# 27. Resumen ejecutivo

La prueba 265 demuestra que la compensación temporal final funciona, pero no resuelve el problema porque:

```text
pose_base ya está fuera de fase
```

La siguiente modificación debe cambiar la fuente de verdad angular:

```text
ANTES:
omega_motion
-> integración acumulativa
-> pose_base
-> extrapolación

AHORA:
R_visual aceptada
-> R_base en t_visual
-> omega_motion sólo para propagación
-> R_control actual
```

La idea clave es:

> **la orientación absoluta fiable de ORB debe corregir directamente el estado estimado; esa corrección no debe transformarse artificialmente en una velocidad física.**

La meta inmediata sigue siendo:

```text
hover ORB completamente estable
```

antes de ejecutar la etapa 3.
