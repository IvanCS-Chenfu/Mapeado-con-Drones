# Instrucciones para Codex — Fase 5H
## Siguiente iteración: `MODERATE_CONFIRMED -> anclaje visual completo`

## 0. Estado actual

Fase 5H sigue:

```text
PARCIAL
```

La arquitectura que se mantiene es:

```text
R_visual aceptada
    ->
R_base en t_visual
    ->
omega_motion sólo para propagación causal
    ->
R_control en el instante actual
```

Este principio se considera correcto y NO debe revertirse.

La última prueba ha mostrado una mejora causal clara al usar:

```text
SMALL -> anclaje visual completo
```

pero el hover ORB todavía no termina correctamente.

El siguiente problema identificado aparece cuando las medidas dejan de ser `SMALL` y pasan a moderadas.

---

# 1. Error actual

Cuando una corrección angular entra en:

```text
MODERATE_PENDING
```

se espera correctamente a tener más evidencia.

Después, si varias muestras consecutivas confirman que la corrección es coherente, el estado pasa a:

```text
MODERATE_CONFIRMED
```

Sin embargo, la política actual sigue aplicando una corrección demasiado limitada.

Patrón observado aproximadamente:

```text
error antes de corregir:
~0.055 rad

corrección aplicada:
~0.015 rad

error restante:
~0.040 rad
```

Por tanto, incluso después de confirmar que la medida visual es fiable, se conserva todavía gran parte del desfase.

Eso vuelve a permitir:

```text
R_base atrasada
    ->
er crece
    ->
tau_er actúa mal
    ->
oscilación angular
    ->
medidas más difíciles
    ->
rechazos
    ->
fallback
```

---

# 2. Hipótesis de esta iteración

La política actual de `MODERATE_CONFIRMED` contradice parcialmente la propia confirmación temporal.

Si una corrección ha superado:

```text
- tracking válido;
- continuidad válida;
- referencia coherente;
- dinámica raw plausible;
- consistencia temporal;
- número mínimo de frames de confirmación;
```

entonces:

```text
MODERATE_CONFIRMED
```

debe significar realmente:

> “esta orientación visual es suficientemente fiable como para volver a usarla como ancla absoluta del estado angular”.

No:

> “la medida es fiable, pero sólo corregimos una pequeña fracción y mantenemos casi todo el error”.

---

# 3. Cambio funcional principal

Modificar la política angular a:

```text
SMALL:
    anclaje visual completo

MODERATE_PENDING:
    no reanclar
    sólo predicción

MODERATE_CONFIRMED:
    anclaje visual completo

REJECTED_EXCESSIVE:
    no reanclar
```

Es decir:

```text
MODERATE_CONFIRMED
    ->
R_base(t_visual) = R_visual_accepted(t_visual)
```

o equivalente exacto según la convención SO(3) del código.

---

# 4. Qué significa “anclaje visual completo”

No significa introducir una velocidad física artificial.

Si antes de la medida:

```text
R_pred(t_visual)
```

y ORB confirma:

```text
R_visual(t_visual)
```

la diferencia:

```text
Log(R_visual * inverse(R_pred))
```

es:

```text
corrección del estimador
```

NO:

```text
movimiento físico que deba convertirse en omega
```

Por tanto:

```text
R_base <- R_visual
```

es una corrección de estado.

Después:

```text
omega_motion
```

se sigue usando exclusivamente para propagar desde:

```text
t_visual
```

hasta:

```text
t_publish / t_control
```

---

# 5. Qué NO hay que hacer

No volver a:

```text
corrección absoluta
    ->
omega_bias grande
    ->
integración prolongada
```

No volver a usar:

```text
omega_motion
```

como fuente absoluta de orientación a largo plazo.

No bajar thresholds raw por intuición.

No modificar:

```text
Kr
Kw
Kp
Kv
```

No cambiar:

```text
GT
mux
W
servidor
reference gate
ORB-SLAM3 core
extrínseca
```

No introducir hardcodes angulares.

No ejecutar etapa 3 si el hover vuelve a fallar.

---

# 6. Condiciones obligatorias para que `MODERATE_CONFIRMED` pueda reanclar

El anclaje completo sólo debe ejecutarse si siguen siendo válidas las condiciones que dieron lugar a la confirmación.

Como mínimo:

```text
tracking_valid == true
local_valid == true
continuity_valid == true

misma map_epoch
reference context coherente

raw motion no rechazado
probation confirmada

medida visual finita y válida
```

No convertir `MODERATE_CONFIRMED` en un bypass de los gates existentes.

---

# 7. `MODERATE_PENDING`

Mientras la corrección siga pendiente:

```text
NO tocar R_base con esa medida
```

Mantener:

```text
predict-only
```

durante un horizonte corto y limitado.

---

# 8. `MODERATE_CONFIRMED`

Una vez confirmada:

```text
R_base = R_visual_accepted
base_timestamp = t_visual
```

Después:

```text
Predict(t_target)
```

propaga una sola vez hasta el instante actual.

No aplicar primero una corrección parcial y después otro anclaje.

No aplicar:

```text
orientation_alpha * residual
```

si eso vuelve a dejar la mayor parte del error vivo.

---

# 9. `SMALL`

Mantener el comportamiento que ya ha mostrado mejora:

```text
SMALL + plausible
    ->
anclaje visual completo
```

Cada `SMALL_ANCHOR` debe seguir dejando aproximadamente:

```text
visual_base_error_after ~= 0
```

---

# 10. `REJECTED_EXCESSIVE`

Si la medida es rechazada:

```text
NO actualizar R_base
```

Mantener:

```text
predict-only
```

durante el horizonte permitido.

Después seguir usando:

```text
decay de omega_motion
degraded state
fallback
```

según la lógica existente.

---

# 11. Problema que se quiere eliminar

Actualmente puede ocurrir:

```text
MODERATE_CONFIRMED
    ->
error visual-base = 0.055 rad
    ->
corrijo sólo 0.015 rad
    ->
quedan 0.040 rad
    ->
sigo prediciendo desde una base todavía atrasada
```

La nueva política debe producir:

```text
MODERATE_CONFIRMED
    ->
error visual-base = 0.055 rad
    ->
reanclaje completo
    ->
error visual-base_after ~= 0
```

---

# 12. Riesgo esperado: jitter

El principal riesgo de esta modificación es el jitter si una medida moderada confirmada sigue teniendo algo de ruido.

No intentar resolver ese posible problema antes de verlo.

Primero probar:

```text
anclaje completo
```

Si el hover se estabiliza pero aparece jitter pequeño, después podrá estudiarse una fusión ligera con ganancia cercana a 1 y sin volver a introducir gran lag.

---

# 13. GTests obligatorios

Mantener todos los tests actuales y añadir como mínimo:

## Test A — MODERATE_PENDING no ancla

Esperado:

```text
R_base no cambia por la medida moderada pendiente
```

## Test B — MODERATE_CONFIRMED ancla completamente

Tras los frames de confirmación:

```text
R_base_after == R_visual_accepted
```

dentro de tolerancia.

Y:

```text
visual_base_error_after ~= 0
```

## Test C — no convertir anclaje en omega artificial

Después del anclaje:

```text
omega_motion
```

debe seguir representando el movimiento raw observado.

No debe aparecer una gran omega únicamente porque se corrigió `R_base`.

## Test D — extrapolación después de MODERATE_CONFIRMED

```text
R_base(t_visual)
+
omega_motion
+
age
```

debe producir la orientación correcta en `t_publish`.

## Test E — REJECTED no ancla

Esperado:

```text
R_base se mantiene
```

## Test F — SMALL sigue anclando

No degradar el comportamiento conseguido.

## Test G — reference switch

Todos los tests existentes de KF siguen pasando.

## Test H — una sola extrapolación

Conservar el resultado de la iteración anterior.

## Test I — bias/deadband/decay

Todos los tests anteriores siguen pasando.

---

# 14. Telemetría obligatoria

Para cada actualización angular registrar:

```text
classification

visual_timestamp
base_timestamp

R_visual
R_base_before
R_base_after

visual_base_error_before_rad
visual_base_error_after_rad

anchor_applied
anchor_type:
  SMALL_ANCHOR
  MODERATE_PENDING
  MODERATE_CONFIRMED_ANCHOR
  REJECTED
  PREDICT_ONLY

anchor_correction_rad

omega_raw
omega_motion
omega_bias
omega_total

prediction_horizon_sec

R_publish
R_control_used

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

# 15. Métricas especialmente importantes

Para `MODERATE_CONFIRMED` comprobar:

```text
visual_base_error_before
visual_base_error_after
```

Quiero ver:

```text
antes:
error moderado

después:
≈ 0
```

No:

```text
después:
queda la mayor parte del error
```

---

# 16. Nueva prueba

Repetir exactamente el mismo hover ORB.

Nombre sugerido:

```text
prueba 267
```

No cambiar misión.

No ejecutar etapa 3 todavía.

No modificar otro subsistema funcionalmente.

---

# 17. Comparación obligatoria con la prueba anterior

Preparar una comparación directa:

```text
                         anterior      267
------------------------------------------------
ORB govern time

SMALL anchors
MODERATE_PENDING
MODERATE_CONFIRMED

mean moderate error before
mean moderate error after

max visual_base_error

max |er|
max |ew|

tau_er positive-work ratio
tau_er net energy

tau_ew net energy
total angular energy

max |omega_GT|
max |omega_motion|

first rejected timestamp
fallback timestamp
tracking loss timestamp

hover success
```

---

# 18. Criterio de éxito inmediato

No considerar éxito porque:

```text
aguanta más segundos
```

La prueba sólo se considera funcionalmente `CONSEGUIDA` si:

```text
hover ORB completa
```

y además:

```text
no hay crecimiento angular tardío
no hay fallback provocado por el estimador
tracking permanece válido
tau_er no vuelve a inyectar energía de forma sostenida
```

---

# 19. Repetición obligatoria si la primera prueba funciona

Si la 267 completa el hover:

```text
NO pasar inmediatamente a etapa 3
```

Repetir el mismo hover al menos una vez más.

Nombre sugerido:

```text
prueba 268
```

Condición:

```text
267 = éxito
268 = éxito
```

Sólo entonces autorizar:

```text
etapa 3
```

---

# 20. Si la prueba 267 mejora pero sigue fallando

Si ocurre:

```text
MODERATE_CONFIRMED deja error casi cero
tau_er baja mucho
pero hover aún cae
```

entonces:

```text
NO revertir el anclaje completo
```

Eso demostraría que esta corrección era correcta y que queda otro mecanismo posterior.

Analizar qué señal empieza primero a crecer antes de hacer otro cambio.

---

# 21. Si el anclaje completo introduce jitter

Si:

```text
hover ya no diverge
pero aparece jitter
```

no volver a la corrección lenta de 0.015 rad.

Estudiar después una fusión ligera con ganancia alta y mínimo lag.

---

# 22. Si falla sin mejora

Si:

```text
MODERATE_CONFIRMED ancla a cero error
pero tau_er sigue creciendo igual
```

investigar entonces:

```text
- si R_visual está en el frame correcto;
- si R_des cambia por la parte translacional;
- si er está calculado correctamente;
- si roll/pitch/yaw tienen comportamientos distintos;
- si omega_motion y R_base usan convenciones distintas;
- si existe otro desfase posterior al anclaje.
```

No tocar gains antes de demostrarlo.

---

# 23. Objetivo funcional de Fase 5

La meta final sigue siendo:

```text
control normal:
ORB
    ->
pose y velocidad
    ->
trayectoria
    ->
control
    ->
motores
```

Sin GT.

GT sólo podrá utilizarse temporalmente en Fase 5 cuando ORB pierda tracking de verdad por motivos visuales legítimos, por ejemplo:

```text
poca textura
zona sin features
condiciones visuales realmente malas
```

No debe ser necesario GT porque el control ORB desestabiliza el dron.

El dron debe poder:

```text
hover
trasladarse
girar
seguir goals
seguir la trayectoria completa
```

sin GT mientras ORB siga localizado.

---

# 24. Qué debe devolver Codex

Respuesta final:

```text
Resultado:
CONSEGUIDO / PARCIAL / NO CONSEGUIDO
```

Incluir:

```text
- archivos modificados;
- política anterior de MODERATE_CONFIRMED;
- política nueva;
- condiciones exactas de anclaje;
- cómo se evita que la corrección genere omega falsa;
- GTests añadidos;
- total GTests;
- builds;
- prueba 267;
- prueba 268 si 267 funciona;
- duración ORB;
- errores visual-base antes/después;
- tau_er;
- tau_ew;
- energía total;
- fallback;
- tracking;
- hover success;
- decisión:
    repetir hover
    autorizar etapa 3
    o mantener STOP.
```

Actualizar:

```text
historial_5H_RESUMEN.md
07_ULTIMA_SESION.md
contrato 5H
```

sin borrar historial previo.

---

# 25. Resumen ejecutivo

El problema actual es:

```text
MODERATE_CONFIRMED
```

ya ha demostrado que la corrección visual es fiable, pero la política actual sólo corrige una pequeña parte y deja gran parte del desfase en `R_base`.

La siguiente política debe ser:

```text
SMALL
    -> anclaje completo

MODERATE_PENDING
    -> sólo predicción

MODERATE_CONFIRMED
    -> anclaje completo

REJECTED
    -> no anclar
```

La idea clave:

> **si una orientación visual ha superado los gates y la confirmación temporal, debe volver a ser la referencia absoluta del estado angular, no una sugerencia que sólo corrige una pequeña parte del error.**

El criterio siguiente no es “aguantar más”.

Es:

```text
hover ORB completo y repetible
```

antes de continuar con la trayectoria.
