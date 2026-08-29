# Instrucciones para Codex — Fase 5H
## Iteración posterior a prueba 267: validar gate raw corregido y, si falla, usar target visual confirmado con convergencia acotada

## 0. Estado actual

Fase 5H permanece:

```text
PARCIAL
```

Última prueba relevante:

```text
Prueba 267: NO CONSEGUIDA
```

Resultados principales:

```text
ORB gobernando: ~5.56 s
fallback: +5.58 s
tracking no válido: +6.12 s

tau_er: +0.030448 J
energía angular total: +0.015622 J
```

La prueba 267 aplicó:

```text
MODERATE_CONFIRMED -> anclaje visual completo
```

y los anclajes confirmados dejaron:

```text
visual_base_error_after = 0
```

Sin embargo, la inestabilidad apareció más rápido que en 266.

También se detectó un defecto experimental importante:

```text
uno de los MODERATE_CONFIRMED_ANCHOR
se aplicó con raw_class=REJECTED
```

Ese incumplimiento ya se corrigió en código:

```text
MODERATE_CONFIRMED sólo puede anclar
si raw_motion_plausible
```

y la versión final pasa:

```text
46/46 GTests
```

pero esa corrección todavía no se ha validado en simulación.

---

# 1. Conclusión que NO debe perderse

La prueba 266 demostró una mejora real al introducir:

```text
SMALL -> anclaje visual completo
```

En la ventana comparable:

```text
tau_er:
+0.153559 J -> +0.002067 J

trabajo angular total:
+0.138374 J -> -0.001945 J

max error angular:
0.666 rad -> 0.264 rad
```

Por tanto:

> El principio de reanclar `R_base` a la orientación visual fiable es correcto.

NO revertir:

```text
SMALL_ANCHOR
```

No volver a:

```text
pose_base integrada indefinidamente desde omega_motion
```

---

# 2. Qué ha enseñado la 267

La 267 demuestra que este extremo:

```text
MODERATE_CONFIRMED
    ->
anclaje completo instantáneo
```

puede ser demasiado brusco.

Se observaron correcciones confirmadas del orden de:

```text
~0.06 rad
~0.087 rad
```

y el controlador recibe inmediatamente una nueva `R_act`.

Aunque la medida visual sea válida, una corrección absoluta de varios grados aplicada en un solo instante puede generar:

```text
salto de er
    ->
tau_er fuerte
    ->
movimiento físico
    ->
nueva realimentación
```

La 267 acumuló energía inestable más rápido que la 266.

Por tanto, tenemos dos extremos demostrados:

```text
266:
corregir poco una sola vez
-> deja demasiado residual

267:
corregir todo instantáneamente
-> demasiado brusco
```

La siguiente solución debe estar entre ambos.

---

# 3. Objetivo de esta iteración

La iteración debe hacerse en DOS pasos posibles.

## PASO A — validar primero la versión ya corregida

Antes de modificar otra vez la arquitectura:

```text
repetir el mismo hover
```

con el código final donde:

```text
MODERATE_CONFIRMED
sólo puede anclar si raw_motion_plausible
```

Nombre sugerido:

```text
prueba 268
```

No ejecutar etapa 3.

No cambiar nada funcional además del gate raw ya corregido.

---

# 4. Resultado esperado de prueba 268

Si la 268 completa el hover:

```text
NO dar por cerrada Fase 5H todavía
```

Repetir el mismo hover una vez más:

```text
prueba 269
```

Si:

```text
268 = éxito
269 = éxito
```

entonces se puede autorizar:

```text
etapa 3
```

Si la 268 falla, NO seguir probando el mismo anclaje completo.

Pasar al PASO B.

---

# 5. PASO B — target visual confirmado persistente

Si la 268 falla, sustituir:

```text
MODERATE_CONFIRMED -> anclaje completo inmediato
```

por:

```text
MODERATE_CONFIRMED
    ->
crear/actualizar un TARGET VISUAL CONFIRMADO
```

Ese target debe permanecer activo hasta que:

```text
R_base converja suficientemente cerca de él
```

o hasta que:

```text
el target deje de ser válido
```

---

# 6. Nueva política angular deseada

La política debe quedar aproximadamente así:

```text
SMALL:
    anclaje visual completo inmediato

MODERATE_PENDING:
    no crear target nuevo
    no anclar
    sólo predicción

MODERATE_CONFIRMED + raw plausible:
    guardar R_visual como confirmed_target
    NO saltar instantáneamente a ella

TARGET ACTIVO:
    converger progresivamente hacia confirmed_target
    a 50 Hz
    con límite angular por ciclo / velocidad de corrección

REJECTED:
    no crear target nuevo
    cancelar o invalidar target si corresponde
    no reanclar
```

---

# 7. Idea central del target confirmado

Ejemplo:

```text
error confirmado:
0.060 rad
```

NO hacer:

```text
266:
corrijo 0.015 una vez
-> quedan 0.045 para siempre
```

NO hacer:

```text
267:
corrijo 0.060 de golpe
-> salto brusco
```

Hacer:

```text
target = orientación visual confirmada

tick 1:
corrijo 0.015
restan 0.045

tick 2:
corrijo 0.015
restan 0.030

tick 3:
corrijo 0.015
restan 0.015

tick 4:
corrijo 0.015
error ~0
```

Los `0.015 rad` son sólo ejemplo.

La implementación debe usar un parámetro configurable.

---

# 8. Diferencia fundamental respecto a la prueba 266

En 266, la corrección parcial se olvidaba.

Es decir:

```text
MODERATE_CONFIRMED
    ->
corrección limitada una vez
    ->
siguiente frame puede ser PREDICT_ONLY
    ->
residual queda vivo
```

Ahora quiero:

```text
MODERATE_CONFIRMED
    ->
confirmed_target queda latched
    ->
se sigue corrigiendo en cada tick
    ->
hasta converger
```

No perder el objetivo confirmado sólo porque la siguiente medida sea:

```text
MODERATE_PENDING
```

---

# 9. Separar tres conceptos

La arquitectura debe distinguir explícitamente:

```text
1. ¿La medida es fiable?
   -> probation / clasificación

2. ¿A qué orientación debe converger el estimador?
   -> confirmed_target

3. ¿A qué velocidad puede corregirse sin excitar el controlador?
   -> correction_rate / correction_step_limit
```

No mezclar los tres conceptos en:

```text
orientation_alpha
```

o en una única corrección instantánea.

---

# 10. Estados sugeridos para el target

No es obligatorio usar exactamente estos nombres, pero sí esta semántica:

```text
NO_TARGET
TARGET_ACTIVE
TARGET_REACHED
TARGET_INVALIDATED
```

Guardar:

```text
target_orientation
target_visual_timestamp
target_created_timestamp
target_reference_kf
target_map_epoch
target_source_class
target_initial_error_rad
target_remaining_error_rad
```

---

# 11. Cuándo crear target

Crear target sólo si:

```text
classification == MODERATE_CONFIRMED
tracking_valid == true
local_valid == true
continuity_valid == true
raw_motion_plausible == true
misma map_epoch
contexto KF coherente
medida finita
```

No crear target si:

```text
raw_class = REJECTED
```

Este punto ya se corrigió después de 267 y debe permanecer.

---

# 12. Cuándo actualizar target

Si llega otro:

```text
MODERATE_CONFIRMED + raw plausible
```

mientras existe un target:

Codex debe implementar una política coherente.

Preferencia:

```text
actualizar target a la orientación confirmada más reciente
```

si:

```text
- misma epoch;
- referencia coherente;
- no hay contradicción grande con target anterior;
- sigue siendo raw plausible.
```

Registrar explícitamente:

```text
TARGET_REFRESH
```

No acumular varios targets antiguos.

---

# 13. Qué hacer con SMALL mientras hay target

Si durante un target activo llega una medida:

```text
SMALL + plausible
```

esa medida tiene prioridad como observación fiable normal.

Por tanto:

```text
SMALL
    ->
anclaje completo a R_visual
    ->
target se considera alcanzado/cancelado
```

porque la nueva medida visual fiable ya proporciona una ancla mejor y más reciente.

---

# 14. Qué hacer con MODERATE_PENDING mientras hay target

Si el target ya fue confirmado anteriormente y llega:

```text
MODERATE_PENDING
```

NO cancelar automáticamente el target.

Continuar convergiendo hacia el target previamente confirmado mientras:

```text
tracking siga válido
raw no sea rechazado de forma incompatible
no cambie epoch
no haya invalidación de referencia
```

Ésta es una diferencia esencial respecto a la 266.

---

# 15. Qué hacer con REJECTED mientras hay target

Si aparece:

```text
REJECTED_EXCESSIVE
```

no seguir corrigiendo ciegamente hacia un target antiguo.

Política recomendada:

```text
TARGET_INVALIDATED
```

o:

```text
suspender corrección del target
```

y pasar a:

```text
predict-only
+
decay de omega_motion
```

según lógica actual.

No mantener una corrección absoluta antigua mientras la observación visual actual está siendo rechazada.

---

# 16. Velocidad de convergencia

La convergencia debe ser:

```text
completa
pero distribuida en el tiempo
```

No usar un alpha fijo sin contexto.

Preferir uno de estos mecanismos:

```text
max_target_correction_rad_per_tick
```

o:

```text
max_target_correction_rate_radps
```

con aplicación a 50 Hz.

Ejemplo conceptual:

```text
max_target_correction_rate = X rad/s
```

entonces:

```text
max_step = X * dt
```

No hardcodear números sin parámetro.

---

# 17. Posible límite de aceleración de corrección

Si una corrección de target empieza con una velocidad de corrección alta, puede volver a excitar control.

Opcionalmente introducir:

```text
max_target_correction_acceleration_radps2
```

para que la corrección absoluta tenga una entrada/salida suave.

No es obligatorio si el step-limit ya resulta suficientemente suave.

---

# 18. Muy importante — no convertir target correction en omega física

La corrección hacia el target es:

```text
corrección de estimación
```

NO:

```text
movimiento real observado
```

Por tanto, distinguir internamente:

```text
omega_motion
target_correction_step
```

No sumar necesariamente:

```text
target correction
```

a `omega_motion` como si fuera velocidad física.

La pose puede corregirse como estado estimado.

La `omega` publicada debe seguir representando:

```text
movimiento físico estimado
```

no la velocidad artificial necesaria para cerrar el target.

Codex debe revisar cuidadosamente esta semántica.

---

# 19. Pero la pose publicada tampoco puede pegar saltos

Aunque la target correction no sea omega física, la orientación publicada al controlador debe cambiar suavemente.

Por eso:

```text
target correction
```

debe estar limitada por ciclo.

La idea es:

```text
corrección de estado gradual
```

no:

```text
salto instantáneo
```

---

# 20. Interacción con predictor temporal

Mantener:

```text
una sola extrapolación temporal
```

como quedó validado en 265.

Secuencia deseada:

```text
R_base en t_visual / base timestamp
    ->
aplicar corrección progresiva de target a la base
    ->
propagar una sola vez con omega_motion
    ->
R_publish
```

Codex debe documentar exactamente dónde se aplica la corrección para evitar doble propagación.

---

# 21. GTests obligatorios del PASO B

Mantener:

```text
46/46
```

y añadir al menos:

## Test A — confirmed crea target, no salto

```text
error confirmado = 0.06 rad
```

Esperado:

```text
target creado
R_base NO salta 0.06 rad de golpe
```

---

## Test B — target converge completamente

Con varios ticks:

```text
remaining_error:
0.06
0.045
0.030
0.015
0
```

o equivalente según parámetro.

Esperado:

```text
target finalmente alcanzado
```

---

## Test C — PENDING no cancela target confirmado

Secuencia:

```text
CONFIRMED
PENDING
PENDING
```

Esperado:

```text
target sigue activo
convergencia continúa
```

---

## Test D — SMALL cancela target y ancla

```text
target activo
+
SMALL plausible
```

Esperado:

```text
R_base = R_visual
target cleared/reached
```

---

## Test E — REJECTED invalida target

Esperado:

```text
no continuar corrección absoluta antigua
```

---

## Test F — raw rejected nunca crea target

Cubrir explícitamente el bug encontrado en 267.

---

## Test G — target refresh

Dos MODERATE_CONFIRMED coherentes.

Esperado:

```text
target se actualiza correctamente
```

---

## Test H — target incompatible

Nueva confirmación incoherente / distinta epoch.

Esperado:

```text
no fusionar targets incompatibles
```

---

## Test I — no omega artificial

La convergencia de pose hacia target no debe producir:

```text
omega_motion espuria
```

---

## Test J — una sola extrapolación

Mantener tests previos.

---

# 22. Telemetría obligatoria

Añadir/registrar:

```text
classification
raw_motion_class

target_state
target_created
target_refreshed
target_invalidated
target_reached

target_initial_error_rad
target_remaining_error_before_rad
target_correction_step_rad
target_remaining_error_after_rad

target_visual_timestamp
target_age_sec

R_visual
R_base_before
R_base_after
R_target

omega_raw
omega_motion
omega_bias
omega_total

prediction_horizon_sec

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

# 23. Pruebas de simulación a ejecutar

## Prueba 268 — versión corregida de 267

Primero:

```text
mismo hover
misma arquitectura de anclaje completo
pero con gate raw ya corregido
```

Objetivo:

```text
validar que el fallo de 267 no fue principalmente
el confirmed anchor aplicado con raw rejected
```

---

## Si 268 funciona

Ejecutar:

```text
prueba 269
```

misma configuración.

Si ambas funcionan:

```text
autorizar etapa 3
```

---

## Si 268 falla

Implementar target confirmado persistente.

Después ejecutar:

```text
prueba 269_target
```

o numeración consecutiva coherente con el historial.

---

# 24. Si la primera prueba con target funciona

No avanzar inmediatamente.

Repetir el hover una segunda vez.

Condición:

```text
dos hovers consecutivos completos
```

Sólo entonces:

```text
autorizar etapa 3
```

---

# 25. Criterio de éxito

No usar:

```text
“aguantó más segundos”
```

como éxito.

Éxito funcional sólo si:

```text
hover ORB completo
sin fallback causado por estimador
sin crecimiento tardío de er
sin crecimiento tardío de tau_er
tracking válido
```

Y, para la arquitectura target:

```text
los targets confirmados convergen a cero error
sin producir saltos bruscos
```

---

# 26. Métricas comparativas obligatorias

Comparar:

```text
266
267
268
target-version
```

con:

```text
ORB govern time

number SMALL anchors
number MODERATE_PENDING
number MODERATE_CONFIRMED

confirmed raw plausible count
confirmed raw rejected count

mean confirmed error before
mean confirmed error after

target count
target reached count
target invalidated count

mean convergence time
max correction step

max |er|
max |ew|

tau_er net energy
tau_ew net energy
total angular energy

first unstable-energy timestamp

first rejected timestamp
fallback timestamp
tracking loss timestamp

hover success
```

---

# 27. Interpretación de resultados

## Caso A — 268 funciona

Conclusión:

```text
el bug raw-rejected era decisivo
```

Repetir hover.

---

## Caso B — 268 falla igual que 267

Conclusión:

```text
el anclaje completo inmediato sigue siendo demasiado brusco
```

Implementar target persistente.

---

## Caso C — target mejora mucho pero aún falla

NO revertir target.

Analizar:

```text
qué señal crece después
```

Puede quedar:

```text
latencia residual
omega_motion
R_des
error por eje
```

pero no volver a thresholds arbitrarios.

---

## Caso D — target estabiliza hover

Repetir hover.

Después:

```text
etapa 3
```

---

# 28. Restricciones arquitectónicas

Mantener:

```text
O = frame continuo de control
W = frame global corregible
estimador local al dron
servidor fuera del lazo rápido
```

No usar:

```text
GT para corregir pose
GT para decidir target
W para corregir orientación local
```

GT sólo diagnóstico/fallback temporal de Fase 5.

---

# 29. Objetivo final de Fase 5

El sistema normal debe ser:

```text
ORB
    ->
estado local O
    ->
trayectoria
    ->
controlador
    ->
motores
```

Sin GT.

GT_FALLBACK sólo cuando:

```text
ORB realmente pierde tracking
```

por razones visuales legítimas.

No debe entrar fallback porque:

```text
el control ORB se vuelve inestable
```

---

# 30. Qué debe devolver Codex

Tras ejecutar esta iteración:

```text
Resultado:
CONSEGUIDO / PARCIAL / NO CONSEGUIDO
```

y explicar:

```text
- si se ejecutó 268 antes de tocar arquitectura;
- resultado 268;
- si fue necesario implementar target persistente;
- archivos modificados;
- lógica final de SMALL/PENDING/CONFIRMED/REJECTED;
- cómo se crea, actualiza e invalida target;
- límite de corrección usado;
- cómo se evita salto;
- cómo se evita omega artificial;
- GTests añadidos;
- total GTests;
- builds;
- pruebas ejecutadas;
- energía tau_er/tau_ew;
- evolución de er/ew;
- target convergence;
- fallback/tracking;
- si hubo dos hovers completos;
- decisión:
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

# 31. Resumen ejecutivo

La evidencia actual dice:

```text
SMALL anchor completo:
correcto

MODERATE_CONFIRMED corrección única pequeña:
demasiado lenta

MODERATE_CONFIRMED anchor completo:
demasiado brusco
```

Por tanto, la siguiente política si 268 vuelve a fallar debe ser:

```text
MODERATE_CONFIRMED
    ->
target visual confirmado persistente
    ->
convergencia completa
    ->
pero repartida en varios ticks
```

La idea clave es:

> **no corregir poco y olvidar el error; no corregir todo de golpe. Confirmar una orientación, conservarla como objetivo y absorber toda la corrección gradualmente hasta converger.**

La meta sigue siendo:

```text
hover ORB completo y repetible
```

antes de seguir con la trayectoria.
