# Fase 5H — Batería cruzada para aislar si falla `R(now)`, `omega(now)` o ambos
## Objetivo: decidir qué parte del estado retardado debe reconstruirse antes de diseñar la solución final

## 0. Estado actual

Trabajar sobre el estado ACTUAL del repositorio:

```text
https://github.com/IvanCS-Chenfu/Mapeado-con-Drones
```

La secuencia de pruebas ya ha demostrado:

```text
276/277:
GT perfecto ~20 Hz
+ nuevo estimador causal de omega_motion
+ sin delay
-> hover estable y reproducible

278:
mismo estimador
+ ~80 ms delay
-> falla

282:
ampliar max_extrapolation_sec 0.10 -> 0.18 s
-> elimina clamp
-> empeora

284:
propagar pose y omega con alpha_hat constante
-> mejora algunas métricas
-> pero sigue fallando antes
```

Resultado vigente:

```text
Diagnóstico: CONSEGUIDO
Solución funcional: NO CONSEGUIDA
Fase 5H: PARCIAL
```

Conclusión actual:

> El clamp no explica el fallo y una extrapolación puramente visual con aceleración angular constante tampoco es suficiente para reconstruir correctamente el estado actual bajo ~80 ms de delay.

Mantener detenidas:

```text
279
280
281
```

No conectar ORB real todavía.

---

# 1. Objetivo de esta nueva batería

Responder de forma casi binaria:

```text
¿falla principalmente R(now)?
¿falla principalmente omega(now)?
¿fallan ambas?
```

La prueba 284 mezcla ambos errores porque:

```text
R_pred(now)
omega_pred(now)
```

se obtienen del mismo predictor retardado.

Ahora hay que cruzar cada componente con GT actual exacta para aislar causalidad.

GT se utilizará ÚNICAMENTE como truth de laboratorio.

No es una propuesta para la arquitectura final.

---

# 2. Mantener fijo todo lo demás

No modificar:

```text
control gains
trayectoria
mux
SMALL/MODERATE
raw gates
bias/deadband
KF/reference
W
ORB-SLAM3 core
nuevo estimador causal
delay artificial
publicación 50 Hz
escenario de hover
```

Mantener:

```text
delay ~80 ms
pose visual de entrada ~20 Hz
mismo escenario
mismas condiciones
```

No implementar todavía:

```text
modelo dinámico con torque
EKF
Delta_target
nuevo filtro
nuevo predictor
```

Primero ejecutar estas pruebas cruzadas.

---

# 3. Referencia — prueba 284

Tomar 284 como baseline:

```text
R_control = R_pred(now)
omega_control = omega_pred(now)
```

Resultado:

```text
NO CONSEGUIDA
RMSE angular ~1.136 rad/s
energía total positiva ~+0.02052 J
gobierno ~1.86 s
primer raw reject ~0.84 s
```

No hace falta repetir 284 salvo que Codex detecte que la instrumentación no permite comparar de forma limpia.

---

# 4. Prueba 285 — `R_pred(now)` + `omega_GT(now)`

## Configuración

Mantener exactamente la orientación predicha de 284:

```text
R_control = R_pred(now)
```

pero sustituir únicamente la velocidad angular entregada al controlador por:

```text
omega_control = omega_GT(now)
```

sin delay.

GT debe sincronizarse al instante real del control.

NO usar:

```text
omega_GT(t_visual)
```

Queremos específicamente:

```text
omega_GT(t_control)
```

## Pregunta

> Si la omega actual es perfecta pero la orientación sigue siendo la predicha desde una medida retardada, ¿el hover se estabiliza?

## Interpretación

Si 285 funciona:

```text
omega(now) es el principal problema residual
```

y la orientación predicha actual es suficientemente buena para control.

Si 285 falla:

```text
R(now) también es insuficiente
```

o existe interacción entre ambos.

---

# 5. Prueba 286 — `R_GT(now)` + `omega_pred(now)`

## Configuración

Sustituir únicamente la orientación usada por el controlador:

```text
R_control = R_GT(now)
```

y mantener:

```text
omega_control = omega_pred(now)
```

de la arquitectura 284.

La posición/translación puede mantenerse igual que en 284 si no es necesario tocarla para esta prueba angular.

## Pregunta

> Si la orientación actual es perfecta pero la omega sigue siendo la predicha desde información retardada, ¿el hover se estabiliza?

## Interpretación

Si 286 funciona:

```text
R(now) es el principal problema residual
```

Si 286 falla:

```text
omega(now) también es insuficiente
```

---

# 6. Prueba 287 — `R_GT(now)` + `omega_GT(now)`

## Configuración

Usar:

```text
R_control = R_GT(now)
omega_control = omega_GT(now)
```

manteniendo el resto del pipeline, source handshake, NavigationState y controlador iguales.

## Objetivo

Sanity check.

Debe demostrar que:

```text
el controlador y el pipeline alrededor
siguen siendo estables
```

con estado angular perfecto en el instante actual.

## Interpretación

Si 287 falla:

```text
STOP TOTAL
```

Hay un problema adicional en el pipeline/controlador que invalida la interpretación de 285/286.

Si 287 funciona:

```text
la batería cruzada es válida
```

---

# 7. Orden obligatorio

Ejecutar:

```text
285
286
287
```

No modificar comportamiento funcional entre ellas salvo la selección explícita de:

```text
R source
omega source
```

No usar resultados de una prueba para recalibrar la siguiente.

---

# 8. Instrumentación obligatoria

Registrar en cada tick de control:

```text
control_stamp

R_pred_now
R_GT_now
R_used

omega_pred_now
omega_GT_now
omega_used

orientation_source:
  PREDICTED
  GT_NOW

omega_source:
  PREDICTED
  GT_NOW

er
ew

tau_er
tau_ew
tau_total

P_er_GT
P_ew_GT
P_total_GT
```

Además:

```text
visual_timestamp
visual_age
prediction_horizon

omega_hat_k
alpha_hat

raw class
omega estimator mode

fallback
tracking
```

---

# 9. Métricas principales

Comparar:

```text
284
285
286
287
```

con:

```text
scenario success

govern time

max |er|
max |ew|

RMSE R_pred vs R_GT
RMSE omega_pred vs omega_GT

lag R
lag omega

sign/direction mismatch omega

energia tau_er
energia tau_ew
energia total

first unstable-energy timestamp
first raw reject timestamp

fallback
tracking loss
```

---

# 10. Tabla de diagnóstico esperada

Codex debe devolver una tabla equivalente a:

```text
Prueba    R usada       omega usada      hover
------------------------------------------------
284       predicted     predicted        ?
285       predicted     GT now           ?
286       GT now        predicted        ?
287       GT now        GT now           ?
```

Y una conclusión explícita.

---

# 11. Árbol de decisión

## Caso A

```text
285 funciona
286 falla
287 funciona
```

Conclusión:

> El problema principal es `omega(now)`.

La futura solución deberá reconstruir mejor la velocidad angular actual.

---

## Caso B

```text
285 falla
286 funciona
287 funciona
```

Conclusión:

> El problema principal es `R(now)`.

La futura solución deberá reconstruir mejor la orientación actual.

---

## Caso C

```text
285 falla
286 falla
287 funciona
```

Conclusión:

> Ambas componentes, `R(now)` y `omega(now)`, son necesarias y deben predecirse conjuntamente.

Éste sería el resultado que más justificaría un estimador dinámico completo.

---

## Caso D

```text
285 funciona
286 funciona
287 funciona
```

Conclusión:

> Cada componente predicha por separado puede ser suficiente, pero su combinación actual es incoherente.

Entonces investigar:

```text
coherencia conjunta pose/omega
frame
timestamp exacto
consistencia entre ambas predicciones
```

antes de implementar un modelo dinámico completo.

---

## Caso E

```text
287 falla
```

Conclusión:

> Existe un problema más básico en pipeline/controlador/diagnóstico.

STOP.

No interpretar 285/286.

---

# 12. Qué NO debe concluirse todavía

No asumir automáticamente que hace falta un modelo dinámico con torque.

Primero ejecutar 285-287.

Sólo después decidir si el siguiente paso debe ser:

```text
A) predecir mejor omega(now)

B) predecir mejor R(now)

C) predecir ambas conjuntamente

D) revisar coherencia pose/omega
```

---

# 13. Si el diagnóstico apunta a `omega(now)`

Si 285 funciona y 286 falla:

NO implementar todavía nada dentro de esta misma autorización.

Documentar como siguiente propuesta:

```text
usar dinámica angular del dron
+
torque comandado
+
inercia conocida
```

para predecir:

```text
omega(t_k -> now)
```

a 50 Hz.

---

# 14. Si el diagnóstico apunta a ambas

Si 285 y 286 fallan pero 287 funciona:

Documentar como siguiente propuesta:

```text
estado angular:
R
omega
```

propagado conjuntamente mediante:

```text
omega_dot = J^-1 (tau - omega x J*omega)
R_dot = R [omega]x
```

o convención equivalente correcta.

El predictor usaría:

```text
estado visual en t_k
+
historial/comando de torque
+
modelo físico
```

para reconstruir:

```text
R(now)
omega(now)
```

No implementar todavía bajo esta batería.

---

# 15. Restricción importante sobre GT

Estas pruebas son exclusivamente diagnósticas.

La arquitectura final debe seguir siendo:

```text
ORB
+
estado interno
+
comandos propios del dron
```

sin GT en funcionamiento normal.

GT sólo:

```text
truth de laboratorio
fallback temporal permitido en Fase 5
```

cuando ORB pierda tracking de verdad.

---

# 16. Builds y tests

Antes de ejecutar:

```text
build paquetes afectados
CTest/GTests
analyzer tests
git diff --check
```

Añadir tests focales para garantizar:

```text
modo 285 sólo sustituye omega
modo 286 sólo sustituye R
modo 287 sustituye ambas

GT actual se toma en t_control
no en t_visual
```

No degradar tests actuales.

---

# 17. Qué debe devolver Codex

Respuesta final:

```text
Resultado diagnóstico:
CONSEGUIDO / PARCIAL / NO CONSEGUIDO
```

Incluyendo:

```text
- archivos modificados;
- modos diagnósticos añadidos;
- confirmación de que no se modificó el estimador causal;
- resultado 285;
- resultado 286;
- resultado 287;
- tabla 284/285/286/287;
- er/ew;
- energía;
- RMSE R;
- RMSE omega;
- lag;
- mismatch;
- fallback/tracking;

- conclusión explícita:
    R_NOW PRINCIPAL
    OMEGA_NOW PRINCIPAL
    AMBAS NECESARIAS
    INCOHERENCIA CONJUNTA
    o
    DIAGNÓSTICO INVALIDADO

- siguiente solución recomendada;
- mantener STOP en 279-281.
```

Actualizar:

```text
historial_5H_RESUMEN.md
07_ULTIMA_SESION.md
```

sin borrar historial anterior.

---

# 18. Resumen ejecutivo

El nuevo estimador causal funciona perfectamente sin delay.

Con ~80 ms de delay:

```text
quitar clamp
no arregla

alpha constante
no arregla
```

Ahora no necesitamos otro filtro.

Necesitamos saber qué estado actual estamos reconstruyendo mal.

Ejecutar:

```text
285:
R_pred + omega_GT(now)

286:
R_GT(now) + omega_pred

287:
R_GT(now) + omega_GT(now)
```

y comparar contra:

```text
284:
R_pred + omega_pred
```

> Esta batería debe decidir si el siguiente estimador necesita corregir principalmente `omega(now)`, `R(now)` o ambas conjuntamente.
