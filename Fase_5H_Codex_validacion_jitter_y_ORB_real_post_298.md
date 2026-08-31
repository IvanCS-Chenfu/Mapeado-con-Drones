# Fase 5H — Validación final del predictor dinámico: timing/jitter realista → ORB real

## 0. Fuente de verdad

Trabajar sobre el estado ACTUAL del repositorio:

```text
https://github.com/IvanCS-Chenfu/Mapeado-con-Drones
```

Revisar el código vigente antes de tocar nada y reutilizar la infraestructura diagnóstica ya creada.

---

# 1. Estado actual validado

El predictor dinámico angular queda validado en laboratorio.

La inercia correcta compartida entre controlador y predictor es:

```text
J = diag(
    0.00803107,
    0.00803107,
    0.015805
) kg·m²
```

Con esta matriz:

```text
296: completada
297: completada
293: completada
298: completada
294: completada
295: completada
```

Resultados globales:

```text
gobierno estable: ~54.62–55.12 s
RMSE omega: ~0.00255–0.00557 rad/s
mismatch máximo: ~0.777 %
energía angular total: negativa
fallback: ninguno
tracking no-OK: ninguno
```

Conclusión vigente:

> El estimador causal de `omega(t_k)` + predictor dinámico con torque e inercia real reconstruyen correctamente el estado angular bajo delay fijo de laboratorio.

Fase 5H sigue:

```text
PARCIAL
```

porque todavía falta validar:

```text
1. timing/jitter realista de ORB;
2. ORB real;
3. repetibilidad con ORB real.
```

---

# 2. Regla principal

CONGELAR por ahora:

```text
estimador causal de omega(t_k)
modelo dinámico rígido
buffer de torque
matriz J
integración angular
control gains
SMALL/MODERATE
raw gates
KF/reference
W
mux
trayectoria
```

No cambiar nada de ese bloque salvo que una prueba posterior demuestre un fallo concreto.

No hacer más calibraciones por intuición.

---

# 3. Prueba 299 — GT perfecto con timing/jitter realista de ORB

## Objetivo

Mantener geometría perfecta de GT, pero reproducir las características temporales reales observadas en ORB:

```text
frecuencia efectiva real
delay variable
jitter realista
dt irregulares
timestamps físicos originales
```

Usar preferentemente la infraestructura/patrón ya medido en pruebas anteriores.

No inventar jitter arbitrario si ya existen datos históricos.

La cadena debe ser:

```text
GT pose perfecta
    ->
muestreo/timing similar a ORB
    ->
estimador causal de omega(t_k)
    ->
predictor dinámico con torque + J
    ->
R(now), omega(now)
    ->
NavigationState
    ->
control 50 Hz
```

GT NO debe aportar:

```text
omega(now)
R(now)
```

al controlador.

GT sólo sirve como truth externa para métricas.

---

# 4. Métricas obligatorias de 299

Registrar:

```text
measurement timestamp
arrival timestamp
visual age
dt entre medidas
jitter
prediction horizon

omega_hat_k
omega_dynamic_now
omega_GT_now

R_dynamic_now
R_GT_now

torque samples usados
torque buffer age
missing torque interval

RMSE omega
MAE omega
correlación
lag
mismatch direccional

max |er|
max |ew|

tau_er
tau_ew
tau_total

energía angular total

fallback
tracking
scenario success
```

También contar:

```text
DEGRADED_DT
REJECTED
resets del historial
prediction clamp
missing torque intervals
```

---

# 5. Criterio de 299

Si completa el hover:

```text
TIMING/JITTER REALISTA VALIDADO
```

Repetir una segunda vez antes de pasar a ORB real.

Nombre sugerido:

```text
300
```

Condición:

```text
299 = CONSEGUIDA
300 = CONSEGUIDA
```

Si 299 falla:

```text
STOP
```

No conectar ORB real.

Analizar exclusivamente:

```text
jitter
dt irregulares
buffer de torque
gaps
resets
prediction horizon
```

sin tocar la arquitectura completa.

---

# 6. Prueba 301 — ORB REAL en hover

Ejecutar sólo si 299 y 300 funcionan.

Ruta real:

```text
cámaras
    ->
ORB-SLAM3
    ->
pose visual O
    ->
estimador causal omega(t_k)
    ->
predictor dinámico torque + J
    ->
R(now), omega(now)
    ->
NavigationState
    ->
control
```

No usar GT para:

```text
pose
velocidad lineal
orientación
omega
correcciones
targets
```

GT sólo puede quedar como:

```text
métrica externa
fallback temporal de Fase 5 ante pérdida REAL de tracking
```

Para este hover, el objetivo es:

```text
ORB gobierna todo el escenario
sin fallback
```

---

# 7. Telemetría extra para ORB real

Registrar además:

```text
tracking_state
map_epoch

reference_kf
reference changes

raw angular step
raw motion class

SMALL
MODERATE_PENDING
MODERATE_CONFIRMED
REJECTED_EXCESSIVE

base_update_type
visual_base_error_before
visual_base_error_after

omega_hat_k
omega_dynamic_now

torque buffer status

fallback cause
fallback timestamp
tracking loss timestamp
```

El objetivo es poder distinguir claramente:

```text
problema temporal
problema dinámico
problema de medida ORB
problema de reference KF
pérdida visual real
```

---

# 8. Criterio de 301

Si completa el hover:

```text
ORB REAL HOVER = CONSEGUIDO
```

Repetir exactamente como:

```text
302
```

Si:

```text
301 = CONSEGUIDA
302 = CONSEGUIDA
```

entonces considerar:

```text
hover ORB estabilizado y reproducible
```

Sólo entonces debatir la siguiente etapa con movimiento/seguimiento de trayectoria.

---

# 9. Si ORB real falla pero 299/300 funcionan

Ésta sería una conclusión muy importante.

No volver automáticamente a tocar:

```text
timing
predictor dinámico
J
torques
omega(t_k)->omega(now)
```

porque ya estarían validados con timing realista.

El siguiente diagnóstico debe centrarse en la propia medida ORB:

```text
R_visual
pose visual
saltos raw

reference KF
cambios de referencia

SMALL/MODERATE
probation
rechazos

tracking
```

Comparar con GT únicamente como truth externa.

---

# 10. Si 301 falla por pérdida REAL de tracking

Distinguir:

```text
A) el dron se vuelve inestable y después ORB se pierde
```

de:

```text
B) ORB pierde tracking primero por mala textura y después entra fallback
```

Caso A:

```text
Fase 5H sigue fallando
```

Caso B:

```text
puede ser comportamiento esperado de Fase 5
```

si el fallback es limpio y la pérdida visual es legítima.

No considerar aceptable:

```text
ORB control -> oscilación -> caída -> tracking loss
```

---

# 11. Builds y tests

Antes de cada prueba:

```text
build paquetes afectados
GTests/CTest
analyzer tests
git diff --check
```

No degradar:

```text
75/75 GTests
```

o el número superior existente en el repositorio actual.

Si se añade nueva infraestructura de timing/jitter, añadir tests para verificar:

```text
timestamps originales conservados
jitter reproducible
delay no falsifica measurement_stamp
torque buffer cubre el intervalo correcto
GT no entra accidentalmente en la salida
```

---

# 12. Comparación obligatoria final

Preparar tabla:

```text
Prueba        Fuente pose       Timing            Omega now            Resultado
--------------------------------------------------------------------------------
296/297       GT                delay fijo        dinámica + torque     OK
293/298       GT 20 Hz          delay fijo        causal+dinámica       OK
294           GT                delay fijo        R/omega dinámica      OK
295           GT                delay fijo        estado normal         OK
299           GT                jitter realista   causal+dinámica       ?
300           GT                jitter realista   causal+dinámica       ?
301           ORB real          real              causal+dinámica       ?
302           ORB real          real              causal+dinámica       ?
```

Y comparar:

```text
RMSE omega
MAE omega
lag
mismatch

max |er|
max |ew|

energía angular
fallback
tracking loss
hover success
```

---

# 13. Qué NO hacer

No modificar todavía:

```text
control gains
J
modelo dinámico
integrador
SMALL/MODERATE
KF logic
W
mux
trayectoria
```

No implementar:

```text
EKF
nuevo filtro
Delta_target
otro predictor
```

salvo que una prueba demuestre la necesidad.

---

# 14. Qué debe devolver Codex

Al terminar:

```text
Resultado:
CONSEGUIDO / PARCIAL / NO CONSEGUIDO
```

Incluir:

```text
- commit/estado de código usado;
- archivos modificados;
- confirmación de que predictor dinámico/J quedaron congelados;

- resultado 299;
- métricas 299;

- resultado 300 si 299 funciona;

- resultado 301 si 299/300 funcionan;
- duración ORB;
- fallback;
- tracking;
- eventos de KF;
- clasificación SMALL/MODERATE/REJECTED;
- RMSE/lag/mismatch;
- energía angular;

- resultado 302 si 301 funciona;

- conclusión:
    TIMING/JITTER VALIDADO / NO VALIDADO
    ORB REAL VALIDADO / NO VALIDADO
    HOVER ORB REPRODUCIBLE / NO REPRODUCIBLE

- siguiente paso recomendado.
```

Actualizar:

```text
historial_5H_RESUMEN.md
07_ULTIMA_SESION.md
contrato 5H
```

sólo si cambia una semántica arquitectónica real.

---

# 15. Resumen ejecutivo

El predictor dinámico ya está validado con:

```text
omega causal en t_k
+
torques aplicados
+
J real
+
delay fijo
```

y completa todas las pruebas 296-298/293-295.

Ahora avanzar estrictamente:

```text
299:
GT perfecto + timing/jitter realista ORB
        ↓ si funciona
300:
repetición
        ↓ si funciona
301:
ORB REAL hover
        ↓ si funciona
302:
repetición ORB REAL
```

> Añadir una sola dificultad por prueba. Detenerse en el primer punto que falle y diagnosticar ese punto antes de modificar otra cosa.
