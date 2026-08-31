# Instrucciones para Codex — Fase 5H
## Validación progresiva del nuevo estimador causal: delay → timing/jitter realista → ORB real

## 0. Fuente de verdad

Trabajar sobre el estado ACTUAL del repositorio:

```text
https://github.com/IvanCS-Chenfu/Mapeado-con-Drones
```

Antes de modificar nada, revisar en `main`:

```text
dron/orbslam3_ros2/src/stereo/navigation-state-estimator.cpp
dron/orbslam3_ros2/src/stereo/navigation-state-estimator.hpp
dron/orbslam3_ros2/test/test_navigation_state_estimator.cpp
```

y reutilizar la infraestructura diagnóstica de Fase 5H usada en 269-277 para:

```text
f5h_gt_timing_mode
GT_LAB_ONLY
publicación 50 Hz
delay/timing artificial
métricas de fase/control
```

No crear un banco de pruebas paralelo si ya existe uno válido.

---

# 1. Estado demostrado

Las pruebas 273-275 aislaron que el problema principal estaba en la antigua derivación/filtrado de `omega_motion`.

Después se implementó el nuevo estimador causal:

```text
últimas tres poses válidas
    ->
velocidades medias entre intervalos
    ->
aceleración angular entre midpoints
    ->
proyección hasta el timestamp de la última medida
    ->
omega_motion
```

Resultados:

```text
Prueba 276: CONSEGUIDA
Prueba 277: CONSEGUIDA
```

Ambas usan:

```text
pose GT perfecta a ~20 Hz
+
omega_motion calculada por el NUEVO estimador
+
control/publicación a 50 Hz
```

y completan el hover:

```text
sin fallback
sin pérdida de tracking
sin oscilación creciente
energía total negativa
```

RMSE angular:

```text
276: ~0.00374 rad/s
277: ~0.00304 rad/s
```

frente a:

```text
270/B antiguo estimador: ~0.43338 rad/s
```

Por tanto:

> El bloque de estimación causal de `omega_motion` queda validado y reproducible en laboratorio SIN delay visual realista.

Fase 5H sigue `PARCIAL` porque todavía falta validar:

```text
1. delay fijo;
2. delay/jitter realista;
3. ORB real.
```

---

# 2. Regla principal

NO modificar todavía el estimador causal.

Congelar:

```text
TWO_SAMPLE
THREE_SAMPLE_PREDICTED
omega_mid
alpha_hat
omega_hat_k
```

No tocar tampoco:

```text
Kr / Kw / Kp / Kv
SMALL / MODERATE
anclajes visuales
raw gates
bias/deadband
decay
reference KF
W
mux
trayectoria
ORB-SLAM3 core
extrínseca
```

La finalidad es añadir una sola dificultad por prueba.

---

# 3. Prueba 278 — GT perfecto 20 Hz + delay fijo ~80 ms

Configurar:

```text
pose fuente: GT perfecta
frecuencia: ~20 Hz
delay artificial: ~80 ms
control/publicación: 50 Hz
omega de salida: NUEVO estimador causal
```

NO usar `omega_GT` como entrada al controlador. Sólo como truth de diagnóstico.

## Semántica temporal obligatoria

La medida retrasada debe conservar su timestamp físico original.

Ejemplo:

```text
pose representa t = 10.000 s
llega en t = 10.080 s
```

No etiquetarla como si representase `10.080 s`.

Debe existir conceptualmente:

```text
measurement = pose(t_visual)
measurement_stamp = t_visual
arrival/receive_stamp = t_visual + delay
```

---

# 4. Métricas de 278

Registrar:

```text
omega_GT
omega_raw
omega_mid_prev/current
alpha_hat
omega_hat_k
omega_motion publicada

pose visual
pose base
pose predicha
pose usada por control

visual_age
receive_age
prediction_horizon
prediction_clamped

er
ew

tau_er
tau_ew
tau_total

P_er_GT
P_ew_GT
P_total_GT

omega_estimator_mode
raw_motion_class
```

Calcular:

```text
RMSE / MAE omega
lag por eje
correlación por eje
mismatch direccional GT/control
energía tau_er
energía tau_ew
energía total
```

## Decisión

Si 278 completa el hover:

```text
DELAY ~80 ms VALIDADO
```

y pasar a 279.

Si falla:

```text
STOP
```

No ejecutar 279 ni ORB real.

Analizar específicamente si:

```text
omega_hat_k es correcta en t_k
pero se mantiene demasiado tiempo hasta now
```

antes de diseñar cambios.

No implementar preventivamente una nueva compensación.

---

# 5. Prueba 279 — GT perfecto con timing/jitter realista de ORB

Ejecutar sólo si 278 funciona.

Mantener pose GT geométricamente perfecta, pero reproducir:

```text
frecuencia efectiva realista
delay variable
jitter
dt reales
timestamps físicos originales
publicación/control 50 Hz
```

Usar preferentemente el patrón ya medido/implementado en 269-272. No inventar jitter arbitrario.

La `omega_motion` debe seguir saliendo del nuevo estimador causal.

## Decisión

Si 279 completa el hover:

```text
20 Hz VALIDADO
delay VALIDADO
jitter/timing realista VALIDADO
```

y pasar a 280.

Si falla:

```text
STOP
```

Analizar:

```text
dt
DEGRADED_DT
REJECTED
historial de tres muestras
resets
prediction clamp
gaps/stale samples
mismatch direccional
```

No cambiar thresholds por intuición.

---

# 6. Prueba 280 — ORB REAL en hover

Ejecutar sólo si 278 y 279 funcionan.

Ruta real:

```text
cámaras
    ->
ORB-SLAM3
    ->
pose O visual
    ->
nuevo estimador causal omega_motion
    ->
NavigationState
    ->
control 50 Hz
```

No usar GT para construir pose, omega, correcciones o targets.

GT sólo puede seguir registrándose para métricas externas y conservar el `GT_FALLBACK` temporal permitido por Fase 5 para una pérdida REAL de tracking.

Para esta prueba de hover el objetivo es:

```text
ORB gobierna TODO el hover
sin fallback
```

Registrar además:

```text
tracking_state
map_epoch
reference_kf
reference changes

SMALL
MODERATE_PENDING
MODERATE_CONFIRMED
REJECTED_EXCESSIVE

base_update_type
visual_base_error

raw angular step
fallback cause
timestamp fallback
timestamp tracking loss
```

---

# 7. Interpretación de 280

Si 280 completa el hover:

```text
adaptación 20->50 Hz   OK
omega causal           OK
delay                   OK
jitter/timing           OK
ORB real                OK en hover
```

Repetir exactamente como:

```text
prueba 281
```

Si 280 y 281 funcionan:

> considerar el hover ORB estabilizado de forma reproducible y sólo entonces debatir/autorizar etapa 3.

Si 278 y 279 funcionan pero 280 falla:

> NO volver a rediseñar automáticamente el adaptador temporal.

El foco siguiente será la calidad/geometría de las medidas ORB reales:

```text
R_visual
raw steps
reference KF
gates
SMALL/MODERATE
tracking
```

---

# 8. Comparación obligatoria

Generar tabla:

```text
                     270/B   273/E   276   277   278   279   280/281
--------------------------------------------------------------------
fuente pose
omega usada
delay
jitter
scenario success
ORB govern time
fallback
tracking loss

RMSE omega
MAE omega
correlación
lag
mismatch direccional

max |er|
max |ew|

energía tau_er
energía tau_ew
energía total

prediction horizon
prediction clamp %
```

Usar `N/A` cuando una métrica no aplique.

---

# 9. Criterio de progreso

No aceptar:

```text
“aguantó unos segundos más”
```

como éxito.

Cada prueba es:

```text
hover completo
o
hover no completo
```

y debe explicarse causalmente.

La secuencia es:

```text
278 -> delay
279 -> jitter/timing
280 -> ORB real
281 -> repetición ORB real
```

Detenerse en el primer punto que falle.

---

# 10. No implementar todavía otras ideas

No retomar todavía:

```text
Delta_target
target visual persistente
EKF
modelo dinámico con torque
fusión con comandos
nuevo filtro pasa-bajos
cambio de gains
```

Sólo si estas pruebas demuestran que hacen falta.

---

# 11. Builds y tests

Antes de cada simulación relevante:

```text
build de paquetes afectados
CTest/GTests del estimador
tests del analizador
git diff --check
```

No degradar los tests vigentes:

```text
55/55 GTests
8/8 tests del analizador
```

o el número superior que exista cuando Codex empiece.

Si se añaden modos diagnósticos, añadir tests focales para:

```text
timestamp original conservado
delay aplicado sólo a arrival/availability
jitter reproducible
omega_GT nunca usada accidentalmente como salida
```

---

# 12. Documentación

Actualizar tras cada prueba:

```text
historial_5H_RESUMEN.md
07_ULTIMA_SESION.md
```

Actualizar contrato 5H sólo si cambia una semántica arquitectónica real.

No borrar historial anterior.

---

# 13. Qué debe devolver Codex

Al terminar:

```text
Resultado:
CONSEGUIDO / PARCIAL / NO CONSEGUIDO
```

Incluyendo:

```text
- estado/commit de código usado;
- archivos modificados;
- confirmación de que el estimador causal se mantuvo sin cambios;
- resultado y métricas de 278;
- decisión CONTINUE/STOP;
- resultado y métricas de 279 si correspondía;
- resultado y métricas de 280 si correspondía;
- resultado 281 si 280 funcionó;
- duración ORB;
- fallback/tracking;
- energía;
- calidad de omega;
- eventos angular/KF;

- conclusión:
    DELAY VALIDADO / NO VALIDADO
    JITTER VALIDADO / NO VALIDADO
    ORB REAL VALIDADO / NO VALIDADO

- siguiente paso recomendado.
```

---

# 14. Resumen ejecutivo

276 y 277 ya demuestran:

```text
GT perfecto 20 Hz
+
omega calculada causalmente desde poses
+
control 50 Hz
=
hover estable y reproducible
```

No tocar ese estimador todavía.

Ahora:

```text
278: GT 20 Hz + ~80 ms delay
        ↓ si funciona
279: GT + timing/jitter realista ORB
        ↓ si funciona
280: ORB REAL en hover
        ↓ si funciona
281: repetir ORB REAL
```

> Añadir una dificultad por prueba y detenerse exactamente en el primer punto que falle.
