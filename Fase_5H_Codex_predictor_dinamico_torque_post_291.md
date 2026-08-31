# Fase 5H — Predictor dinámico angular basado en torque aplicado
## Objetivo: reconstruir `omega(now)` desde una medida visual retardada sin usar GT

## 0. Fuente de verdad

Trabajar sobre el estado ACTUAL del repositorio:

```text
https://github.com/IvanCS-Chenfu/Mapeado-con-Drones
```

Antes de modificar nada, revisar el código vigente relacionado con:

```text
orbslam3_ros2 / NavigationStateEstimator / OrbPosePredictor
control_calcular_fuerzas
aplicar_fuerzas_dron
mixer / reparto a motores
modelo URDF/SDF y parámetros de inercia
```

No asumir dónde está disponible el torque real: auditar primero la cadena completa.

---

# 1. Diagnóstico ya cerrado

Las pruebas 288-291 han aislado el problema inmediato bajo delay.

Resultados:

```text
288:
p/v GT + R_pred + omega_pred
-> FALLA ~2.54 s
-> RMSE omega ~1.811 rad/s
-> energía +0.07979 J

289:
p/v GT + R_pred + omega_GT(now)
-> COMPLETA ~54.46 s
-> RMSE omega ~0.00383 rad/s
-> energía ~-0.000049 J

290:
p/v/R GT + omega_pred
-> FALLA ~5.18 s
-> RMSE omega ~2.024 rad/s
-> energía +0.08887 J

291:
estado GT completo
-> COMPLETA ~55.20 s
-> RMSE omega ~0.00126 rad/s
-> energía ~-0.000066 J
```

Conclusión aceptada:

> Bajo ~80 ms de delay, la causa inmediata de la inestabilidad queda aislada en `omega_pred(now)`.

Además:

```text
R_pred(now)
```

puede mantenerse y el hover completa si:

```text
omega(now)
```

es correcta.

Por tanto, el siguiente bloque debe centrarse en:

```text
omega(t_k) -> omega(now)
```

sin GT.

---

# 2. Idea de la solución

ORB/visión entrega un estado correspondiente a un instante pasado:

```text
t_k
```

En ese instante tenemos conceptualmente:

```text
R(t_k)
omega(t_k)
```

Entre:

```text
t_k
```

y:

```text
now
```

el propio controlador ha ido ordenando torques al dron.

La nueva predicción debe utilizar:

```text
estado angular en t_k
+
historial de torques realmente comandados/aplicados
+
matriz de inercia del dron
+
timestamps
```

para reconstruir:

```text
omega(now)
```

y, después, de forma coherente:

```text
R(now)
```

---

# 3. Modelo dinámico angular

Usar el modelo de cuerpo rígido:

```text
J * omega_dot + omega x (J * omega) = tau
```

por tanto:

```text
omega_dot =
J^-1 * (tau - omega x (J * omega))
```

donde:

```text
J     = matriz de inercia del dron
tau   = torque realmente aplicado/comandado al cuerpo
omega = velocidad angular estimada
```

Después integrar temporalmente:

```text
omega(t + dt)
```

y la orientación:

```text
R(t + dt)
```

con la convención SO(3) correcta del proyecto.

NO copiar fórmulas de left/right integration sin comprobar previamente:

```text
frame de omega
frame de tau
convención de R
orden de multiplicación
```

---

# 4. PASO A — auditar qué torque debe usar el predictor

Antes de implementar el modelo dinámico, recorrer la cadena real:

```text
control_calcular_fuerzas
        ->
comando de fuerza/torque
        ->
mixer / reparto motores
        ->
saturaciones / límites
        ->
aplicar_fuerzas_dron
        ->
wrench aplicado
```

Responder y documentar:

```text
1. ¿Qué variable representa tau deseado?
2. ¿Dónde se aplican saturaciones?
3. ¿Qué torque resulta realmente después del reparto?
4. ¿En qué frame está expresado?
5. ¿Con qué timestamp puede asociarse?
6. ¿Qué señal existe también en ejecución real, sin GT?
```

Preferencia:

> usar la señal más cercana posible al torque efectivamente aplicado al cuerpo después de saturaciones/reparto.

No usar ciegamente `tau_des` si después el mixer modifica lo que realmente recibe el vehículo.

---

# 5. PASO B — auditar la matriz de inercia

Obtener `J` de la fuente real del modelo del dron:

```text
URDF / SDF / parámetros vigentes
```

No duplicar números manualmente si ya existe una fuente de verdad.

Documentar:

```text
Jxx
Jyy
Jzz
Jxy/Jxz/Jyz si aplican
frame en el que está expresada J
```

Si la matriz es diagonal por construcción, demostrarlo a partir del modelo.

---

# 6. Buffer temporal de torque

Crear un buffer corto con los torques aplicados/comandados y su timestamp:

```text
timestamp
tau_body
```

Horizonte suficiente para cubrir:

```text
delay normal de ORB
+
jitter razonable
+
margen
```

Por ejemplo, conceptualmente:

```text
~0.3-0.5 s
```

pero Codex debe justificar el valor con los timings observados.

No guardar historial ilimitado.

---

# 7. Nueva operación de predicción

Cuando existe un estado angular válido en:

```text
t_k
```

y se necesita estado en:

```text
t_target
```

hacer:

```text
omega = omega(t_k)
R = R(t_k)

para cada intervalo de torque entre t_k y t_target:
    calcular omega_dot
    integrar omega
    integrar R
```

El integrador debe utilizar los timestamps reales.

No asumir exactamente:

```text
20 ms
```

aunque el control nominal sea 50 Hz.

---

# 8. Primera implementación: mantenerla simple

Para la primera versión, usar un integrador explícito sencillo pero consistente, por ejemplo:

```text
semi-implicit Euler
```

o equivalente razonablemente estable para los pasos pequeños existentes.

Ejemplo conceptual:

```text
omega_dot =
J^-1 (tau - omega x Jomega)

omega_next =
omega + omega_dot * dt

R_next =
Exp(omega_next * dt) * R
```

SOLO como ejemplo conceptual.

Codex debe elegir el orden correcto según la convención real del proyecto y documentarlo.

No implementar todavía un EKF.

---

# 9. Muy importante — torque y omega deben estar en el mismo frame

Antes de integrar:

```text
tau
omega
J
```

deben ser compatibles.

Si:

```text
omega
```

está en frame body:

```text
tau
```

también debe estar en body y `J` en body.

Si la `omega_motion` actual está en O/world-like frame, convertir explícitamente al frame dinámico adecuado, integrar ahí y volver a convertir si el contrato de `NavigationState` lo requiere.

No mezclar frames silenciosamente.

Añadir GTests específicos para esto.

---

# 10. Separación entre medida visual y predicción dinámica

Mantener la arquitectura:

```text
visión/ORB
    ->
corrige estado en t_k

modelo dinámico
    ->
propaga t_k -> now
```

No convertir el torque en una corrección visual.

No usar el modelo dinámico para alterar:

```text
raw gates
SMALL/MODERATE
reference KF
map epoch
W
```

El modelo sólo cubre el intervalo temporal sin observación.

---

# 11. GT no forma parte de la solución

La implementación final NO puede usar:

```text
omega_GT
R_GT
p_GT
v_GT
```

para construir el estado operativo.

GT se utilizará únicamente en las pruebas siguientes como:

```text
truth externa de laboratorio
```

para validar el predictor.

---

# 12. PRUEBA 292 — validar sólo la dinámica `omega(t_k) -> omega(now)`

Esta prueba debe aislar el nuevo modelo físico.

## Entrada

Usar:

```text
omega inicial = omega_GT(t_k)
R inicial = R_GT(t_k)
```

pero RETARDADAS artificialmente ~80 ms.

Desde `t_k` hasta `now`:

```text
NO usar omega_GT
NO usar R_GT
```

Usar únicamente:

```text
historial real de torque
J
dt
modelo dinámico
```

para predecir:

```text
omega_dynamic(now)
R_dynamic(now)
```

## Uso para control

Para aislar `omega`, preferencia inicial:

```text
p_control = p_GT(now)
v_control = v_GT(now)
R_control = R_GT(now)
omega_control = omega_dynamic(now)
```

Así la única variable bajo prueba en el lazo es la omega predicha físicamente.

## Métricas

Comparar:

```text
omega_dynamic(now)
vs
omega_GT(now)
```

medir:

```text
RMSE
MAE
correlación
lag
sign mismatch
max error
```

y:

```text
ew
tau_ew
energía angular
hover success
```

---

# 13. Criterio de 292

Si 292 falla claramente:

```text
STOP
```

No introducir todavía la omega causal visual.

Revisar:

```text
tau usado
frame
J
timestamp
integración
saturaciones
signos
```

El modelo dinámico debe demostrar primero que puede reconstruir `omega(now)` partiendo de un `omega(t_k)` perfecto.

Si 292 completa el hover:

```text
modelo dinámico básico VALIDADO
```

Repetir una segunda vez para reproducibilidad.

---

# 14. PRUEBA 293 — `omega(t_k)` estimada desde poses GT a 20 Hz + modelo dinámico

Sólo si 292 funciona.

Ahora sustituir:

```text
omega_GT(t_k)
```

por:

```text
omega_hat_k
```

obtenida mediante el estimador causal de tres poses ya validado en 276/277.

Entrada:

```text
poses GT perfectas a ~20 Hz
delay ~80 ms
omega_hat_k causal
R(t_k) de la medida
historial de torque
J
```

Salida:

```text
omega_dynamic(now)
```

Para aislar todavía la omega:

```text
p_control = p_GT(now)
v_control = v_GT(now)
R_control = R_GT(now)
omega_control = omega_dynamic(now)
```

Objetivo:

> comprobar la cadena completa `poses lentas -> omega(t_k) -> dinámica -> omega(now)`.

---

# 15. Criterio de 293

Debe acercarse claramente a:

```text
289 / 291
```

y alejarse de:

```text
288 / 290
```

No exigir RMSE idéntico a GT, pero sí:

```text
hover completo
mismatch bajo
energía no creciente
```

Si funciona, repetir una vez.

---

# 16. PRUEBA 294 — usar también `R_dynamic(now)`

Sólo si 293 funciona.

Usar:

```text
p_control = p_GT(now)
v_control = v_GT(now)

R_control = R_dynamic(now)
omega_control = omega_dynamic(now)
```

Objetivo:

> verificar que pose angular y omega provenientes del mismo modelo dinámico son coherentes y estables.

Esto es importante porque la arquitectura final debe publicar:

```text
R(now)
omega(now)
```

del mismo estado físico.

---

# 17. PRUEBA 295 — estado translacional retardado normal + predictor dinámico angular

Sólo si 294 funciona.

Volver a usar la ruta translacional normal del predictor:

```text
p_control = p_pred(now)
v_control = v_pred(now)

R_control = R_dynamic(now)
omega_control = omega_dynamic(now)
```

con:

```text
GT pose a ~20 Hz
delay ~80 ms
```

Objetivo:

> comprobar que la mejora angular sigue resolviendo el fallo sin ayudas GT en p/v.

GT sólo registra métricas.

Si 295 completa hover:

```text
delay fijo con predictor dinámico angular = VALIDADO
```

---

# 18. Después de 295

NO saltar directamente a trayectoria.

Secuencia siguiente:

```text
296:
GT perfecto con jitter/timing realista ORB
+ predictor dinámico

si funciona:

297:
ORB REAL en hover
+ predictor dinámico

si funciona:

298:
repetición ORB REAL
```

Sólo tras dos hovers ORB reales completos debatir el paso a movimiento/etapa 3.

---

# 19. GTests obligatorios

Añadir como mínimo:

## A — torque cero

```text
tau = 0
omega = 0
```

Esperado:

```text
omega permanece 0
R permanece constante
```

## B — torque constante en un eje principal

Con J diagonal conocida:

```text
omega_dot = tau / J
```

Esperado analíticamente.

## C — signo de torque

Torque positivo/negativo debe producir aceleración angular con signo físico correcto.

## D — término giroscópico

Caso con omega no alineada con eje principal:

```text
omega x (J omega)
```

debe evaluarse correctamente.

## E — timestamps irregulares

Integración debe usar `dt` real.

## F — buffer recortado

No acceder a torque anterior al historial disponible sin marcar degradación.

## G — frame

Verificar transformación:

```text
omega/tau body
```

sin mezclar con O/world.

## H — saturación/mixer

Si existe un torque solicitado distinto al realizable:

```text
predictor usa la señal acordada después de saturación
```

## I — reproducción analítica simple

Una secuencia artificial conocida debe dar:

```text
omega_pred
R_pred
```

dentro de tolerancia.

---

# 20. Telemetría obligatoria

Registrar:

```text
dynamic_predictor_enabled

state_base_timestamp
target_timestamp
dynamic_horizon

omega_initial
R_initial

tau_timestamp
tau_used_body

J

omega_dot
omega_before
omega_after

R_before
R_after

integration_dt
integration_steps

torque_buffer_age
torque_samples_used
missing_torque_interval

omega_dynamic_now
R_dynamic_now

omega_GT_now   # diagnóstico
R_GT_now       # diagnóstico

omega_error
orientation_error
```

Y del controlador:

```text
er
ew
tau_er
tau_ew
tau_total
P_total_GT
```

---

# 21. Manejo de huecos de torque

Si existe un intervalo entre:

```text
t_k
```

y:

```text
now
```

sin torque conocido:

NO inventar silenciosamente.

Registrar:

```text
missing_torque_interval=true
```

y definir una política explícita.

Para laboratorio puede usarse:

```text
último torque conocido durante un horizonte muy corto
```

si se justifica.

Pero medir cuántas veces ocurre.

---

# 22. No rediseñar todavía el controlador

No cambiar:

```text
Kr
Kw
Kp
Kv
```

El diagnóstico 288-291 ya demuestra que el controlador funciona cuando recibe una omega actual correcta.

No compensar errores del predictor cambiando gains.

---

# 23. No eliminar el estimador causal de tres poses

El estimador causal sigue siendo útil para obtener:

```text
omega(t_k)
```

desde visión.

El nuevo modelo dinámico no lo sustituye.

La nueva cadena será:

```text
poses ORB
    ->
omega causal en t_k
    ->
modelo dinámico usando torques
    ->
omega(now)
```

---

# 24. Resultado esperado arquitectónico

Si las pruebas funcionan, la arquitectura final angular debe ser:

```text
                   ORB ~20 Hz
                       |
                       v
              R(t_k), omega(t_k)
                       |
                       |
             torque buffer 50 Hz
                       |
                       v
              Dynamic Predictor
                       |
                       v
               R(now), omega(now)
                       |
                       v
                   Control
```

Sin GT.

---

# 25. Qué debe devolver Codex

Al terminar esta autorización, devolver:

```text
Resultado:
CONSEGUIDO / PARCIAL / NO CONSEGUIDO
```

Incluyendo:

```text
- auditoría de la cadena de torque;
- señal exacta elegida como tau y por qué;
- frame de tau;
- fuente de J;
- matriz J utilizada;
- arquitectura del buffer;
- integrador elegido;

- builds;
- GTests añadidos;
- total GTests;
- analyzer tests;
- git diff --check;

- prueba 292;
- repetición de 292 si funciona;

- prueba 293 si corresponde;
- repetición si funciona;

- prueba 294 si corresponde;

- prueba 295 si corresponde;

- RMSE/MAE/correlación/lag de omega;
- mismatch direccional;
- energía;
- hover success;

- conclusión:
    MODELO DINÁMICO VALIDADO / NO VALIDADO
    CADENA VISUAL+DINÁMICA VALIDADA / NO VALIDADA
    DELAY FIJO RESUELTO / NO RESUELTO

- decisión:
    continuar a jitter
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

# 26. Resumen ejecutivo

Las pruebas 288-291 ya demuestran:

```text
omega_pred(now) mala
-> hover falla

omega_GT(now) correcta
-> hover completa
```

Incluso:

```text
R_pred + omega_GT(now)
```

completa el escenario.

Por tanto, ya no se deben probar más filtros visuales para adivinar el futuro.

La siguiente solución debe usar información que sí existe entre:

```text
t_k
```

y:

```text
now
```

pero que el predictor visual estaba ignorando:

```text
los torques que el propio dron está aplicando.
```

Primero validar:

```text
omega_GT(t_k)
+ torque real
+ J
-> omega(now)
```

Después sustituir progresivamente GT por:

```text
omega causal obtenida desde poses
```

y sólo cuando el modelo físico funcione con delay fijo avanzar a jitter y ORB real.
