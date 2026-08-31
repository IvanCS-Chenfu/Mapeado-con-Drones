# Fase 5H — Aislamiento y corrección del canal translacional `p/v`
## Objetivo: diagnosticar si el fallo restante está principalmente en `v(now)`, en `p(now)` o en ambas, y validar después un predictor dinámico translacional sin GT

## 0. Fuente de verdad

Trabajar sobre el estado ACTUAL del repositorio:

```text
https://github.com/IvanCS-Chenfu/Mapeado-con-Drones
```

Tomar como base:

```text
prueba 299
pruebas 303-306
historial_5H_RESUMEN.md
código actual de control_calcular_fuerzas.cpp
código actual de aplicar_fuerzas_dron.cpp
predictor dinámico angular ya validado
```

Mantener detenidas:

```text
300
301
302
```

No conectar todavía ORB real.

---

# 1. Estado actual ya demostrado

La batería 303-306 ha aislado el problema restante:

```text
303:
p/v GT + angular dinámica
-> COMPLETA ~54.54 s

305:
p/v predichas + angular GT(now)
-> FALLA ~15.06 s

306:
estado GT(now) completo
-> COMPLETA ~54.60 s
```

Conclusión aceptada:

```text
PV PRINCIPAL
```

Por tanto:

> El bloque angular dinámico ya soporta el timing/jitter de 299 cuando `p/v` son correctas.

No volver a tocar ahora:

```text
J
predictor dinámico angular
omega causal
buffer de torque
integración angular
control gains
SMALL/MODERATE
KF/reference
W
mux
```

La nueva investigación debe centrarse exclusivamente en:

```text
p(t_k), v(t_k) -> p(now), v(now)
```

---

# 2. Por qué `p/v` pueden desestabilizar el control

El controlador usa:

```text
ep = p - p_des
ev = v - v_des
```

y después:

```text
F_des =
-Kp * ep
-Kv * ev
+m * (a_des - g)
```

Por tanto, un error temporal en:

```text
p
v
```

modifica directamente:

```text
F_des
```

y con ello:

```text
R_des
torque
movimiento real del dron
```

Especialmente importante:

```text
Kv = 5
```

por lo que una velocidad lineal desfasada puede tener un efecto fuerte.

---

# 3. Primer objetivo: separar posición y velocidad

Antes de implementar el predictor dinámico translacional completo, ejecutar dos pruebas cruzadas.

---

# 4. Prueba 307 — `p_GT(now)` + `v_pred(now)` + angular GT(now)

Configurar:

```text
p_control = p_GT(now)
v_control = v_pred(now)

R_control = R_GT(now)
omega_control = omega_GT(now)
```

Mantener:

```text
mismo timing/jitter de 299
mismo escenario
mismo controlador
mismas ganancias
```

## Pregunta

> ¿La velocidad lineal predicha es por sí sola suficiente para desestabilizar el hover?

## Interpretación

Si 307 falla:

```text
v(now) es crítica
```

---

# 5. Prueba 308 — `p_pred(now)` + `v_GT(now)` + angular GT(now)

Configurar:

```text
p_control = p_pred(now)
v_control = v_GT(now)

R_control = R_GT(now)
omega_control = omega_GT(now)
```

## Pregunta

> ¿La posición predicha es por sí sola suficiente para desestabilizar el hover?

## Interpretación

Si 308 falla:

```text
p(now) también es crítica
```

---

# 6. Sanity de referencia

No hace falta crear una prueba nueva si 306 sigue reproducible:

```text
p_GT(now)
v_GT(now)
R_GT(now)
omega_GT(now)
```

debe completar el hover.

Si existe cualquier duda sobre reproducibilidad:

```text
repetir 306
```

antes de interpretar 307/308.

---

# 7. Árbol de diagnóstico 307-308

## Caso A

```text
307 falla
308 funciona
```

Conclusión:

> `v(now)` es el problema translacional dominante.

---

## Caso B

```text
307 funciona
308 falla
```

Conclusión:

> `p(now)` es el problema translacional dominante.

---

## Caso C

```text
307 falla
308 falla
```

Conclusión:

> `p(now)` y `v(now)` deben reconstruirse conjuntamente.

Ésta sería además la solución físicamente más coherente.

---

## Caso D

```text
307 funciona
308 funciona
```

Conclusión:

> Ninguna componente por separado explica el fallo, pero la combinación `p_pred + v_pred` es incoherente.

Entonces estudiar coherencia conjunta y timestamp antes de cambiar gains.

---

# 8. Telemetría obligatoria de 307/308

Registrar:

```text
control_stamp

p_pred
p_GT_now
p_used

v_pred
v_GT_now
v_used

R_used
omega_used

position_source
velocity_source
orientation_source
omega_source

ep
ev
F_des

er
ew
tau_total

visual_age
dt_visual
prediction_horizon

scenario success
govern time
fallback
tracking
```

Calcular:

```text
RMSE p
RMSE v
MAE p
MAE v
lag p
lag v

max |ep|
max |ev|

energía angular
hover success
```

---

# 9. Si el diagnóstico confirma problema en `p/v`

Después de 307/308, implementar un predictor dinámico translacional.

La filosofía debe ser equivalente a la ya validada para la parte angular.

Arquitectura objetivo:

```text
pose visual en t_k
    |
    +--> p(t_k)
    +--> v(t_k)
    |
fuerzas aplicadas
masa
gravedad
R_dynamic(t)
    |
    v
integración dinámica
    |
    v
p(now), v(now)
```

---

# 10. Física translacional

Si el thrust total efectivo está aplicado en el eje Z del cuerpo:

```text
F_body =
[0, 0, T]
```

entonces:

```text
a_O =
R_O_B * F_body / m
+
g_O
```

donde:

```text
R_O_B = orientación dinámica actual
m = masa total real
g_O = gravedad en frame O
```

Después integrar:

```text
v_next =
v + a * dt
```

y:

```text
p_next =
p + v * dt + 0.5 * a * dt^2
```

o un integrador equivalente coherente.

Codex debe revisar:

```text
frame de F
frame de p/v
convención de R
signo de gravedad
```

antes de implementar literalmente.

---

# 11. Auditar la fuerza realmente aplicada

Antes de usar el modelo dinámico translacional, revisar:

```text
control_calcular_fuerzas
    ->
control/tray/fuerza
    ->
aplicar_fuerzas_dron
    ->
mixer
    ->
fuerzas de motores
    ->
Gazebo
```

Documentar:

```text
1. qué representa exactamente control/tray/fuerza;
2. en qué frame está;
3. si es thrust total o sólo componente Z body;
4. si existen saturaciones después;
5. qué fuerza termina aplicándose realmente;
6. qué timestamp puede asociarse.
```

Preferencia:

> usar la señal más cercana posible a la fuerza realmente aplicada al dron.

Si el mixer no aplica saturaciones ni clipping, documentarlo.

---

# 12. Auditar la masa real

Antes de implementar:

```text
verificar masa compuesta real del modelo Gazebo
```

No asumir automáticamente:

```text
1.4 kg
```

aunque sea el valor actual del controlador.

Comparar:

```text
masa configurada en controlador
masa total real del URDF/SDF/Gazebo
```

Si difieren:

```text
corregir para compartir una única fuente de verdad
```

igual que se hizo con la inercia angular.

---

# 13. Buffer temporal de fuerza

Crear/reutilizar un buffer corto:

```text
timestamp
thrust/fuerza aplicada
```

con horizonte suficiente para cubrir:

```text
delay normal
jitter
margen
```

No almacenar historial ilimitado.

La integración debe usar:

```text
dt reales
```

No asumir 20 ms exactos.

---

# 14. Importante — usar `R_dynamic(t)`

La aceleración world/local depende de la orientación.

Por tanto, durante:

```text
t_k -> now
```

no usar una orientación congelada.

Usar la orientación proveniente del predictor angular dinámico ya validado:

```text
R_dynamic(t)
```

para transformar la fuerza body a O.

---

# 15. Prueba 309 — validar sólo dinámica translacional con estado inicial GT

Ejecutar después de 307/308.

Usar estado inicial retardado:

```text
p_initial = p_GT(t_k)
v_initial = v_GT(t_k)
R_initial / angular evolution = predictor dinámico validado
```

Desde:

```text
t_k -> now
```

NO usar:

```text
p_GT(now)
v_GT(now)
```

para construir salida.

Usar únicamente:

```text
fuerza aplicada
masa real
gravedad
R_dynamic(t)
timestamps
```

para obtener:

```text
p_dynamic(now)
v_dynamic(now)
```

## Control para aislar translación

Preferencia:

```text
p_control = p_dynamic(now)
v_control = v_dynamic(now)

R_control = R_GT(now)
omega_control = omega_GT(now)
```

Así se valida exclusivamente el canal translacional dinámico.

GT actual sólo queda en angular para aislar la prueba.

---

# 16. Criterio de 309

Si falla:

```text
STOP
```

No usar todavía `v(t_k)` estimada desde poses.

Revisar:

```text
masa
fuerza
frame
gravedad
timestamp
integración
orientación usada
```

Si completa:

```text
PREDICTOR DINÁMICO TRANSLACIONAL BÁSICO VALIDADO
```

Repetir una segunda vez.

---

# 17. Prueba 310 — estado inicial translacional estimado desde poses retardadas

Sólo si 309 funciona.

Usar:

```text
p(t_k) de la medida
v_hat(t_k) estimada causalmente desde posiciones
```

más:

```text
fuerza aplicada
masa
gravedad
R_dynamic(t)
```

para obtener:

```text
p_dynamic(now)
v_dynamic(now)
```

Mantener todavía angular GT actual para aislar la traslación.

## Objetivo

Validar:

```text
poses lentas/retardadas
    ->
v(t_k)
    ->
dinámica
    ->
p(now), v(now)
```

---

# 18. Estimación inicial de `v(t_k)`

No asumir que la derivada lineal actual ya es correcta.

Auditar cómo se obtiene hoy:

```text
linear_velocity_
```

y si representa realmente:

```text
v(t_k)
```

o una media retrasada.

Si el mismo problema temporal que sufrió `omega_motion` existe en velocidad lineal, aplicar el mismo principio:

```text
últimas 3 posiciones
    ->
dos velocidades medias
    ->
aceleración estimada
    ->
proyección causal hasta t_k
```

Pero NO implementar esto antes de revisar el código actual y las métricas de 307/308.

---

# 19. Prueba 311 — estado translacional dinámico + angular dinámica

Sólo si 310 funciona.

Usar:

```text
p_control = p_dynamic(now)
v_control = v_dynamic(now)

R_control = R_dynamic(now)
omega_control = omega_dynamic(now)
```

con el timing/jitter realista de 299.

GT sólo para métricas.

## Objetivo

Demostrar que el estado completo:

```text
p
v
R
omega
```

puede reconstruirse sin GT bajo timing/jitter realista.

---

# 20. Repetición

Si 311 completa:

```text
repetir como 312
```

Si:

```text
311 = CONSEGUIDA
312 = CONSEGUIDA
```

considerar:

```text
estado completo bajo timing/jitter realista VALIDADO
```

Sólo entonces reanudar:

```text
301/302 o renumeración equivalente
```

con ORB real.

---

# 21. GTests obligatorios

Añadir como mínimo:

## A — thrust hover ideal

Con:

```text
T = m*g
R nivelada
```

esperar:

```text
a ≈ 0
```

---

## B — thrust cero

Esperar:

```text
a = gravedad
```

---

## C — thrust mayor que peso

Esperar aceleración vertical positiva según convención.

---

## D — inclinación conocida

Con R inclinada:

```text
la fuerza body Z genera componente horizontal correcta
```

---

## E — dt irregular

Integración usa timestamps reales.

---

## F — cambio de fuerza

Buffer aplica cada fuerza en su intervalo correcto.

---

## G — coherencia p/v

Verificar:

```text
p_next
v_next
```

corresponden al mismo instante.

---

## H — masa compartida

Verificar que predictor y controlador consumen la misma masa configurada.

---

## I — frame

No mezclar:

```text
body thrust
O velocity
world/local gravity
```

silenciosamente.

---

# 22. Métricas obligatorias para 309-312

Registrar:

```text
p_initial
v_initial

force/thrust timestamp
force_used

mass
gravity

R_dynamic

a_dynamic
v_dynamic
p_dynamic

p_GT_now
v_GT_now

position_error
velocity_error

integration_dt
integration_steps

force_buffer_age
force_samples_used
missing_force_interval
```

Y calcular:

```text
RMSE p
MAE p

RMSE v
MAE v

lag p
lag v

max error p
max error v

ep
ev
F_des

scenario success
govern time
fallback
tracking
```

---

# 23. Qué NO hacer

No tocar:

```text
Kp
Kv
Kr
Kw
```

El problema ya se reproduce con estado erróneo y desaparece con estado correcto.

No compensar errores de estimación cambiando gains.

No modificar:

```text
predictor angular
J
SMALL/MODERATE
KF
W
mux
ORB-SLAM3 core
```

---

# 24. Limitación secundaria 304

La prueba 304 falló pese a:

```text
GT(t_k)
bracket válido
torque cubierto
```

pero esto queda como limitación secundaria.

No usar 304 para invalidar el predictor angular productivo porque:

```text
303 completa el hover completo
```

con la ruta angular que realmente se quiere conservar.

Documentar 304 como deuda diagnóstica separada.

No mezclar su investigación con el bloque translacional salvo que aparezca evidencia directa.

---

# 25. Restricción sobre GT

GT en estas pruebas es exclusivamente:

```text
truth de laboratorio
aislamiento de variables
```

La arquitectura final debe ser:

```text
ORB
+
estimador causal
+
modelo dinámico angular
+
modelo dinámico translacional
+
comandos propios del dron
```

sin GT durante operación normal.

---

# 26. Qué debe devolver Codex

Al terminar:

```text
Resultado:
CONSEGUIDO / PARCIAL / NO CONSEGUIDO
```

Incluir:

```text
- estado/commit usado;
- archivos modificados;

- resultado 307;
- resultado 308;
- diagnóstico:
    V PRINCIPAL
    P PRINCIPAL
    P Y V
    o
    INCOHERENCIA CONJUNTA;

- auditoría de fuerza;
- señal exacta elegida;
- frame;
- saturaciones si existen;

- masa real del modelo;
- masa usada por controlador/predictor;

- diseño del buffer;

- prueba 309;
- repetición si funciona;

- prueba 310 si corresponde;
- prueba 311 si corresponde;
- prueba 312 si corresponde;

- RMSE/MAE/lag de p/v;
- ep/ev;
- hover success;
- fallback/tracking;

- conclusión:
    PREDICTOR TRANSLACIONAL VALIDADO / NO VALIDADO
    ESTADO COMPLETO CON JITTER VALIDADO / NO VALIDADO

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

No borrar historial previo.

---

# 27. Resumen ejecutivo

Las pruebas 303-306 demuestran:

```text
angular dinámica + p/v GT
-> funciona

angular GT + p/v predichas
-> falla

estado GT completo
-> funciona
```

Por tanto:

> El problema principal restante está en el canal translacional.

Primero aislar:

```text
307:
p GT + v pred

308:
p pred + v GT
```

Después, si se confirma:

```text
p(t_k), v(t_k)
+
fuerza aplicada
+
masa real
+
gravedad
+
R_dynamic(t)
    ->
p(now), v(now)
```

Validar progresivamente:

```text
309:
estado inicial GT(t_k) + dinámica

310:
estado inicial estimado + dinámica

311:
estado completo dinámico con jitter

312:
repetición
```

> No avanzar a ORB real hasta que el estado completo bajo timing/jitter realista complete el hover de forma reproducible.
