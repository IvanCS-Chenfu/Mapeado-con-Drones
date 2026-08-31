# Fase 5H — Validación progresiva de movimiento con ORB real y trayectoria representativa
## Objetivo: demostrar que el estado ORB dinámico ya validado en hover permite seguir movimientos y, finalmente, la trayectoria representativa sin GT en control normal

## 0. Fuente de verdad

Trabajar sobre el estado ACTUAL del repositorio:

```text
https://github.com/IvanCS-Chenfu/Mapeado-con-Drones
```

Tomar como referencia directa:

```text
historial_5H_RESUMEN.md
prueba_326.log
prueba_327.log
prueba_328.log
prueba_329.log
prueba_330.log
prueba_331.log
```

Estado ya demostrado y que NO debe reabrirse sin evidencia nueva:

```text
A_HAT_AMPLIFICATION                 CORREGIDA
MIDPOINT_DYNAMIC                    VALIDADO
LINEAR_VELOCITY_ESTIMATOR           VALIDADO
GRAVITY_FRAME                       VALIDADO
DYNAMIC_PROPAGATION                 VALIDADA
buffers / ZOH / poda                VALIDADOS
integración productiva              VALIDADA
hover ORB real                      VALIDADO
hover ORB real reproducible         VALIDADO
```

Resultados vigentes:

```text
326/327:
shadow, cobertura 100 %
MIDPOINT_DYNAMIC empata con TWO_SAMPLE
y mejora claramente a THREE_SAMPLE

328/329:
MIDPOINT_DYNAMIC productivo validado en shadow
RMSE v ≈ 0.02113 / 0.02460 m/s

330/331:
hover ORB real reproducible
34.78 / 35.30 s
tracking OK
fallback = 0
clamp = 0
max error angular ≈ 0.0674 / 0.0631 rad
energía angular total negativa
```

Fase 5H permanece:

```text
PARCIAL
```

únicamente porque falta validar:

```text
movimiento X/Y/Z
yaw
movimiento combinado
trayectoria representativa
```

---

# 1. Regla principal de esta iteración

NO modificar antes de probar:

```text
MIDPOINT_DYNAMIC
v_hat
omega_hat

predictor angular
predictor translacional

g_O
J
masa

buffers
ZOH
poda

Kp
Kv
Kr
Kw

SMALL
MODERATE
reference KF policy
W
mux productivo
```

El objetivo ahora no es optimizar.

El objetivo es:

> comprobar si la arquitectura ya validada en hover funciona cuando el dron acelera, se desplaza, frena, gira y encadena movimientos.

Sólo modificar comportamiento si una prueba produce evidencia causal clara.

---

# 2. GT durante estas pruebas

Durante el tramo que se esté validando con ORB:

```text
p_control     = ORB
v_control     = ORB
R_control     = ORB
omega_control = ORB
```

GT NO puede entrar como estado normal de control.

GT sólo puede utilizarse para:

```text
métricas externas
diagnóstico
GT_FALLBACK temporal si ORB pierde tracking realmente
```

Mantener la frontera limpia ya validada:

```text
aproximación inicial diagnóstica
ORB shadow
anchor
airborne
settled
activar ORB
confirmar source=ORB
nuevo goal
```

Después del cambio:

```text
ORB gobierna el movimiento que se está probando.
```

---

# 3. Criterio general de validez de una prueba

Una prueba sólo se considera funcionalmente válida si:

```text
orb_navigation_prediction_mode = dynamic

MIDPOINT_DYNAMIC productivo activo

anchor válido antes de ORB

source=ORB confirmado antes del goal evaluado

tracking inicialmente OK

torque coverage = FULL
thrust coverage = FULL

F5H-DYNAMIC-MISSING = 0

sin NaN
sin fallo de infraestructura
```

Si falla algo de infraestructura:

```text
PRUEBA INVÁLIDA
```

Corregir sólo infraestructura y repetir con sufijo:

```text
R
```

No sacar conclusiones funcionales de una prueba inválida.

---

# 4. Filosofía de la batería

No saltar directamente a la trayectoria completa.

Validar progresivamente:

```text
hover validado
    ↓
X
    ↓
Y
    ↓
Z
    ↓
yaw
    ↓
combinación sencilla
    ↓
trayectoria representativa
```

Cada movimiento importante debe quedar:

```text
CONSEGUIDO
+
REPRODUCIDO
```

antes de avanzar.

---

# 5. Prueba 332 — desplazamiento X corto con ORB real

Crear/reutilizar un escenario simple:

```text
hover inicial
→ desplazamiento corto en X
→ frenado
→ hover final
```

Preferencia:

> reutilizar una amplitud/velocidad ya existente en escenarios diagnósticos previos de movimiento simple, si existe.

Si no existe una magnitud ya documentada:

```text
usar un desplazamiento conservador y claramente medible
```

y documentar explícitamente:

```text
distancia
velocidad nominal
aceleración nominal
duración
```

No elegir valores extremos.

No combinar todavía:

```text
yaw significativo
Y
Z
```

Objetivo:

> comprobar aceleración, velocidad sostenida, frenado y vuelta a `v_ORB ≈ 0` en un único eje.

---

# 6. Prueba 333 — repetición X

Sólo si 332 es funcionalmente correcta.

Repetir exactamente:

```text
mismo YAML
mismos parámetros
mismo estimador
misma trayectoria
```

Sin recalibrar.

Criterio:

```text
332 = CONSEGUIDA
333 = CONSEGUIDA
```

para declarar:

```text
MOVIMIENTO X ORB = VALIDADO
```

---

# 7. Prueba 334 — desplazamiento Y corto

Misma estructura:

```text
hover
→ Y corto
→ frenado
→ hover
```

No introducir yaw complejo.

Objetivo adicional:

> comprobar que no existe una asimetría residual de frames/ejes.

Esto es especialmente importante porque anteriormente existió un error real de extrínseca que permutaba ejes.

No asumir que porque X funciona Y funcionará.

---

# 8. Prueba 335 — repetición Y

Si 334 funciona:

```text
335
```

idéntica.

Criterio:

```text
334 + 335
```

para:

```text
MOVIMIENTO Y ORB = VALIDADO
```

---

# 9. Prueba 336 — desplazamiento Z corto

Secuencia:

```text
hover
→ pequeño cambio de altura
→ frenado vertical
→ hover
```

Objetivo:

> validar explícitamente la combinación de thrust, masa y `g_O` durante una maniobra vertical real.

Esta prueba es importante después de la corrección:

```text
g_O = O_R_W * g_W
```

No cambiar `g_O` ni recalcularla durante el movimiento.

---

# 10. Prueba 337 — repetición Z

Si 336 funciona:

```text
337
```

idéntica.

Criterio:

```text
MOVIMIENTO Z ORB = VALIDADO
```

---

# 11. Prueba 338 — yaw lento

Partir de hover.

Ejecutar:

```text
yaw lento
```

sin desplazamiento translacional intencionado grande.

Usar un giro que ya haya sido considerado seguro/diagnóstico en pruebas anteriores si existe.

Objetivo:

```text
validar R
omega
torque + J
seguimiento de yaw
y desacoplamiento traslacional
```

Durante el giro observar especialmente:

```text
posición
velocidad lineal
tracking
reference KF churn
```

para confirmar que yaw no reintroduce deriva translacional.

---

# 12. Prueba 339 — repetición yaw

Si 338 funciona:

```text
339
```

idéntica.

Criterio:

```text
YAW ORB LENTO = VALIDADO
```

---

# 13. Prueba 340 — movimiento combinado sencillo

Sólo si:

```text
X
Y
Z
yaw
```

han pasado individualmente.

Crear una combinación sencilla que obligue a usar simultáneamente más de un canal, por ejemplo:

```text
desplazamiento horizontal sencillo
+
cambio moderado de yaw
```

o una curva sencilla ya presente en infraestructura existente.

No introducir todavía:

```text
recorrido completo alrededor del edificio
multi-dron
yaw rápido extremo
```

Objetivo:

> comprobar que las piezas validadas individualmente siguen siendo coherentes cuando actúan a la vez.

---

# 14. Prueba 341 — repetición combinada

Si 340 funciona:

```text
341
```

idéntica.

Criterio:

```text
MOVIMIENTO COMBINADO ORB = VALIDADO
```

---

# 15. Criterio funcional para CADA movimiento

No aceptar únicamente:

```text
scenario_runner success=true
```

Para cada prueba medir explícitamente:

```text
posición inicial
posición objetivo
posición final

error máximo
RMSE de posición

velocidad máxima
velocidad al finalizar

tiempo de settling final

error angular máximo
omega máxima

ep
ev
er
ew

tracking
fallback
missing
reference KF changes

energía angular
```

Codex debe clasificar:

```text
MOVIMIENTO FUNCIONALMENTE ESTABLE
```

o:

```text
MOVIMIENTO DIVERGENTE / NO CONSEGUIDO
```

---

# 16. Punto crítico — frenado final

En X/Y/Z y movimiento combinado analizar especialmente:

```text
fase de aceleración
fase de velocidad
fase de frenado
hover final
```

Queremos comprobar:

```text
v_ORB → 0
```

cuando termina el movimiento.

Registrar alrededor del frenado:

```text
v_mid
v_hat_tk MIDPOINT_DYNAMIC
v_dynamic_now
v_GT
F_des
ep
ev
```

Ventanas sugeridas:

```text
1 s antes del frenado
durante frenado
0-1 s después
1-3 s después
```

El objetivo es detectar:

```text
overshoot
velocidad residual
oscilación
lag
```

sin introducir filtros nuevos.

---

# 17. MIDPOINT_DYNAMIC bajo aceleración real

Estas pruebas son la primera validación funcional fuerte de:

```text
MIDPOINT_DYNAMIC
```

con el dron realmente gobernado por ORB mientras:

```text
dv/dt != 0
```

Registrar por muestra:

```text
v_mid @ t_mid
midpoint_dynamic_horizon

thrust coverage
R_dynamic
g_O

v_hat_tk
v_dynamic_now

v_GT_tk
v_GT_now
```

GT sólo para métricas.

Comparar:

```text
RMSE v_hat_tk
RMSE v_dynamic_now
```

por fases:

```text
aceleración
velocidad aproximadamente constante
frenado
hover
```

---

# 18. Cambios de reference KF

Registrar:

```text
reference_kf
reference_changed
```

y estudiar las ventanas cercanas a cambios.

No culpar un cambio de KF si:

```text
O permanece continuo
y el error no aumenta.
```

Sólo reabrir ese frente si existe evidencia de:

```text
salto de p/v/R/omega
```

sin movimiento físico equivalente.

---

# 19. Revisiones globales / W

Registrar si ocurren:

```text
pose_revision
global correction
loop/optimization
```

pero mantener la regla:

```text
W puede cambiar
O no debe saltar
g_O del epoch no cambia
```

Si una revisión global coincide con un cambio artificial en:

```text
p_O
v_O
R_O
omega_O
```

marcarlo como bug arquitectónico.

Si no, no atribuirle problemas de seguimiento.

---

# 20. Tracking y fallback

Distinguir siempre:

## Caso esperado

```text
movimiento estable
↓
zona pobre en textura
↓
tracking se pierde primero
↓
GT_FALLBACK temporal
```

Esto puede ser válido en Fase 5.

## Caso no aceptable

```text
ORB empieza a divergir
↓
control mueve mal el dron
↓
imagen empeora
↓
tracking se pierde después
```

Aquí la pérdida visual es consecuencia.

Registrar la cronología.

---

# 21. STOP progresivo

Aplicar la siguiente regla:

Si una prueba individual:

```text
332
334
336
338
340
```

falla funcionalmente:

```text
STOP
```

No ejecutar su repetición ni las pruebas posteriores.

Analizar primero el fallo.

Si la primera ejecución funciona pero la repetición falla:

```text
resultado = NO REPRODUCIBLE
STOP
```

No avanzar.

---

# 22. Si falla X/Y/Z

No tocar gains automáticamente.

Comparar:

```text
p_ORB vs GT
v_ORB vs GT

ep
ev
F_des

v_hat_tk
v_dynamic_now
```

Determinar si el problema nace en:

```text
estimación
trayectoria
control
frame
```

antes de modificar.

---

# 23. Si falla yaw

No tocar translación automáticamente.

Analizar:

```text
R_ORB
omega_ORB
R_des
Omega_des

er
ew

tau_er
tau_ew

tracking
reference KF
```

y comprobar si existe:

```text
inestabilidad angular
```

o:

```text
pérdida visual genuina.
```

---

# 24. Si falla sólo movimiento combinado

Si:

```text
X/Y/Z/yaw pasan
```

pero:

```text
340 falla
```

el problema está en la interacción.

Analizar sincronización de:

```text
p/v
R/omega
trayectoria
```

en el mismo timestamp.

No reabrir individualmente una pieza ya validada sin evidencia.

---

# 25. Tras 332-341 — trayectoria representativa de Fase 5

Sólo si:

```text
332/333 X       OK
334/335 Y       OK
336/337 Z       OK
338/339 yaw     OK
340/341 combo   OK
```

ejecutar la trayectoria representativa acordada para Fase 5.

Preferencia:

```text
reutilizar el YAML representativo ya documentado en Fase 5
```

y NO inventar una misión nueva si ya existe una trayectoria de referencia.

La prueba debe usar:

```text
ORB como fuente normal
```

durante la misión.

GT sólo:

```text
métricas
fallback por pérdida real
```

---

# 26. Numeración de trayectoria representativa

Nombre sugerido:

```text
342
```

Si completa funcionalmente:

```text
343
```

como repetición.

No ejecutar multi-dron todavía si la trayectoria representativa inicial es de un único dron y la validación progresiva no lo exige.

Si el contrato vigente de Fase 5 exige ya dos drones para la prueba final, hacerlo sólo después de cerrar primero la versión de un dron y documentar la transición.

---

# 27. Qué medir en 342/343

Además de todas las métricas anteriores:

```text
número de goals
goals completados

tiempo total ORB
ratio ORB/GT fallback

fallback count
fallback duration
fallback cause

tracking losses
epoch changes
reference KF changes

global revisions

error p/v/R/omega

error de seguimiento de trayectoria

máxima desviación
RMSE

settling tras cada goal

missing torque/thrust

energía angular
```

Y responder:

> ¿El dron puede realizar la trayectoria completa usando ORB como estado normal, sin caídas provocadas por control y sin fallback frecuente salvo pérdidas visuales genuinas?

---

# 28. Criterio sobre fallback en prueba final

No exigir necesariamente:

```text
fallback = 0
```

si existe una pérdida visual genuina por baja textura.

Pero sí exigir:

```text
fallback no frecuente
fallback no provocado por oscilación/caída
handoff limpio
recuperación según contrato actual de Fase 5
```

Fase 5 permite todavía GT temporal cuando ORB se pierde realmente.

La solución final posterior deberá eliminar esa dependencia en Fase 6.

---

# 29. Qué NO hacer en este bloque

No:

```text
recalibrar gains entre pruebas
retocar thresholds para ganar una prueba
añadir filtros sin diagnóstico
cambiar MIDPOINT_DYNAMIC
cambiar g_O
cambiar J/masa
```

No esconder fallos mediante:

```text
mayor tolerancia de scenario_runner
```

No declarar éxito si:

```text
el runner termina
pero el dron no sigue realmente el movimiento.
```

---

# 30. Builds y validaciones

Antes de comenzar:

```text
build orbslam3
build dron_individual
build simulacion_dron

GTests/CTest
analizador
validaciones Python/YAML
git diff --check
```

Mantener al menos:

```text
116/116 GTests
7/7 analizador
```

o el número superior vigente al empezar.

Entre pruebas, si no se modifica código:

```text
no rebuild innecesario
```

pero conservar trazabilidad de configuración.

---

# 31. Telemetría mínima común

Mantener activa:

```text
debug_orb_control_state=true
phase5_pose_metrics_enabled=true
orb_navigation_prediction_mode=dynamic
```

Registrar:

```text
source
tracking
epoch
reference KF

p_ORB
v_ORB
R_ORB
omega_ORB

p_GT
v_GT
R_GT
omega_GT

ep
ev
er
ew

F_des
tau

v_mid
v_hat_tk
v_dynamic_now

dynamic horizon

thrust coverage
torque coverage

g_O
gravity_valid

fallback
fallback cause
```

---

# 32. Tabla resumen obligatoria

Codex debe devolver al final una tabla similar a:

```text
Prueba   Maniobra          Resultado   Reproducible   Tracking   Fallback
---------------------------------------------------------------------------
332      X corto           ?           -
333      X repetición      ?           ?
334      Y corto           ?           -
335      Y repetición      ?           ?
336      Z corto           ?           -
337      Z repetición      ?           ?
338      yaw lento         ?           -
339      yaw repetición    ?           ?
340      combinación       ?           -
341      repetición        ?           ?
342      trayectoria       ?           -
343      repetición        ?           ?
```

Añadir:

```text
max error posición
RMSE posición
max error velocidad
max error angular
tiempo ORB
```

si resulta legible.

---

# 33. Criterio para cerrar 5H

No marcar 5H como `CONSEGUIDA` únicamente por completar X/Y/Z/yaw.

La subfase podrá proponerse como conseguida cuando exista evidencia de:

```text
hover ORB reproducible
+
movimientos elementales reproducibles
+
movimiento combinado reproducible
+
trayectoria representativa completada
```

y que cualquier fallback observado corresponda a:

```text
pérdida visual genuina
```

no a inestabilidad del control.

Si el contrato/documentación vigente exige alguna validación adicional, respetarla.

---

# 34. Qué debe devolver Codex

Al terminar el bloque:

```text
Resultado:
CONSEGUIDO / PARCIAL / NO CONSEGUIDO
```

Incluir:

```text
- commit/estado inicial;
- archivos modificados;
- YAMLs creados/reutilizados;

- confirmación de que no cambió el estimador durante la batería;

- builds;
- GTests;
- analizador;
- git diff --check;

- resultados 332-341;
- repetibilidad;

- métricas por maniobra;
- tracking;
- fallback;
- missing;

- análisis de aceleración/frenado;
- comportamiento de v_ORB al volver a hover;

- cambios de reference KF;
- revisiones W si aparecen;

- resultado 342 si se alcanza;
- resultado 343 si corresponde;

- clasificación:
    X VALIDADO / NO VALIDADO
    Y VALIDADO / NO VALIDADO
    Z VALIDADO / NO VALIDADO
    YAW VALIDADO / NO VALIDADO
    MOVIMIENTO COMBINADO VALIDADO / NO VALIDADO
    TRAYECTORIA REPRESENTATIVA VALIDADA / NO VALIDADA

- decisión final:
    FASE 5H CONSEGUIDA
    o
    FASE 5H PARCIAL

- si queda PARCIAL:
    causa exacta siguiente, sin modificarla sin nueva autorización.
```

Actualizar:

```text
historial_5H_RESUMEN.md
07_ULTIMA_SESION.md
contrato 5H
```

sin borrar historial anterior.

---

# 35. Resumen ejecutivo

El hover ORB ya está validado y reproducido:

```text
330/331
~35 s ORB
tracking OK
fallback 0
clamp 0
energía angular negativa
```

La siguiente pregunta ya no es:

```text
"¿se mantiene quieto?"
```

sino:

```text
"¿puede seguir movimientos reales y frenarlos correctamente?"
```

Ejecutar progresivamente:

```text
332/333  X
334/335  Y
336/337  Z
338/339  yaw
340/341  combinación
```

Sólo si todo eso funciona:

```text
342/343
trayectoria representativa
```

> No volver a modificar el estimador salvo que una de estas pruebas aporte evidencia causal nueva. La arquitectura actual debe tratarse como congelada durante esta batería de validación funcional.
