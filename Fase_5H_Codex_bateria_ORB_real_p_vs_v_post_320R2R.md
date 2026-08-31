# Fase 5H — Batería ORB real para aislar `p` vs `v` tras handoff limpio

## Objetivo

Determinar si el hover ORB real falla principalmente por `v_hat(t_k)`, por la posición ORB, por ambas o por el canal angular, sin modificar todavía el estimador.

## Fuente de verdad

Trabajar sobre el estado ACTUAL de:

```text
https://github.com/IvanCS-Chenfu/Mapeado-con-Drones
```

Usar como base:

```text
prueba_320R2.log
prueba_320R2R.log
historial_5H_RESUMEN.md
```

y mantener toda la arquitectura productiva ya validada:

```text
v_hat(t_k) causal
omega_hat(t_k) causal
BodyThrustDynamicPredictor
predictor dinámico angular
buffers torque/thrust
seed causal + ZOH + poda con predecesora
J real
masa real
```

## Estado actual

`318R2` y `319R` validan la integración productiva de laboratorio sin missing ni fallback.

La primera `320` fue inválida por ejecutar `legacy`. `320R` usa ORB dinámico real, pero ORB tomó autoridad demasiado pronto durante la aproximación.

`320R2R` corrige esa frontera: aproximación con GT, ORB en shadow, anchor válido, airborne, settled y activación ORB posterior. Después del handoff ORB mantiene tracking y no aparecen fallback ni huecos de torque/thrust.

Sin embargo, el hover funcional sigue siendo malo. `scenario_runner success=true` no debe confundirse con hover estable.

La evidencia principal del handoff es aproximadamente:

```text
position jump      ~ 0
rotation jump      ~ 0
omega jump         pequeño
velocity jump      ~ 0.25 m/s
```

Además, durante shadow la pose/orientación ORB son razonables, mientras el error de velocidad lineal es claramente mayor.

Hipótesis principal:

> `v_hat(t_k)` obtenida desde posiciones ORB reales puede estar entregando una condición inicial lineal demasiado ruidosa/inexacta.

No dar todavía esta hipótesis por demostrada.

---

# 1. Antes de la batería: eliminar la carrera authority/goal

En `320R2R` puede existir:

```text
servicio autoriza ORB
↓
se envía goal hover
↓
goal todavía puede aceptarse con source=GT
↓ pocos ms
mux pasa realmente a ORB
```

Corregir únicamente esta carrera de escenario.

El nuevo goal de hover NO debe enviarse hasta observar explícitamente:

```text
navigation_state_mux source == ORB
```

Añadir marcador:

```text
[F5H-ORB-AUTHORITY-CONFIRMED]
stamp=
source=orb
```

Orden obligatorio:

```text
activation_request
<
authority_confirmed
<
goal_sent
<=
goal_accepted
```

No modificar el estimador para esto.

---

# 2. Condiciones comunes

Todas las pruebas deben repetir exactamente:

```text
startup
espera tracking
aproximación gobernada por GT
ORB real en shadow
espera anchor
fin de aproximación
airborne
settled
ventana breve de asentamiento
activar ORB
confirmar source=ORB
enviar NUEVO goal hover
```

No cambiar:

```text
YAML
gains
J
masa
v_hat
omega_hat
predictor angular
predictor translacional
SMALL/MODERATE
KF logic
buffers
```

GT sólo se usará para aislar variables.

---

# 3. Prueba 321A — `p_GT + v_ORB + angular ORB`

Después del handoff:

```text
p_control     = p_GT(now)
v_control     = v_ORB_dynamic(now)
R_control     = R_ORB_dynamic(now)
omega_control = omega_ORB_dynamic(now)
```

Pregunta:

> ¿La velocidad ORB por sí sola basta para desestabilizar el hover?

Si falla:

```text
v_ORB / v_hat(t_k) = sospechoso principal
```

---

# 4. Prueba 321B — `p_ORB + v_GT + angular ORB`

Configurar:

```text
p_control     = p_ORB_dynamic(now)
v_control     = v_GT(now)
R_control     = R_ORB_dynamic(now)
omega_control = omega_ORB_dynamic(now)
```

Pregunta:

> ¿Con velocidad perfecta la posición ORB permite hover estable?

Si funciona y 321A falla:

> La velocidad ORB queda prácticamente aislada como causa principal.

Si también falla:

> La posición ORB también es crítica.

---

# 5. Prueba 321C — `p/v GT + angular ORB`

Configurar:

```text
p_control     = p_GT(now)
v_control     = v_GT(now)
R_control     = R_ORB_dynamic(now)
omega_control = omega_ORB_dynamic(now)
```

Objetivo:

> Validar el canal angular ORB REAL dentro del mismo escenario.

Si completa:

```text
angular ORB real = funcionalmente suficiente
```

Si falla:

```text
no atribuir todo al canal translacional
```

---

# 6. Prueba 321D — baseline ORB completo

Sólo después de 321A-C:

```text
p_control     = p_ORB_dynamic(now)
v_control     = v_ORB_dynamic(now)
R_control     = R_ORB_dynamic(now)
omega_control = omega_ORB_dynamic(now)
```

La única diferencia respecto a 320R2R debe ser que:

```text
source=ORB
```

queda confirmado ANTES de mandar el goal.

Objetivo:

> Medir si la carrera activation/goal tenía algún peso causal.

---

# 7. Orden obligatorio

```text
321A
321B
321C
321D
```

No recalibrar entre pruebas.

Si una ejecución falla por infraestructura:

```text
marcar INVÁLIDA
corregir sólo infraestructura
repetir con sufijo R
```

---

# 8. Criterios de validez

Una prueba sólo cuenta si:

```text
orb_navigation_prediction_mode=dynamic

aproximación=GT
ORB shadow activo

anchor=true
airborne=true
settled=true

source=ORB confirmado antes del goal

tracking=OK en handoff

torque coverage=FULL
thrust coverage=FULL

F5H-DYNAMIC-MISSING=0
```

Si no:

```text
PRUEBA INVÁLIDA
```

---

# 9. Telemetría pre-handoff

Durante los últimos segundos de shadow registrar:

```text
p_ORB / p_GT
v_ORB / v_GT
R_ORB / R_GT
omega_ORB / omega_GT

p_error
v_error
R_error
omega_error

linear_estimator_mode
angular_estimator_mode

visual_age
prediction_horizon
reference KF
tracking
raw class
```

Calcular:

```text
RMSE p
RMSE v
RMSE R
RMSE omega
mean/max error
```

---

# 10. Telemetría de handoff

Registrar:

```text
activation_request_stamp
authority_confirmed_stamp
goal_sent_stamp
goal_accepted_stamp
first_ORB_control_tick

p_jump
v_jump
R_jump
omega_jump
```

---

# 11. Telemetría post-handoff

Registrar:

```text
ep
ev
er
ew

F_des
tau_er
tau_ew
tau_total

p_used
v_used
R_used
omega_used

p_ORB
v_ORB
R_ORB
omega_ORB

p_GT
v_GT
R_GT
omega_GT

tracking
reference KF
raw class
SMALL/MODERATE/REJECTED
fallback
missing
```

Analizar por ventanas:

```text
0–0.1 s
0–0.5 s
0.5–1 s
1–5 s
5–10 s
resto del hover
```

En cada ventana:

```text
mean/max |ep|
mean/max |ev|
mean/max |er|
mean/max |ew|
RMSE p/v/R/omega
energía angular
```

---

# 12. Criterio funcional de hover

No aceptar simplemente:

```text
scenario_runner success=true
```

Reportar al menos:

```text
posición al inicio
posición al final
máxima desviación
RMSE posición
velocidad media/max
```

y clasificar:

```text
HOVER ESTABLE
o
HOVER DIVERGENTE
```

---

# 13. Árbol de diagnóstico

### Caso A — velocidad principal

```text
321A falla
321B funciona
321C funciona
```

Conclusión:

```text
V_ORB PRINCIPAL
```

El siguiente trabajo debe estudiar `v_hat(t_k)` con poses ORB reales.

### Caso B — posición principal

```text
321A funciona
321B falla
321C funciona
```

Conclusión:

```text
P_ORB PRINCIPAL
```

Revisar posición visual, continuidad O y cambios de referencia.

### Caso C — ambas

```text
321A falla
321B falla
321C funciona
```

Conclusión:

```text
P_Y_V_ORB
```

El canal translacional ORB real es el bloqueo.

### Caso D — angular también

```text
321C falla
```

Conclusión:

```text
ANGULAR_TAMBIÉN
```

No modificar translación todavía sin analizar R/omega ORB real.

### Caso E — 321D funciona

Si el baseline completo funciona tras confirmar autoridad antes del goal:

```text
CARRERA_HANDOFF
```

tenía más peso del esperado. Repetir antes de cerrar conclusión.

---

# 14. Si se confirma `v_hat(t_k)`

NO implementar todavía una solución.

Primero registrar por muestra:

```text
p(k-2)
p(k-1)
p(k)

dt1
dt2

v_mid_1
v_mid_2

a_hat
v_hat_tk

v_GT_tk

reference KF
raw translation step
linear estimator mode
```

Buscar si el error nace por:

```text
ruido de posición ORB
correcciones SMALL
cambios de reference KF
dt irregular
sobreamplificación de a_hat
```

Sólo después diseñar la corrección.

---

# 15. No asumir culpa de reference KF

Registrar los cambios, pero determinar causalmente si el error empieza:

```text
antes
durante
o después
```

del cambio de KF.

Un cambio legítimo de reference KF no debería producir movimiento físico en O.

---

# 16. Tests automáticos

Añadir tests focales para asegurar:

```text
1. authority_confirmed precede goal_sent.
2. 321A sustituye únicamente p.
3. 321B sustituye únicamente v.
4. 321C sustituye únicamente p/v.
5. 321D no usa GT en el estado de control.
6. ORB sigue trabajando en shadow durante aproximación.
7. las sustituciones GT diagnósticas no contaminan el estimador ORB interno.
8. el launch mantiene prediction_mode=dynamic.
```

Mantener:

```text
build orbslam3
build dron_individual
build simulacion_dron
GTests/CTest
analizador
git diff --check
```

---

# 17. Qué NO hacer

No modificar:

```text
v_hat
omega_hat
J
masa
predictor dinámico angular
BodyThrustDynamicPredictor
ZOH
buffers
poda
Kp/Kv/Kr/Kw
SMALL/MODERATE
KF policy
W
```

No ejecutar una repetición ordinaria 321 como si 320R2R hubiera sido hover conseguido.

Esta batería sustituye temporalmente esa repetición.

---

# 18. Documentación

Actualizar:

```text
historial_5H_RESUMEN.md
07_ULTIMA_SESION.md
```

Registrar:

```text
320R2 inicial = INVÁLIDA si falló por infraestructura

320R2R = válida para diagnóstico de shadow/handoff,
pero NO considerar hover funcionalmente conseguido
si las métricas muestran divergencia.
```

---

# 19. Qué debe devolver Codex

```text
Resultado diagnóstico:
CONSEGUIDO / PARCIAL / NO CONSEGUIDO
```

Incluir:

```text
- estado/commit usado;
- archivos modificados;
- causa de la 320R2 inválida;
- resumen de 320R2R;
- corrección authority/goal;
- confirmación de que no se tocaron estimadores;

- builds/GTests/analyzer/git diff --check;

- resultado 321A;
- resultado 321B;
- resultado 321C;
- resultado 321D;

- tabla comparativa;

- RMSE p/v/R/omega pre-handoff;
- saltos de handoff;
- ep/ev/er/ew por ventanas;
- energía;
- tracking;
- reference KF;
- fallback;
- missing;

- conclusión explícita:
  V_ORB PRINCIPAL
  P_ORB PRINCIPAL
  P_Y_V_ORB
  ANGULAR_TAMBIÉN
  CARRERA_HANDOFF
  o
  DIAGNÓSTICO NO CONCLUYENTE;

- siguiente solución recomendada.
```

---

# 20. Resumen ejecutivo

`320R2R` ya corrige la activación prematura:

```text
aproximación GT
ORB shadow
anchor
airborne
settled
GT -> ORB
```

y ORB mantiene tracking y autoridad.

Pero el hover funcional sigue divergiendo y el handoff destaca:

```text
velocity jump ~0.25 m/s
```

con pose/orientación prácticamente continuas.

La siguiente batería debe aislar:

```text
321A:
p GT + v ORB + angular ORB

321B:
p ORB + v GT + angular ORB

321C:
p/v GT + angular ORB

321D:
ORB completo con authority confirmada antes del goal
```

> No volver a modificar el estimador hasta demostrar si el bloqueo con ORB real está principalmente en velocidad lineal, posición, ambas o también en el canal angular.
