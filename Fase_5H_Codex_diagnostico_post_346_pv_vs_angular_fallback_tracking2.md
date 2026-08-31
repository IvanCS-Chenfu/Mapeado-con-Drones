# Fase 5H — Diagnóstico causal post-346: localizar el primer canal que diverge y la causa real del fallback con `tracking=2`
## Objetivo: comparar 345 (GT gobierna + ORB shadow) contra 346 (ORB gobierna) sobre la misma ruta de dos fachadas y aislar si falla primero `p/v`, `R/omega` o la validez interna del estado

## 0. Fuente de verdad

Trabajar sobre el estado ACTUAL del repositorio:

```text
https://github.com/IvanCS-Chenfu/Mapeado-con-Drones
```

Tomar como evidencia principal:

```text
prueba_344.reduced.log
prueba_345.reduced.log
prueba_346.reduced.log
historial_5H_RESUMEN.md
```

y revisar el código actual, no una versión recordada.

Antes de modificar:

```text
git status
git rev-parse HEAD
git log -1 --oneline
```

El commit `769a596` fue subido antes de completar 344-346; los cambios posteriores pueden estar aún sin commit. Documentar con precisión el estado real antes de empezar.

---

# 1. Estado ya cerrado

La auditoría 344-345 ha demostrado un bug real y ya corregido:

```text
STALE_RAW_HISTORY
```

El problema NO era mezclar geométricamente referencias KF diferentes.

La entrada raw ya está expresada en:

```text
O
```

El defecto era:

```text
muestra raw rechazada
    ↓
baseline raw no avanza
    ↓
permanece obsoleto demasiado tiempo
    ↓
una muestra futura se compara contra una pose muy antigua
    ↓
raw_dt y raw_step absurdos
```

En 344 se observó aproximadamente:

```text
raw_dt max      = 20.609 s
raw_step        = 2.753 m
O_step max      = 0.088 m
```

La corrección implementada:

```text
si el histórico raw queda temporalmente inválido/obsoleto:
    REBASE exclusivo del histórico raw
```

sin:

```text
aceptar el delta inválido
resetear O
resetear pose física
resetear dinámica
resetear g_O
resetear buffers torque/thrust
```

En 345 la corrección queda validada:

```text
KEEP                    125 -> 5
SUSPICIOUS              168 -> 2
raw_dt max              20.609 -> 0.201 s
racha sin anchor        101 -> 2
```

Por tanto:

```text
STALE_RAW_HISTORY = CORREGIDO
```

No reabrirlo salvo evidencia nueva.

---

# 2. Qué demuestra 346

346 reutiliza la ruta corta de dos fachadas, pero deja que ORB gobierne.

La secuencia relevante del dron 2 es:

```text
ORB source
target = (10,-10,1.3)
    ↓
primer tramo completado

target = (10,0,1.3)
    ↓
segundo tramo
    ↓
GT_FALLBACK con tracking todavía 2
```

El log registra:

```text
source=gt_fallback
reason=tracking_lost
tracking=2
```

y poco después:

```text
reason=orb_qualifying
tracking=2
```

después:

```text
reason=trajectory_source_locked
tracking=2
```

Más tarde comienza otro goal:

```text
target=(10,10,1.3)
source=orb
tracking=2
```

y sólo bastante después aparece la pérdida visual real:

```text
tracking 2 -> 3
local_valid=false
continuity_valid=false
```

Por tanto, distinguir obligatoriamente:

```text
FALLO A:
fallback / no consumibilidad mientras tracking sigue 2

FALLO B:
pérdida visual real 2 -> 3 posterior
```

No tratar ambos como el mismo evento.

---

# 3. Conclusión de partida

La corrección de raw history era necesaria, pero NO es suficiente.

En 346:

```text
raw history permanece sano
```

y aun así:

```text
control ORB vuelve a degradarse
```

antes de la pérdida visual real.

La siguiente pregunta NO es:

```text
"¿qué threshold cambio?"
```

La pregunta es:

> ¿Qué condición exacta hace que el estado ORB deje de ser consumible, y qué componente de `p/v/R/omega` empieza a divergir primero respecto a la ejecución shadow 345?

---

# 4. Hipótesis actuales

NO asumir ninguna como cierta antes del diagnóstico.

Hipótesis candidatas:

```text
A. P/V divergen primero.

B. R/omega divergen primero.

C. p/v y R/omega permanecen razonables,
   pero un flag interno invalida NavigationState.

D. predictor_healthy / rechazo angular / probation
   vuelve el estado no consumible.

E. state_age / timing / DEGRADED_DT provoca la invalidez.

F. interaction control-estimator:
   el estado es inicialmente razonable,
   el control introduce movimiento físico que amplifica el error.

G. una causa distinta aún no instrumentada.
```

La evidencia previa hace especialmente relevante:

```text
R / omega
```

porque:

```text
hover ORB          validado
X/Y/Z cortos       funcionales
yaw 338            falló
dos fachadas 346   falla durante movimiento combinado/cambio de dirección
```

Pero esto sigue siendo una hipótesis.

---

# 5. Regla principal de esta iteración

NO modificar todavía:

```text
MIDPOINT_DYNAMIC
v_hat

omega estimator
omega bias/motion policy

predictor angular
predictor translacional

g_O
J
masa

raw rebase ya corregido

SMALL
MODERATE
REJECT thresholds

Kp
Kv
Kr
Kw

reference KF policy
W
mux behavior
```

Primero diagnóstico.

---

# 6. Auditoría obligatoria del fallback de 346

Buscar en el código actual del mux / wrapper / estimador la ruta exacta que produce:

```text
reason=tracking_lost
```

Responder de forma precisa:

```text
¿qué condición booleana activa ese reason?
```

Revisar como mínimo:

```text
tracking_state
local_valid
local_continuity_valid
velocity_valid
global_valid
predictor_healthy
sample freshness / state age
orb qualifying
reference validity
pending reference
consecutive rejects
angular estimator healthy
linear estimator valid
gravity_o_valid
dynamic coverage
```

Construir la expresión real, por ejemplo:

```text
if (!X || !Y || Z):
    fallback reason=tracking_lost
```

NO asumir que `reason=tracking_lost` significa literalmente:

```text
tracking_state != OK
```

Puede ser una etiqueta demasiado amplia.

---

# 7. Telemetría del mux que falta

Si el log actual no permite saber por qué hace fallback, añadir una línea diagnóstica:

```text
[F5H-FALLBACK-CAUSE-TRACE]

stamp=
source_before=
source_after=
reason=

tracking=
local_valid=
local_continuity_valid=
velocity_valid=
global_valid=

predictor_healthy=
linear_valid=
angular_valid=
gravity_valid=

state_age=
sample_fresh=
reference_valid=

trajectory_active=
source_locked=
qualifying=

exact_failed_predicate=
```

Muy importante:

```text
exact_failed_predicate
```

debe indicar la condición real.

Ejemplos:

```text
LOCAL_INVALID
CONTINUITY_INVALID
VELOCITY_INVALID
PREDICTOR_UNHEALTHY
STATE_STALE
TRACKING_NOT_OK
REFERENCE_INVALID
```

No esconder varias causas bajo `tracking_lost`.

No hace falta cambiar todavía la política de fallback.

---

# 8. Comparación principal 345 vs 346

345 y 346 recorren el mismo escenario geométrico.

La diferencia causal principal es:

```text
345:
GT gobierna
ORB observa en shadow

346:
ORB gobierna
```

Por tanto, usar 345 como baseline.

No comparar sólo máximos globales.

Sincronizar por:

```text
goal
segmento
posición/tiempo relativo dentro del goal
```

Especialmente el segundo tramo:

```text
(10,-10,1.3)
    ->
(10,0,1.3)
```

porque ahí aparece el primer fallback no explicado.

---

# 9. Buscar el PRIMER instante de divergencia

NO empezar el análisis en:

```text
fallback
```

ni en:

```text
tracking 2->3
```

Retroceder al menos:

```text
5 s
```

desde el primer evento donde 346 deja de parecerse a 345.

Si es necesario:

```text
10 s
```

Quiero una cronología:

```text
T0 = estado 345 y 346 todavía equivalentes

T1 = primera métrica que se separa claramente

T2 = el controlador empieza a aumentar error/torque

T3 = primer flag interno no consumible

T4 = fallback

T5 = tracking loss real, si ocurre
```

---

# 10. Variables obligatorias para comparar

## 10.1. Estado ORB vs GT

```text
p_ORB
p_GT
p_error vector
|p_error|

v_ORB
v_GT
v_error vector
|v_error|

R_ORB
R_GT
rotation_error vector
rotation_error_rad

omega_ORB
omega_GT
omega_error vector
|omega_error|
```

## 10.2. Estado visual/base/dinámico

Registrar, si existen:

```text
visual_pose
base_pose
dynamic_pose

v_mid
v_hat_tk
v_dynamic_now

omega_raw
omega_motion
omega_bias
omega_hat_tk
omega_dynamic_now

rotation_innovation
published_pose_rotation_step
```

Objetivo:

> saber en qué capa nace primero la separación.

## 10.3. Control

```text
ep
ev
er
ew

F_des

tau_er
tau_ew
tau_total
torque_norm

R_des
Omega_des
```

Si ya existe energía:

```text
trabajo tau_er
trabajo tau_ew
trabajo total
```

## 10.4. Calidad/validez

```text
tracking
local_valid
local_continuity_valid
velocity_valid
predictor_healthy

linear mode/source
angular mode

state_age
visual_age
dynamic_horizon

gravity_valid

torque coverage
thrust coverage
missing flags
```

## 10.5. ORB / raw / KF

```text
reference_kf
reference_changed
frames_since_reference_change

raw_history_action
raw_dt
raw_step_translation
raw_step_rotation
raw_class

classification
correction_class
base_update_type

pending
confirmed
discarded
rejected
```

---

# 11. Tabla temporal obligatoria alrededor del primer fallo

Crear una tabla como:

```text
t_rel | source | tracking | local | cont | vel_valid |
p_err | v_err | R_err | omega_err |
raw_class | correction | base_update |
predictor_healthy | state_age |
ep | ev | er | ew | torque |
fallback_reason
```

Resolución:

```text
~0.1 s
```

o por evento relevante.

No hace falta imprimir cada tick de 50 Hz si dificulta leerlo.

---

# 12. Clasificar qué canal falla primero

## Caso A — translación primero

Si:

```text
v_error / p_error
```

crecen claramente antes que:

```text
R_error / omega_error
```

y antes del aumento de torque angular:

Conclusión:

```text
TRANSLATIONAL_FIRST
```

Reabrir después:

```text
v_mid
MIDPOINT_DYNAMIC
thrust propagation
visual position quality
```

pero NO bajo esta autorización.

## Caso B — angular primero

Si:

```text
R_error
o
omega_error
```

crecen antes que `p/v`, y después:

```text
er/ew
tau
```

crecen:

Conclusión:

```text
ANGULAR_FIRST
```

Ésta es la hipótesis actualmente más sospechosa por 338 + 346.

## Caso C — flag interno primero

Si:

```text
p/v/R/omega
```

siguen razonables pero:

```text
local_valid=false
continuity_valid=false
velocity_valid=false
predictor_healthy=false
```

aparece antes:

Conclusión:

```text
VALIDITY_POLICY_FIRST
```

Auditar la política de invalidez.

## Caso D — control realimenta primero

Si el estado estimado empieza razonablemente, pero:

```text
torque/force
```

se separan y el movimiento real GT cambia antes de que ORB visual diverja:

Conclusión:

```text
CONTROL_FEEDBACK_FIRST
```

Analizar después coherencia:

```text
state timestamp
desired state
pose/velocity pairing
```

## Caso E — multicausal

Si dos canales empiezan casi simultáneamente:

```text
MULTICAUSAL
```

pero cuantificar cuál tiene mayor anticipación y pendiente.

---

# 13. Pregunta específica: `reason=tracking_lost` con `tracking=2`

Codex debe responder explícitamente:

```text
¿Por qué ocurre?
```

No aceptar:

```text
"porque el estimador se degradó"
```

Quiero:

```text
condición exacta
archivo
función
línea/lógica
valor de cada predicado
```

Si la etiqueta es semánticamente incorrecta:

```text
reason=tracking_lost
```

pero la causa real es, por ejemplo:

```text
velocity_valid=false
```

NO cambiar todavía la política, pero:

> corregir la telemetría/etiqueta puede autorizarse si es puramente observabilidad y no cambia comportamiento.

Preferencia:

```text
reason=local_invalid
reason=velocity_invalid
reason=predictor_unhealthy
...
```

según corresponda.

---

# 14. Prueba 348 — repetir 346 con telemetría causal ampliada

Usar exactamente:

```text
tray_prueba_346.yaml
```

o el mismo YAML funcional equivalente.

No cambiar:

```text
waypoints
duraciones
velocidades
gains
estimadores
```

Sólo añadir telemetría.

Nombre:

```text
348
```

Objetivo:

> reproducir el fallo con datos suficientes para localizar T1-T5.

Si 348 no reproduce:

```text
348R
```

una única repetición sin cambios.

No ejecutar muchas veces hasta que falle.

---

# 15. STOP después de 348/348R

No implementar solución todavía.

Primero devolver diagnóstico.

Sólo si el canal queda claramente aislado se decidirá la siguiente modificación.

---

# 16. Batería cruzada autorizada SÓLO si 348 no basta para aislar

Si la cronología sigue siendo ambigua, ejecutar una batería mínima sobre la misma ruta de dos fachadas.

No usar la vuelta completa.

## Prueba 349A — translación ORB + angular GT

```text
p_control     = p_ORB
v_control     = v_ORB

R_control     = R_GT
omega_control = omega_GT
```

ORB completo sigue funcionando internamente.

GT angular sólo se inyecta en el controlador como diagnóstico.

Pregunta:

> ¿La ruta de dos fachadas funciona si eliminamos el canal angular ORB del control?

## Prueba 349B — translación GT + angular ORB

```text
p_control     = p_GT
v_control     = v_GT

R_control     = R_ORB
omega_control = omega_ORB
```

Pregunta:

> ¿La ruta falla manteniendo sólo el canal angular ORB?

---

# 17. Interpretación 349A / 349B

## Si 349A funciona y 349B falla

```text
ANGULAR_CAUSAL = CONFIRMADO
```

Siguiente bloque:

```text
R/omega visual-base-dynamic
```

## Si 349A falla y 349B funciona

```text
TRANSLATIONAL_CAUSAL = CONFIRMADO
```

Siguiente bloque:

```text
p/v
```

## Si ambas funcionan

La causa puede ser:

```text
interacción entre canales
```

o:

```text
coherencia temporal conjunta
```

Conclusión:

```text
COUPLING / SYNCHRONIZATION
```

## Si ambas fallan

```text
MULTICAUSAL
```

o hay una política de validez independiente de la sustitución.

Revisar flags antes de tocar estimadores.

---

# 18. Condiciones de las pruebas cruzadas

Mantener:

```text
ORB tracking real
ORB estimator real completo
misma trayectoria
misma geometría
mismos KFs
mismos delays
mismos gains
```

GT sólo sustituye variables de entrada al controlador.

No permitir que GT:

```text
alimente ORB
corrija O
corrija mapas
corrija KFs
corrija raw
```

---

# 19. Qué NO hacer

No:

```text
bajar velocidad de la ruta
```

para ocultar el problema.

No:

```text
añadir filtros
subir thresholds
bajar gains
```

antes de aislar causalidad.

No revertir:

```text
STALE_RAW_HISTORY fix
```

No tocar:

```text
MIDPOINT_DYNAMIC
g_O
J
masa
```

No impedir:

```text
reference_kf changes
```

No declarar que el fallo es “ORB por pocos puntos” si:

```text
tracking sigue 2
```

cuando comienza la divergencia.

---

# 20. Relación con limitaciones normales de visión

Una limitación visual normal puede producir:

```text
menos puntos
más ruido
tracking 2 -> 3
```

Eso es aceptable en Fase 5 si:

```text
tracking loss ocurre primero
fallback ocurre después
```

No es aceptable como explicación para:

```text
estado/control diverge
fallback
tracking sigue 2
```

La cronología manda.

---

# 21. Relación con `reference_kf`

Después de 344-345:

```text
reference_kf
```

deja de ser el sospechoso principal.

Los cambios KF normales observados tras la corrección presentan:

```text
raw_dt ~0.05-0.15 s
raw_step coherente con O_step
```

Por tanto:

> seguir registrando Kref, pero NO reabrir su semántica salvo que T1 coincida sistemáticamente con un evento anómalo todavía no explicado.

---

# 22. Builds y tests

Si sólo se añade telemetría:

```text
build orbslam3
build dron_individual
build simulacion_dron

GTests/CTest
analizador
git diff --check
```

Mantener al menos:

```text
117/117 GTests
```

o el número superior vigente.

Añadir test focal si se modifica la codificación de `fallback_reason`:

```text
TRACKING_NOT_OK -> tracking_lost
LOCAL_INVALID   -> local_invalid
VELOCITY_INVALID -> velocity_invalid
...
```

sin cambiar la decisión de fallback.

---

# 23. Qué debe devolver Codex tras 348

La respuesta debe incluir:

```text
Resultado diagnóstico:
CONSEGUIDO / PARCIAL / NO CONSEGUIDO
```

y obligatoriamente:

```text
1. commit/estado inicial;
2. archivos auditados;
3. archivos modificados;

4. expresión exacta que produce el primer fallback;
5. explicación de reason=tracking_lost con tracking=2;

6. timeline T0-T5;

7. primer canal que diverge:
   P/V
   R/OMEGA
   VALIDITY
   CONTROL
   MULTICAUSAL
   NO CONCLUYENTE;

8. diferencias 345 vs 348 en:
   p/v/R/omega
   ep/ev/er/ew
   torque
   validity
   tracking
   raw/classification
   Kref;

9. comprobar si el raw rebase sigue sano;

10. resultado 349A/349B sólo si fueron necesarios;

11. siguiente modificación recomendada;

12. STOP antes de implementarla.
```

---

# 24. Métricas mínimas

Reportar para el segundo tramo y hasta fallback:

```text
RMSE p
RMSE v
RMSE R
RMSE omega

max p/v/R/omega error

momento del primer:
p_error > baseline
v_error > baseline
R_error > baseline
omega_error > baseline

momento de:
er/ew growth
torque growth
validity flag change
fallback
tracking 2->3
```

No fijar thresholds arbitrarios si no existen en contrato.

Usar:

```text
comparación contra 345
```

para definir desviación respecto al baseline sano.

---

# 25. Comparación temporal recomendada

Dividir el segundo tramo en:

```text
inicio del goal
0-5 s
5-10 s
10-20 s
20 s hasta fallback
```

y además:

```text
[-5,0] s respecto a T1
[0,+2] s
[+2,+5] s
```

Esto debe mostrar:

> qué señal cambia antes.

---

# 26. Analizar especialmente el canal angular

Si la evidencia apunta a angular, distinguir:

```text
raw visual rotation
        ↓
omega_raw
        ↓
omega_motion
        ↓
omega_hat(t_k)
        ↓
omega_dynamic(now)
```

y en pose:

```text
visual_R
    ↓
base_R
    ↓
dynamic_R(now)
```

Comparar con GT.

Buscar:

```text
R razonable + omega mala
omega razonable + R mala
ambas malas
```

porque requieren soluciones distintas.

---

# 27. Analizar especialmente validez

Si el primer evento es:

```text
velocity_valid=false
```

determinar:

```text
¿lineal?
¿angular?
¿ambas?
```

Si es:

```text
predictor_healthy=false
```

determinar:

```text
qué contador/criterio
qué medida lo disparó
qué timestamp
```

Si es:

```text
local_continuity_valid=false
```

determinar si:

```text
tracking seguía OK
ref válida
O seguía numéricamente continuo
```

No usar nombres generales.

---

# 28. No ejecutar 347

347 estaba reservado como repetición de una 346 conseguida.

Como:

```text
346 = NO CONSEGUIDA
```

mantener:

```text
347 NO EJECUTADA
```

La nueva numeración empieza en:

```text
348
```

---

# 29. No volver todavía a la vuelta completa

No ejecutar:

```text
trayectoria completa alrededor del edificio
```

hasta que:

```text
dos fachadas ORB
```

sean funcionalmente estables.

La ruta corta ya reproduce el problema y ofrece mejor diagnóstico.

---

# 30. Estado esperado al terminar este bloque

Idealmente debemos pasar de:

```text
"ORB falla en una trayectoria larga"
```

a una conclusión específica como:

```text
"omega_dynamic se separa primero 3.2 s antes del fallback"
```

o:

```text
"velocity_valid cae por X aunque tracking=2"
```

o:

```text
"p/v empiezan a divergir 4.1 s antes que R/omega"
```

Eso será el criterio de éxito del diagnóstico.

---

# 31. Resumen ejecutivo

344-345 han cerrado un defecto real:

```text
STALE_RAW_HISTORY
```

pero 346 demuestra que queda otra causa.

La evidencia crítica de 346 es:

```text
segundo tramo ORB
    ↓
fallback reason=tracking_lost
tracking todavía=2
    ↓
orb_qualifying
tracking=2
    ↓
trajectory_source_locked
tracking=2
```

y sólo mucho después:

```text
tracking 2 -> 3
```

Por tanto, el siguiente trabajo es:

```text
AUDITAR condición exacta del fallback
        +
comparar 345 vs 346
        +
localizar el primer canal que diverge
```

Ejecutar:

```text
348
misma ruta de dos fachadas
misma configuración
sólo telemetría ampliada
```

Si aún no concluye:

```text
349A:
p/v ORB + R/omega GT

349B:
p/v GT + R/omega ORB
```

> No aplicar otro parche hasta saber si el fallo nace primero en translación, angular, política de validez o realimentación de control.
