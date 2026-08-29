# Instrucciones para Codex — Fase 5H tras prueba 261
## Eliminar realimentación angular residual: `omega_bias` en SMALL y retención de `omega_motion` rechazado

## 0. Estado de la iteración

Estado actual:

```text
Fase 5H: PARCIAL
Iteración anterior: NO CONSEGUIDA
Prueba 260: CONSEGUIDA como calibración
Prueba 261: NO CONSEGUIDA
Builds: correctos
GTests: 27/27
Etapa 3: NO ejecutada
```

La arquitectura actual ya separa:

```text
omega_motion
+
omega_bias
=
omega_total
```

Esta separación conceptual se mantiene.

La prueba 261 demuestra, sin embargo, que la separación por sí sola no rompe todavía la realimentación angular.

---

# 1. Evidencia actual que debe tomarse como punto de partida

La prueba 260 se considera válida como calibración con GT gobernando.

Sus valores de referencia permiten mantener, de momento, los límites raw existentes aproximadamente en:

```text
max_raw_rotation_step_rad      ≈ 0.12 rad
max_raw_angular_speed_radps    ≈ 1.0 rad/s
max_raw_angular_acceleration   ≈ 10 rad/s²

GOOD_DT hasta                 ≈ 0.075 s
DEGRADED_DT hasta             ≈ 0.20 s
```

No reducir estos límites para intentar arreglar la prueba 261 salvo que aparezca evidencia nueva.

La prueba 261 muestra:

```text
- handoff GT -> ORB correcto;
- primer error de control prácticamente cero;
- ORB gobierna sólo ~3.82 s antes del fallback;
- aparecen SMALL antes de la divergencia;
- omega_bias ya actúa con residual pequeño;
- después el movimiento físico inducido pasa a raw plausible;
- omega_motion llega aproximadamente a 0.629 rad/s;
- omega_bias está acotado aproximadamente a 0.080 rad/s;
- omega_motion termina dominando omega_total;
- al rechazar raw, el último omega_motion alto se conserva;
- durante tres rechazos se mantienen valores del orden de ~0.56 rad/s;
- el fallback ocurre mientras tracking todavía es válido;
- la pérdida real de tracking llega ~10.25 s después.
```

Conclusión:

> La pérdida de tracking no origina el fallo. El estimador angular entra primero en una realimentación inestable y después termina invalidándose/fallback.

---

# 2. Hipótesis actual

La cadena causal que se quiere romper es:

```text
residual absoluto pequeño
        |
        v
omega_bias actúa
        |
        v
R_control empieza a moverse
        |
        v
controlador genera torque
        |
        v
el dron gira físicamente
        |
        v
ORB observa ese giro real
        |
        v
DeltaR_raw pasa a ser físicamente plausible
        |
        v
omega_motion sigue el giro
        |
        v
omega_total aumenta
        |
        v
más torque
        |
        v
más movimiento físico
```

Una vez que esto empieza, el clasificador raw puede estar funcionando correctamente:

```text
si el dron realmente gira,
ORB observa un giro real.
```

Por tanto:

> NO intentar solucionar esta iteración endureciendo los límites raw.

El problema principal está antes:

```text
omega_bias está actuando demasiado pronto
```

y después:

```text
omega_motion no se amortigua cuando el soporte raw deja de ser fiable
```

---

# 3. Objetivo exacto de la siguiente modificación

Mantener la separación:

```text
omega_motion
omega_bias
```

pero aplicar dos cambios principales:

```text
A. residual SMALL normal -> NO debe generar bias continuo
B. raw rechazado -> NO debe conservar indefinidamente el último omega_motion
```

Además:

```text
C. mientras exista movimiento raw significativo,
   reducir o congelar la corrección de bias.
```

No tocar todavía:

```text
ganancias del controlador
GT
mux
W
servidor
ORB-SLAM3 core
extrínseca
reference gate
trayectorias
```

---

# 4. Cambio A — deadband de bias

## 4.1. Problema actual

En la prueba 261 aparecen muestras donde:

```text
raw_step_rotation_rad es muy pequeño
omega_motion es casi cero
```

pero:

```text
omega_bias ya tiene magnitud bastante mayor
```

Por ejemplo, conceptualmente:

```text
omega_motion ~ casi 0
omega_bias   ~ 0.01-0.02 rad/s
```

Esto hace que, incluso en hover, el estado angular publicado esté siendo movido principalmente por el corrector de bias.

Eso no debe suceder con ruido residual pequeño.

## 4.2. Nueva política

Introducir un deadband explícito sobre el residual angular absoluto:

```text
absolute_residual < bias_deadband_enter
    ->
omega_bias_target = 0
```

Dentro del deadband:

```text
NO corregir orientación absoluta continuamente
```

La idea es:

```text
ruido pequeño de ORB
!=
desalineación que haya que corregir
```

---

# 5. Histéresis del bias

No usar un único threshold de entrada/salida.

Implementar:

```text
bias_deadband_enter
bias_deadband_exit
```

con histéresis.

Ejemplo conceptual:

```text
si bias está OFF:
    sólo puede activarse si residual > ENTER

si bias está ON:
    sólo puede apagarse si residual < EXIT
```

con:

```text
EXIT < ENTER
```

No fijar todavía valores definitivos sin revisar distribución real de residual en 260/261.

Los valores deben ser:

```text
parametrizados
documentados
telemetrizados
```

---

# 6. Bias sólo tras persistencia real

Aunque el residual supere el deadband:

```text
NO activar bias en el primer frame
```

Debe pasar por confirmación temporal.

Estados conceptuales:

```text
BIAS_OFF
BIAS_PENDING
BIAS_ACTIVE
BIAS_DECAY
```

Ejemplo:

```text
residual pequeño
    -> BIAS_OFF

residual supera ENTER
    -> BIAS_PENDING

residual persiste N frames coherentes
    -> BIAS_ACTIVE

residual vuelve bajo EXIT
    -> BIAS_DECAY / OFF
```

No utilizar simplemente:

```text
SMALL -> bias
```

como comportamiento por defecto.

---

# 7. El bias no representa velocidad física

Regla conceptual obligatoria:

```text
omega_bias
```

NO representa:

```text
"el dron está girando físicamente"
```

Representa:

```text
"quiero reducir lentamente un offset absoluto del estimador"
```

Por tanto:

```text
omega_bias debe ser pequeño
lento
acotado
con aceleración acotada
```

y nunca debe dominar el comportamiento del dron en hover.

---

# 8. Cambio B — decay de `omega_motion` cuando raw deja de ser fiable

## 8.1. Problema actual

En 261, cuando aparece:

```text
REJECTED_EXCESSIVE
```

el estimador deja de confiar en la medida raw, pero conserva una `omega_motion` alta de la última muestra válida.

Esto equivale a:

```text
"no confío en la observación"
pero
"seguiré suponiendo el mismo giro"
```

Sin IMU, esto es demasiado agresivo.

## 8.2. Nueva política

Cuando:

```text
raw_motion_class = REJECTED
```

o equivalente:

```text
NO actualizar omega_motion desde raw
```

pero tampoco mantenerla constante.

Debe ocurrir:

```text
omega_motion_target = 0
```

y aplicar un decay rápido y físicamente acotado.

Conceptualmente:

```text
omega_motion = 0.56
    ->
0.30
    ->
0.10
    ->
0.00
```

Los números son ilustrativos.

No hacer:

```text
0.56
0.56
0.56
```

hasta fallback.

---

# 9. Parámetros para decay de movimiento no observado

Introducir un parámetro explícito, por ejemplo:

```text
rejected_motion_decay_acceleration_radps2
```

o:

```text
rejected_motion_decay_time_sec
```

La implementación concreta queda a Codex.

La condición obligatoria es:

> El estado angular debe perder rápidamente velocidad asumida cuando deja de existir una observación raw fiable.

No poner `omega_motion=0` instantáneamente si eso crea discontinuidad demasiado grande.

Debe haber un decay continuo y medible.

---

# 10. Cambio C — reducir bias durante movimiento raw significativo

Mientras:

```text
|omega_motion|
```

o:

```text
|omega_raw|
```

sea claramente significativo, la prioridad debe ser seguir la dinámica física observada.

En ese estado:

```text
bias_gain efectivo -> muy pequeño
```

o:

```text
bias temporalmente congelado
```

Conceptualmente:

```text
si el dron está girando físicamente:
    seguir movimiento raw

cuando vuelve a estar casi estable:
    corregir lentamente offset absoluto
```

Esto evita sumar simultáneamente:

```text
omega_motion grande
+
omega_bias
```

durante un giro.

---

# 11. Propuesta de lógica final

La lógica angular debería quedar aproximadamente:

```text
                    ORB RAW
                       |
                       v
             DeltaR entre medidas
                       |
                       v
                 omega_raw
                       |
                plausibilidad
                       |
        +--------------+--------------+
        |                             |
     PLAUSIBLE                      REJECTED
        |                             |
        v                             v
filtrar hacia raw              decay hacia cero
        |                             |
        +-------------+---------------+
                      |
                      v
                 omega_motion


           R_raw vs R_control
                      |
                      v
              residual absoluto
                      |
         +------------+------------+
         |                         |
   dentro deadband            fuera deadband
         |                         |
         v                         v
   bias_target=0               BIAS_PENDING
                                   |
                              persistencia
                                   |
                                   v
                              BIAS_ACTIVE
                                   |
                   corrección lenta y acotada
                                   |
                                   v
                              omega_bias


si movimiento raw significativo:
    reducir/congelar omega_bias


omega_total =
    omega_motion
    +
    omega_bias
```

---

# 12. Propiedad deseada en hover

En hover estable:

```text
omega_raw        ~ 0
omega_motion     ~ 0
omega_bias       = 0 la mayor parte del tiempo
omega_total      ~ 0
ew               ~ pequeño
torque           ~ pequeño
```

Puede existir ruido de residual:

```text
absolute_residual != 0
```

sin que eso implique:

```text
omega_bias != 0
```

Éste es el cambio conceptual principal.

---

# 13. Qué NO hacer

No:

```text
reducir max_raw_rotation_step_rad
```

No:

```text
reducir max_raw_angular_speed_radps
```

No:

```text
reducir max_raw_angular_acceleration_radps2
```

basándose sólo en 261.

La prueba 260 sirve precisamente para evitar bloquear maniobras físicas reales.

No:

```text
poner omega_motion=0 instantáneamente
```

sin comprobar continuidad.

No:

```text
aumentar damping del controlador
```

No:

```text
reducir Kr/Kw
```

No:

```text
usar GT para corregir el estimador
```

No:

```text
volver a mezclar omega_motion y omega_bias en una única señal interna
```

No:

```text
ejecutar etapa 3 si hover vuelve a fallar
```

---

# 14. GTests obligatorios

Mantener los 27/27 tests actuales y añadir nuevos.

## Test 1 — residual SMALL dentro de deadband

Secuencia:

```text
raw casi fijo
residual absoluto pequeño
```

Esperado:

```text
bias_state = OFF
omega_bias = 0
omega_motion ~ 0
omega_total ~ 0
```

## Test 2 — ruido alternante alrededor de cero

Residual:

```text
+eps
-eps
+eps
-eps
```

Esperado:

```text
bias no se activa
```

## Test 3 — histéresis

Secuencia:

```text
residual sube por encima de ENTER
se confirma
bias se activa
residual baja pero queda entre EXIT y ENTER
bias no conmuta repetidamente
residual baja por debajo de EXIT
bias se apaga/decae
```

## Test 4 — residual persistente fuera de deadband

Esperado:

```text
BIAS_PENDING
-> BIAS_ACTIVE
```

pero:

```text
omega_bias limitado
corrección gradual
```

## Test 5 — raw plausible significativo + residual

Secuencia:

```text
omega_raw representa giro físico real
residual absoluto también existe
```

Esperado:

```text
omega_motion sigue giro
omega_bias reducido/congelado
```

## Test 6 — raw rechazado con omega_motion alta

Inicializar:

```text
omega_motion > 0
```

después introducir:

```text
REJECTED
REJECTED
REJECTED
```

Esperado:

```text
omega_motion decrece hacia cero
```

y no se conserva constante.

## Test 7 — recuperación tras rechazo

Secuencia:

```text
movimiento plausible
rechazo
decay
movimiento plausible nuevo
```

Esperado:

```text
reentrada suave
sin salto
```

## Test 8 — hover largo sintético

Miles de muestras raw con ruido pequeño.

Esperado:

```text
omega_motion bounded
omega_bias prácticamente siempre OFF
orientación no deriva por realimentación interna
```

## Test 9 — coherencia pose/omega

Mantener:

```text
R(t+dt)
~
Exp(omega_total * dt) * R(t)
```

## Test 10 — reference KF

Todos los tests existentes de cambios de KF deben seguir pasando.

No degradar esa parte.

---

# 15. Telemetría nueva/ajustada

Registrar por muestra:

```text
absolute_residual_rad

bias_state
bias_deadband_enter
bias_deadband_exit

bias_pending_frames
bias_active

omega_bias_target
omega_bias_applied

omega_raw
omega_motion_target
omega_motion_applied

raw_motion_class

raw_rejected
motion_decay_active
motion_decay_rate

omega_total

published_pose_rotation_step_rad

ew_norm
er_norm
torque_norm

tracking_state
reference_kf
frames_since_reference_change
```

---

# 16. Qué quiero ver específicamente en la siguiente prueba

Repetir exactamente el hover ORB equivalente a la prueba 261.

No cambiar de escenario.

Quiero poder reconstruir:

```text
handoff
    |
primer residual
    |
bias state
    |
omega_bias
    |
omega_raw
    |
omega_motion
    |
omega_total
    |
ew
    |
torque
    |
movimiento real GT
    |
tracking
```

---

# 17. Criterio de éxito del hover

La nueva prueba sólo será `CONSEGUIDA` si:

```text
1. handoff sigue siendo correcto;
2. ORB gobierna TODO el escenario;
3. no aparece crecimiento sostenido de omega_total;
4. no aparece crecimiento sostenido de ew;
5. no aparece crecimiento sostenido de torque;
6. el bias permanece OFF con ruido SMALL normal;
7. el bias sólo se activa con residual persistente real;
8. durante raw rechazado, omega_motion decae;
9. no hay fallback provocado por el estimador;
10. no hay tracking loss inducido por oscilación del control.
```

---

# 18. Si vuelve a fallar

Si, después de estos cambios:

```text
bias SMALL neutralizado
+
omega_motion rechazado con decay
```

el hover sigue oscilando, NO seguir modificando thresholds por intuición.

El siguiente diagnóstico deberá centrarse en:

```text
latencia/fase de omega_motion
```

porque ORB entrega medidas aproximadamente a 20 Hz mientras el controlador trabaja a 50 Hz.

Hipótesis siguiente, sólo si esta iteración falla:

```text
omega visual llega con suficiente retraso
para introducir desfase en el término angular del controlador
```

Entonces estudiar:

```text
timestamp de medida
timestamp de recepción
timestamp de publicación
state_age
omega_raw
omega_motion
omega recibida por control
ew
torque
```

No investigar esto antes de eliminar los dos defectos ya demostrados en 261.

---

# 19. No avanzar a etapa 3

Regla:

```text
si hover ORB no completa:
    STOP
```

No ejecutar:

```text
etapa 3
etapa 4
...
```

hasta aprobar hover.

---

# 20. Qué debe devolver Codex

Tras implementar:

```text
Resultado: CONSEGUIDO / PARCIAL / NO CONSEGUIDO
```

Explicar:

```text
- archivos modificados;
- cambio exacto en lógica de bias;
- thresholds de deadband/histéresis;
- lógica exacta de BIAS_OFF/PENDING/ACTIVE;
- política de bias durante movimiento raw;
- política de decay de omega_motion rechazada;
- parámetros nuevos;
- GTests nuevos;
- total GTests;
- builds;
- resultado de la nueva prueba de hover;
- duración total gobernando ORB;
- si hubo fallback;
- si hubo pérdida de tracking;
- cronología de cualquier oscilación;
- comparación directa contra prueba 261;
- decisión explícita sobre si se puede ejecutar etapa 3.
```

Actualizar:

```text
historial_5H_RESUMEN.md
07_ULTIMA_SESION.md
contrato 5H
```

sin borrar el historial real anterior.

---

# 21. Resumen ejecutivo

La prueba 261 ha demostrado que la separación:

```text
omega_motion + omega_bias
```

es necesaria pero todavía insuficiente.

Los dos defectos que deben corregirse ahora son:

```text
1. omega_bias actúa demasiado pronto con residual SMALL;
2. omega_motion alta se conserva cuando el raw deja de ser fiable.
```

La nueva política debe ser:

```text
residual pequeño
    ->
bias = 0

residual persistente fuera de deadband
    ->
bias lento y confirmado

movimiento raw significativo
    ->
priorizar omega_motion y reducir bias

raw rechazado
    ->
omega_motion decae hacia cero
```

No endurecer los límites raw calibrados en 260 para esconder el problema.

La meta inmediata sigue siendo una sola:

> **conseguir un hover completamente estable gobernado por ORB antes de continuar con cualquier otra etapa de Fase 5H.**
