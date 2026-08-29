# Instrucciones para Codex — Fase 5H tras pruebas 258/259
## Rediseño del estimador angular ORB para separar movimiento físico de corrección visual

## 0. Objetivo

Este documento sustituye la hipótesis de trabajo anterior como siguiente paso de diagnóstico/implementación de Fase 5H.

Las pruebas 258 y 259 han dado información suficiente para cambiar el foco:

- la prueba 258 demuestra que ORB observado mientras GT gobierna funciona razonablemente en hover, X, Y, Z y yaw lento; la pérdida aparece en yaw rápido;
- la prueba 259 demuestra que un hover gobernado por ORB se vuelve inestable aunque el `GT -> ORB` entre con salto prácticamente nulo;
- en 259 la oscilación empieza **antes** del primer `MODERATE_PENDING`;
- una muestra todavía clasificada `SMALL` llega a publicar aproximadamente `0.058777 rad` de paso angular;
- después la probation confirma un residual creciente y llega a publicar pasos de aproximadamente `0.075 rad`;
- finalmente aparecen rechazos excesivos, pérdida de tracking y fallback;
- el episodio crítico no depende de un cambio reciente de KF: el primer `MODERATE_PENDING` aparece con la misma referencia activa desde hace muchos frames.

Por tanto, la prioridad ya no es endurecer el cambio de KF ni el handoff. La prioridad es:

> **rediseñar cómo las medidas angulares ORB se convierten en dinámica de control, separando el movimiento relativo observado entre frames de la corrección absoluta entre la pose ORB y el predictor.**

No avanzar a las etapas 3-8 mientras el hover ORB de la etapa 2 no sea estable.

---

# 1. Estado actual que Codex debe revisar antes de modificar

Repositorio:

```text
https://github.com/IvanCS-Chenfu/Mapeado-con-Drones
```

Revisar el `main` actual, especialmente:

```text
dron/orbslam3_ros2/src/stereo/navigation-state-estimator.cpp
dron/orbslam3_ros2/src/stereo/navigation-state-estimator.hpp
dron/orbslam3_ros2/src/stereo/stereo-slam-node.cpp
dron/orbslam3_ros2/test/test_navigation_state_estimator.cpp

dron/dron_individual/src/control_tray/control_calcular_fuerzas.cpp

codex/pipeline/fase_5_poses_drones_sin_gt/subfases/subfase_5H.md
codex/pipeline/fase_5_poses_drones_sin_gt/historial/
```

También revisar los registros/documentación de:

```text
prueba 258
prueba 259
historial_5H_RESUMEN.md
07_ULTIMA_SESION.md
```

No asumir que el diagnóstico previo sigue siendo válido si el código real contradice este documento. Si aparece una diferencia relevante, documentarla antes de cambiar arquitectura.

---

# 2. Qué NO hay que tocar

No modificar para intentar solucionar este problema:

```text
GT
navigation_state_mux
ganancias Kp/Kv/Kr/Kw
optimizador global
W / GlobalPoseStore
ORB-SLAM3 core
extrínseca B_T_C
YAML del recorrido representativo
política global de goals
```

No cambiar la arquitectura:

```text
O = frame local continuo de control
W = frame global corregible
```

Una revisión/optimización en `W` no debe mover `O`.

No usar GT como entrada del nuevo estimador ORB.

GT puede seguir utilizándose **únicamente como verdad externa de diagnóstico/metricado**, nunca para decidir o corregir el estado ORB normal.

No volver a filtrar la actitud GT.

No esconder el fallo bajando ganancias del controlador.

---

# 3. Qué han demostrado 258 y 259

## 3.1. La prueba 258

Etapa 1:

```text
GT gobierna
ORB se observa
```

Resultado:

```text
11/11 pasos
7/7 goals
control GT
```

ORB mantiene tracking en:

```text
hover
X
Y
Z
yaw lento
```

y pierde tracking durante yaw rápido, recuperándose después en otro epoch.

Conclusión importante:

> ORB no parece incapaz de producir una estimación útil simplemente por estar en hover o realizar movimientos suaves.

Esto no demuestra que la estimación sea suficientemente buena para control, pero sí permite comparar:

```text
ORB abierto / observado
vs
ORB cerrando el lazo
```

## 3.2. La prueba 259

Etapa 2:

```text
hover gobernado por ORB
```

El handoff entra correctamente.

Después:

```text
ORB gobierna
-> crece oscilación
-> aumenta omega estimada
-> aumenta ew / torque
-> todavía hay medidas SMALL
-> aparece paso publicado ~0.058777 rad
-> primer MODERATE_PENDING
-> residual creciente
-> MODERATE_CONFIRMED
-> correcciones cada vez mayores
-> pasos publicados ~0.075 rad
-> REJECTED_EXCESSIVE
-> tracking perdido
-> fallback
```

Esto cambia la interpretación:

> La probation MODERATE no es la causa inicial. Cuando se activa, el lazo ya está empezando a divergir.

Además, el primer episodio moderado no coincide con un cambio reciente de KF.

Por tanto, **los cambios de KF dejan de ser la hipótesis principal para este fallo concreto**.

---

# 4. Problemas concretos observados en el código actual

## 4.1. El umbral `SMALL` crece con `dt`

Actualmente existe una lógica equivalente a:

```cpp
dynamic_small_allowance =
    0.5 * max_angular_acceleration * dt^2;

small_rotation_limit =
    small_rotation_innovation_rad + dynamic_small_allowance;
```

con cap posterior por el máximo global.

Problema:

> Un gap de medida mayor hace crecer el intervalo que se considera `SMALL`.

Eso puede ser razonable como parte de un modelo dinámico, pero **no debe convertir automáticamente una corrección angular absoluta grande en una medida segura para control**.

Un `dt` peor debería reducir la confianza de la muestra, no simplemente abrir una puerta angular mayor.

---

## 4.2. Se mezclan velocidad angular raw y velocidad angular del predictor

Actualmente se calcula una aceleración implícita con una idea equivalente a:

```cpp
(raw_angular_velocity - angular_velocity_control) / dt
```

Esto mezcla dos señales diferentes:

```text
raw angular velocity:
movimiento observado entre dos medidas ORB

angular_velocity_:
estado dinámico que ya está produciendo el predictor
```

Para decidir si el **movimiento observado por ORB** es físicamente consistente, la aceleración raw debe compararse principalmente con la **velocidad raw anterior**:

```text
alpha_raw =
(omega_raw[n] - omega_raw[n-1]) / dt
```

No con el estado del predictor.

La comparación raw-vs-predictor puede conservarse como diagnóstico, pero no debe ser la definición principal de aceleración física observada.

---

## 4.3. La probation confirma principalmente persistencia del residual

Actualmente el estado moderado usa de forma importante:

```text
rotation_innovation =
R_measurement vs R_predicted
```

y compara la innovación actual con la pendiente anterior mediante:

```text
dirección/coseno
magnitud relativa
persistencia
```

Problema fundamental:

```text
residual persistente
!=
movimiento físico persistente
```

Ejemplo:

```text
medida ORB aproximadamente estable
predictor empieza a alejarse

residual:
0.07
0.11
0.15
0.18
```

La probation puede interpretar:

```text
"la corrección persiste, confirmarla"
```

cuando en realidad puede estar ocurriendo:

```text
"el predictor está divergiendo de una medida que no está girando así"
```

La prueba 259 encaja con esta posibilidad.

---

## 4.4. Una corrección confirmada sigue entrando con `orientation_alpha`

Una vez una moderada queda confirmada, actualmente se vuelve a construir un target del tipo:

```text
target_orientation =
Exp(orientation_alpha * rotation_innovation)
* predicted_orientation
```

Con un `orientation_alpha` alto, por ejemplo `0.70`, una innovación absoluta importante puede transformarse en una corrección angular grande en una sola actualización.

Eso contradice la finalidad de la probation.

Confirmar una corrección significa:

```text
"creo que este offset existe"
```

NO significa:

```text
"el dron acaba de girar el 70 % de ese offset físicamente"
```

La corrección confirmada debe convertirse en una **corrección lenta y acotada del estado**, no en un salto proporcional inmediato.

---

# 5. Nueva separación conceptual obligatoria

A partir de ahora separar dos señales:

```text
A) MOVIMIENTO ANGULAR OBSERVADO
B) ERROR / CORRECCIÓN ANGULAR ABSOLUTA
```

No tratarlas como si fueran lo mismo.

Arquitectura deseada:

```text
             medidas ORB raw
                   |
                   v
       R_raw[n-1] -> R_raw[n]
                   |
                   v
              DeltaR_raw
                   |
          omega_raw / alpha_raw
                   |
          validación dinámica
                   |
                   v
           omega_motion_target
                   |
                   +------------------+
                                      |
                                      v
                         estado angular continuo
                                      |
                                      v
                              R_control + omega
                                      ^
                                      |
                   +------------------+
                   |
       residual absoluto ORB-control
                   |
           confirmación de offset
                   |
           corrección lenta/bias
```

Esto debe reemplazar la idea implícita:

```text
residual absoluto grande y persistente
-> probablemente movimiento físico
-> aplicar gran parte del residual
```

---

# 6. Canal A — movimiento angular observado entre medidas raw

## 6.1. Calcular movimiento raw

Mantener la última medida raw y calcular:

```text
DeltaR_raw =
R_raw[n] * inverse(R_raw[n-1])

r_raw =
Log(DeltaR_raw)

omega_raw =
r_raw / dt_raw
```

El orden exacto debe respetar la convención actual de Sophus del proyecto.

Añadir explícitamente:

```text
previous_raw_angular_velocity
```

y calcular:

```text
alpha_raw =
(omega_raw[n] - omega_raw[n-1]) / dt_raw
```

No usar `angular_velocity_` del predictor como sustituto de `omega_raw[n-1]`.

---

## 6.2. Calidad temporal de la medida

Crear una clasificación explícita de calidad del `dt`.

Conceptualmente:

```text
GOOD_DT
DEGRADED_DT
INVALID_DT
```

Los límites deben ser parámetros y/o derivarse razonablemente de la frecuencia ORB esperada.

Si el `dt` es mayor de lo normal:

```text
NO aumentar sin más el umbral SMALL
```

Debe ocurrir algo más parecido a:

```text
dt degradado
-> menor confianza
-> límites absolutos siguen vigentes
-> evitar derivadas agresivas
```

No inventar todavía valores definitivos sin observar distribución real de `dt` en 258/259.

---

## 6.3. Validar dinámica raw

El movimiento raw debe comprobarse con:

```text
|r_raw|
|omega_raw|
|alpha_raw|
dirección respecto a omega_raw anterior
dt
tracking
contexto de KF
```

Quiero un límite angular absoluto que no crezca indefinidamente con `dt`.

Por ejemplo, conceptualmente:

```text
raw step plausible si:

step < max_raw_rotation_step
AND
omega_raw < max_raw_angular_speed
AND
alpha_raw < max_raw_angular_acceleration
AND
dt_quality aceptable
```

Los nombres/parámetros concretos quedan a Codex, pero no deben quedar hardcodeados sin visibilidad.

---

# 7. Canal B — residual absoluto ORB vs estado continuo

Seguir calculando:

```text
rotation_residual =
Log(
    R_raw *
    inverse(R_control_pred)
)
```

pero cambiar su significado.

Este residual responde a:

> ¿Cuánto difiere la orientación absoluta que ORB estima de la orientación continua que estamos publicando?

NO responde directamente a:

> ¿Cuánto acaba de girar físicamente el dron?

Por tanto, usarlo para:

```text
corrección lenta
alineación/bias
detección de deriva del predictor
diagnóstico
```

No usarlo como fuente principal de velocidad física.

---

# 8. Cómo tratar un offset absoluto persistente

Caso especialmente importante:

```text
R_raw deja de moverse
pero R_control está separado
```

Entonces:

```text
omega_raw ~ 0
alpha_raw ~ 0

residual absoluto != 0
```

Eso no debe producir una gran `omega` como si el dron estuviera girando.

Debe ocurrir:

```text
offset confirmado
-> activar corrección lenta
-> reducir residual progresivamente
```

Introducir una velocidad máxima de corrección de bias/alineación, por ejemplo conceptualmente:

```text
max_orientation_bias_correction_rate_radps
```

y, si hace falta:

```text
max_orientation_bias_correction_acceleration_radps2
```

No fijar valores definitivos sin ensayo.

La corrección absoluta confirmada debe absorberse en varios ciclos, nunca como:

```text
0.70 * residual en un frame
```

---

# 9. Movimiento físico real sostenido

Caso contrario:

```text
el dron realmente gira
```

Entonces las medidas raw mostrarán:

```text
DeltaR_raw persistente
omega_raw consistente
dirección consistente
alpha_raw razonable
```

Ese movimiento debe poder alimentar el estado de control **sin esperar a que crezca un residual absoluto**.

Objetivo:

```text
yaw físico real
-> seguido rápidamente por omega_motion

corrección visual absoluta
-> absorbida lentamente por bias correction
```

Esta separación es la pieza principal del rediseño.

---

# 10. Estado angular final publicado

La salida debe seguir cumpliendo coherencia cinemática.

Quiero distinguir internamente, aunque no necesariamente en el mensaje ROS final:

```text
omega_motion
omega_bias_correction
omega_total
```

con:

```text
omega_total =
omega_motion
+
omega_bias_correction
```

o formulación equivalente si Codex encuentra una implementación SE(3) más correcta.

Después:

```text
R_control(t + dt)
=
Exp(omega_total * dt)
*
R_control(t)
```

dentro de la convención del proyecto.

Mantener la propiedad ya buena del predictor actual:

> la pose publicada debe evolucionar usando la misma velocidad angular publicada.

No volver a copiar orientación raw directamente al mensaje.

---

# 11. Muy importante: evitar acumulación positiva en hover

En hover real debería ocurrir aproximadamente:

```text
omega_raw -> 0
omega_motion -> 0
omega_total -> 0
torque -> pequeño
```

Si existe un residual absoluto pequeño:

```text
bias correction lenta
```

pero nunca:

```text
residual
-> omega aumenta
-> predictor se separa
-> residual aumenta
-> omega aumenta más
```

Añadir un GTest que reproduzca específicamente ese patrón.

---

# 12. Qué hacer con SMALL / MODERATE / EXCESSIVE

No es obligatorio eliminar el enum actual, pero su semántica debe revisarse.

La solución más limpia sería separar:

```text
RawMotionClass
```

de:

```text
AbsoluteCorrectionState
```

Ejemplo conceptual:

```text
RawMotionClass:
  PLAUSIBLE
  DEGRADED_DT
  SUSPICIOUS
  REJECTED

AbsoluteCorrectionState:
  NONE
  PENDING
  CONFIRMED
  DISCARDING
```

Así evitamos que:

```text
SMALL
```

signifique a la vez:

```text
residual absoluto pequeño
```

y:

```text
movimiento seguro para control
```

Si Codex decide mantener las clases actuales para minimizar cambios, debe documentar claramente cómo quedan reinterpretadas y añadir telemetría separada de movimiento raw.

---

# 13. Parámetros que hay que revisar

Actualmente existen parámetros como:

```text
small_rotation_innovation_rad
max_rotation_innovation_rad
moderate_confirmation_frames
moderate_post_reference_confirmation_frames
moderate_max_pending_frames
moderate_direction_consistency
moderate_magnitude_ratio
moderate_timeout_sec
post_reference_switch_frames
orientation_alpha
max_angular_speed_radps
max_angular_acceleration_radps2
```

No eliminarlos sin revisar compatibilidad.

Probablemente harán falta conceptos nuevos, por ejemplo:

```text
max_raw_rotation_step_rad
max_raw_angular_speed_radps
max_raw_angular_acceleration_radps2

raw_dt_nominal_sec
raw_dt_max_good_sec
raw_dt_max_degraded_sec

raw_motion_filter_alpha

max_orientation_bias_correction_rate_radps
max_orientation_bias_correction_acceleration_radps2

absolute_residual_confirmation_frames
absolute_residual_timeout_sec
```

No es obligatorio usar estos nombres exactos.

Prioridad:

```text
configurable
documentado
telemetrizado
sin constantes mágicas
```

---

# 14. Mantener la lógica de KF salvo bug demostrado

No endurecer ahora `NavigationStateEstimator` por intuición.

La prueba 259 muestra que la inestabilidad crítica puede aparecer:

```text
mismo reference KF
muchos frames desde el último switch
```

Por tanto, mantener:

```text
reference gate
continuidad O
probation geométrica multi-KF
W separado de O
```

Añadir contexto de KF a logs, pero no atribuir automáticamente el fallo al churn.

Si durante la implementación se descubre un bug matemático real de referencia, documentarlo por separado antes de modificarlo.

---

# 15. Telemetría nueva obligatoria

La próxima prueba debe permitir reconstruir causalidad.

## 15.1. Medida raw

Por muestra:

```text
measurement_stamp
dt_raw
dt_quality

ref_kf
frames_since_reference_change
tracking_state
map_epoch

raw_rotation_step_vector
raw_rotation_step_rad

omega_raw
previous_omega_raw
alpha_raw

raw_motion_class
raw_motion_confidence si existe
```

## 15.2. Residual absoluto

Registrar:

```text
absolute_rotation_residual_vector
absolute_rotation_residual_rad

absolute_correction_state
pending_id
pending_frames
consistency

bias_correction_target
bias_correction_omega
bias_correction_step
```

## 15.3. Estado dinámico

Registrar:

```text
omega_motion
omega_bias
omega_total_before_limits
omega_total_after_limits

angular_acceleration_applied

published_pose_rotation_step_rad
published_omega

predictor_healthy
```

## 15.4. Controlador

Mantener/añadir:

```text
er_norm
ew_norm
torque_norm
force
R_act
omega_body
pose_source
state_age
```

## 15.5. GT sólo diagnóstico

Durante estas pruebas, registrar como métrica externa:

```text
GT orientation
GT angular velocity
```

Nunca usarlas como entrada de control.

Esto permitirá responder:

```text
¿ORB inventó giro antes de que el dron girase físicamente?

o

¿el dron empezó a girar físicamente y ORB simplemente lo siguió?
```

---

# 16. Cronología que quiero obtener de la próxima prueba fallida o exitosa

Para cualquier episodio de oscilación, Codex debe reconstruir:

```text
1. raw DeltaR
2. omega_raw
3. alpha_raw
4. residual absoluto
5. estado pending/confirmed de bias
6. omega_motion
7. omega_bias
8. omega publicada
9. paso de pose publicado
10. er
11. ew
12. torque
13. GT angular motion
14. tracking state
15. reference KF
```

Con timestamps.

Quiero saber el orden causal.

---

# 17. Qué conclusiones deben poder salir de los logs

## Caso A — ORB raw empieza a oscilar antes del torque

Secuencia:

```text
raw DeltaR crece
omega_raw crece
estado ORB publicado crece
torque crece
GT empieza a oscilar
tracking cae
```

Conclusión probable:

```text
ruido/jitter ORB está excitando el control
```

Entonces habrá que reforzar validación/filtrado de movimiento raw.

---

## Caso B — torque/GT empiezan antes y raw ORB sigue el movimiento

Secuencia:

```text
estado publicado se desvía por predictor/bias
torque crece
GT gira
después omega_raw refleja ese giro
```

Conclusión probable:

```text
el predictor está generando dinámica artificial
```

Éste sería especialmente coherente con la hipótesis de 259.

---

## Caso C — residual crece pero raw motion ~0

Secuencia:

```text
omega_raw ~ 0
residual absoluto crece
```

Esto debe interpretarse como:

```text
desalineación predictor-medida
```

NO como:

```text
giro físico persistente
```

La salida esperada es:

```text
omega_motion ~ 0
bias correction lenta
```

---

## Caso D — giro físico real

Secuencia:

```text
GT omega real
omega_raw coherente
omega_motion coherente
residual acotado
torque razonable
tracking válido
```

Éxito.

---

# 18. GTests obligatorios

Mantener todos los tests actuales.

Actualmente la iteración previa llegó a:

```text
21/21
```

La siguiente modificación no puede reducir cobertura.

Añadir tests específicos.

## Test A — hover sin drift dinámico

Secuencia raw con ruido angular pequeño alrededor de una pose fija.

Esperado:

```text
omega_motion no crece
omega publicada no diverge
pose no entra en oscilación acumulativa
```

---

## Test B — residual absoluto persistente con raw motion cero

Ejemplo:

```text
medida raw salta a offset
después queda fija
```

Esperado:

```text
primer salto no se interpreta entero como movimiento físico
omega_raw posterior ~0
offset puede confirmarse
bias correction lenta
ningún salto proporcional orientation_alpha * residual
```

---

## Test C — predictor separado pero raw estable

Construir explícitamente el caso:

```text
R_control != R_raw
R_raw[n] == R_raw[n-1]
```

Esperado:

```text
no se crea una omega_motion creciente para perseguir el residual
```

Este test es esencial.

---

## Test D — giro físico sostenido

Secuencia:

```text
+delta
+delta
+delta
+delta
```

con `dt` regular.

Esperado:

```text
omega_raw estable
omega_motion sigue el giro
pose publicada sigue el giro
sin esperar a residual grande
```

---

## Test E — yaw físico con aceleración

Secuencia de omega raw creciente pero físicamente plausible.

Esperado:

```text
alpha_raw correcta
seguimiento sin rechazo innecesario
```

---

## Test F — gap de dt

Misma innovación absoluta con:

```text
dt normal
dt grande
```

Esperado:

```text
un dt grande no convierte automáticamente una corrección peligrosa en SMALL
```

---

## Test G — aceleración raw usa raw anterior

Demostrar que:

```text
alpha_raw =
(omega_raw_n - omega_raw_n-1) / dt
```

y que no depende directamente del `angular_velocity_` de control.

---

## Test H — corrección absoluta confirmada acotada

Residual grande persistente.

Esperado:

```text
corrección confirmada
pero correction_step <= límite temporal configurado
```

No permitir:

```text
correction_step ~= orientation_alpha * residual
```

en un solo frame.

---

## Test I — coherencia pose/omega

Conservar:

```text
R(t+dt) ~ Exp(omega_pub*dt) * R(t)
```

con tolerancia explícita.

---

## Test J — cambio de KF sigue siendo continuo

Los tests de reference switch existentes deben seguir pasando.

No degradar la solución anterior.

---

# 19. Simulaciones después de implementar

No ejecutar etapas 3-8 todavía.

## Paso 1 — build + GTests

Builds:

```text
orbslam3
dron_individual
simulacion_dron
```

Todos correctos.

GTests:

```text
100 %
```

---

## Paso 2 — repetir etapa 1 si el cambio del estimador lo justifica

Repetir el escenario equivalente a prueba 258:

```text
GT gobierna
ORB observado
```

Objetivo:

```text
caracterizar omega_raw / alpha_raw / residual
en hover, X, Y, Z, yaw lento y yaw rápido
```

Esto sirve para elegir/tasar parámetros basándose en distribución real.

---

## Paso 3 — repetir etapa 2 hover ORB

Nueva prueba.

Condición de éxito:

```text
handoff correcto
ORB gobierna durante TODO el escenario
sin oscilación angular creciente
sin crecimiento sistemático de ew/torque
sin fallback provocado por la propia inestabilidad
sin tracking loss atribuible al control
```

No declarar éxito porque aguante unos segundos más que la 259.

Debe completar el escenario.

---

# 20. Qué comparar directamente contra prueba 259

Codex debe preparar una tabla/resumen equivalente:

```text
                         259          nueva prueba
---------------------------------------------------
muestras ORB gobernando
primer crecimiento ew
primer crecimiento torque
max omega publicada
max raw omega
max alpha raw
max paso publicado
primer pending residual
max bias correction step
rechazos excesivos
tracking loss
fallback
scenario success
```

Y explicar causalidad, no sólo máximos.

---

# 21. Criterio para avanzar a etapa 3

Sólo avanzar si:

```text
hover ORB estable
```

y los logs muestran:

```text
no feedback positivo predictor -> torque -> movimiento -> ORB
```

Si falla hover:

```text
STOP
analizar
no ejecutar etapas posteriores
```

---

# 22. Criterio para declarar resuelto este subproblema

Este subproblema se puede considerar resuelto cuando se demuestre:

1. `GT -> ORB` sigue entrando sin salto relevante.
2. En hover, `omega_raw` pequeña no genera `omega_control` creciente.
3. Un residual absoluto persistente no se convierte en movimiento físico falso.
4. Una corrección absoluta se absorbe lentamente y de manera acotada.
5. Un giro físico real sí se sigue mediante `DeltaR_raw`.
6. `pose` y `omega` publicadas siguen siendo cinemáticamente coherentes.
7. Los cambios de KF siguen manteniendo continuidad.
8. `W` sigue sin contaminar `O`.
9. El controlador puede mantener hover ORB durante todo el escenario.
10. No hay pérdida de tracking inducida por una oscilación del lazo.

---

# 23. Hipótesis de trabajo actual

La hipótesis principal ya no es:

```text
cambio de KF
-> salto
-> caída
```

La hipótesis actual es:

```text
medida ORB / residual pequeño
        |
        v
predictor interpreta parte como dinámica física
        |
        v
omega / R_control cambian
        |
        v
controlador genera torque
        |
        v
movimiento físico de cámara
        |
        v
ORB cambia todavía más
        |
        v
predictor persigue residual creciente
        |
        v
probation confirma el residual
        |
        v
corrección todavía mayor
        |
        v
tracking perdido
```

La nueva arquitectura debe romper este lazo separando:

```text
movimiento raw entre frames
```

de:

```text
corrección absoluta del estimador
```

---

# 24. Entrega que quiero de Codex

Después de implementar, devolver:

```text
Resultado: CONSEGUIDO / PARCIAL / NO CONSEGUIDO
```

y explicar:

```text
- archivos modificados;
- arquitectura final del estimador angular;
- diferencias exactas respecto al predictor de la prueba 259;
- parámetros añadidos/eliminados;
- semántica final de SMALL/MODERATE o nuevas clases;
- tests añadidos;
- resultado total de GTests;
- builds;
- prueba/s ejecutadas;
- cronología causal de la nueva prueba;
- comparación directa con 259;
- si el hover ORB puede considerarse estable;
- si se autoriza o NO ejecutar etapa 3.
```

Actualizar también:

```text
historial_5H_RESUMEN.md
07_ULTIMA_SESION.md
```

sin borrar ni reescribir el historial real anterior.

---

# 25. Resumen ejecutivo para implementar

La modificación principal puede resumirse así:

```text
ANTES:

R_raw vs R_pred
   |
residual
   |
SMALL / MODERATE
   |
orientation_alpha * residual
   |
omega
   |
R_control


AHORA:

R_raw[n-1] -> R_raw[n]
   |
DeltaR_raw
   |
omega_raw / alpha_raw
   |
MOVIMIENTO físico estimado
   |
omega_motion
   |
   +---------------------> omega_total -> integrar R_control
   |
residual absoluto R_raw vs R_control
   |
confirmar offset
   |
corrección lenta acotada
   |
omega_bias
```

La idea clave es:

> **un offset persistente de pose no es lo mismo que una velocidad angular física persistente.**

La prueba 259 indica que el sistema actual todavía puede confundir ambas cosas.

La siguiente iteración debe corregir precisamente esa separación antes de continuar con la Fase 5H.
