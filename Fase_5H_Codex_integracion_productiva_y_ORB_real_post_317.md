# Fase 5H — Integración productiva del estimador dinámico completo y validación con ORB real
## Objetivo: trasladar a `StereoSlamNode` la arquitectura ya validada en laboratorio sin cambiar su comportamiento matemático

## 0. Fuente de verdad

Trabajar sobre el estado ACTUAL del repositorio:

```text
https://github.com/IvanCS-Chenfu/Mapeado-con-Drones
```

Antes de modificar nada, revisar el código vigente en `main`, especialmente:

```text
dron/orbslam3_ros2/src/stereo/stereo-slam-node.cpp
dron/orbslam3_ros2/src/stereo/stereo-slam-node.hpp
dron/orbslam3_ros2/src/stereo/navigation-state-estimator.cpp
dron/orbslam3_ros2/src/stereo/navigation-state-estimator.hpp
dron/orbslam3_ros2/test/
dron/dron_individual/src/control_tray/control_calcular_fuerzas.cpp
dron/dron_individual/src/control_tray/aplicar_fuerzas_dron.cpp
```

y localizar/reutilizar las clases creadas durante el diagnóstico:

```text
CausalLinearVelocityEstimator
predictor dinámico angular
BodyThrustDynamicPredictor
buffers sellados de torque/thrust
```

No reconstruir esos algoritmos desde cero.

---

# 1. Estado actual validado

La batería 314-317 queda `CONSEGUIDA`.

```text
314/315:
validan v_hat(t_k) causal con angular GT de laboratorio

316/317:
validan el estado completo dinámico
sin GT en ninguna componente del estado usado por control
```

Las cuatro ejecuciones completan llegada + hover sin:

```text
fallback
pérdida de tracking
huecos de thrust
```

En 316/317 el trabajo angular total permanece negativo/disipativo.

Métricas documentadas aproximadamente:

```text
RMSE p:       ~0.103 / 0.096 m
RMSE v:       ~1.184 / 1.093 m/s
RMSE angular: ~0.108 / 0.115 rad/s
```

Conclusión aceptada:

> La reconstrucción completa `p/v/R/omega(now)` con timing/jitter realista queda validada en laboratorio.

Fase 5H sigue `PARCIAL` porque la arquitectura aún no alimenta la ruta ORB productiva de `StereoSlamNode`.

---

# 2. Objetivo de esta iteración

Integrar en la ruta productiva exactamente la arquitectura ya validada:

```text
pose ORB aceptada en t_k
        |
        +--> omega_hat(t_k) causal
        +--> v_hat(t_k) causal
        |
        v
estado físico en t_k
        |
        +--> historial de torque
        +--> historial de thrust
        +--> J real
        +--> masa real
        +--> gravedad
        |
        v
propagación dinámica t_k -> now
        |
        v
p(now), v(now), R(now), omega(now)
        |
        v
orbslam/navigation_state @ 50 Hz
        |
        v
controlador
```

El objetivo NO es diseñar otro estimador.

El objetivo es trasladar a producción la misma lógica validada.

---

# 3. Regla arquitectónica principal — una sola implementación

Evitar dos implementaciones paralelas de la misma matemática:

```text
diagnóstico
productivo
```

Preferencia:

> Extraer/reutilizar las clases validadas en componentes comunes consumidos tanto por el banco diagnóstico como por `StereoSlamNode`.

Debe existir una única fuente de verdad para:

```text
CausalLinearVelocityEstimator
estimación causal angular
predictor dinámico angular
BodyThrustDynamicPredictor
buffers de torque/thrust
J
masa
```

Si ya están correctamente compartidas, no refactorizar innecesariamente.

---

# 4. Frontera productiva actual

`StereoSlamNode` ya publica:

```text
orbslam/navigation_state
```

con timer independiente a aproximadamente:

```text
50 Hz
```

mediante:

```text
PublishPredictedNavigationState()
```

y actualmente usa la predicción del `OrbPosePredictor` para llevar la última medida al timestamp objetivo.

La nueva integración debe entrar en esa frontera:

```text
medida ORB aceptada
        ->
estado base en t_k
        ->
publicación dinámica a 50 Hz
```

sin cambiar el contrato ROS externo innecesariamente.

---

# 5. No romper la separación O / W

Todo el predictor de control trabaja en:

```text
O
```

No usar:

```text
W_T_B
W_T_KF
revisiones globales
optimización global
```

para construir:

```text
p(now)
v(now)
R(now)
omega(now)
```

Las revisiones en W NO deben generar movimiento artificial en O.

---

# 6. Entrada visual productiva en t_k

Cuando TrackStereo produce una muestra válida y la cadena local ha sido aceptada, usar el MISMO sample para:

```text
map_epoch
tracking_state
reference_keyframe_id
Tcr
O_T_B(t_k)
timestamp t_k
```

A partir de esa muestra actualizar conjuntamente:

```text
posición visual aceptada
orientación visual aceptada
historial causal lineal
historial causal angular
```

No mezclar una posición de una muestra con orientación/reference/timestamp de otra.

---

# 7. omega_hat(t_k) productiva

Conservar exactamente la lógica causal angular ya validada:

```text
R(k-2), R(k-1), R(k)
        ->
omega_mid_1, omega_mid_2
        ->
alpha_hat
        ->
omega_hat(t_k)
```

Conservar los modos/semántica actuales equivalentes a:

```text
INIT
TWO_SAMPLE
THREE_SAMPLE_PREDICTED
DEGRADED_DT
REJECTED
```

No volver al filtrado antiguo ni introducir un pasa-bajos lento.

---

# 8. v_hat(t_k) productiva

Usar `CausalLinearVelocityEstimator` validado en 314-317:

```text
p(k-2), p(k-1), p(k)
        ->
v_mid_1, v_mid_2
        ->
a_hat
        ->
v_hat(t_k)
```

La estimación visual termina en `t_k`.

NO extrapolar visualmente hasta `now`.

Desde `t_k` hasta `now` se utiliza la dinámica física.

No volver a usar como velocidad física para control la antigua `linear_velocity_` mezclada con:

```text
position_alpha
position innovation
predicción previa
clamps
```

salvo que siga siendo necesaria internamente para otra función no relacionada con el estado físico publicado.

---

# 9. Estado base coherente en t_k

El estado que inicia la propagación debe ser:

```text
p(t_k)     = posición O visual aceptada
v(t_k)     = v_hat(t_k) causal
R(t_k)     = orientación O aceptada
omega(t_k) = omega_hat(t_k) causal
```

Las cuatro variables corresponden al mismo `t_k`.

---

# 10. Torque y thrust productivos

`StereoSlamNode` necesita conocer los comandos físicos aplicados desde `t_k` hasta `now`.

Reutilizar las mismas señales selladas que fueron validadas en laboratorio.

Auditar la cadena:

```text
controlador
    ->
fuerza/torque
    ->
mixer
    ->
motores
```

y confirmar:

```text
señal exacta
frame
timestamp
semántica respecto a saturaciones/mixer
```

No elegir una señal diferente de la usada en las pruebas 292-317 sin justificarlo.

---

# 11. Suscripciones y buffers

Añadir/reutilizar en `StereoSlamNode` los inputs necesarios para:

```text
tau_body(timestamp)
thrust_body_z(timestamp)
```

Los buffers deben ser:

```text
sellados por timestamp
acotados
suficientes para delay+jitter+margen
```

No guardar historial ilimitado.

No tratar el último comando recibido como si hubiera existido durante todo el intervalo cuando existe historial temporal más preciso.

---

# 12. Parámetros físicos

Mantener exactamente los parámetros ya validados.

## Inercia

```text
J = diag(
    0.00803107,
    0.00803107,
    0.015805
) kg·m²
```

## Masa

```text
m = 1.4 kg
```

Preferencia:

> una única fuente/configuración física compartida por controlador y predictores.

No volver a introducir la J nominal `diag(1e-4)`.

---

# 13. Propagación productiva t_k -> t_pub

En cada tick del timer de publicación a 50 Hz:

```text
t_pub = timestamp objetivo actual
```

partir del último estado base aceptado:

```text
p(t_k)
v(t_k)
R(t_k)
omega(t_k)
```

y reproducir los historiales de:

```text
torque
thrust
```

hasta `t_pub`.

La salida debe ser coherente:

```text
p(t_pub)
v(t_pub)
R(t_pub)
omega(t_pub)
```

Las cuatro variables deben corresponder al MISMO instante.

---

# 14. Dinámica angular

Conservar la ecuación/integración ya validada:

```text
omega_dot =
J^-1 * (tau - omega x (J*omega))
```

más integración SO(3) coherente con el frame actual.

No cambiar el integrador salvo bug demostrado.

---

# 15. Dinámica translacional

Conservar la dinámica validada:

```text
F_body = [0, 0, thrust]
```

```text
a_O = R_O_B * F_body / m + g_O
```

más la integración vigente de:

```text
v
p
```

Usar `R_dynamic(t)` durante la propagación.

No congelar orientación para transformar thrust.

---

# 16. Publicación de NavigationState

`orbslam/navigation_state` debe publicar finalmente:

```text
o_t_body          = estado dinámico en t_pub
velocity.linear   = v(t_pub)
velocity.angular  = omega(t_pub)
stamp             = t_pub
```

respetando el contrato actual.

Mantener la semántica vigente de:

```text
local_valid
local_continuity_valid
velocity_valid
tracking_state
pose_source
map_epoch
reference KF
global_valid
```

El predictor NO puede declarar tracking válido si ORB está realmente perdido.

---

# 17. Tracking y predicción son conceptos distintos

Si ORB está `TRACKING OK` pero la medida está atrasada:

```text
predictor dinámico -> compensa hasta t_pub
```

Si ORB pierde tracking realmente:

```text
mantener política Fase 5
```

incluido el `GT_FALLBACK` temporal permitido.

No usar el predictor para ocultar pérdida visual indefinidamente.

---

# 18. Cambios de reference KF

Un cambio legítimo:

```text
K_old -> K_new
```

NO debe resetear automáticamente el estado físico si `O_T_B` permanece continuo.

Los estimadores se alimentan de la pose continua en O.

Resetear historiales sólo cuando se pierda realmente la continuidad:

```text
nuevo epoch no reconciliado
reset real
tracking/estado local inválido según contrato
```

No resetear por simple cambio de ID de KF.

---

# 19. Corrección visual != velocidad física

Mantener la separación:

```text
corrección visual de pose
!=
v / omega física
```

Eventos como:

```text
SMALL
MODERATE_CONFIRMED
realineamiento de referencia
```

no deben crear impulsos artificiales de velocidad.

---

# 20. Cobertura incompleta de comandos

No ocultar intervalos sin datos.

Telemetría explícita:

```text
missing_torque_interval
missing_thrust_interval
```

Si falta cobertura necesaria para predecir hasta `t_pub`, aplicar una política segura explícita compatible con el contrato existente.

No rellenar huecos con GT en operación ORB normal.

---

# 21. Telemetría productiva obligatoria

Mantener logging focal activable con `debug_orb_control_state` o flag equivalente:

```text
measurement_stamp
arrival_stamp
publish_stamp
visual_age

map_epoch
tracking
reference_kf
reference_changed

linear_estimator_mode
angular_estimator_mode

v_hat_tk
omega_hat_tk

p_base_tk
R_base_tk

dynamic_horizon

torque_samples_used
thrust_samples_used
missing_torque_interval
missing_thrust_interval

p_dynamic_now
v_dynamic_now
R_dynamic_now
omega_dynamic_now

published p/v/R/omega

SMALL/MODERATE/REJECTED

local_valid
continuity_valid
velocity_valid
```

GT puede aparecer en simulación sólo como truth externa.

---

# 22. Paso A — integración sin recalibrar

Primero integrar la arquitectura en `StereoSlamNode`.

NO cambiar durante este paso:

```text
thresholds
gains
J
masa
integradores
SMALL/MODERATE
reference gate
```

Compilar paquetes afectados.

Ejecutar:

```text
GTests/CTest
analyzer tests
git diff --check
```

No degradar los `94/94 GTests` actuales o el número superior que exista al comenzar.

---

# 23. Paso B — prueba de paridad antes de ORB real

Antes de probar ORB real, demostrar que el refactor/integración no ha alterado la matemática validada.

Preferencia:

> El banco diagnóstico y `StereoSlamNode` deben consumir las mismas clases comunes.

Repetir una prueba equivalente a 316/317 si la integración/refactor afecta la ruta compartida.

Nombre sugerido:

```text
318A
```

Configuración:

```text
geometría GT diagnóstica
mismo timing/jitter
mismos predictores compartidos
estado completo dinámico
```

Resultado esperado:

```text
hover completo
sin fallback
sin huecos torque/thrust
```

Si 318A falla:

```text
STOP
```

No ejecutar ORB real.

---

# 24. Prueba 318 — ORB REAL en hover

Sólo después de builds/tests/paridad correctos.

Ruta real:

```text
cámaras
    ->
ORB-SLAM3
    ->
O_T_B visual
    ->
v_hat(t_k)
omega_hat(t_k)
    ->
dinámica thrust/torque
    ->
p/v/R/omega(t_pub)
    ->
NavigationState
    ->
control
```

GT NO puede aportar al control:

```text
p
v
R
omega
```

GT sólo puede ser:

```text
métrica externa
fallback temporal si ORB pierde tracking realmente
```

Objetivo:

> hover completo gobernado por ORB sin caída inducida por control.

---

# 25. Métricas obligatorias de 318

Registrar:

```text
tiempo ORB gobernando
fallback count
fallback causes

tracking transitions
epoch changes
reference KF changes

raw translation step
raw rotation step

v_hat(t_k)
omega_hat(t_k)

dynamic horizon

torque/thrust coverage

ep
ev
er
ew

F_des
tau_total

energía angular
```

Y contra GT exclusivamente como truth:

```text
RMSE p
RMSE v
RMSE R
RMSE omega
```

Separar métricas de vuelo/control activo del contacto inicial con suelo.

---

# 26. Interpretación de 318

## Caso A — completa el hover

```text
ORB REAL HOVER = CONSEGUIDO
```

Repetir exactamente como:

```text
319
```

No modificar parámetros entre 318 y 319.

## Caso B — inestabilidad antes de tracking loss

Si ocurre:

```text
control ORB
-> errores crecen
-> oscilación física
-> tracking loss después
```

resultado:

```text
NO CONSEGUIDO
```

La pérdida de tracking es consecuencia.

Analizar calidad/continuidad de la pose ORB real.

## Caso C — tracking loss primero por visión

Si ocurre:

```text
estado estable
-> pérdida visual real
-> tracking non-OK
-> fallback limpio
```

puede ser comportamiento esperado de Fase 5.

Documentar que no existía inestabilidad previa inducida por control.

---

# 27. Prueba 319 — repetición ORB real

Sólo si 318 completa.

Mismos:

```text
YAML
mundo
gains
estimadores
parámetros
```

Criterio:

```text
318 = CONSEGUIDA
319 = CONSEGUIDA
```

para declarar:

```text
HOVER ORB REAL REPRODUCIBLE
```

---

# 28. Si 318/319 funcionan

No pasar directamente al recorrido completo.

El siguiente bloque deberá introducir movimiento progresivamente, por ejemplo:

```text
320: desplazamiento X corto
321: desplazamiento Y corto
322: desplazamiento Z corto
323: yaw lento
324: combinación/curva sencilla
```

No ejecutar este bloque todavía sin nueva autorización.

---

# 29. Si 318 falla pero 318A funciona

Conclusión importante:

```text
timing/jitter                 VALIDADO
dinámica angular              VALIDADA
dinámica translacional        VALIDADA
integración matemática común  VALIDADA
```

Por tanto, el foco debe pasar a:

```text
pose ORB real
reference KF
SMALL/MODERATE
raw gates
tracking
```

NO volver automáticamente a tocar:

```text
J
masa
torque predictor
thrust predictor
v_hat
omega_hat
gains
```

---

# 30. GTests requeridos

Añadir/asegurar tests focales para:

```text
1. StereoSlamNode usa v_hat causal como condición inicial física.

2. StereoSlamNode usa omega_hat causal.

3. torque buffer aplica muestras por timestamp correcto.

4. thrust buffer aplica muestras por timestamp correcto.

5. p/v/R/omega publicados corresponden al mismo t_pub.

6. cambio normal de reference KF no rompe continuidad.

7. nuevo epoch/invalidez real resetea históricos cuando corresponde.

8. ausencia de torque/thrust queda explícita.

9. ninguna ruta productiva consulta GT.

10. J y masa son las compartidas validadas.

11. una corrección visual no crea impulso ficticio en v/omega.

12. el publish timer continúa a 50 Hz independientemente del ritmo ORB.
```

---

# 31. Qué NO hacer

No cambiar:

```text
Kp
Kv
Kr
Kw

J
masa

SMALL
MODERATE

reference gate

W / optimización global

goal semantics

mux
```

No implementar:

```text
EKF
nuevo low-pass
Delta_target
otro predictor
```

No optimizar RMSE antes de comprobar ORB real.

La prioridad es:

```text
integración correcta
+
estabilidad funcional
```

---

# 32. Contrato final deseado

Cada `NavigationState` productivo debe representar un único instante `t_pub`:

```text
O_T_B(t_pub)
v_O(t_pub)
omega_O(t_pub)
stamp = t_pub
```

Entre observaciones visuales:

```text
state(t+dt)
≈
Integrate(state(t), thrust/torque, dt)
```

ORB corrige periódicamente el estado.

La dinámica cubre hasta el presente.

No son dos fuentes independientes de pose.

---

# 33. Qué debe devolver Codex

Al terminar:

```text
Resultado:
CONSEGUIDO / PARCIAL / NO CONSEGUIDO
```

Incluir:

```text
- commit/estado inicial;
- archivos modificados;

- arquitectura compartida final;
- clases reutilizadas/refactorizadas;

- topics/señales exactas de torque y thrust;
- QoS;
- frames;
- timestamps;
- horizonte de buffers;

- J usada;
- masa usada;

- cómo se forma el estado base en t_k;
- cómo se propaga hasta t_pub;

- confirmación de ausencia de GT en ruta productiva;

- builds;
- GTests;
- analyzer tests;
- git diff --check;

- resultado 318A/paridad;

- resultado 318 ORB real si corresponde;
- tiempo gobernado por ORB;
- fallback y causas;
- tracking;
- epochs;
- reference KF;
- huecos torque/thrust;
- RMSE p/v/R/omega;
- energía angular;
- hover success;

- resultado 319 si corresponde;

- conclusión:
    INTEGRACIÓN PRODUCTIVA VALIDADA / NO VALIDADA
    ORB REAL HOVER VALIDADO / NO VALIDADO
    ORB REAL REPRODUCIBLE / NO REPRODUCIBLE

- siguiente paso recomendado.
```

Actualizar:

```text
historial_5H_RESUMEN.md
07_ULTIMA_SESION.md
contrato 5H
```

si cambia el contrato productivo.

No borrar historial anterior.

---

# 34. Resumen ejecutivo

El laboratorio ya ha demostrado:

```text
omega_hat(t_k) causal               ✅
v_hat(t_k) causal                   ✅

torque + J -> R/omega(now)          ✅
thrust + m + g -> p/v(now)          ✅

estado completo con timing/jitter   ✅
repetición                          ✅
```

El siguiente trabajo NO es inventar otro estimador.

Es:

```text
1. integrar exactamente esa arquitectura en StereoSlamNode;

2. asegurar que diagnóstico y productivo comparten la misma implementación;

3. validar paridad;

4. ejecutar ORB real en hover;

5. repetir ORB real si funciona.
```

Secuencia:

```text
318A:
paridad integración productiva
        ↓ si funciona

318:
ORB REAL hover
        ↓ si funciona

319:
repetición ORB REAL hover
```

> Si la paridad funciona pero ORB real falla, dejar de tocar la compensación temporal y centrar el diagnóstico en la calidad de la medida ORB real y su continuidad geométrica.
