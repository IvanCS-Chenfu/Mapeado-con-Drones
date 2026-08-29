# Instrucciones para Codex — Fase 5: estabilizar el estado ORB usado por el controlador

## 0. Propósito de este documento

Este documento define **qué hay que investigar, modificar, registrar y validar** antes de seguir avanzando con la Fase 5.

El problema actual no debe tratarse como un simple fallo de `GT -> ORB`, ni como un problema que se deba ocultar cambiando ganancias del controlador. Las últimas pruebas apuntan a un problema más concreto:

> El estado de control derivado de ORB puede aceptar correcciones angulares visuales moderadas como si representasen movimiento físico real del dron. Esas correcciones llegan al controlador en forma de pose/orientación y velocidad angular, el controlador reacciona a ellas correctamente, y esa reacción puede generar movimiento brusco de la cámara, degradar el tracking de ORB y terminar en pérdida o colisión.

La tarea de Codex es **reforzar el contrato dinámico del `NavigationState` producido por ORB**, manteniendo la arquitectura actual de marcos `O/W`, los cambios reales de KF de referencia y la autoridad global del servidor.

No se debe avanzar a una nueva prueba completa de dos drones hasta haber validado este comportamiento con pruebas unitarias y simulaciones progresivas.

---

# 1. Repositorio y archivos principales que hay que revisar antes de tocar código

Repositorio:

```text
https://github.com/IvanCS-Chenfu/Mapeado-con-Drones
```

Trabajar sobre el estado actual de `main` y revisar, como mínimo:

```text
dron/orbslam3_ros2/src/stereo/navigation-state-estimator.cpp
dron/orbslam3_ros2/src/stereo/navigation-state-estimator.hpp
dron/orbslam3_ros2/src/stereo/stereo-slam-node.cpp
dron/orbslam3_ros2/test/test_navigation_state_estimator.cpp

dron/dron_individual/src/control_tray/control_calcular_fuerzas.cpp

codex/pipeline/fase_5_poses_drones_sin_gt/pipeline_fase_5.md
codex/pipeline/fase_5_poses_drones_sin_gt/pipeline_fase_5_RESUMEN.md
codex/pipeline/fase_5_poses_drones_sin_gt/subfases/subfase_5H.md
codex/pipeline/fase_5_poses_drones_sin_gt/historial/
```

Antes de implementar nada, comprobar que el código sigue siendo coherente con el diagnóstico descrito aquí. Si `main` ha cambiado desde estas conclusiones, documentar claramente cualquier diferencia.

---

# 2. Qué considero ya correcto y NO quiero rediseñar

## 2.1. Separación entre pose local continua `O` y pose global `W`

La arquitectura debe seguir separando:

```text
O_T_B = pose local continua para control
W_T_B = pose global corregible
```

Las correcciones globales procedentes del servidor pueden modificar `W`, pero **no deben provocar saltos en `O`**.

No mezclar optimizaciones globales dentro del estado de control.

## 2.2. Estimador local dentro del dron

El estado de control debe seguir calculándose localmente en el dron.

No introducir una ida y vuelta al servidor en cada ciclo de control.

## 2.3. Uso de KF real de referencia + `Tcr`

Debe mantenerse la relación real entre:

```text
KF de referencia
Tcr
pose local
```

No sustituirla por:

- último KF creado,
- KF más cercano,
- asociación aproximada,
- datos de frames distintos.

## 2.4. Cambios reales de KF de referencia

ORB puede cambiar de KF de referencia y eso es normal.

No quiero una solución que intente impedir esos cambios.

La lógica existente de `NavigationStateEstimator` para conservar continuidad cuando cambia la referencia debe conservarse conceptualmente.

## 2.5. Controlador

No cambiar las ganancias del controlador para esconder el problema.

No modificar:

```text
Kp
Kv
Kr
Kw
```

salvo que aparezca posteriormente una evidencia independiente de un error real en el controlador.

El controlador está obligado a reaccionar ante una orientación estimada distinta de la deseada; si la estimación contiene un salto no físico, el problema está antes del controlador.

## 2.6. GT_FALLBACK

Mientras Fase 6 todavía no sustituya esta recuperación, `GT_FALLBACK` puede seguir existiendo como mecanismo temporal y explícito.

Pero:

```text
tracking ORB realmente perdido -> fallback puede ser correcto
tracking ORB aparentemente válido pero NavigationState inestable -> es un bug de Fase 5
```

No usar el fallback para esconder un estimador ORB incorrecto.

---

# 3. Diagnóstico concreto del código actual

## 3.1. Gate de cambio de KF

`NavigationStateEstimator` ya protege los cambios de KF de referencia.

Cuando aparece un KF nuevo, la lógica construye una referencia candidata, comprueba incrementos plausibles y espera confirmación.

Actualmente existen parámetros de este tipo:

```text
orb_reference_gate.confirmation_frames
orb_reference_gate.max_pending_frames
orb_reference_gate.max_step_translation_m
orb_reference_gate.max_step_rotation_rad
```

Esto es útil y debe conservarse.

## 3.2. Caso importante: mismo KF activo

Hay que revisar especialmente esta rama:

```cpp
if (reference_keyframe_id == reference_keyframe_id_)
```

En el código actual, cuando no hay otra referencia pendiente, la condición equivalente a:

```cpp
!pending_reference_valid_
```

hace que el retorno al KF activo sea considerado plausible sin utilizar siempre el límite geométrico de paso.

Eso significa que una modificación relevante de `Tcr` respecto al mismo KF activo puede reconstruir una nueva medida `O_T_C` y avanzar hacia el predictor.

No quiero solucionarlo simplemente imponiendo:

```text
si step_rotation > 0.08 rad -> rechazar
```

porque un movimiento físico real del dron también puede producir un cambio relevante.

Lo que quiero es que esa medida se exponga correctamente al **filtro/gate temporal de control**, sin asumir que toda diferencia representa inmediatamente dinámica física.

## 3.3. Predictor angular actual

`OrbPosePredictor` calcula aproximadamente:

```text
rotation_innovation =
Log(R_measurement * R_predicted^-1)
```

y actualmente una orientación se rechaza principalmente cuando:

```text
|rotation_innovation| > max_rotation_innovation_rad
```

con un valor actual del orden de:

```text
0.35 rad
```

Una innovación de la prueba 256 fue aproximadamente:

```text
0.125261 rad
```

por lo que queda muy por debajo de `0.35 rad` y se acepta como corrección posible.

El problema no es que `0.125 rad` sea siempre inválido.

El problema es:

> con un único frame no sabemos si esos ~7 grados representan movimiento físico real, jitter, reajuste visual, cambio relacionado con referencia o inicio de una degradación del tracking.

Por tanto, no quiero resolverlo reduciendo ciegamente `0.35 -> 0.08`.

La prueba 255 ya mostró que un filtrado/rechazo demasiado rígido puede dejar ORB gobernando durante muy poco tiempo y provocar numerosos timeouts/rechazos.

---

# 4. Solución conceptual obligatoria

Separar claramente dos responsabilidades:

```text
ORB-SLAM3
    |
    | Kref + Tcr + local pose
    v
NavigationStateEstimator
    |
    | ¿la cadena geométrica KF/Tcr es coherente?
    v
raw O_T_C / raw O_T_B
    |
    v
OrbPosePredictor / control-state gate
    |
    | ¿esta evolución temporal es físicamente creíble?
    v
O_T_B + v_O + omega_O
    |
    v
CONTROL
```

Y por separado:

```text
Servidor / GlobalPoseStore
    |
    v
W_T_KF -> W_T_B / W_T_O
```

Las correcciones de `W` nunca deben modificar la continuidad de `O`.

---

# 5. Cambio principal a implementar: tratamiento temporal de innovaciones angulares moderadas

No quiero una clasificación binaria:

```text
válida / inválida
```

basada casi únicamente en la magnitud instantánea.

Quiero, como mínimo, tres regiones conceptuales:

```text
innovación pequeña -> aceptar normalmente
innovación moderada -> confirmar temporalmente
innovación claramente imposible -> rechazar
```

Los límites exactos deben ser parámetros configurables y deben justificarse con logs.

**No fijar como verdad universal números arbitrarios.**

## 5.1. Innovación pequeña

Si la innovación es compatible con:

- la predicción actual,
- el `dt`,
- la velocidad angular previa,
- la aceleración angular permitida,
- la evolución reciente,

puede incorporarse normalmente al predictor.

## 5.2. Innovación moderada

Una innovación angular moderada NO debe:

- aceptarse automáticamente como movimiento físico,
- rechazarse automáticamente.

Debe pasar a un estado temporal de confirmación, por ejemplo:

```text
PENDING_MODERATE_ANGULAR_CORRECTION
```

Guardar como mínimo:

```text
timestamp inicial
vector de innovación
magnitud
eje/dirección
KF de referencia
si acaba de haber cambio de KF
número de frames coherentes
número de frames totales pendientes
```

Las siguientes medidas deben decidir si la corrección es consistente.

Ejemplo coherente:

```text
frame n:   +0.12 rad en un eje/dirección
frame n+1: +0.11 rad compatible
frame n+2: +0.10 rad compatible
```

Esto puede indicar una corrección persistente o movimiento real consistente.

Ejemplo no coherente:

```text
frame n:   +0.12 rad
frame n+1: +0.01 rad
frame n+2: -0.02 rad
```

Esto se parece más a una corrección visual aislada/jitter.

En el segundo caso no se debe haber hecho reaccionar violentamente al controlador al primer frame.

## 5.3. Innovación excesiva

Una innovación claramente incompatible con:

- el `dt`,
- límites dinámicos,
- historial reciente,
- coherencia geométrica,

puede seguir rechazándose.

Debe conservarse un contador de rechazos y una política de salud del predictor.

---

# 6. La confirmación NO debe mirar sólo la magnitud

La confirmación temporal debe utilizar, como mínimo:

```text
magnitud de innovación
dirección/vector de innovación
eje aproximado de giro
dt
velocidad angular estimada previamente
aceleración angular implícita
consistencia con los siguientes frames
cambio reciente o no de KF de referencia
```

Idealmente, para dos innovaciones sucesivas `r1` y `r2`, registrar algún indicador de alineación, por ejemplo:

```text
cos(angle(r1, r2))
```

o métrica equivalente robusta.

No hace falta imponer esta fórmula exacta si Codex encuentra una representación mejor, pero el criterio debe distinguir:

```text
misma corrección persistente
```

de:

```text
salto aislado que desaparece/cambia de sentido
```

---

# 7. Qué hacer cuando hay un giro físico real

No quiero un filtro que trate cualquier giro rápido como error.

Sin IMU, hay incertidumbre, así que la clave es la **coherencia temporal**.

Si el dron realmente gira y ORB produce una secuencia compatible:

```text
omega estimada previa ≈ movimiento observado
misma dirección de giro
incrementos consistentes
dt correcto
aceleración razonable
```

la corrección moderada debe confirmarse con rapidez suficiente para no introducir un retardo de control grande.

Objetivo:

```text
rechazar jitter aislado
sin congelar un giro físico real
```

Esto debe demostrarse con pruebas específicas de yaw lento y yaw rápido.

---

# 8. Cómo aplicar una corrección moderada confirmada

Una vez confirmada, no quiero un salto de pose independiente de las velocidades.

La corrección debe incorporarse a través del mismo estado dinámico que publica:

```text
pose
linear_velocity
angular_velocity
```

Mantener la idea actual de integrar la pose usando las velocidades limitadas.

Propiedad deseada:

```text
p(t+dt) ~= p(t) + v(t)*dt
R(t+dt) ~= Exp(omega(t)*dt) * R(t)
```

dentro de tolerancias medibles.

No publicar:

```text
orientación que salta
+
omega pequeña incompatible
```

ni:

```text
omega enorme
+
pose prácticamente fija
```

La pose y las velocidades deben representar el mismo estado físico estimado.

---

# 9. Post-cambio de KF: ventana de mayor sospecha, NO congelación

Después de aceptar un KF de referencia nuevo, marcar temporalmente algo equivalente a:

```text
recent_reference_switch = true
```

durante un pequeño número configurable de medidas ORB.

Esto NO debe congelar el dron ni bloquear toda actualización.

Debe servir para que:

> una innovación angular moderada inmediatamente posterior a un cambio de referencia necesite confirmación temporal antes de inyectarse plenamente en control.

La prueba 256 mostró precisamente un patrón que merece esta protección:

```text
cambio/aceptación de referencia
-> poco después innovación angular moderada
-> inestabilidad posterior
```

Registrar explícitamente cuánto tiempo/frames han pasado desde el último cambio de KF cuando aparece una innovación moderada.

---

# 10. No confundir `rotation_step` con salto realmente publicado

En los logs actuales, revisar la semántica de:

```text
rotation_step_rad
rotation_innovation_rad
```

`rotation_step_rad` puede representar distancia entre la medida ORB y el estado previo/filtrado, no necesariamente el salto final que recibe el controlador.

A partir de esta modificación quiero separar claramente en telemetría:

```text
RAW_MEASUREMENT_STEP
PREDICTION_INNOVATION
APPLIED_CORRECTION
PUBLISHED_POSE_STEP
PUBLISHED_ANGULAR_VELOCITY
```

No utilizar un único número ambiguo para concluir que el controlador recibió exactamente ese salto.

---

# 11. Logs obligatorios

Los logs deben permitir reconstruir exactamente por qué una medida se aceptó, quedó pendiente, se aplicó parcialmente o se rechazó.

No basta con:

```text
innovation=0.12 accepted
```

## 11.1. Por cada medida ORB relevante

Registrar, al menos:

```text
timestamp
dt
drone_id
map_epoch
tracking_state
reference_keyframe_id
reference_changed
frames_since_reference_change

raw_step_translation_m
raw_step_rotation_rad

position_innovation_m
rotation_innovation_vector_xyz
rotation_innovation_rad

previous_linear_velocity
previous_angular_velocity
implied_angular_velocity_from_measurement

classification:
  SMALL
  MODERATE_PENDING
  MODERATE_CONFIRMED
  MODERATE_DISCARDED
  REJECTED_EXCESSIVE

pending_correction_id o secuencia
pending_good_frames
pending_total_frames

consistency_angle/cosine o métrica equivalente

correction_fraction_applied
applied_rotation_correction_rad

linear_velocity_after_limits
angular_velocity_after_limits

published_pose_translation_step_m
published_pose_rotation_step_rad

predictor_healthy
consecutive_rejections
```

No es obligatorio que todo aparezca en una sola línea; puede existir un CSV/JSON de diagnóstico además del log ROS.

## 11.2. Publicación de `NavigationState`

Registrar o almacenar:

```text
publish_timestamp
measurement_timestamp usado
state_age
pose_source
local_valid
local_continuity_valid
velocity_valid

O_T_B
v_O
omega_O

reference_keyframe_id
tracking_state
```

## 11.3. Controlador

Durante pruebas diagnósticas registrar:

```text
timestamp
pose_source

position_error_norm
velocity_error_norm
attitude_error_norm
angular_velocity_error_norm

force
torque_norm

R_act / yaw-roll-pitch si resulta útil
omega_body

NavigationState age
```

Especial interés en correlacionar:

```text
innovación ORB
-> cambio publicado en O_T_B
-> cambio de omega
-> error de actitud
-> torque
-> movimiento real
-> tracking degraded/lost
```

## 11.4. Tracking y referencias

Registrar eventos:

```text
TRACKING_OK
RECENTLY_LOST
LOST
RECOVERED

REFERENCE_PENDING
REFERENCE_ACCEPTED
REFERENCE_REJECTED
REFERENCE_TIMEOUT

map_epoch change
global revision received
```

Con timestamps comparables al log del predictor y del controlador.

---

# 12. Qué quiero poder concluir mirando los logs

## Caso A — jitter/corrección aislada

Si aparece:

```text
innovation moderada aislada
```

quiero ver:

```text
clasificación MODERATE_PENDING
no gran published_pose_rotation_step
no gran omega artificial
no pico grande de torque atribuible a esa medida
siguientes frames no la confirman
MODERATE_DISCARDED
```

Conclusión:

```text
el filtro ha impedido convertir una corrección visual aislada en movimiento físico falso
```

## Caso B — giro físico sostenido

Durante yaw real:

```text
innovaciones coherentes
dirección consistente
omega previa y nueva compatibles
```

quiero ver:

```text
confirmación rápida
MODERATE_CONFIRMED cuando corresponda
pose y omega evolucionando juntas
sin congelación prolongada
sin rechazo repetitivo
sin source fallback innecesario
```

Conclusión:

```text
la validación temporal no destruye la dinámica real
```

## Caso C — cambio de KF

Al hacer:

```text
KF_A -> KF_B
```

quiero ver:

```text
gate geométrico de referencia funcionando
cambio de referencia sin salto importante en O
frames_since_reference_change reiniciado
si aparece innovación moderada post-switch -> PENDING
confirmación o descarte según siguientes frames
```

Conclusión:

```text
el cambio de KF sigue siendo normal y la protección temporal evita absorber inmediatamente una corrección dudosa
```

## Caso D — entrada GT -> ORB

Quiero distinguir:

```text
salto exacto en handoff
```

de:

```text
inestabilidad 0.5-5 s después
```

Registrar específicamente ventanas:

```text
t = 0
0.1 s
0.5 s
1 s
2 s
5 s
```

después de cada entrada a ORB.

Quiero comprobar:

```text
handoff inicial prácticamente sin salto
NavigationState permanece estable después
no aparecen innovaciones acumulativas que disparen torque
```

## Caso E — pérdida de tracking después de una reacción del controlador

Si ocurre una pérdida:

```text
TRACKING_OK
-> innovación ORB
-> pose/omega publicada cambia
-> aumenta attitude_error
-> pico de torque
-> movimiento brusco
-> RECENTLY_LOST/LOST
```

esto apoyaría la hipótesis de bucle positivo:

```text
corrección visual
-> reacción física
-> peor imagen
-> peor tracking
-> más inestabilidad
```

Si, en cambio:

```text
tracking se degrada antes
sin innovación ni reacción de control relevante
```

entonces hay otro problema visual/tracking separado que habrá que estudiar.

Los logs deben permitir distinguir ambos casos.

---

# 13. Tests unitarios que hay que añadir antes de simulación completa

Ampliar:

```text
dron/orbslam3_ros2/test/test_navigation_state_estimator.cpp
```

o crear tests específicos adicionales si mejora la separación.

Como mínimo:

## Test 1 — innovación pequeña

Secuencia suave.

Esperado:

```text
aceptación normal
sin pending
pose y omega coherentes
```

## Test 2 — innovación moderada aislada

Introducir un frame con innovación moderada y luego volver a la trayectoria previa.

Esperado:

```text
MODERATE_PENDING
no salto inmediato equivalente a la medida
posterior descarte
estado publicado permanece suave
```

## Test 3 — innovación moderada persistente

Varias medidas consecutivas coherentes.

Esperado:

```text
pending
confirmación
corrección gradual
sin discontinuidad
```

## Test 4 — giro físico sostenido

Generar rotación continua con velocidad angular razonable.

Esperado:

```text
el estimador sigue el giro
no se queda bloqueado
no produce timeouts por exceso de prudencia
omega publicada representa el giro
```

## Test 5 — giro más rápido pero físicamente plausible

Verificar que la lógica no equivale a un simple low-pass excesivo.

Esperado:

```text
seguimiento con retardo acotado
sin falsa clasificación persistente como outlier
```

## Test 6 — mismo KF + corrección aislada de `Tcr`

Mantener:

```text
reference_keyframe_id constante
```

e introducir una corrección angular puntual en `Tcr`.

Esperado:

```text
la cadena geométrica puede producir raw O_T_C
pero el estado de control no lo adopta inmediatamente como movimiento físico
```

Este test es especialmente importante.

## Test 7 — cambio real de KF coherente

```text
KF_A -> KF_B
```

con geometría consistente.

Esperado:

```text
referencia confirmada
continuidad de O
sin salto
```

## Test 8 — cambio de KF + innovación moderada inmediata

Tras aceptar `KF_B`, introducir corrección moderada.

Esperado:

```text
post-reference-switch marcado
corrección pasa por confirmación temporal
no se inyecta plenamente en el primer frame
```

## Test 9 — outlier grande

Esperado:

```text
rechazo
contador de rechazos
health policy correcta
```

## Test 10 — coherencia pose/omega

Para cada salida:

```text
R(t+dt)
```

debe ser compatible con la integración de:

```text
omega publicada
```

dentro de una tolerancia definida por el test.

Añadir equivalente para traslación/velocidad lineal si no está cubierto.

---

# 14. Pruebas de simulación: orden obligatorio

No saltar directamente a la prueba completa de dos drones.

## Etapa 1 — ORB observado sin gobernar el vuelo

Mover el dron mediante la fuente actualmente estable y registrar en paralelo:

```text
GT
raw ORB
raw O estimada antes del predictor
O publicada
v
omega
Kref
Tcr
innovaciones
tracking
```

Movimientos separados:

```text
hover
X
Y
Z
yaw lento
yaw rápido
giro 90 grados
giro 180 grados
```

Objetivo:

```text
caracterizar qué innovaciones produce ORB en cada maniobra
```

No ajustar umbrales sólo con una única ejecución.

## Etapa 2 — un dron gobernado por ORB en hover

Objetivo:

```text
mantener hover durante un periodo suficiente
sin oscilación creciente
sin picos repetidos de torque
sin caída de tracking provocada por el control
```

## Etapa 3 — traslación recta

Primero un eje.

Después varios ejes.

No introducir todavía el recorrido completo.

## Etapa 4 — yaw aislado

Primero lento.

Después más rápido.

Comparar:

```text
raw ORB
estado filtrado
omega
torque
tracking
```

## Etapa 5 — trayectoria simple con curva

Sólo cuando hover, rectas y yaw estén estables.

## Etapa 6 — handoff aislado GT -> ORB

Comprobar:

```text
t=0
0.1
0.5
1
2
5 s
```

No declarar éxito sólo porque el salto inicial sea cero.

## Etapa 7 — recorrido representativo, un dron

Usar la trayectoria representativa de Fase 5.

## Etapa 8 — recorrido representativo, dos drones

Sólo después de superar las etapas anteriores.

---

# 15. Parámetros

Las nuevas decisiones deben quedar parametrizadas.

Posibles conceptos de parámetros, con nombres definitivos a decidir manteniendo coherencia con el proyecto:

```text
orb_state_filter.small_rotation_innovation_rad
orb_state_filter.max_rotation_innovation_rad

orb_state_filter.moderate_confirmation_frames
orb_state_filter.moderate_max_pending_frames
orb_state_filter.moderate_direction_consistency
orb_state_filter.moderate_timeout_sec

orb_state_filter.post_reference_switch_frames
```

No asumir que todos estos nombres son obligatorios.

Lo obligatorio es que los comportamientos importantes:

```text
small
moderate pending
confirmation
timeout/discard
excessive rejection
post-reference-switch sensitivity
```

sean configurables y visibles.

No hacer tuning oculto en constantes dispersas por el código.

---

# 16. Qué NO hacer

No:

```text
bajar max_rotation_innovation_rad a un número pequeño y darlo por solucionado
```

No:

```text
filtrar la actitud GT otra vez
```

La prueba 253 ya mostró que introducir lag artificial sobre GT puede desestabilizar el lazo.

No:

```text
cambiar ganancias del controlador para que reaccione menos
```

No:

```text
mezclar correcciones W dentro de O
```

No:

```text
impedir los cambios de KF
```

No:

```text
considerar cualquier giro grande como error visual
```

No:

```text
hacer fallback a GT simplemente porque una medida moderada es incómoda
```

No:

```text
declarar éxito porque el handoff GT->ORB tenga salto cero
```

Hay que demostrar estabilidad durante los segundos posteriores.

No:

```text
volver directamente a la prueba completa de dos drones sin tests intermedios
```

---

# 17. Criterio de éxito de esta modificación

La modificación puede considerarse técnicamente correcta cuando se demuestre:

```text
1. Los cambios de KF siguen funcionando.

2. O mantiene continuidad.

3. Una corrección angular moderada aislada no produce una reacción física grande.

4. Una corrección persistente puede incorporarse gradualmente.

5. Un giro físico real sigue siendo seguido sin congelación ni retardo inaceptable.

6. Pose y omega publicadas son dinámicamente coherentes.

7. El controlador no recibe saltos de orientación incompatibles con omega.

8. GT -> ORB no sólo tiene handoff sin salto, sino estabilidad posterior.

9. Las optimizaciones/revisiones globales W no modifican O.

10. El dron puede hacer hover, traslaciones y yaw gobernado por ORB antes del recorrido completo.

11. La trayectoria representativa funciona con un dron.

12. Finalmente funciona con dos drones.
```

---

# 18. Qué debe entregar Codex después de cada iteración

Después de implementar, Codex debe indicar claramente:

```text
- archivos modificados
- motivo exacto de cada modificación
- parámetros nuevos/modificados
- tests unitarios añadidos
- resultado de build
- resultado de GTests
- comandos de simulación que debe ejecutar el usuario
- qué logs guardar
- qué patrones concretos buscar en esos logs
```

Tras recibir los logs del usuario, Codex no debe limitarse a decir "funciona/no funciona".

Debe reconstruir cronológicamente, para cada evento relevante:

```text
tracking
Kref
raw innovation
classification
pending/confirmation
correction applied
pose published
omega published
attitude error
torque
tracking posterior
```

y responder específicamente:

```text
¿la medida era aislada o persistente?
¿el predictor la trató como se esperaba?
¿la pose y omega fueron coherentes?
¿el controlador reaccionó antes o después de degradarse tracking?
¿el cambio de KF tuvo relación temporal?
¿el fallback fue consecuencia de tracking real o del propio estado inestable?
```

---

# 19. Hipótesis que queremos confirmar o descartar

La hipótesis principal de trabajo es:

```text
corrección/jitter angular ORB
        ->
estado O cambia de forma no suficientemente robusta para control
        ->
controlador genera torque correctivo
        ->
movimiento físico brusco de la cámara
        ->
empeoran features / tracking
        ->
ORB se vuelve aún menos estable
        ->
pérdida / fallback / choque
```

Esto es una hipótesis, no una conclusión definitiva.

Los nuevos logs deben permitir demostrarla o descartarla.

Si los logs muestran que:

```text
tracking empieza a degradarse claramente antes
```

de cualquier innovación, cambio de estado o reacción del controlador, entonces habrá que abrir un diagnóstico separado del tracking visual.

---

# 20. Resultado final esperado de Fase 5 en este punto

La salida ORB utilizada para control no debe ser simplemente:

```text
"la pose más reciente que ORB considera geométricamente válida"
```

Debe ser:

> **un estado local continuo, temporalmente coherente y físicamente plausible, construido a partir de ORB, cuya pose y velocidades describan conjuntamente la misma evolución del dron y que pueda cerrar el lazo de control sin convertir correcciones visuales puntuales en movimientos físicos falsos.**

Sólo después de demostrar esto se debe volver a evaluar el cierre de 5H y de la Fase 5 completa.
