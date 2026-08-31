# Fase 5H — Diagnóstico 349A/349B: aislar translación vs canal angular tras la prueba 348

## Objetivo

Determinar causalmente si la divergencia que queda en la ruta de dos fachadas nace principalmente en:

```text
p / v
R / omega
```

o si cada bloque funciona por separado pero el problema está en la **coherencia conjunta del estado**.

No aplicar todavía una nueva solución al estimador. Este bloque debe terminar con diagnóstico y `STOP`.

---

## 1. Fuente de verdad

Repositorio de referencia:

```text
https://github.com/IvanCS-Chenfu/Mapeado-con-Drones/tree/main
```

IMPORTANTE: `main` está prácticamente igual al workspace actual, pero falta la última modificación diagnóstica realizada después de 348.

Por tanto, Codex debe trabajar sobre el **workspace local real** y no resetearlo para igualarlo a GitHub.

Antes de modificar:

```bash
git status
git rev-parse HEAD
git log -1 --oneline
git diff
git diff --cached
```

Documentar qué diferencias existen frente a `origin/main` y preservar especialmente:

```text
- instrumentación causal añadida para 348;
- corrección STALE_RAW_HISTORY;
- telemetría exacta del source gate/fallback.
```

No sobrescribir cambios locales más nuevos.

---

## 2. Estado ya validado que debe quedar congelado

No reabrir sin evidencia nueva:

```text
B_T_C correcto
handoff GT -> ORB geométricamente limpio
J real
predictor angular dinámico
g_O = O_R_W * g_W congelada por map_epoch
dinámica translacional
buffers torque/thrust + ZOH + poda
MIDPOINT_DYNAMIC
hover ORB real reproducible
X corto
Y funcional cerca de pared/fiducial
Z corto
STALE_RAW_HISTORY corregido
```

No modificar durante 349:

```text
MIDPOINT_DYNAMIC
v_hat
omega estimator
omega_motion / omega_bias
predictor angular
predictor translacional
g_O
J
masa
buffers
SMALL/MODERATE/REJECT thresholds
Kp/Kv/Kr/Kw
reference_kf policy
W / optimizador
```

---

## 3. Qué demuestra 348

La prueba 348 reproduce el fallo:

```text
ORB gobierna                         41.59 s
divergencia angular comienza        +1.98 s
error lineal supera 0.1 m/s         +2.51 s
primer fallback                     +41.59 s
tracking en ese fallback            2
local_valid                         false
local_continuity_valid              false
recuperación de validez             ~100 ms
pérdida visual real                 19.54 s después
```

Además:

```text
STALE_RAW_HISTORY sigue corregido
raw history permanece sano
```

Conclusión actual:

```text
PARCIAL / MULTICAUSAL
CONTROL-ESTIMATOR COUPLING
```

con ligera precedencia angular, pero sin causalidad aislada.

---

## 4. Interpretación correcta del fallback

El pulso:

```text
local_valid=false
local_continuity_valid=false
```

**dispara** el fallback, pero no origina la inestabilidad.

La divergencia ya llevaba unos 40 s creciendo.

Por tanto, NO hacer ahora:

```text
histeresis del validity gate
ignorar local_valid
ignorar continuity_valid
eliminar TRAJECTORY_SOURCE_LOCKED
aumentar grace period
forzar ORB a seguir gobernando
```

Eso sólo permitiría que un estado ya divergente controle más tiempo.

La robustez del gate se estudiará después.

---

## 5. Opinión técnica actual

La hipótesis angular ha ganado peso porque:

```text
- yaw 338 falló;
- en 348 R/omega preceden ~0.53 s a p/v;
- errores de R/omega pueden orientar mal el thrust y contaminar p/v;
- las rutas simples funcionan mejor que movimiento + giro.
```

Pero esto **no está demostrado**.

También puede ocurrir:

```text
A. translación sea la causa real;
B. ambos canales tengan defectos independientes;
C. ambos sean aceptables individualmente pero incompatibles temporalmente juntos;
D. exista una ruta común de validez/control que degrade los dos.
```

La batería 349 debe distinguir estos casos.

---

# 6. Única modificación funcional autorizada: override diagnóstico de canales

Implementar/reutilizar un mecanismo **sólo diagnóstico** en la última frontera antes del controlador.

Arquitectura deseada:

```text
ORB-SLAM3
   ↓
NavigationState ORB completo
   ↓
mux / estado común
   ↓
OVERRIDE DIAGNÓSTICO POR CANAL
   ↓
control
```

GT NO puede entrar en:

```text
ORB-SLAM3
NavigationStateEstimator
raw history
KFs
O
v_hat
omega_hat
predictor dinámico
mapa
anclajes
```

Debe sustituir únicamente componentes del estado que finalmente consume el controlador.

---

## 7. Localizar la frontera real en el código

En el workspace actual buscar:

```bash
rg "local_continuity_valid"
rg "TRAJECTORY_SOURCE_LOCKED"
rg "tracking_lost"
rg "navigation_state"
rg "pose_source"
rg "gt_fallback"
```

Preferencia:

> reutilizar la infraestructura diagnóstica de 321A/B/C/D si todavía existe.

No crear otra arquitectura paralela si ya existe un punto seguro de sustitución.

El repositorio `main` ya contiene la ruta dinámica de `orbslam3_ros2`; no modificar sus estimadores para esta batería.

---

# 8. Modos diagnósticos

Añadir o reutilizar un parámetro explícito:

```text
NONE
ORB_PV_GT_ANGULAR
GT_PV_ORB_ANGULAR
```

### `NONE`

```text
p = ORB
v = ORB
R = ORB
omega = ORB
```

Debe seguir siendo el default y comportarse exactamente como ahora.

### `ORB_PV_GT_ANGULAR` — 349A

```text
p_control      = p_ORB_dynamic(now)
v_control      = v_ORB_dynamic(now)

R_control      = R_GT_aligned(now)
omega_control  = omega_GT_aligned(now)
```

### `GT_PV_ORB_ANGULAR` — 349B

```text
p_control      = p_GT_aligned(now)
v_control      = v_GT_aligned(now)

R_control      = R_ORB_dynamic(now)
omega_control  = omega_ORB_dynamic(now)
```

ORB debe seguir calculando los cuatro componentes internamente en ambas pruebas.

---

# 9. Coherencia temporal obligatoria

No hacer una comparación falsa del tipo:

```text
ORB(t_k) + GT(now)
```

si el controlador consume el ORB ya propagado a `now`.

Comparar/sustituir estados equivalentes:

```text
ORB_dynamic(now)
GT alineado/interpolado al mismo instante efectivo
```

Registrar:

```text
control_stamp
orb_state_stamp
gt_effective_stamp
orb_state_age
gt_alignment_skew
```

Si el skew no es suficientemente pequeño o no puede demostrarse:

```text
PRUEBA INVÁLIDA
```

---

# 10. Coherencia de frames

Las sustituciones GT deben entrar en los mismos frames/convenios que espera el controlador.

No usar directamente:

```text
pose GT world
```

si el controlador espera:

```text
O
```

Reutilizar la alineación diagnóstica ya empleada en pruebas anteriores.

Verificar explícitamente:

```text
p y v en frame correcto
R con convención correcta
omega en frame correcto
quaternion normalizado
```

---

# 11. Telemetría de fuente efectiva

Añadir:

```text
position_source=
linear_velocity_source=
orientation_source=
angular_velocity_source=
```

Valores:

```text
ORB
GT_DIAGNOSTIC
```

En 349A debe aparecer:

```text
position_source=ORB
linear_velocity_source=ORB
orientation_source=GT_DIAGNOSTIC
angular_velocity_source=GT_DIAGNOSTIC
```

En 349B:

```text
position_source=GT_DIAGNOSTIC
linear_velocity_source=GT_DIAGNOSTIC
orientation_source=ORB
angular_velocity_source=ORB
```

---

# 12. Mantener ORB completo en shadow

Aunque un canal GT llegue al controlador, seguir registrando:

```text
p_ORB
v_ORB
R_ORB
omega_ORB

visual/base/dynamic pose

v_mid
v_hat_tk
v_dynamic_now

omega_raw
omega_motion
omega_bias
omega_hat_tk
omega_dynamic_now

tracking
local_valid
local_continuity_valid
velocity_valid

raw class
correction class
reference_kf
state_age
```

Esto permitirá distinguir:

```text
error interno del estimador
```

de:

```text
error creado/amplificado por feedback del control.
```

---

# 13. No modificar todavía el fallback

Mantener:

```text
local_valid logic
local_continuity_valid logic
source gate
TRAJECTORY_SOURCE_LOCKED
```

Conservar la instrumentación de 348 que muestre el predicado exacto que provoca fallback.

Si todavía existe una etiqueta genérica:

```text
reason=tracking_lost
```

cuando `tracking=2`, se permite mejorar **sólo observabilidad**:

```text
exact_failed_predicate=LOCAL_INVALID
exact_failed_predicate=CONTINUITY_INVALID
...
```

sin cambiar la decisión de fallback.

---

# 14. Tests focales obligatorios

Añadir tests para demostrar:

```text
1. NONE no cambia la salida productiva.

2. 349A:
   p/v salen de ORB;
   R/omega salen de GT diagnóstico.

3. 349B:
   p/v salen de GT diagnóstico;
   R/omega salen de ORB.

4. El override no modifica:
   NavigationStateEstimator,
   v_hat,
   omega_hat,
   raw history,
   reference KF.

5. Quaternion normalizado y frames correctos.

6. Timestamps/skew quedan trazables.

7. El source gate y TRAJECTORY_SOURCE_LOCKED
   siguen actuando exactamente igual.

8. El modo diagnóstico no puede activarse
   accidentalmente en configuración productiva default.
```

---

# 15. Builds

Antes de simular:

```text
build orbslam3
build dron_individual
build simulacion_dron
```

y:

```text
predictor GTests
mux tests
analyzer tests
git diff --check
```

No degradar:

```text
predictor 117/117
mux 14/14
analyzer 8/8
```

o los números superiores resultantes.

---

# 16. Escenario

Usar exactamente la misma ruta de dos fachadas que reprodujo 346/348.

NO modificar:

```text
waypoints
velocidades
aceleraciones
altura
distancia a fachada
anchor
frontera GT->ORB
duraciones
```

No usar todavía la vuelta completa al edificio.

---

# 17. Prueba 349A — ORB translacional + GT angular

Ejecutar:

```text
p/v     = ORB
R/omega = GT diagnóstico
```

Pregunta causal:

> ¿El dron puede recorrer las dos fachadas si eliminamos únicamente el canal angular ORB del lazo de control?

No aceptar sólo `scenario success=true`.

Medir:

```text
RMSE/max p
RMSE/max v
RMSE/max R
RMSE/max omega

ep
ev
er
ew

F_des
tau_er
tau_ew
tau_total
energía angular

tracking
validity flags
fallback
reference KF
raw status
```

---

# 18. Prueba 349B — GT translacional + ORB angular

Sin cambiar nada más:

```text
p/v     = GT diagnóstico
R/omega = ORB
```

Pregunta causal:

> ¿El fallo se reproduce manteniendo sólo el canal angular ORB en control?

Mismas métricas y mismo YAML.

---

# 19. Timeline T0-T6

Para 349A y 349B calcular:

```text
T0 = ORB authority / inicio del tramo evaluado
T1 = primera divergencia angular
T2 = primera divergencia lineal
T3 = primer crecimiento claro de errores de control
T4 = primer validity pulse
T5 = fallback
T6 = tracking real 2 -> 3
```

Si no ocurre:

```text
NONE
```

Baseline 348:

```text
T1 angular  ~ +1.98 s
T2 lineal   ~ +2.51 s
T5 fallback ~ +41.59 s
T6 tracking ~ 19.54 s después de T5
```

---

# 20. Interpretación

### 349A funciona + 349B falla

```text
ANGULAR_CAUSAL
```

Conclusión fuerte:

```text
p/v ORB son suficientes;
R/omega ORB son necesarios para reproducir el fallo.
```

Siguiente bloque futuro: aislar `R` frente a `omega`.

### 349A falla + 349B funciona

```text
TRANSLATIONAL_CAUSAL
```

Siguiente bloque: aislar `p` frente a `v` con la arquitectura actual.

### 349A funciona + 349B funciona

```text
JOINT_STATE_COHERENCE
```

Interpretación:

> Los dos canales son suficientemente buenos por separado, pero el estado ORB completo no es coherente cuando p/v/R/omega se usan juntos.

Siguiente auditoría futura:

```text
timestamp común
horizonte lineal
horizonte angular
base state común
R usada para propagar thrust
omega usada para propagar orientación
orden de actualización
```

### 349A falla + 349B falla

```text
MULTIPLE_INDEPENDENT_ERRORS
```

o:

```text
COMMON_VALIDITY_PATH
```

No aplicar un parche doble. Analizar ambos fallos.

---

# 21. Analizar el canal ORB sustituido en shadow

Esto es muy importante.

Ejemplo 349A:

Si con angular GT el dron se vuelve estable pero:

```text
R_ORB / omega_ORB
```

siguen divergiendo en shadow:

```text
el problema angular está dentro del estimador.
```

Si al estabilizar el control:

```text
R_ORB / omega_ORB dejan de divergir
```

entonces hay evidencia fuerte de:

```text
CONTROL-ESTIMATOR COUPLING
```

Aplicar el razonamiento inverso en 349B para `p/v`.

---

# 22. Analizador comparativo

Generar una tabla:

```text
Métrica                      348       349A      349B
-----------------------------------------------------
T angular divergence
T linear divergence
RMSE p
RMSE v
RMSE R
RMSE omega
max ep
max ev
max er
max ew
energía angular
fallback time
fallback predicate
tracking loss time
local_invalid count
continuity_invalid count
reference KF changes
raw suspicious count
```

Separar por tramo de misión y no sólo máximos globales.

---

# 23. Repetibilidad

Primero:

```text
349A
349B
```

una vez cada una.

Si aparece una asimetría clara:

```text
una funciona
otra falla
```

repetir ambas sin cambios como:

```text
350A
350B
```

antes de modificar el estimador.

Si ambas funcionan o ambas fallan:

```text
STOP
```

después de 349A/349B y devolver el diagnóstico.

Si una ejecución es inválida por infraestructura, repetir como:

```text
349AR
349BR
```

después de corregir únicamente infraestructura.

---

# 24. Qué NO hacer

No:

```text
cambiar gains
bajar velocidad para que pase
subir thresholds
añadir low-pass
añadir clamps
ignorar local_valid
ignorar continuity_valid
eliminar source lock
impedir cambios de reference KF
revertir STALE_RAW_HISTORY
revertir MIDPOINT_DYNAMIC
revertir g_O
```

No usar GT más allá del override diagnóstico final.

---

# 25. Qué hacer después según resultado — sólo documentar, NO implementar

Si `ANGULAR_CAUSAL`:

```text
siguiente batería:
p/v GT común
R_ORB + omega_GT
R_GT + omega_ORB
```

Aunque existieron pruebas antiguas parecidas, el sistema actual es distinto tras corregir J, gravedad, p/v, MIDPOINT_DYNAMIC, timing, buffers y raw history.

Si `TRANSLATIONAL_CAUSAL`:

```text
angular GT común
p_ORB + v_GT
p_GT + v_ORB
```

Si `JOINT_STATE_COHERENCE`:

auditar sincronización de los cuatro componentes publicados al controlador.

Si ambas fallan:

aislar por separado cada canal sin introducir dos soluciones simultáneas.

---

# 26. Qué debe devolver Codex

Al terminar:

```text
Resultado diagnóstico:
CONSEGUIDO / PARCIAL / NO CONSEGUIDO
```

Incluir:

```text
- commit y estado exacto;
- diferencias workspace vs origin/main;
- archivos modificados;

- lugar exacto del override diagnóstico;
- confirmación de que GT no contamina ORB interno;

- builds;
- GTests;
- mux tests;
- analyzer;
- git diff --check;

- resultado 349A;
- resultado 349B;

- fuentes efectivas por canal;
- skew temporal;

- timeline T0-T6 de ambas;

- tabla 348 / 349A / 349B;

- evolución del canal ORB sustituido en shadow;

- fallback predicate exacto;
- tracking;
- validity pulses;
- raw history;

- clasificación UNA de:
  ANGULAR_CAUSAL
  TRANSLATIONAL_CAUSAL
  JOINT_STATE_COHERENCE
  MULTIPLE_INDEPENDENT_ERRORS
  COMMON_VALIDITY_PATH
  NO_CONCLUYENTE;

- 350A/350B sólo si procedió confirmación;

- siguiente modificación recomendada;

- STOP antes de implementarla.
```

Actualizar:

```text
historial_5H_RESUMEN.md
07_ULTIMA_SESION.md
```

sin borrar resultados anteriores.

---

# 27. Resumen ejecutivo

348 ya demuestra que el fallback no origina el problema:

```text
angular divergence  +1.98 s
linear divergence   +2.51 s
fallback            +41.59 s con tracking=2
real tracking loss  19.54 s después
```

El siguiente paso debe aislar los canales sobre la MISMA ruta de dos fachadas:

```text
349A
p/v = ORB
R/omega = GT

349B
p/v = GT
R/omega = ORB
```

Interpretación:

```text
349A pasa, 349B falla
→ ANGULAR

349A falla, 349B pasa
→ TRANSLACIÓN

ambas pasan
→ COHERENCIA CONJUNTA p/v/R/omega

ambas fallan
→ MULTICAUSAL / PATH COMÚN
```

No aplicar todavía una corrección del estimador.

> El éxito de este bloque consiste en demostrar qué parte del estado ORB es necesaria para reproducir la inestabilidad, no en hacer pasar artificialmente la trayectoria.
