# Fase 5H — Auditoría y corrección de coherencia al cambiar `reference_kf`
## Objetivo: confirmar si los cambios de KF están mezclando históricos incompatibles y reproducir el fallo con una ruta corta de dos fachadas

## 0. Fuente de verdad

Trabajar sobre el estado **ACTUAL** del repositorio:

```text
https://github.com/IvanCS-Chenfu/Mapeado-con-Drones
```

Antes de tocar código:

```text
git status
git rev-parse HEAD
git log -1 --oneline
```

Revisar especialmente:

```text
dron/orbslam3_ros2/src/stereo/navigation-state-estimator.cpp
dron/orbslam3_ros2/src/stereo/navigation-state-estimator.hpp
dron/orbslam3_ros2/src/stereo/stereo-slam-node.cpp
dron/orbslam3_ros2/src/stereo/stereo-slam-node.hpp
dron/orbslam3_ros2/test/test_navigation_state_estimator.cpp
```

y cualquier clase/archivo separado que actualmente implemente:

```text
estimador angular causal
raw motion classification
reference-KF probation
MIDPOINT_DYNAMIC
históricos de pose/rotación raw
```

Tomar como evidencia principal:

```text
prueba_342.log
prueba_342R.log
```

No asumir que el código actual coincide exactamente con el estado recordado: auditar `main` real.

---

# 1. Estado que NO debe reabrirse sin evidencia

Mantener congelado:

```text
B_T_C correcto
O/W separados

MIDPOINT_DYNAMIC
v_hat productiva

omega causal/dinámica angular
J real

g_O por map_epoch
masa
dinámica translacional

buffers torque/thrust
seed/ZOH/poda

control gains
goal semantics
mux normal
```

Ya existen pruebas anteriores que validan:

```text
hover ORB real
trayectorias cortas X/Y/Z
```

con comportamiento funcionalmente razonable.

No intentar resolver el nuevo problema cambiando otra vez:

```text
Kp/Kv/Kr/Kw
J
masa
g_O
MIDPOINT_DYNAMIC
thresholds SMALL/MODERATE
```

sin demostrar antes que alguno de ellos es causal.

---

# 2. Qué muestran 342 / 342R

La trayectoria larga alrededor del edificio presenta un patrón distinto al hover y a los desplazamientos cortos.

## 2.1. El handoff GT -> ORB NO muestra un salto geométrico grande

Ejemplo de `342`, dron 2:

```text
p_jump     = 0
r_jump     = 0
v_jump     ≈ 0.0152 m/s
omega_jump ≈ 0.00082 rad/s
```

En `342R` los handoffs siguen siendo pequeños.

Por tanto, la hipótesis de trabajo NO debe ser:

```text
GT -> ORB
    ->
salto instantáneo
    ->
caída
```

El cambio de autoridad parece razonablemente continuo.

---

# 3. Evidencia más importante de 342R

En el dron 2 aparece un cambio de referencia:

```text
ref_kf = 32
reference_changed = true
frames_since_reference_change = 0
post_reference_switch = true
```

con:

```text
dt = 0.149 s
```

pero simultáneamente:

```text
raw_dt = 32.950 s
raw_dt_quality = INVALID_DT

raw_step_translation = 3.095208 m
raw_step_rotation    = 0.550826 rad

raw_class       = SUSPICIOUS
correction      = MODERATE_DISCARDED
```

mientras la innovación respecto del estado continuo es mucho menor:

```text
position_innovation ≈ 0.012924 m
rotation_innovation ≈ 0.066876 rad
```

Esto es altamente sospechoso.

Hay otros ejemplos en 342R donde, ya después de un cambio de referencia:

```text
reference_changed = false
post_reference_switch = true
```

pero todavía aparecen:

```text
raw_dt de varios segundos
raw_step_translation ~0.8 m
raw_class=SUSPICIOUS
MODERATE_DISCARDED
```

a pesar de que el `dt` normal de las imágenes es del orden de:

```text
~0.05 s
```

---

# 4. Hipótesis principal — NO darla todavía por demostrada

La hipótesis a confirmar es:

> Algún histórico usado para calcular `raw_dt`, `raw_step_translation`, `raw_step_rotation`, `raw_omega` o la clasificación visual conserva una muestra perteneciente a `Kold` y la compara con una muestra expresada respecto de `Knew`.

Conceptualmente, sería incorrecto hacer:

```text
raw_pose(Kold, t_old)
        vs
raw_pose(Knew, t_new)
```

como si ambas poses perteneciesen al mismo frame.

Eso puede producir artificialmente:

```text
metros de raw translation
décimas de radian
raw_dt absurdamente grande
```

aunque:

```text
O_T_B
```

siga siendo continuo.

---

# 5. Pero NO confundir correlación con causalidad

Codex debe verificar primero si estos valores anómalos:

```text
raw_step
raw_dt
raw_class
MODERATE_DISCARDED
```

realmente afectan al estado productivo o son sólo telemetría.

Auditar la ruta exacta:

```text
raw pose
    ->
raw delta
    ->
raw class
    ->
correction class
    ->
base_update_applied / PREDICT_ONLY
    ->
omega_hat / R_base
    ->
estado dinámico
    ->
NavigationState
    ->
control
```

Responder:

```text
¿qué variable productiva cambia por raw_class=SUSPICIOUS?

¿MODERATE_DISCARDED impide corregir R_base?

¿cuántos frames consecutivos quedan PREDICT_ONLY?

¿la parte lineal MIDPOINT_DYNAMIC depende o no de raw_step?

¿el raw_dt inválido sólo afecta angular, también lineal o ninguno?

¿el estado publicado empieza a divergir precisamente después?
```

No arreglar un dato de log que no tenga efecto causal.

---

# 6. Aclarar una telemetría potencialmente confusa

En 342/342R aparecen líneas del tipo:

```text
linear_mode=THREE_SAMPLE_PREDICTED
linear_source=MIDPOINT_DYNAMIC
```

Esto puede ser correcto si:

```text
linear_mode
```

describe el estado interno/diagnóstico del estimador visual y:

```text
linear_source=MIDPOINT_DYNAMIC
```

describe la velocidad realmente usada productivamente.

Codex debe confirmarlo.

No declarar regresión a THREE_SAMPLE sólo por el nombre del campo.

Pero si el productivo realmente está usando THREE_SAMPLE en algún camino no esperado, documentarlo inmediatamente.

---

# 7. Invariantes correctos ante cambio de `reference_kf`

Un cambio legítimo:

```text
Kold -> Knew
```

NO debe significar un reset físico.

Deben distinguirse dos tipos de histórico.

## 7.1. Históricos expresados en `O`

Si:

```text
local_continuity_valid = true
```

pueden conservarse:

```text
O_T_B
p_O
histórico de p_O
v_O
R_O_B
estado dinámico
g_O del epoch
torque/thrust buffers
```

porque:

```text
O
```

es continuo por contrato.

## 7.2. Históricos expresados respecto de una KF concreta

NO pueden atravesar un cambio Kref sin una transformación explícita.

Ejemplos posibles:

```text
previous_raw_pose
previous_raw_rotation
previous_raw_timestamp
raw_delta_base
previous Tcr usado para raw motion
raw omega history dependiente de Kref
```

Ante:

```text
Kold -> Knew
```

hay sólo dos soluciones correctas:

```text
A. reexpresar rigurosamente la muestra anterior en el frame nuevo;
```

o, preferentemente si no hace falta conservarla:

```text
B. invalidar/resetear SÓLO ese histórico raw dependiente de Kref.
```

Nunca comparar muestras de referencias distintas directamente.

---

# 8. No resetear cosas físicas por un cambio de KF

Si se confirma el bug, NO hacer un reset indiscriminado.

No resetear:

```text
O_T_B
estado dinámico continuo
v/omega físicas si siguen válidas
g_O
torque buffer
thrust buffer
goal
trayectoria
```

El arreglo debe ser quirúrgico:

> reset/rebase sólo del histórico cuya semántica depende de la KF de referencia.

---

# 9. Semántica deseada tras un cambio de KF

Ejemplo:

```text
frame N:
ref = Kold
raw sample válida

frame N+1:
ref = Knew
```

En `N+1`:

```text
O_T_B debe seguir siendo continuo
```

pero NO debe calcularse:

```text
raw_delta(Kold -> Knew)
```

como movimiento físico.

La primera muestra con `Knew` debe:

```text
inicializar/rebasar el histórico raw de Knew
```

y marcar, por ejemplo:

```text
raw_delta_valid = false
reason = REFERENCE_REBASE
```

En la siguiente muestra que siga usando `Knew`:

```text
raw_delta_valid = true
raw_dt ≈ dt real entre imágenes
```

Sólo entonces calcular:

```text
raw_step
raw_omega
clasificación de movimiento basada en delta
```

salvo que la implementación pueda reexpresar matemáticamente la muestra antigua con garantía equivalente.

---

# 10. Timestamps deben pertenecer al mismo histórico que la pose

Muy importante:

Si se resetea/rebasa:

```text
previous_raw_pose
```

también debe resetearse/rebasarse:

```text
previous_raw_timestamp
previous_raw_reference_id
```

No permitir estados como:

```text
pose raw nueva
+
timestamp raw de hace 30 s
```

que produzcan:

```text
raw_dt = 32.95 s
```

---

# 11. Auditoría obligatoria antes de modificar

Codex debe construir una tabla con cada histórico relevante:

```text
Variable/histórico
Frame en el que vive
Timestamp asociado
Reference KF asociada
Se conserva al cambiar Kref?
Se resetea?
Se reexpresa?
Consumidor
Efecto productivo
```

Como mínimo revisar:

```text
last_o_t_camera
o_t_reference

previous raw pose
previous raw rotation
previous raw timestamp

linear position history
angular raw history
omega motion history
omega bias history

predictor base pose
predictor base velocity
predictor base omega

Tcr actual/anterior

reference probation state
post_reference_switch state
frames_since_reference_change
```

No modificar hasta entender esta tabla.

---

# 12. Telemetría nueva para demostrar el problema

Añadir sólo telemetría diagnóstica si la actual no basta.

En cada cambio de KF registrar:

```text
[F5H-REF-SWITCH-TRACE]

stamp=
epoch=

old_ref_kf=
new_ref_kf=

previous_raw_ref_kf=
previous_raw_stamp=
current_raw_stamp=

raw_history_valid_before=
raw_history_valid_after=

raw_history_action=
    KEEP / RESET / REBASE

O_translation_step=
O_rotation_step=

raw_translation_step_valid=
raw_rotation_step_valid=
raw_dt_valid=

raw_translation_step=
raw_rotation_step=
raw_dt=

linear_history_action=
angular_history_action=

base_update_type=
correction_class=
```

Y en los primeros 3-5 frames después:

```text
frames_since_reference_change
ref_kf
raw_dt
raw_step
raw_class
correction_class
base_update_type
position_innovation
rotation_innovation
p/v/R/omega publicados
```

---

# 13. PRUEBA PRINCIPAL — ruta de dos fachadas

No volver todavía a ejecutar toda la vuelta al edificio.

Crear una trayectoria diagnóstica derivada de:

```text
prueba_tipica_rodeo_edificio_dos_fiduciales.yaml
```

La nueva prueba debe reproducir el comienzo útil de esa misión pero detenerse pronto.

## Requisito geométrico

La misión debe:

```text
1. realizar la aproximación/anclaje inicial como en la prueba representativa;
2. activar ORB con la misma frontera ya validada;
3. recorrer aproximadamente DOS fachadas del edificio;
4. incluir al menos una esquina/cambio de dirección;
5. generar cambios naturales de reference KF;
6. detenerse antes de llegar al fiducial 1;
7. terminar en hover corto.
```

No añadir un fiducial artificial.

No forzar manualmente cambios de KF.

Queremos cambios naturales producidos por ORB durante una misión parecida a la real.

---

# 14. Preferencia: aislar el dron que falló

La evidencia más clara de 342R aparece en:

```text
dron_2
```

Por tanto, preferencia diagnóstica:

> reproducir la rama/dirección de trayectoria del dron 2 durante las primeras dos fachadas.

Si la infraestructura permite una prueba con un solo dron:

```text
usar sólo dron_2 / equivalente
```

para reducir ruido.

Si `multi_dron.launch.py` obliga a mantener dos:

```text
dron_2:
    ejecuta la ruta de dos fachadas

dron_1:
    permanece en hover seguro / no participa en la trayectoria diagnóstica
```

y el análisis causal se centra en dron 2.

No necesitamos todavía una prueba multi-dron completa.

---

# 15. Mantener velocidades y geometría representativas

No hacer la ruta artificialmente lenta sólo para que pase.

Reutilizar de la trayectoria original:

```text
velocidades
aceleraciones
alturas
separación respecto a fachada
yaw/heading
waypoints
```

durante el segmento equivalente.

La única simplificación debe ser:

```text
recortar la longitud total a ~2 fachadas.
```

Objetivo:

> reproducir las condiciones visuales, cambios de KF y dinámica de la vuelta real sin esperar a completar el edificio.

---

# 16. Prueba 344 — dos fachadas con GT gobernando y ORB en shadow

Esta debe ser la primera simulación.

Configurar:

```text
GT gobierna físicamente la trayectoria de dos fachadas
ORB dynamic sigue completamente activo en shadow
```

La finalidad es máxima:

> comprobar si las anomalías de `reference_kf/raw_dt/raw_step` aparecen incluso cuando el control ORB NO puede mover mal el dron.

Si aparecen:

```text
raw_dt enormes
raw_step imposibles
MODERATE_DISCARDED
PREDICT_ONLY
```

durante cambios de KF con el dron físicamente estable según GT:

> el defecto está dentro de la estimación/reference handling, no es creado inicialmente por feedback del controlador.

---

# 17. Qué medir en 344

Por cada cambio Kref:

```text
old_ref
new_ref

timestamp frame anterior
timestamp frame actual
dt real

raw previous ref
raw current ref
raw_dt

raw_step_translation
raw_step_rotation

O_step_translation
O_step_rotation

position_innovation
rotation_innovation

raw_class
correction_class
base_update_type

linear_source
angular_mode

p_ORB vs GT
v_ORB vs GT
R_ORB vs GT
omega_ORB vs GT
```

Crear un resumen por evento.

---

# 18. Criterio diagnóstico de 344

## Caso A — reproduce anomalía

Ejemplo:

```text
O step pequeño
pero
raw step enorme / raw_dt absurdo
```

en torno al cambio Kref.

Conclusión:

```text
REFERENCE_RAW_HISTORY_BUG = FUERTEMENTE CONFIRMADO
```

Continuar a corregir únicamente esa semántica.

## Caso B — no reproduce

Si todos los cambios de KF tienen:

```text
raw_dt coherente
raw_step físicamente razonable
```

con GT gobernando:

```text
STOP
```

No implementar el arreglo supuesto.

Reanalizar 342R buscando interacción con movimiento/control.

---

# 19. Modificación autorizada SÓLO si 344 + auditoría confirman el bug

Si el código demuestra que existe un histórico raw cruzando referencias incompatibles:

corregirlo.

Política recomendada:

```text
on reference switch:

    preservar estado continuo O

    preservar estado físico/dinámico

    preservar torque/thrust

    invalidar o rebasar:
        previous_raw_pose
        previous_raw_rotation
        previous_raw_timestamp
        previous_raw_reference_id
        deltas raw dependientes de Kref

    primera muestra Knew:
        inicializa raw history
        raw_delta_valid=false

    siguiente muestra misma Knew:
        raw_delta_valid=true
        raw_dt normal
```

No usar un fake delta cero como si fuese una medida física válida.

Marcar explícitamente:

```text
REFERENCE_REBASE
```

para que gates/probation sepan que no existe delta comparable todavía.

---

# 20. Interacción con `post_reference_switch`

Mantener si hace falta una ventana:

```text
post_reference_switch=true
```

pero no alimentarla con un delta inválido entre KFs.

La ventana puede seguir sirviendo para:

```text
mayor sospecha
confirmación temporal
```

pero debe trabajar sobre:

```text
deltas válidos dentro del nuevo frame / O continuo
```

no sobre una comparación Kold-Knew artificial.

---

# 21. GTests obligatorios si se modifica

Añadir al menos:

## A — cambio Kref sin movimiento físico

```text
O_T_B constante
Kold -> Knew
```

Esperado:

```text
O sin salto
raw delta inválido en primer frame nuevo
no raw_step artificial
no raw_dt antiguo
```

## B — segunda muestra con Knew

Esperado:

```text
raw delta vuelve a ser válido
raw_dt = dt entre las dos muestras Knew
```

## C — movimiento físico durante cambio Kref

El dron se mueve realmente mientras cambia referencia.

Esperado:

```text
O conserva movimiento
no se congela pose/velocidad
no se interpreta cambio Kref como movimiento adicional
```

## D — múltiples cambios consecutivos

```text
K1 -> K2 -> K3
```

sin mezclar históricos.

## E — cambio KF no borra estado físico

Comprobar que sobreviven:

```text
g_O
torque buffer
thrust buffer
estado dinámico O
```

## F — timestamps

Nunca reutilizar:

```text
previous_raw_timestamp
```

de una referencia incompatible.

## G — W independiente

Una revisión global W no debe ejecutar el mismo reset de raw history salvo que cambie realmente la referencia local relevante.

---

# 22. Prueba 345 — repetir dos fachadas en shadow tras el arreglo

Mismo YAML que 344.

Misma velocidad.

Misma configuración.

Esperado:

```text
cambios Kref naturales
O continuo

raw_dt coherente
sin raw_step artificial multimetro por Kref

no cadenas largas artificiales de:
SUSPICIOUS
MODERATE_DISCARDED
PREDICT_ONLY
```

No exigir que TODO raw step sea pequeño: el dron sí se mueve.

Exigir que sea físicamente compatible con:

```text
dt
GT
O_step
```

---

# 23. Comparación 344 vs 345

Codex debe devolver una tabla:

```text
                               344 antes      345 después
---------------------------------------------------------
nº cambios Kref
max raw_dt
p95 raw_dt
max raw translation step
max raw rotation step
raw INVALID_DT
SUSPICIOUS post-KF
MODERATE_DISCARDED post-KF
PREDICT_ONLY consecutivos
max O step
RMSE p
RMSE v
RMSE R
RMSE omega
```

Especialmente:

> separar eventos dentro de ±0.5 s de un cambio Kref del resto de la trayectoria.

---

# 24. Prueba 346 — las mismas dos fachadas gobernadas por ORB

Sólo si 345 demuestra que el arreglo es correcto.

Usar exactamente el mismo recorrido:

```text
dos fachadas
sin llegar al fiducial 1
```

pero ahora:

```text
ORB gobierna p/v/R/omega
```

después de la frontera normal de activación.

GT sólo:

```text
métricas
fallback temporal por pérdida visual REAL
```

No cambiar gains.

No cambiar velocidades entre 345 y 346.

---

# 25. Qué debe demostrar 346

Queremos observar:

```text
GT -> ORB limpio
    ↓
varios cambios naturales Kref
    ↓
O sigue continuo
    ↓
p/v/R/omega siguen acotados
    ↓
control estable
    ↓
dos fachadas completadas
```

Un fallback sólo es aceptable si:

```text
tracking se pierde primero
```

por una causa visual razonable.

No es aceptable:

```text
estado/control divergen primero
    ↓
tracking se pierde después
```

---

# 26. Repetición 347

Si 346 funciona funcionalmente:

```text
347
```

idéntica.

Sin recalibrar.

Criterio:

```text
346 = CONSEGUIDA
347 = CONSEGUIDA
```

para declarar:

```text
REFERENCE-KF HANDLING EN TRAYECTORIA = VALIDADO
```

antes de volver a la vuelta completa al edificio.

---

# 27. STOP obligatorio

Detenerse si:

```text
344 no confirma la hipótesis
```

o si:

```text
345 sigue mostrando anomalías cross-KF
```

o si:

```text
346 diverge antes de pérdida visual
```

No volver directamente a:

```text
342 completa
```

hasta explicar el fallo de la ruta corta.

---

# 28. Qué NO hacer

No:

```text
impedir cambios de reference KF
```

No fijar permanentemente una KF.

No:

```text
subir thresholds SMALL/MODERATE
```

para aceptar los saltos.

No:

```text
convertir raw_dt inválido en dt normal a mano
```

sin arreglar la semántica del histórico.

No:

```text
poner raw_step=0
```

y seguir como si fuera una medida válida.

No resetear:

```text
O
goal
g_O
buffers físicos
```

por cada cambio KF.

No cambiar:

```text
MIDPOINT_DYNAMIC
gains
J
masa
gravedad
```

durante este bloque.

---

# 29. Qué quiero ver en los logs para concluir

Para cada evento de cambio Kref, reconstruir cronológicamente:

```text
1. último frame con Kold
2. primer frame con Knew
3. segundo frame con Knew
4. 3-5 frames posteriores
```

y mostrar:

```text
timestamps
Tcr / referencia
O_T_B

raw history ref
raw history stamp

raw_dt
raw_step

innovation
classification
base update

p/v/R/omega publicados
control ep/ev/er/ew
torque
tracking
```

La pregunta central:

> ¿El primer frame Knew se compara todavía contra una muestra raw perteneciente a Kold?

Y después:

> ¿esa comparación inválida hace que se rechacen correcciones y que el estado productivo empiece a vivir demasiado tiempo en `PREDICT_ONLY`?

---

# 30. Cómo distinguir el bug de una limitación visual normal

Una limitación normal de visión estéreo puede producir:

```text
más ruido
menos precisión
tracking loss por pocos puntos
```

especialmente con puntos lejanos.

Eso NO es el mismo problema.

El bug buscado es:

```text
cambio Kref
+
O continuo
+
tracking todavía OK
+
raw delta/timestamp físicamente imposible
```

causado por una incoherencia interna de frames/históricos.

No intentar “arreglar ORB” por tener menos features.

---

# 31. Relación con la prueba completa del edificio

No volver a la vuelta completa hasta validar la ruta de dos fachadas.

La ruta corta debe contener suficiente dificultad:

```text
trayectoria real
esquina
cambio de dirección
varios KFs
cambios reference_kf
```

pero evita:

```text
muchos minutos de log
fiducial 1
segunda mitad del edificio
otros eventos posteriores que confundan causalidad
```

Una vez 346/347 funcionen:

```text
volver a la trayectoria completa
```

en una autorización posterior.

---

# 32. Builds y validaciones

Antes de la primera prueba:

```text
build orbslam3
build dron_individual
build simulacion_dron

GTests/CTest
analizador
git diff --check
```

No degradar el número actual de tests.

Después del parche:

```text
repetir builds/tests
```

y documentar diferencias.

---

# 33. Documentación

Actualizar:

```text
historial_5H_RESUMEN.md
07_ULTIMA_SESION.md
```

Actualizar contrato 5H sólo si queda demostrada/cambiada una semántica arquitectónica real.

No borrar:

```text
342
342R
```

ni reinterpretarlas como pruebas válidas/conseguidas si fallaron.

---

# 34. Qué debe devolver Codex

Al finalizar:

```text
Resultado diagnóstico:
CONSEGUIDO / PARCIAL / NO CONSEGUIDO
```

Incluir:

```text
- commit/estado inicial;
- archivos auditados;
- archivos modificados;

- tabla de históricos y frames;

- causa exacta de raw_dt/raw_step anómalos;

- confirmar o descartar:
    CROSS_REFERENCE_RAW_HISTORY

- explicar qué consumidores productivos eran afectados;

- confirmar semántica:
    linear_mode
    linear_source=MIDPOINT_DYNAMIC;

- YAML de dos fachadas creado/reutilizado;

- resultado 344;

- nº de cambios Kref;
- eventos raw inválidos alrededor de Kref;

- modificación aplicada, si estaba confirmada;

- GTests nuevos;
- total GTests;
- builds;
- analizador;
- git diff --check;

- resultado 345;
- comparación 344 vs 345;

- resultado 346 si corresponde;
- resultado 347 si corresponde;

- tracking;
- fallback;
- RMSE p/v/R/omega;
- ep/ev/er/ew;
- energía;
- missing;

- conclusión:
    REFERENCE_KF_HISTORY_BUG CONFIRMADO / DESCARTADO
    REFERENCE_KF_FIX VALIDADO / NO VALIDADO
    DOS_FACHADAS_ORB VALIDADO / NO VALIDADO

- STOP antes de volver a la vuelta completa.
```

---

# 35. Resumen ejecutivo

Las pruebas 342/342R no apuntan principalmente a un mal handoff GT->ORB.

El handoff presenta:

```text
p/r jump ~ 0
v/omega jump pequeños
```

La evidencia sospechosa aparece después, especialmente alrededor de cambios de:

```text
reference_kf
```

Ejemplo real de 342R:

```text
reference_changed=true
Kref=32

dt=0.149 s
raw_dt=32.950 s

raw_step_translation=3.095 m
raw_step_rotation=0.551 rad

pero:
position_innovation=0.0129 m
rotation_innovation=0.0669 rad
```

Eso sugiere que puede existir:

```text
histórico raw Kold
    +
muestra raw Knew
    ->
delta inválido
```

La nueva secuencia debe ser:

```text
AUDITORÍA
    ↓
344:
dos fachadas con GT gobernando + ORB shadow
    ↓
si confirma cross-KF history bug:
corrección quirúrgica
    ↓
345:
mismas dos fachadas shadow
    ↓
346:
mismas dos fachadas ORB
    ↓
347:
repetición
```

La trayectoria diagnóstica debe derivarse de la vuelta real:

```text
mismos movimientos representativos
aprox. 2 fachadas
al menos una esquina
varios cambios Kref
STOP antes del fiducial 1
```

> No hay que impedir que ORB cambie de KF. Hay que garantizar que cada histórico y cada delta se calculen en un frame temporal y geométricamente coherente cuando ese cambio ocurre.
