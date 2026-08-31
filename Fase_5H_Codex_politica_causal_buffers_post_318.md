# Fase 5H — Política causal de arranque para buffers físicos y repetición de la integración productiva
## Objetivo: eliminar `F5H-DYNAMIC-MISSING` al inicio sin inventar comandos y validar después la ruta productiva antes de ORB real

## 0. Fuente de verdad

Trabajar sobre el estado ACTUAL del repositorio:

```text
https://github.com/IvanCS-Chenfu/Mapeado-con-Drones
```

Tomar como base:

```text
historial_5H_RESUMEN.md
prueba_318.log
integración productiva post-317
StereoSlamNode dynamic path
buffers de torque/thrust sellados
```

No modificar otra vez la matemática del estimador salvo bug demostrado.

---

# 1. Estado actual

La integración productiva dinámica ya está implementada.

Validaciones actuales:

```text
build orbslam3: correcto
build dron_individual: correcto
build simulacion_dron: correcto

GTests: 94/94
analizador: 8/8
validaciones Python/YAML: correctas
git diff --check: correcto
```

La prueba 318:

```text
completa el escenario
duración ~92 s
sin fallback
```

pero registra un:

```text
F5H-DYNAMIC-MISSING
```

al arrancar un movimiento.

La causa observada:

```text
base dinámica < timestamp de la primera muestra disponible de torque
```

por tanto el buffer no puede demostrar formalmente la cobertura completa de:

```text
[base_stamp, target_stamp]
```

Aunque el escenario funcionalmente termina, según el criterio acordado:

```text
318 = NO CONSEGUIDA
```

y permanecen detenidas:

```text
319
320
ORB real
```

---

# 2. Interpretación del fallo

El problema NO es actualmente:

```text
v_hat(t_k)
omega_hat(t_k)
J
masa
modelo dinámico angular
modelo dinámico translacional
integradores
control gains
tracking
```

El problema es de:

```text
semántica causal del historial de actuación
```

Conceptualmente ocurre:

```text
base t_k
   |
   |  intervalo inicial sin muestra previa en buffer
   |
   +------------------------+
                            |
                       primera muestra
                       de torque/thrust
                            |
                            v
                         target
```

El predictor no debe inventar qué ocurrió en ese intervalo.

---

# 3. Objetivo de esta iteración

Definir una política causal correcta para que los buffers físicos puedan cubrir:

```text
t_base -> t_target
```

desde el arranque normal del sistema.

La política debe diferenciar explícitamente:

```text
COLD START
OPERACIÓN NORMAL
RESET DEL ESTIMADOR
HUECO REAL DESCONOCIDO
```

No resolver todos los casos con:

```text
"si falta algo -> asumir cero"
```

---

# 4. Política propuesta — COLD START

Antes de recibir el primer comando del controlador, verificar en el código real que:

```text
torque aplicado = 0
thrust aplicado = 0
```

es la condición física inicial real de los actuadores.

Auditar:

```text
control_calcular_fuerzas
aplicar_fuerzas_dron
motores/plugins Gazebo
```

y documentar:

```text
1. si los actuadores arrancan realmente a cero;
2. desde qué instante puede afirmarse;
3. si existe algún comando inicial previo;
4. si motor/plugin mantiene explícitamente cero hasta primer comando.
```

Sólo si queda demostrado:

> sembrar los buffers productivos al arrancar con un estado físico conocido de actuación cero.

Ejemplo conceptual:

```text
startup_stamp
tau_body = (0,0,0)
thrust = 0
source = COLD_START_KNOWN_ZERO
```

---

# 5. El seed inicial NO debe ser un parche silencioso

No añadir:

```text
if buffer empty:
    tau = 0
```

dentro del predictor.

El cero debe existir como una muestra causal explícita del buffer:

```text
timestamp
valor
origen
```

De esta forma se puede demostrar la cobertura temporal.

---

# 6. Semántica ZOH de los comandos

La actuación debe interpretarse con:

```text
zero-order hold
```

entre muestras.

Si existen:

```text
(t0, u0)
(t1, u1)
(t2, u2)
```

la semántica es:

```text
[t0, t1) -> u0
[t1, t2) -> u1
```

y así sucesivamente.

Para integrar desde:

```text
t_base
```

el buffer debe encontrar:

> la última muestra de actuación conocida con timestamp menor o igual a `t_base`.

Después aplicar sucesivamente cada cambio de comando hasta:

```text
t_target
```

---

# 7. Cobertura exacta del intervalo

Añadir una función/resultado explícito conceptualmente similar a:

```text
CoverageResult CoverInterval(
    t_base,
    t_target
)
```

que pueda devolver:

```text
FULL_COVERAGE
MISSING_PREFIX
MISSING_INTERNAL
MISSING_SUFFIX
EMPTY
```

o nombres equivalentes.

No limitar el diagnóstico a:

```text
buffer.size()
```

Queremos saber exactamente qué parte del intervalo no puede reconstruirse.

---

# 8. OPERACIÓN NORMAL

Durante operación normal:

```text
cada nuevo torque
cada nuevo thrust
```

debe guardarse con:

```text
timestamp real
valor real
```

No reemplazar el historial por una única muestra "latest".

Mantener suficiente buffer para cubrir:

```text
delay ORB
jitter
margen de seguridad
```

---

# 9. RESET DEL ESTIMADOR

Muy importante:

> Un reset del estimador visual NO implica que desaparezca la historia física del dron.

Por tanto, ante:

```text
reference KF change
reset de continuidad visual
map epoch
reanclaje
reset de CausalLinearVelocityEstimator
reset de angular estimator
```

NO borrar automáticamente:

```text
torque buffer
thrust buffer
```

salvo que exista una razón física real.

La actuación sigue existiendo aunque cambie el estimador.

---

# 10. Si por arquitectura hay que recrear un buffer

Si un nodo/componente necesita recrear su buffer durante vuelo:

NO sembrarlo con:

```text
0
```

por defecto.

Sembrarlo con:

```text
último torque físico conocido
último thrust físico conocido
timestamp correcto
```

si esa continuidad puede demostrarse.

Si no puede demostrarse:

```text
marcar MISSING
```

y no inventar.

---

# 11. HUECO REAL DESCONOCIDO

Si en operación aparece un intervalo para el que no existe información causal suficiente:

```text
F5H-DYNAMIC-MISSING
```

debe mantenerse.

No eliminar el warning simplemente para conseguir la prueba.

El objetivo es:

> conseguir cobertura real, no ocultar la falta de cobertura.

Registrar:

```text
missing_type
missing_start
missing_end
missing_duration
buffer_oldest_stamp
buffer_newest_stamp
base_stamp
target_stamp
```

---

# 12. Torque y thrust deben seguir la misma política

Aplicar el mismo contrato a:

```text
torque buffer
thrust buffer
```

No arreglar sólo el torque si el thrust puede tener el mismo problema.

El estado completo requiere:

```text
angular coverage
+
translational coverage
```

---

# 13. GTests obligatorios

Añadir como mínimo:

## A — cold start conocido a cero

Buffer sembrado en:

```text
t0
```

con:

```text
tau = 0
```

Primera muestra real en:

```text
t1 > t0
```

Predicción:

```text
t_base entre t0 y t1
```

Esperado:

```text
FULL_COVERAGE
```

y se usa cero hasta `t1`.

---

## B — primer comando no cero

```text
seed cero en t0
comando no cero en t1
target > t1
```

Esperado:

```text
[t_base,t1) -> 0
[t1,target] -> comando real
```

---

## C — arranque sin seed

Si no existe evidencia del estado inicial y buffer empieza después de base:

Esperado:

```text
MISSING_PREFIX
```

No asumir cero.

---

## D — reset del estimador con comando activo

Antes del reset:

```text
tau != 0
thrust != 0
```

Resetear estimador visual.

Esperado:

```text
buffers físicos conservados
```

y cobertura posterior válida.

---

## E — recreación de buffer con último comando conocido

Si la arquitectura requiere recreación:

```text
seed = último comando conocido
```

no cero arbitrario.

---

## F — hueco interno real

Simular pérdida de muestras que impide reconstrucción.

Esperado:

```text
MISSING_INTERNAL
```

---

## G — thrust equivalente

Repetir casos anteriores con:

```text
thrust
```

---

## H — cobertura conjunta

Sólo considerar el estado dinámico plenamente soportado si:

```text
torque_coverage == FULL
AND
thrust_coverage == FULL
```

---

# 14. Telemetría nueva

Registrar al menos:

```text
[F5H-ACTUATION-SEED]
type=ZERO / LAST_KNOWN / NONE
stamp=
tau=
thrust=
reason=
```

En cada predicción:

```text
torque_coverage
thrust_coverage

torque_oldest_stamp
torque_newest_stamp
thrust_oldest_stamp
thrust_newest_stamp

base_stamp
target_stamp

torque_samples_used
thrust_samples_used
```

Si falta cobertura:

```text
missing_prefix_sec
missing_internal_sec
missing_suffix_sec
```

---

# 15. No cambiar el predictor dinámico

Mantener congelados:

```text
J
masa
gravedad
modelo angular
modelo translacional
v_hat(t_k)
omega_hat(t_k)
integradores
```

La única modificación funcional de este bloque debe ser:

```text
semántica causal del historial de actuación
```

---

# 16. No cambiar el controlador

No tocar:

```text
Kp
Kv
Kr
Kw
```

No alterar:

```text
control/tray/fuerza
torques
mixer
```

salvo que la auditoría descubra que la hipótesis de actuadores inicialmente a cero es falsa.

Si es falsa:

```text
STOP
```

y diseñar el seed con la condición física real.

---

# 17. Prueba 318R — repetición de la integración productiva

Después de implementar:

```text
cold-start seed
ZOH
cobertura explícita
persistencia de buffers
```

repetir exactamente el escenario 318.

No cambiar:

```text
YAML
gains
J
masa
estimadores
timing/jitter
```

Nombre sugerido:

```text
318R
```

o reutilizar 318 con revisión claramente documentada.

---

# 18. Criterio de éxito de 318R

Debe cumplir simultáneamente:

```text
scenario success = true
hover/llegada completos
fallback = 0

F5H-DYNAMIC-MISSING = 0

missing torque intervals = 0
missing thrust intervals = 0

torque coverage FULL
thrust coverage FULL
```

Además:

```text
sin pérdida de tracking inducida por control
sin NaN
sin gaps de estado
```

---

# 19. Si 318R funciona

Repetir exactamente una segunda vez:

```text
319
```

sin cambios de parámetros.

Si:

```text
318R = CONSEGUIDA
319 = CONSEGUIDA
```

entonces considerar:

```text
INTEGRACIÓN PRODUCTIVA DINÁMICA = VALIDADA
```

y sólo entonces autorizar:

```text
ORB REAL
```

---

# 20. Si 318R vuelve a registrar MISSING

STOP.

No ejecutar 319.

Analizar el intervalo exacto:

```text
missing type
base
target
seed
oldest/newest sample
primer comando real
```

y determinar si el problema es:

```text
seed demasiado tardío
buffer recortado
reset inesperado
timestamp mal alineado
suscripción tardía
```

No asumir otro cero.

---

# 21. Si 318R funciona pero aparece inestabilidad

Separar:

```text
coverage correcta
```

de:

```text
estado dinámico incorrecto
```

Si ya no hay `MISSING` pero el hover empeora:

> la política de seed puede estar usando un valor físicamente incorrecto o con timestamp incorrecto.

Comparar contra la 318 original.

---

# 22. ORB real después de validación productiva

Sólo después de:

```text
318R
319
```

conseguidas.

Entonces ejecutar la prueba ORB real que estaba pendiente.

Nombre sugerido:

```text
320_ORB
```

Ruta:

```text
cámaras
-> ORB-SLAM3
-> p/R visual en t_k
-> v_hat/omega_hat causal
-> thrust/torque buffers
-> dinámica
-> NavigationState
-> control
```

GT exclusivamente:

```text
métricas externas
fallback temporal por pérdida REAL de tracking
```

No usar GT para formar estado ORB.

---

# 23. Qué NO hacer todavía

No ejecutar ORB real si:

```text
F5H-DYNAMIC-MISSING > 0
```

No avanzar a trayectorias complejas.

No modificar:

```text
SMALL/MODERATE
reference KF
W
gains
J
masa
v_hat
omega_hat
```

---

# 24. Documentación

Actualizar:

```text
historial_5H_RESUMEN.md
07_ULTIMA_SESION.md
contrato 5H
```

si se formaliza el contrato de actuación.

Añadir explícitamente al contrato:

> La propagación dinámica sólo es válida cuando torque y thrust tienen cobertura causal completa de `[t_base,t_target]`.

Y:

> Cold start puede usar seed cero únicamente si la condición física inicial cero ha sido demostrada por la cadena real de actuación.

---

# 25. Qué debe devolver Codex

Al terminar:

```text
Resultado:
CONSEGUIDO / PARCIAL / NO CONSEGUIDO
```

Incluir:

```text
- auditoría de actuadores al cold start;
- prueba de que torque/thrust iniciales son cero o descripción de la condición real;

- archivos modificados;

- política exacta de seed;
- política ZOH;
- comportamiento en reset;
- comportamiento ante hueco real;

- GTests nuevos;
- total GTests;
- builds;
- analyzer;
- git diff --check;

- resultado 318R;
- número F5H-DYNAMIC-MISSING;
- cobertura torque;
- cobertura thrust;
- missing durations;
- scenario success;
- fallback;
- tracking;

- resultado 319 si corresponde;

- conclusión:
    ACTUATION COVERAGE VALIDADA / NO VALIDADA
    INTEGRACIÓN PRODUCTIVA VALIDADA / NO VALIDADA

- decisión:
    pasar a ORB real
    o mantener STOP.
```

---

# 26. Resumen ejecutivo

La prueba 318 funcionalmente completa el escenario, pero la ruta productiva no puede considerarse validada porque aparece un hueco causal:

```text
base dinámica
<
primera muestra del buffer
```

El siguiente cambio debe garantizar una historia física válida desde el arranque.

Política:

```text
COLD START demostrado:
    seed tau=0 / thrust=0

OPERACIÓN:
    ZOH entre comandos

RESET VISUAL:
    no borrar historia física

RECREACIÓN:
    seed con último comando conocido

HUECO REAL:
    F5H-DYNAMIC-MISSING
    no inventar
```

Después:

```text
318R
    ↓ si no hay ningún missing
319
    ↓ si también funciona
ORB REAL
```

> No tocar otra vez la matemática de estimación/dinámica hasta cerrar causalmente la cobertura de los comandos físicos.
