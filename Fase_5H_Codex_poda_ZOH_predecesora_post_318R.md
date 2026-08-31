# Fase 5H — Corrección de poda ZOH conservando muestra predecesora
## Objetivo: eliminar definitivamente `MISSING_PREFIX` sin aumentar el buffer ni tocar la dinámica validada

## 0. Fuente de verdad

Trabajar sobre el estado ACTUAL del repositorio:

```text
https://github.com/IvanCS-Chenfu/Mapeado-con-Drones
```

Tomar como referencia directa:

```text
historial_5H_RESUMEN.md
prueba_318R.log
integración productiva dinámica post-317
política causal post-318
```

Mantener detenidas:

```text
319R
320
321
ORB real
```

hasta cerrar esta corrección.

---

# 1. Estado actual

La política causal post-318 ya ha demostrado:

```text
cold start cero físicamente válido
seed explícito
ZOH
cobertura explícita
persistencia ante resets visuales
```

Validaciones:

```text
builds correctos
GTests: 98/98
analizador: 8/8
```

La prueba 318R:

```text
completa el escenario
sin fallback
```

pero queda:

```text
NO CONSEGUIDA
```

porque aparece:

```text
MISSING_PREFIX ≈ 70 ms
```

La causa ya está localizada:

> La poda temporal de `0.5 s` elimina la muestra seed/predecesora durante una espera larga sin nuevos comandos. Cuando aparece después una base dinámica ligeramente anterior al primer comando conservado, el buffer ya no puede reconstruir causalmente ese pequeño prefijo.

No es un fallo del predictor.

---

# 2. Qué NO modificar

Congelar completamente:

```text
v_hat(t_k)
omega_hat(t_k)

predictor dinámico angular
predictor dinámico translacional

J
masa
gravedad

integradores
control gains

SMALL/MODERATE
KF/reference
W
mux
trayectoria
timing/jitter
```

La única modificación funcional autorizada es:

```text
política de poda de buffers ZOH
```

para torque y thrust.

---

# 3. Problema exacto de la poda actual

La poda actual elimina muestras antiguas mediante una ventana aproximada:

```text
cutoff = now - 0.5 s
```

Si se borran todas las muestras:

```text
stamp < cutoff
```

se pierde un dato esencial de una señal ZOH:

```text
la última muestra anterior al cutoff
```

Esa muestra sigue definiendo físicamente el comando aplicado al inicio de la ventana.

Ejemplo:

```text
seed:
t = 0 s
tau = 0
thrust = 0

... 32 s sin comandos ...

primer comando:
t = 32.0 s
```

Aunque el seed sea muy antiguo:

```text
u(t) = 0
```

sigue siendo válido durante toda la espera por semántica ZOH.

No debe eliminarse si no existe otra muestra posterior que lo sustituya.

---

# 4. Nueva política de poda ZOH

Para cada buffer:

```text
torque
thrust
```

usar:

```text
cutoff = now - history_window
```

y conservar:

```text
1. la ÚLTIMA muestra con stamp <= cutoff;
2. TODAS las muestras con stamp > cutoff.
```

Eliminar únicamente:

```text
las muestras anteriores a esa única predecesora.
```

Conceptualmente:

```text
... old old old PREDECESSOR | cutoff | recent recent recent
                ^ conservar
```

Después de la poda:

```text
PREDECESSOR
+
todas las recientes
```

Nunca:

```text
todo el historial antiguo
```

---

# 5. Coste de memoria esperado

No aumentar el historial a:

```text
32 s
```

ni a un horizonte indefinido.

Mantener:

```text
history_window = 0.5 s
```

o el valor vigente.

El coste adicional máximo debe ser aproximadamente:

```text
1 muestra antigua extra por buffer
```

Por tanto:

```text
1 predecesora
+
muestras de los últimos 0.5 s
```

---

# 6. Semántica correcta tras la poda

Ejemplo:

```text
t0 = seed 0
t1 = primer comando no cero
```

Si:

```text
t_base < t1 < t_target
```

la integración debe reconstruir:

```text
[t_base, t1)     -> valor del seed/predecesor
[t1, t_target]   -> nuevo comando
```

y devolver:

```text
FULL_COVERAGE
```

No `MISSING_PREFIX`.

---

# 7. Cuando la predecesora debe cambiar

La muestra predecesora no es necesariamente el seed inicial para siempre.

Ejemplo:

```text
t0: 0
t1: comando A
t2: comando B
t3: comando C
```

Cuando el cutoff avanza más allá de `t2`:

```text
última muestra <= cutoff = t2
```

entonces:

```text
t2
```

debe convertirse en la nueva muestra predecesora.

Se puede borrar:

```text
t0
t1
```

si ya no son necesarias.

El algoritmo debe conservar siempre:

> exactamente la última muestra que define el ZOH al comienzo de la ventana.

---

# 8. Torque y thrust

Aplicar EXACTAMENTE la misma política a:

```text
torque buffer
thrust buffer
```

No arreglar sólo uno.

Una predicción completa sólo tiene soporte físico si:

```text
torque coverage = FULL
AND
thrust coverage = FULL
```

---

# 9. Cold start

Mantener el seed ya demostrado:

```text
type=ZERO
reason=COLD_START_KNOWN_ZERO
```

No cambiar esta política.

El objetivo de esta iteración es:

> impedir que la poda elimine prematuramente ese seed mientras siga siendo la predecesora ZOH válida.

---

# 10. Resets visuales

Mantener la regla vigente:

```text
reset visual
!=
reset físico
```

No borrar los buffers de:

```text
torque
thrust
```

por:

```text
reference KF change
map epoch
reset de estimadores causales
reanclaje
```

salvo evento físico real que lo justifique.

---

# 11. Huecos realmente desconocidos

No convertir cualquier `MISSING_PREFIX` en cobertura artificial.

Si realmente no existe ninguna muestra capaz de definir el comando antes de:

```text
t_base
```

debe mantenerse:

```text
MISSING_PREFIX
```

La nueva política sólo debe evitar perder por poda una muestra que sí existía y seguía siendo físicamente válida.

---

# 12. GTests obligatorios

Añadir como mínimo los siguientes tests para torque Y thrust.

## A — seed antiguo + espera larga

Entrada:

```text
t0 = 0
u0 = 0

sin muestras durante 32 s
history_window = 0.5 s
```

Tras podas repetidas:

Esperado:

```text
u0 sigue presente
```

como muestra predecesora.

---

## B — seed + primer comando después de espera larga

```text
t0 = seed 0
t1 = 32.0 s, comando A
```

Predecir:

```text
t_base = 31.93
t_target = 32.02
```

Esperado:

```text
FULL_COVERAGE

31.93 -> 32.00 : 0
32.00 -> 32.02 : A
```

---

## C — sustitución de predecesora

```text
t0 = seed
t1 = A
t2 = B
```

Cuando:

```text
cutoff > t1
```

y:

```text
cutoff < t2
```

Esperado:

```text
A = nueva predecesora
seed eliminado
```

---

## D — múltiples muestras recientes

Esperado:

```text
1 predecesora
+
todas las muestras dentro de ventana
```

No perder muestras necesarias para reconstruir cambios internos.

---

## E — buffer sin predecesora real

Si el buffer empieza después de:

```text
t_base
```

y nunca existió una muestra previa:

Esperado:

```text
MISSING_PREFIX
```

No inventar cero.

---

## F — poda repetida

Ejecutar muchas podas consecutivas sin muestras nuevas.

Esperado:

```text
la misma predecesora sigue viva
```

---

## G — torque/thrust conjunto

Verificar que:

```text
torque FULL
thrust FULL
```

son independientes y ambos necesarios para declarar soporte dinámico completo.

---

# 13. Telemetría

Mantener y, si es necesario, ampliar:

```text
oldest_stamp
newest_stamp
predecessor_stamp
predecessor_value
cutoff_stamp
samples_before_prune
samples_after_prune
```

Añadir opcionalmente:

```text
[F5H-ACTUATION-PRUNE]
buffer=torque/thrust
cutoff=
predecessor_preserved=true/false
predecessor_stamp=
removed_count=
remaining_count=
```

Sólo bajo debug.

---

# 14. Prueba 318R2

Después de:

```text
build
GTests
analyzer
git diff --check
```

repetir EXACTAMENTE el escenario de 318R.

No modificar:

```text
YAML
gains
timing
J
masa
predictor
history_window=0.5 s
```

Nombre sugerido:

```text
318R2
```

---

# 15. Criterio de éxito de 318R2

Debe cumplirse:

```text
scenario success = true
llegada/hover completos

fallback = 0

F5H-DYNAMIC-MISSING = 0
MISSING_PREFIX = 0
MISSING_INTERNAL = 0

missing torque intervals = 0
missing thrust intervals = 0

torque coverage = FULL
thrust coverage = FULL
```

Y:

```text
sin NaN
sin gaps de NavigationState
sin pérdida inducida por control
```

---

# 16. Si 318R2 funciona

Repetir EXACTAMENTE:

```text
319R
```

sin tocar parámetros.

Si:

```text
318R2 = CONSEGUIDA
319R  = CONSEGUIDA
```

con:

```text
0 missing
0 fallback
FULL coverage
```

concluir:

```text
INTEGRACIÓN PRODUCTIVA DINÁMICA = VALIDADA
ACTUATION COVERAGE = VALIDADA
```

---

# 17. Sólo después: ORB real

Si 318R2 y 319R funcionan:

autorizar la siguiente prueba:

```text
320:
ORB REAL en hover
```

Ruta:

```text
cámaras
-> ORB-SLAM3
-> p/R(t_k)
-> v_hat/omega_hat
-> torque/thrust
-> dinámica completa
-> NavigationState
-> control
```

GT sólo:

```text
métricas externas
fallback temporal ante pérdida visual REAL
```

No usar GT para formar el estado normal.

---

# 18. Si 318R2 sigue mostrando `MISSING_PREFIX`

STOP.

No ejecutar 319R ni ORB.

Registrar exactamente:

```text
base_stamp
target_stamp

cutoff
predecessor_stamp

oldest torque stamp
oldest thrust stamp

first command stamp

prune history
```

Comprobar si:

```text
la predecesora se eliminó
no se insertó
se recreó el buffer
se reseteó
hay doble reloj
```

No aumentar el buffer como primera reacción.

---

# 19. Si desaparece MISSING pero cambia el comportamiento

Si:

```text
coverage = FULL
```

pero el escenario se vuelve inestable:

comparar contra 318R.

Eso indicaría:

```text
la muestra predecesora conservada no representa el comando físico real
```

o:

```text
su timestamp/semántica es incorrecta
```

No culpar automáticamente al predictor.

---

# 20. Qué NO hacer

No:

```text
aumentar history_window a decenas de segundos
```

No tocar:

```text
v_hat
omega_hat
J
masa
gains
integradores
SMALL/MODERATE
KF
W
```

No silenciar:

```text
F5H-DYNAMIC-MISSING
```

El objetivo es cobertura causal real.

---

# 21. Documentación

Actualizar:

```text
historial_5H_RESUMEN.md
07_ULTIMA_SESION.md
contrato 5H
```

si la semántica de poda ZOH pasa a formar parte del contrato.

Añadir explícitamente:

> La poda de un buffer ZOH debe conservar la última muestra anterior o igual al cutoff, porque esa muestra define el valor vigente al comienzo de la ventana.

Y:

> La ventana temporal limita el historial de cambios, no autoriza a eliminar el estado predecesor necesario para reconstruir el inicio del intervalo.

---

# 22. Qué debe devolver Codex

Al terminar:

```text
Resultado:
CONSEGUIDO / PARCIAL / NO CONSEGUIDO
```

Incluyendo:

```text
- archivos modificados;

- algoritmo exacto de poda anterior;
- algoritmo nuevo;

- política de muestra predecesora;

- cómo se aplica a torque;
- cómo se aplica a thrust;

- GTests añadidos;
- total GTests;

- builds;
- analyzer;
- git diff --check;

- resultado 318R2;

- total F5H-DYNAMIC-MISSING;
- total MISSING_PREFIX;
- total MISSING_INTERNAL;

- torque coverage;
- thrust coverage;

- scenario success;
- fallback;
- tracking;

- resultado 319R si corresponde;

- conclusión:
    ZOH PRUNE VALIDADA / NO VALIDADA
    ACTUATION COVERAGE VALIDADA / NO VALIDADA
    INTEGRACIÓN PRODUCTIVA VALIDADA / NO VALIDADA

- decisión:
    pasar a ORB real
    o mantener STOP.
```

---

# 23. Resumen ejecutivo

318R demuestra que el sistema funcional completa el escenario, pero el buffer pierde su seed/predecesora por la poda de `0.5 s`.

No ampliar el buffer.

Corregir la poda:

```text
mantener:
    última muestra <= cutoff

+
    todas las muestras > cutoff
```

Eliminar únicamente las muestras anteriores a esa predecesora.

Aplicar a:

```text
torque
thrust
```

Después:

```text
318R2
   ↓ si 0 missing
319R
   ↓ si también 0 missing
ORB REAL
```

> No volver a modificar la estimación ni la dinámica: el bloqueo actual es exclusivamente la conservación causal de la muestra predecesora ZOH durante la poda.
