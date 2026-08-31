# Fase 5H — Corrección de infraestructura temporal para 349A/349B
## Objetivo: validar el override diagnóstico con `max_gt_alignment_skew=30 ms` sin introducir retardo artificial y completar la batería causal translación vs angular

## 0. Fuente de verdad

Trabajar sobre el estado ACTUAL del workspace.

Repositorio de referencia:

```text
https://github.com/IvanCS-Chenfu/Mapeado-con-Drones/tree/main
```

Logs relevantes:

```text
prueba_349A.reduced.log
prueba_349AR.reduced.log
prueba_349AR2.reduced.log
```

IMPORTANTE:

> `349A`, `349AR` y `349AR2` NO deben considerarse pruebas causales válidas todavía, porque el override angular GT no se aplicó de forma continua durante todo el tramo ORB.

No resetear cambios locales.

Antes de modificar:

```bash
git status
git rev-parse HEAD
git log -1 --oneline
git diff
git diff --cached
```

---

# 1. Qué ha ocurrido

El override diagnóstico ya existe y selecciona correctamente los canales cuando consigue una muestra GT temporalmente válida.

Para 349A queremos:

```text
p/v     = ORB
R/omega = GT diagnóstico
```

Sin embargo, en 349AR2 el soporte GT causal sólo quedó disponible en una parte de las publicaciones ORB posteriores al handoff.

Resultado resumido informado:

```text
override aplicado:
2494 / 6407 publicaciones ORB posteriores al handoff
```

En las demás:

```text
gt_alignment_skew > límite actual
```

o no existía soporte causal aceptable dentro de:

```text
20 ms
```

y el sistema volvía a:

```text
R/omega = ORB
```

Por tanto el experimento alternaba:

```text
ORB p/v + GT angular
```

con:

```text
ORB completo
```

y queda contaminado.

---

# 2. Evidencia temporal observada

Las muestras válidas muestran repetidamente:

```text
gt_alignment_skew ≈ 10 ms
gt_alignment_skew ≈ 20 ms
```

con:

```text
gt_causal_propagation=true
```

Es decir:

> el límite de 20 ms está prácticamente encima de la fase natural entre ambos flujos.

Pequeño jitter/scheduling hace que una muestra equivalente pase de:

```text
~19.99 ms
```

a algo ligeramente superior al límite y sea descartada.

No se ha demostrado un desfase físico de 80-100 ms en esta infraestructura diagnóstica.

---

# 3. Decisión acordada

Autorizar únicamente:

```text
max_gt_alignment_skew:
20 ms -> 30 ms
```

para esta infraestructura diagnóstica.

NO retardar deliberadamente todavía la salida del estado.

NO rediseñar aún el flujo con interpolación futura.

Razón:

> 30 ms es una ampliación mínima destinada a cubrir un periodo normal + jitter de scheduling, manteniendo la propagación causal hasta el `control_stamp`.

---

# 4. Semántica temporal que debe mantenerse

El objetivo NO es consumir un GT 30 ms antiguo directamente.

La ruta debe seguir siendo:

```text
muestra GT causal previa
        ↓
propagación causal
        ↓
GT efectivo en control_stamp
```

La telemetría debe seguir mostrando:

```text
gt_effective_stamp ≈ control_stamp
```

cuando:

```text
gt_causal_propagation=true
```

El parámetro de 30 ms define:

```text
máxima separación permitida para encontrar soporte causal
```

no:

```text
latencia añadida al controlador.
```

---

# 5. Regla importante: 30 ms NO es un threshold para seguir ampliando

No aplicar esta secuencia:

```text
30 ms falla
→ 40 ms
→ 50 ms
→ 100 ms
→ ...
```

30 ms es el último aumento simple autorizado.

Si sigue habiendo una proporción apreciable de misses:

```text
STOP
```

y el siguiente trabajo deberá estudiar explícitamente:

```text
buffer temporal
interpolación
o retardo controlado
```

pero NO bajo esta autorización.

---

# 6. Modificación

Modificar exclusivamente el parámetro diagnóstico:

```text
max_gt_alignment_skew_sec = 0.030
```

o equivalente.

Preferencia:

```text
parámetro ROS/launch diagnóstico configurable
```

No hardcodear el valor si ya existe una configuración equivalente.

Mantener default productivo:

```text
sin override diagnóstico
```

---

# 7. No modificar

No tocar:

```text
MIDPOINT_DYNAMIC
v_hat
omega estimator
omega_motion
omega_bias

predictor angular
predictor translacional

g_O
J
masa

raw history / STALE_RAW_HISTORY fix

SMALL
MODERATE
REJECT thresholds

Kp
Kv
Kr
Kw

reference KF
W
source lock
fallback policy
```

---

# 8. Telemetría obligatoria

Para cada publicación ORB posterior al handoff registrar:

```text
override_requested=
override_valid=
override_applied=

control_stamp=
orb_state_stamp=
gt_effective_stamp=

gt_alignment_skew=
gt_interpolated=
gt_causal_propagation=

position_source=
linear_velocity_source=
orientation_source=
angular_velocity_source=
```

---

# 9. Métricas nuevas de cobertura

El analizador debe calcular para el tramo donde el override DEBE aplicarse:

```text
override_requested_count
override_valid_count
override_applied_count
override_missed_count

override_applied_ratio

max_consecutive_override_misses
```

Además:

```text
gt_alignment_skew_mean
gt_alignment_skew_p50
gt_alignment_skew_p95
gt_alignment_skew_p99
gt_alignment_skew_max
```

y contadores:

```text
gt_interpolated_count
gt_causal_propagation_count
```

---

# 10. Definir correctamente el denominador

No calcular el ratio sobre toda la simulación.

El denominador debe incluir únicamente publicaciones donde:

```text
- ya ocurrió el handoff;
- ORB es la fuente que se está diagnosticando;
- el modo de override solicitado es 349A/349B;
- la prueba está dentro del tramo evaluado.
```

No contar:

```text
startup
aproximación GT
pre-anchor
shadow previo al handoff
```

como misses del override.

---

# 11. Criterio de infraestructura válida

Para aceptar el experimento:

```text
override_applied_ratio >= 99.5 %
```

Preferencia:

```text
≈ 100 %
```

y:

```text
max_consecutive_override_misses = 0
```

Idealmente.

Se puede tolerar algún miss aislado sólo si:

```text
- está claramente identificado;
- no crea alternancia sostenida;
- no coincide con el inicio de una divergencia;
- el ratio global sigue >= 99.5 %.
```

Si no:

```text
PRUEBA INVÁLIDA
```

---

# 12. Prueba 349AR3 — validación de infraestructura y 349A real

Repetir EXACTAMENTE:

```text
tray_prueba_346.yaml
```

con:

```text
p/v     = ORB
R/omega = GT diagnóstico
```

y:

```text
max_gt_alignment_skew = 30 ms
```

No cambiar:

```text
waypoints
velocidades
aceleraciones
altura
anchor
gains
estimadores
```

---

# 13. Primer criterio de 349AR3

ANTES de interpretar si el dron va bien o mal, comprobar:

```text
override_applied_ratio
max_consecutive_override_misses
skew distribution
```

Si:

```text
override_applied_ratio < 99.5 %
```

entonces:

```text
349AR3 = INVÁLIDA
STOP
```

No ejecutar 349B.

No aumentar más el skew.

---

# 14. Si 349AR3 alcanza cobertura válida

Entonces 349AR3 pasa a ser la primera ejecución **causalmente válida de 349A**.

Durante el tramo evaluado debe verse prácticamente siempre:

```text
position_source=ORB
linear_velocity_source=ORB

orientation_source=GT_DIAGNOSTIC
angular_velocity_source=GT_DIAGNOSTIC
```

No una alternancia frecuente con:

```text
orientation_source=ORB
angular_velocity_source=ORB
```

---

# 15. Métricas funcionales de 349AR3

Si la infraestructura es válida, medir:

```text
RMSE/max p_ORB vs GT
RMSE/max v_ORB vs GT

R_ORB vs GT en shadow
omega_ORB vs GT en shadow

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
local_valid
local_continuity_valid
velocity_valid
fallback
reference KF
raw state
```

---

# 16. Pregunta causal de 349AR3

Responder:

> ¿El dron completa las dos fachadas cuando p/v siguen siendo ORB pero R/omega usados por el control son GT temporalmente alineados?

Además:

> ¿R_ORB/omega_ORB divergen internamente en shadow aunque ya no controlen el dron?

Esto permite distinguir:

```text
estimador angular intrínsecamente malo
```

de:

```text
control-estimator coupling.
```

---

# 17. Sólo después: prueba 349B

Ejecutar 349B únicamente si:

```text
349AR3 infraestructura = VÁLIDA
```

Configuración:

```text
p/v     = GT diagnóstico
R/omega = ORB
```

Usar la MISMA política temporal:

```text
max_gt_alignment_skew = 30 ms
```

y exigir exactamente:

```text
override_applied_ratio >= 99.5 %
```

para los canales GT translacionales.

---

# 18. Prueba 349B — fuentes esperadas

Durante el tramo ORB:

```text
position_source=GT_DIAGNOSTIC
linear_velocity_source=GT_DIAGNOSTIC

orientation_source=ORB
angular_velocity_source=ORB
```

GT no debe entrar en el estimador ORB.

---

# 19. Interpretación conjunta

## Caso A

```text
349AR3 funciona
349B falla
```

Conclusión:

```text
ANGULAR_CAUSAL
```

El canal:

```text
R/omega ORB
```

es necesario para reproducir el fallo.

---

## Caso B

```text
349AR3 falla
349B funciona
```

Conclusión:

```text
TRANSLATIONAL_CAUSAL
```

El canal:

```text
p/v ORB
```

es necesario para reproducir el fallo.

---

## Caso C

```text
349AR3 funciona
349B funciona
```

Conclusión:

```text
JOINT_STATE_COHERENCE
```

Los canales son suficientemente buenos por separado, pero juntos:

```text
p/v/R/omega
```

no representan de forma coherente el mismo estado físico/temporal.

---

## Caso D

```text
349AR3 falla
349B falla
```

Conclusión:

```text
MULTIPLE_INDEPENDENT_ERRORS
```

o:

```text
COMMON_VALIDITY_PATH
```

No implementar todavía una solución doble.

---

# 20. Repetibilidad

Si aparece asimetría clara:

```text
A pasa
B falla
```

o:

```text
A falla
B pasa
```

repetir sin cambios como:

```text
350A
350B
```

antes de modificar el estimador.

No recalibrar entre ejecuciones.

---

# 21. Si 30 ms no alcanza cobertura

Si 349AR3 sigue mostrando:

```text
override_missed_count alto
```

o:

```text
applied_ratio < 99.5 %
```

concluir:

```text
CURRENT_CAUSAL_PROPAGATION_INFRASTRUCTURE_INSUFFICIENT
```

STOP.

No ejecutar 349B.

No subir más el skew.

Siguiente diseño a debatir:

```text
A. buffer temporal común

B. interpolación bracketed

C. retardo deliberado y cuantificado
```

Pero no implementarlo todavía.

---

# 22. Por qué no retardar ahora la salida

No elegir todavía:

```text
retardar controller/output para esperar interpolación completa
```

porque eso introduciría:

```text
latencia artificial nueva
```

en una fase donde ya se han diagnosticado problemas de:

```text
fase temporal
pose/omega
20 -> 50 Hz
delay/jitter
```

Primero comprobar si 30 ms resuelve simplemente el borde de scheduling observado.

---

# 23. Tests

Añadir/actualizar tests de la infraestructura:

```text
20 ms:
caso >20 ms debe quedar inválido

30 ms:
muestra causal 20-30 ms debe poder propagarse

>30 ms:
debe seguir siendo inválida

propagación:
gt_effective_stamp == control_stamp

NONE:
sin cambio productivo

349A:
p/v ORB
R/omega GT

349B:
p/v GT
R/omega ORB
```

No usar estos tests para modificar estimadores.

---

# 24. Builds

Ejecutar:

```text
build orbslam3
build dron_individual
build simulacion_dron
```

y:

```text
predictor tests
mux tests
analyzer tests
git diff --check
```

No degradar los números vigentes.

---

# 25. Qué NO hacer

No:

```text
subir skew >30 ms

retardar deliberadamente el control
sin que 30 ms haya fallado primero

cambiar gains

cambiar estimadores

ignorar misses y considerar la prueba válida

aceptar success=true como criterio causal

mezclar ORB/GT alternadamente
y extraer conclusiones

ejecutar vuelta completa
```

---

# 26. Qué debe devolver Codex

Al terminar:

```text
Resultado:
CONSEGUIDO / PARCIAL / NO CONSEGUIDO
```

Incluir:

```text
- commit/estado inicial;
- diferencias workspace/origin-main;
- archivos modificados;

- parámetro anterior:
  20 ms

- parámetro nuevo:
  30 ms

- explicación de la propagación causal;

- tests;
- builds;
- git diff --check;

- resultado 349AR3;

- requested_count;
- applied_count;
- missed_count;
- applied_ratio;
- max consecutive misses;

- skew mean/p50/p95/p99/max;

- causal propagation count;
- interpolation count;

- confirmación de fuentes efectivas por canal;

- clasificación:
  349AR3 VÁLIDA / INVÁLIDA;

- si válida:
  resultado funcional de 349A;

- resultado 349B sólo si procedía;
- cobertura temporal 349B;

- tabla 348 / 349A válida / 349B;

- clasificación causal:
  ANGULAR_CAUSAL
  TRANSLATIONAL_CAUSAL
  JOINT_STATE_COHERENCE
  MULTIPLE_INDEPENDENT_ERRORS
  COMMON_VALIDITY_PATH
  NO_CONCLUYENTE;

- resultado 350A/350B si hubo repetición;

- siguiente paso recomendado;

- STOP antes de aplicar la solución al estimador.
```

Actualizar:

```text
historial_5H_RESUMEN.md
07_ULTIMA_SESION.md
```

sin borrar intentos inválidos anteriores.

---

# 27. Resumen ejecutivo

349AR2 no es válida porque el override temporal sólo se aplica en una parte de las publicaciones y alterna:

```text
R/omega GT
↔
R/omega ORB
```

El límite actual:

```text
20 ms
```

está prácticamente sobre la fase natural observada:

```text
~10-20 ms
```

Por tanto:

```text
1. ampliar sólo el skew diagnóstico:
   20 -> 30 ms

2. mantener propagación causal
   hasta control_stamp

3. ejecutar 349AR3

4. exigir:
   override_applied_ratio >= 99.5 %
   idealmente ~100 %

5. si falla cobertura:
   STOP
   no subir más el threshold

6. si 349AR3 es válida:
   interpretarla como 349A válida

7. ejecutar después 349B

8. comparar causalmente:
   p/v vs R/omega
```

> La meta de esta iteración es arreglar únicamente la validez temporal del experimento. No debe confundirse con una corrección del estimador ni del controlador.
