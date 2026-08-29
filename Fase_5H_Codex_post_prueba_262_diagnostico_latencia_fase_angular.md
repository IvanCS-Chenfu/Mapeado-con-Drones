# Instrucciones para Codex — Fase 5H tras prueba 262
## Diagnóstico causal de latencia/fase en el canal angular ORB antes de cualquier nueva modificación funcional

## 0. Estado actual

Estado de Fase 5H:

```text
PARCIAL
```

Última iteración:

```text
Prueba 260: CONSEGUIDA como calibración con GT gobernando
Prueba 261: NO CONSEGUIDA
Prueba 262: NO CONSEGUIDA
Builds: correctos
GTests: 37/37
Etapa 3: NO ejecutada
```

La prueba 262 valida dos correcciones concretas:

```text
- omega_bias permanece en cero con ruido SMALL normal;
- el decay de omega_motion funciona cuando raw deja de ser fiable.
```

A pesar de eso:

```text
- ORB gobierna sólo ~5.92 s;
- omega_motion vuelve a oscilar;
- omega_motion alcanza aproximadamente 0.617 rad/s;
- aparecen tres residuos/rechazos excesivos;
- se activa fallback;
- tracking 2 -> 3 ocurre ~0.54 s después del fallback.
```

Conclusión:

> Los cambios de deadband/histéresis de bias y decay de movimiento rechazado funcionan, pero no eliminan la causa inicial de la oscilación.

No ejecutar etapa 3 todavía.

---

# 1. Qué queda descartado o muy debilitado

A partir de las pruebas 249-262, NO volver a centrar el diagnóstico inmediato en:

```text
- handoff GT -> ORB;
- optimización global W;
- cambio de KF como causa principal del fallo actual;
- omega_bias actuando con SMALL;
- retención de omega_motion durante raw rechazado;
- pérdida de tracking como causa inicial;
- tuning de ganancias del controlador;
- nuevas reducciones arbitrarias de thresholds raw.
```

Esto no significa que esos subsistemas sean perfectos para siempre.

Significa:

> La prueba 262 falla aun después de corregir los dos defectos demostrados en 261, así que la siguiente iteración debe investigar otro mecanismo.

---

# 2. Nueva hipótesis principal

La hipótesis que ahora hay que demostrar o descartar es:

> `omega_motion` derivada de visión llega al controlador con suficiente latencia/desfase para que el término angular de damping deje de amortiguar correctamente y, durante ciertos intervalos, excite el movimiento real del dron.

Cadena conceptual:

```text
movimiento físico del dron
        |
        v
cámara / imágenes ORB ~20 Hz
        |
        v
DeltaR_raw
        |
        v
omega_raw
        |
   filtrado / hold
        |
        v
omega_motion
        |
        v
NavigationState
        |
        v
controlador 50 Hz
        |
        v
ew
        |
        v
tau_damping = -Kw * ew
        |
        v
movimiento físico
```

Si existe suficiente desfase temporal:

```text
el controlador corrige un movimiento que ya ocurrió
```

y el término que debería amortiguar puede convertirse parcialmente en:

```text
anti-damping
```

alimentando la oscilación.

---

# 3. MUY IMPORTANTE — no asumir que la hipótesis ya está demostrada

No modificar todavía:

```text
omega_motion filter
control gains
controller rate
ORB rate
predictor
Kw
```

para “arreglar latencia”.

Primero hay que medir.

La siguiente iteración debe ser principalmente:

```text
INSTRUMENTACIÓN + ANÁLISIS
```

y sólo después, si la evidencia es clara, se decidirá la modificación funcional.

---

# 4. Evidencia que motiva este diagnóstico

En la prueba 262:

```text
omega_bias = 0
```

durante el episodio crítico.

Sin embargo `omega_motion` crece progresivamente y termina alrededor de:

```text
~0.617 rad/s
```

El error angular de velocidad del controlador crece en paralelo.

En hover:

```text
omega_des ~ 0
```

por tanto:

```text
ew ~ omega_body estimada
```

y el controlador genera torque para oponerse a ella.

Pero si esa `omega` llega retrasada respecto al movimiento físico real, el signo/fase del torque puede no ser amortiguador en ese instante.

La prueba 260 es especialmente útil como comparación:

```text
GT gobierna
ORB observa
```

y ORB soporta movimientos mucho más exigentes sin cerrar el lazo de control con su propia velocidad.

La diferencia 260 vs 262 refuerza la necesidad de estudiar el lazo cerrado.

---

# 5. Objetivo exacto de la siguiente iteración

Responder cuantitativamente:

```text
1. ¿Cuánto retraso existe entre omega física real y omega_raw?

2. ¿Cuánto retraso adicional introduce omega_motion?

3. ¿Qué omega recibe exactamente el controlador y con qué edad?

4. ¿El torque de damping se opone realmente al movimiento físico?

5. ¿Existen intervalos donde el torque angular añade energía en vez de quitarla?

6. ¿La oscilación aparece primero en GT, raw, motion, ew o torque?

7. ¿El hold/predictor a 50 Hz entre medidas visuales de ~20 Hz agrava el desfase?
```

No aceptar respuestas cualitativas del tipo:

```text
"parece retrasado"
```

Necesitamos números.

---

# 6. Señales que deben registrarse con timestamps comparables

## 6.1. Ground Truth — sólo diagnóstico

Añadir/registrar:

```text
gt_stamp
R_GT
omega_GT_world
omega_GT_body
```

La representación principal para comparación con el controlador debe ser:

```text
omega_GT_body
```

si el controlador trabaja con velocidad angular en body.

NO usar GT como entrada del estimador.

---

## 6.2. ORB raw

Por cada medida ORB:

```text
image/input timestamp
measurement_receive_timestamp
raw_dt

R_raw
raw_rotation_step
omega_raw
alpha_raw

tracking_state
reference_kf
map_epoch
```

---

## 6.3. Estimador angular

Registrar:

```text
omega_motion_target
omega_motion_applied

omega_bias
omega_total

raw_motion_class

measurement_timestamp_used
predictor_timestamp
publish_timestamp

state_age_sec
time_since_last_visual_measurement
```

Muy importante:

```text
distinguir edad ROS de la muestra
de la edad física de la información visual.
```

No asumir que:

```text
publish_stamp - receive_stamp pequeño
```

implica ausencia de desfase respecto al movimiento real.

---

## 6.4. NavigationState recibido por el controlador

Registrar exactamente al recibir el mensaje:

```text
controller_receive_stamp

navigation_state_measurement_stamp
navigation_state_publish_stamp
navigation_state_age

omega_world/local recibida
omega_body utilizada finalmente
R_act
```

Queremos conocer la señal REAL utilizada en:

```cpp
ew = ...
```

---

## 6.5. Controlador

Registrar a 50 Hz:

```text
control_stamp

R_act
R_des

omega_body_used
Omega_des

er vector
er_norm

ew vector
ew_norm

tau_er = -Kr * er
tau_ew = -Kw * ew

tau_total
tau_total_norm

force
```

No basta con `torque_norm`.

Necesitamos los vectores para estudiar signo/fase.

---

# 7. Señal nueva fundamental: potencia angular del damping

Calcular offline o en telemetría:

```text
P_damping_GT =
tau_ew · omega_GT_body
```

Interpretación:

```text
P_damping_GT < 0
    -> el término Kw está quitando energía al movimiento físico

P_damping_GT > 0
    -> el término Kw está añadiendo energía al movimiento físico
```

Ésta es una de las medidas principales de esta iteración.

Registrar también:

```text
P_total_GT =
tau_total · omega_GT_body
```

para distinguir:

```text
efecto de Kw
vs
efecto de Kr + resto del torque
```

---

# 8. Qué patrón demostraría anti-damping por fase

Hipótesis confirmada si aparece repetidamente una secuencia como:

```text
omega_GT cambia de signo o dirección
        |
        v
omega_motion mantiene todavía el signo anterior
        |
        v
ew usa la señal retrasada
        |
        v
tau_ew apunta en una dirección que coincide con omega_GT actual
        |
        v
tau_ew · omega_GT > 0
        |
        v
aumenta amplitud física de la oscilación
```

No hace falta que ocurra en cada ciclo.

Pero si ocurre de forma repetitiva y correlacionada con el crecimiento de la oscilación, sería evidencia fuerte.

---

# 9. Qué patrón descartaría la hipótesis

La hipótesis de fase/latencia pierde fuerza si:

```text
omega_motion sigue omega_GT con poco retraso
```

y además:

```text
tau_ew · omega_GT < 0
```

durante prácticamente toda la oscilación.

En ese caso:

```text
el término Kw sí amortigua
```

y habrá que buscar otra causa.

Posibles siguientes líneas, sólo si esto ocurre:

```text
- R_act angularmente incorrecta;
- error de transformación world/body de omega;
- dinámica de actitud no modelada;
- ruido raw suficientemente grande aunque temporalmente alineado;
- interacción Kr/er frente a Kw/ew;
- problema de translación que induce actitud deseada variable.
```

No investigarlas todavía salvo que los datos obliguen.

---

# 10. Medición de latencia temporal

Codex debe preparar análisis cuantitativo por eje:

```text
roll
pitch
yaw
```

o directamente componentes body X/Y/Z.

Comparar:

```text
omega_GT_body
omega_raw
omega_motion
omega_control_used
```

Usar:

```text
cross-correlation
```

o método equivalente.

Para cada señal reportar:

```text
lag de máxima correlación
coeficiente de correlación
signo
```

Ejemplo de salida deseada:

```text
axis X:
GT -> raw      lag = 55 ms, corr = 0.91
GT -> motion   lag = 95 ms, corr = 0.88
GT -> control  lag = 110 ms, corr = 0.86
```

Los valores anteriores son sólo ejemplo.

No inventar resultados.

---

# 11. Comparar también fase a frecuencia dominante

Si la oscilación muestra una frecuencia dominante suficientemente clara:

1. estimar frecuencia:

```text
f_osc
```

2. convertir lag a fase aproximada:

```text
phi = 2*pi*f_osc*lag
```

o grados equivalentes.

Esto permitirá saber si el retardo es pequeño o peligroso respecto al periodo de oscilación.

Ejemplo conceptual:

```text
f = 2 Hz
lag = 100 ms

fase ~72 grados
```

Un retraso aparentemente “pequeño” en ms puede ser grande en fase.

---

# 12. Hold entre medidas visuales

ORB mide aproximadamente a:

```text
20 Hz
```

y control trabaja a:

```text
50 Hz
```

Por tanto, estudiar específicamente qué ocurre entre dos medidas visuales.

Queremos saber si:

```text
omega_motion
```

se:

```text
- mantiene constante;
- extrapola;
- decae;
- interpola.
```

Registrar un ejemplo de varios ciclos:

```text
medida ORB n
control tick 1
control tick 2
medida ORB n+1
control tick 3
...
```

y mostrar:

```text
omega usada en cada tick
```

---

# 13. Prueba diagnóstica nueva

Repetir el MISMO escenario:

```text
f5h_etapa_2_hover_orb.yaml
```

No cambiar misión.

No ejecutar etapa 3.

Nombre sugerido:

```text
prueba 263
```

Objetivo:

```text
diagnóstico causal temporal
```

No intentar que esta prueba “pase” mediante nuevos filtros.

---

# 14. No modificar funcionalmente antes de prueba 263

Antes de la 263 sólo se permiten cambios necesarios para:

```text
logs
timestamps
métricas
postprocesado
```

No cambiar:

```text
deadband
bias
raw thresholds
decay
omega_motion gain
control gains
predictor
reference gate
mux
```

La prueba 263 debe representar funcionalmente el mismo sistema que 262.

---

# 15. Análisis offline obligatorio

Codex debe crear una herramienta/script reproducible, preferiblemente dentro de:

```text
codex/archivos_auxiliares/
```

o ubicación coherente con el repositorio.

Debe leer el log/CSV/rosbag disponible y generar al menos:

```text
timeline.csv
```

con columnas sincronizadas:

```text
time

gt_omega_x
gt_omega_y
gt_omega_z

raw_omega_x
raw_omega_y
raw_omega_z

motion_omega_x
motion_omega_y
motion_omega_z

control_omega_x
control_omega_y
control_omega_z

er_x
er_y
er_z

ew_x
ew_y
ew_z

tau_er_x
tau_er_y
tau_er_z

tau_ew_x
tau_ew_y
tau_ew_z

tau_total_x
tau_total_y
tau_total_z

p_damping_gt
p_total_gt

tracking_state
pose_source
reference_kf
state_age
visual_age
```

La sincronización puede utilizar interpolación sólo para ANÁLISIS, nunca para alterar control.

Documentar cómo se ha sincronizado.

---

# 16. Gráficas deseadas

Generar para el episodio ORB de hover:

## Gráfica A

```text
omega_GT vs omega_raw vs omega_motion vs omega_control
```

una por eje si hace falta.

## Gráfica B

```text
ew
tau_ew
omega_GT
```

## Gráfica C

```text
P_damping_GT
```

con línea de cero.

## Gráfica D

```text
tracking
pose_source
```

sobre timeline.

## Gráfica E

```text
state_age
visual_age
```

## Gráfica F

```text
cross-correlation GT/raw
GT/motion
GT/control
```

o resumen numérico equivalente.

---

# 17. Cronología causal obligatoria

Codex debe reconstruir el episodio de divergencia de la prueba 263 indicando tiempos relativos desde entrada ORB.

Formato esperado:

```text
t = 0.000
handoff GT -> ORB

t = ...
primer crecimiento visible de omega_GT

t = ...
primer crecimiento de omega_raw

t = ...
primer crecimiento de omega_motion

t = ...
primer crecimiento de ew

t = ...
primer aumento significativo de tau_ew

t = ...
primer intervalo P_damping_GT > 0

t = ...
amplitud física empieza a crecer

t = ...
primer REJECTED

t = ...
fallback

t = ...
tracking cambia 2 -> 3
```

---

# 18. Comparación 260 / 262 / 263

Preparar una tabla:

```text
                         260          262          263
---------------------------------------------------------
control source
ORB govern time
max |omega_GT|
max |omega_raw|
max |omega_motion|
max |omega_control|
max |ew|
max |tau_ew|
% ciclos P_damping_GT > 0
max intervalo anti-damping
GT->raw lag
GT->motion lag
GT->control lag
tracking loss
fallback
```

La 260 servirá de referencia abierta:

```text
GT gobierna
ORB observa
```

La 262/263:

```text
ORB cierra lazo
```

---

# 19. Criterio de diagnóstico CONSEGUIDO

Esta iteración diagnóstica se considera `CONSEGUIDA` si permite responder con evidencia a una de estas dos conclusiones:

## Conclusión A — fase/latencia confirmada

Ejemplo:

```text
omega_motion/control tiene un lag significativo
+
tau_ew · omega_GT > 0 repetidamente durante crecimiento
+
la oscilación aumenta después
```

Entonces:

```text
la siguiente iteración funcional debe corregir fase/latencia
```

## Conclusión B — fase/latencia descartada

Ejemplo:

```text
lag pequeño
+
tau_ew · omega_GT < 0 durante el crecimiento
```

Entonces:

```text
buscar otra causa
```

No cerrar diagnóstico con:

```text
"parece que"
"probablemente"
```

si los datos permiten medirlo.

---

# 20. Si se confirma la hipótesis — NO implementar aún sin documentar propuesta

Si la 263 confirma el problema, Codex debe PROPONER antes de implementar.

Posibles familias de solución a evaluar:

```text
A. predictor temporal de omega_motion al instante actual;

B. extrapolación usando timestamp real de la medida;

C. compensación de latency conocida;

D. interpolación/predictor a 50 Hz;

E. modificación del uso de omega visual dentro del término Kw;

F. combinación de pose angular actual + derivada visual correctamente timestamped.
```

No escoger automáticamente una.

Explicar:

```text
ventajas
riesgos
impacto en yaw real
impacto en hover
impacto en cambios KF
impacto sin IMU
```

---

# 21. Restricciones arquitectónicas

Seguir respetando:

```text
O continuo
W corregible
```

No usar GT para compensar fase en operación normal.

No introducir dependencia del servidor en el lazo de 50 Hz.

No enviar correcciones globales al estado local.

No cambiar ORB-SLAM3 core salvo bug demostrado.

---

# 22. Qué debe entregar Codex

Después de la prueba 263 devolver:

```text
Resultado diagnóstico:
CONSEGUIDO / PARCIAL / NO CONSEGUIDO
```

y:

```text
- archivos modificados sólo para instrumentación;
- nuevos campos de logs;
- script de análisis;
- resultado de builds;
- resultado de GTests;
- prueba 263;
- duración ORB;
- cronología causal;
- lags GT->raw, GT->motion, GT->control;
- frecuencia dominante de oscilación;
- fase equivalente;
- porcentaje/intervalos con P_damping_GT > 0;
- relación entre anti-damping y crecimiento de amplitud;
- conclusión explícita:
    LATENCIA/FASE CONFIRMADA
    o
    LATENCIA/FASE DESCARTADA
    o
    DATOS INSUFICIENTES;
- propuesta de siguiente paso;
- NO ejecutar etapa 3.
```

Actualizar:

```text
historial_5H_RESUMEN.md
07_ULTIMA_SESION.md
```

sin borrar historial previo.

---

# 23. Resumen ejecutivo

La prueba 262 demuestra:

```text
omega_bias no es ya la causa
decay de raw rechazado funciona
```

pero:

```text
omega_motion sigue creciendo
```

y el hover vuelve a divergir.

La siguiente pregunta no es:

```text
"¿qué threshold cambio?"
```

La siguiente pregunta es:

> **¿la velocidad angular visual llega al controlador con suficiente retraso/desfase como para que el término de damping angular añada energía al movimiento en vez de eliminarla?**

La próxima iteración debe medirlo y demostrarlo antes de tocar otra vez el comportamiento funcional.
