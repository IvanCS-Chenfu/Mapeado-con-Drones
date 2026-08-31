# Fase 5H — Corrección del frame de gravedad en la propagación translacional dinámica
## Objetivo: corregir la causa dominante de `DYNAMIC_PROPAGATION` con ORB real sin tocar todavía `THREE_SAMPLE`

## 0. Fuente de verdad

Trabajar sobre el estado ACTUAL del repositorio:

```text
https://github.com/IvanCS-Chenfu/Mapeado-con-Drones
```

Tomar como base directa:

```text
historial_5H_RESUMEN.md
prueba_322.log
diagnóstico 322/323
```

Estado ya demostrado:

```text
v_mid RMSE            ≈ 0.01984 m/s
TWO_SAMPLE RMSE       ≈ 0.01988 m/s
THREE_SAMPLE v_hat    ≈ 0.03457 m/s
v_dynamic(now) RMSE   ≈ 0.43308 m/s
```

Conclusión diagnóstica vigente:

```text
MULTICAUSAL
```

con dos efectos distintos:

```text
1. A_HAT_AMPLIFICATION
2. degradación dominante posterior en DYNAMIC_PROPAGATION
```

El primer efecto existe, pero NO debe corregirse todavía en esta iteración.

---

# 1. Hipótesis física principal a comprobar/corregir

La propagación translacional actual usa conceptualmente:

```text
F_body = [0, 0, thrust]

a_O =
R_O_B * F_body / m
+
g_O
```

El problema sospechado es que actualmente se está usando implícita o explícitamente:

```text
g_O = (0, 0, -9.81)
```

como si:

```text
Z_O == Z_W
```

Esto puede ser cierto en pruebas de laboratorio con frame alineado, pero NO está garantizado con ORB real.

En Fase 5:

```text
O = frame local continuo de control
W = frame global
```

y:

```text
O != W
```

por diseño.

Por tanto, la gravedad física conocida debe partir de:

```text
g_W = (0, 0, -9.81)
```

y expresarse en O mediante la rotación entre W y O:

```text
g_O =
O_R_W * g_W
```

---

# 2. Evidencia observada en prueba 322

El diagnóstico 322 demuestra que:

```text
v_hat(t_k)
```

es relativamente razonable, pero:

```text
v_dynamic(now)
```

se degrada de forma dominante durante:

```text
t_k -> now
```

La magnitud observada:

```text
0.03457 m/s
    ->
0.43308 m/s
```

es demasiado grande para atribuirla únicamente al ruido de `a_hat`.

Además, en los logs aparecen intervalos donde:

```text
thrust ≈ m*g
```

pero el predictor introduce componentes grandes de velocidad horizontal/vertical en apenas decenas de milisegundos.

La hipótesis a validar es:

> El thrust se transforma correctamente desde body a O, pero la gravedad se suma todavía como si estuviera alineada con el eje Z de O.

Esto produciría una aceleración ficticia incluso durante hover físico.

IMPORTANTE:

> Esta causa concreta es una inferencia física a partir de los logs y del diagnóstico 322/323. Codex debe verificarla directamente en el código actual antes de modificar nada.

---

# 3. Primera tarea obligatoria — auditar el código actual

Antes de modificar:

buscar exactamente dónde se calcula la aceleración translacional en:

```text
BodyThrustDynamicPredictor
o clase equivalente
```

Documentar:

```text
1. frame de p;
2. frame de v;
3. frame de R usada;
4. frame del thrust;
5. frame actual de gravity;
6. fórmula exacta actual de acceleration;
7. dónde se define el vector gravedad;
8. si existe ya alguna transformación W<->O disponible.
```

No asumir que la implementación coincide exactamente con el pseudocódigo anterior.

Si la gravedad YA está correctamente transformada:

```text
STOP
```

y explicar por qué la hipótesis queda descartada.

No modificar por inercia.

---

# 4. Contrato correcto de frames

La implementación debe respetar:

```text
p_O
v_O
a_O
```

El thrust nace en body:

```text
F_B = (0,0,T)
```

y se transforma:

```text
F_O =
O_R_B * F_B
```

La gravedad nace físicamente en W:

```text
g_W =
(0,0,-9.81)
```

y debe transformarse:

```text
g_O =
O_R_W * g_W
```

Entonces:

```text
a_O =
F_O / m
+
g_O
```

Todo debe quedar en el MISMO frame antes de sumar.

---

# 5. Cómo obtener `O_R_W`

No introducir GT.

La arquitectura de Fase 5 ya dispone de una relación entre:

```text
W
O
```

cuando el dron queda globalmente anclado.

Conceptualmente:

```text
W_T_O
```

o una transformación equivalente.

Extraer únicamente su rotación:

```text
W_R_O
```

y obtener:

```text
O_R_W =
inverse(W_R_O)
```

o la operación equivalente según la convención SE(3) usada en el código.

No adivinar el sentido de la transformación.

Añadir test específico para evitar invertirla al revés.

---

# 6. Regla CRÍTICA — congelar `g_O` por `map_epoch`

NO recalcular `g_O` continuamente a partir de revisiones globales.

Motivo:

```text
W puede cambiar por:
- optimización global
- revisiones de KF
- loops
- nueva pose_revision
```

pero:

```text
O debe permanecer continuo
```

Si `g_O` se recalcula con cada revisión W:

```text
optimización global
    ->
cambia W_R_O
    ->
cambia artificialmente g_O
    ->
cambia a_O
    ->
impulso falso en control
```

Eso viola la arquitectura de Fase 5.

Por tanto:

> `g_O` debe calcularse UNA VEZ cuando el epoch obtiene un anclaje global fiable y luego quedar congelada para ese `map_epoch`.

---

# 7. Ciclo de vida propuesto de la gravedad

## Epoch nuevo sin anclaje

Estado:

```text
gravity_o_valid = false
```

No inventar orientación de gravedad.

Durante una ruta que necesite propagación translacional productiva ORB:

```text
si gravity_o_valid == false
```

no declarar plenamente válido el predictor dinámico translacional ORB.

Aplicar la política segura ya existente.

No usar GT para rellenarlo.

---

## Primer anclaje fiable del epoch

Cuando exista por primera vez una relación válida:

```text
W_T_O
```

para el `map_epoch` activo:

calcular:

```text
O_R_W_epoch
g_O_epoch = O_R_W_epoch * g_W
```

y guardar:

```text
gravity_o_valid = true
gravity_map_epoch = current_epoch
```

---

## Revisiones posteriores de W

No modificar:

```text
g_O_epoch
```

aunque cambie:

```text
W_T_KF
pose_revision
optimización global
```

---

## Nuevo `map_epoch`

Invalidar:

```text
gravity_o_valid = false
```

y esperar nuevo anclaje fiable.

No reutilizar silenciosamente la gravedad del epoch anterior si no existe continuidad explícita acordada.

---

# 8. No mezclar esta gravedad con el objetivo global

`g_O` es una propiedad física del frame local continuo.

No debe depender de:

```text
goal absoluto
goal relativo
trayectoria activa
```

Una vez congelada por epoch, debe ser independiente de la misión.

---

# 9. Fuente exacta del anclaje

Codex debe auditar cuál es la fuente correcta para obtener la rotación inicial `W_R_O`.

Preferencia:

```text
la misma relación W/O aceptada por Fase 5
```

y NO reconstruir otra relación paralela desde:

```text
latest KF
nearest KF
pose GT
```

Usar únicamente autoridad geométrica ya válida del sistema.

Documentar exactamente:

```text
qué objeto/variable
qué timestamp
qué map_epoch
qué validity
```

se utiliza.

---

# 10. No contaminar O con futuras optimizaciones

Añadir protección explícita:

```text
if gravity already valid for this epoch:
    do not overwrite
```

aunque llegue:

```text
nueva revision global
nuevo W_T_KF
nuevo anchor update
```

a menos que exista:

```text
nuevo map_epoch
```

o un reset explícito del contrato.

---

# 11. Telemetría nueva obligatoria

Añadir log bajo debug:

```text
[F5H-GRAVITY-O-INIT]
map_epoch=
stamp=
source=
W_R_O=
O_R_W=
g_W=
g_O=
norm=
```

Esperado:

```text
|g_O| ≈ 9.81
```

En cada predicción translacional, opcionalmente bajo debug:

```text
[F5H-DYNAMIC-TRANSLATION]
map_epoch=
gravity_valid=
g_O=
thrust=
F_O=
a_O=
dt=
v_before=
v_after=
```

No inundar logs normales si no está activado debug.

---

# 12. Sanity físico obligatorio

En un hover aproximadamente estacionario:

si:

```text
T ≈ m*g
```

y la orientación física es coherente:

la suma:

```text
F_O/m + g_O
```

debe quedar aproximadamente:

```text
0
```

No necesariamente exactamente cero por ruido/control.

Pero no debe producir:

```text
~9.8 m/s²
```

en un eje horizontal de O.

Añadir métrica:

```text
hover_acceleration_residual_norm
```

durante shadow.

---

# 13. GTests obligatorios

Añadir como mínimo:

## A — O alineado con W

```text
O_R_W = I
g_O = (0,0,-g)
```

---

## B — O rotado 90 grados

Crear una rotación conocida.

Verificar:

```text
g_O = O_R_W * g_W
```

con dirección correcta.

---

## C — norma

Para cualquier rotación válida:

```text
|g_O| = |g_W|
```

dentro de tolerancia.

---

## D — hover en frame O rotado

Elegir:

```text
R_O_B
```

coherente con un dron físicamente nivelado en W.

Con:

```text
T = m*g
```

esperar:

```text
a_O ≈ 0
```

aunque O esté rotado respecto de W.

Éste es el test más importante.

---

## E — inversión incorrecta detectada

Crear un caso donde:

```text
W_R_O
```

y:

```text
O_R_W
```

produzcan resultados claramente diferentes.

Verificar que se usa la correcta.

---

## F — freeze por epoch

Inicializar:

```text
epoch 0
g_O_0
```

Después simular:

```text
nueva revisión W
```

Esperado:

```text
g_O_0 NO cambia
```

---

## G — nuevo epoch

Cambiar a:

```text
epoch 1
```

Esperado:

```text
gravity_valid=false
```

hasta nuevo anclaje.

---

## H — no GT

Test/inspección para asegurar:

```text
gravity O productiva
```

no consume:

```text
sensor/GT/*
```

---

# 14. No tocar todavía `THREE_SAMPLE`

Aunque 322/323 demuestra:

```text
v_mid            0.01984
TWO_SAMPLE       0.01988
THREE_SAMPLE     0.03457
```

NO cambiar todavía el estimador lineal productivo.

Motivo:

> Queremos modificar una sola causa cada vez.

Primero corregir:

```text
frame de gravedad
```

y volver a medir.

Después se decidirá si:

```text
TWO_SAMPLE
```

debe sustituir o complementar a:

```text
THREE_SAMPLE
```

---

# 15. Prueba 324 — repetir shadow 322 con gravedad correcta

Después de implementar y pasar tests:

repetir exactamente el escenario 322.

Nombre sugerido:

```text
324
```

Configuración:

```text
GT gobierna
ORB dynamic completo en shadow
airborne + settled
```

No permitir todavía ORB como fuente de control.

---

# 16. Métricas obligatorias de 324

Recalcular exactamente:

```text
RMSE v_mid
RMSE TWO_SAMPLE
RMSE THREE_SAMPLE v_hat_tk
RMSE v_dynamic_now
```

Además:

```text
RMSE por eje X/Y/Z
bias por eje
std
p95
max
```

y:

```text
g_O
|g_O|
hover_acceleration_residual_norm
```

Comparar:

```text
322 vs 324
```

---

# 17. Criterio principal de 324

Queremos observar que:

```text
v_dynamic_now
```

deja de degradarse masivamente respecto de:

```text
v_hat_tk
```

No fijar un umbral artificial exacto antes de medir.

Pero la mejora debe ser clara frente a:

```text
0.43308 m/s
```

de la 322.

Calcular:

```text
gain_dynamic =
RMSE(v_dynamic_now) / RMSE(v_hat_tk)
```

Antes:

```text
~12.5
```

Esperado:

```text
muy inferior
```

si la hipótesis es correcta.

---

# 18. Comprobar específicamente la componente vertical/horizontal

En 322 la física sospechada genera aceleración ficticia transversal por expresar mal la gravedad.

En 324 registrar:

```text
a_thrust_O
g_O
a_total_O
```

durante hover.

Comprobar que:

```text
a_thrust_O ≈ -g_O
```

cuando:

```text
T ≈ m*g
```

y el dron está nivelado físicamente.

---

# 19. Prueba 325 — repetición

Si 324 demuestra mejora fuerte y escenario válido:

repetir como:

```text
325
```

sin cambios.

Objetivo:

```text
reproducibilidad
```

No pasar a control ORB con una sola ejecución.

---

# 20. Si 324/325 validan la corrección

Sólo entonces discutir el segundo problema:

```text
A_HAT_AMPLIFICATION
```

La evidencia actual sugiere que:

```text
TWO_SAMPLE
```

es mejor que:

```text
THREE_SAMPLE
```

con ORB real.

Pero esa será una iteración separada.

No cambiar ambas cosas en el mismo commit experimental si puede evitarse.

---

# 21. Si 324 NO mejora `v_dynamic`

STOP.

No ejecutar 325.

Revisar:

```text
frame real de thrust
frame real de R_O_B
sentido O_R_W
timestamp del anclaje
si g_O se calcula desde una W/O incorrecta
```

y comparar numéricamente:

```text
a_predicha
vs
a_GT
```

durante hover.

No tocar `THREE_SAMPLE` todavía si el error dominante sigue estando en propagación.

---

# 22. Si `g_O` no está disponible antes del handoff

No inventarla.

El escenario de Fase 5 ya exige:

```text
anchor válido
```

antes de activar ORB en hover.

Aprovechar esa frontera:

```text
anchor válido
    ->
gravity_o_valid
    ->
ORB dinámico translacional consumible
```

Si no se puede obtener una relación W/O válida del anclaje actual:

```text
STOP
```

y documentar qué autoridad falta.

No usar GT como solución productiva.

---

# 23. Relación con la arquitectura futura sin GT

La solución final debe seguir siendo:

```text
ORB
+
anclaje global/fiducial
+
dinámica propia
```

La gravedad mundial:

```text
g_W
```

es una constante física conocida.

No es GT.

Lo único necesario es conocer cómo está orientado O respecto de W, que ya forma parte del problema de anclaje global de Fase 5.

---

# 24. Qué NO hacer

No modificar:

```text
v_hat
TWO_SAMPLE/THREE_SAMPLE productivo

omega_hat

J
masa

torque/thrust buffers
ZOH
poda

gains

SMALL/MODERATE

KF policy

W optimizer
```

No recalcular `g_O` en cada optimización global.

No usar `R_O_B` actual para inferir directamente la dirección de gravedad sin una referencia global.

No introducir IMU inexistente.

---

# 25. Builds y validaciones

Ejecutar:

```text
build orbslam3
build dron_individual
build simulacion_dron

GTests/CTest
analizador
validaciones Python/YAML
git diff --check
```

No degradar:

```text
102/102 GTests
```

o el número superior vigente al iniciar.

---

# 26. Documentación

Actualizar:

```text
historial_5H_RESUMEN.md
07_ULTIMA_SESION.md
contrato 5H
```

si se confirma esta semántica.

Añadir al contrato, si la prueba la valida:

> La gravedad utilizada por la dinámica translacional se expresa en el frame local continuo O y queda congelada por map_epoch a partir del primer anclaje global fiable. Las revisiones posteriores de W no modifican la gravedad de control del epoch activo.

---

# 27. Qué debe devolver Codex

Al terminar:

```text
Resultado:
CONSEGUIDO / PARCIAL / NO CONSEGUIDO
```

Incluir:

```text
- commit/estado inicial;
- archivos modificados;

- fórmula translacional anterior exacta;
- frame anterior de gravity;
- confirmación de si la hipótesis era correcta;

- fuente exacta de W_R_O;
- convención usada;
- momento de inicialización de g_O;
- política por map_epoch;
- política ante revisión global;
- política ante nuevo epoch;

- valor g_O observado en 324;
- norma;

- GTests añadidos;
- total GTests;
- builds;
- analyzer;
- git diff --check;

- prueba 324;

- comparación 322 vs 324:
    RMSE v_mid
    RMSE TWO_SAMPLE
    RMSE THREE_SAMPLE
    RMSE v_dynamic_now
    gain_dynamic
    métricas por eje;

- hover_acceleration_residual_norm;

- prueba 325 si corresponde;

- conclusión explícita:
    GRAVITY_FRAME CONFIRMADO / DESCARTADO
    DYNAMIC_PROPAGATION CORREGIDA / NO CORREGIDA

- mantener STOP antes de cambiar THREE_SAMPLE.
```

---

# 28. Resumen ejecutivo

322/323 demuestra:

```text
v_mid            ≈ 0.01984 m/s
TWO_SAMPLE       ≈ 0.01988 m/s
THREE_SAMPLE     ≈ 0.03457 m/s
v_dynamic(now)   ≈ 0.43308 m/s
```

Hay dos problemas:

```text
secundario:
a_hat amplifica ruido

principal:
la propagación dinámica degrada mucho más
```

La hipótesis física dominante es:

```text
thrust -> transformado a O
gravedad -> todavía tratada como -Z_O
```

aunque O no está alineado con W.

Corregir exclusivamente:

```text
g_W = (0,0,-9.81)

g_O =
O_R_W(epoch_anchor) * g_W

freeze g_O durante todo el map_epoch
```

Después:

```text
324:
repetir 322 en shadow

325:
repetición si 324 mejora claramente
```

> No tocar todavía `THREE_SAMPLE`. Primero comprobar si expresar correctamente la gravedad en O elimina la degradación dominante de `DYNAMIC_PROPAGATION`.
