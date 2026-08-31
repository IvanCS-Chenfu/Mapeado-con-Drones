# Fase 5H — Sustitución diagnóstica de `THREE_SAMPLE` por propagación física desde `t_mid`
## Objetivo: eliminar `A_HAT_AMPLIFICATION` reutilizando la dinámica translacional ya validada

## 0. Fuente de verdad

Trabajar sobre el estado ACTUAL del repositorio:

```text
https://github.com/IvanCS-Chenfu/Mapeado-con-Drones
```

Tomar como referencia directa:

```text
historial_5H_RESUMEN.md
prueba_322.log
prueba_324.log
prueba_325.log
diagnóstico 322/323
corrección GRAVITY_FRAME post-323
```

Estado ya demostrado:

```text
v_mid RMSE                  ≈ 0.01984 m/s
TWO_SAMPLE RMSE             ≈ 0.01988 m/s
THREE_SAMPLE v_hat(t_k)     ≈ 0.03457 m/s

v_dynamic(now) antes        ≈ 0.43308 m/s
v_dynamic(now) 324          ≈ 0.03583 m/s
v_dynamic(now) 325          ≈ 0.03707 m/s

gain_dynamic antes          ≈ 12.53
gain_dynamic 324            ≈ 0.992
gain_dynamic 325            ≈ 1.003
```

Conclusiones ya cerradas:

```text
GRAVITY_FRAME = CONFIRMADO
DYNAMIC_PROPAGATION = CORREGIDA
```

La gravedad productiva queda:

```text
g_O = O_R_W * g_W
```

obtenida del primer anclaje global `AUTHORITATIVE`, congelada durante el `map_epoch` e invalidada al cambiar de epoch.

---

# 1. Problema restante

Después de corregir la gravedad:

```text
v_dynamic(now)
```

ya no amplifica el error de:

```text
v_hat(t_k)
```

La degradación restante aparece antes:

```text
v_mid
    ≈ 0.01984 m/s
        ↓
THREE_SAMPLE
    ≈ 0.03457 m/s
```

La causa demostrada es:

```text
A_HAT_AMPLIFICATION
```

La implementación actual calcula conceptualmente:

```text
v_mid_1
v_mid_2
    ↓
a_hat =
(v_mid_2 - v_mid_1) / dt_mid
    ↓
v_hat(t_k) =
v_mid_2 + a_hat * (t_k - t_mid_2)
```

Esto deriva dos veces posiciones ORB reales y amplifica el ruido visual.

---

# 2. Regla principal

NO modificar todavía directamente la salida productiva.

Primero implementar y validar en paralelo una alternativa diagnóstica:

```text
MIDPOINT_DYNAMIC
```

comparándola contra:

```text
TWO_SAMPLE
THREE_SAMPLE
```

sobre LAS MISMAS muestras ORB.

Sólo si la alternativa queda claramente validada se autoriza sustituir `THREE_SAMPLE`.

---

# 3. Idea de `MIDPOINT_DYNAMIC`

La última diferencia de posiciones:

```text
v_mid =
[p(k) - p(k-1)]
/
[t(k) - t(k-1)]
```

representa aproximadamente la velocidad en:

```text
t_mid =
(t(k-1) + t(k)) / 2
```

No tratar:

```text
v_mid
```

como si perteneciera exactamente a:

```text
t_k
```

y no estimar una aceleración visual de segunda derivada.

En su lugar:

```text
v_mid(t_mid)
        +
dinámica física real
desde t_mid hasta t_k
        ↓
v_mid_dynamic(t_k)
```

---

# 4. Dinámica a reutilizar

Usar EXACTAMENTE la dinámica translacional ya validada:

```text
a_O(t) =
R_O_B(t) * F_B(t) / m
+
g_O
```

donde:

```text
F_B = [0,0,thrust]
m = masa compartida validada
g_O = gravedad congelada del map_epoch
R_O_B(t) = orientación dinámica coherente
```

No crear un segundo modelo translacional.

Reutilizar:

```text
BodyThrustDynamicPredictor
```

o extraer una función común equivalente.

---

# 5. Intervalo `t_mid -> t_k`

El nuevo estimador causal debe hacer:

```text
p(k-1), t(k-1)
p(k),   t(k)

        ↓

v_mid
t_mid

        ↓

historial de thrust
R_dynamic(t)
g_O
masa

        ↓

propagación física

        ↓

v_hat_midpoint_dynamic(t_k)
```

No usar:

```text
a_hat visual
```

en esta ruta.

---

# 6. Importante: cobertura temporal

Para propagar:

```text
[t_mid, t_k]
```

debe existir:

```text
thrust coverage = FULL
```

y la orientación dinámica necesaria debe estar disponible/coherente.

No inventar comandos.

Si falta cobertura:

```text
MIDPOINT_DYNAMIC_INVALID
```

y mantener la política segura.

No usar GT para rellenar huecos.

---

# 7. Posición en `t_k`

En esta iteración NO modificar la posición base.

Mantener:

```text
p(t_k) = posición visual aceptada
```

El nuevo método sólo sustituye cómo se obtiene:

```text
v(t_k)
```

---

# 8. Prueba 326 — ablation shadow en paralelo

Crear una prueba equivalente a 324/325:

```text
GT gobierna
ORB dynamic real en shadow
airborne + settled
tracking estable
```

Calcular simultáneamente sobre la misma muestra:

```text
A) TWO_SAMPLE

v_two_tk =
v_mid
```

```text
B) THREE_SAMPLE actual

v_three_tk =
v_mid + a_hat_visual * horizon
```

```text
C) MIDPOINT_DYNAMIC

v_midpoint_dynamic_tk =
Propagate(
    v_mid @ t_mid,
    thrust,
    R_dynamic,
    g_O,
    m,
    t_mid -> t_k
)
```

NO usar ninguna de estas variantes nuevas para gobernar el dron.

---

# 9. GT sincronizado

Comparar las tres variantes contra:

```text
v_GT(t_k)
```

GT exclusivamente como truth diagnóstica.

No comparar una velocidad en:

```text
t_mid
```

directamente con GT en:

```text
t_k
```

sin compensar timestamp.

---

# 10. Telemetría obligatoria de 326

Por cada muestra válida:

```text
t_k
t_k1
t_mid

dt

p_k1
p_k

v_mid

v_two_tk

v_mid_previous
a_hat_visual
v_three_tk

midpoint_dynamic_horizon

thrust_samples_mid_to_tk
thrust_coverage_mid_to_tk

g_O

R_dynamic_mid
R_dynamic_tk

v_midpoint_dynamic_tk

v_GT_tk
```

Errores:

```text
error_two
error_three
error_midpoint_dynamic
```

---

# 11. Métricas obligatorias

Calcular:

```text
RMSE_TWO
MAE_TWO
p95_TWO

RMSE_THREE
MAE_THREE
p95_THREE

RMSE_MIDPOINT_DYNAMIC
MAE_MIDPOINT_DYNAMIC
p95_MIDPOINT_DYNAMIC
```

Además por eje:

```text
bias X/Y/Z
std X/Y/Z
max X/Y/Z
```

Separar por:

```text
GOOD_DT
DEGRADED_DT
```

y por bins de dt si aporta información.

---

# 12. No limitar el diagnóstico al hover totalmente quieto

La prueba 326 debe incluir principalmente hover estable para comparabilidad con 322-325.

Si es fácil y no altera el objetivo principal, incluir además un tramo corto con:

```text
movimiento/aceleración suave
```

gobernado por GT mientras ORB sigue en shadow.

Motivo:

> TWO_SAMPLE puede ser muy bueno en hover simplemente porque la velocidad real es casi constante, pero `MIDPOINT_DYNAMIC` debe demostrar ventaja semántica cuando existe aceleración real.

No introducir todavía una trayectoria compleja.

---

# 13. Prueba opcional 327 — shadow con movimiento suave

Ejecutar sólo si 326 muestra que:

```text
TWO_SAMPLE
```

y:

```text
MIDPOINT_DYNAMIC
```

son casi equivalentes durante hover.

Usar:

```text
GT gobernando
ORB shadow
```

y un movimiento sencillo:

```text
X corto
```

o:

```text
aceleración/deceleración suave
```

sin yaw complejo.

Comparar otra vez:

```text
TWO
THREE
MIDPOINT_DYNAMIC
```

contra:

```text
v_GT(t_k)
```

---

# 14. Criterio de elección

No elegir automáticamente la menor RMSE de hover si la diferencia es mínima.

Preferencia arquitectónica:

```text
MIDPOINT_DYNAMIC
```

si:

```text
1. no empeora significativamente frente a TWO_SAMPLE en hover;
2. mejora o iguala TWO_SAMPLE durante aceleración;
3. sigue siendo claramente mejor que THREE_SAMPLE;
4. mantiene semántica temporal correcta;
5. reutiliza la dinámica ya validada.
```

---

# 15. Hipótesis esperada

La expectativa es:

```text
TWO_SAMPLE
≈ 0.020 m/s

THREE_SAMPLE
≈ 0.035 m/s

MIDPOINT_DYNAMIC
≈ TWO_SAMPLE
o mejor
```

con mayor ventaja de `MIDPOINT_DYNAMIC` cuando:

```text
dv/dt != 0
```

porque TWO_SAMPLE sigue estando centrada temporalmente en `t_mid`.

---

# 16. Si MIDPOINT_DYNAMIC gana

Sólo si 326 y, si corresponde, 327 lo validan:

modificar el estimador productivo para que el modo principal pase de:

```text
THREE_SAMPLE_PREDICTED
```

a una semántica equivalente a:

```text
MIDPOINT_DYNAMIC_PREDICTED
```

La lógica será:

```text
2 posiciones válidas
        ↓
v_mid @ t_mid
        ↓
dinámica física
        ↓
v_hat(t_k)
```

---

# 17. Historial necesario

Con `MIDPOINT_DYNAMIC`, la estimación lineal ya NO necesita tres posiciones para obtener aceleración visual.

El mínimo matemático pasa a ser:

```text
2 posiciones
```

pero NO eliminar estructuras antiguas sin revisar su uso en:

```text
telemetría
tests
fallback
debug
```

Preferencia:

> retirar `a_hat_visual` del camino productivo, pero conservarlo temporalmente como diagnóstico si resulta útil.

---

# 18. Política de dt

Mantener la clasificación actual:

```text
GOOD_DT
DEGRADED_DT
INVALID_DT
```

No asumir que MIDPOINT_DYNAMIC hace aceptable cualquier gap.

La diferencia visual:

```text
p(k)-p(k-1)
```

sigue perdiendo calidad con dt excesivo.

Para:

```text
INVALID_DT
```

no producir velocidad válida.

Para:

```text
DEGRADED_DT
```

mantener política conservadora actual hasta medir.

No ampliar umbrales en esta iteración.

---

# 19. Correcciones visuales y KF

Mantener el contrato ya vigente:

```text
corrección visual de pose
!=
velocidad física
```

No usar saltos artificiales de:

```text
anchor
reference realignment
W revision
```

como desplazamiento físico.

El cálculo de:

```text
v_mid
```

debe usar posiciones O aceptadas y continuas.

Un simple cambio legítimo de `reference_kf` no debe reiniciar automáticamente el método si la continuidad O permanece válida.

---

# 20. GTests para MIDPOINT_DYNAMIC

Añadir como mínimo:

## A — velocidad constante

Sin aceleración real:

```text
MIDPOINT_DYNAMIC ≈ TWO_SAMPLE
```

---

## B — aceleración constante conocida

Construir movimiento donde:

```text
v(t_mid) != v(t_k)
```

Esperado:

```text
MIDPOINT_DYNAMIC
```

más cerca de:

```text
v(t_k)
```

que TWO_SAMPLE.

---

## C — hover con O rotado

Usar:

```text
g_O
```

correcta y:

```text
T = m*g
```

Esperado:

```text
v(t_k) ≈ v_mid
```

sin deriva ficticia.

---

## D — thrust variable

Cambiar thrust dentro de:

```text
[t_mid,t_k]
```

Esperado:

```text
ZOH temporal correcto
```

---

## E — orientación variable

Usar:

```text
R_dynamic(t)
```

durante el intervalo.

No congelar R.

---

## F — cobertura incompleta

Esperado:

```text
MIDPOINT_DYNAMIC_INVALID
```

No inventar.

---

## G — dt irregular

Usar timestamps reales.

---

## H — no `a_hat`

Verificar que el camino productivo MIDPOINT_DYNAMIC no depende de:

```text
a_hat_visual
```

---

# 21. Prueba 328 — validación productiva shadow

Si se decide sustituir productivamente THREE_SAMPLE:

ejecutar una nueva shadow equivalente a 324/325.

Nombre sugerido:

```text
328
```

Usar:

```text
v_hat(t_k) productiva = MIDPOINT_DYNAMIC
```

y recalcular:

```text
RMSE v_hat_tk
RMSE v_dynamic_now
gain_dynamic
```

Esperado:

```text
v_dynamic_now
≈
v_hat_tk
```

como ya ocurre tras corregir gravedad.

---

# 22. Prueba 329 — repetición

Si 328 funciona:

```text
329
```

sin cambios.

Criterio:

```text
328 = CONSEGUIDA
329 = CONSEGUIDA
```

para declarar:

```text
LINEAR_VELOCITY_ESTIMATOR PRODUCTIVO VALIDADO EN SHADOW
```

---

# 23. Sólo después: volver a hover ORB real

Después de:

```text
328
329
```

conseguidas:

repetir el hover ORB completo sin GT en control.

Nombre sugerido:

```text
330
```

Mantener la frontera limpia ya validada:

```text
aproximación GT diagnóstica
ORB shadow
anchor
airborne
settled
activar ORB
confirmar source=ORB
nuevo goal hover
```

Después:

```text
p/v/R/omega = ORB dynamic
```

sin sustituciones GT.

---

# 24. Criterio de 330

No aceptar sólo:

```text
scenario success=true
```

Medir:

```text
posición inicial/final
máxima desviación
RMSE posición
velocidad media/max

ep
ev
er
ew

tracking
fallback
missing
```

Clasificar:

```text
HOVER ORB ESTABLE
```

o:

```text
HOVER ORB DIVERGENTE
```

---

# 25. Si 330 funciona

Repetir exactamente:

```text
331
```

Si:

```text
330 = CONSEGUIDA
331 = CONSEGUIDA
```

entonces sí considerar:

```text
HOVER ORB REAL REPRODUCIBLE
```

y acordar después:

```text
X corto
Y corto
Z corto
yaw lento
curva
trayectoria representativa
```

---

# 26. Si MIDPOINT_DYNAMIC no mejora TWO_SAMPLE

No forzarlo.

Si 326/327 demuestra:

```text
TWO_SAMPLE
```

igual o mejor incluso durante aceleración:

documentar el resultado.

Entonces la alternativa más simple puede ser:

```text
TWO_SAMPLE
```

productiva.

Pero esa decisión debe salir de los datos, no de preferencia estética.

---

# 27. Qué NO hacer

No tocar:

```text
gravedad en O
J
masa

predictor angular
omega_hat

buffers
ZOH
poda

gains

SMALL/MODERATE
KF policy
W
```

No añadir:

```text
low-pass arbitrario
EKF
clamp nuevo
```

No modificar simultáneamente:

```text
THREE_SAMPLE
+
otra parte del estimador
```

---

# 28. Builds y validaciones

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
108/108 GTests
```

o el número superior vigente al empezar.

---

# 29. Documentación

Actualizar:

```text
historial_5H_RESUMEN.md
07_ULTIMA_SESION.md
contrato 5H
```

si se cambia la semántica productiva.

Si MIDPOINT_DYNAMIC queda validado, documentar:

> La velocidad visual se interpreta en el midpoint temporal del último intervalo de posiciones. La estimación en `t_k` se obtiene mediante propagación física usando thrust, orientación dinámica, masa y gravedad expresada en O, evitando derivar visualmente una aceleración de segunda diferencia.

---

# 30. Qué debe devolver Codex

Al terminar:

```text
Resultado:
CONSEGUIDO / PARCIAL / NO CONSEGUIDO
```

Incluir:

```text
- estado/commit inicial;
- archivos modificados;

- fórmula exacta actual THREE_SAMPLE;
- implementación diagnóstica MIDPOINT_DYNAMIC;

- resultado 326;

- RMSE/MAE/p95:
    TWO_SAMPLE
    THREE_SAMPLE
    MIDPOINT_DYNAMIC;

- métricas por eje;
- métricas por dt;

- resultado 327 si se ejecuta;
- comparación en movimiento;

- decisión:
    MIDPOINT_DYNAMIC
    TWO_SAMPLE
    o mantener THREE_SAMPLE;

- si se modifica productivo:
    GTests nuevos
    total GTests
    resultado 328
    resultado 329;

- RMSE v_hat_tk
- RMSE v_dynamic_now
- gain_dynamic;

- si 328/329 validan:
    resultado 330 ORB real;
    resultado 331 si corresponde;

- conclusión explícita:
    A_HAT_AMPLIFICATION CORREGIDA / NO CORREGIDA
    LINEAR_VELOCITY_ESTIMATOR VALIDADO / NO VALIDADO
    HOVER ORB REAL VALIDADO / NO VALIDADO

- siguiente paso recomendado.
```

---

# 31. Resumen ejecutivo

Tras corregir gravedad:

```text
v_mid                  ≈ 0.01984 m/s
TWO_SAMPLE             ≈ 0.01988 m/s
THREE_SAMPLE           ≈ 0.03457 m/s
v_dynamic(now)         ≈ 0.036 m/s
```

La dinámica física ya es correcta.

El único error claro restante es:

```text
a_hat visual
```

de THREE_SAMPLE.

No sustituirlo ciegamente por TWO_SAMPLE.

Probar primero:

```text
v_mid @ t_mid
        ↓
thrust + R_dynamic + g_O + masa
        ↓
MIDPOINT_DYNAMIC
        ↓
v_hat(t_k)
```

Comparar sobre las mismas muestras:

```text
TWO_SAMPLE
THREE_SAMPLE
MIDPOINT_DYNAMIC
```

Primero en hover (`326`) y, si hace falta, con movimiento suave (`327`).

Si MIDPOINT_DYNAMIC queda validado:

```text
328/329:
shadow productivo

330/331:
hover ORB real
```

> La idea es dejar de derivar visualmente una aceleración ruidosa y utilizar para la media ventana restante el modelo físico que ya ha quedado validado de forma reproducible.
