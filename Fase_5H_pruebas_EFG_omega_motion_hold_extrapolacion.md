# Fase 5H — Pruebas para aislar el fallo de `omega_motion` y la adaptación 20 Hz -> 50 Hz

## Objetivo

Antes de modificar otra vez el estimador ORB, ejecutar una batería mínima de pruebas con GT perfecto para separar qué parte del adaptador temporal está provocando la inestabilidad.

Las pruebas 269-272 ya demostraron:

```text
GT 50 Hz:
estable

GT perfecto 20 Hz:
ya aparece giro sostenido y energía positiva

GT 20 Hz + 80 ms:
peor

GT con timing realista ORB:
peor todavía
```

Por tanto, el fallo principal se reproduce sin error geométrico ORB.

La siguiente pregunta es:

> ¿El problema está en la derivación/filtrado de `omega_motion`, en el hold 20->50 Hz, o en la extrapolación de pose?

No implementar todavía:

```text
Delta_target
estimador retardado
nuevos filtros ORB
nuevos thresholds
```

---

# Prueba E — GT pose 20 Hz + omega GT exacta

Usar:

```text
pose GT muestreada a 20 Hz
omega GT exacta
```

No calcular `omega_motion` mediante diferencias entre poses.

Mantener el resto del pipeline lo más igual posible:

```text
NavigationState
publicación 50 Hz
controlador 50 Hz
```

Objetivo:

> comprobar si sustituir la `omega_motion` derivada/filtrada por la velocidad angular física exacta elimina la inestabilidad.

## Interpretación

Si esta prueba funciona:

```text
problema principal localizado en:
derivación / filtrado de omega_motion
```

Si falla:

```text
el problema no está sólo en omega_motion
```

y hay que continuar con la prueba F.

---

# Prueba F — GT pose 20 Hz + omega GT exacta + ZERO-ORDER HOLD de pose

Usar:

```text
pose GT a 20 Hz
omega GT exacta
```

Pero entre medidas:

```text
NO extrapolar orientación
```

Mantener la última pose recibida:

```text
R_control(t) = última R_GT disponible
```

hasta la siguiente muestra.

La `omega` entregada al controlador sigue siendo GT exacta.

Objetivo:

> comprobar si el controlador puede mantenerse estable con pose escalonada a 20 Hz cuando la velocidad angular es correcta.

## Interpretación

Si E falla pero F funciona:

```text
la extrapolación de pose es el principal sospechoso
```

Si F también falla:

```text
el simple hold de pose 20 Hz + control 50 Hz sigue siendo problemático
```

y habrá que estudiar específicamente el uso de una pose lenta en `er`.

---

# Prueba G — GT pose 20 Hz + omega GT exacta + extrapolación de pose a 50 Hz

Usar:

```text
pose GT a 20 Hz
omega GT exacta
```

y ahora sí extrapolar:

```text
R_pred(t_now)
=
Propagate(
    R_GT(t_sample),
    omega_GT,
    t_now - t_sample
)
```

respetando exactamente las convenciones SO(3) y frame actuales.

No filtrar `omega_GT`.

Objetivo:

> aislar si la propagación angular 20->50 Hz es correcta cuando parte de una pose y una omega perfectas.

## Interpretación

Si F funciona y G falla:

```text
bug en la propagación/extrapolación angular
```

Si G funciona:

```text
pose + omega perfectas pueden adaptarse 20->50 Hz correctamente
```

y el principal culpable vuelve a ser la estimación/filtrado de `omega_motion`.

---

# Orden obligatorio

Ejecutar:

```text
E
-> F
-> G
```

No saltar directamente a ORB.

No modificar funcionalmente el sistema entre pruebas salvo lo necesario para seleccionar el modo diagnóstico.

---

# Métricas comunes

Registrar exactamente las mismas señales:

```text
GT orientation
GT omega

pose entregada al controlador
omega entregada al controlador

er
ew

tau_er
tau_ew
tau_total

P_er_GT = tau_er · omega_GT
P_ew_GT = tau_ew · omega_GT
P_total_GT = tau_total · omega_GT

frecuencia efectiva
edad de muestra
prediction horizon

max |er|
max |ew|
max |omega|

energía angular acumulada
crecimiento de oscilación
scenario success
```

---

# Dato especialmente importante

Comparar en cada prueba:

```text
sign(omega_GT)
vs
sign(omega_control)
```

y medir cuánto tiempo permanecen con signo/dirección incompatible.

Las pruebas anteriores mostraron casos donde el movimiento físico ya había cambiado de dirección mientras `omega_motion` seguía apuntando en la dirección anterior.

Queremos saber si eso desaparece completamente al usar:

```text
omega GT exacta
```

---

# Árbol de diagnóstico

```text
E: pose 20 Hz + omega GT exacta
   |
   +-- funciona
   |      ->
   |   culpa principal:
   |   derivación/filtrado de omega_motion
   |
   +-- falla
          |
          v
F: pose 20 Hz hold + omega GT exacta
   |
   +-- funciona
   |      ->
   |   problema en extrapolación/predicción de pose
   |
   +-- falla
          |
          ->
      pose a 20 Hz en el lazo de er
      también es problemática


G: pose 20 Hz + omega GT exacta + extrapolación
   |
   +-- funciona
   |      ->
   |   adaptación 20->50 Hz es viable
   |   si pose/omega son coherentes
   |
   +-- falla
          ->
      bug en propagación temporal/frame/SO3
```

---

# Reglas

No cambiar:

```text
Kr
Kw
Kp
Kv
```

No usar filtros nuevos.

No cambiar thresholds ORB.

No implementar `Delta_target`.

No implementar todavía el estimador retardado.

No ejecutar trayectoria/etapa 3.

El único objetivo es:

> aislar exactamente qué componente del adaptador 20->50 Hz está generando la inestabilidad.

---

# Entrega final de Codex

Al terminar, devolver una tabla:

```text
                 E             F             G
------------------------------------------------
scenario success
max |er|
max |ew|
max |omega_control|
energia tau_er
energia tau_ew
energia total
sign mismatch GT/control
prediction horizon
oscilacion
```

Y una conclusión explícita entre:

```text
A) omega_motion derivada/filtrada es la causa principal

B) el hold de pose 20 Hz es insuficiente

C) la extrapolación de pose está mal

D) hay más de un problema

E) datos todavía insuficientes
```

Actualizar el historial de Fase 5H sin borrar pruebas anteriores.
