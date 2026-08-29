# Fase 5H — Pruebas diagnósticas para aislar el origen del fallo angular

## Objetivo

Antes de seguir modificando el estimador ORB, ejecutar una batería corta de pruebas para responder una sola pregunta:

> ¿El dron se desestabiliza por las características temporales de una fuente visual a ~20 Hz con retraso, o por la propia estimación ORB?

No implementar todavía `Delta_target`, un estimador retardado ni nuevos filtros.

Usar el mismo hover, mismo controlador y mismas ganancias en todas las pruebas.

GT se usa únicamente como señal perfecta de laboratorio.

---

## Prueba A — GT normal a 50 Hz

Controlar el dron con GT normal a 50 Hz.

Debe servir como referencia.

Esperado:

```text
hover estable
sin oscilación creciente
sin fallback
```

Si falla, detenerse: el problema es básico de control/pipeline.

---

## Prueba B — GT limitado a 20 Hz, sin retraso

Usar GT perfecto, pero entregar muestras al pipeline/control como una fuente de 20 Hz.

Entre muestras, reproducir el comportamiento normal del estado usado por el controlador.

Objetivo:

> comprobar si el simple hecho de tener una fuente de 20 Hz frente a un controlador de 50 Hz causa la inestabilidad.

Si A funciona y B falla:

```text
problema de frecuencia / hold / pipeline 20 Hz -> 50 Hz
```

---

## Prueba C — GT a 20 Hz + 80 ms de retraso

Mantener GT perfecto, pero:

```text
frecuencia = 20 Hz
retraso artificial ≈ 80 ms
```

Los ~80 ms se basan en el retraso visual medido anteriormente.

La muestra debe seguir representando su timestamp físico original; no falsificarla como si perteneciera al instante de recepción.

Si A y B funcionan, pero C reproduce la oscilación:

```text
latencia/fase temporal = causa principal muy probable
```

---

## Prueba D — GT con timing realista de ORB

Usar GT perfecto, pero imitar lo mejor posible las características temporales reales observadas en ORB:

```text
~20 Hz
retraso variable
jitter
timestamps reales/originales
mismo hold/predicción/publicación del NavigationState
```

Utilizar los datos medidos en logs anteriores para caracterizar ese timing; no inventar jitter arbitrario.

La pose sigue siendo GT perfecta: sólo se degradan sus características temporales.

Si D reproduce el fallo de ORB:

```text
el problema está en la arquitectura temporal/pipeline
```

Si A, B, C y D funcionan bien pero ORB real sigue fallando:

```text
la hipótesis temporal queda debilitada
y hay que volver a estudiar la propia estimación ORB
```

---

## Métricas iguales en las cuatro pruebas

Registrar y comparar:

```text
pose/orientación entregada al controlador
omega entregada al controlador

er
ew

tau_er
tau_ew
tau_total

energía:
tau_er · omega_GT
tau_ew · omega_GT
tau_total · omega_GT

timestamps
edad de muestra
frecuencia efectiva
jitter

omega_GT
orientación GT

crecimiento de oscilación
fallback
tracking/source
```

---

## Árbol de diagnóstico esperado

```text
GT 50 Hz
   |
   +-- falla -> problema básico de control/pipeline
   |
   +-- funciona
        |
        v
GT 20 Hz sin delay
   |
   +-- falla -> problema frecuencia/hold 20->50 Hz
   |
   +-- funciona
        |
        v
GT 20 Hz + 80 ms
   |
   +-- falla -> latencia/fase confirmada como sospechoso principal
   |
   +-- funciona
        |
        v
GT con timing/jitter realista ORB
   |
   +-- falla -> problema temporal/pipeline realista
   |
   +-- funciona
        |
        v
ORB real falla
   ->
problema principalmente en la estimación ORB,
no en sus características temporales
```

---

## Reglas

- No cambiar ganancias.
- No cambiar thresholds ORB.
- No implementar `Delta_target`.
- No implementar todavía el estimador de medidas retardadas.
- No ejecutar etapas posteriores de trayectoria.
- Hacer sólo estas pruebas y analizar causalmente los resultados.

Al terminar, actualizar el historial y devolver una comparación clara A/B/C/D con una conclusión sobre dónde está el fallo.
