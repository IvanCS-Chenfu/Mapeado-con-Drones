# Historial 5F - resumen

## Estado

```text
Conclusion: PARCIAL
Prueba vigente: 234
Puerta humana: ACEPTADA para relación pose/KFs y corrección tras optimizar
```

El recolector genera CSV/JSON/PNG por timestamp y mide posicion, orientacion,
frecuencia, jitter y latencia relativa. Simulacion 230 `success=true`, sin
errores y con revisiones naturales. Frecuencia ~19.15 Hz; p95 de jitter 1 ms
(dron 1) y 3 ms (dron 2). W posicion: MAE 2.953 m para dron 1 y 0.191 m para
dron 2. W angular: MAE 1.907 y 2.104 rad.

Interpretacion revisada: esos agregados recorren todas las muestras
autoritativas y varias revisiones; no aislan el estado posterior a cada
optimizacion ni la revision final. El error GT->W esperado por deriva y
convergencia no implica por si solo un fallo. W mejora la posicion respecto a
O en ambos drones, pero falta medir por revision y verificar la convencion
angular body/camera/GT. 5F permanece PARCIAL y la puerta humana sigue pendiente;
esto no autoriza 5G.

La prueba 234 ejecuta correctamente los 17 pasos y 22 goals del YAML típico
exacto usando control GT legacy independiente. El usuario confirma en RViz2
que ambas `W_T_B` siguen el camino de KFs y se corrigen al optimizar. Los
problemas restantes del optimizador se dejan explícitamente para Fase 3.

Queda un defecto visual 5F: los ejes parpadean sin pérdida real. El wrapper
propaga una W continua como `PROVISIONAL` al cambiar reference KF, pero
`global_valid` solo es true para `AUTHORITATIVE`; el visualizador borra los
markers durante esa espera. Corrección propuesta pendiente de autorización:
mostrar W `PROVISIONAL` y `AUTHORITATIVE` con local/continuidad válidas, y
ocultar solo W `INVALID`, pérdida real o epoch nuevo sin anchor.

La corrección se aplicó en la prueba 235: build correcto, CTest 12/12 y
simulación `success=true`. No se analizaron logs; queda la confirmación visual
del usuario.
