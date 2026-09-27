# Prueba 6.4.2: correccion por loop de regiones ya ancladas

## Fuente de evidencia

Esta prueba reutiliza la ejecucion `c6_2_4_loop_backpressure_v3`. Los dos
submapas participantes estaban anclados y cada resultado
`[F3Q-LOOP-OPT]` registra `submaps=2`, una arista de loop, aceptacion y commit.
No se usa Ground Truth: el objeto de estudio es la consistencia del grafo al
introducir una restriccion visual entre regiones ya globales.

La fuente procesada es el sublog F3 conservado en:

```text
Pruebas/Capitulo 6/6_2_4_loop_y_backpressure/datos_procesados/eventos_f3_reducidos.log
```

## Resultados

Se registraron cinco optimizaciones por loop. Todas declaran
`optimized=true`, `accepted=true`, `committed=true` y
`reason=atomic_covisible_loop_commit`. Las tablas en `datos_procesados/`
recogen ventana, controles, aristas, iteraciones, error, coste, tiempos y KFs
movidos. Las figuras muestran la reduccion medida y el orden temporal de los
commits.

El error de traslacion paso, respectivamente, de `2.141`, `1.117`, `0.727`,
`0.415` y `0.390 m` a `0.021`, `0.048`, `0.022`, `0.023` y `0.015 m`.
Los cinco commits corrigieron entre 121 y 180 KFs. El coste paso de los rangos
`18.411--112.355` a `2.958--6.656`.

Las figuras generadas son:

- `figuras/error_y_coste_loop.{png,pdf}`;
- `figuras/estructura_y_tiempos_loop.{png,pdf}`;
- `figuras/timeline_optimizaciones_loop.{png,pdf}`.

No se genera una figura XY antes/despues de KFs para esta prueba por acuerdo:
esa visualizacion se realizara en la optimizacion por fiducial (6.3.3). La
ejecucion reutilizada no guardo poses individuales de KFs antes y despues de
cada commit de loop.

## Reproduccion

```bash
PYTHONNOUSERSITE=1 python3 \
  'Pruebas/Capitulo 6/scripts/procesar_c6_4_2_optimizacion_loop.py' \
  --log 'Pruebas/Capitulo 6/6_2_4_loop_y_backpressure/datos_procesados/eventos_f3_reducidos.log' \
  --data-dir 'Pruebas/Capitulo 6/6_4_2_optimizacion_por_loop/datos_procesados' \
  --figure-dir 'Pruebas/Capitulo 6/6_4_2_optimizacion_por_loop/figuras'
```
