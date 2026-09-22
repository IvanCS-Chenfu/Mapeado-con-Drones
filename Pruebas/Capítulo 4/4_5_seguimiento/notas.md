# Prueba 4.5: influencia del tiempo y la velocidad

## Objetivo

Comparar el seguimiento de la misma ruta con cuatro configuraciones: dos
trayectorias cúbicas y dos trapezoidales. Todas las pruebas usan un único
dron, fuente `GT`, mundo `empty`, GUI de Gazebo y los mismos nueve waypoints
absolutos. ORB-SLAM3, Fase 6, servidor global, fiduciales y GUIs auxiliares
permanecen desactivados.

Después de cada recorrido se mantienen 60 s de simulación para la grabación
manual. El vídeo no se guarda automáticamente en el repositorio.

## Waypoints comunes

```text
( 0,  0, 1,   0 deg)
( 8,  0, 1,   0 deg)
( 8,  8, 2,  45 deg)
( 0,  8, 2,  90 deg)
( 0,  0, 3, 180 deg)
(-8,  0, 3, 180 deg)
(-8, -8, 1,-135 deg)
( 0, -8, 1, -90 deg)
( 0,  0, 1,   0 deg)
```

La posición de spawn no coincide con el primer waypoint, por lo que el tiempo
y las gráficas incluyen el desplazamiento inicial hasta `(0, 0, 1)`.

## Configuraciones

| Prueba | Perfil | Configuración adicional |
|---|---|---|
| Cúbica lenta | `tipo_trayectoria: 0` | tiempos 24 s para el primer waypoint y 16 s para los restantes |
| Cúbica normal | `tipo_trayectoria: 0` | tiempos 12 s para el primer waypoint y 8 s para los restantes |
| Trapezoidal lenta | `tipo_trayectoria: 1` | `v_max_lin=0.8 m/s`, `v_max_ang=0.5 rad/s`, esperas de 4 s |
| Trapezoidal rápida | `tipo_trayectoria: 1` | `v_max_lin=1.6 m/s`, `v_max_ang=1.0 rad/s`, esperas de 2 s |

En la trapezoidal lenta el movimiento conserva la velocidad nominal; el efecto
lento procede de duplicar el asentamiento entre waypoints. En la rápida se
duplican los límites de velocidad mediante `trajectory_fast.yaml`. Los CSV
confirman aproximadamente `0.8 m/s` frente a `1.6 m/s` en X/Y.

## Tiempos medidos

La suma de `t_total` es el tiempo efectivo de movimiento de los nueve goals.
La duración registrada es el intervalo común entre el primer y el último dato
sincronizado e incluye las esperas entre waypoints y la transición inicial.

| Prueba | `t_total` de goals [s] | Duración registrada [s] |
|---|---:|---:|
| Cúbica lenta | 152.000 | 153.260 |
| Cúbica normal | 76.000 | 76.950 |
| Trapezoidal lenta | 141.600 | 174.590 |
| Trapezoidal rápida | 93.546 | 110.480 |

Los tiempos de los segmentos quedan conservados en los logs reducidos de cada
ejecución. La trapezoidal lenta suma además 32 s de asentamientos y la rápida
16 s.

## Resultados cuantitativos

| Métrica | Cúbica lenta | Cúbica normal | Trapezoidal lenta | Trapezoidal rápida |
|---|---:|---:|---:|---:|
| RMSE de posición [m] | 0.023118 | 0.148723 | 0.025235 | 0.077503 |
| MAE de posición [m] | 0.017730 | 0.117639 | 0.021206 | 0.063862 |
| Error máximo de posición [m] | 0.090066 | 0.514862 | 0.058969 | 0.210434 |
| Error final de posición [m] | 0.016373 | 0.133695 | 0.009549 | 0.038231 |

El resultado confirma la tendencia esperada: duplicar los tiempos cúbicos
reduce claramente el error, mientras que duplicar la velocidad trapezoidal
incrementa el error. En la comparación trapezoidal, la variante lenta también
dispone de más tiempo de asentamiento entre segmentos.

## Artefactos

- Configuraciones: `configuracion/`.
- Datos sincronizados: `datos/cubica_lenta/`, `cubica_normal/`,
  `trapezoidal_lenta/` y `trapezoidal_rapida/`.
- Gráficas XY, matrices 3x4 y errores: `figuras/<variante>/`.
- Métricas y tabla: `resultados/`.
- Logs reducidos: `codex/archivos_auxiliares/logs/prueba_f45_<variante>.reduced.log`.

## Movimiento único lento

Se añaden dos ensayos de un único tramo medido desde `(0, 0, 1, 0 deg)` hasta
`(10, 10, 3, 180 deg)`: uno cúbico de 24 s por eje y otro trapezoidal con los
límites nominales `0.8 m/s` y `0.5 rad/s`. El despegue previo desde la altura
de spawn y los 3 s de asentamiento no pertenecen al análisis: el procesador
selecciona el último goal mediante el reinicio de `t_act`. Cada ensayo genera
solo `gt_vs_trayectoria_3x4.png` y `errores_3x4.png`, con tiempo relativo en el
eje horizontal, bajo `figuras/movimiento_unico_<perfil>/`.

| Perfil | Muestras sincronizadas | Duración medida [s] | RMSE posición [m] | Error máximo de posición [m] |
|---|---:|---:|---:|---:|
| Cúbica lenta | 1199 | 23.990 | 0.016862 | 0.044641 |
| Trapezoidal lenta | 1143 | 22.860 | 0.038377 | 0.065571 |

En la trayectoria trapezoidal el objetivo de `180 deg` se representa como
`-180 deg`: ambas expresiones describen la misma orientación y el generador
elige el giro angular más corto. Las figuras conservan ese signo para reflejar
la referencia realmente aplicada.

La primera ejecución técnica de la trapezoidal rápida se descartó porque el
override numérico de launch no llegaba al nodo. Se repitió usando
`trajectory_fast.yaml`; solo esta segunda captura forma parte de los resultados.
