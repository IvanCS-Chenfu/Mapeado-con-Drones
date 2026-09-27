# Prueba 7.8.1 - Trayectoria corta usando ORB

Cada perfil usa un dron, la misma aproximación GT a `(0,-10,1,90°)`, autoridad ORB efectiva y el tramo hasta `(-7,-10,1,90°)`. GT se conserva solo para evaluación; el control del tramo evaluado usa exclusivamente ORB y no permite fallback.

## Cúbica

| Ejecucion | Estado | Resultado |
|---|---|---|
| run_01 | Conseguida | Tramo cúbico ORB de 7 m completado en 24 s; sin fallback, sin tracking no OK y llegada ORB a 0.220 m del destino. |

- 1180 muestras ORB; RMSE seguimiento 0.0934 m, máximo 0.1828 m y RMSE yaw 0.0060 rad.
- ORB-GT alineado: RMSE 0.0485 m, máximo 0.1514 m, final 0.0705 m.
- Estado visual: 19 cambios de KF, 0 muestras no OK, ORB 100.0%, 0 fallbacks, edad media/máxima 0.0004/0.0130 s.

## Trapezoidal

| Ejecucion | Estado | Resultado |
|---|---|---|
| run_01 | Conseguida | Tramo trapezoidal ORB de 7 m completado en 13.666 s; sin fallback, sin tracking no OK y llegada ORB a 0.070 m del destino. |

- 668 muestras ORB; RMSE seguimiento 0.0715 m, máximo 0.1629 m y RMSE yaw 0.0058 rad.
- ORB-GT alineado: RMSE 0.0363 m, máximo 0.1130 m, final 0.0564 m.
- Estado visual: 13 cambios de KF, 0 muestras no OK, ORB 100.0%, 0 fallbacks, edad media/máxima 0.0006/0.0128 s.

## Comparación

| Métrica | Cúbica | Trapezoidal |
|---|---:|---:|
| Duración tramo ORB | 24.000 s | 13.666 s |
| RMSE posición seguimiento | 0.0934 m | 0.0715 m |
| Máximo posición seguimiento | 0.1828 m | 0.1629 m |
| RMSE velocidad | 0.3011 m/s | 0.3126 m/s |
| RMSE yaw | 0.0060 rad | 0.0058 rad |
| RMSE ORB-GT | 0.0485 m | 0.0363 m |
| Error ORB-GT final | 0.0705 m | 0.0564 m |
| Cambios de KF | 19 | 13 |
| Fallbacks | 0 | 0 |
| Edad media `NavigationState` | 0.0004 s | 0.0006 s |
| Edad máxima `NavigationState` | 0.0130 s | 0.0128 s |

Las dos ejecuciones cumplen el criterio de validez. Las cifras permiten comparar el comportamiento: la trapezoidal tuvo menor RMSE posicional y ORB-GT en esta ejecución, mientras que la cúbica tuvo un RMSE de velocidad ligeramente menor. No se declara un perfil mejor de forma automática.

## Figuras

Cada perfil conserva `figura_a_referencia_orb.png`, `figura_b_error_seguimiento.png`, `figura_c_orb_gt.png`, `figura_d_velocidad.png`, `gt_vs_trayectoria_3x4.png` y `errores_gt_vs_trayectoria_3x4.png` en su directorio `processed/`. Las láminas 3x4 organizan posición, velocidad y aceleración por X/Y/Z/yaw; la aceleración GT se deriva numéricamente de la velocidad GT registrada.
