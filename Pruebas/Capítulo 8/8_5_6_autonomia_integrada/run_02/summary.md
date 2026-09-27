# 8.5.6 / run_02 - Resumen de evidencia

## Resultado

**PARCIAL.** El escenario `autonomous_gt_integrated_8_5_6` terminó con `success=true` y el servidor permaneció estable. La ejecución demuestra el ciclo autónomo real de D2, pero no el movimiento posterior de D1.

## Cadena observada

```text
asignar D1/D2
-> LOOK_AND_CAPTURE de ambos
-> depth y materialización de ambos
-> D* / reserva / MOVE_AND_CAPTURE de D2
-> STOP seguro por corredor OCCUPIED/INFLATED
-> nueva selección D2
-> D* / reserva / MOVE_AND_CAPTURE
-> VIEW_ADVANCE / DEPTH_OCCUPIED / coverage
-> nueva selección D2
-> D* / reserva / MOVE_AND_CAPTURE
-> segundo VIEW_ADVANCE / coverage
```

D1 recibió `map_section_level_0_AB` y D2 `map_section_level_0_BC`. Ambos completaron la primera captura `vista_unknown` con `depth=1`. D1 no tuvo una continuación posterior. D2 materializó también una `vista_pared`, dos `view_advance` y otros `vista_unknown`.

## Métricas

| Métrica | Valor |
|---|---:|
| Capturas materializadas | 8 |
| Eventos de coverage | 2 |
| Reservas D* | 3 |
| Máximo de vóxeles reservados | 204 |
| Planes D* de D2 | 3 (2, 2 y 3 waypoints) |
| Claims activos finales D2 | 22 / 192 |
| Coverage final D2 | 11,46 % |
| Recorrido post-handoff D1 | 0,00 m |
| Recorrido post-handoff D2 | 2,31 m |
| Revisión final de mapa | 1478 |
| Vóxeles raw finales | 15153 |

La primera reserva de D2 encontró un cambio de corredor y se detuvo de forma segura (`occupied_or_inflated_corridor`). La planificación se retomó posteriormente; no se relajó la política estricta FREE.

## Limitación

D1 quedó después de la primera materialización sin un nuevo workflow, plan, reserva ni movimiento. Por ello el requisito de movimiento autónomo de ambos drones no queda demostrado, ni puede afirmarse un conflicto entre reservas de drones distintos.

Los datos de origen están en `raw/`; las tablas derivadas en `processed/` y las cuatro figuras SVG requeridas en `figures/`.
