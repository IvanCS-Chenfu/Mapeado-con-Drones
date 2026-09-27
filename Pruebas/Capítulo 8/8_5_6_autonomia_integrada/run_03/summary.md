# 8.5.6 / run_03 - Resumen de evidencia

## Resultado

**CONSEGUIDA.** El escenario `autonomous_gt_integrated_8_5_6` terminó con
`success=true`. D1 y D2 ejecutaron autonomía real bajo decisión del servidor:
selección de tarea, D* Lite, reserva, movimiento y captura depth. No hubo
abortos de `task_server`.

## Métricas

| Métrica | Valor |
|---|---:|
| Observaciones depth materializadas | 26 |
| Commits de reserva | 18 |
| Máximo de vóxeles reservados | 573 |
| Recorrido post-handoff D1 | 31,30 m |
| Recorrido post-handoff D2 | 7,48 m |
| Claims U activos finales D1 | 13 / 192 |
| Coverage final D1 | 6,77 % |
| Claims U activos finales D2 | 15 / 192 |
| Coverage final D2 | 7,81 % |
| Revisión final de mapa | 5474 |
| Vóxeles raw finales | 16367 |

Las primeras capturas de algunos ciclos generaron solo evidencia `FREE`: depth
se tomó, pero no pudo activar coverage. Más adelante ambos drones generaron
`DEPTH_OCCUPIED`; D1 elevó `map_section_level_0_AB` a 13 claims activos y D2
elevó `map_section_level_1_AB` a 15.

## Límite de cobertura

La corrección para un objetivo `FREE` bloqueado por inflación deja el workflow
en planificación y solicita el prefijo FREE alcanzable más lejano. Esta
ejecución no activó directamente esa rama: D1 recibió inicialmente `UNKNOWN` y
usó el fallback FREE existente. La integración de los dos drones sí queda
validada; la cobertura directa de `action=prefix_free` requiere una escena
específica que fuerce esa condición.

Los datos de origen están en `raw/`; las tablas derivadas en `processed/` y
las figuras SVG en `figures/`.
