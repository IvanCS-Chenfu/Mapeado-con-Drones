# Prueba 8.2-B - Evolución de coverage

## Evidencia empleada

Esta prueba no necesitó una ejecución independiente. Se documenta con la ejecución conseguida [`8.4-B/run_14`](../8_4_B_view_wall/run_14/summary.md), que realizó una observación frontal válida de la tarea `map_section_level_0_AB`.

## Secuencia comprobada

| Hito | Evidencia |
|---|---|
| Tarea y sección pendientes | La fachada se inicializó con `u_sections=192`; el workflow fijo quedó asociado a la sección `111`. |
| Movimiento y captura | D1 ejecutó `MOVE_AND_CAPTURE` a `(4,-9,1,90 grados)` y finalizó con `depth=1`. |
| Materialización depth | Se escribieron tres fuentes: `FREE=1`, `DIRECT_FREE=1` y `OCCUPIED=1`. |
| Actualización coverage | La fuente `depth_occupied:2108:1:0:33` quedó pendiente para la sección `111` y, tras materializarse, produjo `claims=13`, `active=13` y `changed=true`. |

## Resultado

**CONSEGUIDA.** La cobertura no se incrementó por el movimiento de D1 ni por la llegada a la pose. El cambio ocurrió después de que la fuente `OCCUPIED` de depth se materializara y reclamara secciones U.

El plan de fachada contiene 192 secciones. Por tanto, los 13 claims activos equivalen a un progreso derivado de `13 / 192 = 0,0677` (`6,77 %`). El log no publicó una muestra independiente del campo GUI `progress` antes y después; ese porcentaje se calcula directamente del recuento de secciones reportado por el servidor.

## Trazabilidad

- Ejecución fuente: `8.4-B/run_14`.
- Tarea: `map_section_level_0_AB`.
- Sección origen: `111`.
- Fuente que activa coverage: `depth_occupied:2108:1:0:33`.
- Marcadores: `[F6H-COVERAGE-SOURCE-PENDING]` y `[F6H-COVERAGE-CLAIMS]`.
- La GUI del servidor estuvo activa y el usuario confirmó visualmente la ejecución antes de cerrarla. No se guardó una captura GUI before/after en este directorio.

## Distinción respecto a 8.4-A

La 8.4-A también obtuvo depth, pero fue `VIEW_UNKNOWN`: escribió `FREE=1`, `DIRECT_FREE=0`, `OCCUPIED=0` y no produjo coverage. La evidencia de esta prueba procede exclusivamente de la captura frontal de 8.4-B, donde sí hubo `OCCUPIED` directo y claims de coverage.
