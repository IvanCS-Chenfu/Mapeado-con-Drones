# Prueba 8.5.6 - Autonomía integrada multidrón

## Conclusión vigente

**CONSEGUIDA en `run_03`.** Tras serializar el estado compartido y corregir la continuidad de un objetivo `FREE` bloqueado por inflación, ambos drones realizaron ciclos autónomos de planificación, reserva, movimiento y captura depth. El escenario terminó con `success=true`, sin abortos de `task_server`; D1 y D2 produjeron evidencia `DEPTH_OCCUPIED` y sus respectivas tareas elevaron la cobertura U a `13/192 = 6,77 %` y `15/192 = 7,81 %`.

La condición que bloqueó D1 en `run_02` ya no deja la tarea en una reselección silenciosa: cuando el recheck encuentra un objetivo `FREE` no transitable, conserva `TRAJECTORY_PLANNING` y arma el fallback existente de prefijo FREE alcanzable. La política sigue siendo estricta: no cruza `UNKNOWN`, `OCCUPIED` ni `RESERVED`.

## Evidencia de `run_02`

- Asignación: D1 recibió `map_section_level_0_AB`; D2, `map_section_level_0_BC`.
- D1 completó `LOOK_AND_CAPTURE`, con `depth=1` y fuente `vista_unknown` (`FREE=1`, `DIRECT_FREE=0`, `OCCUPIED=0`); quedó en `(-2,-10,1.01)` y recorrió `0.00 m` tras el handoff.
- D2 completó siete comandos autónomos antes del cierre y dejó un octavo `LOOK_AND_CAPTURE` aceptado. Sus tres planes D* tuvieron 2, 2 y 3 waypoints; las reservas comprometieron 130, 102 y 204 vóxeles.
- D2 pasó por `vista_pared` y dos `view_advance`. Los dos `DEPTH_OCCUPIED` reclamaron primero 17 y después 15 secciones, con 22 claims activos al final. El progreso de su tarea pasó de `0 %` a `8,85 %` y `11,46 %`.
- La telemetría termina con revisión de mapa `1478`, `15153` vóxeles raw y cero reservas activas. La reserva máxima observada fue de 204 vóxeles.
- El escenario terminó con `success=true`; al cierre no quedaban nodos ROS ni apareció `malloc`, `FATAL` ni muerte de proceso en la reducción de log.

## Diagnóstico de la detención de D1

D1 no recibió un `STOP` ni perdió su pose. Su primer `LOOK_AND_CAPTURE` terminó correctamente y materializó `FREE=1`. En la revalidación posterior, el servidor registró:

```text
[F6H-DEPTH-TARGET-RECHECK] drone=1 ... target=7:-41:4 state=free action=reselect
```

`action=reselect` con `state=free` solo aparece cuando esa celda FREE no es transitable en el `NavigationSnapshot` estricto. El perfil de fachada aplicado a D1 usa inflación `(3,3,2)` y la prueba añadió un voxel de margen, así que el punto libre quedó bloqueado por el volumen de seguridad aunque no estuviese OCCUPIED.

El comportamiento posterior es un defecto de continuidad, no una espera intencional: `RunPointSelectionWorker` guarda el ratio rechazado y reencola el workflow, pero `SelectFacadeTaskCandidate` no consume `rejected_inspection_target_ratios`. Por ello puede seleccionar repetidamente el mismo candidato FREE no transitable, sin enviar un comando, sin modificar `TaskState` y sin producir un log de rechazo. D1 permanece en `RUNNING` y quieto.

D2 no sufrió esa condición en su primer ciclo: su marcador fue `state=free action=plan`, por lo que generó D*, reserva y movimiento. La corrección aplicada evita la reselección silenciosa: conserva el workflow en planificación y hace que el planificador derive un prefijo FREE alcanzable.

## Evidencia de `run_03`

- El handoff abrió la autonomía a los `95,517 s`; el wrapper y el escenario terminaron con `success=true`.
- D1 recorrió `31,30 m` tras el handoff y comprometió 14 reservas D*. D2 recorrió `7,48 m` y comprometió 4; se observaron 18 commits de reserva y un máximo simultáneo de 573 vóxeles reservados.
- Se materializaron 26 observaciones depth. Muchas de las primeras fueron únicamente `FREE`, por lo que no cambiaron coverage; no faltó depth: sus rayos no cumplieron la clasificación de superficie directa (`normal_valid` e incidencia frontal máxima de 30 grados). D1 y D2 generaron después `DEPTH_OCCUPIED`, con 13 y 15 claims U activos finales, respectivamente.
- D1 terminó en `map_section_level_0_AB`, `RUNNING`, con `6,77 %`; D2 terminó en `map_section_level_1_AB`, `RUNNING`, con `7,81 %`. No se exige completar la fachada para esta prueba: su criterio es comprobar que ambos drones avanzan y aportan evidencia/cobertura bajo decisión del servidor.
- El mapa finalizó en revisión 5474, con 16367 vóxeles raw. No hubo `malloc`, `FATAL` ni muerte de `task_server`; el mensaje tardío de salida de la GUI de Gazebo ocurrió tras `SIM-DONE` y no afectó al servidor ni al resultado.

La nueva rama `action=prefix_free` quedó compilada y reutiliza el planificador estricto sobre el prefijo FREE más lejano de una ruta exploratoria. En `run_03` no se activó directamente: el primer recheck de D1 fue `UNKNOWN` y usó el fallback FREE ya existente. La prueba integrada valida el avance de ambos drones, pero una escena que fuerce expresamente `FREE` e inflado sigue siendo necesaria para cubrir esa rama de forma directa.

Diagnóstico detallado de las primeras capturas `FREE`: la normal se estima desde la nube estéreo aceptada, no desde el yaw de la orden. La primera `vista_pared` de D1 (`command ...:2:5`) aceptó depth pero tuvo `normal=false`, soporte 138 y confianza `0,548`, por debajo del mínimo interno `0,700`; no se creó `depth_occupied`. La captura posterior de D1 que sí creó coverage (`...:2:20`) tuvo `normal=true`, soporte 205 y confianza `0,704`. Por tanto el bloqueo era la coherencia geométrica de las normales locales, no la falta de movimiento ni de depth. Las capturas `vista_unknown` producen solo `FREE` por contrato incluso cuando su normal sea válida.

## Configuración posterior a `run_03`

Para la próxima ejecución de Fase 6, el launch reduce
`depth_normal_min_confidence` de `0,70` a `0,50` y amplía la incidencia directa
máxima de `30°` a `45°`. `run_03` conserva sus valores originales y no se usa
como evidencia de este ajuste. Se mantienen soporte mínimo, filtros de textura,
discontinuidad y rango; la repetición deberá contrastar el aumento de
`DEPTH_OCCUPIED` con posibles falsos ocupados.


## Validación de continuidad depth: runs 13--15

`run_13` y `run_14` fueron ejecuciones diagnósticas interrumpidas, no evidencia
final: confirmaron que ambos drones podían aceptar órdenes, pero la
materialización depth quedaba detrás de una cola de evidencia pasiva. En
`run_14`, D1 llegó a esperar `10,64 s` entre
`F6F-DEPTH-SOURCES-WRITTEN` y `F6F-DEPTH-SOURCES-APPLIED`; la cola superó 900
transacciones. La limitación previa de una transacción por tick evitaba
callbacks gigantes, pero la prioridad genérica `depth_*` no distinguía la
captura que desbloquea un workflow de las muestras pasivas, y cada transacción
forzaba un `RefreshNavigation` completo.

La corrección conserva el presupuesto de materialización, los vetos estrictos,
reservas, STOP, riesgo visual e inflación. `EvidenceDatabase` adelanta ahora
las `source_id` exactas de continuaciones pendientes junto con los
predecesores del mismo keyframe; el refresco de navegación pasivo se agrupa a
500 ms, mientras que una fuente de continuación aplicada fuerza publicación y
actualización inmediatas.

`run_15` VALIDÓ la corrección antes de terminar la ventana post-escenario por
haber reunido evidencia suficiente. D1 y D2 recibieron y encadenaron órdenes
concurrentes: D1 alcanzó el comando `...:3:13` y D2 `...:3:15`, incluyendo
planes D*, reservas, movimientos, depth y claims `DEPTH_OCCUPIED` en ambos.
La primera captura de D1 se aplicó en `0,17 s`; varias posteriores tardaron
menos de un segundo aun con la cola pasiva activa. El primer ciclo de D2 tardó
más por la llegada de la pose global del keyframe, pero no quedó bloqueado y
continuó con comandos posteriores. El capturador registró 46 eventos de tarea,
317 revisiones de VoxelMap, 1035 estados de navegación, revisión máxima 2287 y
469 vóxeles reservados simultáneamente. No hubo `malloc` ni `FATAL`; al cierre
manual no quedaban nodos ROS. Esta validación confirma el objetivo operativo:
el servidor mantiene activados y progresando a ambos drones sin que depth de
uno congele al otro.

`run_16` fue una repetición visual de `run_15`, detenida ordenadamente por el
usuario después de grabar y fotografiar una ejecución que calificó como
correcta. No cambia la evidencia técnica primaria. Antes del cierre ambos
drones habían aceptado órdenes autónomas repetidas (D1 hasta `...:3:15` y D2
hasta `...:3:20`), con ciclos de depth, aplicación de fuentes, reservas D* y
avances. No apareció `malloc` ni `FATAL`; tras cerrar el grupo de simulación,
`ros2 node list` quedó vacío.


## Artefactos

- [Resumen de run_03](run_03/summary.md)
- [Métricas procesadas](run_03/processed/metrics.json)
- [Figura A: trayectorias globales](run_03/figures/A_trayectorias_globales.svg)
- [Figura B: coverage temporal](run_03/figures/B_coverage_temporal.svg)
- [Figura C: reservas temporales](run_03/figures/C_reservas_temporales.svg)
- [Figura D: tipos de observación](run_03/figures/D_tipos_observacion.svg)

## Historial de ejecuciones

| Run | Resultado | Evidencia |
|---|---|---|
| `run_01` | NO CONSEGUIDA | El handoff asignó ambas tareas y D1 llegó a reservar y avanzar, pero `task_server` abortó con `malloc(): unaligned fastbin chunk detected` antes de que D2 progresara. |
| `run_02` | PARCIAL | Sin abortos: ambos drones hicieron depth y D2 cerró varios ciclos D*/reserva/movimiento/cobertura. D1 no recibió el siguiente ciclo autónomo. |
| `run_03` | CONSEGUIDA | Ambos drones planificaron, reservaron, se movieron y materializaron depth con `DEPTH_OCCUPIED` y coverage U; sin abortos de `task_server`. La rama específica de prefijo FREE no fue ejercitada directamente. |
| run_04 | PARCIAL | Repetición con normal >= 0,50 e incidencia <= 45°. D2 realizó reservas, movimiento y DEPTH_OCCUPIED; D1 quedó RUNNING sin comando por una carrera entre la inicialización de fachada y la cola POINT_SELECTION. El scenario_runner agotó además la espera del handoff, aunque el servidor permaneció vivo. No altera la conclusión de run_03. |
| run_13 | INTERRUMPIDA | Diagnóstico: D1 y D2 aceptaron órdenes, pero D1 quedó tras la cola de evidencia; se descubrió la reselección repetida de sección sin prefijo FREE y se corrigió. |
| run_14 | INTERRUMPIDA | Diagnóstico de rendimiento: la prioridad genérica `depth_*` no adelantaba la fuente de continuación y el refresh de navegación por transacción retrasaba la pose global de keyframe. |
| run_15 | VALIDACIÓN CONSEGUIDA | Tras priorizar `source_id` de continuaciones y agrupar el refresh pasivo, ambos drones encadenaron comandos, reservas, depth y coverage concurrentemente; cierre manual tras evidencia suficiente. |
| run_16 | CONFIRMACIÓN VISUAL CONSEGUIDA | Repetición exacta de run_15 para vídeo y fotos. Ambos drones progresaron con órdenes, depth, reservas y avances; se cerró a petición del usuario tras la grabación, sin errores fatales ni nodos residuales. |

## Diagnóstico de run_04

run_04 no es una caída del servidor: no hubo malloc, FATAL ni muerte de task_server. El handoff sí devolvió error al scenario_runner por exceder su espera de 10 s, pero el servicio se atendió después y asignó ambas tareas.

D1 recibió map_section_level_0_AB y se encoló POINT_SELECTION antes de que F6H-FACADE-INITIALIZED creara su runtime. RunPointSelectionWorker consume esa entrada, detecta runtime == nullptr y retorna sin reencolarla. Cuando D1 acepta la tarea y queda RUNNING, ya no conserva trabajo pendiente y no se despacha ningún comando. Es un defecto de carrera de inicialización, no una restricción del mapa, un STOP ni una pérdida de navegación.

D2 confirma que la relajación de depth funciona: registró 8 observaciones, 3 reservas D*, 6 eventos de claims, y fuentes DEPTH_OCCUPIED en avances. Sus pausas proceden de la cadena serializada resultado -> integración -> materialización -> recheck -> D* -> reserva; el primer ciclo tardó en especial porque los callbacks comparten el grupo de mapa y el handoff inicial quedó retrasado. run_04 requiere corregir el reintento de selección y hacer tolerante a reintentos la espera de handoff antes de reutilizarlo como evidencia integrada.

Artefactos de esta ejecución: [métricas](run_04/processed/metrics.json), [trayectorias](run_04/figures/A_trayectorias_globales.svg), [coverage](run_04/figures/B_coverage_temporal.svg) y [reservas](run_04/figures/C_reservas_temporales.svg).
