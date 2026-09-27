# Run 04 - Repetición con umbrales depth relajados

## Resultado

**PARCIAL.** El servidor no abortó, pero la ejecución no cumple el criterio integrado de avance autónomo de ambos drones. run_03 sigue siendo la evidencia concluida de 8.5.6.

## Evidencia

- scenario_runner agotó la espera del handoff; el servicio se atendió tarde y el servidor asignó las dos tareas sin caerse.
- D1 quedó RUNNING en map_section_level_0_AB sin comando autónomo. La entrada POINT_SELECTION fue consumida antes de inicializar su runtime de fachada y no se reencoló.
- D2 ejecutó 8 observaciones depth, 3 reservas D* y 6 eventos de claims. Las capturas de avance produjeron DEPTH_OCCUPIED, por lo que normal_min_confidence=0,50 e incidencia 45° son efectivos para D2.
- Métricas finales: revisión de mapa 1291, 14616 vóxeles raw y máximo de 305 vóxeles reservados.

No aparece malloc, FATAL ni terminación de task_server en el log reducido. Las pausas de D2 corresponden al pipeline de integración y planificación serializado, agravado por el retraso inicial del handoff.
