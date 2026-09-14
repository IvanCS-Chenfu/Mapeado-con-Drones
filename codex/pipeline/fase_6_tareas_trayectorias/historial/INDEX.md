# Historial - Fase 6

| Subfase | Estado | Resumen |
| --- | --- | --- |
| 6A | CONSEGUIDA | Arquitectura, configuracion y telemetria base validadas. |
| 6B | CONSEGUIDA | Geometria real de 12 subROIs y visualizacion GUI validadas. |
| 6C | CONSEGUIDA | Contratos, registro idempotente y snapshots validados. |
| 6D | PARCIAL | Voxel reversible OCCUPIED/FREE, soporte reversible de cuatro MapPoints y consumo delta coalescido; depth/resync real pendientes. |
| 6E | CONSEGUIDA | Tareas regionales, asignacion inicial y snapshot real. |
| 6F | CONSEGUIDA | Confirmacion local sin ejecucion autonoma. |
| 6G | CONSEGUIDA | D* Lite jerarquico con snapshot/delta; demo visual remota conseguida en 629. |
| 6H | PARCIAL | Coverage volumetrico y metas UNKNOWN; `no_safe_escape` reintenta sin bloquear. Pendientes: puntualidad bajo backlog y lifecycle de rama. |
| 6I | PARCIAL | FREE reversible, ruta D* depurada y empalmes Pol3 C1 con mínimo 8 s/ventana 3 s; 678 valida el recorrido automático y 679 el lifecycle visual de ruta `ACTIVE`. Pendientes: suavidad física, barrido y decisión sobre inflación. |
| 6J | PARCIAL | Reserva física por owner, replan previo a espera, HOLD local, STOP causal y proyección GUI RESERVED validados con dos drones en 687; falta sampler curvo compartido y prueba de geometrías más variadas. |
| 6K | PARCIAL | FIFO de subtareas y runtime por dron validados en 686, incluida ejecución ACTIVE paralela y STOP/HOLD local; faltan prioridades completas y reparación estructural de ruta. |
| 6L | CONSEGUIDA | 717 valida yaw horizontal; 721 valida sector vertical, STOP y pitch local -25 grados con epoch estable, sin ruta ni reserva de servidor para el giro. |
| 6M | CONSEGUIDA | Transporte fisico y persistencia yaw/pitch ordinaria validados en 700/701; no hay politica de vista ordinaria y depth/normales quedan para 6N. |
| 6N | PARCIAL | Barrido de fachada implementado; 737 valida la guarda fisica y 738 revela bloqueo por cola global y perdida del frame depth exacto. |

6N tiene implementacion y validacion parcial. 6O y 6P no tienen ejecucion
funcional completa todavia. Desde la reordenacion documental, 6N es depth, 6O
es GO_TO/ANCHOR/fiduciales y 6P es el cierre de integracion; los historiales
anteriores que nombren el viejo 6P depth se conservan como evidencia
cronologica.
