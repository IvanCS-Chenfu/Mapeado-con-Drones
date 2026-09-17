# Historial - Fase 6

| Subfase | Estado | Resumen |
| --- | --- | --- |
| 6A | PARCIAL | Nueva base FIFO/IDs/STOP generacional compilada y CTest 8/8; productores legacy aun sin migrar. |
| 6B | PARCIAL | Asignacion por proximidad entra por `TASK_ASSIGNMENT` y crea `POINT_SELECTION`; falta su consumer y smoke ROS. |
| 6C | PARCIAL | Servicios autonomos replicados y compilados; falta aceptar llamadas reales y ejecutar/reportar. |
| 6D | PARCIAL | Runtime local de ordenes autonomas, STOP/`TRACKING_RISK` y terminal correlacionado compilados; falta producer de ordenes desde la seleccion nueva y prueba integrada. |
| 6E | PARCIAL | Migracion nueva: `EvidenceDatabase` y `KeyframeEvidenceWorker` compilan; CTest pendiente por limite externo. |
| 6F | PARCIAL | Depth autonomo desacoplado y `VIEW_ADVANCE` validado en 779: captura, fuentes y continuacion tras `sources_applied`; quedan STOP y avance perceptivo de fachada. |
| 6G | PARCIAL | Claims U reversibles por fuente/KF: 786 materializa 18 secciones; falta ver retirada visual tras optimizacion/tombstone depth. |
| 6H | PARCIAL | 786 valida activacion U por evidencia frontal y vecinos; faltan relevo `TO_FINISH` y progreso lateral sostenido. |
| 6I | PARCIAL | Inflacion adicional `1`, reservas y diagnostico causal validados en 786; quedan STOPs por cambios navegables no resueltos y validacion fisica sostenida. |
| 6J | PARCIAL | Reserva física por owner, replan previo a espera, HOLD local, STOP causal y proyección GUI RESERVED validados con dos drones en 687; falta sampler curvo compartido y prueba de geometrías más variadas. |
| 6K | PARCIAL | FIFO de subtareas y runtime por dron validados en 686, incluida ejecución ACTIVE paralela y STOP/HOLD local; faltan prioridades completas y reparación estructural de ruta. |
| 6L | CONSEGUIDA | 746 valida deteccion al 65 %, STOP y correccion `-25 deg` sin perdida ni cambio de epoch. |
| 6M | CONSEGUIDA | Transporte fisico y persistencia yaw/pitch ordinaria validados en 700/701; no hay politica de vista ordinaria y depth/normales quedan para 6N. |
| 6N | PARCIAL | Pareja depth atomica y fallback implementados. La 769 valida prioridad FIFO por pareja y KFs propios D1/D2; quedan criterios depth/cobertura y reducir reintentos de pose. |

6N tiene implementacion y validacion parcial. 6O y 6P no tienen ejecucion
funcional completa todavia. Desde la reordenacion documental, 6N es depth, 6O
es GO_TO/ANCHOR/fiduciales y 6P es el cierre de integracion; los historiales
anteriores que nombren el viejo 6P depth se conservan como evidencia
cronologica.
