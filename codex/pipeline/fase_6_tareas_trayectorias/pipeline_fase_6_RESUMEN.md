# Fase 6 - Tareas, inspeccion y navegacion autonoma

## Estado vigente

```text
Infraestructura de workers: parcial
Migracion semantica de inspeccion U: pendiente
Pruebas de la arquitectura objetivo: pendientes tras la migracion
```

Este documento y las subfases `6A` a `6L` sustituyen los contratos anteriores
de Fase 6. Los historiales conservan pruebas y decisiones cronologicas, pero
no especifican la arquitectura que se implementara a partir de ahora.

## Arquitectura acordada

La fase se organiza como workers FIFO finitos. Un worker consume un trabajo,
emite una orden o una continuacion y no espera movimiento, depth ni una
respuesta remota.

```text
registro -> asignacion de subROI -> eleccion de punto -> D*
                                             |              |
                                             |              v
                                             |     monitor/reservas/GUI
                                             v              |
                                      mirada + captura <----+
                                             |
                                       integrar evidencia
                                             |
EvidenceDatabase <- KeyframeEvidenceWorker  |
        |                                    |
        +--------> VoxelMapBuilder ----------+
                         |
                    delta/map_revision
```

El servidor manda `SubmitAutonomousCommand` al dron y recibe `accepted` de
inmediato. El dron ejecuta localmente y, al finalizar, llama
`ReportAutonomousResult`; ese callback solo valida, persiste y encola. Todo se
correlaciona con `drone_id`, `task_id`, `workflow_id`, `command_id` y
`map_epoch`.

`STOP` es local, asincrono y prioritario. Solo puede existir una orden normal
activa por dron. El monitor reserva el corredor antes del vuelo, reacciona a
deltas relevantes y limpia reservas y GUI en cualquier finalizacion.

`EvidenceDatabase` almacena fuentes reversibles relativas a KFs. Los workers
de KFs y depth escriben en ella; `VoxelMapBuilder` es el unico que materializa
el mundo, por deltas, reproyecciones y tombstones. Las continuaciones depth se
liberan solo cuando confirma `sources_applied`.

## Subfases vigentes

| Subfase | Responsabilidad |
| --- | --- |
| 6A | Contrato de workflows, IDs y colas FIFO. |
| 6B | Registro de drones y asignacion/cesion de subROIs. |
| 6C | Servicios desacoplados de orden y resultado. |
| 6D | Runtime local, STOP y `TRACKING_RISK`. Parcial: el dron ejecuta y reporta ordenes correlacionadas; falta que la seleccion nueva las produzca. |
| 6E | EvidenceDatabase y evidencia procedente de KFs. |
| 6F | Resultado depth autonomo, evidencia tipada y continuaciones por fuentes aplicadas. Parcial: integra tambien `RESULT_ABORTED` con depth; falta simulacion integrada desde workers finales. |
| 6G | Materializacion incremental del mapa por `VoxelMapBuilder`. |
| 6H | Seleccion de secciones U exteriores, coverage y relevo de subROI. Pendiente de migracion. |
| 6I | D* estrictamente FREE y despacho `MOVE_AND_CAPTURE`/prefijo FREE. Pendiente de migracion. |
| 6J | Monitor, reservas, STOP por corredor y GUI activa. |
| 6K | Fiduciales oportunistas durante una tarea. |
| 6L | Integracion, observabilidad, pruebas y cierre. |

## Flujo operativo

1. Un dron anclado y libre recibe un subROI cercano y una seccion pendiente de
   la U exterior, situada a dos voxeles de tres caras del subROI.
2. La pose de inspeccion combina coste de seccion, pared a 4 m, altura central
   y desplazamiento. Busca una fachada `OCCUPIED` score `>0.4` o una sonda si
   falta pared en una esquina; el yaw mira a la fachada.
3. Si pose y corredor son FREE, D* entrega `MOVE_AND_CAPTURE`; si la pose es
   UNKNOWN, `LOOK_AND_CAPTURE` solo la despeja. No se elige por score
   intermedio `0.2..0.6`.
4. Depth o KF escriben evidencia; `VoxelMapBuilder` aplica deltas, activa de
   forma atomica la seccion por impacto depth `OCCUPIED=1` o `SIN_FACHADA`, y
   libera la continuacion correlacionada.
5. D* ejecuta una ruta totalmente FREE o solo su prefijo FREE anterior al
   primer UNKNOWN. No cruza UNKNOWN y el prefijo no toma depth.
6. El monitor protege la trayectoria activa con reservas y STOP ante un
   conflicto real. Al terminar, el dron reporta y se encola la siguiente fase.

## Limites deliberados

- La migracion preserva temporalmente los contratos externos de fiducial de
  Fase 4/5; no reescribe su optimizacion.
- El depth puede no ser utilizable. En ese caso no se inventa FREE: se avanza
  solo por prefijo FREE posible y se selecciona de nuevo.
- Los reintentos por `RESERVED` no tienen limite inicial, pero retornan al final
  de FIFO o esperan una revision, sin bloqueo ocupado.
- La topologia avanzada de ramas y la nube densa global se tratan despues.

## Criterio global de cierre

La Fase 6 quedara lista cuando los workflows sean correlacionados e
idempotentes, los dos drones progresen sin esperas bloqueantes, las reservas y
STOP funcionen por delta, y el mapa se materialice por evidencia incremental
sin reconstruirlo completo en cada actualizacion.
