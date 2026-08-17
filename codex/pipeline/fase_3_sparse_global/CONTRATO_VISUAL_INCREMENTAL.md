# Contrato visual incremental de Fase 3

## Propiedad

`3B` es propietaria del motor de observabilidad: bridge ROS->SSE, aplicacion
web, semantica live y topologia base. Cada subfase posterior es propietaria de
añadir al grafo las clases y transferencias que active realmente.

El bridge y la web no son vertices del pipeline. Son infraestructura de
observacion. RViz2 se representa cuando recibe por primera vez una salida
global real.

## Reglas

- Un vertice solo aparece cuando su clase/componente existe en el codigo
  activo y participa en el runtime probado.
- Una arista solo aparece cuando existe la transferencia real. No se dibujan
  conexiones futuras, deseadas o simuladas.
- El evento se emite en la frontera de entrega, no al planificarla ni mediante
  un replay decorativo.
- La telemetria transporta IDs, revisiones, cantidades, prioridad, estado y
  tiempos; nunca nubes, descriptores, matrices o payloads pesados completos.
- La instrumentacion es acotada y no bloqueante. Perder telemetria no cambia el
  resultado funcional.
- Primera conexion SSE empieza live. Una reconexion recupera solo eventos aun
  disponibles mediante `Last-Event-ID`; un gap produce `state_reset`.
- El frontend drena por `requestAnimationFrame`, sin cola temporizada que
  convierta eventos antiguos en actividad presente.

## Incrementos previstos

| Subfase | Incremento visual propietario |
|---|---|
| 3B | `Wrappers ORB-SLAM3` y `GlobalMapServer`, cero aristas. |
| 3C | `PrimaryQueue`, `PrimaryWorker`, `RawMapDatabase` y `ScenarioRunner / MissionGate`; recepción live, enqueue, dequeue/start, commit raw y transición de backpressure. En replay no pulsa wrapper->server. |
| 3D | `GlobalPoseStore`; `RawInsertResult/ChangeSet` hacia poses. |
| 3E | `FiducialAnchorManager`; observacion y anchor inicial. |
| 3F | `GlobalMapBuilder` y `RViz2`; captura/build y cloud/KFs reales. |
| 3G | Aristas diferenciadas de full snapshot y reconciliacion. |
| 3H | Revisita fiducial, `opt_fid`, `SecondaryTaskQueue` y `SecondaryWorker`; prioridad MAX, enqueue, dequeue/start, tarea activa, revalidacion `STALE` y parada de mision. Se reservan HIGH/NORMAL sin actividad ficticia. |
| 3I | `PoseGraphBuilder`; entrada acotada y grafo privado. |
| 3J | `OptimizationManager`; grafo->propuesta privada. |
| 3K | Commit atomico en `GlobalPoseStore`, propagacion de ventana/late-window/tail y `pose_dirty` con solo IDs de KFs hacia `GlobalMapBuilder`. La cola y el worker ya pertenecen a 3H. |
| 3L | `Validation`; decisiones `STALE`, commit completo, commit parcial/refinamiento y fallo duro, sin apply/rollback visible. |
| 3M | `CovisibilityDatabase`; patch raw->covisibilidad. |
| 3N | `LoopDetector`; indexado y candidatos por `LoopTask`. |
| 3O | `SubcloudLoopVerifier`; candidatos->geometria verificada. |
| 3P | `LoopDecisionManager` y `FusedLandmarkManager`; decision/fusion. |
| 3Q | Ramas reales de optimizacion por loop y commit multi-base. |
| 3S | `LandmarkScoreManager`; eventos/patches de score y builder dirty. |
| 3T | IDs, revisiones y ownership completos en eventos ya existentes. |
| 3U | Auditoria de completitud, frescura, gaps, reconexion y stress visual. |
| 3V/3W | Regresion, limites y causas finales de backpressure sin duplicar el mission gate ni añadir topologia ficticia. |
| 3X | Retirada de rutas/aristas obsoletas junto con el codigo sustituido. |

La tabla es un reparto de propiedad, no permiso para adelantar componentes.
Si el diseño real cambia, la subfase debe actualizar su incremento antes de
implementarlo.

## Flujo acordado 3H-3L

Cuando la tarea supera la revalidacion, el grafo debe representar transferencias
reales con este orden:

```text
FiducialAnchorManager --opt_fid/MAX--> SecondaryTaskQueue
SecondaryTaskQueue --dequeue/start--> SecondaryWorker
SecondaryWorker --> PoseGraphBuilder
PoseGraphBuilder --> OptimizationManager
OptimizationManager --> Validation
Validation --full|partial--> GlobalPoseStore
GlobalPoseStore --pose_dirty(kf_ids)--> GlobalMapBuilder
```

La arista de observacion fiducial pulsa para todas las observaciones. La arista
`first_anchor` solo pulsa si el submapa no estaba anclado. Una tarea que queda en
umbral al dequeue termina como `STALE` y no activa grafo ni solver.

`pose_dirty` significa que `GlobalMapBuilder` ha registrado KFs movidos; no que
haya transformado ya la nube. El builder expande internamente esos KFs a MPs y
recalcula la geometria solo cuando lo ejecuta el siguiente `PrimaryInput`. El
worker secundario no publica ni espera confirmacion del builder, RViz2 o web.

Una pasada parcial valida conserva el mismo `task_id`, mantiene activo al worker
y vuelve a recorrer graph->solver->validation. Un fallo duro no activa commit ni
dirty y debe verse como estado bloqueante, no como exito o descarte silencioso.

## Cierre De Subfase

Toda subfase ejecutada debe validar el grafo en vivo. Si produce poses, KFs,
nube u otra geometria global, tambien debe validarse en RViz2. Si aun no
produce salida espacial, RViz2 debe arrancar sin mostrar resultados globales
incorrectos.

Una subfase con criterio visual pendiente permanece `PARCIAL` aunque build,
tests y scenario sean correctos. Solo pasa a `CONSEGUIDA` tras comprobar que la
topologia, orden, payload y resultado visual corresponden al runtime real.
