# Visualizador `pipeline_flow`

<!-- ACUERDOS_CIERRE_F2_2026_08_24_START -->
## Semántica de debug y deuda de consumo detectada

> **Vigencia:** acuerdo cerrado el 2026-08-24. Este bloque prevalece sobre cualquier
> frase anterior incompatible del mismo documento. No borra ni reescribe evidencia
> histórica; distingue siempre entre estado actual, deuda conocida y arquitectura objetivo.

`pipeline_flow` es observabilidad interna de Fase 3 y no debe trabajar cuando está
desactivado.

Estado actual detectado durante la auditoría de Fase 2:
- con `debug_pipeline_flow_web=false` no arranca el bridge/web;
- sin embargo `GlobalMapServer` sigue creando/publicando `/global_mapping/flow_events`
  y construyendo JSON/eventos. Esto es una deuda a corregir en la implementación del
  cierre de Fase 2.

Objetivo:
```text
debug_pipeline_flow_web=false
=> sin bridge + sin HTTP/SSE + sin navegador
=> sin publisher/eventos/serialización específicos de pipeline_flow
```

La comprobación debe hacerse también en el lado productor, no solo mirando procesos web.
`system_architecture` no debe necesitar `/global_mapping/flow_events`.
<!-- ACUERDOS_CIERRE_F2_2026_08_24_END -->

## Estado activo

Visualizador web incremental de Fase 3. El bridge transforma
`/global_mapping/flow_events` en SSE y sirve Cytoscape.js en localhost.

```text
src/visualizer/pipeline_flow_bridge.py
src/visualizer/pipeline_flow_browser.py
web/pipeline_flow/{index.html,styles.css,graph_definition.js,app.js}
test/test_pipeline_flow_contract.py
```

El helper espera `/health=ready` y abre una sola pestaña con cursor fresco. El
transporte conserva un buffer de 512 eventos, reconexion por `Last-Event-ID`,
`state_reset` para cursores expirados y ningun canal web->ROS.

## Topologia 3P

23 nodos y 39 aristas. Mantiene el flujo principal/fiducial/loop y añade:

```text
FiducialAnchorManager --opt_fid / MAX--> SecondaryTaskQueue
SecondaryTaskQueue --dequeue/start--> SecondaryWorker
SecondaryWorker --retry / LOW--> SecondaryTaskQueue
SecondaryWorker --build graph--> PoseGraphBuilder
PoseGraphBuilder --solve--> OptimizationManager
OptimizationManager --validate--> Validation
SecondaryWorker --STALE--> Validation
Validation --atomic commit--> GlobalPoseStore
GlobalPoseStore --pose/anchor dirty--> GlobalMapBuilder

RawMapDatabase --MEDIA--> SecondaryTaskQueue
SecondaryWorker --> CovisibilityDatabase --> SecondaryTaskQueue
SecondaryWorker --> LoopDetector --> LoopBoWIndex
LoopBoWIndex --> SubcloudLoopVerifier --> LoopDecision
LoopDecision --> LoopAnchorConstraintStore --> GlobalPoseStore
LoopDecision --> FusedLandmarkManager
FusedLandmarkManager --> CovisibilityDatabase
FusedLandmarkManager --> LandmarkScoreManager
FusedLandmarkManager --> GlobalMapBuilder
```

La arista `GlobalPoseStore -> GlobalMapBuilder` solo indica KFs dirty; no
representa una reconstruccion secundaria. El builder se ejecuta cuando vuelve
a llegar una tarea principal.

El escritorio usa tres bandas inspiradas en el flujo real: principal en la
superior, fiduciales/poses/optimizacion en la central y cola/loops/fusion en la
inferior. Las columnas y bandas tienen separacion amplia para distinguir
vertices, labels y aristas; las aristas opuestas queue/worker usan curvas
separadas y `score evidence` rodea el bloque de anchor. Movil conserva su
columna estable. Cloud y MarkerArray mantienen curvas paralelas hacia RViz2.

## Eventos

Las aristas se activan solo por eventos reales emitidos por el servidor. La
revalidacion `STALE` evita mostrar grafo/solver cuando una tarea ya fue
corregida. Backpressure conserva el enlace servidor->mission gate.

En 3R el nodo `LandmarkScoreManager` describe base ORB, distancia,
aislamiento y score fused derivado. La arista desde fusion representa inliers
raw, media fused y visibilidad solo diagnostica; no afirma penalizaciones
sparse. Los eventos transportan cantidades/revisiones, nunca arrays completos
de score. El contrato web 1/1 pasa tras el cambio.

Los pulsos primarios siguen durando 240 ms. Para secundario, `app.js` mantiene
owners por `task_id`: `secondary_task_lifecycle/start` abre la tarea, cada etapa
anade acumulativamente nodos/aristas al camino iluminado y `done` libera el
conjunto tras 420 ms. Dos tareas no comparten temporizadores ni apagan recursos
que otra siga usando. `state_reset` limpia owners y timers.

## Referencias

```text
rg -n "phase|fused_landmark_manager|covisibility_database|subcloud_loop_verifier" \
  simulacion_dron/web/pipeline_flow/graph_definition.js
rg -n "secondaryTasks|secondary_task_lifecycle" \
  simulacion_dron/web/pipeline_flow/app.js
rg -n "F3P-FLOW-WEB-READY" simulacion_dron/src/visualizer/pipeline_flow_bridge.py
```

- marcador: `[F3P-FLOW-WEB-READY] ... topology=23_nodes_39_edges`;
- contrato Python: 9/9, incluido lifecycle y camino secundario latched;
- live 154: bridge listo, tarea de anchor loop completa y cola secundaria
  drenada. El usuario observa el camino loop permanentemente iluminado. No hay
  tarea bloqueada: 2301 tareas consecutivas y `SECONDARY_DONE_HOLD_MS=420`
  solapan owners recientes. La semantica visual queda abierta para correccion.
- prueba 160: bridge listo con topologia 23/38 y lifecycle secundario drenado;
  el usuario confirma posteriormente que el grafo web se vio muy bien.
- prueba 161: contrato 9/9 con topologia 23/39; la arista de retorno solo se
  activa cuando un stale/rollback crea realmente otra BAJA.
- pulido posterior 3P: contrato fuente 9/9 y capturas desktop 1440x900/movil
  390x844; visualizador aislado abierto sin Gazebo en
  `http://127.0.0.1:8765/` para revision del usuario.

El snapshot previo fue retirado en 3T después de validar que esta copia activa
cubre topología, lifecycle, reconexión y apertura desde launch.
