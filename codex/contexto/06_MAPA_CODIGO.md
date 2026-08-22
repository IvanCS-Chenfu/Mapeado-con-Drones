# 06 — Mapa de código para Codex

Para ahorrar tokens, leer primero:

```text
codex/contexto/00_BOOTSTRAP_MINIMO.md
codex/contexto/01_ESTADO_ACTUAL_RESUMEN.md
codex/contexto/08_POLITICA_TOKENS_DOCUMENTACION.md
```

Usar este mapa cuando ya se sepa qué paquete/subfase toca revisar.

## Propósito

Este documento ayuda a localizar rápido zonas importantes. Si el código contradice este documento, priorizar siempre el código actual y actualizar la documentación tras validar.

Usar `rg` antes de abrir archivos grandes.

## Fase 3 entregada

```text
Planificación activa: subfase_3A.md a subfase_3T.md
Subfase 3A: baseline histórica conservada
Subfase 3B: CONSEGUIDA; runtime/grafo base validados en prueba 78
Subfase 3C: CONSEGUIDA; raw y flujo principal activos
Subfase 3D: CONSEGUIDA; backend/poses validados
Subfase 3E: CONSEGUIDA; primer anchor fiducial y replay
Subfase 3F: CONSEGUIDA; implementacion, replay, live y layout visual aceptados
Subfase 3G: CONSEGUIDA; snapshots y rendimiento live 2/3 drones validados
Subfases 3H-3P: CONSEGUIDAS
Subfase 3Q: CONSEGUIDA; mejora futura documentada
Subfase 3R: CONSEGUIDA; scoring raw/fused
Subfase 3S: CONSEGUIDA; observabilidad y debug
Subfase 3T: CONSEGUIDA; limpieza/configuracion/handoff final
```

Los contratos residuales `12R-*`/`13`-`15` y los contratos absorbidos
de arquitectura, visualizador, regresion y rendimiento fueron retirados en 3T.
Sus historiales siguen disponibles en `historial/absorbidas/`.

## Servidor global activo y snapshot

Paquete:

```text
orbslam3_server
```

Archivo activo principal:

```text
orbslam3_server/src/global_map_server.cpp
```

Símbolos activos:

```text
orbslam3_server::GlobalMapServer / PrimaryQueue / SecondaryTaskQueue
WorkerLoop / SecondaryWorkerLoop / BackpressureHysteresis
[F3C-PRIMARY-ENQUEUE] / [F3E-FID-FIRST-ANCHOR]
[F3F-SCORE-UPDATE] / [F3F-GLOBALMAP-PUBLISH]
```

Búsquedas útiles:

```bash
rg -n "class GlobalMapServer|WorkerLoop|BuildAndPublishGlobalMap|F3F-" orbslam3_server/src
rg -n "class PrimaryQueue|MarkReady|BackpressureHysteresis" orbslam3_server/include
rg -n "RawMapDatabase|InsertDelta|ProcessLoopTask" orbslam3_server orbslam3_multi
```

El servidor recibe `OrbMap` delta, delega en `SparseGlobalBackend` y publica la
vista coherente cloud/KFs al final de la misma tarea principal.

## Backend objetivo

Paquete:

```text
orbslam3_multi
```

Estado activo hasta 3R:

```text
orbslam3_multi/include/orbslam3_multi/raw_map_types.hpp
orbslam3_multi/include/orbslam3_multi/raw_map_database.hpp
orbslam3_multi/include/orbslam3_multi/global_pose_types.hpp
orbslam3_multi/include/orbslam3_multi/global_pose_store.hpp
orbslam3_multi/include/orbslam3_multi/sparse_global_backend.hpp
orbslam3_multi/include/orbslam3_multi/landmark_score_manager.hpp
orbslam3_multi/include/orbslam3_multi/global_map_builder.hpp
orbslam3_multi/src/raw_map_database.cpp
orbslam3_multi/src/global_pose_store.cpp
orbslam3_multi/src/sparse_global_backend.cpp
orbslam3_multi/src/landmark_score_manager.cpp
orbslam3_multi/src/global_map_builder.cpp
orbslam3_multi/test/test_raw_map_database.cpp
orbslam3_multi/test/test_sparse_global_backend.cpp
orbslam3_multi/test/test_landmark_score_manager.cpp
orbslam3_multi/test/test_global_map_builder.cpp
```

Las copias antiguas fueron retiradas en 3T. Para recuperar una decisión
histórica se consulta primero el historial de subfase y, solo si no basta, el
checkpoint Git `1b96a7a`.

## Mapa de capacidades ejecutadas

| Subfase | Archivo | Código esperado |
|---|---|---|
| 3A | `subfase_3A.md` | Realizada: no corrigió código; capturó baseline del servidor actual. |
| 3B | `subfase_3B.md` | Conseguida: baseline vacía y grafo live de dos nodos/cero aristas, validados técnica y visualmente. |
| 3C | `subfase_3C.md` | Conseguida de nuevo: deltas raw, `PrimaryQueue`/`PrimaryWorker`, journal/replay, backpressure 8/2 y grafo 6/5; snapshots quedan para 3G. |
| 3D | `subfase_3D.md` | Realizada: `GlobalPoseStore`, replay debug y separación raw/world; la herencia simple queda obsoleta para cola fiducial larga. |
| 3E | `subfase_3E.md` | Realizada: `FiducialAnchorManager`, anclaje inicial real por fiducial simulado y `.record` v2 con observaciones fiduciales. |
| 3F | `subfase_3F.md` | Conseguida: score y builder incrementales, cloud/KFs coherentes, replay/live correctos y layout web final aceptado. |
| 3G | `subfase_3G.md` | Conseguida: snapshots selectivos, dirty diferido, record delta-only y rendimiento live. |
| 3H | `subfase_3H.md` | Conseguida: revisit, tarea MAX, queue/worker, stale, visitas v3 y mission gate. |
| 3I | `subfase_3I.md` | Conseguida: grafo temporal mono-submapa 30/20 e inactivos filtrados. |
| 3J | `subfase_3J.md` | Conseguida: solver SE(3) privado, control fijo y target absoluto. |
| 3K | `subfase_3K.md` | Conseguida: commit atomico, revision retry, late-window/tail y dirty KFs. |
| 3L | `subfase_3L.md` | Conseguida: validacion/refinamiento y pruebas tecnicas/visuales. |
| 3M | `subfase_3M.md` | Conseguida: `CovisibilityDatabase` importa aristas ORB-SLAM3 y `PoseGraphBuilder` consulta ventanas con aristas confirmadas. |
| 3N | `subfase_3N.md` | Conseguida: `LoopDetector` BoW real, filtros y skip por covisibilidad confirmada; live/replay validados. |
| 3O | `subfase_3O.md` | `SubcloudLoopVerifier`. |
| 3P | `subfase_3P.md` | Conseguida: `LoopDecisionManager`, fusion transitiva y commit incremental. |
| 3Q | `subfase_3Q.md` | Conseguida: optimizacion covisible loop/fiducial; incidencia 194 y mejora futura conservadas. |
| 3R | `subfase_3R.md` | Conseguida: scoring centralizado y fused tracks. |
| 3S | `subfase_3S.md` | Conseguida: `fase3_debug.yaml` consumido por launches y modo silencioso validado. |
| 3T | `subfase_3T.md` | Cierre documental y handoff. |

Las auditorias provisionales se cerraron con arquitectura, visualizacion,
regresiones y limites ya implantados. 3T retiro sus contratos independientes y
conserva sus resumenes historicos.

## Wrapper ORB-SLAM3

Paquete:

```text
orbslam3_ros2
```

Búsqueda:

```bash
rg -n "BuildOrbMap|FillMapPointMsg|FillKeyFrameMsg|map_epoch|orb_map_delta|get_full_map|HashMapPoint|HashKeyFrame" orbslam3_ros2
```

Regla: no tocar salvo necesidad fuerte.

## Mensajes ROS

Paquete:

```text
orbslam3_msgs
```

Búsqueda:

```bash
find orbslam3_msgs -maxdepth 3 -type f
```

Regla: no rediseñar ni añadir score global durante Fase 1 salvo subfase explícita.

## Simulación y escenarios

Paquete:

```text
simulacion_dron
```

Launch oficial:

```bash
ros2 launch simulacion_dron multi_dron.launch.py
```

YAMLs automáticos:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml
codex/archivos_auxiliares/trayectorias/tray_prueba_2.yaml
...
```

Pruebas tipicas reutilizables:

```text
codex/contexto/pruebas_clave/pruebas_tipicas.md
```

Regla: `tray_prueba_X.yaml` es el alias mecanico que consume `run_simulation.sh`; una trayectoria estable debe conservarse tambien con nombre semantico de prueba tipica y documentarse en historial.

Logs:

```text
SCENARIO-RUNNER
SIM-DONE
SIM-EXIT-CODE
```

## Herramientas Codex

```text
codex/herramientas/build_selected_packages.sh
codex/herramientas/reduce_build_log.sh
codex/herramientas/run_simulation.sh
codex/herramientas/reduce_simulation_log.sh
```

Build habitual de Fase 3:

```bash
./codex/herramientas/build_selected_packages.sh orbslam3_msgs orbslam3_multi orbslam3_server
```

Si se toca simulación:

```bash
./codex/herramientas/build_selected_packages.sh dron_individual simulacion_dron
```

Los logs completos terminan en `codex/archivos_auxiliares/logs/*.log`; los reducidos terminan en `*.reduced.log`.
