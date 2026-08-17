# Historial 3K.3 — Publicación por cobertura de KeyFrames corregidos

## 2026-07-23 18:20 — Subfase 3K — MapPoints tras apply por cobertura de KFs

- objetivo intentado:
  - hacer que una optimización aceptada en HTML/dry-run se refleje también en
    RViz2 mediante `/global_sparse_cloud`;
  - evitar que `GlobalMapBuilder` pierda MapPoints por tratar
    `submap_last_correction` como veto de todo el submapa;
  - repetir la prueba típica corta con ambos drones saliendo de fiducial 2,
    yendo por lados opuestos a fiducial 1 y parando allí.
- archivos modificados:
  - `orbslam3_multi/include/orbslam3_multi/global_map_builder.hpp`;
  - `orbslam3_multi/src/global_map_builder.cpp`;
  - `orbslam3_server/src/global_map_server.cpp`;
  - `codex/pipeline/fase_3_sparse_global/subfases/subfase_3K.md`;
  - `codex/pipeline/fase_3_sparse_global/subfases/subfase_3L.md`;
  - documentación de paquetes y resúmenes de estado.
- cambio implementado:
  - `GlobalMapBuilder` decide por MapPoint si toca un KF corregido por
    servidor mediante `GetKeyFrameServerCorrection`;
  - si toca un KF corregido y este es publicable, proyecta el punto desde la
    pose final de ese KF;
  - si toca KFs corregidos pero ninguno es publicable, lo salta y cuenta
    `server_corrected_missing_keyframe_skipped` e `invalid_pose_skipped`;
  - si no toca ningún KF corregido, conserva el fallback legacy
    `world_T_local * p_local_raw`;
  - `GetSubmapLastServerCorrection` queda solo para herencia de KFs futuros y
    ya no prohíbe el fallback de todo el submapa.
- paquetes compilados:
  - `orbslam3_multi`;
  - `orbslam3_server`.
- resultado de build:
  - `BUILD-EXIT-CODE 0` antes y después del ajuste fino de selección de KFs
    corregidos publicables.
- pruebas Gazebo:
  - `prueba_29`: validación positiva previa al último ajuste fino;
  - `prueba_30`: no concluyente porque el grafo grande se creó tarde y no llegó
    al apply antes del cleanup;
  - `prueba_31`: validación final con el código exacto actual.
- comando base de validación:

  ```bash
  ./codex/herramientas/run_simulation.sh --prueba 31 --yaml /home/chenfu/Gazebo/src/codex/archivos_auxiliares/trayectorias/prueba_tipica_fiducial_2_a_1_dos_lados.yaml --startup-wait-sec 2 --post-scenario-wait-sec 180 --timeout-sec 1600 --max-gazebo-retries 1 --launch "ros2 launch simulacion_dron multi_dron.launch.py pose_graph_vertex_selection_ratio:=0.30 pose_graph_use_covisibility_edges:=false pose_graph_fiducial_neighborhood_vertex_ratio:=0.20 loop_bow_min_mappoints:=1000000 f1l_debug_animation_enabled:=true f1l_graph_dump_enabled:=true f1l_gt_kf_debug_enabled:=true f1l_gt_kf_debug_max_dt_sec:=1.0"
  ```

- evidencia positiva de `prueba_31`:
  - `SCENARIO-RUNNER-DONE scenario='prueba_tipica_fiducial_2_a_1_dos_lados'
    success=true`;
  - `SIM-DONE prueba=31 success=true`, `SIM-EXIT-CODE 0`;
  - `task_id=1` (`drone_2`, fiducial 1): error `21.289176 m -> 0`,
    `optimized_kfs=26`, `propagated_kfs=69`, `raw_db_modified=false`;
  - `task_id=1`: `[F1L-POST-APPLY-GLOBALMAP-CHECK] ok=true`,
    `published_points_before=36644`, `published_points_after=36644`,
    `server_corrected_after=27180`, `invalid_pose_skipped_before=0`,
    `invalid_pose_skipped_after=0`;
  - `task_id=1`: `[F1L-GLOBALMAP-KF-PROJECTION]`
    `server_corrected_candidates_after=27180`,
    `server_corrected_missing_kf_after=0`;
  - `task_id=2` (`drone_1`, fiducial 1): error `0.444713 m -> 0`,
    `optimized_kfs=27`, `propagated_kfs=73`, `raw_db_modified=false`;
  - `task_id=2`: `published_points_before=37415`,
    `published_points_after=37415`, `invalid_pose_skipped 0 -> 0`,
    `server_corrected_missing_kf_after=0`;
  - ambos tasks emiten `[F1K-GLOBALMAP-PUBLISH-AFTER-APPLY]` con
    `topic=/global_sparse_cloud`, `frame_id=world`, `decision=ACCEPT`;
  - HTML 3D:
    `codex/archivos_auxiliares/html/f3l_debug_animation_task_1.html` y
    `codex/archivos_auxiliares/html/f3l_debug_animation_task_2.html`;
  - dumps reproducibles:
    `codex/archivos_auxiliares/repeticiones/f3i_window_task_1.tsv`,
    `codex/archivos_auxiliares/repeticiones/f3l_graph_task_1.tsv`,
    `codex/archivos_auxiliares/repeticiones/f3i_window_task_2.tsv` y
    `codex/archivos_auxiliares/repeticiones/f3l_graph_task_2.tsv`.
- evidencia negativa o ausente:
  - no se inspeccionó RViz2 manualmente desde Codex; la evidencia disponible es
    por logs de publish/republish de `/global_sparse_cloud` tras `ACCEPT`;
  - `gazebo ... exit code 255` aparece durante cleanup después de
    `SIM-DONE success=true`, patrón ya clasificado como no bloqueante;
  - `prueba_30` no se usa como cierre porque no dio tiempo a aplicar el grafo
    grande antes del final de la simulación.
- conclusión:
  - `CONSEGUIDA` para la deuda concreta de publicación/`invalid_pose_skipped`
    tras apply;
  - `3K` permanece `PARCIAL/REABIERTA` solo por el contrato más amplio de
    transacción privada/commit atómico descrito en `subfase_3K.md`.
- siguiente paso recomendado:
  - volver a mirar pesos/aristas/propagación en `3I/3J` usando los dumps
    guardados, ahora que la publicación ya no rechaza por veto global de
    submapa corregido.

## 2026-07-24 01:18 — Subfase 3K — Rebase de cola tras ventana optimizada

- objetivo intentado:
  - aplicar la transformación relativa del último KF optimizado/propagado de la
    ventana a todos los KFs posteriores del mismo submapa;
  - cubrir también KFs creados mientras el solver, HTML y validación estaban en
    curso;
  - comprobarlo con la prueba típica larga de rodeo del edificio.
- archivos modificados:
  - `orbslam3_multi/include/orbslam3_multi/optimization_result.hpp`;
  - `orbslam3_multi/src/optimization_manager.cpp`;
  - `orbslam3_server/src/global_map_server.cpp`;
  - `codex/pipeline/fase_3_sparse_global/subfases/subfase_3K.md`;
  - documentación de paquetes y resúmenes de estado.
- cambio implementado:
  - `ApplyCandidateResult()` localiza el último KF realmente movido por la
    ventana y calcula `delta_tail = T_last_after * inverse(T_last_before)`;
  - recorre `RawMapDatabase` para rebasar KFs posteriores del mismo submapa con
    `T_kf_after = delta_tail * T_kf_before`;
  - si un KF posterior existe en raw pero todavía no tiene pose world, primero
    se inicializa con `RegisterNewKeyFrameIfAnchored`;
  - los KFs rebasados se guardan como propagados por servidor y se cuentan en
    `tail_rebased_kfs`;
  - el backup previo al apply incluye esa cola para que un rollback de `3L`
    también la restaure;
  - la corrección heredable futura sigue tomando como referencia el último KF de
    la ventana, no el último KF de cola.
- paquetes compilados:
  - `orbslam3_multi`;
  - `orbslam3_server`.
- resultado de build:
  - `BUILD-EXIT-CODE 0`.
- pruebas Gazebo:
  - `prueba_34`: `success=true`, detectó el caso intermedio
    `right_tail_world_pose_missing` en dos tasks y confirmó un apply posterior
    con `tail_rebased_kfs=62`;
  - `prueba_35`: repetición tras registrar poses world faltantes de la cola,
    `SCENARIO-RUNNER-DONE success=true`, `SIM-DONE success=true`,
    `SIM-EXIT-CODE 0`.
- comando base de validación:

  ```bash
  ./codex/herramientas/run_simulation.sh --prueba 35 --yaml /home/chenfu/Gazebo/src/codex/archivos_auxiliares/trayectorias/prueba_tipica_rodeo_edificio_dos_fiduciales.yaml --startup-wait-sec 2 --post-scenario-wait-sec 180 --timeout-sec 2200 --max-gazebo-retries 1 --launch "ros2 launch simulacion_dron multi_dron.launch.py pose_graph_vertex_selection_ratio:=0.30 pose_graph_use_covisibility_edges:=false pose_graph_fiducial_neighborhood_vertex_ratio:=0.20 loop_bow_min_mappoints:=1000000 f1l_debug_animation_enabled:=true f1l_graph_dump_enabled:=true f1l_gt_kf_debug_enabled:=true f1l_gt_kf_debug_max_dt_sec:=1.0 rawdb_record_enabled:=false f1g_debug_mark_optimized_kf:=false pose_store_debug_opt_enabled:=false"
  ```

- evidencia positiva de `prueba_35`:
  - `task_id=75`: fiducial 1, `error_t=24.554201 m -> 0`,
    `optimized_kfs=32`, `propagated_kfs=83`, `tail_rebased_kfs=2`,
    `invalid_pose_skipped 0 -> 0`;
  - `task_id=76`: fiducial 1, `error_t=3.079297 m -> 0`,
    `optimized_kfs=44`, `propagated_kfs=126`, `tail_rebased_kfs=13`,
    `invalid_pose_skipped 0 -> 0`;
  - `task_id=77`: vuelta a fiducial 2, `error_t=2.588243 m -> 0`,
    `optimized_kfs=26`, `propagated_kfs=67`, `tail_rebased_kfs=0`,
    `invalid_pose_skipped 0 -> 0`;
  - no aparecen `right_tail_world_pose_missing`, `global_map_check_failed`,
    `F1L-POST-APPLY-REJECT` ni rollback;
  - `SERVER_OPTIMIZATION_RIGHT_TAIL_REBASE` aparece en `39` líneas de log;
  - estadísticas finales de nube:
    `points=59910`, `server_corrected_points=38906`,
    `server_corrected_missing_keyframe_skipped=0`,
    `invalid_pose_skipped=0`;
  - HTML 3D:
    `codex/archivos_auxiliares/html/f3l_debug_animation_task_75.html`,
    `codex/archivos_auxiliares/html/f3l_debug_animation_task_76.html` y
    `codex/archivos_auxiliares/html/f3l_debug_animation_task_77.html`;
  - dumps reproducibles:
    `codex/archivos_auxiliares/repeticiones/f3l_graph_task_75.tsv`,
    `codex/archivos_auxiliares/repeticiones/f3l_graph_task_76.tsv` y
    `codex/archivos_auxiliares/repeticiones/f3l_graph_task_77.tsv`.
- evidencia negativa o ausente:
  - no se inspeccionó RViz2 manualmente desde Codex; la señal disponible es por
    logs, contadores de publicación y ausencia de skips/rollback;
  - quedan warnings no bloqueantes de C++ por funciones/parámetros no usados en
    código histórico de optimización.
- conclusión:
  - `CONSEGUIDA` para la regla concreta de rebase de KFs posteriores ya
    existentes y KFs creados durante la optimización;
  - `3K` permanece `PARCIAL/REABIERTA` solo por la deuda arquitectónica más
    amplia de candidato privado/commit atómico.
- siguiente paso recomendado:
  - inspeccionar visualmente RViz2/HTML de `task_id=75..77`; si se ve coherente,
    puede darse por cerrada provisionalmente la optimización por error de
    fiducial antes de pasar a covisibilidad/loops.

## 2026-07-24 02:10 — Subfase 3K — decisión `active_tail_anchor`

- objetivo intentado:
  - documentar el problema observado tras las pruebas largas: el HTML puede
    mostrar una optimización fiducial correcta mientras RViz2 conserva KFs en
    posiciones incoherentes;
  - definir cómo debe convivir `GlobalPoseStore` con poses raw mutables de
    ORB-SLAM3.
- archivos modificados:
  - documentación de subfases `3D`, `3E`, `3I` y `3K`;
  - documentación de paquetes `orbslam3_multi` y `orbslam3_server`;
  - resúmenes de contexto, pipeline, historial e última sesión.
- paquetes compilados:
  - ninguno.
- resultado de build:
  - no ejecutado; sesión solo documental.
- pruebas Gazebo/replay:
  - no ejecutadas.
- patrones de reducción:
  - no aplica.
- evidencia positiva:
  - queda fijada la regla de autoridad: `RawMapDatabase` es raw mutable y
    `GlobalPoseStore` conserva poses world aceptadas por el servidor;
  - `submap_last_correction`/`delta_tail` quedan como compatibilidad temporal,
    no como diseño vigente para KFs futuros;
  - KFs nuevos o de cola deben calcularse con:

  ```text
  T_world_new = T_world_ref_accepted * inverse(T_raw_ref_current) * T_raw_new_current
  ```

- evidencia negativa o ausente:
  - no hay implementación todavía;
  - falta repetir la prueba aislada antihoraria después del cambio.
- conclusión:
  - `PARCIAL`: contrato documental actualizado; implementación pendiente.
- siguiente paso recomendado:
  - implementar `accepted_keyframe_anchors`/`active_tail_anchor` en
    `GlobalPoseStore`, `RegisterNewKeyFrameIfAnchored`,
    `ReconcileAfterRawIngestResult`, `OptimizationManager::ApplyCandidateResult`
    y backup/rollback;
  - validar primero un solo dron antihorario
    `fiducial 2 -> fiducial 1 -> fiducial 2`.

## 2026-07-24 17:30 — Subfase 3K — autoridad accepted/derived implementada

- objetivo intentado:
  - impedir que cambios raw de ORB-SLAM3 muevan ventanas ya aceptadas;
  - reproyectar KFs posteriores desde el último ancla aceptada;
  - evitar una optimización nueva dentro de la misma visita fiducial.
- archivos modificados:
  - `GlobalPoseStore`, `OptimizationManager`, `FiducialAnchorManager`;
  - coordinación/logs en `global_map_server`;
  - test local `test_global_pose_store_tail_anchor`;
  - documentación de contexto, paquetes y subfase `3K`.
- paquetes compilados:
  - `orbslam3_multi`, `orbslam3_server`.
- resultado de build:
  - `BUILD-EXIT-CODE 0`; solo warnings previos de símbolos/parámetros no usados.
- pruebas:
  - test local de pose aceptada, cola derivada, cambio raw y rollback: exit `0`;
  - offline `task 48`: target `16.7376 -> 0 m`, GT medio
    `6.8336 -> 1.2605 m`, `worsened_kfs=0`;
  - offline `task 49`: target `3.0183 -> 0 m`, GT medio
    `1.1462 -> 0.3401 m`;
  - `prueba_38` y `prueba_39`: escenario completado, pero no válidas como
    prueba integral porque ORB-SLAM3 creó otro `map_epoch` antes del fiducial 1.
- evidencia positiva:
  - `accepted_keyframe_anchors` no cambia su pose world ante cambios raw;
  - `derived_tail_keyframes` se reproyecta desde `active_tail_anchor`;
  - backup/rollback conserva ambos estados;
  - `invalid_pose_skipped=0` y
    `server_corrected_missing_keyframe_skipped=0` en las publicaciones finales
    observadas;
  - HTML 3D nuevos:
    `f3l_offline_graph_task_48_active_tail_3d.html` y
    `f3l_offline_graph_task_49_active_tail_3d.html`.
- evidencia negativa o ausente:
  - no hubo apply live dentro de una sola subnube en `prueba_38/39`;
  - RViz2 no puede considerarse validado;
  - la estabilidad de tracking/epoch de la trayectoria larga es el bloqueo
    inmediato.
- conclusión:
  - `PARCIAL`: implementación y validación local/offline correctas; cierre live
    pendiente.
- siguiente paso recomendado:
  - conseguir el rodeo en un único `map_epoch` o reproducir integralmente una
    ventana guardada con `GlobalPoseStore`/apply/publicación;
  - comprobar dos applies, ausencia de tercera tarea de la misma visita y
    continuidad visual en RViz2.

## 2026-07-27 — Autoridad de poses revalidada con loop interno desactivado

- `prueba_41` completa fiducial 2 -> 1 -> 2 en un único `map_epoch` y aplica
  dos optimizaciones del servidor.
- No hay loops ni merges internos de ORB-SLAM3. Los cambios raw que permanecen
  son compatibles con Local BA y no movieron poses world `accepted` ni
  `server_optimized`.
- La cola `derived_tail` se reproyectó desde `active_tail_anchor`; el mayor
  cambio world derivado fue `0.089053 m`.
- Ambos applies registraron `raw_db_modified=false`; la publicación terminó
  con `invalid_pose_skipped=0` y
  `server_corrected_missing_keyframe_skipped=0`.
- La segunda optimización del servidor empeoró la media GT de su ventana
  `0.218745 -> 1.358109 m`; esta métrica debug queda anotada para `3I/3J`.
- El 2026-07-28 el usuario confirmó en RViz2 que ambas optimizaciones, la cola
  posterior y los MapPoints se ven perfectamente, sin KFs que recuperen poses
  raw antiguas.
- Con esa evidencia, autoridad, apply y publicación de `3K` quedan
  `CONSEGUIDOS`.
