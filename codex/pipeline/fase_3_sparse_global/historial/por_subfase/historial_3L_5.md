# Historial 3L.5

## 2026-07-27 — Loop interno desactivado y validación live `prueba_41`

- objetivo:
  - impedir que ORB-SLAM3 aplique loop closing/merge sobre poses históricas;
  - repetir con un solo dron el recorrido antihorario fiducial 2 -> 1 -> 2;
  - comprobar dos optimizaciones del servidor, publicación y autoridad
    `accepted`/`derived_tail`.
- cambios:
  - `loopClosing: 0` en las configuraciones mono/stereo de `dron_individual` y
    `simulacion_dron`;
  - ruta inactiva de `LoopClosing::InsertKeyFrame()` sin cola: los KFs se
    indexan directamente en `KeyFrameDatabase`;
  - marcador runtime
    `ORB-SLAM3-LOOP-CLOSING-CONFIG active=false policy=index_without_loop_queue`.
- build:
  - `cmake --build ORB_SLAM3/build -j4`: correcto;
  - `orbslam3_multi`, `orbslam3_server`, `dron_individual` y
    `simulacion_dron`: `BUILD-EXIT-CODE 0`.
- prueba:
  - YAML:
    `prueba_rodeo_antihorario_un_dron_fid2_fid1_fid2.yaml`;
  - `SCENARIO-RUNNER-DONE success=true`;
  - `SIM-DONE prueba=41 success=true`;
  - `SIM-EXIT-CODE 0`;
  - tracking continuo, un único `map_epoch` por dron;
  - el `gazebo exit code 255` posterior a `SIM-DONE` pertenece al cleanup.
- ORB-SLAM3:
  - dos frontends con `active=false` y
    `policy=index_without_loop_queue`;
  - `*Loop detected=0`, `*Merge detected=0`;
  - ninguna parada de `LocalMapping` causada por loop;
  - covisibilidad final preservada:
    `confirmed_edges=5552`, `orbslam3_native=5552`;
  - ORB-SLAM3 todavía puede cambiar poses raw mediante su optimización local.
    En un snapshot se observaron `raw_pose_changed=40`, compatible con Local BA
    y sin evidencia de loop, merge, reset o pérdida de tracking.
- autoridad global:
  - los KFs `accepted` y `server_optimized` conservaron su pose world frente a
    cambios raw;
  - la reconciliación registró `KEEP-ACCEPTED` y
    `KEEP-SERVER-OPTIMIZED`, sin rebase de esas poses;
  - los KFs `derived_tail` se reproyectaron desde `active_tail_anchor`, como
    exige el diseño, con cambio world máximo `0.089053 m`;
  - ambos applies informaron `raw_db_modified=false`;
  - publicación final: `invalid_pose_skipped=0`,
    `server_corrected_missing_keyframe_skipped=0`, `points=37259`.
- task 1, llegada a fiducial 1:
  - target: `16.740643 -> 0 m`;
  - coste: `24165071.121071 -> 84709.167286`;
  - GT ventana: media `5.824808 -> 0.910463 m`, máximo
    `16.740643 -> 2.078310 m`, `worsened_kfs=43`;
  - apply: `optimized=27`, `propagated=70`, sin skips.
- task 2, regreso a fiducial 2:
  - target: `2.392760 -> 0 m`;
  - coste: `575219.885877 -> 28646.322430`;
  - GT ventana: media `0.218745 -> 1.358109 m`, máximo
    `2.392760 -> 3.080204 m`, `worsened_kfs=74`;
  - apply: `optimized=33`, `propagated=85`, sin skips;
  - el replay offline reproduce la misma degradación, por lo que no es un
    problema exclusivo de publicación o RViz2.
- artefactos:
  - `logs/prueba_41.log`, reducido e índice de sublogs;
  - `html/f3l_debug_animation_task_1.html`;
  - `html/f3l_debug_animation_task_2.html`;
  - dumps `f3i_window_task_1/2.tsv` y `f3l_graph_task_1/2.tsv`.
- conclusión:
  - desactivación del loop interno: `CONSEGUIDA`;
  - autoridad de poses aceptadas y publicación: validada por logs;
  - conclusión inicial automática: `PARCIAL`, porque la métrica GT debug de
    task 2 empeora aunque el target se corrige;
  - validación visual del usuario del 2026-07-28: ambos applies y el mapa
    completo se ven perfectamente en RViz2; no hay KFs ni MapPoints fuera de
    sitio y no reaparece la geometría previa a la primera optimización;
  - cierre de aplicación/publicación `3K` y diagnóstico `3L`: `CONSEGUIDOS`.
- siguiente paso:
  - conservar la discrepancia de GT de task 2 para una revisión futura de
    pesos/priors y criterio de aceptación en `3I/3J`;
  - no modificar ahora el solver: HTML, logs y RViz2 confirman un resultado
    operativo correcto.
