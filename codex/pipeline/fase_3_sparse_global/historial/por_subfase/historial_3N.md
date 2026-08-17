# Historial subfase 3N

## 2026-07-17 — Infraestructura provisional sin contrato BoW

- objetivo intentado:
  - adelantar el despacho de KFs nuevos y la frontera de `LoopDetector` sin
    inventar una comparación BoW que el checkout no puede verificar.
- archivos modificados:
  - `RawMapDatabase`, `LoopCandidate`, `LoopDetector` y `global_map_server`;
  - documentación de `3N` y de los paquetes afectados.
- cambios realizados:
  - `RawInsertResult` entrega `new_keyframe_ids`;
  - servidor despacha nuevos KFs en live, snapshots y replay;
  - detector devuelve `bow_data_unavailable_in_current_checkout` hasta disponer
    del contrato BoW/`FeatureVector` real;
  - marcadores normalizados de `F1M-LOOP-*` a `F1N-LOOP-*`.
- paquetes compilados:
  - no ejecutado: faltan `orbslam3_multi/CMakeLists.txt` y `orbslam3_msgs`.
- pruebas Gazebo/replay:
  - pendientes en VS Code/workspace completo.
- conclusión:
  - `3N` queda **por implementar**; no se producen candidatos ficticios ni se
    confirma ningún loop sin BoW/geometría.
- siguiente paso recomendado:
  - conectar los campos BoW reales de `OrbKeyFrame`, indexar/rankear y consultar
    `CovisibilityDatabase` antes de enviar candidatos a `3O`.

## 2026-07-20 21:07 — Subfase 3N — BoW real validado parcialmente

- objetivo intentado:
  - implementar `LoopDetector` con candidatos BoW reales, búsqueda amplia,
    filtros iniciales y consulta obligatoria a `CovisibilityDatabase` antes de
    devolver candidatos a `3O`.
- archivos modificados:
  - `orbslam3_multi/include/orbslam3_multi/loop_candidate.hpp`;
  - `orbslam3_multi/include/orbslam3_multi/loop_detector.hpp`;
  - `orbslam3_multi/src/loop_detector.cpp`;
  - `orbslam3_server/src/global_map_server.cpp`;
  - `codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml`;
  - `codex/archivos_auxiliares/trayectorias/tray_prueba_2.yaml`;
  - `codex/archivos_auxiliares/trayectorias/tray_prueba_3.yaml`;
  - documentación de paquetes, estado, subfase e historial.
- cambios realizados:
  - `LoopDetector` construye vectores BoW desde `bow_word_ids` y
    `bow_word_values`, calcula similitud coseno esparsa y recorre KFs de todos
    los submapas de `RawMapDatabase`;
  - aplica filtros `query_no_bow`, `no_bow`, `query_bad_or_incomplete`,
    `bad_or_incomplete`, `too_recent_same_submap`, `low_bow_score`,
    `confirmed_covisibility`, `max_candidates` y `max_candidates_per_submap`;
  - añade campos diagnósticos a `LoopCandidate` y `LoopCandidateResult`;
  - `global_map_server` declara/configura `loop_bow_*`, despacha KFs nuevos y
    loggea búsqueda, filtros, candidatos y skips `[F1N-BOW-SKIP-CONFIRMED-COVIS]`;
  - la ruta solo genera candidatos: no subnubes, no RANSAC, no fusión, no
    optimización y no GT para loops.
- paquetes compilados:
  - `orbslam3_multi`;
  - `orbslam3_server`;
  - `simulacion_dron`.
- resultado de build:
  - `BUILD-EXIT-CODE 0`;
  - `orbslam3_multi` terminado en `3.53s`;
  - `simulacion_dron` terminado en `22.6s`;
  - `orbslam3_server` terminado en `19.4s`;
  - no hizo falta reducir log de build.
- pruebas Gazebo/replay:
  - `prueba_1` live con `ros2 launch simulacion_dron multi_dron.launch.py` y
    `--post-scenario-wait-sec 20`: `SCENARIO-RUNNER-DONE success=true`,
    `SIM-DONE prueba=1 success=true`, `SIM-EXIT-CODE 0`;
  - `prueba_2` replay con `global_orb_map_server.launch.py
    rawdb_replay_enabled:=true rawdb_record_enabled:=false
    rawdb_replay_period_sec:=0.05 f1g_full_snapshot_enabled:=false
    f1j_dryrun_enabled:=false f1k_apply_enabled:=false
    f1l_validation_enabled:=false`: `SIM-EXIT-CODE 0` y
    `[F1C-REPLAY-DONE] entries=177 journal=177 deltas=164 full=13
    fiducial_observations=37 submaps=4 kfs=470 mps=39300`;
  - `prueba_3` replay equivalente para filtros/diversidad: `SIM-EXIT-CODE 0`.
- patrones de reducción:
  - prueba 1:
    `SCENARIO-RUNNER|GOAL|RESULT|success|F1N-|F1C-RAWDB|F1F-GLOBALMAP|ERROR|FATAL|Segmentation fault|Killed`;
  - prueba 2:
    `SCENARIO-RUNNER|GOAL|RESULT|success|F1N-|F1C-REPLAY|F1C-RAWDB|ERROR|FATAL|Segmentation fault|Killed`;
  - prueba 3:
    `SCENARIO-RUNNER|GOAL|RESULT|success|F1N-LOOP-CANDIDATE-FILTER|F1N-LOOP-CANDIDATE-SUMMARY|too_recent_same_submap|ERROR|FATAL|Segmentation fault|Killed`.
- evidencia positiva:
  - prueba 1 contiene `476` despachos/queries/búsquedas/summaries, `2824`
    candidatos, `27388` eventos de filtro, `308` skips por covisibilidad
    confirmada, `475` logs `[F1N-LOOP-BOW-CANDIDATES]` y un
    `[F1N-LOOP-NO-CANDIDATES]`;
  - prueba 2 contiene `470` queries/búsquedas/summaries, `2778` candidatos,
    `27004` filtros, `308` skips por covisibilidad confirmada y candidatos con
    `candidate_has_world_pose=true`/`candidate_is_anchored=true`;
  - prueba 3 repite la evidencia de replay y muestra filtros
    `too_recent_same_submap`, `max_candidates_per_submap` y
    `confirmed_covisibility`;
  - no aparecen marcadores de loop confirmado, subnubes, RANSAC, fusión ni tarea
    de optimización por loop producidos por la ruta `3N`.
- evidencia negativa o ausente:
  - en `prueba_1` el proceso ORB-SLAM3 `stereo-8` de `dron_1` muere con
    `exit code -6` antes del cierre del escenario;
  - los logs contienen errores/rechazos `F1I-GRAPH-REJECT` y
    `F1I-GRAPH-BUILD-SUMMARY ... reason=bad_window_coverage`, deuda ajena de
    la ruta fiducial/3L;
  - por el criterio de `3N`, estos errores impiden marcar la subfase como
    `CONSEGUIDA` aunque build, replay y ruta BoW funcionen.
- conclusión:
  - `PARCIAL`.
- siguiente paso recomendado:
  - aislar/reparar el aborto live `stereo-8` y repetir `prueba_1`; además,
    aislar o desactivar la ruta fiducial `F1I/F1L` en pruebas de `3N` para que
    los logs de cierre no arrastren errores ajenos.

## 2026-07-21 12:22 — Subfase 3N — recuperación ORB_SLAM3 y revalidación live

- objetivo intentado:
  - recuperar `ORB_SLAM3` desde los commits antiguos sin parches improvisados y
    comprobar si eso elimina el `stereo-8 exit code -6` observado en `prueba_1`.
- archivos modificados:
  - `ORB_SLAM3/CMakeLists.txt`;
  - `ORB_SLAM3/include/System.h`;
  - `ORB_SLAM3/src/System.cc`;
  - documentación compacta de `ORB_SLAM3`, estado, última sesión e índice.
- diagnóstico de historial:
  - el repo padre solo tuvo gitlink temprano para `ORB_SLAM3` en los commits
    `47b5b8e` y `279c6a2`, ambos apuntando a
    `4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4`;
  - el checkout interno de `ORB_SLAM3` ya estaba en ese upstream, con cambios
    locales únicamente en `CMakeLists.txt`, `include/System.h` y `src/System.cc`.
- cambios realizados:
  - se restauró primero el upstream puro y se comprobó que C++11 no compila con
    el `sigslot` actual de Pangolin (`std::decay_t`/`std::enable_if_t` requieren
    C++14);
  - se dejó solo el ajuste de C++14 en `CMakeLists.txt`;
  - se reañadieron `System::GetAllKeyFrames()` y `System::GetAllMapPoints()`
    como passthroughs a `Atlas`, porque el ejecutable ROS instalado `stereo`
    fallaba con `undefined symbol` al eliminarlos.
- paquetes compilados:
  - build interno `cmake --build ORB_SLAM3/build -j4`;
  - `orbslam3_multi`;
  - `orbslam3_server`;
  - `simulacion_dron`.
- resultado de build:
  - `cmake --build ORB_SLAM3/build -j4`: código 0;
  - `nm -D ORB_SLAM3/lib/libORB_SLAM3.so` exporta
    `ORB_SLAM3::System::GetAllKeyFrames()` y
    `ORB_SLAM3::System::GetAllMapPoints()`;
  - `install/orbslam3/lib/orbslam3/stereo` carga la `.so` y muestra el `Usage`
    esperado sin `undefined symbol`;
  - `build_selected_packages.sh orbslam3_multi orbslam3_server simulacion_dron`:
    `BUILD-EXIT-CODE 0`;
  - intento no válido: `build_selected_packages.sh ORB_SLAM3` lanzó colcon con
    `-j16` y falló por OOM/kill de `cc1plus` en `FrameDrawer.cc.o` y
    `Tracking.cc.o`; no fue error de fuente.
- pruebas Gazebo/replay:
  - `prueba_1` live con `ros2 launch simulacion_dron multi_dron.launch.py`.
- patrones de reducción:
  - `SCENARIO-RUNNER|SIM-DONE|SIM-EXIT-CODE|process has died|exit code -6|SO3::exp failed|PIPE0-WRAPPER-TRACK|F1M-COVIS-SUMMARY|F1N-LOOP-CANDIDATE-SUMMARY|F1N-LOOP-CANDIDATE|F1N-LOOP-KF-QUERY|F1N-LOOP-BOW-CANDIDATES|undefined symbol|Killed|fatal error`.
- evidencia positiva:
  - `SCENARIO-RUNNER-DONE success=true`, `SIM-DONE prueba=1 success=true`,
    `SIM-EXIT-CODE 0`;
  - `10` goals del runner con `success=true`;
  - `F1M-COVIS-SUMMARY` final con `confirmed_edges=6776`,
    `orbslam3_native=6776`;
  - `2127` candidatos `[F1N-LOOP-CANDIDATE]`, con `1151` inter-dron
    (`same_drone=false`) y `976` intra-dron (`same_drone=true`).
- evidencia negativa o ausente:
  - `prueba_1` reproduce exactamente el fallo grave:
    `SO3::exp failed! omega: -nan -nan -nan` y `stereo-8` muere con
    `exit code -6`;
  - la recuperación del gitlink/upstream y de los símbolos del wrapper no corrige
    el abort de Sophus; el fallo queda aislado como degeneración interna de
    ORB-SLAM3 durante tracking/relocalización.
- conclusión:
  - `PARCIAL`.
- siguiente paso recomendado:
  - no seguir restaurando a ciegas `ORB_SLAM3`; diagnosticar el punto que genera
    NaN antes de `Sophus::SO3f::exp`, especialmente `Sim3Solver.cc`, y aplicar
    una guarda mínima justificada si se confirma la degeneración.

## 2026-07-21 12:49 — Subfase 3N — guarda Sim3Solver y revalidación live

- objetivo intentado:
  - eliminar el abort live `stereo-8 exit code -6` sin editar ORB_SLAM3 a ciegas,
    documentando la incidencia como punto importante de mantenimiento.
- archivos modificados:
  - `ORB_SLAM3/include/Sim3Solver.h`;
  - `ORB_SLAM3/src/Sim3Solver.cc`;
  - `codex/contexto/paquetes/ORB_SLAM3/00_summary.md`;
  - `codex/contexto/paquetes/ORB_SLAM3/sim3_solver_guard.md`;
  - documentación de estado, pipeline e historial de `3N`.
- cambios realizados:
  - `Sim3Solver::ComputeSim3(...)` pasa de `void` a `bool`;
  - los bucles RANSAC descartan la muestra si `ComputeSim3(...)` devuelve
    `false`;
  - se añadieron guardas `allFinite()`/`std::isfinite()` en entradas,
    centroides, matrices, eigensolver, autovalores/autovectores, vector de
    rotación, escala, traslación y transformaciones finales;
  - si la parte imaginaria del cuaternión tiene norma casi cero, se usa vector
    de rotación cero para preservar la rotación identidad válida;
  - no se modificó Sophus.
- paquetes compilados:
  - build interno `cmake --build ORB_SLAM3/build -j4`;
  - `orbslam3_multi`;
  - `orbslam3_server`;
  - `simulacion_dron`.
- resultado de build:
  - `cmake --build ORB_SLAM3/build -j4`: código 0;
  - `install/orbslam3/lib/orbslam3/stereo` carga la `.so` y muestra el `Usage`
    esperado sin `undefined symbol`;
  - `build_selected_packages.sh orbslam3_multi orbslam3_server simulacion_dron`:
    `BUILD-EXIT-CODE 0`.
- pruebas Gazebo/replay:
  - `prueba_1` live con `ros2 launch simulacion_dron multi_dron.launch.py`;
  - se conserva como evidencia acumulada de `3N` la validación replay
    `prueba_2`/`prueba_3` del 2026-07-20, porque el cambio afecta al runtime
    live de ORB_SLAM3 y no a la ruta replay BoW.
- patrones de reducción:
  - `SCENARIO-RUNNER|SIM-DONE|SIM-EXIT-CODE|process has died|exit code -6|SO3::exp failed|PIPE0-WRAPPER-TRACK|F1M-COVIS-SUMMARY|F1N-LOOP-CANDIDATE-SUMMARY|F1N-LOOP-CANDIDATE|F1N-LOOP-KF-QUERY|F1N-LOOP-BOW-CANDIDATES|undefined symbol|Killed|fatal error`.
- evidencia positiva:
  - `SCENARIO-RUNNER-DONE success=true`;
  - `10` goals del runner con `success=true`;
  - `SIM-DONE prueba=1 success=true`;
  - `SIM-EXIT-CODE 0`;
  - `SO3::exp failed`: `0` apariciones;
  - `exit code -6`: `0` apariciones;
  - `process has died`: `0` apariciones;
  - `undefined symbol`: `0` apariciones;
  - `F1M-COVIS-SUMMARY` final con `confirmed_edges=6820`,
    `orbslam3_native=6820`;
  - `1947` candidatos `[F1N-LOOP-CANDIDATE]`, con `1056` inter-dron
    (`same_drone=false`) y `891` intra-dron (`same_drone=true`);
  - `188` skips `[F1N-BOW-SKIP-CONFIRMED-COVIS]`.
- evidencia negativa o ausente:
  - no se repitieron `prueba_2` y `prueba_3` después de la guarda porque ya
    estaban validadas en replay y no ejecutan ORB_SLAM3 live; si se desea una
    regresión completa antes de `3O`, repetirlas es barato frente a Gazebo live;
  - `3L` sigue `PARCIAL` como deuda fiducial/post-apply independiente.
- conclusión:
  - `CONSEGUIDA`.
- siguiente paso recomendado:
  - empezar `3O` construyendo verificación geométrica/subnubes/RANSAC sobre los
    candidatos BoW de `3N`.

## 2026-07-28 — Revalidación integrada pendiente

- estado histórico:
  `3N` permanece `CONSEGUIDA`; el detector BoW y la guarda de `Sim3Solver`
  conservan su evidencia anterior.
- prerrequisito:
  no iniciar esta revalidación hasta que la regresión de `3M` pase.
- comprobación mínima:
  repetir las pruebas live/replay del contrato de `3N`, confirmar candidatos
  BoW intra e inter-dron, skips por covisibilidad confirmada y ausencia de
  `exit code -6`, `SO3::exp failed` o muerte de procesos.
- siguiente paso:
  si pasa, continuar y revalidar `3O` con el estado actual.

## 2026-07-28 16:18 — Subfase 3N — regresión integrada live y replay real

- objetivo intentado:
  - comprobar `LoopDetector` después del cierre de `3I-3L` y tras la regresión
    positiva de `3M`;
  - confirmar candidatos BoW, filtros, ranking, skips por covisibilidad y
    ausencia de aborts de ORB-SLAM3.
- archivos modificados:
  - ninguno de código; solo documentación de historial/estado al cierre.
- paquetes compilados:
  - `orbslam3_multi`;
  - `orbslam3_server`;
  - `simulacion_dron`.
- resultado de build:
  - `./codex/herramientas/build_selected_packages.sh orbslam3_multi orbslam3_server simulacion_dron`
    termina con `BUILD-EXIT-CODE 0`.
- pruebas Gazebo/replay:
  - `prueba_1` live con `ros2 launch simulacion_dron multi_dron.launch.py`;
  - `prueba_2` replay real con
    `ros2 launch orbslam3_server global_orb_map_server.launch.py n_drones:=2 use_sim_time:=false rawdb_replay_enabled:=true rawdb_record_enabled:=false rawdb_replay_period_sec:=0.02 fiducial_sim_enabled:=false f1g_full_snapshot_enabled:=false`.
- patrones de reducción:
  - `SCENARIO-RUNNER|GOAL|RESULT|success|F1N-|F1C-REPLAY|F1C-RAWDB|ERROR|FATAL|Segmentation fault|Killed|process has died|exit code -6|SO3::exp failed|undefined symbol`.
- evidencia positiva live:
  - `SCENARIO-RUNNER-DONE success=true`, `SIM-DONE prueba=1 success=true`,
    `SIM-EXIT-CODE 0`;
  - `F1N-LOOP-NEW-KF-DISPATCH`: `534`;
  - `F1N-LOOP-BOW-SEARCH`: `534`;
  - `F1N-LOOP-CANDIDATE-SUMMARY`: `534`;
  - `F1N-LOOP-BOW-CANDIDATES`: `532`;
  - `F1N-LOOP-CANDIDATE`: `3773`;
  - `F1N-LOOP-CANDIDATE-FILTER`: `30316`;
  - `F1N-LOOP-CANDIDATE-RANK`: `3773`;
  - `F1N-BOW-SKIP-CONFIRMED-COVIS`: `290`;
  - ejemplos finales comparan `indexed_kfs=426`, `compared_kfs=425` y cientos
    de `raw_candidates`.
- evidencia positiva replay:
  - `F1C-REPLAY-LOAD success=true entries=156 deltas=138 full=18 submaps=6 kfs=533 mps=41219 fiducial_observations=24`;
  - se reinyectan `148` entradas antes de cerrar el YAML de espera;
  - `SCENARIO-RUNNER-DONE success=true`, `SIM-DONE prueba=2 success=true`,
    `SIM-EXIT-CODE 0`;
  - `F1N-LOOP-NEW-KF-DISPATCH`: `533`;
  - `F1N-LOOP-BOW-SEARCH`: `533`;
  - `F1N-LOOP-CANDIDATE-SUMMARY`: `533`;
  - `F1N-LOOP-BOW-CANDIDATES`: `532`;
  - `F1N-BOW-SKIP-CONFIRMED-COVIS`: `290`;
  - `F1N-LOOP-CANDIDATE-FILTER`: `30316`;
  - `F1N-LOOP-CANDIDATE-RANK`: `3773`;
  - ejemplos finales comparan `indexed_kfs=533`, `compared_kfs=532` y devuelven
    `filtered_candidates=10`.
- evidencia negativa o ausente:
  - no aparece `F1C-REPLAY-DONE`, porque el YAML de `prueba_2` finaliza tras
    150 s y el replay directo alcanzó `148/156` entradas. Para una revalidación
    completa de `3O` conviene ampliar la espera o subir el ritmo solo si el
    procesamiento lo permite.
  - en live hay `process has died` de Gazebo en retry inicial y cleanup, pero no
    durante el resultado funcional; en replay no hay errores reales.
  - no reaparecen `SO3::exp failed`, `exit code -6`, `undefined symbol`,
    `Segmentation fault`, `LOOP_CONFIRMED`, `LOOP_OPT_TASK_CREATED`,
    `FUSION_EVENT`, `RAWDB-POSE-OVERWRITE-BY-OPT` ni `raw_db_modified=true`.
  - los logs `F1N-SUBCLOUD-ERROR` pertenecen al consumidor `3O` ya activo, no a
    un fallo de `LoopDetector`.
- conclusión:
  - `CONSEGUIDA`: regresión integrada de `3N` superada en live y replay real
    con cobertura suficiente.
- siguiente paso recomendado:
  - continuar con la revalidación de `3O`; si se quiere exigir cierre completo
    del journal, ajustar la espera de `prueba_2`.

## 2026-07-28 20:58 — Subfase 3N — candidatos cercanos no vetados por gap

- objetivo intentado:
  - corregir el contrato de `LoopDetector` para que los candidatos BoW cercanos
    del mismo submapa no sean descartados solo por cercanía;
  - ejecutar `prueba_tipica_anclaje_diferencial.yaml`, donde se esperan muchos
    candidatos inter-dron y actividad de covisibilidad.
- archivos modificados:
  - `orbslam3_multi/include/orbslam3_multi/loop_candidate.hpp`;
  - `orbslam3_multi/include/orbslam3_multi/loop_detector.hpp`;
  - `orbslam3_multi/src/loop_detector.cpp`;
  - `orbslam3_server/src/global_map_server.cpp`;
  - documentación de paquete, contrato e historial de `3N`.
- cambios realizados:
  - `LoopDetector` consulta `CovisibilityDatabase::HasConfirmedEdge` antes de
    aplicar la etiqueta de cercanía;
  - si el par ya está confirmado, conserva el skip
    `[F1N-BOW-SKIP-CONFIRMED-COVIS]`;
  - si el par no está confirmado y es cercano dentro del mismo submapa, ya no
    ejecuta `continue`: incrementa `near_same_submap_candidates` y devuelve el
    `LoopCandidate` normal;
  - `global_map_server` añade `near_same_submap_candidates` a
    `[F1N-LOOP-KF-QUERY]`/`[F1N-LOOP-CANDIDATE-SUMMARY]` y
    `near_same_submap=true/false` a `[F1N-LOOP-CANDIDATE]`.
- paquetes compilados:
  - `orbslam3_multi`;
  - `orbslam3_server`;
  - `simulacion_dron`.
- resultado de build:
  - `./codex/herramientas/build_selected_packages.sh orbslam3_multi orbslam3_server simulacion_dron`
    termina con `BUILD-EXIT-CODE 0`.
- pruebas Gazebo/replay:
  - intento inicial con ruta YAML relativa: `SIM-EXIT-CODE 1` por
    `scenario_runner_node` incapaz de cargar el YAML; no fue fallo funcional de
    loops;
  - repetición con ruta absoluta
    `/home/chenfu/Gazebo/src/codex/archivos_auxiliares/trayectorias/prueba_tipica_anclaje_diferencial.yaml`:
    `SCENARIO-RUNNER-DONE success=true`, `SIM-DONE prueba=45 success=true`,
    `SIM-EXIT-CODE 0`.
- patrones de reducción:
  - `SCENARIO-RUNNER|GOAL|RESULT|success|F1N-|F1M-|F1O-|SUBCLOUD|LOOP|COVIS|near_same_submap|ERROR|FATAL|Segmentation fault|Killed|process has died|exit code -6|SO3::exp failed|undefined symbol|raw_db_modified|LOOP_OPT_TASK_CREATED|FUSION_EVENT`.
- evidencia positiva:
  - `F1N-LOOP-NEW-KF-DISPATCH`: `177`;
  - `F1N-LOOP-BOW-SEARCH`: `177`;
  - `F1N-LOOP-CANDIDATE-SUMMARY`: `177`;
  - `F1N-LOOP-BOW-CANDIDATES`: `167`;
  - `F1N-LOOP-CANDIDATE`: `852`;
  - `F1N-LOOP-CANDIDATE-RANK`: `852`;
  - `F1N-BOW-SKIP-CONFIRMED-COVIS`: `175`;
  - `same_drone=false`: `452`, `same_drone=true`: `400`;
  - `near_same_submap=true`: `139`;
  - summaries con `near_same_submap_candidates=[1-9]`: `260`;
  - no queda ninguna aparición de `too_recent_same_submap` en código ni en el
    log de `prueba_45`.
- evidencia secundaria de fases posteriores:
  - `geometry_confirmed=true`: `225`;
  - `LOOP_OPTIMIZATION_CANDIDATE`: `28`;
  - `FUSION_CANDIDATE`: `122`;
  - `F1M-COVIS-SUMMARY` final:
    `confirmed_edges=3542 orbslam3_native=3542 server_loop_geometric=0`;
  - `server_loop_geometric=0` es esperado en este punto: `3N` no inserta nuevas
    aristas confirmadas en `CovisibilityDatabase`.
- evidencia negativa o ausente:
  - no aparecen `SO3::exp failed`, `exit code -6`, `undefined symbol`,
    `Segmentation fault`, `LOOP_OPT_TASK_CREATED`, `FUSION_EVENT`,
    `RAWDB-POSE-OVERWRITE-BY-OPT` ni `raw_db_modified=true`;
  - aparecen un `[ERROR]` de `generador_URDF` y una muerte de `gazebo` después
    de `SIM-DONE` durante el cierre: no bloquean la conclusión funcional.
- conclusión:
  - `CONSEGUIDA` para la corrección de `3N`: la cercanía no confirmada ya no
    impide que un candidato BoW llegue a consumidores posteriores;
  - `PARCIAL` respecto a “muchas uniones nuevas en la base de covisibilidad”,
    porque esa inserción pertenece a fases posteriores y sigue correctamente en
    `server_loop_geometric=0`.
- siguiente paso recomendado:
  - cuando se aborde la fase de inserción de loops geométricos, usar esta misma
    prueba para exigir que los candidatos `geometry_confirmed=true` generen
    aristas `SERVER_LOOP_GEOMETRIC` con error asociado.

## 2026-08-15 — Subfase 3N — indice incremental y LoopTask BAJA

- objetivo intentado: ejecutar una tarea causal por KF, incluidos submapas no
  anclados, con indice BoW derivado, regiones y coalescencia por revisiones.
- archivos modificados: `loop_task.hpp`, `loop_pipeline.{hpp,cpp}`,
  `sparse_global_backend.{hpp,cpp}`, `secondary_queue.hpp` y servidor.
- paquetes compilados: los tres paquetes de la cadena sparse, build exit 0.
- pruebas: `test_loop_pipeline`, `test_secondary_queue`, regresion completa,
  replay 152/153 y live 154.
- patrones de reduccion: `F3N-LOOP-ENQUEUE`, lifecycle secundario,
  `F3O-LOOP-DONE`, cierre y errores.
- evidencia positiva: deduplicacion solo por revision causal exacta; una
  revision nueva conserva como maximo un rerun; replay 153 procesa 806 tareas
  y termina vacio; B/KF5 y B/KF7 participan aun antes del anchor fiducial.
- evidencia negativa o ausente: el primer diseño verificaba demasiadas
  geometrías intra-submapa y usaba hasta 120 iteraciones, dejando backlog en
  replay 152. Se limito la subnube a 320 puntos, RANSAC a 80 y se omite
  geometria redundante ante covisibilidad ORB fuerte.
- conclusion: `CONSEGUIDA`; el replay 153 reduce las ultimas tareas a
  aproximadamente 0.16-0.18 s y conserva `max_active=1`.
- siguiente paso recomendado: mantener los umbrales como parametros y revisar
  falsos positivos con escenas repetitivas al integrar 3P/3Q.
