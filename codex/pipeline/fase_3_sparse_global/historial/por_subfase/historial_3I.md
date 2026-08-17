# Historial 3I

> Extraido mecanicamente de `historial_fase_3.md`. Leer este archivo antes de abrir otros historiales de subfase.

## Nota de reclasificación 2026-07-14

Parte de la evidencia generada durante la sesión de `3L` pertenece realmente a
`3I`: selección métrica de vecinos fiduciales, muestreo por distancia acumulada,
vertices de esquina, splits de aristas largas y cobertura dura del grafo. Esa
evidencia sigue archivada en `historial_3L_2.md` a `historial_3L_4.md`, pero debe
leerse como evolución del contrato de `PoseGraphBuilder`.

## 2026-07-09 — Subfase 3I — PoseGraphBuilder temporal

- objetivo intentado:
  - crear `PoseGraphProblem` y `PoseGraphBuilder` en `orbslam3_multi`;
  - construir un grafo temporal desde `FiducialOptimizationTask`;
  - loggear vertices, fijos/variables, aristas, priors, pesos y `PropagationPlan`;
  - validar live y replay sin ejecutar solver ni aplicar poses.
- archivos modificados:
  - `orbslam3_multi/include/orbslam3_multi/pose_graph_problem.hpp`;
  - `orbslam3_multi/include/orbslam3_multi/pose_graph_builder.hpp`;
  - `orbslam3_multi/src/pose_graph_builder.cpp`;
  - `orbslam3_multi/CMakeLists.txt`;
  - `orbslam3_server/src/global_map_server.cpp`;
  - `orbslam3_server/launch/global_orb_map_server.launch.py`;
  - `codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml`;
  - `codex/archivos_auxiliares/trayectorias/tray_prueba_2.yaml`;
  - documentacion de contexto, paquetes, subfase e historial.
- cambios realizados:
  - `PoseGraphProblem` contiene vertices, aristas, priors, KFs fijos, KFs variables, KFs afectados no variables y `PropagationPlan`;
  - `PoseGraphBuilder::BuildForFiducialTask` construye el problema desde `FiducialOptimizationTask`, `RawMapDatabase` y `GlobalPoseStore`;
  - la ventana se limita al submapa `(drone_id, map_epoch)` de la tarea;
  - KFs hard fiducial entran fijos y KFs normales permanecen variables;
  - se crean aristas temporales y priors `FIDUCIAL_HARD`, `FIDUCIAL_TARGET` y `CURRENT_POSE_SOFT`;
  - `global_map_server` invoca el builder al detectar tareas pendientes de `FiducialAnchorManager`;
  - se anadieron parametros `pose_graph_*` y `f1i_debug_force_task_enabled`;
  - `tray_prueba_1.yaml` queda como la prueba tipica larga de rodeo con dos fiduciales;
  - `tray_prueba_2.yaml` queda como replay de espera con tarea debug equivalente habilitable.
- paquetes compilados:
  ```bash
  ./codex/herramientas/build_selected_packages.sh orbslam3_multi
  ./codex/herramientas/build_selected_packages.sh orbslam3_multi orbslam3_server
  ./codex/herramientas/build_selected_packages.sh simulacion_dron
  ```
- resultado de build:
  - `orbslam3_multi`: `BUILD-EXIT-CODE 0`;
  - `orbslam3_multi orbslam3_server`: `BUILD-EXIT-CODE 0`;
  - `simulacion_dron`: `BUILD-EXIT-CODE 0`;
  - no hizo falta ejecutar `reduce_build_log.sh`.
- pruebas ejecutadas:
  - prueba `1`: live larga con `ros2 launch simulacion_dron multi_dron.launch.py rawdb_record_enabled:=false f1g_debug_mark_optimized_kf:=false`;
  - prueba `2`: replay con `ros2 launch orbslam3_server global_orb_map_server.launch.py ... rawdb_replay_enabled:=true ... f1i_debug_force_task_enabled:=true`.
- resultado de simulacion:
  - `prueba_1`: `SCENARIO-RUNNER-DONE ... success=true`, `SIM-DONE prueba=1 success=true`, `SIM-EXIT-CODE 0`;
  - `prueba_2`: `SCENARIO-RUNNER-DONE ... success=true`, `SIM-DONE prueba=2 success=true`, `SIM-EXIT-CODE 0`.
- patrones previstos para reduccion:
  - prueba 1:
    ```text
    SCENARIO-RUNNER|GOAL|RESULT|success|F1I-|F1H-FID-TASK-CREATED|F1E-FID|F1D-POSESTORE|F1C-RAWDB|ERROR|FATAL|Segmentation fault|Killed
    ```
  - prueba 2:
    ```text
    SCENARIO-RUNNER|GOAL|RESULT|success|F1I-|F1H-FID-TASK-CREATED|F1E-FID|F1D-POSESTORE|F1C-REPLAY|F1C-RAWDB|ERROR|FATAL|Segmentation fault|Killed
    ```
- nota sobre logs:
  - por instruccion final del usuario, no se regeneraron `prueba_1.reduced.log` ni `prueba_2.reduced.log` en el cierre;
  - la evidencia se consulto directamente en `codex/archivos_auxiliares/logs/prueba_1.log` y `codex/archivos_auxiliares/logs/prueba_2.log`.
- evidencia positiva encontrada:
  - `prueba_1.log`: `109` lineas `[F1H-FID-TASK-CREATED]`;
  - `prueba_1.log`: `109` lineas `[F1I-GRAPH-BUILD-SUMMARY] ... success=true`;
  - `prueba_1.log`: `109` lineas `[F1I-GRAPH-PROPAGATION-PLAN]`;
  - `prueba_2.log`: `16` lineas `[F1H-FID-TASK-CREATED]` reales desde `REPLAY_RECORDED_FIDUCIAL`;
  - `prueba_2.log`: `17` lineas `[F1I-GRAPH-BUILD-SUMMARY] ... success=true`;
  - `prueba_2.log`: aparece `[F1I-DEBUG-TASK-CREATED] task_id=9000000001`;
  - grafos con `vertices=12`, `edges=11`, `priors=12` y propagacion no vacia;
  - no aparecieron `FATAL`, `Segmentation fault`, `Killed`, `OPT-APPLY`, `OPTIMIZATION-APPLIED` ni `SET_OPTIMIZED_POSE`.
- evidencia negativa o ausente:
  - no se ejecuta solver ni apply en esta subfase;
  - `prueba_1.log` contiene `gazebo ... exit code 255` despues de `SIM-DONE success=true`, durante cleanup; patron no bloqueante ya observado;
  - no se regeneraron logs reducidos por la instruccion de cierre.
- documentacion actualizada:
  - `codex/contexto/07_ULTIMA_SESION.md`;
  - `codex/contexto/01_ESTADO_ACTUAL.md`;
  - `codex/contexto/06_MAPA_CODIGO.md`;
  - `codex/contexto/paquetes/orbslam3_multi/orbslam3_multi.md`;
  - `codex/contexto/paquetes/orbslam3_multi/pose_graph_builder.md`;
  - `codex/contexto/paquetes/orbslam3_multi/pose_graph_problem.md`;
  - `codex/contexto/paquetes/orbslam3_server/global_map_server.md`;
  - `codex/contexto/paquetes/orbslam3_server/launches.md`;
  - `codex/pipeline/fase_3_sparse_global/pipeline_fase_3.md`;
  - `codex/pipeline/fase_3_sparse_global/subfases/subfase_3I.md`;
  - `codex/pipeline/fase_3_sparse_global/historial/historial_fase_3.md`.
- conclusion: `CONSEGUIDA`.
- siguiente paso recomendado:
  - ejecutar `subfase_3J.md`: dry-run de optimizacion sobre `PoseGraphProblem`, sin aplicar cambios persistentes todavia.

## 2026-07-09 — Revalidacion Subfase 3I — cierre de evidencia reducida

- objetivo intentado:
  - comprobar el estado real de `subfase_3I.md` tras un cierre anterior incompleto por capacidad;
  - repetir el ciclo automatico de build, simulacion, reduccion y analisis definido en `AGENTS.md`;
  - conservar evidencia compacta en `prueba_1.reduced.log` y `prueba_2.reduced.log`.
- archivos modificados:
  - no se modifico codigo fuente;
  - no se modificaron YAMLs de prueba;
  - se actualizo documentacion de cierre:
    - `codex/contexto/07_ULTIMA_SESION.md`;
    - `codex/contexto/01_ESTADO_ACTUAL.md`;
    - `codex/pipeline/fase_3_sparse_global/subfases/subfase_3I.md`;
    - `codex/pipeline/fase_3_sparse_global/historial/historial_fase_3.md`.
- paquetes compilados:
  ```bash
  ./codex/herramientas/build_selected_packages.sh orbslam3_multi
  ./codex/herramientas/build_selected_packages.sh orbslam3_multi orbslam3_server
  ./codex/herramientas/build_selected_packages.sh simulacion_dron
  ```
- resultado de build:
  - los tres builds terminaron con `BUILD-EXIT-CODE 0`;
  - no hizo falta ejecutar `reduce_build_log.sh`.
- pruebas Gazebo ejecutadas:
  - prueba `1`: live larga con `ros2 launch simulacion_dron multi_dron.launch.py rawdb_record_enabled:=false f1g_debug_mark_optimized_kf:=false`;
  - prueba `2`: replay/debug con `ros2 launch orbslam3_server global_orb_map_server.launch.py ... rawdb_replay_enabled:=true ... f1i_debug_force_task_enabled:=true`;
  - ambas terminaron con `SCENARIO-RUNNER-DONE ... success=true`, `SIM-DONE ... success=true` y `SIM-EXIT-CODE 0`.
- patrones usados para reducir logs:
  - se probo primero el patron amplio de la subfase; en `prueba_1.reduced.log` resulto demasiado verboso y oculto marcadores `F1I` relevantes por limite de reduccion;
  - se consulto el log completo como exige `AGENTS.md`;
  - se regeneraron los reducidos con patrones centrados en:
    ```text
    SCENARIO-RUNNER|SIM-DONE|SIM-EXIT-CODE|GOAL|RESULT|success|F1H-FID-TASK-CREATED|F1H-FID-TASK-STATS|F1I-|F1C-REPLAY|ERROR|FATAL|Segmentation fault|Killed|OPT-APPLY|OPTIMIZATION-APPLIED|SET_OPTIMIZED_POSE
    ```
- evidencia positiva encontrada:
  - `prueba_1.reduced.log`: `15` tareas `[F1H-FID-TASK-CREATED]`;
  - `prueba_1.reduced.log`: `15` grafos `[F1I-GRAPH-BUILD-SUMMARY] ... success=true`;
  - `prueba_1.reduced.log`: aparecen `F1I-TASK-RX`, `F1I-GRAPH-BUILD-START`, `F1I-GRAPH-WINDOW`, `F1I-GRAPH-VERTEX-SELECT`, `F1I-GRAPH-EDGES`, `F1I-GRAPH-PRIORS`, `F1I-GRAPH-WEIGHTS`, `F1I-GRAPH-PROPAGATION-PLAN` y `F1I-GRAPH-PROBLEM-CREATED`;
  - `prueba_2.reduced.log`: `16` tareas reales `[F1H-FID-TASK-CREATED]`;
  - `prueba_2.reduced.log`: `17` grafos `[F1I-GRAPH-BUILD-SUMMARY] ... success=true`, incluyendo la tarea debug `task_id=9000000001`;
  - la tarea debug construyo un grafo con `vertices=12`, `variable=10`, `fixed=2`, `hard_fiducial=2`, `edges=11`, `priors=12`, `affected_non_variable=5` y `propagation=5`.
- evidencia negativa o ausente:
  - no aparecieron eventos reales `OPT-APPLY`, `OPTIMIZATION-APPLIED` ni `SET_OPTIMIZED_POSE`, correcto para `3I`;
  - no aparecieron `FATAL`, `Segmentation fault`, `Killed` ni `std::bad_alloc`;
  - `prueba_1.reduced.log` conserva el `gazebo ... exit code 255` posterior a `SIM-DONE`, durante cleanup; se mantiene como no bloqueante.
- conclusion: `CONSEGUIDA`.
- siguiente paso recomendado:
  - ejecutar `subfase_3J.md`: dry-run de optimizacion sobre `PoseGraphProblem`, sin aplicar todavia cambios persistentes.

## 2026-07-10 — Subfase 3I — Revalidacion con preservacion de anclajes fiduciales previos

- objetivo intentado:
  - corregir la construccion del `PoseGraphProblem` para que una tarea fiducial multi-anclaje no llegue con `fixed=0 hard_fiducial=0`;
  - incluir/proteger fiduciales previos del mismo submapa como frontera fija;
  - emitir logs explicitos de conectividad fiducial, ramas y preservacion de anclajes;
  - validar con prueba larga sin ejecutar dry-run/apply/post-apply.
- archivos modificados:
  - `orbslam3_multi/include/orbslam3_multi/pose_graph_problem.hpp`;
  - `orbslam3_multi/include/orbslam3_multi/pose_graph_builder.hpp`;
  - `orbslam3_multi/src/pose_graph_builder.cpp`;
  - `orbslam3_server/src/global_map_server.cpp`;
  - `codex/contexto/01_ESTADO_ACTUAL.md`;
  - `codex/contexto/07_ULTIMA_SESION.md`;
  - `codex/contexto/paquetes/orbslam3_multi/pose_graph_builder.md`;
  - `codex/contexto/paquetes/orbslam3_multi/pose_graph_problem.md`;
  - `codex/contexto/paquetes/orbslam3_server/global_map_server.md`;
  - `codex/contexto/paquetes/orbslam3_server/orbslam3_server.md`;
  - `codex/pipeline/fase_3_sparse_global/pipeline_fase_3.md`;
  - `codex/pipeline/fase_3_sparse_global/subfases/subfase_3I.md`;
  - `codex/pipeline/fase_3_sparse_global/subfases/subfase_3J.md`;
  - `codex/pipeline/fase_3_sparse_global/historial/historial_fase_3.md`.
- cambios realizados:
  - `PoseGraphProblem` anade `FiducialConnectivityEdge`, `FiducialConnectivityEdgeStatus` y `PoseGraphAnchorPreservationSummary`;
  - `PoseGraphBuilder` recopila KFs hard fiducial previos del mismo `(drone_id, map_epoch)`, selecciona anclajes frontera por rama temporal cercana y los inserta como vertices fijos;
  - la ventana se expande para incluir esos anclajes aunque queden fuera del path nominal;
  - los fiduciales dominados por un anclaje mas cercano en la misma rama se registran como `SUBDIVIDED_CONFIRMED`;
  - `global_map_server` declara/configura `pose_graph_fiducial_connectivity_enabled`, `pose_graph_branch_selection_enabled` y `pose_graph_subdivision_candidate_radius_m`;
  - `global_map_server` emite `[F1I-FID-CONNECTIVITY-BRANCHES]`, `[F1I-FID-CONNECTIVITY-EDGE]`, `[F1I-FID-CONNECTIVITY-SUBDIVISION]`, `[F1I-GRAPH-ANCHOR-PRESERVATION]` y `[F1I-GRAPH-PREVIOUS-FIDUCIAL-ANCHOR]`.
- YAMLs de prueba:
  - se reutilizo `codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml` como prueba larga de rodeo;
  - se reutilizo `codex/archivos_auxiliares/trayectorias/tray_prueba_2.yaml` como live equivalente a replay porque `rawdb_prueba_1.record` no estaba disponible.
- paquetes compilados:
  ```bash
  ./codex/herramientas/build_selected_packages.sh orbslam3_multi
  ./codex/herramientas/build_selected_packages.sh orbslam3_server
  ./codex/herramientas/build_selected_packages.sh simulacion_dron
  ```
- resultado de build:
  - los tres builds terminaron con salida `0`;
  - no hizo falta ejecutar `reduce_build_log.sh`;
  - `orbslam3_multi` mostro solo un warning preexistente de funcion no usada en `optimization_manager.cpp`.
- pruebas Gazebo ejecutadas:
  - prueba `1` live larga:
    ```bash
    ./codex/herramientas/run_simulation.sh --prueba 1 --launch "ros2 launch simulacion_dron multi_dron.launch.py rawdb_record_enabled:=false f1j_dryrun_enabled:=false f1k_apply_enabled:=false f1l_validation_enabled:=false" --post-scenario-wait-sec 60 --startup-wait-sec 20 --timeout-sec 1600 --max-gazebo-retries 1
    ```
  - prueba `2` live equivalente a replay:
    ```bash
    ./codex/herramientas/run_simulation.sh --prueba 2 --launch "ros2 launch simulacion_dron multi_dron.launch.py rawdb_record_enabled:=false f1j_dryrun_enabled:=false f1k_apply_enabled:=false f1l_validation_enabled:=false" --post-scenario-wait-sec 60 --startup-wait-sec 20 --timeout-sec 1600 --max-gazebo-retries 1
    ```
- resultado de simulacion:
  - `prueba_1`: `14` goals `success=true`, `SCENARIO-RUNNER-DONE scenario='prueba_tipica_rodeo_edificio_dos_fiduciales' success=true`, `SIM-DONE prueba=1 success=true`, `SIM-EXIT-CODE 0`;
  - `prueba_2`: `14` goals `success=true`, `SCENARIO-RUNNER-DONE scenario='prueba_1l_rodeo_edificio_rollback_forzado' success=true`, `SIM-DONE prueba=2 success=true`, `SIM-EXIT-CODE 0`.
- patrones usados para reducir logs:
  ```text
  SCENARIO-RUNNER-GOAL-RESULT|SCENARIO-RUNNER-DONE|SIM-DONE|SIM-EXIT-CODE|F1H-FID-TASK-CREATED|F1I-GRAPH|F1I-FID-CONNECTIVITY|ERROR|FATAL|Segmentation fault|Killed|process has died
  ```
- logs reducidos:
  - `codex/archivos_auxiliares/logs/prueba_1.reduced.log`;
  - `codex/archivos_auxiliares/logs/prueba_2.reduced.log`.
- evidencia positiva:
  - `prueba_1.log`: `33` `[F1H-FID-TASK-CREATED]`, `33` `[F1I-GRAPH-BUILD-SUMMARY]`, `33` `[F1I-GRAPH-ANCHOR-PRESERVATION]`;
  - `prueba_2.log`: `16` `[F1H-FID-TASK-CREATED]`, `16` `[F1I-GRAPH-BUILD-SUMMARY]`, `16` `[F1I-GRAPH-ANCHOR-PRESERVATION]`;
  - caso critico de `prueba_1`: `task_id=1` con `error_t=22.312883` ahora muestra `required=true satisfied=true previous_fiducial_fixed_count=1` y `fixed=1 hard_fiducial=1`;
  - caso critico equivalente de `prueba_2`: `task_id=1` muestra `required=true satisfied=true previous_fiducial_fixed_count=1` y `fixed=1 hard_fiducial=1`;
  - aparecen logs de conectividad con `DIRECT_UNCERTAIN`, `DIRECT_OBSERVED` y `SUBDIVIDED_CONFIRMED`;
  - no aparecen `OPT-APPLY`, `OPTIMIZATION-APPLIED` ni `SET_OPTIMIZED_POSE`, como corresponde a una validacion estricta de `3I`.
- evidencia negativa o notas:
  - no se ejecuto replay real porque no existia `codex/archivos_auxiliares/repeticiones/rawdb_prueba_1.record`;
  - el cierre conocido de Gazebo `exit code 255` aparece despues de `SIM-DONE success=true` y no invalida la prueba;
  - no se observo RViz2 manualmente durante esta ejecucion.
- conclusion:
  - `CONSEGUIDA`.
- siguiente paso recomendado:
  - ejecutar `subfase_3J.md` para revalidar `OptimizationManager::RunDryRun` con los grafos multi-anclaje ya protegidos.

## 2026-07-21 — Subfase 3I — Trayectoria fiducial completa sin límites ni covisibilidad

- objetivo intentado:
  - eliminar el límite métrico de `4 m`, el máximo de vértices y el recorte por
    longitud topológica;
  - usar todos los KFs consecutivos del mismo submapa entre hard fiducial previo
    y target;
  - excluir temporalmente las aristas de covisibilidad del grafo fiducial;
  - repetir replay y prueba live canónica.
- archivos de código/launch modificados:
  - `orbslam3_multi/include/orbslam3_multi/pose_graph_builder.hpp`;
  - `orbslam3_multi/src/pose_graph_builder.cpp`;
  - `orbslam3_server/src/global_map_server.cpp`;
  - `orbslam3_server/launch/global_orb_map_server.launch.py`;
  - `simulacion_dron/launch/multi_dron.launch.py`.
- cambios:
  - retirados `pose_graph_max_vertices`, `pose_graph_max_path_length`,
    `pose_graph_max_temporal_edge_kf_gap` y
    `pose_graph_max_temporal_edge_length_m`;
  - selección `all_consecutive_between_fiducials` sin muestreo ni
    `PropagationPlan` interno;
  - las aristas conectan vecinos reales de la secuencia aunque haya huecos de
    ID local;
  - añadido `pose_graph_use_covisibility_edges=false`; el servidor pasa
    `nullptr` al builder y registra `returned_edges=0`.
- paquetes compilados:
  - `orbslam3_multi`, `orbslam3_server`, `simulacion_dron`.
- resultado de build:
  - `BUILD-EXIT-CODE 0`, tres paquetes terminados en `21.2 s`;
  - no hizo falta `reduce_build_log.sh`.
- pruebas:
  - `prueba_14` replay del record previo con `tray_prueba_2.yaml`;
  - `prueba_15` live con
    `prueba_tipica_rodeo_edificio_dos_fiduciales.yaml`;
  - logs reducidos `prueba_14.reduced.log` y `prueba_15.reduced.log`.
- evidencia positiva replay:
  - `F1C-REPLAY-LOAD success=true`, `145` entradas, `409` KFs y `24`
    observaciones fiduciales;
  - ventana `[50,160]`: `window_keyframes=79`, `vertices=79`, `edges=78`,
    `propagation=0`, `coverage_complete=true`;
  - `enabled_for_fiducial_graph=false returned_edges=0`;
  - solver `success=true`, coste `2493405.016794 -> 606986.267820`, target
    `22.274834 m -> 0`;
  - HTML generado en
    `codex/archivos_auxiliares/html/f3l_debug_animation_task_1.html`;
  - coherencia interna post-apply `ok=true`, cero aristas rotas y hard fiducial
    inmóvil.
- evidencia negativa replay:
  - `global_map_check_failed`: puntos publicados `27239 -> 26661` e
    `invalid_pose_skipped 1105 -> 1683`; `3L` hizo `REJECT_ROLLBACK`;
  - este rechazo sucede después de construir/optimizar el grafo y queda como
    deuda de `3K/3L`.
- evidencia live:
  - `14` goals con `success=true`, `SCENARIO-RUNNER-DONE success=true`,
    `SIM-DONE success=true`, `SIM-EXIT-CODE 0`;
  - `405` skips BoW por covisibilidad confirmada, `104` verificaciones
    geométricas positivas y `464` negativas; loops y covisibilidad diagnóstica
    siguieron activos;
  - no hubo `stereo-8 exit code -6`, `SO3::exp failed`, crash ni NaN;
  - Gazebo terminó con `exit code 255` durante cleanup, después de
    `SIM-DONE success=true`.
- evidencia negativa live:
  - la ejecución no registró ninguna observación de `fid=1`;
  - creó una tarea prematura `fid=2`, `error_t=0.435635`, que se rechazó con
    `previous_fiducial_anchor_missing`; no hubo pareja fiducial live válida.
  - el live sobrescribió `rawdb_prueba_1.record`; su último save tiene `139`
    entradas, `468` KFs y `30` observaciones, todas sin `fid=1`, por lo que ese
    record ya no reproduce la tarea válida usada por `prueba_14`.
- conclusión: `PARCIAL`.
- siguiente paso recomendado:
  - repetir la prueba live hasta obtener observaciones de ambos fiduciales y,
    después, diagnosticar por separado `global_map_check_failed` en `3K/3L`.

## 2026-07-21 - Subfase 3I - Muestreo porcentual y extremos fiduciales obligatorios

- objetivo:
  - recuperar la politica de grafo anterior con cambios minimos;
  - retirar el maximo absoluto y el limite metrico de `4 m`;
  - convertir solo un porcentaje de KFs en vertices, repartidos por toda la
    trayectoria;
  - incluir siempre los KFs del fiducial previo y del fiducial target;
  - mantener fuera las aristas de covisibilidad.
- archivos de codigo, launch y prueba modificados:
  - `orbslam3_multi/include/orbslam3_multi/pose_graph_builder.hpp`;
  - `orbslam3_multi/src/pose_graph_builder.cpp`;
  - `orbslam3_server/src/global_map_server.cpp`;
  - `orbslam3_server/launch/global_orb_map_server.launch.py`;
  - `simulacion_dron/launch/multi_dron.launch.py`;
  - `codex/archivos_auxiliares/trayectorias/prueba_tipica_rodeo_edificio_dos_fiduciales.yaml`.
- implementacion:
  - nuevo `pose_graph_vertex_selection_ratio=0.30`, acotado a `[0,1]`;
  - numero de controles `ceil(ratio * KFs disponibles)`, minimo dos y sin
    maximo absoluto;
  - insercion obligatoria de `previous_fiducial_anchor` y
    `target_fiducial_error` antes del muestreo;
  - muestras por distancia planar acumulada y completado determinista por
    maxima separacion sobre la trayectoria;
  - aristas temporales entre controles seleccionados y `PropagationPlan` para
    el resto;
  - sin rechazo ni subdivision por distancia, gap de ID o longitud de ventana;
  - el radio fiducial de `4 m` queda solo como etiqueta de controles ya
    seleccionados para el solver;
  - procesamiento fiducial de KFs nuevos tambien en full snapshots;
  - buffer GT de simulacion ampliado a `5000` muestras para asociaciones
    atrasadas por timestamp;
  - barridos laterales y espera en `fid=1` dentro de la trayectoria canonica.
- paquetes compilados:
  - `orbslam3_multi`, `orbslam3_server`, `simulacion_dron` durante la iteracion;
  - build final de `orbslam3_multi orbslam3_server` tras cerrar el cupo exacto.
- resultado de build:
  - todos los builds terminaron con `BUILD-EXIT-CODE 0`;
  - no fue necesario `reduce_build_log.sh`.
- pruebas relevantes:
  - `prueba_19` replay inicial: `93` KFs, `28` vertices, `65` propagaciones,
    solver `1.728275 -> 0` y HTML;
  - `prueba_22` live final con la trayectoria canonica modificada;
  - `prueba_24` replay final del record de `prueba_22` con el cupo porcentual
    estricto. `prueba_23` no cuenta: fallo por una ruta YAML relativa incorrecta
    y se repitio correctamente como `prueba_24`.
- patrones de reduccion:
  ```text
  SCENARIO-RUNNER|SIM-|F1C-REPLAY|F1E-FID|F1H-FID|F1I-GRAPH|F1J-OPT|F1M-COVIS|F1L-DEBUG-ANIMATION|FATAL|ERROR|Segmentation|process has died|exit code -6
  ```
- evidencia de replay final:
  - `F1C-REPLAY-LOAD success=true`, `103` entradas, `498` KFs y `44`
    observaciones fiduciales;
  - tarea debug larga: `window_keyframes=168`, `vertices=51`, exactamente
    `ceil(0.30 * 168)`, `edges=50`, `propagation=117`;
  - aparecen `previous_fiducial_anchor fixed=true` y
    `target_fiducial_error variable=true`;
  - configuracion `covisibility_edges=false`; no hay aristas `F1M_*`;
  - solver `success=true`, 8 iteraciones, coste
    `371528.383852 -> 73.967050`, `hard_fixed_moved=false`;
  - HTML generado en
    `codex/archivos_auxiliares/html/f3l_debug_animation_task_9000000001.html`;
  - `SCENARIO-RUNNER-DONE success=true`, `SIM-DONE success=true` y
    `SIM-EXIT-CODE 0`, sin errores graves.
- evidencia live:
  - `18` resultados de goal correctos, escenario y simulacion `success=true`;
  - `fid=2` se asocia en epoch 1; `fid=1` se asocia en epoch 2 mediante full
    snapshot (`drone_id=2`, KFs `198/197/196`), y el KF `196` crea tarea;
  - al final aparecen observaciones de `fid=2` en epoch 4;
  - no existe una pareja de fiduciales distintos dentro del mismo
    `(drone_id,map_epoch)`, por lo que los grafos reales se rechazan con
    `previous_fiducial_anchor_missing`.
  - Gazebo publica `process has died ... exit code 255` durante el cleanup,
    despues de `SIM-DONE success=true`; no afecta al resultado del escenario.
- conclusion: `PARCIAL`.
- siguiente paso recomendado:
  - diagnosticar por que ORB-SLAM3 cambia `map_epoch` durante el rodeo y obtener
    una ejecucion con ambos fiduciales en el mismo submapa; no relajar la
    invariante ni optimizar cruzando epochs.

## 2026-07-21 - Subfase 3I - Cierre live aislando la carga de loops

- objetivo:
  - repetir la trayectoria canonica con la ruta fiducial activa y el pipeline
    BoW desactivado para aislar la validacion de `3I`.
- cambio auxiliar:
  - se expuso el parametro ya existente `loop_bow_min_mappoints` en
    `global_orb_map_server.launch.py` y `multi_dron.launch.py`;
  - su default sigue siendo `15`; `prueba_25` uso `1000000` para impedir
    verificaciones de loops durante el tracking.
- build:
  - `orbslam3_server` y `simulacion_dron`: `BUILD-EXIT-CODE 0`.
- prueba:
  - `prueba_25` con
    `prueba_tipica_rodeo_edificio_dos_fiduciales.yaml`, recording desactivado,
    ratio `0.30`, covisibilidad fiducial desactivada y apply habilitado;
  - `18` goals `success=true`, `SCENARIO-RUNNER-DONE success=true`,
    `SIM-DONE success=true`, `SIM-EXIT-CODE 0`.
- grafos reales:
  - tarea `2`, dron 1 epoch 0: `106` KFs, `32` vertices, `31` aristas y `74`
    propagaciones;
  - tarea `3`, dron 2 epoch 0: `151` KFs, `46` vertices, `45` aristas y `105`
    propagaciones;
  - tarea `4`, dron 1 epoch 3: `86` KFs, `26` vertices, `25` aristas y `60`
    propagaciones;
  - en todos los casos el conteo es `ceil(0.30 * N)`, aparecen
    `previous_fiducial_anchor` y `target_fiducial_error`, y
    `[F1M-COVIS-QUERY] returned_edges=0`.
- optimizacion:
  - tarea `2`: coste `98139.209921 -> 89.275858`, error
    `0.967464 m -> 0`, apply correcto y rollback posterior por
    `global_map_check_failed` (`543` puntos no publicables);
  - tarea `3`: coste `4489132.211522 -> 1523513.632731`, error
    `29.892396 m -> 0`, apply correcto y rollback posterior por
    `global_map_check_failed` (`614` puntos no publicables);
  - tarea `4`: coste `20305.421992 -> 823.276936`, error
    `0.444541 m -> 0`, apply correcto y
    `[F1L-POST-APPLY-ACCEPT]`; puntos publicados `73917 -> 73917`, cero poses
    invalidas, propagacion continua y hard fiducial inmovil.
- artefactos:
  - `codex/archivos_auxiliares/logs/prueba_25.log`;
  - `codex/archivos_auxiliares/logs/prueba_25.reduced.log`;
  - `codex/archivos_auxiliares/html/f3l_debug_animation_task_2.html`;
  - `codex/archivos_auxiliares/html/f3l_debug_animation_task_3.html`;
  - `codex/archivos_auxiliares/html/f3l_debug_animation_task_4.html`.
- errores graves:
  - no hay `FATAL`, `Segmentation fault`, `exit code -6`, abort ni NaN;
  - Gazebo emite el `exit code 255` conocido durante cleanup, despues de
    `SIM-DONE success=true`.
- conclusion de `3I`: `CONSEGUIDA`.
- deuda separada:
  - investigar en `3K/3L` por que dos propuestas correctas de KF dejan algunos
    MapPoints sin pose publicable; no volver a introducir todos los KFs ni
    limites absolutos en `PoseGraphBuilder`.

## 2026-07-21 - Subfase 3I - Reapertura por errores de grafo

- decision del usuario:
  - volver a situar `3I` como subfase actual porque siguen observandose errores
    en la creacion del grafo;
  - preparar documentacion suficiente y compacta para el siguiente chat.
- cambio de interpretacion:
  - la tarea `4` aceptada de `prueba_25` sigue siendo evidencia positiva;
  - las tareas `2` y `3` ya no se clasifican de antemano como deuda externa de
    `3K/3L`;
  - `global_map_check_failed`, `invalid_pose_skipped +543/+614` y los rollbacks
    se consideran sintomas compatibles con vertices, aristas, priors o
    propagacion incorrectos hasta que el diagnostico pruebe otra causa.
- estado funcional conservado:
  - ratio `0.30`, extremos fiduciales obligatorios, cero covisibilidad, sin
    maximo absoluto ni limite de `4 m` y propagacion de no vertices;
  - no se debe deshacer esta politica para intentar ocultar el fallo.
- trabajo requerido:
  - comparar estructura completa de `task_id=2`, `3` y `4`;
  - validar identidad/orden de endpoints, transformaciones temporales, soporte,
    priors y todos los segmentos de `PropagationPlan`;
  - generar dump reproducible de los casos live;
  - ejecutar replay, live aislada por dron y regresion normal antes de cerrar.
- archivos documentales actualizados:
  - contexto minimo, estado corto/largo, bootstraps, pipeline corto/largo,
    contrato `subfase_3I.md`, historial/indice, ultima sesion y docs de paquetes.
- codigo/build/simulacion:
  - no se modifico codigo;
  - no se ejecuto build ni simulacion en esta reapertura documental.
- conclusion: `PARCIAL`.
- siguiente paso recomendado:
  - actuar como `planificador_fase` de `3I`, producir dumps comparables de las
    tareas fallidas/aceptada y corregir solo el primer defecto estructural
    demostrado.

## 2026-07-22 21:51 — Subfase 3I — política fiducial y de grafo aclarada

- objetivo intentado:
  - documentar la semántica acordada para optimizar cuando un submapa ya
    anclado llega a un fiducial;
  - aclarar la diferencia entre primer fiducial que ancla un submapa y primer
    fiducial de una optimización;
  - dejar explícitos los criterios de selección de vértices, objetivo 6D,
    vecindades fiduciales protegidas y dudas sobre pesos de aristas.
- archivos modificados:
  - `codex/pipeline/fase_3_sparse_global/subfases/subfase_3I.md`;
  - `codex/pipeline/fase_3_sparse_global/subfases/subfase_3H.md`;
  - `codex/pipeline/fase_3_sparse_global/pipeline_fase_3.md`;
  - `codex/pipeline/fase_3_sparse_global/pipeline_fase_3_RESUMEN.md`;
  - `codex/contexto/CONTEXTO_MINIMO_ACTUAL.md`;
  - `codex/contexto/01_ESTADO_ACTUAL_RESUMEN.md`;
  - `codex/contexto/07_ULTIMA_SESION.md`;
  - `codex/contexto/paquetes/orbslam3_multi/00_summary.md`;
  - `codex/contexto/paquetes/orbslam3_multi/fiducial_anchor_manager.md`;
  - `codex/contexto/paquetes/orbslam3_multi/pose_graph_builder.md`;
  - `codex/contexto/paquetes/orbslam3_multi/pose_graph_problem.md`;
  - `codex/contexto/paquetes/orbslam3_multi/optimization_manager.md`;
  - `codex/contexto/paquetes/orbslam3_server/global_map_server.md`.
- paquetes compilados:
  - no aplica; no se modificó código.
- resultado de build:
  - no ejecutado.
- pruebas Gazebo/replay:
  - no ejecutadas.
- patrones usados para reducir logs:
  - no aplica.
- evidencia positiva:
  - el contrato de `3I` ahora indica que cualquier fiducial posterior al anchor
    inicial puede crear tarea si el error 6D supera umbral;
  - la selección de vértices queda definida como porcentual y relativa a la
    ventana, sin umbrales absolutos ni esquinas yaw-only;
  - el target fiducial debe llegar a la pose absoluta conocida en posición y
    orientación completa;
  - las vecindades pares de ambos fiduciales deben conservar pose relativa local
    mediante pose inducida fija o restricciones de peso muy alto;
  - la política de pesos de aristas temporales queda marcada como provisional y
    revisable tras inspección de grafos/HTML y pruebas.
- evidencia negativa o ausente:
  - no hay build ni simulación en esta sesión documental;
  - `3I` sigue sin cerrar los rollbacks de `prueba_25` en tareas `2` y `3`.
- conclusión: `PARCIAL`.
- siguiente paso recomendado:
  - implementar/diagnosticar `PoseGraphBuilder` y `OptimizationManager` contra
    esta política: comparar tareas `2`, `3` y `4`, sustituir señales yaw-only
    por geometría 3D/SE(3), verificar vecindades protegidas y revalidar con
    replay, live aislada y regresión normal.

## 2026-07-22 22:20 — Subfase 3I — implementación 3D y ventana reproducible

- objetivo intentado:
  - aplicar la politica acordada: cualquier fiducial posterior al anchor puede
    crear tarea, vertices porcentuales sin limites absolutos, esquinas 3D/SE(3)
    y vecindades protegidas de ambos fiduciales;
  - sustituir el HTML diagnostico 2D por HTML 3D navegable con poses reales y
    flechas de orientacion;
  - guardar ventanas de KFs para reproducir grafo/solver sin repetir Gazebo.
- codigo modificado:
  - `orbslam3_multi/include/orbslam3_multi/pose_graph_builder.hpp`;
  - `orbslam3_multi/src/pose_graph_builder.cpp`;
  - `orbslam3_multi/src/optimization_manager.cpp`;
  - `orbslam3_multi/src/test_opt_graph_offline.cpp`;
  - `orbslam3_server/src/global_map_server.cpp`;
  - `orbslam3_server/launch/global_orb_map_server.launch.py`;
  - `simulacion_dron/launch/multi_dron.launch.py`.
- comportamiento implementado:
  - `PoseGraphBuilder` anade `fiducial_neighborhood_vertex_ratio=0.20` y
    selecciona un numero par de vecinos protegidos, mitad cerca del fiducial
    previo y mitad cerca del target;
  - la deteccion de esquinas usa angulo 3D entre tramos de trayectoria y cambio
    rotacional `SO(3)`, con `selection_reason=corner_3d_vertex`;
  - `OptimizationManager` acepta `corner_3d_vertex` y conserva
    `corner_yaw_vertex` como compatibilidad con dumps anteriores;
  - `ExportF1LDebugAnimation` y `test_opt_graph_offline` generan HTML 3D con
    canvas, orbitado por raton/rueda, aristas, frames inicial/grafo/optimizado
    y flechas de orientacion por KF;
  - `DumpPoseGraphProblemForOffline` guarda tambien
    `f3i_window_task_<task_id>.tsv` con KFs de ventana, poses mapa, poses GT
    debug si existen y KFs fiduciales.
- build:
  - `./codex/herramientas/build_selected_packages.sh orbslam3_multi orbslam3_server`
    termina con `BUILD-EXIT-CODE 0`;
  - `./codex/herramientas/build_selected_packages.sh simulacion_dron` termina
    con `BUILD-EXIT-CODE 0`;
  - warnings conocidos/preexistentes: `dry_run` no usado y helpers internos no
    usados en `optimization_manager.cpp`.
- prueba replay:
  - comando base: replay raw sobre
    `codex/archivos_auxiliares/repeticiones/rawdb_prueba_1.record`, tarea debug
    forzada, `pose_graph_vertex_selection_ratio=0.30`,
    `pose_graph_use_covisibility_edges=false`,
    `loop_bow_min_mappoints=1000000`, dump y HTML activados;
  - resultado: `SIM-DONE prueba=26 success=true`, `SIM-EXIT-CODE 0`;
  - replay carga `103` entradas, `5` submapas, `498` KFs, `38662` MPs y `44`
    observaciones fiduciales.
- evidencia F1I:
  - `[F1I-GRAPH-BUILDER-CONFIG]` publica
    `fiducial_neighborhood_vertex_ratio=0.200` y
    `corner_3d_threshold_rad=0.524`;
  - tarea debug `task_id=9000000001`: `window_keyframes=168`, `vertices=51`,
    `edges=50`, `priors=51`, `variables=50`, `fixed=1`,
    `propagation=117`;
  - aparecen `17` vertices `corner_3d_vertex`, `5`
    `previous_fiducial_neighborhood` y `5`
    `target_fiducial_neighborhood`;
  - `[F1I-WINDOW-DUMP] success=true` guarda
    `codex/archivos_auxiliares/repeticiones/f3i_window_task_9000000001.tsv`;
  - `[F1L-GRAPH-DUMP] success=true` guarda
    `codex/archivos_auxiliares/repeticiones/f3l_graph_task_9000000001.tsv`;
  - `[F1L-DEBUG-ANIMATION-EXPORT] ... format=html_3d_canvas` guarda
    `codex/archivos_auxiliares/html/f3l_debug_animation_task_9000000001.html`.
- prueba offline:
  - `ros2 run orbslam3_multi test_opt_graph_offline --graph
    /home/chenfu/Gazebo/src/codex/archivos_auxiliares/repeticiones/f3l_graph_task_9000000001.tsv
    --html
    /home/chenfu/Gazebo/src/codex/archivos_auxiliares/html/f3l_offline_graph_task_9000000001_3d.html`;
  - `[F1L-OFFLINE-LOAD] success=true`;
  - `[F1L-OFFLINE-HTML] success=true`;
  - target `before_t=2.07112`, `after_t=0`,
    `before_rotation=0.2`, `after_rotation=0`;
  - decision `useful=true`: `initial_cost=596033.269207`,
    `final_cost=13310.040467`.
- validacion de mapa:
  - `[F1L-POST-APPLY-GLOBALMAP-CHECK] ok=true`;
  - puntos publicados `29150 -> 29150`;
  - `invalid_pose_skipped` no aumenta: `89 -> 89`.
- errores graves:
  - no hay `FATAL`, `Segmentation fault`, `process has died` ni `exit code -6`
    en el log reducido;
  - hay `invalid_pose_skipped` durante replay, pero el check final no lo
    incrementa.
- artefactos:
  - `codex/archivos_auxiliares/logs/prueba_26.log`;
  - `codex/archivos_auxiliares/logs/prueba_26.reduced.log`;
  - `codex/archivos_auxiliares/repeticiones/f3i_window_task_9000000001.tsv`;
  - `codex/archivos_auxiliares/repeticiones/f3l_graph_task_9000000001.tsv`;
  - `codex/archivos_auxiliares/html/f3l_debug_animation_task_9000000001.html`;
  - `codex/archivos_auxiliares/html/f3l_offline_graph_task_9000000001_3d.html`.
- conclusion: `PARCIAL`.
- siguiente paso recomendado:
  - inspeccionar el HTML 3D y el TSV de ventana; repetir primero offline con la
    ventana/grafo guardados cuando se ajusten pesos, y revalidar despues en live
    los casos largos `2`/`3` que antes hicieron rollback.

## 2026-07-23 17:10 — Subfase 3I — prueba live corta fiducial 2 a fiducial 1

- objetivo intentado:
  - crear y ejecutar una prueba similar al rodeo de edificio con dos fiduciales,
    pero parando al llegar a fiducial 1;
  - hacer que ambos drones anclen en fiducial 2 y recorran lados opuestos;
  - verificar errores de ambos drones en fiducial 1 y exportar HTML/TSV para no
    repetir Gazebo al ajustar grafo/solver.
- archivos modificados:
  - `codex/archivos_auxiliares/trayectorias/prueba_tipica_fiducial_2_a_1_dos_lados.yaml`;
  - `codex/contexto/pruebas_clave/pruebas_tipicas.md`;
  - `codex/contexto/paquetes/simulacion_dron/00_summary.md`;
  - `codex/contexto/paquetes/simulacion_dron/launches.md`;
  - `codex/contexto/07_ULTIMA_SESION.md`;
  - `codex/contexto/CONTEXTO_MINIMO_ACTUAL.md`;
  - `codex/contexto/01_ESTADO_ACTUAL_RESUMEN.md`;
  - `codex/pipeline/fase_3_sparse_global/pipeline_fase_3_RESUMEN.md`;
  - `codex/pipeline/fase_3_sparse_global/historial/INDEX.md`;
  - `codex/pipeline/fase_3_sparse_global/historial/por_subfase/historial_pruebas_tipicas.md`;
  - `codex/pipeline/fase_3_sparse_global/historial/por_subfase/historial_3I.md`.
- paquetes compilados:
  - no aplica; solo YAML/documentacion.
- resultado de build:
  - no ejecutado.
- pruebas Gazebo/replay:
  - live `prueba_27` con
    `codex/archivos_auxiliares/trayectorias/prueba_tipica_fiducial_2_a_1_dos_lados.yaml`;
  - replay offline de
    `codex/archivos_auxiliares/repeticiones/f3l_graph_task_2.tsv` con
    `test_opt_graph_offline --html`.
- patrones de reduccion:

  ```text
  SCENARIO-RUNNER|SIM-|F1E-FID|F1H-FID|F1I-GRAPH|F1I-WINDOW-DUMP|F1J-OPT|F1L-GRAPH-DUMP|F1L-DEBUG-ANIMATION|F1L-POST-APPLY|F1L-GT|F1M-COVIS|invalid_pose_skipped|global_map_check_failed|FATAL|ERROR|Segmentation fault|process has died|exit code -6|Killed
  ```

- evidencia positiva:
  - `SCENARIO-RUNNER-DONE scenario='prueba_tipica_fiducial_2_a_1_dos_lados' success=true`;
  - `SIM-DONE prueba=27 success=true`, `SIM-EXIT-CODE 0`;
  - `drone_1` en fiducial 1: `kf=226`, `error_t=0.158574`,
    `error_rot=0.025352`, `error_yaw=0.021771`, `decision=OK`;
  - `drone_2` en fiducial 1: `kf=203`, `error_t=28.937918`,
    `error_rot=2.908612`, `error_yaw=2.905030`,
    `decision=TASK_CREATED`, `task_id=2`;
  - grafo de `task_id=2`: `window_keyframes=130`, `vertices=44`,
    `edges=43`, `coverage_complete=true`;
  - dry-run live: target `28.937918 m -> 0`, yaw `2.905030 -> 0`,
    coste `69777420.092643 -> 512473.703446`;
  - artefactos:
    `codex/archivos_auxiliares/repeticiones/f3i_window_task_2.tsv`,
    `codex/archivos_auxiliares/repeticiones/f3l_graph_task_2.tsv`,
    `codex/archivos_auxiliares/html/f3l_debug_animation_task_2.html`,
    `codex/archivos_auxiliares/html/f3l_offline_graph_task_2_prueba_27_3d.html`;
  - replay offline: `[F1L-OFFLINE-LOAD] success=true` y
    `[F1L-OFFLINE-HTML] success=true`.
- evidencia negativa o ausente:
  - no se inspecciono visualmente RViz2 desde esta sesion; la prueba queda lista
    para abrir RViz2 en ejecuciones interactivas;
  - una tarea temprana de `drone_2` en fiducial 2 falla al construir grafo con
    `previous_fiducial_anchor_missing`;
  - el apply live de `task_id=2` se rechaza por `global_map_check_failed`,
    `invalid_pose_skipped_before=153`, `invalid_pose_skipped_after=408`;
  - el replay offline baja la media GT de ventana `7.18291 -> 5.0564`, pero
    empeora `75` KFs y el maximo queda alto (`11.059 m`);
  - aparece `gazebo ... exit code 255` durante cleanup tras `SIM-DONE`, patron
    no bloqueante.
- conclusion: `PARCIAL`.
- siguiente paso recomendado:
  - iterar offline sobre `f3l_graph_task_2.tsv` y `f3i_window_task_2.tsv`,
    revisando pesos de aristas, vecindades protegidas y propagacion antes de
    repetir `prueba_27` con RViz2 abierto.

## 2026-07-23 17:28 — Subfase 3I — cobertura equilibrada de vertices

- objetivo intentado:
  - corregir el problema observado en el HTML: demasiados vertices juntos en
    esquinas y zonas con KFs sin controles;
  - mantener esquinas 3D/SE(3) como señal relevante sin permitir que acaparen
    el cupo porcentual;
  - repetir la prueba tipica corta y comparar grafo/HTML.
- archivos modificados:
  - `orbslam3_multi/src/pose_graph_builder.cpp`;
  - `orbslam3_server/src/global_map_server.cpp`;
  - `codex/contexto/paquetes/orbslam3_multi/00_summary.md`;
  - `codex/contexto/paquetes/orbslam3_multi/pose_graph_builder.md`;
  - `codex/contexto/paquetes/orbslam3_multi/pose_graph_problem.md`;
  - `codex/contexto/paquetes/orbslam3_server/global_map_server.md`;
  - `codex/contexto/07_ULTIMA_SESION.md`;
  - `codex/contexto/CONTEXTO_MINIMO_ACTUAL.md`;
  - `codex/contexto/01_ESTADO_ACTUAL_RESUMEN.md`;
  - `codex/pipeline/fase_3_sparse_global/pipeline_fase_3_RESUMEN.md`;
  - `codex/pipeline/fase_3_sparse_global/historial/INDEX.md`;
  - `codex/pipeline/fase_3_sparse_global/historial/por_subfase/historial_3I.md`.
- paquetes compilados:
  - `orbslam3_multi`;
  - `orbslam3_server`.
- resultado de build:
  - `BUILD-EXIT-CODE 0` antes y despues de renombrar la politica de log.
- pruebas Gazebo/replay:
  - live `prueba_28` con
    `codex/archivos_auxiliares/trayectorias/prueba_tipica_fiducial_2_a_1_dos_lados.yaml`;
  - replay offline de
    `codex/archivos_auxiliares/repeticiones/f3l_graph_task_4.tsv` con
    `test_opt_graph_offline --html`.
- patrones usados para reducir logs:

  ```text
  SCENARIO-RUNNER|SIM-|F1E-FID|F1H-FID|F1I-GRAPH|F1I-WINDOW-DUMP|F1J-OPT|F1L-GRAPH-DUMP|F1L-DEBUG-ANIMATION|F1L-POST-APPLY|F1K-OPT|F1L-GT|invalid_pose_skipped|global_map_check_failed|previous_fiducial_anchor_missing|FATAL|ERROR|Segmentation fault|process has died|exit code -6|Killed
  ```

- cambio implementado:
  - la coordenada de cobertura mezcla `70 %` distancia acumulada 3D y `30 %`
    indice temporal;
  - el selector completa el cupo partiendo el mayor hueco relativo entre
    controles ya elegidos;
  - dentro de cada hueco, las esquinas 3D/SE(3) tienen bonus, pero no se
    insertan todas como obligatorias;
  - las muestras no esquina quedan etiquetadas como `balanced_coverage_sample`;
  - los logs nuevos publican `vertex_policy=balanced_coverage_sample`.
- evidencia positiva:
  - `SCENARIO-RUNNER-DONE scenario='prueba_tipica_fiducial_2_a_1_dos_lados' success=true`;
  - `SIM-DONE prueba=28 success=true`, `SIM-EXIT-CODE 0`;
  - antes, `prueba_27` tenia `36` vertices `corner_3d_vertex` en el grafo
    comparable; ahora el caso `task_id=4` tiene `5` esquinas y `15`
    `balanced_coverage_sample`;
  - `task_id=4`: `window_keyframes=85`, `vertices=26`, `edges=25`,
    `max_consecutive_id_gap=11`, `max_consecutive_distance_m=6.131`;
  - dry-run live `task_id=4`: target `10.051937 m -> 0`, yaw `0.610661 -> 0`;
  - offline `task_id=4`: `mean_before=3.96151 -> mean_after=0.653821`,
    `max_before=10.0466 -> max_after=1.68853`;
  - inspeccion visual del usuario: la optimizacion mostrada en el HTML es muy
    buena; el grafo ya no presenta el racimo de esquinas que motivó la
    correccion;
  - HTML live:
    `codex/archivos_auxiliares/html/f3l_debug_animation_task_4.html`;
  - HTML offline:
    `codex/archivos_auxiliares/html/f3l_offline_graph_task_4_prueba_28_3d.html`.
- evidencia negativa o ausente:
  - no se inspecciono RViz2 manualmente;
  - el apply live de `task_id=4` se rechaza por `global_map_check_failed`,
    `invalid_pose_skipped_before=0`, `invalid_pose_skipped_after=551`;
  - por ese rechazo, `ValidatePostApply` fuerza `REJECT_ROLLBACK`: el HTML queda
    como propuesta diagnostica correcta, pero RViz2 vuelve a ver el estado
    anterior porque la optimizacion no queda aceptada en `GlobalPoseStore`;
  - `task_id=3` conserva un salto temporal `220 -> 308`, pero la ventana
    guardada no contiene KFs intermedios entre esos IDs, asi que no es un hueco
    creado por el selector;
  - siguen apareciendo tareas tempranas con `previous_fiducial_anchor_missing`,
    deuda separada.
- conclusión: `PARCIAL`.
- siguiente paso recomendado:
  - inspeccionar visualmente `f3l_debug_animation_task_4.html`; si la
    distribucion ya es aceptable, pasar al rechazo post-apply: pesos de aristas,
    priors, vecindades protegidas, propagacion y check de mapa global.

## 2026-07-28 — Subfase 3I — cierre del grafo fiducial

- objetivo:
  cerrar la revisión de construcción del grafo después de integrar selección
  equilibrada, esquinas 3D/SE(3), vecindades fiduciales y cobertura de ventana.
- archivos modificados:
  documentación de contratos, estado, historial y handoff; no se modificó
  código en este cierre.
- build y pruebas:
  no se ejecutaron de nuevo; se usa la evidencia ya registrada de
  `prueba_28`, `prueba_31` y `prueba_41-44`.
- patrones/logs:
  consultar las entradas anteriores y los índices reducidos de cada prueba.
- evidencia acumulada:
  `prueba_28` eliminó el exceso de controles en esquinas; `prueba_31` despejó
  el falso bloqueo de publicación; `prueba_41-44` validaron aplicaciones
  secuenciales, concurrentes y KFs llegados durante el solver.
- validación visual:
  el usuario confirmó que los HTML 3D y el resultado final en RViz2 son
  correctos, incluido el caso que antes dejaba KFs aislados.
- conclusión:
  `CONSEGUIDA Y CERRADA`. Los pesos y aristas temporales pueden revisarse al
  integrar covisibilidad/loops, pero no son un bloqueo actual.
- siguiente paso:
  revalidar `3M`, después `3N` y continuar la regresión integrada de `3O`.

## 2026-08-14 - Subfase 3I - Grafo temporal reimplementado

- objetivo intentado: construir ventana mono-submapa desde el ultimo control,
  con ratio 30 %, vecindades 20 % y aristas temporales SE(3).
- archivos modificados: `pose_graph_problem.hpp`, `pose_graph_builder.hpp/.cpp`,
  backend, tests y parametros launch.
- pruebas: test de grafo 4/4; replay 144; live 145; replay v3 146.
- evidencia: replay 144 construye 10 grafos sin fallo; live/replay v3 construyen
  30 grafos aceptados, con ventana maxima de 128 KFs. El primer control queda
  fijo y cada target alcanza error cero.
- intentos fallidos conservados: prueba 142 produjo
  `target_not_after_control`; prueba 143 produjo `world_pose_missing`. La causa
  fue orden legacy v2 e intermedios invalidados, no la seleccion 30/20.
- correccion: inferencia v2 ordenada, observaciones intra-arrival ordenadas y
  omision de intermedios inactivos manteniendo el hard control como frontera.
- conclusion: `PARCIAL`; algoritmo y ejecucion tecnica conseguidos, pendiente
  confirmacion visual de la prueba 145.
- siguiente paso recomendado: cerrar tras observacion del grafo/RViz2.
