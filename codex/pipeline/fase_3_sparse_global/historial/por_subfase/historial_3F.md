# Historial 3F

> Extraido mecanicamente de `historial_fase_3.md`. Leer este archivo antes de abrir otros historiales de subfase.

## 2026-07-08 — Subfase 3F — `GlobalMapBuilder`, `LandmarkScoreManager` y `/global_sparse_cloud`

- objetivo intentado: crear una salida sparse global publicable en `world`, con score inicial por punto y usando solo submapas anclados por `GlobalPoseStore`.
- archivos modificados:
  - `orbslam3_multi/include/orbslam3_multi/global_sparse_point.hpp`;
  - `orbslam3_multi/include/orbslam3_multi/landmark_score_manager.hpp`;
  - `orbslam3_multi/src/landmark_score_manager.cpp`;
  - `orbslam3_multi/include/orbslam3_multi/global_map_builder.hpp`;
  - `orbslam3_multi/src/global_map_builder.cpp`;
  - `orbslam3_multi/include/orbslam3_multi/global_pose_store.hpp`;
  - `orbslam3_multi/src/global_pose_store.cpp`;
  - `orbslam3_multi/CMakeLists.txt`;
  - `orbslam3_server/src/global_map_server.cpp`;
  - `orbslam3_server/CMakeLists.txt`;
  - `orbslam3_server/launch/global_orb_map_server.launch.py`;
  - `codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml`;
  - `codex/archivos_auxiliares/trayectorias/tray_prueba_2.yaml`;
  - documentación de contexto, paquetes, pruebas clave y pipeline relacionada con `3F`/`3G`.
- cambios realizados:
  - se crea `GlobalSparsePoint` como tipo de salida publicable;
  - se crea `LandmarkScoreManager` con eventos semánticos y score inicial raw:
    `0.55 * min(observations_count/8,1) + 0.35 * found_ratio + 0.10 * descriptor_valid`, con `is_bad => 0`;
  - se crea `GlobalMapBuilder`, que consulta `RawMapDatabase`, `GlobalPoseStore` y `LandmarkScoreManager`;
  - `GlobalPoseStore` expone `GetSubmapWorldTransform` para consumidores de solo lectura;
  - `global_map_server` actualiza scores al recibir deltas/replay, construye `PointCloud2` y publica `/global_sparse_cloud`;
  - el `PointCloud2` usa frame `world` y campos `x`, `y`, `z`, `score`, `drone_id` y `map_epoch`;
  - `global_orb_map_server.launch.py` expone `global_sparse_cloud_topic`, `global_map_min_score_to_publish` y `global_map_publish_period_sec`;
  - `tray_prueba_1.yaml` queda como prueba live con recorrido ida/vuelta al fiducial;
  - `tray_prueba_2.yaml` queda como replay lento a `1.0` s por delta.
- paquetes compilados:
  ```bash
  ./codex/herramientas/build_selected_packages.sh orbslam3_multi
  ./codex/herramientas/build_selected_packages.sh orbslam3_server
  ./codex/herramientas/build_selected_packages.sh orbslam3_multi orbslam3_server
  ```
- resultado de build:
  - `orbslam3_multi`: `BUILD-EXIT-CODE 0`;
  - `orbslam3_server`: `BUILD-EXIT-CODE 0`;
  - build final conjunto: `2 packages finished`, `BUILD-EXIT-CODE 0`;
  - no hizo falta ejecutar `reduce_build_log.sh`.
- pruebas Gazebo/replay ejecutadas:
  - prueba `1`: Gazebo live con launch oficial `ros2 launch simulacion_dron multi_dron.launch.py`;
  - prueba `2`: replay lento sin Gazebo visual usando `ros2 launch orbslam3_server global_orb_map_server.launch.py` con `rawdb_replay_period_sec:=1.0`, `fiducial_sim_enabled:=false`, `pose_store_debug_enabled:=false` y `global_map_publish_period_sec:=1.0`.
- patrones usados para reducir logs:
  - prueba 1:
    ```text
    SCENARIO-RUNNER|GOAL|RESULT|success|F1C-RAWDB|F1D-POSESTORE|F1E-FID|F1F-SCORE|F1F-GLOBALMAP|POINTCLOUD|global_sparse_cloud|ERROR|FATAL|Segmentation fault|Killed
    ```
  - prueba 2:
    ```text
    SCENARIO-RUNNER|GOAL|RESULT|success|F1C-REPLAY|F1C-RAWDB|F1D-POSESTORE|F1E-FID|F1E-FID-REPLAY|F1F-SCORE|F1F-GLOBALMAP|POINTCLOUD|global_sparse_cloud|ERROR|FATAL|Segmentation fault|Killed
    ```
- evidencia positiva encontrada:
  - `prueba_1`: `SIM-DONE prueba=1 success=true`;
  - `prueba_1`: `scenario_runner_node` terminó con `success=true`;
  - `prueba_1`: se guardó `.record` con `318` deltas, `72` observaciones fiduciales, `3` submapas, `179` KFs y `22885` MPs;
  - `prueba_1`: `[F1F-SCORE-UPDATE-ORBSLAM]` y `[F1F-SCORE-STATS]` aparecen durante la ingesta live;
  - `prueba_1`: antes de anclar no se publican puntos globales, con `anchored_submaps=0` y `returned_points=0`;
  - `prueba_1`: tras anclar aparecen publicaciones en `/global_sparse_cloud` con `points_published=22394`;
  - `prueba_2`: `SIM-DONE prueba=2 success=true`;
  - `prueba_2`: `[F1C-REPLAY-LOAD] ... entries=318 deltas=318 ... fiducial_observations=72`;
  - `prueba_2`: `[F1C-REPLAY-DONE] entries=318 journal=318 deltas=318 full=0 fiducial_observations=72 submaps=3 kfs=179 mps=22885`;
  - `prueba_2`: `[F1F-GLOBALMAP-PUBLISH] reason=timer topic=/global_sparse_cloud frame_id=world points_from_backend=22394 points_published=22394 min_score_to_publish=0.000 score_field=true drone_id_field=true map_epoch_field=true`;
  - en ambas pruebas el builder informa `anchored_submaps=2` y `skipped_unanchored=1`, por lo que no publica el submapa sin anchor.
- evidencia negativa o ausente:
  - no se realizó inspección manual en RViz2; queda lista para revisión del usuario;
  - no hay fusión real de landmarks en `3F`, por lo que `is_fused=false` y pueden existir duplicados raw entre submapas anclados;
  - no hay full snapshots, loops, optimización ni correcciones publicadas todavía;
  - en `prueba_1` aparece `gazebo ... exit code 255` durante el cierre posterior a `SIM-DONE success=true`, patrón no bloqueante ya observado;
  - no aparecen `FATAL`, `Segmentation fault` ni `Killed`.
- documentación actualizada:
  - `codex/contexto/00_LEER_PRIMERO.md`;
  - `codex/contexto/01_ESTADO_ACTUAL.md`;
  - `codex/contexto/04_TOPICS_SERVICES_ACTIONS.md`;
  - `codex/contexto/06_MAPA_CODIGO.md`;
  - `codex/contexto/07_ULTIMA_SESION.md`;
  - `codex/contexto/paquetes/orbslam3_multi/orbslam3_multi.md`;
  - `codex/contexto/paquetes/orbslam3_multi/global_pose_store.md`;
  - `codex/contexto/paquetes/orbslam3_multi/landmark_score_manager.md`;
  - `codex/contexto/paquetes/orbslam3_multi/global_map_builder.md`;
  - `codex/contexto/paquetes/orbslam3_server/orbslam3_server.md`;
  - `codex/contexto/paquetes/orbslam3_server/global_map_server.md`;
  - `codex/contexto/paquetes/orbslam3_server/launches.md`;
  - `codex/contexto/pruebas_clave/fase_3F_global_sparse_cloud.md`;
  - `codex/pipeline/fase_3_sparse_global/pipeline_fase_3.md`;
  - `codex/pipeline/fase_3_sparse_global/subfases/subfase_3F.md`;
  - `codex/pipeline/fase_3_sparse_global/subfases/subfase_3G.md`.
- conclusión: `CONSEGUIDA`.
- siguiente paso recomendado: ejecutar `subfase_3G.md` para pedir full snapshots de ORB-SLAM3 y reconciliarlos con `RawMapDatabase`/`GlobalPoseStore` sin destruir poses globales ancladas.

## 2026-07-08 — Hotfix Subfase 3F — `body_T_camera` para `/global_sparse_cloud`

- objetivo intentado: corregir la nube sparse global observada en RViz2 porque los anchors fiduciales estaban usando directamente la pose world del cuerpo del dron como pose de cámara/KF.
- diagnóstico:
  - el launch legacy ya contenía parámetros `body_T_camera_*`;
  - la ruta nueva `3E/3F` guardaba y reproducía observaciones fiduciales con pose world del cuerpo;
  - `FiducialAnchorManager` calculaba `world_T_local` sin pasar por la extrínseca cuerpo-cámara.
- archivos modificados:
  - `orbslam3_multi/include/orbslam3_multi/global_pose_store.hpp`;
  - `orbslam3_multi/src/global_pose_store.cpp`;
  - `orbslam3_multi/include/orbslam3_multi/fiducial_anchor_manager.hpp`;
  - `orbslam3_multi/src/fiducial_anchor_manager.cpp`;
  - `orbslam3_server/src/global_map_server.cpp`;
  - `orbslam3_server/launch/global_orb_map_server.launch.py`;
  - documentación de contexto, paquetes, prueba clave y subfase `3F`.
- cambios realizados:
  - `GlobalPoseStore` incorpora `BodyCameraTransformConfig`, `ConfigureBodyCameraTransform`, `GetBodyCameraTransform`, `TransformBodyPoseToCameraPose` y `GetBodyCameraTransformConfig`;
  - `body_T_camera` queda parametrizado con defaults `0.10/0.03/0.03`, `0/-90/90` y `use_camera_optical_frame_convention=true`;
  - `FiducialObservation` pasa a representar explícitamente `world_T_body_fiducial`;
  - `FiducialAnchorManager` calcula `world_T_camera_fiducial = world_T_body_fiducial * body_T_camera` antes de `world_T_local`;
  - el servidor declara `body_T_camera_*`, los pasa al backend y emite `[F1F-BODY-CAMERA-CONFIG]`;
  - el servidor loggea `[F1F-BODY-CAMERA-APPLY]` con `body_t` y `camera_t`;
  - el `.record` de la prueba diferencial se conserva renombrado como `codex/archivos_auxiliares/repeticiones/prueba_diff_anclaje.record`.
- paquetes compilados:
  ```bash
  ./codex/herramientas/build_selected_packages.sh orbslam3_multi
  ./codex/herramientas/build_selected_packages.sh orbslam3_server
  ```
- resultado de build:
  - `orbslam3_multi`: `BUILD-EXIT-CODE 0`;
  - `orbslam3_server`: `BUILD-EXIT-CODE 0`;
  - no hizo falta ejecutar `reduce_build_log.sh`.
- pruebas ejecutadas:
  - replay lento `prueba_2` con `rawdb_replay_period_sec:=1.0`, `fiducial_sim_enabled:=false`, `pose_store_debug_enabled:=false` y `global_map_publish_period_sec:=1.0`;
  - durante la ejecución el path usado fue `src/codex/archivos_auxiliares/repeticiones/rawdb_prueba_1.record`;
  - tras validar, ese archivo se renombró a `src/codex/archivos_auxiliares/repeticiones/prueba_diff_anclaje.record` para conservarlo sin duplicar espacio.
- patrones usados para reducir logs:
  ```text
  SCENARIO-RUNNER|GOAL|RESULT|success|F1C-REPLAY|F1C-RAWDB|F1D-POSESTORE|F1E-FID|F1E-FID-REPLAY|F1F-BODY-CAMERA|F1F-SCORE|F1F-GLOBALMAP|POINTCLOUD|global_sparse_cloud|ERROR|FATAL|Segmentation fault|Killed
  ```
- evidencia positiva encontrada:
  - `SIM-DONE prueba=2 success=true`;
  - `SCENARIO-RUNNER-DONE scenario='prueba_1f_global_sparse_cloud_replay_lento' success=true`;
  - `[F1F-BODY-CAMERA-CONFIG] source=launch use_optical=true body_T_camera_t=(0.100,0.030,0.030) rpy_deg=(0.000,-90.000,90.000)`;
  - `[F1F-BODY-CAMERA-APPLY]` aparece para observaciones fiduciales y muestra `body_t` distinto de `camera_t`;
  - `[F1C-REPLAY-DONE] entries=318 journal=318 deltas=318 full=0 fiducial_observations=72 submaps=3 kfs=179 mps=22885`;
  - `[F1F-GLOBALMAP-PUBLISH] ... topic=/global_sparse_cloud frame_id=world points_from_backend=22394 points_published=22394 ...`;
  - no aparecieron `FATAL`, `Segmentation fault` ni `Killed`.
- evidencia negativa o ausente:
  - la validación automática no decide si visualmente ya desapareció todo el problema; esa confirmación queda en manos del usuario en RViz2;
  - siguen siendo esperables doble pared, deriva entre anchors y ruido porque `3F` aún no optimiza ni fusiona landmarks.
- conclusión: `CONSEGUIDA`.
- siguiente paso recomendado: si la inspección visual del usuario confirma la nube corregida, pasar a `subfase_3G.md`; si no, revisar primero la convención exacta de `body_T_camera` contra el corrector legacy y el frame óptico del wrapper.

## 2026-08-12 14:49 - Subfase 3F - Reimplementacion incremental

- objetivo intentado: implementar score ORB autoritativo, builder stateful,
  cloud/KFs coherentes y topologia web 3F conforme al contrato cerrado;
- archivos principales: nuevos `landmark_score_manager.*`,
  `global_map_builder.*` y tests; integracion en `sparse_global_backend.*` y
  `global_map_server.cpp`; CMake/package, launch, RViz, web y escenario 3F;
- comportamiento: dirty sets raw/pose/score, asociacion KF estable,
  reproyeccion `world_T_kf * inverse(local_T_kf) * p_local`, sin fallback de
  submapa; publicacion serial `PointCloud2` + `MarkerArray` al final de
  `PrimaryTask`;
- build intento inicial: fallo mecanico al inicializar agregados `PointField`;
  se introdujo `MakePointField` explicito y el build conjunto termino exit 0;
- tests intento inicial: score 1/2 y builder 2/3; se corrigio la expectativa de
  formula a 0.69, se conservo `is_bad` con score cero y se hizo que la metrica
  cuente reproyecciones aunque el resultado geometrico coincida;
- build final: tres paquetes, exit 0, solo aviso Drake preexistente;
- tests finales: 26/26 C++ y 8/8 web;
- conclusion: `CONSEGUIDA` para implementacion estatica/unitaria; pendiente
  validacion integrada replay/live.

## 2026-08-12 14:51 - Subfase 3F - Replay 94 no ejercitado

- objetivo intentado: reproducir el record 3E a 100 ms con RViz2 y web;
- prueba: `f3f_replay.launch.py`, `rawdb_prueba_3e.record`,
  `tray_prueba_92.yaml`, startup 5 s, timeout 60 s y post-wait 5 s;
- resultado: herramienta exit 1 antes de iniciar el scenario; agoto dos
  reintentos al interpretar la ausencia deliberada de Gazebo como caida;
- diagnostico reducido: bridge/helper/servidor si arrancaron; RViz2 murio al
  cargar `libpthread` desde `/snap/core20` heredada por `GTK_PATH`;
- correccion: opcion explicita `--without-gazebo` y entorno RViz saneado en el
  launch de replay; sintaxis y rebuild `simulacion_dron` correctos;
- conclusion: `NO CONSEGUIDA`; no valida 3F y nunca se reescribe como pase.

## 2026-08-12 14:54 - Subfase 3F - Replay 95 valido

- objetivo intentado: repetir el replay tras las dos correcciones mecanicas;
- prueba: mismo record/YAML/retardo, ahora con `--without-gazebo`;
- resultado: scenario exit 0, `SIM-DONE success=true`, herramienta exit 0;
- reduccion: marcadores 3B/3E/3F, replay, worker, errores y cierre;
- evidencia positiva: bridge 11/15 y pestaña abierta; 150 entradas,
  `max_active=1`, 2 anchors, 61 poses/KFs activas, 2 hard, 8450 scores y 5812
  puntos; 49 skips antes del primer mapa publico y 101 publicaciones;
- invariantes: frame `world`, campos score/RGB/identidad y
  `fallback_submap=0`; cero errores runtime y RViz2 estable;
- conclusion: `CONSEGUIDA` tecnicamente para replay.

## 2026-08-12 14:56 - Subfase 3F - Live 96 valido

- objetivo intentado: validar Gazebo/RViz2/web integrados con dos drones,
  anchors en fiducial 2, espera de backfill y avance a x=-8;
- prueba: `multi_dron.launch.py`, `tray_prueba_93.yaml`, startup 15 s, timeout
  360 s y post-wait 10 s;
- resultado: ambos pares de goals terminaron `success=true`, las esperas de 12
  y 30 s se completaron, `SIM-DONE success=true` y herramienta exit 0;
- evidencia positiva: first anchors `(1,0)/kf21` y `(2,0)/kf27`; 28 skips
  pre-anchor; 156 publicaciones desde 22 KFs/2157 puntos hasta 126 KFs/13191
  puntos; frame/campos correctos, `fallback_submap=0` y cero errores runtime;
- cierre: el cleanup posterior a `SIM-DONE` escalo SIGINT/SIGTERM/SIGKILL para
  terminar el grupo; no invalida el escenario ya completado;
- evidencia pendiente: observacion del usuario de colores/contenido RViz2 y
  pulsos del grafo web;
- conclusion: `PARCIAL`; validacion automatica conseguida, cierre visual
  pendiente.

## 2026-08-12 15:51 - Subfase 3F - Repeticion visual 97

- objetivo intentado: repetir exclusivamente el live para que el usuario
  pudiera observar con mas calma RViz2 y el grafo web;
- prueba: mismo `multi_dron.launch.py` y `tray_prueba_93.yaml` de 3F;
- resultado operativo: scenario exit 0, `SIM-DONE success=true` y herramienta
  exit 0; por peticion expresa no se redujo ni analizo el log;
- observacion posterior del usuario: la disposicion web no es aceptable; en
  particular `GlobalMapBuilder -> GlobalMapServer` atraviesa el centro del
  grafo por debajo de vertices y otras aristas;
- correccion visual autorizada: compactar posiciones en tres carriles, enrutar
  ese retorno por un carril inferior exterior, separar las dos curvas hacia
  RViz2 y elevar aristas activas sin superar el z-order de los nodos;
- segunda correccion y valoracion: se elimino el `taxi`, se adopto la
  composicion manual propuesta por el usuario y se reabrio solo el grafo con
  URL fresh; el usuario responde `Perfecto` y pide concluir 3F;
- conclusion revisada de la prueba: `CONSEGUIDA`; runtime, RViz2 y grafo quedan
  aceptados, conservando documentado que la primera disposicion visual fallo.

## 2026-08-14 - Subfase 3F - Revision cruzada de live 145

- objetivo revisado: comprobar el criterio 14 de color estable y distinguible
  por submapa usando la ejecucion 145, que genero 7 submapas reales;
- evidencia conservada: ambos drones avanzaron de epoch, con estados finales
  visibles `(1,3)` y `(2,2)`;
- observacion del usuario: los KFs parecian coloreados por dron, no por
  submapa, porque un nuevo epoch no mostraba un cambio reconocible;
- diagnostico: `SubmapColor()` si recibe `(drone_id,map_epoch)`, pero el XOR de
  un epoch pequeno solo mueve unos pocos grados el hue. `(1,0)/(1,3)` quedan
  en 205/206 grados y `(2,0)/(2,2)` en 50/48;
- conclusion: `PARCIAL`; la prueba 97 no se reescribe ni su layout deja de ser
  valido, pero el criterio de color por submapa no esta conseguido ante resets;
- siguiente paso recomendado: usar una paleta discreta o hash con mezcla real
  del par completo y añadir un test de separacion perceptual entre epochs.

## 2026-08-14 - Subfase 3F - Color perceptual por submapa

- objetivo intentado: hacer que un nuevo `map_epoch` cambie claramente el
  color de los frustums aunque pertenezca al mismo dron;
- archivos modificados: nuevo `submap_color.hpp`, uso desde
  `global_map_server.cpp`, CMake y `test_submap_color.cpp`;
- implementacion: base determinista por dron y salto de hue de 137.507764
  grados por epoch, con alpha estable;
- build/tests: tres paquetes exit 0; 2/2 tests de color dentro de 49/49 C++,
  exigiendo distancia RGB minima entre epochs consecutivos y entre drones;
- live 151: escenario completo con 6 submapas, epochs 0/1/2 de ambos drones;
- conclusion: `PARCIAL`; implementacion y cobertura automatica conseguidas,
  pendiente confirmacion visual del usuario en RViz2.
