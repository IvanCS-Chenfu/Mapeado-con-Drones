# Historial 3E

> Extraido mecanicamente de `historial_fase_3.md`. Leer este archivo antes de abrir otros historiales de subfase.

## 2026-07-08 — Subfase 3E — `FiducialAnchorManager` y record fiducial

- objetivo intentado: crear en `orbslam3_multi` una capa backend `FiducialAnchorManager` que reciba observaciones fiduciales ya asociadas a KeyFrames, calcule `world_T_local`, ancle submapas en `GlobalPoseStore`, marque KFs hard fiducial y persista esas observaciones en `.record` para replay sin GT vivo.
- acción previa solicitada por el usuario:
  - se borró `codex/archivos_auxiliares/repeticiones/rawdb_prueba_1.record` anterior para ahorrar espacio;
  - se generó un nuevo `.record` versión 2 desde la simulación viva de `3E`.
- archivos modificados:
  - `orbslam3_multi/include/orbslam3_multi/raw_map_types.hpp`;
  - `orbslam3_multi/include/orbslam3_multi/raw_map_database.hpp`;
  - `orbslam3_multi/src/raw_map_database.cpp`;
  - `orbslam3_multi/include/orbslam3_multi/global_pose_store.hpp`;
  - `orbslam3_multi/src/global_pose_store.cpp`;
  - `orbslam3_multi/include/orbslam3_multi/fiducial_anchor_manager.hpp`;
  - `orbslam3_multi/src/fiducial_anchor_manager.cpp`;
  - `orbslam3_multi/CMakeLists.txt`;
  - `orbslam3_server/src/global_map_server.cpp`;
  - `orbslam3_server/CMakeLists.txt`;
  - `orbslam3_server/launch/global_orb_map_server.launch.py`;
  - `codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml`;
  - `codex/archivos_auxiliares/trayectorias/tray_prueba_2.yaml`;
  - documentación de contexto, paquetes, pruebas clave y pipeline relacionada con `3E`/`3F`.
- cambios realizados:
  - `RawMapDatabase` pasa a guardar `.record` versión 2, compatible con versión 1, añadiendo `fiducial_observation_journal`;
  - se añade `RecordedFiducialObservation` con `arrival_id`, identidad de KF, `fiducial_id`, pose world del KF, timestamps, distancia y fuente;
  - `GlobalPoseStore` añade marca/consulta de KFs hard fiducial y estadísticas;
  - se crea `FiducialAnchorManager` sin dependencias ROS/Gazebo;
  - `global_map_server` añade subscripción a `/dron_X/sensor/GT/pose` solo en live con `fiducial_sim_enabled=true`;
  - el servidor asocia KFs a fiducial por timestamp y radio, llama al backend, guarda el journal fiducial y reproduce observaciones persistidas en replay;
  - `global_orb_map_server.launch.py` expone parámetros `fiducial_sim_enabled`, `fiducial_gt_max_dt_sec`, `fiducial_gt_buffer_max_samples` y configuración del fiducial 2.
- paquetes compilados:
  ```bash
  ./codex/herramientas/build_selected_packages.sh orbslam3_multi
  ./codex/herramientas/build_selected_packages.sh orbslam3_server
  ```
- resultado de build:
  - `orbslam3_multi`: `BUILD-EXIT-CODE 0`;
  - `orbslam3_server`: `BUILD-EXIT-CODE 0`;
  - no se compiló `simulacion_dron` porque solo se tocaron YAMLs auxiliares, no código/launch del paquete.
- pruebas Gazebo/replay ejecutadas:
  - prueba `1`: Gazebo live con launch oficial `ros2 launch simulacion_dron multi_dron.launch.py`;
  - prueba `2`: replay sin Gazebo visual usando `ros2 launch orbslam3_server global_orb_map_server.launch.py` con `rawdb_replay_enabled:=true`, `fiducial_sim_enabled:=false` y `pose_store_debug_enabled:=false`.
- patrones usados para reducir logs:
  - prueba 1:
    ```text
    SCENARIO-RUNNER|GOAL|RESULT|success|F1B-|F1C-|F1D-|F1E-|FID|POSESTORE|RAWDB|ORBMAP|WRAPPER|ERROR|FATAL|Segmentation fault|Killed
    ```
  - prueba 2:
    ```text
    SCENARIO-RUNNER|GOAL|RESULT|success|F1C-REPLAY|F1E-FID-REPLAY|F1E-FID|F1D-POSESTORE|RAWDB|ERROR|FATAL|Segmentation fault|Killed
    ```
- evidencia positiva encontrada:
  - `prueba_1`: `SIM-DONE prueba=1 success=true`;
  - `prueba_1`: `scenario_runner_node` terminó con `success=true`;
  - `prueba_1`: aparecen `[F1E-FID-KF-ASSOC]`, `[F1E-FID-OBS]`, `[F1E-FID-FIRST-ANCHOR]`, `[F1E-FID-WORLD-T-LOCAL]`, `[F1E-FID-KF-HARD]` y `[F1E-FID-JOURNAL-SAVE]`;
  - `prueba_1`: `[F1C-RAWDB-SAVE] reason=shutdown ... journal=364 deltas=364 full=0 fiducial_observations=60 submaps=3 kfs=175 mps=21095`;
  - `codex/archivos_auxiliares/repeticiones/rawdb_prueba_1.record` nuevo mide `398239640` bytes;
  - `prueba_2`: `SIM-DONE prueba=2 success=true`;
  - `prueba_2`: `[F1C-REPLAY-LOAD] ... fiducial_observations=60`;
  - `prueba_2`: `60` eventos `[F1E-FID-REPLAY-OBS]`;
  - `prueba_2`: `[F1E-FID-STATS] reason=REPLAY_RECORDED_FIDUCIAL observations=60 accepted=60 rejected=0 anchors_created=2 replay_observations=60 hard_fiducial_kfs=60`;
  - `prueba_2`: `[F1C-REPLAY-DONE] entries=364 journal=364 deltas=364 full=0 fiducial_observations=60 submaps=3 kfs=175 mps=21095`.
- evidencia negativa o ausente:
  - no se publica nube global visible todavía; eso queda para `3F`;
  - no hay segunda visita fiducial con optimización; queda para `3H`;
  - no hay loops, fusión real de landmarks ni score;
  - en `prueba_1` aparece `gazebo ... exit code 255` durante el cierre posterior a `SIM-DONE success=true`, patrón no bloqueante ya observado;
  - tras el `.record` nuevo el disco queda con margen muy bajo, aproximadamente `397M`, por lo que conviene liberar espacio antes de repetir simulaciones largas.
- documentación actualizada:
  - `codex/contexto/00_LEER_PRIMERO.md`;
  - `codex/contexto/01_ESTADO_ACTUAL.md`;
  - `codex/contexto/06_MAPA_CODIGO.md`;
  - `codex/contexto/07_ULTIMA_SESION.md`;
  - `codex/contexto/paquetes/orbslam3_multi/orbslam3_multi.md`;
  - `codex/contexto/paquetes/orbslam3_multi/raw_map_database.md`;
  - `codex/contexto/paquetes/orbslam3_multi/global_pose_store.md`;
  - `codex/contexto/paquetes/orbslam3_multi/fiducial_anchor_manager.md`;
  - `codex/contexto/paquetes/orbslam3_server/orbslam3_server.md`;
  - `codex/contexto/paquetes/orbslam3_server/global_map_server.md`;
  - `codex/contexto/paquetes/orbslam3_server/launches.md`;
  - `codex/contexto/pruebas_clave/fase_3C_rawdb_replay.md`;
  - `codex/contexto/pruebas_clave/fase_3E_fiducial_record_replay.md`;
  - `codex/pipeline/fase_3_sparse_global/pipeline_fase_3.md`;
  - `codex/pipeline/fase_3_sparse_global/subfases/subfase_3E.md`;
  - `codex/pipeline/fase_3_sparse_global/subfases/subfase_3F.md`.
- conclusión: `CONSEGUIDA`.
- siguiente paso recomendado: ejecutar `subfase_3F.md` para crear `LandmarkScoreManager` y `GlobalMapBuilder`, publicando la primera nube sparse global en `world` usando solo submapas anclados.

## 2026-08-12 13:04 - Subfase 3E - Reimplementación y tests focales

- objetivo intentado: reconstruir 3E sobre el runtime 3D sin reutilizar el
  acoplamiento de la implementación histórica.
- archivos modificados: tipos, base raw, pose store, manager y backend de
  `orbslam3_multi`; ring GT, servidor, launch y tests de `orbslam3_server`;
  launch live/replay, grafo web, tests y escenarios 91/92 de
  `simulacion_dron`; documentación activa.
- cambios principales: record v2 con observaciones normalizadas; manager puro;
  primer anchor atómico; un hard KF; ring 50; executor multihilo; replay sin GT;
  topología web 8/8.
- paquetes compilados: `orbslam3_multi orbslam3_server simulacion_dron`.
- resultado de build: rebuild final 13:10:02-13:10:15, exit 0, tres paquetes
  finalizados.
- tests: 14 casos en `orbslam3_multi`, 7 en `orbslam3_server` y 8 en
  `simulacion_dron`; total 29/29.
- evidencia negativa: el `colcon test` agregado también ejecutó linters sobre
  `legacy2` y archivos históricos, produciendo fallos de estilo ajenos al
  runtime activo. Las pruebas funcionales focales no fallaron.
- conclusión: `CONSEGUIDA` para implementación, build y tests focales.
- siguiente paso recomendado: ejecutar live y replay acordados.

## 2026-08-12 13:12 - Subfase 3E - Prueba 91 intento 1

- objetivo intentado: abrir Gazebo, RViz2 y web; llevar ambos drones al
  fiducial 2 y registrar el dataset v2.
- prueba: `tray_prueba_91.yaml`, startup 15 s, timeout global 360 s, goals
  simultáneos y espera visual de 35 s.
- patrones de reducción: `F3E`, commits raw/pose, record, runner, `SIM-*`,
  errores, Gazebo, web y RViz2.
- evidencia positiva: servidor, GT, wrappers, RViz2 y bridge arrancaron; el
  servidor cerró limpio y guardó el estado parcial.
- evidencia negativa: Gazebo murió con exit 255 antes de enviar los goals. Los
  action servers recibieron después los goals sin física y agotaron 240 s. El
  cierre registró 0 observaciones/anchors y un record sin fiduciales.
- artefactos preservados: `prueba_91_intento_1.log` y
  `prueba_91_intento_1.reduced.log`.
- conclusión: `NO CONSEGUIDA`; no aporta cobertura funcional de 3E. La causa es
  la caída temprana de Gazebo, no el pipeline fiducial.
- siguiente paso recomendado: repetir mecánicamente con la misma configuración.

## 2026-08-12 13:18 - Subfase 3E - Prueba 91 intento 2

- objetivo intentado: repetir exactamente la validación live acordada.
- prueba: misma trayectoria, launch, parámetros, tiempos y record que el intento
  anterior.
- patrones de reducción: `F3E`, `F3C-RAW-COMMIT`, `F3D-POSE-*`, record,
  runner, `SIM-*`, errores, Gazebo, web y topics globales.
- evidencia positiva:
  - ambos goals terminaron `success=true` en 22 s y el escenario completó la
    espera de 35 s;
  - web lista `topology=8_nodes_8_edges`;
  - primer anchor `(2,1)/kf18`, fiducial 2, 17 poses creadas;
  - primer anchor `(1,0)/kf23`, fiducial 2, 24 poses creadas;
  - cierre: 22 observaciones, 2 anchors, 61 poses activas, 2 hard y 20
    observaciones posteriores diferidas;
  - record v2: 150 entradas, 3 submapas, 62 KFs, 8450 MPs y 22 observaciones.
- evidencia visual revisada tras hablar con el usuario:
  - vio activarse `GlobalMapServer -> FiducialAnchorManager` como observación
    fiducial;
  - no vio activarse `FiducialAnchorManager -> GlobalPoseStore` como `first
    anchor`, por lo que visualmente no pudo confirmar los dos anchors ni el
    poblado de poses;
  - los logs conservan dos commits `status=applied`: `(2,1)/kf18` creó 17 poses
    y `(1,0)/kf23` creó 24; el cierre confirma 2 anchors, 61 poses activas y 2
    hard;
  - el servidor llama a `EmitFlowEvent(..., "first_anchor_commit", ...)`
    inmediatamente antes de cada marcador de anchor. El frontend mantiene un
    pulso solo 240 ms y una lista de 16 eventos; hubo solo dos pulsos first
    anchor frente a 22 pulsos de observación, por lo que el commit pudo ser
    imperceptible o salir pronto del historial visual;
  - no existe evidencia de que Chrome recibiera/renderizara esos dos eventos,
    solo de que el servidor los publicó.
- otra evidencia: Gazebo 255 aparece después de `SIM-DONE`, durante cleanup, y
  no invalida la prueba. El código activo no crea publishers globales.
- log reducido: `codex/archivos_auxiliares/logs/prueba_91.reduced.log`.
- revisión conversada: el usuario considera posible que no percibiera el pulso
  breve y decide explícitamente no modificar el visualizador. Acepta la
  evidencia de backend/replay y da 3E por concluida; volverá a observar la
  arista durante la prueba de la siguiente subfase.
- conclusión revisada: `CONSEGUIDA`.
- siguiente paso recomendado: preparar 3F sin cambiar la telemetría 3E.

## 2026-08-12 13:21 - Subfase 3E - Prueba 92 replay

- objetivo intentado: reproducir el dataset live sin Gazebo, wrappers ni GT en
  directo.
- prueba: `f3e_replay.launch.py` sobre `rawdb_prueba_3e.record`, escenario de
  espera 30 s, timeout 90 s.
- patrones de reducción: `F3C-REPLAY`, `F3E`, commits raw/pose, GT subscribed,
  web, runner, `SIM-*` y errores.
- evidencia positiva:
  - carga 150 entradas y 22 observaciones;
  - reproduce los mismos KFs hard y los mismos `world_T_local`;
  - cierre: 22 observaciones, 2 anchors, 61 poses, 2 hard y 20 diferidas;
  - `F3C-REPLAY-DONE` informa 150 entradas y `max_active=1`;
  - `F3E-REPLAY-FID-DONE` informa `gt_live_subscriptions=0`;
  - escenario y herramienta terminan `success=true`, exit 0.
- interpretación de `journal=0` al cierre: correcta; replay consume el journal
  persistido con `append_to_journal=false` para no duplicarlo.
- log reducido: `codex/archivos_auxiliares/logs/prueba_92.reduced.log`.
- conclusión: `CONSEGUIDA` para equivalencia record/replay.
- siguiente paso recomendado: cerrar la conclusión agregada cuando el usuario
  confirme RViz2 y grafo de la prueba 91 intento 2.
