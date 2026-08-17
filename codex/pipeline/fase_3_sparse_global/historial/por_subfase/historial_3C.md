# Historial 3C

> Extraido mecanicamente de `historial_fase_3.md`. Leer este archivo antes de abrir otros historiales de subfase.

## 2026-07-06 12:46 — Subfase 3C

- objetivo intentado: crear `RawMapDatabase` en `orbslam3_multi`, conectar el servidor mínimo, asignar `arrival_id` a cada `OrbMap`, guardar estado raw/journal y validar replay sin Gazebo.
- cambios realizados:
  - `orbslam3_multi/include/orbslam3_multi/raw_map_types.hpp`: tipos raw por `(drone_id, map_epoch)`, IDs de KF/MP y estadísticas.
  - `orbslam3_multi/include/orbslam3_multi/raw_map_database.hpp`: interfaz de `RawMapDatabase`.
  - `orbslam3_multi/src/raw_map_database.cpp`: inserción de deltas/snapshots, journal ordenado, `SaveToPath` y `LoadFromPath`.
  - `orbslam3_multi/CMakeLists.txt`: añade `src/raw_map_database.cpp`.
  - `orbslam3_server/src/global_map_server.cpp`: inserta deltas en `RawMapDatabase`, emite logs `[F1C-*]`, guarda rawdb y reproduce journal por timer.
  - `orbslam3_server/CMakeLists.txt`: enlaza `global_map_server` con `orbslam3_multi`.
  - `orbslam3_server/launch/global_orb_map_server.launch.py`: añade parámetros `rawdb_record_enabled`, `rawdb_record_path`, `rawdb_replay_enabled`, `rawdb_replay_path` y `rawdb_replay_period_sec`.
  - `codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml`: escenario `prueba_1c_rawdb_record_simple`.
  - `codex/archivos_auxiliares/trayectorias/tray_prueba_4.yaml`: escenario `prueba_1c_rawdb_replay` con espera de 20 s.
- comandos de build:
  ```bash
  ./codex/herramientas/build_selected_packages.sh orbslam3
  ./codex/herramientas/build_selected_packages.sh orbslam3_multi
  ./codex/herramientas/build_selected_packages.sh orbslam3_server
  ./codex/herramientas/build_selected_packages.sh dron_individual
  ./codex/herramientas/build_selected_packages.sh simulacion_dron
  ./codex/herramientas/build_selected_packages.sh orbslam3_msgs
  ```
- resultado de build: `OK`.
  - todos los builds finales terminaron con `BUILD-EXIT-CODE 0`;
  - se compiló en llamadas separadas para no bloquear el ordenador con paquetes pesados;
  - hubo un fallo intermedio de enlace en `orbslam3_server` por una librería `orbslam3_multi` incoherente tras la interrupción/apagado;
  - se ejecutó `reduce_build_log.sh`;
  - se limpió solo `build/install/orbslam3_multi` y se reconstruyó `orbslam3_multi`;
  - warning no bloqueante preexistente en `MultiDroneSystem.cpp`.
- pruebas Gazebo ejecutadas:
  - `prueba_1`: `OK`, grabación con Gazebo.
    ```bash
    ./codex/herramientas/run_simulation.sh --prueba 1 --launch "ros2 launch simulacion_dron multi_dron.launch.py" --post-scenario-wait-sec 20 --timeout-sec 900
    ```
  - `prueba_4`: `OK`, replay sin Gazebo usando solo `orbslam3_server`.
    ```bash
    ./codex/herramientas/run_simulation.sh --prueba 4 --launch "ros2 launch orbslam3_server global_orb_map_server.launch.py n_drones:=2 namespace_base:=dron use_sim_time:=false rawdb_record_enabled:=false rawdb_replay_enabled:=true rawdb_replay_path:=src/codex/archivos_auxiliares/repeticiones/rawdb_prueba_1.record rawdb_replay_period_sec:=0.05" --post-scenario-wait-sec 5 --startup-wait-sec 2 --timeout-sec 120 --max-gazebo-retries 0
    ```
- patrones de reducción usados:
  - prueba 1:
    ```text
    SCENARIO-RUNNER|GOAL|RESULT|success|F1B-|F1C-RAWDB|F1C-REPLAY|ORBMAP|WRAPPER|ERROR|FATAL|Segmentation fault|Killed
    ```
  - prueba 4:
    ```text
    SCENARIO-RUNNER|success|F1C-REPLAY|F1C-RAWDB|ERROR|FATAL|Segmentation fault|Killed
    ```
- evidencia positiva:
  - `prueba_1`: `SIM-DONE prueba=1 success=true`;
  - `prueba_1`: 6 goals de `scenario_runner_node`, todos con `success=true`;
  - `codex/archivos_auxiliares/repeticiones/rawdb_prueba_1.record` generado con tamaño aproximado `272M`;
  - `prueba_1`: `284` `[F1C-RAWDB-DELTA-RX]`;
  - `prueba_1`: `284` `[F1C-RAWDB-INSERT-DELTA]`;
  - `prueba_1`: `3` `[F1C-RAWDB-NEW-SUBMAP]`;
  - `prueba_1`: `[F1C-RAWDB-SAVE] reason=shutdown ... journal=284 deltas=284 full=0 submaps=3 kfs=142 mps=17884`;
  - `prueba_4`: `SIM-DONE prueba=4 success=true`;
  - `prueba_4`: `[F1C-REPLAY-LOAD] ... success=true entries=284 deltas=284 full=0 submaps=3 kfs=142 mps=17884`;
  - `prueba_4`: `284` `[F1C-REPLAY-DELTA]`;
  - `prueba_4`: `[F1C-REPLAY-DONE] entries=284 journal=284 deltas=284 full=0 submaps=3 kfs=142 mps=17884`.
- evidencia negativa o ausente:
  - no aparece `[F1C-RAWDB-INSERT-FULL]` porque la prueba validada solo ejercita deltas; los métodos de snapshot quedan implementados pero no ejercitados en esta ejecución;
  - no hay nube global, fiduciales, loops, fused landmarks ni optimización; queda fuera de `3C`;
  - el único `ERROR` observado es `gazebo ... exit code 255` durante cleanup posterior a `SIM-DONE success=true`;
  - no aparecen `FATAL`, `Segmentation fault`, `Killed`, `std::bad_alloc` ni `No space left`;
  - el dataset raw dejó el disco con margen bajo, unos `467M`.
- conclusión: `CONSEGUIDA`.
- siguiente paso recomendado: ejecutar `subfase_3D.md` para crear `GlobalPoseStore` usando el dataset raw/replay generado por `3C`, vigilando espacio en disco antes de simulaciones largas.

## 2026-08-11 - Subfase 3C - Reimplementacion activa y tests

- objetivo intentado: reconstruir raw, FIFO/worker principal, replay,
  backpressure y visualización sobre el runtime vacío de 3B.
- archivos modificados: tipos/base/tests raw en `orbslam3_multi`; cola, servidor,
  tests y launch en `orbslam3_server`; launches, YAML, bridge, web y tests en
  `simulacion_dron`.
- paquetes compilados: `orbslam3_multi`, `orbslam3_server`,
  `simulacion_dron`.
- resultado de build: builds finales exit 0. Tests RawDB 1/1, PrimaryQueue 1/1
  y contrato web 1/1 passed; el contrato web tuvo un fallo intermedio por una
  aserción colocada en la función equivocada y pasó tras la corrección mecánica.
- evidencia positiva: `RawMapDatabase` devuelve cambios/revisiones y record
  versionado; la cola conserva FIFO live/replay y una tarea activa máxima;
  backpressure aplica histéresis 8/2; la telemetría se emite antes de liberar
  cada entrada al worker.
- evidencia negativa o ausente: full snapshots no se implementan en 3C; quedan
  expresamente para 3G.
- conclusión: `PARCIAL` hasta completar live y replay.

## 2026-08-11 - Prueba 79 - Primer live 3C

- objetivo intentado: ruta de seis goals, flujo raw, gate, record, web y RViz2.
- prueba Gazebo: escenario completado con seis goals correctos y 212 commits;
  record guardado con 5 submapas, 175 KFs y 17739 MPs.
- evidencia positiva: `pending=0`, `max_active=1`, bridge y flujo raw activos.
- evidencia negativa: la modificación temporal del delay no se aplicó a tiempo;
  no hubo backpressure/gate observable.
- conclusión: `PARCIAL`; no valida el criterio de gate.

## 2026-08-11 - Prueba 81 - Backpressure demasiado tardio

- objetivo intentado: repetir el live provocando backlog durante la ruta.
- evidencia positiva: seis goals correctos, topología web 6/5 y transición
  `backpressure=true` en pending=8.
- evidencia negativa: high llegó 1.2 s después de comenzar el último movimiento,
  sin siguiente lote que bloquear; delay quedó en 5000 al cleanup y no se
  guardó un record válido.
- conclusión: `PARCIAL`; conservar como aprendizaje de temporización.

## 2026-08-11 - Prueba 83 - Timeout independiente y auditoria visual

- objetivo intentado: activar/liberar delay durante la ruta y auditar web/RViz2.
- evidencia positiva: `/health` llegó a secuencia 76; RViz2 estaba suscrito pero
  `/global_sparse_cloud` y `/global_keyframes` tenían `Publisher count: 0`;
  shutdown guardó 19 entradas con `pending=0`, `max_active=1`.
- evidencia negativa: el delay se cambió después del último delta útil,
  backpressure permaneció false y el segundo movimiento agotó 240 s. Las
  capturas revelaron textos residuales 3B y cero eventos por conexión SSE tardía.
- conclusión: `NO CONSEGUIDA`. El timeout no fue causado por el gate.
- correcciones posteriores: rotulación dinámica 3C, layout móvil y cola con
  `MarkReady()` para impedir `START` visual adelantado.

## 2026-08-11 - Prueba 85 - Live de cierre

- objetivo intentado: ejecutar la ruta acordada con delay 5000 desde launch,
  provocar high, liberar a 0, guardar record y cerrar con drenaje.
- patrones de reducción: `F3C-*`, `SCENARIO-RUNNER`, RViz2, topics globales,
  `SIM-*`, errores y procesos muertos.
- evidencia positiva:
  - `backpressure=true` en pending=8 y `false` en pending=2;
  - `MOVE-GATE-WAIT` y `MOVE-GATE-CLEAR`, espera 67.956 s;
  - seis goals correctos en fiducial2 -> x=-8 -> fiducial2;
  - 262 entradas, 3 submapas, 188 KFs, 21659 MPs;
  - record de 325M, `pending=0`, `processed=262`, `max_active=1`;
  - ningún `PRIMARY-START` precedió a su `PRIMARY-ENQUEUE`;
  - `SIM-DONE success=true`, sin errores funcionales 3C.
- interpretación revisada el 2026-08-12: high se alcanzó durante el `wait`
  inicial, antes del primer lote de movimiento. Los commits posteriores de esa
  cola muestran 0 KFs nuevos y 143-181 MPs actualizados por delta. El wrapper
  descarta deltas realmente vacíos; la actividad procedía de estado raw mutable,
  no de ticks sin payload. El grafo no expone aún esa diferencia.
- evidencia negativa: Chrome con espera virtual no produjo captura de pulsos
  por la conexión SSE continua; health/logs y el SSE de replay verifican la
  actividad, y el render estático final se validó aparte.
- conclusión: `CONSEGUIDA` para el live.

## 2026-08-11 - Prueba 86 - Replay de cierre sin Gazebo

- objetivo intentado: cargar el record 85 y recorrer la misma cola/worker.
- prueba: `f3c_replay.launch.py`, puerto 8766 y escenario de espera.
- evidencia positiva:
  - carga y procesa 262 entradas;
  - resultado exacto: journal 262, 3 submapas, 188 KFs, 21659 MPs;
  - backpressure 8/2, `pending=0`, `max_active=1`;
  - bridge listo antes de load, secuencia 587; SSE inspeccionado con eventos
    `primary_queue_worker`, `primary_worker_raw_db` y `backpressure_off`;
  - ningún `START` replay precedió a `REPLAY-ENQUEUE`;
  - `SIM-DONE success=true`.
- evidencia negativa: wrapper->server no se ilumina en replay por diseño.
- conclusión: `CONSEGUIDA`.

## 2026-08-12 - Cierre visual y agregado 3C

- render final: escritorio 1440x900 y móvil 390x844, topología 6/5, rotulación
  3C y leyenda completa; contrato web final 1/1 passed.
- conclusión agregada: `CONSEGUIDA`.
- siguiente paso recomendado: preparar 3D y reutilizar íntegramente el flujo
  principal de 3C.
