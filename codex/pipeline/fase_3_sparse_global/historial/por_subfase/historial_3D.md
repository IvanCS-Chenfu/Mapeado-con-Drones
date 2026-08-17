# Historial 3D

> Extraido mecanicamente de `historial_fase_3.md`. Leer este archivo antes de abrir otros historiales de subfase.

## 2026-07-08 — Subfase 3D — `GlobalPoseStore` ligero

- objetivo intentado: crear en `orbslam3_multi` una base ligera de poses globales de KeyFrames, separada de `RawMapDatabase`, capaz de guardar anchors de submapas, poses `world_T_kf`, poses optimizadas registradas y una corrección heredable por submapa.
- archivos modificados:
  - `orbslam3_multi/include/orbslam3_multi/raw_map_types.hpp`;
  - `orbslam3_multi/include/orbslam3_multi/raw_map_database.hpp`;
  - `orbslam3_multi/src/raw_map_database.cpp`;
  - `orbslam3_multi/include/orbslam3_multi/global_pose_store.hpp`;
  - `orbslam3_multi/src/global_pose_store.cpp`;
  - `orbslam3_multi/CMakeLists.txt`;
  - `orbslam3_server/src/global_map_server.cpp`;
  - `orbslam3_server/launch/global_orb_map_server.launch.py`;
  - `codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml`;
  - documentación de contexto, paquetes y pipeline relacionada con `3D`/`3E`.
- cambios realizados:
  - `RawKeyFrameId` y `RawMapPointId` pasan a ser comparables para poder indexar consultas ligeras;
  - `RawMapDatabase` añade getters constantes `GetKeyFrame` y `GetMapPoint`;
  - se crea `GlobalPoseStore` con `AnchorSubmap`, `RegisterNewKeyFrameIfAnchored`, `SetOptimizedKeyFramePose`, consultas de pose y estadísticas;
  - `GlobalPoseStore` calcula poses globales solo cuando recibe un anchor externo y no modifica el raw;
  - se añade una política temporal `3D` de corrección heredable para KFs nuevos posteriores a una pose optimizada;
  - `global_map_server` integra un modo debug `pose_store_debug_*` para validar con replay de `rawdb_prueba_1.record`;
  - `global_orb_map_server.launch.py` expone los parámetros debug, desactivados por defecto;
  - `tray_prueba_1.yaml` queda como escenario de espera para replay sin movimientos.
- paquetes compilados:
  ```bash
  ./codex/herramientas/build_selected_packages.sh orbslam3_multi orbslam3_server
  ./codex/herramientas/reduce_build_log.sh
  ./codex/herramientas/build_selected_packages.sh orbslam3_multi
  ./codex/herramientas/build_selected_packages.sh orbslam3_server
  ```
- resultado de build:
  - el primer build conjunto falló por falta de espacio: `No space left on device`;
  - se redujo el log con `reduce_build_log.sh`;
  - se hizo limpieza mínima de artefactos generados de `orbslam3_multi`, `orbslam3_server` y un log antiguo de build; también se retiraron logs auxiliares antiguos `prueba_2.log` y `prueba_2.reduced.log` dentro de `src`;
  - build final de `orbslam3_multi`: `BUILD-EXIT-CODE 0`;
  - build final de `orbslam3_server`: `BUILD-EXIT-CODE 0`;
  - quedó un warning no bloqueante ya existente en `global_pose_corrector.cpp` por `deg_to_rad` sin uso.
- pruebas Gazebo/replay ejecutadas:
  - prueba principal `1`: replay sin Gazebo visual, usando `run_simulation.sh` con launch `ros2 launch orbslam3_server global_orb_map_server.launch.py`;
  - parámetros relevantes: `rawdb_replay_enabled:=true`, `rawdb_replay_path:=src/codex/archivos_auxiliares/repeticiones/rawdb_prueba_1.record`, `pose_store_debug_enabled:=true`, anchor sintético tras 5 entradas y corrección sintética tras 10 entradas;
  - no se ejecutó la prueba opcional de generación de dataset porque `rawdb_prueba_1.record` ya existía desde `3C`.
- patrones usados para reducir logs:
  ```text
  SCENARIO-RUNNER|GOAL|RESULT|success|F1C-REPLAY|F1D-SERVER|F1D-POSESTORE|F1D-POSESTORE-ANCHOR|F1D-POSESTORE-OPT|F1D-POSESTORE-CORRECTION|F1D-POSESTORE-NEW-KF|F1D-POSESTORE-STATS|ERROR|FATAL|Segmentation fault|Killed
  ```
- evidencia positiva encontrada:
  - `SIM-DONE prueba=1 success=true`;
  - `[SCENARIO-RUNNER-DONE] scenario='prueba_1d_global_pose_store_replay' success=true`;
  - `[F1C-REPLAY-DONE] entries=284 journal=284 deltas=284 full=0 submaps=3 kfs=142 mps=17884`;
  - `[F1D-POSESTORE-INIT] anchors=0 world_poses=0 optimized_kfs=0 corrections=0`;
  - `[F1D-POSESTORE-ANCHOR-SUMMARY] drone_id=1 epoch=0 source=DEBUG_TEST anchored_kfs=1`;
  - `[F1D-POSESTORE-OPT-POSE-SET] drone_id=1 epoch=0 kf=0 source=DEBUG_TEST_OPT`;
  - `[F1D-POSESTORE-CORRECTION-SET] drone_id=1 epoch=0 kf=0 source=DEBUG_TEST_OPT dx=0.150 dy=-0.030 dz=0.000 dyaw=0.050`;
  - cuatro eventos `[F1D-POSESTORE-NEW-KF-CORRECTION-INHERIT]` para KFs nuevos del submapa anclado;
  - estadísticas finales con `anchors=1`, `world_poses=5`, `optimized_kfs=1` y `corrections=1`.
- evidencia negativa o ausente:
  - no hay anchor real todavía; el anchor de `3D` es sintético/debug y el primer anchor real queda para `3E`;
  - no se publica mapa global visible en RViz2 en `3D`;
  - no se tocaron fiduciales reales, loops, fused landmarks ni optimización local real;
  - no aparecen `FATAL`, `Segmentation fault`, `Killed` ni errores graves en los logs reducidos; la palabra `ERROR` aparece solo en la línea de patrones de reducción;
  - el disco quedó con margen bajo, aproximadamente `569M` tras el cierre documental, por lo que conviene vigilar espacio antes de simulaciones largas.
- documentación actualizada:
  - `codex/contexto/00_LEER_PRIMERO.md`;
  - `codex/contexto/01_ESTADO_ACTUAL.md`;
  - `codex/contexto/06_MAPA_CODIGO.md`;
  - `codex/contexto/07_ULTIMA_SESION.md`;
  - `codex/contexto/paquetes/orbslam3_multi/orbslam3_multi.md`;
  - `codex/contexto/paquetes/orbslam3_multi/raw_map_database.md`;
  - `codex/contexto/paquetes/orbslam3_multi/global_pose_store.md`;
  - `codex/contexto/paquetes/orbslam3_server/orbslam3_server.md`;
  - `codex/contexto/paquetes/orbslam3_server/global_map_server.md`;
  - `codex/contexto/paquetes/orbslam3_server/launches.md`;
  - `codex/pipeline/fase_3_sparse_global/pipeline_fase_3.md`;
  - `codex/pipeline/fase_3_sparse_global/subfases/subfase_3D.md`;
  - `codex/pipeline/fase_3_sparse_global/subfases/subfase_3E.md`.
- conclusión: `CONSEGUIDA`.
- siguiente paso recomendado: ejecutar `subfase_3E.md` para crear `FiducialAnchorManager` y convertir una observación fiducial simulada en el primer anclaje real de submapa, poblando `GlobalPoseStore` sin usar ground truth para construir el mapa general.

## 2026-08-12 01:15 — Subfase 3D — reimplementación incremental autorizada

- objetivo intentado: rehacer 3D sobre el runtime 3C, manteniendo raw y poses
  como autoridades separadas, con linaje explícito, commits atómicos y una rama
  de poses condicional dentro de la misma tarea principal.
- archivos modificados: tipos y base raw, nuevos `global_pose_types`,
  `GlobalPoseStore` y `SparseGlobalBackend`; integración del servidor; launch
  de replay; topología web 3D; tests y escenarios 87/89.
- paquetes compilados: `orbslam3_multi`, `orbslam3_server` y
  `simulacion_dron`.
- resultado de build: exit 0. El único aviso fue el prefijo Drake inexistente
  ya conocido. Un rebuild posterior de `simulacion_dron`, tras ajustar el
  submapa sintético del replay, también terminó con exit 0.
- tests: RawDB 5/5, `GlobalPoseStore`/`SparseGlobalBackend` 4/4,
  `PrimaryQueue` 4/4 y contrato web 6/6. `flake8` focalizado y `uncrustify`
  focalizado pasan.
- evidencia negativa o ausente: el `colcon test` completo también escanea
  archivos históricos de `legacy2` y conserva fallos de lint ajenos al target
  activo; una incidencia de formato activa detectada allí se corrigió y las
  suites funcionales permanecen correctas.
- conclusión: `PARCIAL`, pendiente aún de replay y live en este punto.
- siguiente paso recomendado: validar primero determinismo/anchor con replay y
  después el flujo live sin anchor sintético.

## 2026-08-12 01:16 — Subfase 3D — prueba 87 replay inicial

- objetivo intentado: reproducir `rawdb_prueba_85.record` por la cola/worker de
  3C y validar un anchor sintético con poses world.
- pruebas Gazebo/replay: prueba 87, launch `f3d_replay.launch.py`, sin Gazebo.
- patrones de reducción: `SIM-DONE`, `F3C-REPLAY`, `F3D-POSE`,
  `F3D-SYNTHETIC-ANCHOR`, `F3D-REPLAY-POSE-DONE`, shutdown y errores graves.
- evidencia positiva: `success=true`; 262 entradas, 3 submapas, 188 KFs,
  21659 MPs, `max_active=1`, un anchor y una pose; shutdown con `pending=0`.
- evidencia negativa o ausente: el anchor se aplicó a `(1,0)` cuando ya no
  llegaron KFs posteriores de ese submapa. La prueba validó el anclaje inicial,
  pero no la actualización incremental posterior.
- conclusión: `PARCIAL`. No se reinterpreta como pasada por el éxito del intento
  posterior.
- siguiente paso recomendado: repetir cambiando únicamente el submapa sintético
  a `(1,1)`, que sí conserva entradas posteriores en el record.

## 2026-08-12 01:18 — Subfase 3D — prueba 89 replay definitivo

- objetivo intentado: cubrir anchor, altas y reconciliaciones incrementales sin
  alterar el journal ni las estadísticas raw.
- pruebas Gazebo/replay: prueba 89 con el mismo record y anchor sintético
  explícito sobre `(1,1)`.
- patrones de reducción: los mismos de prueba 87, incluyendo estados
  `unanchored`, `applied` y ramas `SKIP`.
- evidencia positiva: `success=true`; raw reproduce exactamente 262 entradas,
  3 submapas, 188 KFs y 21659 MPs. El anchor en `arrival_id=92` crea una pose;
  entradas posteriores generan commits incrementales con altas y cambios. El
  resumen final informa `anchors=1`, `poses=101`, `active=101`, `inactive=0`,
  `commits=43`, `revision=43` y `debug_anchor=true`.
- evidencia adicional: los submapas no anclados siguen produciendo
  `unanchored`; las entradas sin cambios relevantes de pose producen `SKIP`.
- evidencia negativa o ausente: no hay anchor real, publicación ROS espacial ni
  validación visual live, por contrato de 3D.
- conclusión: `CONSEGUIDA` para la validación replay de 3D.
- siguiente paso recomendado: ejecutar live con el anchor sintético desactivado.

## 2026-08-12 01:20 — Subfase 3D — prueba 88 live

- objetivo intentado: validar el runtime completo con dos drones, web y RViz2,
  sin anchor sintético ni publicación espacial.
- pruebas Gazebo/replay: prueba 88 reutilizando `tray_prueba_79.yaml`; ruta
  fiducial 2 -> x=-8 -> fiducial 2, tres movimientos por cada dron.
- patrones de reducción: `SCENARIO-RUNNER`, goals/resultados, `F3C-*`, `F3D-*`,
  bridge web, RViz2, topics globales, shutdown y errores graves.
- evidencia positiva: herramienta y escenario terminan `success=true`; los seis
  goals devuelven éxito; el bridge anuncia `mode=live topology=7_nodes_6_edges`;
  aparecen tanto `F3D-POSE-SKIP` como `F3D-POSE-STAGE status=unanchored`; no se
  crea anchor sintético ni commit `applied`; shutdown informa `pending=0`,
  `processed=206`, `active=0`, `max_active=1`.
- auditoría durante ejecución: `/health` devuelve `ready/live`, capacidad 512 y
  secuencia 203. `/global_sparse_cloud` y `/global_keyframes` tienen una
  suscripción de RViz2 y cero publishers. La búsqueda estática confirma que los
  paquetes activos no crean esos publishers.
- evidencia negativa o ausente: el usuario confirmó que Chrome se abrió, pero
  no mostró ningún vértice, arista ni grafo; indicó que lo mismo ocurrió en la
  última prueba 3C. Por tanto, `/health`, el contrato Python y una captura
  headless previa no bastaban para validar la apertura real. Gazebo devuelve
  255 únicamente después de `SIM-DONE`, durante el cierre escalado de procesos;
  no es causa de fallo del escenario. La observación de RViz2 aún no se ha
  incorporado.
- conclusión: `PARCIAL`; el flujo técnico live funciona, pero el criterio visual
  web de la prueba 88 no se consiguió.
- siguiente paso recomendado: corregir apertura/cache del navegador y validarlo
  de forma aislada, sin repetir Gazebo hasta que el usuario vea el grafo.

## 2026-08-12 01:41 — Subfase 3D — corrección visual aislada

- objetivo intentado: eliminar el lienzo vacío observado en Chrome y volver a
  abrir el grafo sin ejecutar Gazebo.
- diagnóstico: el frontend servido directamente desde source sí renderiza 7
  nodos y 6 aristas y los assets instalados existen. El launch abría Chrome en
  paralelo al bridge y no impedía reutilizar assets en caché, dejando una
  carrera de readiness y una copia web potencialmente antigua.
- archivos modificados: bridge HTTP, helper nuevo
  `pipeline_flow_browser.py`, launches live/replay, launch aislado
  `pipeline_flow_only.launch.py`, bootstrap HTML, CMake, tests y documentación.
- paquetes compilados: `simulacion_dron`.
- resultado de build: exit 0; único warning no bloqueante por prefijo Drake
  inexistente.
- pruebas: contrato web 8/8, `flake8` focalizado correcto y launch aislado en
  puerto 8765, sin Gazebo, drones, wrappers, servidor global ni RViz2.
- evidencia positiva: `/health` devuelve `ready/live`; HTML 200 incluye
  `Cache-Control: no-store`; el helper registra `[F3D-FLOW-BROWSER-OPEN]` con
  URL `fresh`; captura 1440x900 del frontend corregido muestra 7 nodos y 6
  aristas.
- evidencia visual del usuario: confirmó que la nueva ventana sí muestra el
  grafo con sus vértices y aristas.
- evidencia negativa o ausente: esta ejecución aislada no contiene mensajes
  reales porque, deliberadamente, no arrancó Gazebo ni el servidor global.
- conclusión: `CONSEGUIDA` para la corrección visual aislada.
- siguiente paso recomendado: repetir la prueba live para observar actividad
  real en el grafo corregido.

## 2026-08-12 01:48 — Subfase 3D — prueba 90 live con grafo corregido

- objetivo intentado: repetir el recorrido live de prueba 88 después de validar
  la corrección de apertura, observando mensajes reales en web y manteniendo
  RViz2 sin publicación espacial.
- pruebas Gazebo/replay: `multi_dron.launch.py` con
  `tray_prueba_79.yaml`, startup 15 s, timeout 420 s y espera final 10 s.
- patrones de reducción: `SIM`, escenario/goals, `F3C-RAW-COMMIT`, ciclo del
  worker, backpressure, `F3D-*`, bridge/helper web, RViz2, topics globales y
  errores graves.
- evidencia positiva: herramienta y escenario terminan con exit 0 y
  `SIM-DONE success=true`; los seis goals completan en 22 s con
  `success=true`; bridge anuncia topología 7/6 y el helper abre una URL `fresh`
  después de `ready`.
- flujo principal: 101 entradas producen `F3D-POSE-SKIP` y 87 producen
  `F3D-POSE-STAGE status=unanchored`; no aparecen anchors sintéticos ni estado
  `applied`. El shutdown informa `pending=0`, `processed=175`, `active=0` y
  `max_active=1`.
- evidencia visual del usuario: observó mensajes alcanzando
  `GlobalPoseStore` aunque ningún submapa estaba anclado. Esto confirma la rama
  esperada: el store recibe cambios de pose, consulta su autoridad de anchors y
  responde `unanchored` sin crear ni almacenar poses globales.
- evidencia negativa o ausente: queda pendiente la descripción del contenido
  de RViz2. Gazebo devuelve 255 después de `SIM-DONE`, durante el cierre
  escalado; no causó el fallo del escenario.
- conclusión: `PARCIAL`; criterios automáticos conseguidos, validación visual
  humana pendiente.
- siguiente paso recomendado: incorporar lo observado por el usuario a esta
  misma entrada y decidir el cierre agregado de 3D.
