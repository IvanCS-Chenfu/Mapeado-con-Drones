# Historial 3B

> Extraido mecanicamente de `historial_fase_3.md`. Leer este archivo antes de abrir otros historiales de subfase.

## 2026-07-06 11:42 — Subfase 3B — Servidor nuevo mínimo y congelación del legacy

- fase y subfase: Fase 3, `subfase_3B.md`.
- objetivo intentado: sustituir el servidor monolítico activo por un servidor mínimo que arranca desde el launch oficial, se suscribe a `OrbMap` delta multi-dron y emite logs `[F1B-*]`, dejando el legacy congelado como referencia.
- archivos modificados:
  - `orbslam3_server/src/global_map_server.cpp`;
  - `orbslam3_server/src/global_map_server_antiguo.cpp`;
  - `orbslam3_server/launch/global_orb_map_server.launch.py`;
  - `orbslam3_server/launch/global_orb_map_server_antiguo.launch.py`;
  - `orbslam3_server/CMakeLists.txt`;
  - `orbslam3_multi/legacy/README.md`;
  - `orbslam3_multi/legacy/*_antiguo.cpp`;
  - `orbslam3_multi/legacy/*_antiguo.hpp`;
  - `codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml`;
  - `codex/contexto/00_LEER_PRIMERO.md`;
  - `codex/contexto/01_ESTADO_ACTUAL.md`;
  - `codex/contexto/03_ARQUITECTURA_ACTUAL.md`;
  - `codex/contexto/04_TOPICS_SERVICES_ACTIONS.md`;
  - `codex/contexto/06_MAPA_CODIGO.md`;
  - `codex/contexto/07_ULTIMA_SESION.md`;
  - `codex/contexto/paquetes/orbslam3_server/orbslam3_server.md`;
  - `codex/contexto/paquetes/orbslam3_server/global_map_server.md`;
  - `codex/contexto/paquetes/orbslam3_server/global_map_server_antiguo.md`;
  - `codex/contexto/paquetes/orbslam3_server/launches.md`;
  - `codex/contexto/paquetes/orbslam3_multi/orbslam3_multi.md`;
  - `codex/contexto/paquetes/simulacion_dron/simulacion_dron.md`;
  - `codex/pipeline/fase_3_sparse_global/pipeline_fase_3.md`;
  - `codex/pipeline/fase_3_sparse_global/subfases/subfase_3B.md`;
  - `codex/pipeline/fase_3_sparse_global/subfases/subfase_3C.md`;
  - `codex/pipeline/fase_3_sparse_global/historial/historial_fase_3.md`.
- cambios de código realizados:
  - `global_map_server.cpp` ahora define un `GlobalMapServer` mínimo autocontenido;
  - el servidor declara/lee `use_sim_time`, `world_frame`, `namespace_base`, `n_drones` y `f1b_stats_period_s`;
  - se crea un subscriber por dron a `/dron_X/orbslam/orb_map_delta`;
  - el callback `OnOrbMapDelta` loggea `drone_id`, `map_epoch`, `map_sequence`, `frame_id`, `map_frame`, KFs y MPs;
  - `PublishStatsLog` emite acumulados por dron y epochs vistos;
  - el servidor activo no incluye `global_map_server.hpp` heredado;
  - `CMakeLists.txt` deja de enlazar `global_map_server` con `ORB_SLAM3/g2o`.
- cambios legacy:
  - `global_map_server_antiguo.cpp` conserva el servidor monolítico anterior y no se compila;
  - `global_orb_map_server_antiguo.launch.py` conserva el launch anterior y no es el activo;
  - `orbslam3_multi/legacy/` contiene copias congeladas no compiladas de módulos actuales.
- paquetes compilados:
  ```bash
  ./codex/herramientas/build_selected_packages.sh orbslam3_msgs orbslam3_multi orbslam3_server simulacion_dron
  ```
- resultado de build:
  - primer intento: `BUILD-EXIT-CODE 2` por `No space left on device`;
  - se ejecutó `reduce_build_log.sh`;
  - se limpió de forma mínima `/home/chenfu/Gazebo/build/orbslam3_server` y `/home/chenfu/Gazebo/log/build_2026-07-06_11-35-41`;
  - segundo intento: `BUILD-EXIT-CODE 0`;
  - `4 packages finished`: `orbslam3_msgs`, `orbslam3_multi`, `orbslam3_server`, `simulacion_dron`;
  - warning no bloqueante: `global_pose_corrector.cpp` tiene variable `deg_to_rad` no usada.
- prueba Gazebo ejecutada:
  ```bash
  ./codex/herramientas/run_simulation.sh --prueba 1 --launch "ros2 launch simulacion_dron multi_dron.launch.py" --post-scenario-wait-sec 20
  ```
- YAML usado:
  - `codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml`;
  - escenario `prueba_1b_recepcion_orbmap_multidron`;
  - trayectoria multi-dron de 3A: fiducial 2, izquierda y vuelta, con `dron_2` a mayor altura.
- resultado de simulación:
  - `SIM-SCENARIO-EXIT-CODE 0`;
  - `SIM-DONE prueba=1 success=true`;
  - `SIM-EXIT-CODE 0`;
  - 6 goals enviados y 6 goals con `success=true`.
- patrones usados para reducir logs:
  ```text
  SCENARIO-RUNNER|GOAL|RESULT|success|PIPE0-WRAPPER-DELTA-PUB|PIPE0-WRAPPER-FULL-SNAPSHOT|PIPE0-WRAPPER-TRACK|F1B-SERVER-INIT|F1B-SERVER-PARAMS|F1B-SERVER-SUBSCRIBED|F1B-ORBMAP-RX|F1B-ORBMAP-RX-KFS|F1B-ORBMAP-RX-MPS|F1B-SERVER-STATS|ERROR|FATAL|Segmentation fault|Killed
  ```
- logs generados:
  - `codex/archivos_auxiliares/logs/colcon_build.log`: `39` líneas;
  - `codex/archivos_auxiliares/logs/colcon_build.reduced.log`: generado para el primer build fallido por espacio;
  - `codex/archivos_auxiliares/logs/prueba_1.log`: `1906` líneas;
  - `codex/archivos_auxiliares/logs/prueba_1.reduced.log`: `1788` líneas.
- evidencia positiva encontrada:
  - `[F1B-SERVER-INIT] node=global_orb_map_server mode=minimal_rx_only`;
  - `[F1B-SERVER-PARAMS] use_sim_time=true world_frame=world namespace_base=dron drones=2`;
  - `[F1B-SERVER-SUBSCRIBED]` para `drone_id=1 topic=/dron_1/orbslam/orb_map_delta`;
  - `[F1B-SERVER-SUBSCRIBED]` para `drone_id=2 topic=/dron_2/orbslam/orb_map_delta`;
  - `[F1B-ORBMAP-RX]` aparece para `drone_id=1` y `drone_id=2`;
  - conteo reducido: `174` recepciones de `drone_id=1` y `173` de `drone_id=2`;
  - `[F1B-ORBMAP-RX-KFS]` y `[F1B-ORBMAP-RX-MPS]` aparecen con conteos y rangos de IDs;
  - `[F1B-SERVER-STATS]` final muestra `rx_maps=329`, `rx_kfs=2030`, `rx_mps=482182`, `drones_seen=2`, `epochs_seen=3`;
  - `PIPE0-WRAPPER-TRACK` y `PIPE0-WRAPPER-DELTA-PUB` confirman wrappers activos.
- evidencia negativa o ausente:
  - no hay nube global, fiduciales, loops, fused landmarks ni optimización; es esperado en `3B`;
  - no se observó RViz2 porque no es criterio de éxito;
  - el único `ERROR` es `gazebo ... exit code 255` durante cleanup posterior a `SIM-DONE success=true`;
  - no aparecen `FATAL`, `Segmentation fault`, `Killed`, `std::bad_alloc` ni `No space left` durante la simulación;
  - el disco sigue con poco margen libre, unos `180M` tras la simulación, y conviene vigilarlo en la siguiente subfase.
- criterio de éxito:
  - build final `0`: cumplido;
  - prueba Gazebo ejecutada: cumplido;
  - `scenario_runner_node` y goals `success=true`: cumplido;
  - marcadores `[F1B-*]` obligatorios: cumplido;
  - recepción de al menos dos drones: cumplido;
  - legacy no compilado: cumplido;
  - documentación e historial actualizados: cumplido.
- conclusión: `CONSEGUIDA`.
- siguiente paso recomendado: ejecutar `subfase_3C.md` para crear `RawMapDatabase`, asignar `arrival_id`, guardar estado raw/journal y preparar replay sin Gazebo.

## 2026-07-06 — Subfase 3B reabierta — Comentarios y trazabilidad por subfase

- motivo: el usuario no quiere dar por concluida `3B` hasta que el código nuevo quede comentado con suficiente detalle y trazabilidad por subfase.
- decisión de estado:
  - la evidencia técnica de `3B` se conserva: build final `0`, simulación `success=true`, recepción `OrbMap` de dos drones y logs `[F1B-*]`;
  - `3B` vuelve a ser la subfase actual;
  - `3C` vuelve a quedar `sin hacer`;
  - la conclusión operativa pasa a `PARCIAL` hasta revisar comentarios/trazabilidad.
- regla añadida:
  - funciones nuevas o modificadas deben explicar objetivo, entradas relevantes, efecto y subfase;
  - callbacks ROS deben explicar topic procesado, estado actualizado y qué subfase los introdujo;
  - bucles y guards importantes deben explicar invariantes y motivo;
  - los comentarios deben usar etiquetas como `F1B`, `F1C`, etc. cuando correspondan a una subfase concreta.
- archivos modificados para reglas de agente/contexto:
  - `AGENTS.md`;
  - `.codex/agents/implementador_fase.toml`;
  - `codex/contexto/00_LEER_PRIMERO.md`;
  - `codex/contexto/01_ESTADO_ACTUAL.md`;
  - `codex/contexto/03_ARQUITECTURA_ACTUAL.md`;
  - `codex/contexto/06_MAPA_CODIGO.md`;
  - `codex/contexto/07_ULTIMA_SESION.md`;
  - `codex/pipeline/fase_3_sparse_global/pipeline_fase_3.md`;
  - `codex/pipeline/fase_3_sparse_global/subfases/subfase_3B.md`;
  - `codex/pipeline/fase_3_sparse_global/subfases/subfase_3C.md`.
- archivos de paquete/documentación actualizados:
  - `codex/contexto/paquetes/orbslam3_server/global_map_server.md`;
  - `codex/contexto/paquetes/orbslam3_server/launches.md`.
- archivos de código comentados:
  - `orbslam3_server/src/global_map_server.cpp`;
  - `orbslam3_server/launch/global_orb_map_server.launch.py`;
  - `orbslam3_server/CMakeLists.txt`.
- build/simulación:
  - no se reejecutan en esta entrada porque los cambios son comentarios y documentación; no modifican lógica, topics, parámetros ni dependencias.
- conclusión: `PARCIAL`.
- siguiente paso recomendado: revisar si los comentarios `F1B` son suficientes; si el usuario los acepta, repetir build/simulación si se desea una validación formal y marcar `3B` como `realizado`.

## 2026-07-06 — Subfase 3B cerrada tras revisión de comentarios

- decisión del usuario: aceptar la subfase `3B` como conseguida tras la revisión de comentarios y trazabilidad `F1B`.
- estado actualizado:
  - `subfase_3B.md`: `realizado`;
  - `subfase_3C.md`: `actual`;
  - `pipeline_fase_3.md`: `3B` pasa a `realizado` y `3C` pasa a `actual`;
  - contexto operativo actualizado para que un chat nuevo arranque en `3C`.
- evidencia conservada:
  - build final `BUILD-EXIT-CODE 0`;
  - simulación `SIM-DONE prueba=1 success=true`;
  - recepción `OrbMap` de `dron_1` y `dron_2`;
  - logs `[F1B-*]` obligatorios;
  - comentarios `F1B` añadidos en servidor, launch y CMake.
- build/simulación:
  - no se reejecutan en este cierre administrativo porque solo cambia estado documental; la evidencia técnica de `3B` ya existe.
- conclusión: `CONSEGUIDA`.
- siguiente paso recomendado: ejecutar `subfase_3C.md` para crear `RawMapDatabase`, asignar `arrival_id`, guardar journal/rawdb y validar replay.

## 2026-08-10 13:20 — Subfase 3B rehecha — Prueba 77 intento 1

- objetivo intentado: validar que los paquetes reiniciados compilan y que la
  simulacion arranca con `global_map_server` vacio, sin movimientos ni salidas
  sparse globales;
- cambios previos: runtime anterior y sus MD congelados en `legacy2`;
  `orbslam3_multi` vacio; servidor y launch minimos; launch de simulacion sin
  parametros legacy; YAML de espera de 5 s;
- paquetes compilados: `orbslam3_multi`, `orbslam3_server` y
  `simulacion_dron`;
- resultado de build: exit code `0`, tres paquetes finalizados;
- prueba Gazebo: prueba 77 con RViz2, bridge y navegador desactivados, startup
  15 s, timeout 60 s y post-wait 3 s;
- resultado: `scenario_runner_node=1`, herramienta exit code `1`,
  `success=false`;
- patrones de reducción: servidor vacio, scenario/YAML, errores, excepciones y
  cierre;
- evidencia positiva: `[F3B-EMPTY-SERVER-INIT]`; el servidor arranco y termino
  limpiamente;
- evidencia negativa: el runner no pudo abrir el YAML porque recibió una ruta
  relativa a `src/` desde el workspace padre;
- artefactos: `prueba_77_intento_1.log` y
  `prueba_77_intento_1.reduced.log`;
- conclusión: `NO CONSEGUIDA` por configuración mecánica de la ruta, sin fallo
  del servidor ni del launch;
- siguiente paso recomendado: repetir con ruta YAML absoluta sin cambiar el
  escenario ni los criterios.

## 2026-08-10 13:21 — Subfase 3B rehecha — Prueba 77 intento 2

- objetivo intentado: repetir la misma validación con la ruta YAML corregida;
- cambios entre intentos: ninguno en codigo o comportamiento; solo ruta
  absoluta al invocar `run_simulation.sh`;
- paquetes compilados: se reutiliza el build correcto del intento anterior;
- resultado de build: build incremental exit `0`; tras detectar symlinks stale
  en install se limpiaron solo los artefactos generados de ambos paquetes y el
  rebuild limpio finalizó también con exit `0` y tres paquetes;
- prueba Gazebo: mismo launch y tiempos, YAML absoluto;
- resultado: `scenario_runner_node=0`, `SIM-DONE prueba=77 success=true` y
  herramienta exit code `0`;
- patrones de reducción: marcador del servidor, scenario, outputs globales,
  backpressure, RawDB, loops, fiduciales, errores y cierre;
- evidencia positiva: único inicio `[F3B-EMPTY-SERVER-INIT]`, wait de 5 s
  completado y `global_map_server` finalizado limpiamente;
- auditoria final: Colcon descubre cada paquete una sola vez; el install del
  servidor contiene solo el ejecutable/launch activos y el de `orbslam3_multi`
  no contiene cabeceras, librerias, tests ni ejecutables;
- evidencia negativa o ausente: no aparece actividad sparse global del
  servidor. Los `PIPE0-WRAPPER-DELTA-PUB` visibles pertenecen a wrappers
  locales y se ignoran al no existir subscriptions en el servidor;
- nota de cierre: `gazebo exit code 255` aparece tras `SIM-DONE`, durante el
  cleanup habitual de la herramienta;
- artefactos: `prueba_77.log`, `prueba_77.reduced.log`, build final en
  `colcon_build.log` y build incremental preservado en
  `colcon_build_3B_incremental.log`;
- conclusión: `CONSEGUIDA`;
- siguiente paso recomendado: preparar y autorizar 3C para reintroducir la
  recepción raw según su contrato, reutilizando selectivamente `legacy2`.

## 2026-08-10 14:12 — Subfase 3B ampliada — Prueba 78 y grafo runtime base

- objetivo intentado: dejar creada en 3B la infraestructura web live que
  crecerá subfase a subfase, manteniendo el runtime global vacío y validando
  conjuntamente simulación, RViz2 y el grafo;
- archivos funcionales modificados: bridge y frontend bajo
  `simulacion_dron/src/visualizer/` y `simulacion_dron/web/pipeline_flow/`,
  test CMake/Pytest, metadatos del paquete y `tray_prueba_78.yaml`;
- snapshot: el visualizador anterior quedó congelado en
  `simulacion_dron/legacy2/pipeline_flow_visualizer/` y su MD en el `legacy2`
  documental;
- comportamiento obtenido: grafo con `Wrappers ORB-SLAM3` y
  `GlobalMapServer`, cero aristas; SSE live sin replay inicial, reconexión por
  `Last-Event-ID`, `state_reset` ante gaps y drenaje por
  `requestAnimationFrame`;
- paquetes compilados: `orbslam3_multi`, `orbslam3_server` y
  `simulacion_dron`; build completo exit `0`. Tras descubrir que el bridge no
  tenía permiso ejecutable se corrigió mecánicamente y el rebuild de
  `simulacion_dron` volvió a terminar con exit `0`;
- tests: Pytest directo `4 passed`; CTest integrado
  `pipeline_flow_contract` 1/1 passed;
- validación web aislada: `/health` devolvió `ready/live`, capacidad 512,
  secuencia 0 y sin replay inicial. Capturas 1440x900 y 390x844 mostraron dos
  nodos, cero aristas, SSE conectado, cero eventos/gaps y sin solapes;
- prueba Gazebo: prueba 78 con launch visual por defecto, espera inicial de
  14 s, goals simultáneos a `(0,-9,1.0)` y `(0,-9,1.3)`, y observación de
  30 s en fiducial 2;
- patrones de reducción: marcadores 3B del servidor/bridge,
  `SCENARIO-RUNNER`, goals/waits, RViz2, topics globales, errores y cierre;
- evidencia positiva: ambos goals terminaron `success=true` en 22 s; la espera
  de 30 s se completó; `SIM-DONE prueba=78 success=true`; RViz2 y bridge
  arrancaron y cerraron limpiamente; el bridge anunció
  `mode=live topology=2_nodes_0_edges` y `/health` integrado mantuvo secuencia
  0;
- evidencia negativa o ausente: no aparecen `global_sparse_map` ni
  `global_keyframes`; el servidor activo tampoco contiene publishers,
  subscriptions o timers;
- confirmación visual posterior del usuario: en RViz2 no apareció nube ni KFs
  globales y en el grafo web no apareció actividad/conexión; coincide con los
  dos nodos y cero aristas exigidos para 3B;
- nota de cierre: `gazebo exit code 255` ocurre después de `SIM-DONE`, durante
  el cleanup habitual, y no invalida el escenario;
- artefactos: `prueba_78.log`, `prueba_78.reduced.log` y
  `colcon_build.log`; el log completo no se leyó directamente;
- conclusión revisada: `CONSEGUIDA`; los criterios automáticos y visuales de
  la misma prueba 78 están confirmados;
- siguiente paso recomendado: preparar 3C con el usuario y reintroducir solo la
  conexión/ingesta raw que pertenece a esa subfase.

## 2026-08-12 - Subfase 3B - Apertura integrada y permisos operativos

- objetivo intentado: ejecutar build y pruebas futuras sin comandos GUI ad hoc
  ni peticiones repetidas fuera de las dos herramientas autorizadas;
- cambios: `pipeline_flow_browser.py` espera `/health=ready`, abre una unica
  pestaña y emite marcadores de exito/error; los launches son propietarios de
  Gazebo, RViz2, bridge y helper; `run_simulation.sh` incorpora
  `--without-gazebo` para replay y mantiene el healthcheck normal en live;
- correccion adicional: `f3f_replay.launch.py` sanea variables Snap/VS Code
  antes de iniciar RViz2, igual que el launch principal;
- build: `orbslam3_multi orbslam3_server simulacion_dron`, exit 0; rebuild
  mecanico de `simulacion_dron`, exit 0;
- tests: contrato web 8/8 y sintaxis Bash/Python correctas;
- evidencia runtime: replay 95 y live 96 registran
  `[F3B-FLOW-BROWSER-OPEN]` despues de
  `[F3F-FLOW-WEB-READY]`; el live arranca todo desde
  `multi_dron.launch.py` sin comandos de navegador separados;
- evidencia negativa: replay 94 revelo que la herramienta exigia Gazebo a un
  launch sin Gazebo y que RViz heredaba una `libpthread` de Snap; el intento se
  conserva en 3F y ambos problemas quedaron corregidos mecanicamente;
- conclusion: `CONSEGUIDA`; la ampliacion preserva el contrato funcional 3B y
  deja una ruta unica y repetible para las subfases siguientes.
