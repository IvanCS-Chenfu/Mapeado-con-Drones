# Historial 3G

> Extraido mecanicamente de `historial_fase_3.md`. Leer este archivo antes de abrir otros historiales de subfase.

## 2026-07-08 — Subfase 3G — Full snapshots y reconciliación segura

- objetivo intentado: pedir full snapshots a los wrappers ORB-SLAM3, guardarlos en el journal raw con `arrival_id`, reconciliar KFs/MPs existentes y proteger el estado global de `GlobalPoseStore`.
- archivos modificados:
  - `orbslam3_multi/include/orbslam3_multi/raw_map_types.hpp`;
  - `orbslam3_multi/src/raw_map_database.cpp`;
  - `orbslam3_multi/include/orbslam3_multi/global_pose_store.hpp`;
  - `orbslam3_multi/src/global_pose_store.cpp`;
  - `orbslam3_server/src/global_map_server.cpp`;
  - `orbslam3_server/launch/global_orb_map_server.launch.py`;
  - `codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml`;
  - `codex/archivos_auxiliares/trayectorias/tray_prueba_2.yaml`;
  - documentación de contexto, paquetes, pipeline e historial relacionada con `3G`.
- cambios realizados:
  - `RawInsertResult` pasa a informar submapa, tipo de entrada, `arrival_id`, KFs/MPs nuevos, actualizados y bad;
  - `RawMapDatabase::InsertFullSnapshot` aplica política conservadora `insert_update_no_absent_delete`;
  - se detectan cambios de pose local raw en KFs existentes y se registran como `RawPoseChange`;
  - `GlobalPoseStore::ReconcileAfterRawIngestResult` recalcula poses derivadas de anchor y conserva poses optimizadas del servidor;
  - `global_map_server` crea clientes `/dron_X/orbslam/get_full_map`, solicita snapshots al arranque y periódicamente, procesa respuestas y reproduce snapshots desde `.record`;
  - se filtran snapshots vacíos/no inicializados antes de tratarlos como evidencia útil;
  - se borró el antiguo `codex/archivos_auxiliares/repeticiones/prueba_diff_anclaje.record` solicitado por el usuario.
- paquetes compilados:
  ```bash
  ./codex/herramientas/build_selected_packages.sh orbslam3_multi
  ./codex/herramientas/build_selected_packages.sh orbslam3_server
  ```
- resultado de build:
  - `orbslam3_multi`: `BUILD-EXIT-CODE 0`;
  - `orbslam3_server`: `BUILD-EXIT-CODE 0`;
  - no hizo falta ejecutar `reduce_build_log.sh`.
- pruebas Gazebo/replay ejecutadas:
  - prueba `1`: Gazebo live con launch oficial `ros2 launch simulacion_dron multi_dron.launch.py`;
  - prueba `2`: replay sin Gazebo visual usando `ros2 launch orbslam3_server global_orb_map_server.launch.py` con `rawdb_replay_enabled:=true`, `rawdb_replay_period_sec:=0.05`, `fiducial_sim_enabled:=false`, `pose_store_debug_enabled:=false`, `f1g_full_snapshot_enabled:=false` y `f1g_debug_mark_optimized_kf:=true`.
- patrones usados para reducir logs:
  - prueba 1:
    ```text
    SCENARIO-RUNNER|GOAL|RESULT|success|F1G-FULL-SNAPSHOT|F1G-RAWDB|F1G-SNAPSHOT|F1G-RAW-POSE|F1G-POSESTORE|F1G-GLOBALMAP|F1F-GLOBALMAP|ERROR|FATAL|Segmentation fault|Killed
    ```
  - prueba 2:
    ```text
    SCENARIO-RUNNER|GOAL|RESULT|success|F1G-REPLAY|F1G-FULL-SNAPSHOT|F1G-RAWDB|F1G-SNAPSHOT|F1G-DEBUG|F1G-POSESTORE|KEEP-OPTIMIZED|REBASE-ANCHOR|F1C-REPLAY|ERROR|FATAL|Segmentation fault|Killed
    ```
- evidencia positiva encontrada:
  - `prueba_1`: `SIM-DONE prueba=1 success=true`;
  - `prueba_1`: 5 resultados `SCENARIO-RUNNER-GOAL-RESULT ... success=true`;
  - `prueba_1`: 8 `[F1G-FULL-SNAPSHOT-RX]`, 8 `[F1G-RAWDB-INSERT-FULL]` y 8 `[F1G-POSESTORE-RECONCILE-SUMMARY]`;
  - `prueba_1`: snapshot de `drone_id=1 epoch=1` con `new_kfs=3 updated_kfs=24 new_mps=379 updated_mps=1611 raw_pose_changed=23`;
  - `prueba_1`: snapshot de `drone_id=2 epoch=1` con `new_kfs=0 updated_kfs=27 updated_mps=1863 raw_pose_changed=26`;
  - `prueba_1`: aparecen `F1G-POSESTORE-REBASE-ANCHOR` y `F1G-POSESTORE-KEEP-OPTIMIZED`;
  - el record nuevo queda en `codex/archivos_auxiliares/repeticiones/rawdb_prueba_1.record` con tamaño aproximado `472M`;
  - `prueba_2`: `SIM-DONE prueba=2 success=true`;
  - `prueba_2`: `[F1C-REPLAY-LOAD] ... entries=368 deltas=356 full=12 submaps=4 kfs=225 mps=26165 fiducial_observations=74`;
  - `prueba_2`: 12 `[F1G-REPLAY-FULL-SNAPSHOT]`, 12 `[F1G-RAWDB-INSERT-FULL]` y 12 `[F1G-POSESTORE-RECONCILE-SUMMARY]`;
  - `prueba_2`: `[F1C-REPLAY-DONE] entries=368 journal=368 deltas=356 full=12 fiducial_observations=74 submaps=4 kfs=225 mps=26165`.
- evidencia negativa o ausente:
  - la primera grabación intermedia de `3G` contenía un snapshot vacío/no inicializado y se descartó; el código quedó corregido para ignorar ese caso;
  - al primer intento de replay le faltó espacio de disco para escribir `prueba_2.log`; se limpió solo logs temporales antiguos/parciales dentro de `codex/archivos_auxiliares` y se repitió correctamente;
  - en `prueba_1` aparece `gazebo ... exit code 255` durante el cierre posterior a `SIM-DONE success=true`, patrón no bloqueante ya observado;
  - no aparecieron `FATAL`, `Segmentation fault` ni `Killed`;
  - `3G` no implementa optimización real, loops ni fused landmarks.
- documentación actualizada:
  - `codex/contexto/00_LEER_PRIMERO.md`;
  - `codex/contexto/01_ESTADO_ACTUAL.md`;
  - `codex/contexto/04_TOPICS_SERVICES_ACTIONS.md`;
  - `codex/contexto/06_MAPA_CODIGO.md`;
  - `codex/contexto/07_ULTIMA_SESION.md`;
  - `codex/contexto/paquetes/orbslam3_msgs/orbslam3_msgs.md`;
  - `codex/contexto/paquetes/orbslam3_multi/orbslam3_multi.md`;
  - `codex/contexto/paquetes/orbslam3_multi/raw_map_database.md`;
  - `codex/contexto/paquetes/orbslam3_multi/global_pose_store.md`;
  - `codex/contexto/paquetes/orbslam3_server/orbslam3_server.md`;
  - `codex/contexto/paquetes/orbslam3_server/global_map_server.md`;
  - `codex/contexto/paquetes/orbslam3_server/launches.md`;
  - `codex/pipeline/fase_3_sparse_global/pipeline_fase_3.md`;
  - `codex/pipeline/fase_3_sparse_global/subfases/subfase_3G.md`;
  - `codex/pipeline/fase_3_sparse_global/subfases/subfase_3H.md`.
- conclusión: `CONSEGUIDA`.
- siguiente paso recomendado: ejecutar `subfase_3H.md` para medir una segunda visita fiducial y crear una tarea de optimización fiducial sin optimizar todavía.

## 2026-08-12 - Subfase 3G - Preparacion de la reimplementacion

- objetivo conversado: rehacer snapshots como reconciliacion diferencial,
  selectiva y diferida sobre el runtime reconstruido hasta 3F;
- acuerdo cerrado:
  - snapshots y deltas comparten FIFO, `arrival_id` y `PrimaryWorker`;
  - `RawMapDatabase` devuelve categorias precisas para pose, asociaciones,
    geometria, score e invalidaciones;
  - snapshot no-op termina y snapshot material actualiza solo las autoridades
    afectadas;
  - `GlobalMapBuilder` acumula dirty, pero espera al siguiente delta normal para
    consultar caches, construir y publicar;
  - los elementos activos ausentes se invalidan sin borrarse;
  - el `.record` permanece delta-only: el efecto material de un snapshot se
    guarda como delta normalizado y el snapshot completo nunca se persiste;
  - una prueba debug one-shot omitira un delta live para demostrar que el
    snapshot lo recupera;
  - live llevara ambos drones al fiducial 2; replay reconstruira sin Gazebo el
    mismo estado final desde deltas originales y normalizados;
  - el grafo añade tres aristas snapshot y conserva el layout 3F aceptado;
- archivos modificados: contratos y documentación de estado; no se modificó
  código, launch, YAML ni configuración;
- paquetes compilados: ninguno;
- pruebas ejecutadas: ninguna;
- conclusión: `PREPARACION CERRADA`, implementación pendiente de autorización
  explícita posterior.

## 2026-08-12 17:30 - Subfase 3G - Implementacion y validacion focal

- objetivo intentado: implementar full snapshots diferenciales y diferidos,
  preservar autoridad world, mantener el record delta-only y ampliar la
  observabilidad web sin alterar el layout 3F aceptado;
- archivos modificados:
  - tipos, base raw, pose store, score, builder y backend de `orbslam3_multi`;
  - `PrimaryQueue`, `global_map_server.cpp` y launch de `orbslam3_server`;
  - launch multi-dron, bridge, definicion web y pruebas de `simulacion_dron`;
  - YAMLs `tray_prueba_98.yaml` y `tray_prueba_99.yaml`;
- cambios principales:
  - `RawInsertResult` granular e `InsertFullSnapshot()` con invalidacion sin
    borrado, no-op sin journal y delta material normalizado;
  - pose world aceptada preservada mediante base raw y correccion separadas;
  - dirty snapshot acumulado en `GlobalMapBuilder` hasta el siguiente delta;
  - misma FIFO/worker para `delta/full_snapshot`, scheduler 35/35, una request
    en vuelo, control por backpressure y drop one-shot de test;
  - grafo 11/18 con `snapshot`, `full commit` y `snapshot reconcile`;
- paquetes compilados: `orbslam3_multi orbslam3_server simulacion_dron`;
- resultado de build: exit 0. Un primer test focal fallo por una referencia a
  vector invalidada dentro de la propia fixture; se corrigio reservando
  capacidad, se recompilo y no reaparecio;
- tests: 29/29 C++ correctos y 8/8 del contrato web correctos;
- pruebas Gazebo/replay: preparadas 98 live y 99 replay;
- evidencia negativa o ausente: todavia no habia validacion visual del usuario;
- conclusion: implementacion focal `CONSEGUIDA`, subfase aun `EN VALIDACION`;
- siguiente paso recomendado: ejecutar live 98 y replay 99.

## 2026-08-12 17:33 - Subfase 3G - Prueba 98 live con perdida recuperada

- objetivo intentado: omitir exactamente un delta live, recuperarlo por full
  snapshot, anclar ambos drones en fiducial 2 y comprobar que el snapshot no
  construye ni publica por si mismo;
- launch: `multi_dron.launch.py` con record activo, snapshots 35/35 y drop
  one-shot para `drone_id=1`;
- escenario: espera de tracking, ambos drones a fiducial 2, espera 40 s, ambos
  a `x=-8` y observacion final de 35 s;
- resultado: scenario exit 0, herramienta exit 0 y `SIM-DONE success=true`;
- patrones de reduccion: `SIM-|SCENARIO-RUNNER|GOAL|F3G-|F3E-FID-FIRST-ANCHOR|F3F-GLOBALMAP-PUBLISH|F3C-RECORD-SAVE|F3C-PRIMARY-SHUTDOWN|ERROR|FATAL|Segmentation fault|Killed`;
- evidencia positiva:
  - `[F3G-DEBUG-DELTA-DROPPED]` para dron 1 y snapshot inmediato material;
  - snapshots con `builder_executed=false publish=false` y deltas posteriores
    con `[F3G-DIRTY-CONSUMED-BY-DELTA]`;
  - dos `[F3E-FID-FIRST-ANCHOR]`, 103 poses, 94 activas y 2 hard;
  - `[F3G-POSE-WORLD-PRESERVED]` tras snapshots de submapas ya anclados;
  - record de 54 entradas, 2 submapas, 103 KFs, 10938 MPs y 12 observaciones;
  - ultima vista: 6264 puntos y 94 KFs;
- evidencia negativa o ausente:
  - el usuario informo posteriormente de bloqueos perceptibles del ordenador
    durante la simulacion, por lo que la ejecucion no es aceptable como prueba
    live de rendimiento/operacion aunque el escenario terminara;
  - tras la prueba, el sistema mostraba 13 GiB de RAM, solo 2,2 GiB disponibles
    y los 2 GiB de swap completamente ocupados; no hubo OOM-kill del kernel;
  - tareas snapshot 45 y 47 tardaron aproximadamente 4,50 s y 4,81 s, y el
    delta final unos 4,10 s;
  - la inspeccion focal encontro una copia completa de `response->map`, otra
    copia completa transitoria al crear el delta normalizado y copias por cada
    registro raw modificado; ambos drones solicitan snapshots a la vez;
  - la confirmacion visual de RViz2/grafo no pudo considerarse fiable;
  - tras `SIM-DONE` aparecieron un fallo URDF de cleanup y la salida del bridge
    por contexto ROS invalidado, sin ser la causa del bloqueo durante runtime;
- conclusion de prueba revisada: `NO CONSEGUIDA` como validacion live integral;
  la correccion funcional queda demostrada, pero el consumo de recursos es una
  regresion bloqueante para cerrar 3G;
- siguiente paso recomendado: eliminar copias grandes, limitar picos de
  snapshots, instrumentar RSS/latencia y repetir live antes de cerrar 3G.

## 2026-08-12 17:39 - Subfase 3G - Prueba 99 replay delta-only

- objetivo intentado: reconstruir el estado final desde el record de la prueba
  98 sin wrappers, snapshots, GT live ni Gazebo;
- launch: `global_orb_map_server.launch.py`, `use_sim_time=false`, snapshots y
  fiducial simulado deshabilitados, delay de replay 20 ms;
- resultado: scenario exit 0, herramienta exit 0 y `SIM-DONE success=true`;
- patrones de reduccion: `SIM-|SCENARIO-RUNNER|F3C-REPLAY|F3C-PRIMARY|F3C-RAW|F3E-FID|F3F-SCORE|F3F-GLOBALMAP|F3G-|ERROR|FATAL|Segmentation fault|Killed`;
- evidencia positiva:
  - carga y proceso completos de 54 entradas, todas `source=replay kind=delta`;
  - no aparecen eventos runtime `F3G-SNAPSHOT` ni `kind=full_snapshot`;
  - `max_active=1`, cola drenada y 54 tareas correctas;
  - mismo estado final live: 2 submapas, 103 KFs, 10938 MPs, 2 anchors, 94
    poses activas y 2 hard;
  - misma ultima vista: 6264 puntos y 94 KFs;
- evidencia negativa o ausente: no hubo errores runtime. La equivalencia se
  comprobo por agregados/revisiones finales y por el test unitario de estado
  exacto del delta normalizado, no mediante un hash criptografico runtime;
- conclusion de prueba: `CONSEGUIDA`;
- siguiente paso recomendado: incorporar la observacion visual de prueba 98 y
  fijar la conclusion agregada de 3G.

## 2026-08-12 18:21 - Subfase 3G - Baseline protegida y scheduler global

- objetivo intentado: instrumentar recursos, fijar una baseline segura y
  serializar snapshots de varios drones hasta fin de commit;
- cambios: monitor CSV/resumen con guarda anti-bloqueo; gate global, cola
  deduplicada y liberacion del mensaje antes de pedir el siguiente snapshot;
- build/tests: `orbslam3_server` exit 0; PrimaryQueue 5/5 y
  GroundTruthBuffer 3/3;
- prueba 100: abortada por guarda a 4 s, antes del escenario, con 975 MiB
  disponibles y swap 2047.8 MiB; valida la instrumentacion, no el scheduler;
- prueba 101: perfil sin RViz/web, abortado a 6 s con 738.7 MiB disponibles;
  servidor 35.5 MiB, ORB 612.8 MiB y proceso externo `code` 2058 MiB;
- prueba 102: guarda 512 MiB, abortada a 10 s con minimo 337.5 MiB, grupo
  1397.6 MiB, ORB 1006.8 MiB y servidor 35.3 MiB;
- evidencia funcional de prueba 102: dron 1 fue solicitado/recibido/comprometido
  y liberado antes de iniciar dron 2; el snapshot debug posterior uso el mismo
  gate. No coexistieron dos capturas completas;
- conclusion: scheduler global `CONSEGUIDO`; simulacion integral aun
  `NO CONSEGUIDA` por presion previa/externa al servidor;
- siguiente paso: eliminar copias completas y continuar con smokes protegidos.

## 2026-08-12 18:36 - Subfase 3G - Recepcion zero-copy y prueba 103

- objetivo intentado: eliminar la copia completa de `OrbMap` al recibir
  `GetOrbMap` y comprobarla en una simulacion comparable bajo guarda;
- cambio: el `shared_ptr` del mapa usa ownership aliasing sobre la respuesta
  ROS, de modo que conserva viva la respuesta sin ejecutar
  `make_shared<OrbMap>(response->map)`;
- build/tests: `orbslam3_server` exit 0; `test_primary_queue` 5/5 y
  `test_ground_truth_buffer` 3/3;
- prueba 103 intento 1: Gazebo murio con exit 255 durante el arranque. La guarda
  no salto; se preservaron log reducido, CSV y resumen con sufijo
  `_intento_1`. El intento no es una validacion integrada;
- prueba 103 intento 2: perfil intermedio con Gazebo, dos wrappers ORB y
  servidor, sin RViz2/web; la guarda detuvo limpiamente la ejecucion a los
  102 s con exit 125 y sin congelacion observada por la herramienta;
- patrones de reduccion: `SIM-GAZEBO|SIM-RESOURCE|SIM-DONE|SCENARIO-RUNNER|F3G-SNAPSHOT|F3C-PRIMARY|zero_copy_rx|ERROR|FATAL|Killed`;
- evidencia positiva:
  - todos los snapshots aceptados marcaron `zero_copy_rx=true`;
  - el snapshot periodico grande del dron 2 contenia 12 KFs y 1105 MPs y se
    proceso despues de liberar el del dron 1, sin dos mapas completos en vuelo;
  - el drop de prueba del dron 1 produjo un diff material y el snapshot
    periodico siguiente del mismo dron fue no-op;
  - PSI full de memoria e iowait maximo permanecieron en 0.00 y 0.75 %;
- evidencia negativa o ausente:
  - la guarda salto con `MemAvailable=485.8 MiB`; swap siguio practicamente
    llena (2047.9 MiB);
  - RSS maximo medido: grupo 1771.0 MiB, servidor 191.7 MiB y ORB agregado
    1197.6 MiB; el proceso externo `code` alcanzo 2098.8 MiB;
  - los picos no son comparables directamente con prueba 102 porque aquella
    termino a los 10 s y esta avanzo 102 s; el ahorro de la copia RX es una
    propiedad del ownership, no una reduccion aislada cuantificada;
  - el escenario no termino debido a la guarda y no se ejecuto RViz2/web;
- conclusion de prueba: optimizacion `zero-copy RX` `CONSEGUIDA`; validacion
  live integral `NO CONSEGUIDA` por margen global insuficiente;
- siguiente paso recomendado: eliminar la copia completa usada para construir
  el delta normalizado de snapshot y repetir el mismo perfil protegido.

## 2026-08-12 18:43 - Subfase 3G - Normalizacion selectiva y prueba 104

- objetivo intentado: impedir que `RawMapDatabase` copie un `OrbMap` completo
  para vaciarlo despues al construir un delta normalizado;
- cambio: `MakeNormalizedDeltaShell()` copia exclusivamente cabecera,
  identidad, secuencia e intrinsecos; solo los full snapshots crean ese shell y
  solo se añaden entidades materiales. Los deltas normales ya no crean un
  segundo mensaje temporal;
- build/tests: `orbslam3_multi` exit 0; RawMapDatabase 7/7 y
  SparseGlobalBackend/GlobalPoseStore 6/6. `ctest` no pudo escribir su log fuera
  de `src` por el sandbox, por lo que se ejecutaron los mismos binarios GTest
  directamente;
- prueba 104: mismo perfil protegido que prueba 103, sin RViz2/web; guarda a
  los 90 s, exit 125, sin bloqueo de la maquina;
- patrones de reduccion: `F3C-RECORD|F3C-PRIMARY-SHUTDOWN|F3C-RAW-COMMIT|SIM-RESOURCE-GUARD|SCENARIO-RUNNER-*|F3G-SNAPSHOT-*|ERROR|FATAL|Killed`;
- evidencia positiva:
  - snapshots con RX zero-copy, gate global y deltas normalizados correctos;
  - ambos goals al fiducial terminaron correctamente en 22 s;
  - maximos: servidor 171.7 MiB y grupo 1654.2 MiB, frente a 191.7 MiB y
    1771.0 MiB en prueba 103; ORB tambien fue menor, por lo que esta diferencia
    no se atribuye por completo al cambio;
  - PSI full de memoria 0.18, iowait 1.46 % y cero paginas de swap-out;
- evidencia negativa o ausente:
  - minimo disponible 508.1 MiB y swap 2047.8 MiB; la guarda impidio terminar
    el escenario y no se validaron RViz2/web;
  - a tiempos iguales el RSS del servidor no fue uniformemente menor porque
    las ejecuciones ORB generaron mapas distintos; la eliminacion de la copia
    queda demostrada por el codigo y tests, no como ahorro RSS aislado exacto;
  - el journal alcanzo 146 entradas y el record 100 MiB en 90 s. Con drones ya
    quietos se seguian reteniendo deltas de unas 1337/1408 actualizaciones de
    MPs por segundo, identificando crecimiento lineal residente;
- conclusion de prueba: normalizacion selectiva `CONSEGUIDA`; live integral
  `NO CONSEGUIDA`, con el journal residente como siguiente cuello principal;
- siguiente paso recomendado: eliminar copias por entidad y asignaciones del
  diff; despues sustituir el journal residente por escritura incremental.

## 2026-08-12 18:47 - Subfase 3G - Comparacion raw por referencia y prueba 105

- objetivo intentado: eliminar la copia profunda del valor raw anterior por
  cada KF/MP modificado antes de clasificar su diff;
- cambio: los dos valores `old` son referencias const; pose, actividad,
  asociaciones, geometria y score se comparan antes de reemplazar la entrada;
- build/tests: `orbslam3_multi` exit 0; RawMapDatabase 7/7 y
  SparseGlobalBackend/GlobalPoseStore 6/6;
- prueba 105: mismo perfil protegido; guarda a 100 s, exit 125, sin bloqueo;
- evidencia positiva:
  - ambos goals iniciales terminaron en 22 s, se completo la espera de
    snapshots/anchors y se llego a iniciar el segundo movimiento;
  - coste raw de deltas estacionarios con al menos 1000 MPs: 86 muestras,
    media 11.303 ms y 8.113 ms por 1000 MPs; prueba 104 daba 80 muestras,
    11.191 ms y 8.173 ms/1000 MPs. La diferencia es pequena y queda dentro del
    ruido, pero no hay regresion medible;
  - PSI full de memoria 0.18 e iowait maximo 0.90 %;
- evidencia negativa o ausente:
  - guarda con 494.8 MiB disponibles; servidor 198.1 MiB, grupo 1692.0 MiB y
    ORB 1147.0 MiB;
  - el mayor avance produjo 162 entradas residentes, 95 KFs, 11790 MPs y un
    record de 130 MiB, por lo que el RSS mayor no contradice la eliminacion de
    la copia temporal;
  - hubo 83 paginas de swap-out y el escenario no termino ni activo RViz2/web;
- conclusion de prueba: comparacion por referencia `CONSEGUIDA`; live integral
  `NO CONSEGUIDA` por retencion lineal del journal;
- siguiente paso recomendado: dejar de construir conjuntos de IDs recibidos en
  cada delta normal, ya que solo se consultan para full snapshots.

## 2026-08-12 18:52 - Subfase 3G - IDs recibidos solo en snapshot y prueba 106

- objetivo intentado: eliminar dos `std::set` de miles de elementos que cada
  delta normal construia aunque solo los full snapshots consultan ausencias;
- cambio: los deltas no insertan IDs recibidos; los snapshots usan dos
  `unordered_set` reservados al tamaño de sus vectores;
- build/tests: `orbslam3_multi` exit 0; RawMapDatabase 7/7 y
  SparseGlobalBackend/GlobalPoseStore 6/6;
- prueba 106: mismo perfil protegido; guarda a 62 s, exit 125, sin bloqueo;
- evidencia positiva:
  - ambos goals iniciales terminaron en 22 s y los snapshots siguieron
    serializados y materiales;
  - 24 deltas estacionarios con al menos 1000 MPs dieron 9.038 ms de media y
    6.462 ms por 1000 MPs, frente a 11.303 ms y 8.113 ms/1000 MPs en prueba
    105. Es una mejora consistente cercana al 20 %, aunque no un benchmark
    determinista porque ORB genero mapas distintos;
  - servidor 165.1 MiB, PSI full 0.18 y cero paginas de swap-out;
- evidencia negativa o ausente:
  - la maquina partio con menos margen y la guarda salto con 465.5 MiB
    disponibles; ORB agregado era 1131.7 MiB y `code` externo 2129.8 MiB;
  - solo se procesaron 90 entradas antes del cierre, con 94 KFs/11347 MPs, por
    lo que los picos RSS no son comparables con pruebas mas largas;
  - no termino el escenario ni se activo RViz2/web;
- conclusion de prueba: IDs temporales acotados `CONSEGUIDO`; validacion live
  integral aun `NO CONSEGUIDA` por falta de margen y journal residente;
- siguiente paso recomendado: calcular las diferencias de asociaciones en una
  sola pasada con indices hash reutilizados, conservando salida ordenada.

## 2026-08-12 18:57 - Subfase 3G - Diff bidireccional y prueba 107

- objetivo intentado: reemplazar dos llamadas a `Difference()` que construian
  cuatro `std::set` por KF modificado;
- cambio: una sola pareja de indices hash clasifica añadidos y retirados; ambos
  resultados se ordenan/deduplican para conservar determinismo;
- build/tests: `orbslam3_multi` exit 0; RawMapDatabase 8/8, incluido nuevo test
  con IDs desordenados, y SparseGlobalBackend/GlobalPoseStore 6/6;
- prueba 107 intento 0: Gazebo murio temprano con exit 255, sin guarda y sin
  entradas raw; la herramienta limpio y aplico el reintento mecanico acordado;
- prueba 107 intento 1: ejecucion valida hasta guarda a 72 s, exit 125;
- evidencia positiva:
  - ambos goals al fiducial terminaron en 22 s y PrimaryWorker mantuvo
    `max_active=1`;
  - 44 deltas estacionarios: 11.371 ms de media y 7.154 ms/1000 MPs. Queda
    entre prueba 106 (6.462) y prueba 105 (8.113), sin regresion y con variacion
    propia de mapas ORB no deterministas;
  - PSI full maximo 0.18 y cero paginas de swap in/out;
- evidencia negativa o ausente:
  - guarda con 497.2 MiB disponibles; ORB agregado 1212.6 MiB, servidor
    195.8 MiB y grupo 1764.3 MiB;
  - se retuvieron 111 entradas, 3 submapas, 136 KFs y 11676 MPs; el escenario
    no termino ni activo RViz2/web;
- conclusion de prueba: diff bidireccional `CONSEGUIDO`; live integral sigue
  `NO CONSEGUIDA` y requiere eliminar el journal residente;
- siguiente paso recomendado: implementar persistencia incremental delta-only
  con memoria acotada y compatibilidad de lectura de records v1/v2.

## 2026-08-12 19:08 - Subfase 3G - Record incremental y prueba 108

- objetivo intentado: eliminar el crecimiento lineal de RAM causado por
  retener todos los `shared_ptr<OrbMap>` del record hasta el destructor;
- cambios:
  - `RawMapDatabase` dispone de modos `InMemory`, `Disabled` e `Incremental`,
    fijados antes del primer delta;
  - live con record escribe cada delta v2 en `<path>.in_progress`; al cierre
    añade observaciones, actualiza el contador y renombra sobre el destino;
  - live/replay sin record conserva contadores logicos pero cero mensajes
    historicos residentes;
  - `F3C-RAW-COMMIT` expone `resident_journal` y `record_bytes`;
- build/tests: `orbslam3_multi` y `orbslam3_server` exit 0; RawMapDatabase
  10/10, backend/poses 6/6, PrimaryQueue 5/5 y GroundTruthBuffer 3/3. Los tests
  verifican v2 legible, fiduciales, cero retencion y preservacion de un record
  previo hasta el rename final;
- prueba 108: perfil Gazebo protegido, guarda a 47 s, exit 125, sin bloqueo;
- evidencia positiva:
  - 38/38 commits auditados con `resident_journal=0`;
  - record final valido de 36 entradas, 17 fiduciales y 66,109,814 bytes;
  - servidor entre aproximadamente 31 y 70 MiB; maximo 72.1 MiB frente a
    165-198 MiB en los smokes anteriores con journal residente;
  - a 40 s servidor 57.1 MiB con 9815 MPs, frente a 64.0-87.2 MiB en pruebas
    104-107; el record crece en disco y no en el heap;
  - PSI full 0.18 y cero paginas de swap in/out;
- evidencia negativa o ausente:
  - la maquina partio con menos margen y la guarda salto con 497.7 MiB
    disponibles; `code` externo alcanzo 2169.2 MiB y ORB 1089.3 MiB;
  - el escenario no termino ni activo RViz2/web;
- conclusion de prueba: persistencia incremental y memoria del journal
  `CONSEGUIDAS`; live integral aun `NO CONSEGUIDA` por margen global del host;
- siguiente paso recomendado: reproducir el artefacto real para demostrar
  compatibilidad de carga y estado final.

## 2026-08-12 19:10 - Subfase 3G - Replay del stream incremental, prueba 109

- objetivo intentado: cargar y ejecutar el record v2 producido por prueba 108;
- launch: `f3f_replay.launch.py`, sin Gazebo, ORB, RViz2 ni navegador; delay
  5 ms y monitor de recursos;
- resultado: herramienta exit 0, `SIM-DONE success=true`, sin guarda;
- evidencia positiva:
  - carga y proceso 36/36, todos `source=replay kind=delta`, `max_active=1`;
  - estado final: 2 submapas, 87 KFs, 9815 MPs, 2 anchors, 87 poses activas,
    2 hard fiducials y 17 observaciones;
  - vista final: 6395 puntos y 87 KFs; score revision 36;
  - todos los commits de replay marcaron `resident_journal=0`;
  - minimo disponible 1988.1 MiB, servidor 113.9 MiB, grupo 212.2 MiB y PSI
    de memoria 0.00;
- evidencia negativa o ausente: el loader de replay aun carga el record entero
  y encola por adelantado; no afecta live, pero sera relevante para records
  largos y se incluye en el analisis de escalabilidad posterior;
- conclusion de prueba: replay del record incremental `CONSEGUIDO`;
- siguiente paso recomendado: reducir churn material de score/dirty y despues
  repetir live con el journal ya acotado.

## 2026-08-13 - Subfase 3G - Score material y prueba 110

- objetivo intentado: impedir que cambios de entradas ORB que conservan el
  mismo score publico ensucien `GlobalMapBuilder` y eleven `score_revision`;
- cambios:
  - `ScoreChangeSet` separa `input_updated_ids` de los cambios materiales;
  - `LandmarkScoreManager` actualiza trazabilidad interna, pero solo considera
    material un cambio de score o de `is_bad`;
  - el servidor emite `raw -> score` para cambios de store y
    `score -> builder` exclusivamente para cambios visibles;
- build/tests: `orbslam3_multi` y `orbslam3_server` exit 0; score 3/3,
  backend/poses 6/6, builder 3/3, PrimaryQueue 5/5 y GroundTruthBuffer 3/3,
  total 20/20;
- prueba 110: perfil Gazebo protegido; la guarda se activo a 11 s, antes del
  escenario, y la herramienta termino con exit 125 sin bloquear el equipo;
- evidencia positiva:
  - llegada 2: 119 inputs score modificados, 116 cambios materiales;
  - llegada 3: 124 inputs score modificados, 118 cambios materiales;
  - 9 de 243 entradas internas quedaron correctamente fuera del dirty visible;
  - stream incremental finalizado con 3 entradas y journal residente cero;
- evidencia negativa o ausente:
  - minimo disponible 480.3 MiB, servidor 35.6 MiB, ORB agregado 1009.2 MiB,
    grupo 1312.9 MiB y proceso externo `code` 2210.4 MiB;
  - swap ya llena al arrancar; 14 paginas de swap-in y 69 de swap-out, PSI
    full maximo 0.32;
  - no se alcanzo el escenario, los anchors ni una ventana suficiente para
    comparar latencia o RSS sostenidas;
- conclusion de prueba: separacion semantica de score `CONSEGUIDA`; prueba de
  rendimiento live `PARCIAL` por presion global previa del host;
- siguiente paso recomendado: evitar que `GlobalMapBuilder` recorra dirty de
  submapas sin ancla y hacer backfill completo cuando aparezca el primer anchor.

## 2026-08-13 - Subfase 3G - Diferimiento pre-anchor y prueba 111

- objetivo intentado: sustituir el recorrido repetido de miles de dirty de
  submapas sin anchor por una marca acotada y backfill completo al anclarse;
- cambios:
  - `GlobalMapBuilder` conserva una marca por submapa no anclado y elimina sus
    IDs dirty tras contabilizarlos;
  - al detectar el primer anchor obtiene de `RawMapDatabase` todos los KFs y
    MPs activos, incluidos MPs no listados por un KF;
  - la telemetria expone contadores `deferred_*` y `backfilled_*`;
- build/tests: ambos paquetes exit 0; GlobalMapBuilder 3/3,
  RawMapDatabase 10/10 y backend/poses 6/6, total 19/19. El test de backfill
  incluye expresamente un MP no enumerado en `keyframe.mappoint_ids`;
- prueba 111: Gazebo y dos ORB, sin RViz/web, monitorizada; guarda a 10 s,
  exit 125, antes del scenario y antes del primer delta;
- evidencia positiva:
  - cierre protegido sin bloqueo ni paginas nuevas de swap in/out;
  - servidor maximo 35.8 MiB y record incremental finalizado limpiamente con
    journal residente cero;
- evidencia negativa o ausente:
  - no hubo deltas, anchors ni invocaciones del builder, por lo que el ahorro
    runtime no pudo medirse en este intento;
  - minimo disponible 366.6 MiB; ORB agregado 964.9 MiB, grupo 1308.8 MiB,
    `code` externo 2280.7 MiB y swap 2047.9 MiB;
  - el exit 255 de Gazebo fue posterior a la guarda y pertenece al cleanup;
- conclusion de prueba: correccion focal `CONSEGUIDA`; simulacion de
  rendimiento `NO CONCLUYENTE` por falta de margen previa al primer delta;
- siguiente paso recomendado: validar deferred/backfill sobre replay real sin
  ORB y continuar con lectura replay streaming/acotada.

## 2026-08-13 - Subfase 3G - Replay focal del builder, prueba 112

- objetivo intentado: validar en un record real el diferimiento pre-anchor y
  su backfill sin el coste de Gazebo ni de los dos procesos ORB;
- prueba: `f3f_replay.launch.py`, record de prueba 98 con 54 entradas, delay
  5 ms, sin Gazebo/RViz/browser, monitorizada;
- resultado: scenario y herramienta exit 0, `SIM-DONE success=true`, 38 s y
  sin guarda;
- evidencia positiva:
  - llegadas 1-35: cero MPs recalculados/publicados; hasta 2 submapas y 2063 MPs
    de una llegada quedaron resumidos en marcas diferidas;
  - entre `F3F-SCORE-UPDATE` y `F3F-BUILDER-SKIP` el tramo builder pre-anchor
    fue normalmente de aproximadamente 1-2 ms aun con mas de 1000 IDs;
  - llegada 36: primer anchor del dron 2 y backfill unico de 32 KFs/3057 MPs;
  - llegada 37: primer anchor del dron 1 y backfill unico de 43 KFs/3176 MPs;
  - estado final identico al esperado: 54/54, 2 submapas, 103 KFs, 10938 MPs,
    2 anchors, 94 poses activas y vista 6264 puntos/94 KFs;
  - RSS servidor 123.5 MiB, grupo 222.6 MiB, minimo disponible 1638.4 MiB,
    PSI memoria 0 y cero paginas de swap in/out;
- evidencia negativa o ausente:
  - el loader deserializa todavia el record completo y la cola llego a 23
    entradas pendientes; esta retencion escala con el tamaño del record;
- conclusion de prueba: diferimiento y backfill `CONSEGUIDOS`; queda abierto
  acotar memoria y profundidad del replay;
- siguiente paso recomendado: lector secuencial v1/v2 y alimentacion de replay
  con ventana acotada por capacidad de cola.

## 2026-08-13 - Subfase 3G - Replay streaming y Gazebo 113

- objetivo intentado: eliminar la carga completa del record y limitar la cola
  replay al high watermark sin polling;
- cambios:
  - primera pasada de metadata/fiduciales y segunda pasada que deserializa una
    sola entrada cada vez, compatible con v1/v2;
  - `PrimaryQueue::WaitUntilPendingBelow()` bloquea el feeder mediante
    `condition_variable` y un `pop` lo despierta;
  - `F3C-REPLAY-FEED-DONE` expone `max_pending` y capacidad;
- build/tests: ambos paquetes exit 0; raw 10/10, builder 3/3, backend/poses
  6/6 y queue/backpressure 6/6, total 25/25;
- prueba Gazebo 113: perfil protegido con dos ORB y sin UI; guarda a 9 s,
  exit 125, antes del scenario y antes del primer delta;
- evidencia positiva:
  - cierre protegido sin bloqueo; servidor maximo 35.6 MiB y record finalizado
    con cero entradas residentes;
- evidencia negativa o ausente:
  - minimo disponible 323.4 MiB, ORB 947.8 MiB, grupo 1250.8 MiB, `code`
    externo 2286.0 MiB y swap 2047.8 MiB;
  - hubo 5 paginas de swap-in y el dron 1 termino con exit -6 durante cleanup;
    Gazebo 255 tambien fue posterior a la guarda;
  - al no existir actividad replay ni deltas, Gazebo no mide el nuevo lector;
- conclusion de prueba: build/semantica focal `CONSEGUIDOS`; Gazebo
  `NO CONCLUYENTE` por presion previa del host;
- siguiente paso recomendado: repetir el mismo record de prueba 98 mediante el
  nuevo feeder y comparar `max_pending`/RSS contra replay 112.

## 2026-08-13 - Subfase 3G - Medicion replay streaming, prueba 114

- objetivo intentado: comparar el nuevo feeder con replay 112 usando el mismo
  record de 72 MB, delay 5 ms y perfil sin Gazebo/ORB/RViz/browser;
- resultado: scenario/herramienta exit 0, `SIM-DONE success=true`, 38 s y sin
  guarda;
- evidencia positiva:
  - metadata v2: 54 entradas y 12 observaciones; feed secuencial 54/54;
  - `max_pending=8`, exactamente la capacidad, frente a 23 observadas en 112;
  - estado final idéntico: 2 submapas, 103 KFs, 10938 MPs, 2 anchors, 94 poses
    activas, vista 6264 puntos/94 KFs y `max_active=1`;
  - RSS servidor 82.0 MiB frente a 123.5 MiB en 112: ahorro 41.5 MiB,
    equivalente a 33.6%; grupo 181.1 frente a 222.6 MiB;
  - minimo disponible 1569.9 MiB, PSI memoria 0 y cero swap in/out;
- evidencia negativa o ausente: la memoria final de bases/caches sigue siendo
  proporcional al mapa, como corresponde; se elimino solo la copia histórica
  y la cola no acotada;
- conclusion de prueba: replay streaming/bounded `CONSEGUIDO`;
- siguiente paso recomendado: medir escalabilidad sintetica con mas de dos
  submapas y localizar costes por entidad antes de decidir mejoras adicionales.

## 2026-08-13 - Subfase 3G - Escalabilidad 2/4/8 y Gazebo 115

- objetivo intentado: medir backend con mas de dos drones sin confundirlo con
  el coste de lanzar una instancia ORB completa por dron;
- benchmark determinista: 50 KFs y 5000 MPs por submapa; insert inicial,
  diferimiento sin anchors, anclaje y un delta geometrico completo posterior
  por dron; cada escala se ejecuto en un proceso independiente;
- resultados:
  - 2 drones/10k MPs: RSS 22.1 MiB, insert 130.7 ms, defer 7.4 ms,
    anchors/build 737.0 ms, updates 707.4 ms, 353.7 ms por dron;
  - 4 drones/20k MPs: RSS 33.7 MiB, insert 278.8 ms, defer 15.3 ms,
    anchors/build 1435.2 ms, updates 1366.1 ms, 341.5 ms por dron;
  - 8 drones/40k MPs: RSS 56.4 MiB, insert 562.9 ms, defer 31.9 ms,
    anchors/build 2886.6 ms, updates 2815.3 ms, 351.9 ms por dron;
  - las tres escalas pasaron conteos exactos y el coste por delta-dron se
    mantuvo estable: backend casi lineal hasta ocho submapas;
- prueba Gazebo 115: obligatoria tras añadir instrumentacion; guarda a 10 s,
  exit 125 y cero deltas antes del scenario;
- evidencia de recursos Gazebo:
  - servidor 35.4 MiB frente a ORB agregado 1003.7 MiB; grupo 1308.0 MiB;
  - minimo disponible 378.9 MiB, `code` externo 2281.1 MiB, swap llena,
    259 paginas swap-out y PSI full maximo 0.55;
  - cierre protegido sin bloqueo; Gazebo 255 posterior a la guarda;
- conclusion: escalabilidad backend >2 drones `CONSEGUIDA`; live Gazebo
  `NO CONCLUYENTE` y actualmente limitado antes del pipeline por memoria de
  los frontends ORB y estado global del host;
- siguiente paso recomendado: reducir coste sostenido por MP si aporta margen
  para fases futuras y separar explicitamente presupuesto servidor/frontends.

## 2026-08-13 - Subfase 3G - Batch ligero de score y Gazebo 116

- objetivo intentado: eliminar una copia profunda de `OrbMapPoint` y un lock
  raw por cada candidato de score;
- cambios: `GetMapPointScoreInputs()` extrae en un unico lock solo contador,
  ratio, validez de descriptor e `is_bad`; `LandmarkScoreManager` conserva la
  misma formula y semantica material;
- build/tests: ambos paquetes exit 0; score 3/3, raw 10/10, backend/poses 6/6,
  builder 3/3 y escalabilidad 3/3;
- benchmark:
  - con 8 drones/40k MPs, los ocho deltas completos tardaron 1987 ms;
  - insert raw + score fueron 66.4 ms y builder 1920.6 ms: el builder representa
    96.7% del coste restante;
  - el total fue 29% menor que la muestra anterior de 2815 ms, aunque 2/4
    drones mostraron variacion al alza; no se atribuye toda la diferencia al
    batch sin una serie estadistica;
- prueba Gazebo 116: guarda a 10 s, exit 125, cero deltas antes del scenario;
  servidor 35.7 MiB, ORB 974.8 MiB, grupo 1338.1 MiB, minimo disponible
  387.6 MiB, PSI full 0.30 y cero swap in/out;
- conclusion: batch score `CONSEGUIDO`; Gazebo `NO CONCLUYENTE` por presion
  preexistente y memoria frontend antes de ejercitar el cambio;
- siguiente paso recomendado: batch ligero de geometria/KFs para
  `GlobalMapBuilder`, actual cuello del 96.7%.

## 2026-08-13 - Subfase 3G - Snapshot ligero builder y Gazebo 117

- objetivo intentado: eliminar `GetMapPoint/GetKeyFrame` por entidad y las
  copias completas de mensajes ORB dentro del builder;
- cambios: `RawBuilderSnapshot` expande dirty/asociaciones bajo un lock y
  conserva solo pose/validez de KFs y geometria/referencia/observadores de MPs;
  un lote suplementario preserva IDs ensuciados al retirar un KF cacheado;
- build/tests: ambos paquetes exit 0; builder 3/3, raw 10/10, backend/poses 6/6,
  score 3/3 y escala 3/3;
- benchmark:
  - en la primera serie, builder 2/4 drones bajo 720->592 ms (-17.7%) y
    1428->1201 ms (-15.9%); RSS temporal aumento 1.3-1.6 MiB;
  - cinco repeticiones de 8 drones dieron 2417.8, 1524.5, 1487.2, 1629.4 y
    1550.8 ms; mediana 1550.8 ms, 19.3% menor que la muestra previa de
    1920.6 ms, con un outlier por presion variable del host;
  - todos los conteos finales permanecieron exactos en cada repeticion;
- prueba Gazebo 117: guarda a 10 s antes de deltas, exit 125; servidor 35.9
  MiB, ORB 1014.8 MiB, grupo 1325.3 MiB, minimo disponible 363.0 MiB, PSI
  full 0.30 y 3 paginas swap-in;
- conclusion: snapshot ligero `CONSEGUIDO`; Gazebo `NO CONCLUYENTE` por el
  mismo limite frontend/host anterior al pipeline;
- siguiente paso recomendado: cachear una transformada validada por KF para no
  repetir locks de pose, conversiones e inversas por cada MP.

## 2026-08-13 - Subfase 3G - Cache de proyeccion y pruebas 118/119

- objetivo intentado: validar cada KF una vez por build y evitar recomputar
  `world_T_kf * inverse(local_T_kf)` por cada MP;
- cambios: cache persistente invalidada por cambios local/world y sets
  `usable/unusable` por `Update`; retirar un KF elimina tambien su proyeccion;
- build/tests: ambos paquetes exit 0; builder 3/3, backend/poses 6/6, raw 10/10
  y score 3/3. El test de KF movido demuestra invalidacion correcta;
- benchmark de 8 drones, cinco procesos:
  - builder: 594.5, 603.1, 613.4, 553.2 y 397.8 ms; mediana 594.5 ms;
  - frente a mediana snapshot 1550.8 ms: -61.7%;
  - update total mediano 692.1 ms, 86.5 ms por delta-dron frente a baseline
    inicial 351.9 ms: -75.4%; conteos exactos y RSS 57.3 MiB;
- Gazebo 118: guarda a 10 s, exit 125; alcanzo un unico delta de 452 MPs y lo
  difirio correctamente pre-anchor. Servidor 35.9 MiB, ORB 964.3 MiB, grupo
  1242.7 MiB y minimo disponible 403.3 MiB;
- replay 119, mismo record que 114:
  - 54/54, `max_pending=8`, 2 anchors, 103 KFs, 10938 MPs, 94 poses activas y
    vista final 6264/94, sin errores;
  - load->done 2.22 s frente a 5.91 s en 114: -62.4%;
  - backfills 36/37 aproximadamente 76/87 ms frente a 308/331 ms;
  - servidor 83.4 MiB frente a 82.0 MiB: coste de cache +1.4 MiB; PSI 0;
- conclusion: cache de proyeccion `CONSEGUIDA`; rendimiento backend ofrece
  margen amplio para mas drones/fases. El bloqueo live restante es de startup
  Gazebo/frontends/host, anterior al pipeline;
- siguiente paso recomendado: reducir pico de arranque mediante perfil
  headless y/o escalonado, conservando el launch visual por defecto.

## 2026-08-13 - Subfase 3G - Perfil de launch y prueba 120

- cambio aislado: `multi_dron.launch.py` incorpora `launch_gazebo_gui`,
  `launch_mission_gui` y `drone_start_stagger_sec`; con GUI desactivada usa
  `gzserver`, y permite escalonar los grupos de cada dron;
- build de `simulacion_dron`: correcto;
- prueba 120 headless: `NO CONSEGUIDA` por una ruta relativa al YAML que el
  runner resolvio desde el workspace padre; el launch si arranco, proceso 37
  deltas y no mostro un fallo funcional del perfil;
- recursos observados antes del error: minimo disponible 525.1 MiB, grupo
  1660.3 MiB RSS, ORB 1079.3 MiB y servidor 37.7 MiB;
- aprendizaje: usar siempre ruta absoluta en pruebas lanzadas desde la
  herramienta; esta ejecucion no demuestra margen suficiente.

## 2026-08-13 - Subfase 3G - Perfil headless completo y prueba 121

- cambio nuevo: ninguno; repeticion correcta del perfil de launch con ruta
  absoluta y escenario live completo;
- resultado: guarda de recursos a los 46 s, minimo disponible 507.3 MiB,
  ORB 1102.8 MiB, Gazebo 379.3 MiB y servidor 48.7 MiB;
- conclusion: desactivar GUIs por si solo es insuficiente; el cuello es la
  memoria privada de los frontends ORB, no RViz2 ni el grafo web.

## 2026-08-13 - Subfase 3G - Medicion PSS y prueba 122

- cambio aislado: el monitor añade PSS de grupo, servidor, ORB, Gazebo, RViz2
  y web sin retirar las columnas RSS anteriores;
- prueba corta headless conseguida: minimo disponible 543.7 MiB, grupo
  RSS/PSS 1731.7/1444.4 MiB, ORB RSS/PSS 1063.8/974.9 MiB, Gazebo PSS 340.4
  MiB y servidor PSS 18.1 MiB;
- conclusion: RSS estaba sobrecontando memoria compartida, pero el PSS confirma
  aproximadamente 975 MiB reales en solo dos procesos ORB.

## 2026-08-13 - Subfase 3G - Arenas glibc y prueba 123

- cambio aislado: los nodos mono/estereo reciben `MALLOC_ARENA_MAX=2`;
- build de `dron_individual`: correcto;
- prueba conseguida: PSS ORB estable 973.5->969.0 MiB, ahorro aproximado de
  4.5 MiB (0.5%);
- conclusion: se conserva por ser inocuo y reversible, pero no resuelve la
  duplicacion principal.

## 2026-08-13 - Subfase 3G - Camara a 20 Hz y pruebas 124/125

- cambio aislado: `sensores.camara.publish_rate` pasa de 30 a 20 Hz para
  coincidir con `Camera.fps=20`;
- build de `dron_individual`: correcto;
- prueba 124: `NO CONSEGUIDA`; Gazebo no arranco porque su puerto estaba
  ocupado por un proceso anterior. No se atribuye al cambio;
- prueba 125 repetida tras el cierre normal del proceso: conseguida;
- resultado estable frente al perfil de 30 Hz: CPU de sistema 35.7->27.8%
  (-22.1%) y CPU del grupo 367.9->306.9% (-16.6%), sin ahorro material de PSS;
- conclusion: 20 Hz elimina imagenes que ORB no necesitaba procesar y se
  conserva.

## 2026-08-13 - Subfase 3G - Clasificacion de heap ORB y prueba 126

- cambio aislado: el monitor añade `Pss_Anon`, `Pss_File` y `Pss_Shmem` para
  los procesos ORB;
- la guarda se activo a los 29 s por el margen global del host, pero la muestra
  es valida para clasificar memoria: PSS ORB 969.6 MiB, de los que 940.7 MiB
  son anonimos, 34.8 MiB fichero y 1.4 MiB compartidos;
- conclusion: no era cache del archivo de vocabulario ni memoria compartida;
  cada frontend materializaba privadamente el arbol DBoW2 completo.

## 2026-08-13 - Subfase 3G - Vocabulario ORB compacto y pruebas 127/128

- cambio aislado: se crea `generate_compact_orb_vocabulary.py` y se genera
  `ORBvoc_L5.txt`, 111078 nodos/99969 palabras y 15.8 MB, frente a 145.3 MB del
  vocabulario completo L6. El generador remapea padres y convierte nivel 5 en
  hojas con peso IDF medio validado;
- `generar_dron.launch.py` permite elegir `orb_vocabulary_path`; el launch
  multi-dron usa L5 por defecto y el launch individual conserva L6 por defecto;
- build correcto;
- prueba 127 corta headless: PSS ORB 969.6->212.1 MiB (-78.1%), heap anonimo
  940.7->175.6 MiB; ambos vocabularios cargados y pipeline activo;
- prueba 128 live completa headless: 172 s, guarda inactiva, minimo disponible
  965.3 MiB, PSI full maximo 0.18, grupo PSS 896.1 MiB, ORB 357.0 MiB y
  servidor 87.7 MiB;
- funcionalidad: ambos drones alcanzaron tracking, se crearon dos anchors, el
  drop se recupero y los dirty de snapshot fueron consumidos por deltas;
- limite aceptado: L5 es el perfil de rendimiento para simulacion multi-dron;
  L6 sigue disponible para benchmarks de maxima fidelidad y debe compararse en
  futuras pruebas especificas de relocalizacion/loop antes de retirar esa via.

## 2026-08-13 - Subfase 3G - Validacion visual previa, pruebas 129/130

- cambio nuevo: ninguno; se midio el perfil visual completo con vocabulario L5
  y stagger explicito de 8 s;
- prueba 129 corta: Gazebo GUI, RViz2, bridge, pestaña web y GUI de mision
  estuvieron activos; minimo disponible 725.7 MiB, grupo PSS 554.4 MiB,
  ORB 209.6 MiB, RViz2 124.3 MiB y web 49.9 MiB;
- prueba 130 live completa: 176 s, guarda inactiva, minimo disponible 556.5
  MiB, PSI full 0.18, grupo PSS 739.2 MiB, ORB 366.6 MiB, servidor 85.8 MiB,
  RViz2 124.7 MiB y web 50.6 MiB;
- conclusion: funcionalmente correcta, pero el margen de 556.5 MiB sigue
  demasiado cerca de la guarda para sumar fases densas o un tercer dron con
  todas las interfaces abiertas.

## 2026-08-13 - Subfase 3G - Camara 480x360 y prueba 131

- cambio aislado: camaras simuladas 640x480->480x360, intrinsecos/baseline
  recalculados y `ORBextractor.nFeatures` 1000->900; Xacro y generador URDF
  propagan ancho y alto;
- build inicial: correcto;
- prueba 131, dos intentos: `NO CONSEGUIDA`; `generador_URDF` termino con
  `ParameterNotDeclaredException` porque se leian `sensores.camara.width` y
  `height` sin declararlos;
- diagnostico: fallo mecanico de integracion, no de ORB ni de calibracion;
  no repetir una ampliacion de parametros sin declarar defaults en el nodo.

## 2026-08-13 - Subfase 3G - Declaracion de dimensiones y pruebas 132/133

- correccion mecanica: se declaran `sensores.camara.width/height` con fallback
  640x480 antes de leerlos; build correcto;
- prueba 132 corta visual: ambos wrappers informan `camera_valid=true`,
  480x360 y baseline 0.057 m. Frente a 129, PSS ORB 207.3->186.8 MiB (-9.9%),
  CPU sistema 38.3->29.3% (-23.5%) y CPU grupo 248.2->194.0% (-21.8%);
- prueba 133 live visual completa: 176 s, guarda inactiva, minimo disponible
  612.3 MiB, PSI de memoria 0, grupo PSS 640.7 MiB, ORB 273.8 MiB, servidor
  58.7 MiB, RViz2 127.1 MiB y web 41.4 MiB;
- funcionalidad: dos submapas estables, dos anchors, 90 KFs, 9787 MPs, drop
  recuperado, snapshots diferidos y 174/174 entradas con `max_active=1`;
- conclusion: cambio conseguido; mejora simultaneamente CPU, memoria y
  estabilidad de epochs sin bajar de una resolucion util para esta fase.

## 2026-08-13 - Subfase 3G - Stagger seguro por defecto y prueba 134

- cambio aislado: `drone_start_stagger_sec` pasa de 0 a 8 s por defecto;
- build de `simulacion_dron`: correcto;
- prueba visual corta con el launch ordinario, sin pasar el argumento:
  conseguida; los grupos de dron 1 y 2 arrancaron separados, minimo disponible
  948.9 MiB, grupo PSS 541.2 MiB, ORB 188.3 MiB, RViz2 109.2 MiB y web 49.8
  MiB, guarda inactiva;
- conclusion: el uso normal hereda el escalonado; N drones arrancan a
  0, 8, 16... s sin cambiar su comportamiento posterior.

## 2026-08-13 - Subfase 3G - Tres drones reales, prueba 135

- cambio temporal: `dron.numero=3`, seguido de build de `simulacion_dron`;
- prueba headless con stagger 0/8/16 s y sin movimiento: conseguida;
- evidencia funcional: tres clientes `get_full_map`, tres calibraciones validas,
  tres modelos insertados y tres procesos ORB; el dron 3 entro en tracking 2,
  publico 20 deltas y el servidor termino con `max_active=1`;
- recursos: 55 s, guarda inactiva, minimo disponible 1124.0 MiB, PSI full 0.37,
  cero swap-in/out, grupo PSS maximo 837.4 MiB, ORB PSS 289.7 MiB, Gazebo PSS
  369.9 MiB y servidor PSS 18.9 MiB;
- zona estable desde 25 s: memoria disponible media 1312.6 MiB, CPU de sistema
  26.0%, CPU del grupo 299.8% y PSS de grupo 694.9 MiB;
- conclusion: escalabilidad live a mas de dos drones demostrada en perfil
  operativo headless; no se exigieron anchors por ser un smoke sin goals.

## 2026-08-13 - Subfase 3G - Restauracion y prueba 136

- cambio aislado: se restaura `dron.numero=2` y se recompila
  `simulacion_dron`;
- prueba con launch ordinario, Gazebo GUI, GUI de mision, RViz2, web y stagger
  por defecto: conseguida;
- ambos modelos y wrappers 480x360 validos; bridge 11/18, pestaña web y RViz2
  activos; el error Gazebo 255 aparece solo tras `SIM-DONE`, durante cleanup;
- recursos: guarda inactiva, minimo disponible 975.6 MiB, PSI memoria 0, grupo
  PSS maximo 541.1 MiB, ORB 188.8 MiB, RViz2 111.4 MiB y web 49.9 MiB;
- durante la ventana plenamente activa: CPU sistema media 27.2%, grupo 197.8%
  y PSS grupo medio 538.2 MiB;
- conclusion: estado normal restaurado y validado, sin dejar el proyecto en
  configuracion experimental.

## 2026-08-13 - Subfase 3G - Regresion final y cierre de rendimiento

- build final: `orbslam3_multi`, `orbslam3_server`, `dron_individual` y
  `simulacion_dron`, 4/4 correctos;
- una primera invocacion con `--gtest_brief=1` solo mostro la ayuda porque esta
  version de GTest no soporta la opcion; no se conto como prueba;
- repeticion valida sin opciones: 37/37 tests C++ correctos, repartidos en raw
  10, backend 6, anchors 3, score 3, builder 3, escala 3, cola 6 y GT buffer 3;
- contrato del grafo web: 8/8 tests correctos;
- benchmark final 2/4/8: 84.3/85.5/89.3 ms por delta-dron, RSS
  23.4/34.4/57.0 MiB para 10k/20k/40k MPs; conserva escalado casi lineal;
- decision de suficiencia: no reducir mas resolucion, features ni profundidad
  de vocabulario sin una prueba de calidad que lo justifique. El backend deja
  margen para fases posteriores; para tres o mas drones y para fases dense se
  debe usar Gazebo headless y activar RViz2/web/GUI de mision solo cuando sean
  necesarios. El proceso externo `code` consumio aproximadamente 2.3 GiB en
  todas las mediciones y no pertenece al pipeline;
- conclusion agregada: incidencia de bloqueo resuelta y subfase 3G
  `CONSEGUIDA` tecnicamente. El perfil visual completo queda para desarrollo
  con dos drones; el perfil headless/selectivo es el perfil de escala.

## 2026-08-13 - Subfase 3G - Carga real con tres drones, prueba 137

- motivo de la prueba adicional: el smoke 135 demostro tres procesos y un
  tercer flujo activo, pero dos drones permanecieron estaticos; se exigio una
  evidencia mas fuerte antes del cierre definitivo;
- cambio temporal: `dron.numero=3` y escenario 137 con dos goals simultaneos
  por dron, carriles separados dentro del radio del fiducial 2 y snapshots
  periodicos activos; build de `simulacion_dron` correcto;
- resultado funcional: seis goals correctos de 24 s, tres first anchors, 36
  observaciones fiduciales, 3 anchors/3 hard, 166 poses totales/141 activas,
  vista final 7981 puntos/141 KFs y `fallback_submap=0`;
- el servidor proceso 254 entradas, termino `pending=0 active=0 max_active=1` y
  consumio dirty de snapshots mediante deltas posteriores;
- recursos durante 178 s: guarda inactiva, minimo disponible 878.8 MiB, PSI
  full maximo 0.18, PSS grupo maximo 1041.4 MiB, ORB 436.4 MiB, Gazebo 363.2
  MiB y servidor 82.7 MiB;
- ventana de carga activa: 997.6 MiB disponibles de media, CPU sistema 33.1%,
  CPU grupo 340.8% y PSS grupo 952.5 MiB. Las 281 paginas de swap-out ocurrieron
  en un unico pico de startup; durante la carga activa hubo cero swap-out;
- conclusion: escalabilidad funcional live a tres drones `CONSEGUIDA`, no solo
  escalabilidad de procesos. Mantener headless/selectivo para este perfil.

## 2026-08-13 - Subfase 3G - Restauracion definitiva, prueba 138

- cambio: `dron.numero` restaurado de 3 a 2; build `simulacion_dron` correcto;
- prueba con launch ordinario y todos sus defaults: Gazebo GUI, GUI de mision,
  RViz2, bridge/pestana web, snapshots, vocabulario L5 y stagger 8 s;
- resultado: dos clientes snapshot, dos calibraciones 480x360 validas, dos
  modelos insertados, bridge 11/18 y escenario correcto;
- recursos: guarda inactiva, minimo disponible 946.6 MiB, PSI memoria 0, PSS
  grupo 539.1 MiB, ORB 187.9 MiB, RViz2 105.3 MiB y web 41.5 MiB;
- conclusion: estado normal restaurado sin efectos laterales. Cierre 3G
  definitivo `CONSEGUIDO`.

## 2026-08-13 - Subfase 3G - Regresion visual solicitada, prueba 139

- objetivo intentado: comprobar despues de la optimizacion el recorrido visual
  de dos drones `fiducial 2 -> x=-8 -> fiducial 2`, con pausas para observar
  RViz2 y el grafo web;
- archivo creado:
  `codex/archivos_auxiliares/trayectorias/tray_prueba_139.yaml`; no se modifico
  codigo, launch ni configuracion permanente y no fue necesario recompilar;
- escenario: espera de tracking de 14 s, tres lotes simultaneos de dos goals,
  pausas de 25 s en fiducial 2 y en x=-8, y espera final de 60 s tras regresar;
- ejecucion: launch ordinario `multi_dron.launch.py`, monitor de recursos con
  guarda de 512 MiB, timeout 900 s y post-wait 10 s;
- resultado mecanico: los seis goals terminaron `success=true`, el runner y la
  herramienta devolvieron 0, `SIM-DONE success=true` y `guard_triggered=false`;
- evidencia de servidor: dos `F3E-FID-FIRST-ANCHOR`, final de 29 observaciones,
  2 anchors, 141 poses totales/120 activas y 2 hard; publicacion final de 6343
  puntos/120 KFs; cola final `pending=0 processed=269 active=0 max_active=1`;
- observabilidad automatica: el bridge respondio `/health=ready` con topologia
  11 nodos/18 aristas y el helper registro apertura correcta de
  `http://127.0.0.1:8765`; RViz2 renderizo MPs y KFs. Esto solo demuestra
  procesos/servidor, no usabilidad de las ventanas;
- observacion posterior del usuario: no aparecio la ventana de Gazebo; Chrome
  mostro `ERR_CONNECTION_REFUSED` para 127.0.0.1; RViz2 mostro MPs/KFs pero no
  permitio girar ni desplazar la vista. Por tanto fallaron los tres criterios
  visuales aunque el pipeline de mapa funcionase;
- diagnostico causal: la rama ejecutada fue `gazebo`, no `gzserver`; el comando
  visual es el mismo que antes salvo la condicion de perfil. Browser/bridge y
  `sparse_global_debug.rviz` no cambiaron frente a la baseline pre-rendimiento;
  RViz conserva `Orbit` y `MoveCamera`. La combinacion health local correcto,
  rechazo en Chrome y GUIs no utilizables apunta principalmente al entorno o
  aislamiento de la ejecucion, pero requiere una prueba visual aislada para
  confirmarlo;
- recursos durante 233 s y 182 muestras: minimo disponible 658.7 MiB, PSS
  maximo de grupo 698.8 MiB, ORB 312.4 MiB, servidor 78.2 MiB, RViz2 123.6
  MiB y web 50.4 MiB; PSI full maximo 0.32;
- log completo preservado y no leido en
  `codex/archivos_auxiliares/logs/prueba_139.log`; reducido en
  `prueba_139.reduced.log`. El reductor aviso `Broken pipe` por el corte de
  salida, pero genero correctamente el artefacto;
- unica incidencia grave aparente: Gazebo exit 255 durante cleanup, despues de
  `SIM-DONE`, igual que en pruebas anteriores;
- conclusion revisada: `NO CONSEGUIDA` como prueba visual integral; pipeline
  automatico conseguido y sin regresion del mapa, pero Gazebo/web/RViz2 no
  fueron utilizables. No considerar una GUI validada solo porque su proceso
  arranque o termine limpiamente.

## 2026-08-13 - Subfase 3G - Repeticion visual sin cambios, prueba 140

- objetivo: repetir exactamente el escenario de la prueba 139 mediante
  `multi_dron.launch.py`, sin cambios de codigo ni diagnostico posterior, para
  que el usuario comprobase de nuevo Gazebo, RViz2 y el grafo web;
- YAML reutilizado:
  `codex/archivos_auxiliares/trayectorias/tray_prueba_139.yaml`;
- ejecucion: monitor de recursos activo y guarda en 512 MiB, igual que en la
  prueba anterior; acceso grafico solicitado para el launch;
- resultado mecanico: la guarda se activo a los 146 s por
  `MemAvailable=485300 KiB`; scenario exit 125, herramienta exit 125 y
  `guard_triggered=true`. El escenario no termino y no se realizo diagnostico
  de logs por peticion expresa del usuario;
- recursos registrados: minimo disponible 463.8 MiB, PSS maximo de grupo
  627.3 MiB, ORB 275.1 MiB, servidor 63.0 MiB, RViz2 119.2 MiB, web 50.4 MiB
  y PSI full maximo 1.03;
- observacion del usuario: no vio activarse la arista `snapshot`; aprecio una
  diferencia notable de KFs entre ambos drones y posibles KFs retirados de
  RViz2;
- analisis tematico posterior autorizado por el usuario: se procesaron ocho
  snapshots, cuatro por dron, con pares alrededor de 35/70/105/140 s. El
  servidor emitio `wrapper_server_snapshot` para cada respuesta, pero el
  frontend resalta una arista solo 240 ms, por lo que el pulso es facil de
  perder;
- los snapshots confirmaron conteos raw activos diferentes: 36 KFs para dron 1
  y 59 para dron 2. Tambien registraron invalidaciones de KFs, entre ellas
  1/3/1 para dron 1 y 2/10 para dron 2 en distintas reconciliaciones. El
  builder publica solo KFs activos y usa markers `DELETE`; el dirty snapshot se
  consume en un delta posterior, por lo que la desaparicion no tiene por que
  coincidir visualmente con el pulso snapshot;
- la guarda es externa al pipeline ROS: vigila `MemAvailable` del host. Se
  configuro a 512 MiB y exige tres muestras consecutivas por debajo; no es el
  watermark 8/2 de la cola. La prueba alcanzo 485300 KiB al disparar y 463.8
  MiB como minimo, con la swap del host practicamente llena;
- conclusion: `NO CONSEGUIDA` mecanicamente por cierre preventivo de la guarda;
  snapshots y retirada selectiva de KFs funcionaron conforme al contrato, pero
  la telemetria web de 240 ms no permite una observacion fiable.

## 2026-08-13 - Subfase 3G - Repeticion con margen de RAM, prueba 141

- objetivo: repetir exactamente la trayectoria de 139/140, despues de que el
  usuario cerrase numerosas pestanas de Chrome, y comparar el consumo completo
  del ordenador;
- archivos modificados antes de ejecutar: ninguno de codigo, launch o
  configuracion; se reutilizo
  `codex/archivos_auxiliares/trayectorias/tray_prueba_139.yaml`;
- ejecucion: launch ordinario `multi_dron.launch.py`, timeout 900 s, post-wait
  10 s, monitor de recursos y guarda de 512 MiB con tres muestras consecutivas;
- baseline externa inmediatamente anterior: 7463 MiB usados, 5919 MiB
  disponibles y 1456 MiB de swap usada;
- resultado mecanico: los seis goals y el escenario terminaron, scenario exit
  0, herramienta exit 0, `SIM-DONE success=true` y
  `guard_triggered=false`; duracion monitorizada 233 s en 188 muestras;
- memoria del host: maximo usado 9008.4 MiB y minimo disponible 4826.3 MiB. La
  primera muestra del monitor tenia 7892.5/5942.2 MiB usados/disponibles y la
  ultima 7979.3/5855.4 MiB; tras el cierre, `free` midio 7491 MiB usados y
  5869 MiB disponibles;
- consumo atribuible al grupo de simulacion: PSS medio 652.5 MiB y maximo
  751.2 MiB. Maximos PSS por categoria: ORB 320.5 MiB, RViz2 121.5 MiB,
  servidor 77.8 MiB, web 50.4 MiB y Gazebo 39.1 MiB. El RSS agregado alcanzo
  1208.8 MiB, pero incluye paginas compartidas repetidas y no es la cifra
  adecuada para sumar consumo real;
- swap y presion: maximo 1455.5 MiB, esencialmente igual que antes de arrancar;
  cero paginas expulsadas a swap, solo 120 paginas recuperadas, y PSI de
  memoria `some/full=0.00/0.00`. No hubo thrashing;
- CPU/IO: CPU de sistema media/maxima 34.23/54.27 %, grupo ROS medio/maximo
  232.8/331.3 % sobre varios nucleos, iowait maximo 3.22 % y PSI IO full
  maximo 0.51. El mayor proceso externo observado siguio siendo `code`, con
  2335.7 MiB RSS;
- evidencia funcional minima del reducido: dos first anchors, snapshots
  periodicos, estado final de 134 poses/112 activas y publicacion final de 6353
  puntos/112 KFs; cola principal 269 procesadas, cero pendientes y
  `max_active=1`;
- unica incidencia: Gazebo devuelve 255 durante el cleanup posterior a
  `SIM-DONE`, igual que en pruebas anteriores; no afecta al recorrido ni a la
  medida;
- artefactos: log completo preservado y no leido en
  `codex/archivos_auxiliares/logs/prueba_141.log`, reducido en
  `prueba_141.reduced.log`, CSV y resumen de recursos con el mismo prefijo;
- observacion visual posterior del usuario: RViz2 y el grafo web se vieron y
  funcionaron correctamente durante toda la ejecucion; acepta el resultado sin
  incidencias visuales pendientes;
- conclusion revisada: `CONSEGUIDA` mecanicamente, visualmente y como prueba de recursos. Cerrar las
  pestanas libero margen suficiente: frente a los 463.8 MiB minimos y cierre
  preventivo de 140, esta ejecucion termino con 4826.3 MiB minimos disponibles,
  PSI de memoria cero y consumo PSS del pipeline inferior a 0.75 GiB.
