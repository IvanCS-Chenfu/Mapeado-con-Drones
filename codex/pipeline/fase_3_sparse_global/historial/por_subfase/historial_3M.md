# Historial subfase 3M

## 2026-07-14 — Reserva de nueva subfase 3M

- objetivo intentado:
  - reservar una subfase nueva entre `3L` y la antigua `3M`;
  - desplazar la planificación posterior para que las antiguas `3M-3W` pasen a
    `3N-3X`.
- archivos modificados:
  - `codex/pipeline/fase_3_sparse_global/subfases/subfase_3M.md`;
  - documentos de pipeline, contexto e índices que apuntan a la subfase actual.
- paquetes compilados:
  - no aplica; cambio documental.
- pruebas Gazebo/replay:
  - no aplica; cambio documental.
- conclusión:
  - en ese momento `3M` queda temporalmente como hueco pendiente;
  - después se redefine como `CovisibilityDatabase` en la entrada siguiente.
- siguiente paso recomendado:
  - sustituido por la definición de `CovisibilityDatabase` documentada debajo.

## 2026-07-14 — Definición de `CovisibilityDatabase`

- objetivo intentado:
  - convertir `3M` en una subfase real para crear una base de covisibilidad
    confirmada;
  - documentar cómo la consumirán `3N`, `3O`, `3P` y `3Q`.
- archivos modificados:
  - `codex/pipeline/fase_3_sparse_global/subfases/subfase_3M.md`;
  - `codex/pipeline/fase_3_sparse_global/subfases/subfase_3N.md`;
  - subarchivos de `3O`, `3P` y `3Q`;
  - resúmenes e índices de contexto/pipeline.
- paquetes compilados:
  - no aplica; cambio documental.
- pruebas Gazebo/replay:
  - no aplica; cambio documental.
- conclusión:
  - `3M` queda como `CovisibilityDatabase`;
  - la base no guarda candidatos ni campo `state`;
  - solo entran relaciones confirmadas por ORB-SLAM3 nativo o por geometría del
    servidor;
  - las aristas guardan soporte, fuente y pose relativa medida/current.
- siguiente paso recomendado:
  - implementar `CovisibilityDatabase` antes de iniciar `3N`.

## 2026-07-16 — Implementación pendiente de validación en simulación

- objetivo intentado:
  - implementar la base confirmada de covisibilidad y conectarla a la ingesta
    raw y a `PoseGraphBuilder`.
- archivos modificados:
  - `orbslam3_multi/include/orbslam3_multi/covisibility_database.hpp`;
  - `orbslam3_multi/src/covisibility_database.cpp`;
  - `orbslam3_multi/include/orbslam3_multi/pose_graph_builder.hpp`;
  - `orbslam3_multi/src/pose_graph_builder.cpp`;
  - `orbslam3_server/src/global_map_server.cpp`;
  - resúmenes y documentación de paquete de `3M`.
- cambios realizados:
  - importación de `connected_keyframe_ids/weights` con umbral configurable;
  - orden canónico de parejas, consultas de vecinos/ventana y conservación de
    `relative_pose_measured`;
  - integración de aristas `SoftConsistency` sin crear vértices nuevos;
  - logs de importación/resumen y reset de covisibilidad para replay.
- paquetes compilados:
  - no ejecutado: el checkout remoto no contiene `orbslam3_multi/CMakeLists.txt`
    ni el paquete `orbslam3_msgs` necesario para compilar fuera del workspace
    completo.
- pruebas Gazebo/replay:
  - pendiente: este entorno Codex Web no permite validar la simulación.
- conclusión:
  - `3M` queda **a probar en simulación**; no iniciar `3N` hasta obtener
    evidencia de importación y consultas con datos reales.
- siguiente paso recomendado:
  - compilar en el workspace completo y ejecutar replay/Gazebo verificando
    `[F1M-COVIS-IMPORT]`, `[F1M-COVIS-SUMMARY]` y aristas de ventana en
    `PoseGraphBuilder`.

## 2026-07-17 10:59 — Subfase 3M — build corregido y simulación bloqueada

- objetivo intentado:
  - comprobar la implementación de `CovisibilityDatabase`;
  - compilar los paquetes afectados;
  - ejecutar `prueba_1` Gazebo para validar importación ORB-SLAM3 y consultas
    `[F1M-COVIS-*]`.
- archivos modificados:
  - `orbslam3_multi/CMakeLists.txt`;
  - `orbslam3_multi/src/covisibility_database.cpp`;
  - documentación compacta, doc de paquete e historial de `3M`.
- paquetes compilados:
  - `orbslam3_multi`;
  - `orbslam3_server`.
- resultado de build:
  - primer fallo: faltaba `src/covisibility_database.cpp` en la librería
    `orbslam3_multi`, causando referencias indefinidas desde
    `PoseGraphBuilder`;
  - segundo fallo: `RawKeyFrameId` no tenía `operator!=`; se reemplazó por
    `!(edge.kf_a == edge.kf_b)`;
  - tercer fallo: faltaba `src/loop_detector.cpp` en la librería
    `orbslam3_multi`, causando referencia indefinida desde `global_map_server`;
  - build final:
    `./codex/herramientas/build_selected_packages.sh orbslam3_multi orbslam3_server`
    con `BUILD-EXIT-CODE 0`.
- pruebas Gazebo/replay:
  - `./codex/herramientas/run_simulation.sh --prueba 1 --launch "ros2 launch simulacion_dron multi_dron.launch.py"`;
  - repetida también fuera de sandbox al diagnosticar logs ROS;
  - ambas quedan en `SIM-EXIT-CODE 1` antes del scenario runner.
- patrones de reducción:
  - `F1M-COVIS|SCENARIO-RUNNER|SIM-|ERROR|FATAL|libORB|not found|stereo|global_map_server|F1B-SERVER-STATS|F1C-RAWDB-STATS|Gazebo|gzserver|process has died`.
- evidencia positiva:
  - el servidor arranca y emite `[F1M-COVIS-SUMMARY]`;
  - `CovisibilityDatabase` enlaza correctamente tras el build final;
  - la implementación mantiene claves `(drone_id,map_epoch,kf)`, orden canónico,
    ausencia de `state` y preservación de `relative_pose_measured`.
- evidencia negativa o ausente:
  - `orbslam3/stereo` muere con `exit code 127`;
  - error: `libORB_SLAM3.so: cannot open shared object file`;
  - `install/orbslam3/lib/libORB_SLAM3.so` es symlink a
    `src/ORB_SLAM3/lib/libORB_SLAM3.so`, pero `src/ORB_SLAM3/` está vacío;
  - no llegan `OrbMap`: `rx_maps=0`, `journal=0`, `kfs=0`, `mps=0`;
  - no aparecen `[F1M-COVIS-IMPORT]`, `[F1M-COVIS-EDGE-ADD]` ni
    `[F1M-COVIS-QUERY]` con datos reales.
- conclusión:
  - `BLOQUEADA`: la implementación compila, pero `3M` no puede declararse
    `CONSEGUIDA` porque la prueba no alcanza datos ORB-SLAM3 reales.
- siguiente paso recomendado:
  - restaurar/buildar `ORB_SLAM3` para que exista
    `src/ORB_SLAM3/lib/libORB_SLAM3.so` o corregir el runtime path;
  - repetir `prueba_1` o replay con dataset válido y exigir importaciones
    `[F1M-COVIS-IMPORT]` con `connections>0` y aristas confirmadas útiles.

## 2026-07-19 13:44 — Subfase 3M — reintento live sigue bloqueado

- objetivo intentado:
  - repetir la validación de `3M` tras indicación de que ORB-SLAM3 podía estar
    restaurado;
  - confirmar si Gazebo ya produce `OrbMap` reales y marcadores
    `[F1M-COVIS-*]`.
- archivos modificados:
  - solo documentación de estado, paquete e historial.
- paquetes compilados:
  - `orbslam3_multi`;
  - `orbslam3_server`.
- resultado de build:
  - `./codex/herramientas/build_selected_packages.sh orbslam3_multi orbslam3_server`
    con `BUILD-EXIT-CODE 0`.
- pruebas Gazebo/replay:
  - `./codex/herramientas/run_simulation.sh --prueba 1 --launch "ros2 launch simulacion_dron multi_dron.launch.py"`;
  - `SIM-EXIT-CODE 1` tras tres intentos de arranque.
- patrones de reducción:
  - `F1M-COVIS|SCENARIO-RUNNER|SIM-|ERROR|FATAL|libORB|not found|stereo|global_map_server|F1B-SERVER-STATS|F1C-RAWDB-STATS|Gazebo|gzserver|process has died`.
- evidencia positiva:
  - build final correcto;
  - `global_map_server` arranca y emite `[F1M-COVIS-SUMMARY]`.
- evidencia negativa o ausente:
  - `orbslam3/stereo` muere con `exit code 127`;
  - error persistente: `libORB_SLAM3.so: cannot open shared object file`;
  - no llegan mapas al servidor: `rx_maps=0`, `journal=0`, `kfs=0`, `mps=0`;
  - `[F1M-COVIS-SUMMARY] confirmed_edges=0`;
  - no aparece `[F1M-COVIS-IMPORT]`, por lo que no se valida
    `connected_keyframe_ids/weights`.
- conclusión:
  - `BLOQUEADA`: `3M` no se puede cerrar; la implementación compila, pero la
    prueba sigue sin datos ORB-SLAM3 reales.
- siguiente paso recomendado:
  - comprobar que el archivo real
    `src/ORB_SLAM3/lib/libORB_SLAM3.so` existe y que `ldd
    install/orbslam3/lib/orbslam3/stereo` deja de mostrar `not found`;
  - repetir `prueba_1` y exigir `[F1M-COVIS-IMPORT] connections>0`.

## 2026-07-19 14:08 — Subfase 3M — runtime ORB-SLAM3 recuperado

- objetivo:
  - resolver el bloqueo runtime de `orbslam3/stereo` tras la recuperación de
    `ORB_SLAM3/`.
- cambios:
  - `ORB_SLAM3/` restaurado desde upstream
    `4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4`;
  - `ORB_SLAM3/CMakeLists.txt` pasa a compilar con `-std=c++14` por
    compatibilidad con Pangolin/sigslot actual;
  - `ORB_SLAM3::System` expone `GetAllKeyFrames()` y `GetAllMapPoints()` como
    passthrough a `Atlas`, API que el wrapper `orbslam3_ros2` esperaba para
    exportar `OrbMap`.
- verificaciones:
  - `make -j4 ORB_SLAM3` termina con target `ORB_SLAM3` construido;
  - `ldd install/orbslam3/lib/orbslam3/stereo` resuelve `libORB_SLAM3.so`,
    `libDBoW2.so` y `libg2o.so` desde `src/ORB_SLAM3`;
  - `nm -D src/ORB_SLAM3/lib/libORB_SLAM3.so` confirma
    `System::GetAllKeyFrames()` y `System::GetAllMapPoints()`;
  - `./codex/herramientas/run_simulation.sh --prueba 1 --launch "ros2 launch simulacion_dron multi_dron.launch.py"`
    termina con `SIM-EXIT-CODE 0`.
- evidencia live:
  - el log no contiene `libORB_SLAM3.so: cannot open shared object file`,
    `symbol lookup error` ni `undefined symbol`;
  - aparecen eventos `[F1M-COVIS-IMPORT]`;
  - `rx_maps` deja de quedar bloqueado en cero y llega al menos a
    `rx_maps=4`, `rx_kfs=1`, `rx_mps=745`.
- conclusión:
  - el bloqueo por librería ausente y ABI incompleta queda resuelto;
  - `3M` queda desbloqueada para revisión de evidencia funcional de
    covisibilidad nativa y consultas.

## 2026-07-19 14:26 — Subfase 3M — prueba live ejecuta pero no valida aristas

- objetivo intentado:
  - repetir `prueba_1` una vez recuperado el runtime de ORB-SLAM3;
  - comprobar si `CovisibilityDatabase` recibe conexiones nativas y crea aristas
    confirmadas.
- archivos modificados:
  - solo documentación de estado, paquete e historial.
- paquetes compilados:
  - no se recompiló en esta repetición; se reutilizó el build anterior ya
    validado.
- resultado de build:
  - build previo de `orbslam3_multi orbslam3_server` con `BUILD-EXIT-CODE 0`.
- pruebas Gazebo/replay:
  - `./codex/herramientas/run_simulation.sh --prueba 1 --launch "ros2 launch simulacion_dron multi_dron.launch.py"`;
  - `SCENARIO-RUNNER-DONE success=true`;
  - `SIM-EXIT-CODE 0`.
- patrones de reducción:
  - `F1M-COVIS|SCENARIO-RUNNER|SIM-|ERROR|FATAL|libORB|not found|stereo|global_map_server|F1B-SERVER-STATS|F1C-RAWDB-STATS|process has died|ORBMAP|OrbMap`.
- evidencia positiva:
  - `ldd install/orbslam3/lib/orbslam3/stereo` resuelve `libORB_SLAM3.so`;
  - ya no aparece el error `libORB_SLAM3.so: cannot open shared object file`;
  - el wrapper de simulación y el escenario terminan correctamente;
  - llegan `OrbMap` al servidor (`rx_maps` avanza y el journal deja de ser
    cero);
  - aparecen eventos `[F1M-COVIS-IMPORT]`.
- evidencia negativa o ausente:
  - `global_map_server` muere durante la prueba con `exit code -6`;
  - todos los `[F1M-COVIS-IMPORT]` revisados muestran `connections=0`,
    `added=0`, `updated=0`;
  - `[F1M-COVIS-SUMMARY]` permanece en `confirmed_edges=0`,
    `orbslam3_native=0`, `server_loop_geometric=0`;
  - los deltas publicados hasta el crash siguen indicando `kfs=0` salvo el
    primer KeyFrame conservado en la base, por lo que no hay evidencia de
    `connected_keyframe_ids/weights` poblados.
- conclusión:
  - `NO CONSEGUIDA`: el bloqueo runtime queda resuelto, pero la prueba live no
    valida la propiedad funcional de `3M` y revela una caída del servidor.
- siguiente paso recomendado:
  - diagnosticar el abort de `global_map_server` alrededor de los `OrbMap`
    posteriores a `arrival_id=22`;
  - revisar por qué el wrapper/raw DB no entrega más KeyFrames ni conexiones
    nativas durante esta trayectoria;
  - repetir `prueba_1` o replay y exigir `[F1M-COVIS-IMPORT] connections>0`,
    `[F1M-COVIS-EDGE-ADD]` y `confirmed_edges>0` antes de cerrar `3M`.

## 2026-07-20 20:25 — Subfase 3M — conseguida tras corregir aliasing Eigen

- objetivo intentado:
  - completar `3M` tras identificar que el abort venía de las modificaciones de
    `CovisibilityDatabase`;
  - corregir la caída de `global_map_server`;
  - validar importación ORB-SLAM3 nativa, aristas confirmadas y consulta por
    `PoseGraphBuilder`.
- archivos modificados:
  - `orbslam3_multi/src/covisibility_database.cpp`;
  - `orbslam3_multi/src/test_covisibility_database.cpp`;
  - `orbslam3_multi/CMakeLists.txt`;
  - documentación de contexto, paquete, pipeline e historial.
- paquetes compilados:
  - `orbslam3_multi`;
  - `orbslam3_server`.
- resultado de build:
  - `./codex/herramientas/build_selected_packages.sh orbslam3_multi orbslam3_server`
    termina con `BUILD-EXIT-CODE 0`.
- pruebas Gazebo/replay:
  - test local:
    `./install/orbslam3_multi/lib/orbslam3_multi/test_covisibility_database`,
    código `0`;
  - Gazebo:
    `./codex/herramientas/run_simulation.sh --prueba 1 --launch "ros2 launch simulacion_dron multi_dron.launch.py"`,
    `SCENARIO-RUNNER-DONE success=true` y `SIM-EXIT-CODE 0`.
- patrones usados para reducir logs:
  - `F1M-COVIS|SCENARIO-RUNNER|SIM-|ERROR|FATAL|Aliasing|assert|exit code -6|process has died|global_map_server|F1B-SERVER-STATS|F1C-RAWDB-STATS|ORBMAP|OrbMap|PIPE0-WRAPPER-EPOCH`.
- evidencia positiva:
  - el abort desaparece: no hay `Aliasing problem`, `exit code -6` ni muerte de
    `global_map_server`;
  - `F1M-COVIS-IMPORT` llega con conexiones reales;
  - aparecen `[F1M-COVIS-EDGE-ADD]` y `[F1M-COVIS-EDGE-UPDATE]`;
  - resumen final: `confirmed_edges=3025`, `orbslam3_native=3025`,
    `server_loop_geometric=0`, `revision=3043`;
  - `PoseGraphBuilder` consulta la base:
    `[F1M-COVIS-QUERY] task_id=1 window_kfs=25 returned_edges=63 min_weight=15.000`;
  - recepción final con ambos drones: `drones_seen=2`, `epochs_seen=2`,
    `submaps=2`, `kfs=210`.
- evidencia negativa o ausente:
  - no se observaron loops `SERVER_LOOP_GEOMETRIC`, porque pertenecen a fases
    posteriores; la subfase solo deja lista la inserción y prueba sintética.
- conclusión:
  - `CONSEGUIDA`: `CovisibilityDatabase` guarda relaciones confirmadas, importa
    covisibilidad ORB-SLAM3 con umbral, mantiene pose medida/current y es
    consultable por optimización/loops posteriores.
- siguiente paso recomendado:
  - continuar con `3N` implementando búsqueda BoW real y saltando pares ya
    confirmados por `CovisibilityDatabase`.

## 2026-07-28 — Revalidación integrada pendiente

- estado histórico:
  `3M` permanece `CONSEGUIDA`; no se ha detectado un fallo nuevo de
  `CovisibilityDatabase`.
- motivo de la nueva prueba:
  el bloque fiducial `3I-3L` cambió la autoridad de poses, la publicación y la
  ejecución concurrente. Se debe comprobar que la importación y consulta de
  covisibilidad siguen intactas con el código integrado.
- comprobación mínima:
  compilar `orbslam3_multi` y `orbslam3_server`, ejecutar
  `test_covisibility_database` y repetir la prueba live indicada por el
  contrato de `3M`, exigiendo conexiones importadas, aristas confirmadas no
  nulas y ausencia de aborts.
- siguiente paso:
  si pasa, registrar la regresión y ejecutar la revalidación de `3N`.

## 2026-07-28 16:18 — Subfase 3M — regresión integrada tras cierre 3I-3L

- objetivo intentado:
  - comprobar que `CovisibilityDatabase` sigue funcionando después de los
    cambios de `3I-3L`;
  - validar build, test local, importación ORB-SLAM3 nativa y ausencia de los
    aborts históricos.
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
  - `./install/orbslam3_multi/lib/orbslam3_multi/test_covisibility_database`:
    código `0`;
  - `prueba_1` live con `ros2 launch simulacion_dron multi_dron.launch.py`:
    hubo un primer intento con Gazebo muerto temprano y retry automático; el
    segundo intento termina con `SCENARIO-RUNNER-DONE success=true`,
    `SIM-DONE prueba=1 success=true` y `SIM-EXIT-CODE 0`.
- patrones de reducción:
  - `SCENARIO-RUNNER|GOAL|RESULT|success|F1M-|F1N-|F1C-RAWDB|F1F-GLOBALMAP|ERROR|FATAL|Segmentation fault|Killed|process has died|exit code -6|SO3::exp failed|undefined symbol`.
- evidencia positiva:
  - `rawdb_prueba_1.record` queda regenerado con `260M`;
  - `F1M-COVIS-IMPORT`: `163` eventos;
  - `F1M-COVIS-EDGE-ADD`: `96` eventos;
  - `F1M-COVIS-EDGE-UPDATE`: `2` eventos;
  - `F1M-COVIS-SUMMARY`: `55` eventos;
  - resumen final:
    `confirmed_edges=9145`, `orbslam3_native=9145`,
    `server_loop_geometric=0`, `revision=9179`.
- evidencia negativa o ausente:
  - no aparece `F1M-COVIS-QUERY` en live; es coherente con el estado actual,
    porque `pose_graph_use_covisibility_edges=false` mantiene la covisibilidad
    fuera del grafo fiducial. La API de consulta queda cubierta por
    `test_covisibility_database`.
  - aparecen dos `process has died` de Gazebo: uno en el intento inicial que el
    script reintenta y otro durante cleanup tras `SIM-DONE`; no hay muerte de
    `global_map_server` ni abort ORB-SLAM3.
- conclusión:
  - `CONSEGUIDA`: regresión integrada de `3M` superada.
- siguiente paso recomendado:
  - usar esta base como prerequisito cumplido para `3N`; continuar con
    revalidación de `3O`.

## 2026-08-15 — Subfase 3M — reimplementacion MEDIA y cierre

- objetivo intentado: integrar covisibilidad derivada en el worker secundario
  mediante una `DatabaseUpdateTask` MEDIA por `ChangeSet` y encadenar loops
  BAJOS sin bloquear el principal.
- archivos modificados: nuevos `covisibility_database.{hpp,cpp}` y
  `loop_task.hpp`; integracion en `SparseGlobalBackend`, cola y servidor.
- paquetes compilados: `orbslam3_multi`, `orbslam3_server`,
  `simulacion_dron`.
- resultado de build: build final 3/3, exit 0.
- pruebas: regresion final 53/53 C++ y 9/9 web; replays 152/153 y live 154.
- patrones de reduccion: `F3M-DATABASE-*`, `F3N-LOOP-ENQUEUE`, cierre de colas,
  backpressure, errores y recursos.
- evidencia positiva: patches canonicos/idempotentes cubiertos por test; MEDIA
  compromete y encola BAJAS; live 154 procesa 371 entradas principales y
  termina `pending=0`, `hard_failed=0`, `max_active=1`.
- evidencia negativa o ausente: replay 152 dejo unas 384 tareas pendientes por
  coste excesivo del consumidor loop; no fue fallo de la base y se conserva
  como intento `PARCIAL`.
- conclusion: `CONSEGUIDA`; replay 153 repite la carga tras optimizar el
  consumidor y drena toda la cola.
- siguiente paso recomendado: consumir las aristas geometricas server solo
  despues de la fusión de 3P, manteniendo separada su fuente.
