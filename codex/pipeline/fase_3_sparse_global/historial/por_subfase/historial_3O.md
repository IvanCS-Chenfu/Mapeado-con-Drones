# Historial — Subfase 3O

## 2026-07-21 14:49 — SubcloudLoopVerifier conseguido

- objetivo intentado:
  verificar geométricamente candidatos BoW de `3N` mediante subnubes, matching
  ORB, reducción espacial robusta, RANSAC 3D-3D y cálculo de error de pose. La
  subfase solo produce evidencia preliminar: no fusiona, no optimiza, no inserta
  covisibilidad confirmada y no modifica poses.
- archivos modificados:
  `orbslam3_multi/include/orbslam3_multi/subcloud.hpp`,
  `orbslam3_multi/include/orbslam3_multi/loop_verification_result.hpp`,
  `orbslam3_multi/include/orbslam3_multi/subcloud_loop_verifier.hpp`,
  `orbslam3_multi/src/subcloud_loop_verifier.cpp`,
  `orbslam3_multi/CMakeLists.txt`,
  `orbslam3_server/src/global_map_server.cpp`,
  `codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml`,
  `tray_prueba_2.yaml` y `tray_prueba_3.yaml`.
- paquetes compilados:
  `orbslam3_multi`, `orbslam3_server`, `simulacion_dron`.
- resultado de build:
  `./codex/herramientas/build_selected_packages.sh orbslam3_multi orbslam3_server simulacion_dron`
  terminó con `Summary: 3 packages finished [25.9s]` y `BUILD-EXIT-CODE 0`.
- pruebas Gazebo/replay:
  `prueba_1` live con
  `ros2 launch simulacion_dron multi_dron.launch.py`;
  `prueba_2` y `prueba_3` replay reales con
  `ros2 launch orbslam3_server global_orb_map_server.launch.py
  rawdb_replay_enabled:=true rawdb_record_enabled:=false
  rawdb_replay_period_sec:=0.05 use_sim_time:=false n_drones:=2`.
- patrones de reducción:
  `SCENARIO-RUNNER|GOAL|RESULT|success|F1N-|F1O-SUBCLOUD-SKIP-CONFIRMED-COVIS|F1M-COVIS|F1C-RAWDB|F1F-GLOBALMAP|ERROR|FATAL|Segmentation fault|Killed`
  para live;
  `SCENARIO-RUNNER|GOAL|RESULT|success|F1N-|F1O-SUBCLOUD-SKIP-CONFIRMED-COVIS|F1M-COVIS|F1C-REPLAY|F1C-RAWDB|ERROR|FATAL|Segmentation fault|Killed`
  para replay general;
  `SCENARIO-RUNNER|GOAL|RESULT|success|F1N-SUBCLOUD-CANDIDATE-REDUCE|F1N-SUBCLOUD-MATCH-REFINED|fallback|ERROR|FATAL|Segmentation fault|Killed`
  para la prueba específica de reducción.
- evidencia positiva:
  `prueba_1` terminó con `SCENARIO-RUNNER-DONE success=true`,
  `SIM-DONE prueba=1 success=true`, `458` verificaciones, `458` RANSAC,
  `78` decisiones con `geometry_confirmed=true`, `380` rechazos,
  `265` reducciones `fallback=false reason=robust_match_box`,
  `286` skips BoW por covisibilidad confirmada y sin errores graves.
  `prueba_2` cargó replay con `entries=177 deltas=163 full=14 submaps=3
  kfs=299 mps=29096`, terminó con `success=true`, `298` verificaciones,
  `298` RANSAC, `71` confirmaciones geométricas, `227` rechazos y `223`
  reducciones robustas sin fallback. `prueba_3` repitió la misma evidencia de
  replay y validó específicamente los marcadores de reducción/refinado.
- evidencia negativa o ausente:
  no aparecieron `[ERROR]`, `[FATAL]`, `Segmentation fault`, `Killed`,
  `exit code -6`, `SO3::exp failed` ni `process has died` en las pruebas
  finales. `[F1N-SUBCLOUD-ERROR]` aparece como marcador técnico de error de
  pose calculado, no como error ROS.
- aprendizaje de prueba:
  para replay real de `3O`, usar launch directo de `global_orb_map_server` con
  `rawdb_replay_enabled:=true` y `rawdb_record_enabled:=false`. Lanzar
  `multi_dron.launch.py` para `prueba_2`/`prueba_3` puede arrancar live y pisar
  `rawdb_prueba_1.record`.
- conclusión:
  `CONSEGUIDA`.
- siguiente paso recomendado:
  ejecutar `3P`: decidir qué hacer con `LoopVerificationResult` e insertar
  covisibilidad confirmada si procede, manteniendo separadas fusión y
  optimización por loop según el contrato de la subfase.

## 2026-07-21 17:20 — Revalidación live con vuelta opuesta inter-dron

- objetivo intentado:
  comprobar explícitamente el caso funcional esperado por el usuario: dos drones
  rodean el edificio en sentidos opuestos y cada uno pasa por zonas ya visitadas
  por el otro, para ver si aparecen loops inter-dron y si se clasifican como
  saltados por covisibilidad, rechazados por geometría o aceptados con error.
- archivo modificado:
  `codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml`.
- trayectoria:
  `dron_1` sale por el lado oeste, cruza norte hacia el este, baja por el este y
  cierra por el sur; `dron_2` hace el recorrido complementario por este, norte,
  oeste y sur. Las alturas son distintas para evitar colisión (`1.0` y `1.3` m).
- prueba ejecutada:
  `./codex/herramientas/run_simulation.sh --prueba 1 --launch "ros2 launch simulacion_dron multi_dron.launch.py" --post-scenario-wait-sec 20`.
- resultado:
  `SCENARIO-RUNNER-DONE scenario='prueba_1o_subcloud_live_vuelta_opuesta'
  success=true`, todos los goals reportan `success=true`,
  `SIM-DONE prueba=1 success=true`, `SIM-EXIT-CODE 0`.
- diagnóstico de loops:
  `F1N-BOW-SKIP-CONFIRMED-COVIS` suma `3869` candidatos BoW cancelados por
  covisibilidad ORB-SLAM3 confirmada; `399` candidatos pasan a verificación de
  subnubes; `346` se rechazan por geometría y `53` se aceptan con error de pose.
- diagnóstico inter-dron:
  `236` candidatos inter-dron verificados, `41` aceptados y `195` rechazados.
  Todos los aceptados inter-dron tienen error asociado; `error_t_min=0.010898`,
  `error_t_mean=0.065706`, `error_t_max=0.199698`.
- diagnóstico intra-dron:
  `163` candidatos intra-dron verificados, `12` aceptados y `151` rechazados.
- rechazos geométricos:
  `not_enough_inliers=183`, `low_inlier_ratio=38`,
  `query_no_world_pose=124`, `candidate_seed_no_world_pose=1`.
- incidencia:
  aparece un `[ERROR]` de `dron_2.generador_URDF` durante cleanup, después de
  `SIM-DONE success=true`; todos los procesos terminan cleanly. No es fallo de
  ORB-SLAM3, de Gazebo durante el escenario ni de `SubcloudLoopVerifier`.
- conclusión:
  la prueba específica queda ejecutada y confirma que `3O` detecta loops
  inter-dron cuando un dron pasa por zonas del otro, y asocia error de pose a
  cada loop aceptado. `3O` se mantiene `CONSEGUIDA`.

## 2026-07-21 18:50 — Revalidación con prueba típica de rodeo y dos fiduciales

- objetivo intentado:
  repetir la prueba live con la trayectoria canónica indicada por el usuario:
  dos drones rodean el edificio en sentidos opuestos, ven todo el edificio en
  momentos distintos y pasan por fiducial 2, fiducial 1 y de nuevo fiducial 2.
- YAML usado:
  `codex/archivos_auxiliares/trayectorias/prueba_tipica_rodeo_edificio_dos_fiduciales.yaml`.
- prueba ejecutada:
  `./codex/herramientas/run_simulation.sh --prueba 6 --yaml /home/chenfu/Gazebo/src/codex/archivos_auxiliares/trayectorias/prueba_tipica_rodeo_edificio_dos_fiduciales.yaml --launch "ros2 launch simulacion_dron multi_dron.launch.py" --post-scenario-wait-sec 20`.
- resultado:
  `SCENARIO-RUNNER-DONE scenario='prueba_tipica_rodeo_edificio_dos_fiduciales'
  success=true`, todos los goals principales con `success=true`,
  `SIM-DONE prueba=6 success=true`, `SIM-EXIT-CODE 0`.
- patrones de reducción:
  `SCENARIO-RUNNER|GOAL|RESULT|success|F1E-|F1H-|F1I-|F1J-|F1K-|F1L-|F1N-|F1O-SUBCLOUD-SKIP-CONFIRMED-COVIS|F1M-COVIS|F1C-RAWDB|F1F-GLOBALMAP|ERROR|FATAL|Segmentation fault|Killed|exit code -6|SO3::exp failed|process has died`.
- diagnóstico de loops:
  `F1N-BOW-SKIP-CONFIRMED-COVIS` suma `3060` candidatos BoW cancelados por
  covisibilidad ORB-SLAM3 confirmada; `459` candidatos pasan a verificación por
  subnubes; `312` se rechazan por geometría y `147` se aceptan con error de
  pose asociado.
- diagnóstico inter-dron:
  `321` candidatos inter-dron verificados, `130` aceptados y `191` rechazados.
  De los aceptados inter-dron, `129` son `LOOP_OPTIMIZATION_CANDIDATE` y `1` es
  `FUSION_CANDIDATE`. Todos tienen error asociado; `error_t_min=0.334387`,
  `error_t_mean=6.688109`, `error_t_max=14.492848`.
- diagnóstico intra-dron:
  `138` candidatos intra-dron verificados, `17` aceptados y `121` rechazados.
  De los aceptados intra-dron, `5` son `LOOP_OPTIMIZATION_CANDIDATE` y `12`
  `FUSION_CANDIDATE`.
- fiduciales:
  se crean `2` tareas fiduciales (`F1H-FID-TASK-CREATED`) y ambas ejecutan
  dry-run con `F1J-OPT-DRYRUN-RESULT success=true`. No hay apply final porque
  ambas decisiones son `useful=false partial_candidate=false
  reason=cost_not_reduced` y el servidor cierra con
  `F1J-SERVER-DRYRUN-DONE no_apply=true`.
- evidencia negativa o ausente:
  aparece un `[ERROR]` de `gazebo-1` con `exit code 255` durante cleanup,
  después de `SIM-DONE success=true`; el script devuelve `SIM-EXIT-CODE 0`.
  No aparecen `exit code -6`, `SO3::exp failed`, `Segmentation fault` ni fallo
  del verificador.
- conclusión:
  la prueba canónica confirma la expectativa principal de `3O`: hay muchos
  loops inter-dron cuando ambos drones recorren zonas comunes en tiempos
  distintos, y cada loop aceptado queda acompañado de error de pose para fases
  posteriores. La ruta fiducial se activa en dry-run pero no se aplica por el
  criterio de seguridad de coste. `3O` se mantiene `CONSEGUIDA`.

## 2026-07-28 — Punto de continuación tras cerrar `3I-3L`

- estado histórico:
  `3O` permanece `CONSEGUIDA`; la implementación de subnubes, matching ORB,
  reducción espacial, RANSAC y error de pose ya existe.
- prerrequisitos:
  ejecutar en orden y cerrar las regresiones de `3M` y `3N`.
- continuación requerida:
  repetir la prueba live típica
  `prueba_tipica_rodeo_edificio_dos_fiduciales.yaml` y los replays definidos
  por el contrato de `3O`, comprobando que sigue habiendo candidatos,
  verificaciones geométricas, decisiones con error de pose y ninguna mutación
  de `GlobalPoseStore` por la ruta de loops.
- criterio de salida:
  si la regresión integrada pasa, mantener `3O` cerrada y empezar el desarrollo
  nuevo de `3P`. Si falla, reabrir únicamente la primera subfase donde aparezca
  el error reproducible.

## 2026-07-28 22:18 — Revalidación diferencial integrada

- objetivo intentado:
  comprobar que `3O` sigue verificando correspondencia geométrica después de
  los cierres de `3M/3N`, incluidos candidatos inter-dron y KFs cercanos del
  mismo submapa, sin ejecutar responsabilidades de `3P/3Q`.
- archivos funcionales modificados:
  ninguno; la auditoría y la prueba no encontraron una regresión inequívoca.
- paquetes compilados:
  `orbslam3_multi`, `orbslam3_server`, `simulacion_dron`.
- resultado de build:
  `BUILD-EXIT-CODE 0`; solo apareció el warning no bloqueante del prefijo
  inexistente `/opt/drake/share/drake`.
- prueba Gazebo:
  `prueba_46` con
  `prueba_tipica_anclaje_diferencial.yaml`,
  `rawdb_record_enabled:=false`, timeout de `900 s` y espera final de `30 s`.
  El escenario, sus `6` goals y el wrapper terminaron con `success=true` y
  código `0`.
- patrones de reducción:
  `SCENARIO-RUNNER|GOAL|RESULT|success|F1N-|F1O-|F1M-COVIS|F1C-RAWDB|F1F-GLOBALMAP|near_same_submap|geometry_confirmed|FUSION_CANDIDATE|LOOP_OPTIMIZATION_CANDIDATE|ERROR|FATAL|FUSION_APPLY|LOOP_OPT_TASK_CREATED|SERVER_LOOP_OPTIMIZATION_APPLY|RAWDB-POSE-OVERWRITE-BY-OPT|raw_db_modified`.
- evidencia positiva:
  `680` candidatos BoW, `359` inter-dron y `168`
  `near_same_submap=true`. Hubo `149` queries: `13` sin candidatos y `136` con
  lista BoW. Como `loop_verify_max_candidates_per_query=1`, solo el rank 1 de
  cada lista llegó al verificador: `136` intentos y `544` candidatos restantes
  sin evaluar, no rechazados.
  De los `136` intentos, `54` terminaron antes de una comparación geométrica
  útil por `query_no_world_pose`; quedaron `82` pares evaluables, `77`
  confirmados por RANSAC y `5` rechazados por `low_inlier_ratio`. Por tanto, la
  tasa geométrica relevante es `77/82`, no `77/136`.
  Las decisiones finales son `76 FUSION_CANDIDATE`,
  `1 LOOP_OPTIMIZATION_CANDIDATE` y `59 REJECT`.
  De `95` intentos inter-dron, `49` quedan confirmados. Los `41` intentos
  intra-dron producen `28` confirmaciones: `19` cercanas del mismo submapa y
  `9` intra-dron no cercanas.
- evidencia negativa o ausente:
  los `54 query_no_world_pose` no son rechazos geométricos: corresponden al
  periodo anterior al primer anchor world de cada submapa. En dron 1 afectan a
  queries `KF0..KF26` y el primer anchor llega en `KF27`; en dron 2 afectan a
  `KF1..KF40` y el primer anchor llega en `KF42`. Los KFs ya tienen identidad
  de submapa y datos raw, pero aún no existe una transformación global para
  comparar ambas subnubes en `world`. La ruta actual no reintenta
  automáticamente esos candidatos tras el anclaje.
  No aparecen `FUSION_APPLY`,
  `FUSION_EVENT_APPLIED`, `LOOP_OPT_TASK_CREATED`,
  `SERVER_LOOP_OPTIMIZATION_APPLY`, apply real de `3K`,
  `RAWDB-POSE-OVERWRITE-BY-OPT` ni `raw_db_modified=true`.
  El GT visible pertenece a la asociación fiducial simulada; no aparece en
  marcadores de `SubcloudLoopVerifier` y `F1L-GT-DEBUG-CONFIG enabled=false`.
  Gazebo devuelve `255` durante la limpieza posterior a
  `SIM-DONE success=true`; servidor y nodos stereo ya habían terminado
  limpiamente.
- conclusión:
  `CONSEGUIDA`. La revalidación no requiere cambios de código.
- siguiente paso recomendado:
  preparar `3P` con el usuario. Esa subfase deberá consumir la evidencia de
  `LoopVerificationResult` para decidir e insertar covisibilidad/fusión; `3O`
  debe permanecer de solo lectura. Antes conviene decidir si mantener el límite
  de un candidato por query y cómo reintentar candidatos anteriores al anchor,
  porque ambas políticas reducen la evidencia disponible para futuras aristas.

## 2026-08-15 — Subfase 3O — geometria y anchor loop end-to-end

- objetivo intentado: verificar regiones mediante subnubes/RANSAC, decidir el
  conjunto completo y anclar atomicamente un submapa sin fiducial tras dos
  queries independientes.
- archivos modificados: `loop_pipeline.{hpp,cpp}`, `global_pose_store.{hpp,cpp}`,
  backend/servidor, visualizador 22/34 y `tray_prueba_154.yaml`.
- paquetes compilados: `orbslam3_multi`, `orbslam3_server`,
  `simulacion_dron`; build final exit 0.
- pruebas: 53/53 C++ y 9/9 web; replay 152 `PARCIAL`, replay 153
  `CONSEGUIDA`, live 154 `CONSEGUIDA` tecnicamente.
- patrones de reduccion: task exacta 1000000002034, queries B/KF5 y B/KF7,
  asociaciones fiduciales, backfill, cierre secundario, errores y recursos.
- evidencia positiva: B/KF5 obtiene 18/71 inliers y queda esperando segundo
  apoyo; B/KF7 obtiene 19/73 y confirma la hipotesis. El batch ancla `(2,0)`,
  incluye 8 KFs dirty y el siguiente principal hace backfill de 9 KFs/1013
  MPs. Solo `(1,0)` recibe fiducial; cierre: anchors=2, hard=1, poses=248,
  active=222, pending=0, hard_failed=0.
- evidencia negativa o ausente: 3O reporta pares de fusion y evidencia de
  optimizacion, pero no los aplica por contrato. La inspeccion humana de RViz2
  y del grafo web de live 154 queda pendiente del usuario.
- recursos live 154: server RSS max 146.9 MiB, grupo 1537.8 MiB,
  MemAvailable min 2815.1 MiB, PSI full max 1.28 y guard inactivo.
- conclusion: `CONSEGUIDA TECNICAMENTE`; queda abierta solo la interpretación
  visual posterior de la misma prueba.
- siguiente paso recomendado: preparar 3P para aplicar fusiones compatibles sin
  convertirlas en optimizaciones y comprometer despues la arista server.

## 2026-08-15 — Prueba 154 — interpretacion visual revisada con el usuario

- evidencia objetiva conservada: el anchor loop de `(2,0)`, el backfill, la
  cola vacia y cero hard failures siguen siendo correctos.
- observacion nueva: el camino loop del grafo web permanecio aparentemente
  encendido durante casi toda la prueba.
- diagnostico: se ejecutaron 2301 `LoopTask` para 248 KFs; el frontend conserva
  cada owner 420 ms despues de `done`, por lo que tareas consecutivas se
  solapan visualmente aunque `max_active=1` y `pending=0` al cierre.
- conteos de live 154: 1184 sin candidato, 207 rechazadas por geometria, 536
  deferred, 125 candidatas de fusion, 174 evidencias para optimizacion futura,
  1 anchor aplicado y 74 stale; 1382 resultados RANSAC aceptados no equivalen
  a pares fisicos unicos.
- optimizaciones reales: 0 en live 154. Los replays 152/153 reproducen las
  mismas 3 optimizaciones fiduciales completas; 3O no ejecuta optimizacion por
  loop.
- revision funcional: el código registra el KF de anchor loop como ultimo
  control aceptado. Un fiducial posterior con error alto crea por ello una
  optimizacion desde ese control. Esta politica trata una evidencia blanda con
  demasiada autoridad y debe revisarse.
- interpretacion posterior acordada: la iluminacion continua no es un fallo del
  grafo. El unico worker proceso tareas consecutivas reales y el hold de 420 ms
  solo evita huecos visuales entre owners; `max_active=1` descarta paralelismo.
- causa de las 2301 tareas: una `LoopTask` identifica una revision procesable,
  no un KF unico. Los cambios sucesivos de pose raw, asociaciones de MPs,
  covisibilidad, geometria o anchor vuelven a habilitar KFs ya procesados. La
  sustitucion latest-wins solo elimina revisiones aun pendientes; si la previa
  ya termino, la siguiente vuelve a ejecutar el pipeline completo.
- interpretacion de decisiones: los 1184 `no candidate` son revisiones cuyo
  BoW quedo vacio tras filtros o falta de candidatos utilizables. Los 536
  `deferred` no son tareas pendientes al cierre: combinan primeras evidencias
  cross-submapa esperando una segunda query independiente y geometria
  same-submapa meramente diagnostica, que hoy comparte esa etiqueta.
- propuesta funcional del usuario: el primer fiducial observado por un
  submapa/componente anclado por loop debe sustituir el anchor blando mediante
  un reanchor rigido atomico. La relacion loop se conserva para que una fase
  futura pueda optimizar varios submapas entre fiduciales usando covisibilidad.
- acuerdo funcional posterior: mientras el hijo siga blando, una correccion
  del KF de apoyo del padre propaga el delta rigido a todo el componente hijo
  mediante batch atomico. El primer fiducial directo reancla todo el hijo como
  first anchor, lo convierte siempre en hard y corta la dependencia. Los
  fiduciales posteriores usan 3H-3L. Esta politica se reemplazara por una
  optimizacion covisible multi-submapa futura.
- conclusion revisada de live 154: `PARCIAL`; anclaje, commit y representacion
  visual conseguidos. Quedan por acordar e implementar el reanchor post-loop,
  la reduccion semantica de reevaluaciones y la separacion de telemetria.
- siguiente paso: cerrar el contrato de esas correcciones antes de preparar 3P.

## 2026-08-16 — Subfase 3O — autoridad post-loop y scheduling semantico

- objetivo intentado: implementar propagacion rigida padre-hijo, primer
  reanchor fiducial hard, separacion de telemetria y reduccion de reevaluaciones.
- archivos modificados: `GlobalPoseStore`, `SparseGlobalBackend`, tipos/raw de
  loop, `LoopPipeline`, `SecondaryTaskQueue`, servidor, tests y YAML 155.
- build: build inicial de `orbslam3_multi`, `orbslam3_server` y
  `simulacion_dron` 3/3; rebuilds posteriores 1/1 o 2/2, todos con exit 0.
- tests: agregado Colcon marco lint de `legacy2`/estilo preexistente; CTests
  funcionales dirigidos terminaron 8/8 dominio, 4/4 servidor y 1/1 web. Tras
  ampliar la celda semantica, un fixture de 0.20 m fallo correctamente y se
  ajusto a 0.60 m; repeticion final 8/8 + 4/4.
- evidencia positiva: `validation_revision` exacta queda fuera de la igualdad
  de cola; la huella semantica usa apariencia, pose gruesa, tres estados de
  madurez y soporte fuerte. Tests prueban coalescencia de refinamiento y stale
  exacto. El reanchor corta la dependencia y un hard nunca sigue al padre.
- evidencia negativa: la primera huella con MPs exactos no redujo carga; la
  version por bloques lineales de 16 solo la redujo modestamente.
- conclusion: `CONSEGUIDA TECNICAMENTE`; faltaba validacion integrada final.

## 2026-08-16 — Prueba 155 — primer escenario dirigido

- objetivo intentado: A hard en fiducial 2, B por loop y optimizacion posterior
  de A en fiducial 1 con propagacion del hijo.
- resultado: runner exit 0, `success=true`, guard inactivo y cola final vacia.
- evidencia positiva: B se ancla por loop con 26 KFs; A llega al fiducial 1 con
  4.162 m de error y compromete una ventana de 159 KFs.
- evidencia negativa: B se ancla contra A/KF11, anterior al primer hard A/KF30.
  La ventana KF30..KF190 no mueve ese apoyo y, por tanto, el escenario no
  ejercita propagacion. Procesa 2817 secundarias para 316 poses.
- conclusion: `PARCIAL`; el algoritmo no fallo, pero la prueba no midio su
  objetivo. Se conserva el log y se corrige el recorrido norte.

## 2026-08-16 — Prueba 157 — propagacion rigida demostrada

- objetivo intentado: repetir 155 haciendo que B observe la fachada norte
  cartografiada despues del primer fiducial.
- resultado: runner exit 0, `success=true`, recursos estables y cola vacia.
- evidencia positiva: B/KF4 ancla contra A/KF72, posterior al hard A/KF27. El
  fiducial 1 produce commit full de ventana 152, mueve 213 KFs y marca 78
  `control_propagated`. Una segunda MAX se revalida `STALE`. `max_active=1`.
- evidencia negativa: 2475 secundarias/269 poses = 9.20 por KF; la primera
  coalescencia no resolvia la carga.
- conclusion: `CONSEGUIDA` para propagacion; `PARCIAL` para rendimiento.

## 2026-08-16 — Prueba 158 — version semantica intermedia

- objetivo intentado: conservar anchor/propagacion tras separar huella
  semantica y validacion exacta, y medir carga.
- resultado: runner exit 0, `success=true`, anchor por loop presente, cola
  vacia y guard inactivo.
- evidencia positiva: no hay regresion funcional ni fallo duro.
- evidencia negativa: 2477 secundarias/300 poses = 8.26 por KF; la mejora es
  solo del 10 %. Los bloques lineales de 16 asociaciones permiten demasiadas
  transiciones mientras madura cada KF.
- conclusion: `PARCIAL`; motiva los tres estados finales de madurez.

## 2026-08-16 — Prueba 156 — trayectoria tipica con binario final

- objetivo intentado: regresion completa con
  `prueba_tipica_rodeo_edificio_dos_fiduciales.yaml`, incluyendo reanchor,
  optimizaciones, cola, backpressure, recursos y scheduling final.
- resultado: escenario exit 0, `success=true`; 412 s, guard inactivo,
  MemAvailable minimo 6134.8 MiB y PSS servidor maximo 204.8 MiB.
- evidencia positiva: `(1,0)` crea hard fiducial; `(2,0)` se ancla por loop y
  su primer fiducial ejecuta `[F3O-FID-LOOP-REANCHOR]` sobre 32 KFs. Hay tres
  commits fiduciales full. Cierre: 497 principales, 1060 secundarias, 89 stale,
  pending=0, hard_failed=0 y max_active=1. La carga baja a 2.18 tareas/pose.
- decisiones loop: 769 loops; 305 evidencias 3Q, 39 fusion candidates, 19
  constraints, 78 diagnosticos same-submapa, 24 esperas independientes, 207
  rechazos, 82 stale y un anchor aplicado.
- evidencia negativa o limite: ORB crea siete submapas; cuatro quedan anclados
  y tres diferidos. El `exit 255` de Gazebo ocurre tras `SIM-DONE` en cleanup.
  RViz2 y grafo web no fueron interpretados por Codex durante la ejecucion.
  Posteriormente, el usuario confirmo que ambos se veian correctamente.
- conclusion revisada: `CONSEGUIDA`. Se conserva como validacion futura que 3P
  y 3Q comprueben de extremo a extremo las evidencias de fusion y optimizacion
  producidas por 3O.
- siguiente paso: preparar 3P con el usuario.
