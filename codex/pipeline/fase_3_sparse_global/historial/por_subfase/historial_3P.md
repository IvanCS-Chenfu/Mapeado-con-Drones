# Historial 3P - Fusión de landmarks confirmados

## 2026-07-29 12:46 - Implementación y pruebas locales

- objetivo intentado: implementar tracks transitivos para los pares inlier de
  `FUSION_CANDIDATE`, covisibilidad y publicación sin duplicados raw.
- archivos funcionales principales: `fused_landmark_track.hpp`,
  `fused_landmark_manager.*`, `loop_decision_manager.*`,
  `global_map_builder.*`, `global_map_server.cpp`, `CMakeLists.txt` y
  `test_fused_landmark_manager.cpp`.
- paquetes compilados: `orbslam3_multi`, `orbslam3_server`.
- builds:
  - primer build: código `0`, con warning Eigen por include incompleto;
  - segundo build: código `0`, sin warnings propios tras añadir `Eigen/LU`;
  - tercer build: código `0` tras ajustar la cola pre-anchor.
- límites respetados: no se modifican `RawMapDatabase`, `GlobalPoseStore`,
  ORB-SLAM3 ni `orbslam3_msgs`; no se crean grafos u optimizaciones.

### Ejecución local `test_fused_landmark_manager`

- resultado: código `0`.
- evidencia: creación, ampliación, merge transitivo, refuerzo repetido, rechazo
  de identidad igual, índice inverso, descriptor medoid, score creciente,
  covisibilidad y publicación de un track omitiendo cuatro miembros raw.
- conclusión: `CONSEGUIDA`.

### Regresión local `test_covisibility_database`

- resultado: código `0`.
- evidencia: importación, canonización, loop geométrico y actualización current
  siguen funcionando.
- conclusión: `CONSEGUIDA`.

### Regresión local `test_global_pose_store_tail_anchor`

- resultado: código `0`.
- evidencia: anchors, cola derivada y rollback no sufren regresión.
- conclusión: `CONSEGUIDA`.

## 2026-07-29 12:49 - `prueba_47`

- objetivo intentado: ejecutar
  `prueba_tipica_anclaje_diferencial.yaml`.
- comando: `run_simulation.sh --prueba 47` con YAML relativo.
- log: `codex/archivos_auxiliares/logs/prueba_47.log`.
- resultado: `success=false`, código `1`.
- evidencia negativa: `scenario_runner_node` informa `bad file` porque recibió
  la ruta relativa desde el workspace superior; la trayectoria no comenzó.
- aprendizaje: usar ruta absoluta en esta herramienta. El intento mostró
  además reintentos innecesarios de candidatos aún sin pose; la cola se corrigió
  para esperar ambas poses antes de invocar 3O.
- conclusión: `NO CONSEGUIDA`; fallo operativo, no evalúa 3P.

## 2026-07-29 12:54 - `prueba_48`

- objetivo intentado: validar 3P con la trayectoria diferencial acordada.
- YAML:
  `/home/chenfu/Gazebo/src/codex/archivos_auxiliares/trayectorias/prueba_tipica_anclaje_diferencial.yaml`.
- launch: `ros2 launch simulacion_dron multi_dron.launch.py rawdb_record_enabled:=false`.
- log: `codex/archivos_auxiliares/logs/prueba_48.log`.
- reducido: `codex/archivos_auxiliares/logs/prueba_48.reduced.log`.
- resultado runner: código `0`, `SCENARIO-RUNNER-DONE success=true` y
  `SIM-DONE success=true`.
- decisiones 3O entregadas a 3P: `173`; `91 FUSION_CANDIDATE`,
  `80 REJECT` y `2 LOOP_OPTIMIZATION_CANDIDATE`.
- covisibilidad: las `91` fusiones candidatas producen `91` aristas
  `SERVER_LOOP_GEOMETRIC`: `58` inter-dron y `33` intra-dron.
- pares: `10366` recibidos, `6725` aplicados y `3641` rechazados. Todos los
  rechazos son `same_raw_mappoint`: loops intra-dron donde ambos lados ya
  señalan la misma identidad raw.
- acciones aplicadas: `1025 CREATE_TRACK`, `1283 ADD_MEMBER`,
  `4066 REINFORCE_TRACK` y `351 MERGE_TRACKS`.
- estado final: `674` tracks, `3333` miembros raw y `581` tracks multi-dron.
- cola pre-anchor: `46` candidatos aplazados, reintentados en bloques de `2` y
  `44`; termina con `still_pending=0`.
- publicación final: `16259` raw, `3333` miembros omitidos, `674`
  representantes y `13600` puntos publicados. Se cumple
  `16259 - 3333 + 674 = 13600`.
- exclusiones comprobadas: los dos candidatos de error alto quedan
  `handled=false`; 3P no crea `LoopOptimizationTask`, grafos, apply ni
  modificaciones de pose.
- errores graves: ninguno. El único `[ERROR]` es la muerte de Gazebo durante el
  cleanup posterior a `SIM-DONE`.
- revisión RViz2 del 2026-07-29: el usuario no distingue uniones concretas por
  la densidad de la nube. No comunica pérdidas ni anomalías y acepta que, en
  esta subfase, el log sea el criterio principal; espera que la fusión sea más
  visible cuando aumente el umbral de score en fases futuras.
- conclusión revisada: `CONSEGUIDA`; evidencia automática completa y revisión
  visual inconclusa pero no bloqueante por decisión expresa del usuario.

## 2026-07-29 13:33 - `prueba_49`

- objetivo intentado: repetir la trayectoria diferencial para comprobar la
  reproducibilidad de 3P mediante logs, dado que la densidad de RViz2 impide
  distinguir con claridad cada unión.
- cambios funcionales antes de esta repetición: ninguno.
- YAML:
  `/home/chenfu/Gazebo/src/codex/archivos_auxiliares/trayectorias/prueba_tipica_anclaje_diferencial.yaml`.
- launch: `ros2 launch simulacion_dron multi_dron.launch.py rawdb_record_enabled:=false`.
- log: `codex/archivos_auxiliares/logs/prueba_49.log`.
- reducido: `codex/archivos_auxiliares/logs/prueba_49.reduced.log`.
- resultado runner: código `0`, `SCENARIO-RUNNER-DONE success=true`,
  `SIM-DONE success=true` y `SIM-EXIT-CODE 0`.
- decisiones entregadas a 3P: `258`; `90 FUSION_CANDIDATE`, `110 REJECT` y
  `58 LOOP_OPTIMIZATION_CANDIDATE`.
- covisibilidad: los `90` candidatos de fusión producen exactamente `90`
  relaciones `SERVER_LOOP_GEOMETRIC`: `45` inter-dron y `45` intra-dron.
- pares: `9637` recibidos, `5614` aplicados y `4023` rechazados. Todos los
  rechazos son `same_raw_mappoint`.
- acciones aplicadas: `980 CREATE_TRACK`, `1040 ADD_MEMBER`,
  `3305 REINFORCE_TRACK` y `289 MERGE_TRACKS`.
- estado final: `691` tracks, `3000` miembros raw y `544` tracks multi-dron.
- publicación final de los dos submapas anclados: `18111` raw, `3000` miembros
  omitidos, `691` representantes y `15802` puntos publicados. Se cumple
  exactamente `18111 - 3000 + 691 = 15802`.
- variación de ejecución: `dron_1` detecta un reset y abre `epoch=1`. Ese tercer
  submapa no obtiene anchor y `GlobalMapBuilder` lo conserva fuera de la nube
  global con `skipped_unanchored=1`.
- cola pre-anchor: `95` candidatos se aplazan, se reintentan `2 + 61 = 63` y
  quedan `32`; se cumple `95 - 63 = 32`. Los pendientes dependen del submapa
  sin pose global, por lo que conservarlos es el comportamiento correcto.
- exclusiones comprobadas: los `58` candidatos de error alto quedan fuera de
  3P; no se crea ninguna tarea, grafo o apply de optimización por loop.
- errores graves: ninguno; no aparecen `ERROR`, `FATAL`, segfault, assertion ni
  core dump.
- comparación con `prueba_48`: se reproducen el volumen de candidatos de
  fusión (`90` frente a `91`), los tracks (`691` frente a `674`), la reducción
  exacta de puntos y la ausencia de optimización. Las diferencias son
  compatibles con la aparición del tercer submapa no anclado.
- revisión visual posterior: aunque la densidad impide seguir cada track, el
  usuario observa puntos desaparecer y cambiar de posición durante la
  ejecución, comportamiento compatible con omitir miembros raw y actualizar
  sus representantes fusionados. Elevar el umbral de score para verlo con más
  claridad queda para una fase posterior.
- conclusión revisada: `CONSEGUIDA`; segunda validación live reproducible con
  evidencia automática completa y comportamiento visual compatible.

## 2026-07-29 14:07 - `prueba_50`

- objetivo intentado: validar estabilidad, latencia y convivencia de 3P con la
  ruta fiducial en la trayectoria larga
  `prueba_tipica_rodeo_edificio_dos_fiduciales.yaml`.
- cambios funcionales antes de esta ejecución: ninguno respecto a
  `prueba_49`.
- launch:
  `ros2 launch simulacion_dron multi_dron.launch.py rawdb_record_enabled:=false`.
- log: `codex/archivos_auxiliares/logs/prueba_50.log`.
- reducido: `codex/archivos_auxiliares/logs/prueba_50.reduced.log`.
- resultado runner: código `0`, `SCENARIO-RUNNER-DONE success=true`,
  `SIM-DONE success=true` y `SIM-EXIT-CODE 0`; no hubo OOM, segfault,
  `bad_alloc` ni caída del servidor.
- observación RViz2: el usuario ve que los drones avanzan pero los puntos
  nuevos tardan varios segundos en publicarse; tampoco observa optimización de
  KFs al llegar a los fiduciales.
- carga geométrica: `505` verificaciones 3O, con media `0.431650 s`, máximo
  `0.874439 s`, `248` por encima de `0.5 s` y `217.983 s` acumulados.
- reintentos pre-anchor: dos lotes con `76` candidatos consumen `36.428 s`; el
  primer lote procesa `62` de forma síncrona y bloquea el callback `31.064 s`.
- causa de latencia: cada callback de delta/snapshot ejecuta en serie
  fiduciales, todos los reintentos, detección/verificación de cada KF y solo al
  final reconstruye/publica la nube. El nodo usa `rclcpp::spin`, y mapas, GT y
  temporizadores comparten `live_state_mutex_`; mientras 3O trabaja no se
  reciben datos ni se ejecuta el timer de publicación.
- coste 3P observado: `43 FUSION_CANDIDATE`; el tramo desde resultado
  geométrico hasta `[F1P-SUMMARY]` suma aproximadamente `0.181 s`, con máximo
  `0.037 s`. La fusión no explica las pausas de decenas de segundos.
- ruta fiducial: configuración de optimización y apply activadas, `12`
  asociaciones `[F1E-FID-KF-ASSOC]`, pero todas con `fid=2`; no se asocia
  ningún KF con `fid=1`, hay `9` revisits y `0` tareas.
- pérdida temporal: los candidatos fiduciales tienen un hueco completo de
  stamps `180-279 s`, intervalo que contiene la visita y barrido de
  `fiducial 1`. La carga síncrona bloquea callbacks y las colas
  `KeepLast(20/50)` descartan mensajes antiguos; al retomarse, los KFs ya no
  encuentran GT dentro de la tolerancia temporal.
- consecuencia: `accepted_visits_` conserva `fid=2` desde el anclaje inicial.
  Al volver a `fid=2`, cuatro revisits con error `25.3-25.6 m` se clasifican
  como `same_fiducial_visit_already_accepted`, no crean tarea y el solver nunca
  llega a ejecutarse.
- interpretación: no falla el solver ni está desactivado; la optimización no
  recibe ninguna tarea por starvation de callbacks y por la supresión de
  misma visita, que queda incorrectamente activa al perderse `fid=1`.
- archivos funcionales modificados tras la prueba: ninguno; el usuario pidió
  solo diagnóstico.
- conclusión: `NO CONSEGUIDA`. La estabilidad de proceso pasa, pero fallan la
  latencia de publicación y la ruta fiducial, dos objetivos funcionales de la
  regresión larga.
- siguiente paso recomendado: acordar antes de modificar cómo sacar la
  verificación geométrica/reintentos del callback de ingestión, imponer
  presupuesto por ciclo y hacer robusta la separación de visitas fiduciales.

## 2026-07-29 19:29 - `prueba_51`

- objetivo intentado: corregir la regresión de `prueba_50` con una cola ligera,
  un worker geométrico y backpressure global con histéresis, y repetir
  `prueba_tipica_rodeo_edificio_dos_fiduciales.yaml`.
- archivos funcionales modificados:
  `subcloud_loop_verifier.hpp/.cpp`, `global_map_server.cpp`,
  `scenario_runner_node.cpp` y dependencias CMake/package de
  `orbslam3_server` y `simulacion_dron`.
- arquitectura probada: cola de IDs/candidato, un worker fijo,
  preparación de subnubes separada del cálculo RANSAC, commit serializado,
  `backlog=queued+active`, umbrales `high=10`/`low=3` y topic Bool transient
  local `/global_mapping/backpressure_active`.
- control de prueba: el runner guarda destinos, cancela de forma intencionada,
  manda una meta corta de hasta `1 m` y `20 grados`, y reanuda el destino
  original. La pose/velocidad GT se usa solo para este frenado simulado; no
  alimenta mapa, loops, fusión ni la futura solución real.
- build:
  `./codex/herramientas/build_selected_packages.sh orbslam3_multi orbslam3_server simulacion_dron`;
  código `0`, tres paquetes correctos.
- ejecución: `prueba_51`, startup `20 s`, post `45 s`, timeout `1500 s`;
  log completo `prueba_51.log` y reducido `prueba_51.reduced.log`.
- resultado: `SCENARIO-RUNNER-STEP-FAILED` en el paso `13`,
  `SIM-SCENARIO-EXIT-CODE 124` y `SIM-EXIT-CODE 124`. No hubo segfault,
  OOM, `bad_alloc` ni caída de `global_map_server`.
- evidencia positiva de scheduling: `16` activaciones y `15`
  desactivaciones recibidas, `14` pausas completadas y `26` metas originales
  reenviadas. La histéresis activa exactamente en `10` y libera en `3`.
- evidencia positiva de publicación: `1095` publicaciones, intervalo medio
  `1.424 s` y máximo `24.426 s`; mejora sobre `prueba_50`
  (`3.232 s` medio, `32.343 s` máximo), aunque aún hay pausas visibles.
- evidencia fiducial recuperada: `fid=1` crea `task_id=1` con
  `error_t=1.131494 m`; aplica `37` KFs optimizados y `110` propagados,
  conserva el hard fiducial, reduce el error real a `0` y cierra la tarea por
  `post_apply_accept`.
- fusión activa: `182 FUSION_CANDIDATE`, `167` resúmenes de fusión,
  `2072` tracks creados, `463` merges y publicación final observada con
  `1609` tracks, `5517` miembros omitidos y `57452` puntos.
- carga geométrica: `882` trabajos terminados antes del cierre; preparación
  media/máxima `1082.4/23832.4 ms`, cálculo medio/máximo
  `300.3/835.0 ms` y tiempo total medio `1382.7 ms`.
- entrada de trabajo: `956` encolados; `403` desde delta, `344` desde
  `live_full_snapshot` y `209` desde retry tras anchor. Hay `747` pares únicos.
- revisión posterior de la interpretación: los `209` pares repetidos son todos
  `delta_retry_after_anchor`; ninguna repetición exacta procede de un snapshot.
  Además, los `344` queries de snapshot aparecen por primera vez en la cola:
  `DispatchLoopDetector` solo procesa `new_keyframe_ids`, por lo que el snapshot
  estaba recuperando KFs nuevos para el servidor, no reprocesando KFs iguales.
- ineficiencia pre-anchor real: `240` trabajos terminan
  `query_no_world_pose` y `2`, `candidate_seed_no_world_pose`. Se encolan y
  entran al worker solo para descubrir que aún no pueden verificarse; después
  deben reintentarse legítimamente al existir ambas poses. Esos rechazos
  consumen aproximadamente `114.6 s` de preparación, principalmente espera o
  uso de `live_state_mutex_`.
- fallo de convergencia: el último estado periódico conserva `74` trabajos
  en cola, `1` activo y `74` resultados calculados pendientes de commit. La
  carga de KFs nuevos y la liberación de candidatos pre-anchor producían
  trabajo más rápido que el único worker/commit.
- limitación del indicador: el backpressure acordado solo cuenta
  `queued+active`; no incluye `completed_waiting`, que llegó al menos a `88`.
  Por ello puede liberar movimiento mientras el commit serial sigue atrasado.
- tracking: se detectan `7` cambios de epoch entre ambos drones. La vuelta
  final a `fid=2` ancla `dron_2 epoch=3` y `dron_1 epoch=4`, pero la misión no
  completa el paso ni la espera final antes del timeout.
- conclusión: `NO CONSEGUIDA`. Se recuperan la ingestión concurrente, la
  publicación y la optimización fiducial que faltaban en `prueba_50`, pero la
  carga no converge y la trayectoria larga no termina.
- siguiente decisión necesaria antes de modificar: diferir candidatos sin pose
  antes del worker, contar `completed_waiting` en el backpressure y reducir el
  tiempo bajo `live_state_mutex_`. Para soportar cambios futuros de KFs se
  acuerda además una caché por revisiones: sin cambios no se reprocesa, cambios
  pequeños fusionan solo MPs nuevos y cambios materiales permiten RANSAC
  completo. No basta con ajustar `10/3`.

## 2026-07-29 19:47 - Segunda corrección de carga

- objetivo intentado: aplicar las tres correcciones acordadas tras
  `prueba_51`: defer pre-anchor antes del worker, captura acotada bajo
  `live_state_mutex_` con construcción de subnubes fuera y backpressure sobre
  toda la carga pendiente.
- archivos funcionales modificados:
  `subcloud_loop_verifier.hpp`, `subcloud_loop_verifier.cpp` y
  `global_map_server.cpp`.
- implementación:
  `CapturedLoopVerification` copia solo KFs, MapPoints, poses y scores de la
  query/ventana candidata; `CaptureCandidate` se ejecuta bajo el mutex y
  `PrepareCapturedCandidate` construye las subnubes fuera;
  `mapping_load=queued+active+completed_waiting+committing`;
  los callbacks drenan hasta `16` commits antes de admitir más trabajo;
  candidatos sin ambas poses se conservan directamente en
  `deferred_loop_candidates_`.
- límites conservados: un worker fijo, commits serializados, histéresis
  `high=10`/`low=3`, sin caché por revisiones ni fusión incremental en esta
  iteración.
- build:
  `./codex/herramientas/build_selected_packages.sh orbslam3_multi orbslam3_server simulacion_dron`;
  segundo build código `0`, tres paquetes correctos y sin stderr propio.
- conclusión de implementación: `CONSEGUIDA`; compila y expone tiempos
  separados `lock_wait_ms`, `capture_ms`, `prepare_ms` y `compute_ms`.

## 2026-07-29 19:55 - `prueba_52`

- objetivo intentado: repetir la ruta larga con dos fiduciales tras la segunda
  corrección de carga.
- YAML entregado al runner: ruta relativa
  `codex/archivos_auxiliares/trayectorias/prueba_tipica_rodeo_edificio_dos_fiduciales.yaml`.
- log: `codex/archivos_auxiliares/logs/prueba_52.log`.
- resultado: `scenario_runner_node` código `1` y simulación código `1`.
- evidencia negativa: `bad file`; el runner cambia su directorio de trabajo y
  no puede resolver esa ruta relativa. La trayectoria no empieza.
- conclusión: `NO CONSEGUIDA`; fallo operativo independiente que no evalúa los
  cambios funcionales. Se repite como `prueba_53` con ruta absoluta y sin
  cambiar código ni criterios.

## 2026-07-29 20:26 - `prueba_53`

- objetivo intentado: validar tiempo, estabilidad, publicación, fiduciales,
  backpressure y fusión con
  `prueba_tipica_rodeo_edificio_dos_fiduciales.yaml`.
- configuración: YAML absoluto, startup `20 s`, post `45 s`, timeout
  `1500 s`, un reintento Gazebo y
  `rawdb_record_enabled:=false`.
- logs: `codex/archivos_auxiliares/logs/prueba_53.log` y
  `prueba_53.reduced.log`.
- resultado: paso `13/13` iniciado pero no terminado;
  `SCENARIO-RUNNER-STEP-FAILED`, runner `124` y simulación `124`.
- estabilidad: no hay segfault, OOM, `bad_alloc` ni caída de
  `global_map_server`. El error Gazebo aparece durante cleanup forzado por el
  timeout.
- defer pre-anchor: `215` admisiones se difieren antes del worker
  (`192 query_no_world_pose`, `23 candidate_seed_no_world_pose`); `175` se
  liberan tras anchor y quedan `40`. Se cumple `215-175=40` y ningún
  `[F1P-WORK-DONE]` termina por falta de pose.
- carga admitida: `492` trabajos, todos pares query/candidate distintos;
  `161 delta`, `156 live_full_snapshot`, `104 delta_retry_after_anchor` y
  `71 live_full_snapshot_retry_after_anchor`. No hay repetición exacta de par.
- tiempos sobre `492` trabajos:
  `capture_ms` media/p95/máximo `26.1/35.6/48.5`;
  `prepare_ms` `47.3/65.3/86.3`;
  `compute_ms` `478.2/790.1/1064.5`;
  `lock_wait_ms` `1885.9/3964.0/13937.0`;
  `total_ms` `2437.5/4647.7/14700.2`.
- comparación con `prueba_51`: la preparación baja de
  `1082.4 ms` de media y `23832.4 ms` de máximo a `47.3/86.3 ms`.
  El coste restante dominante es esperar `live_state_mutex_`, porque los
  callbacks ROS todavía lo poseen durante ingestión, fiduciales, BoW,
  reconstrucción y publicación completas.
- commits/backpressure: pico anunciado `118`, máximo observado
  `completed_waiting=1`; ya no reaparece la cola de hasta `88` resultados
  calculados de `prueba_51`. Hay `6` activaciones exactas en carga `10` y `6`
  liberaciones exactas en carga `3`. Las pausas cerradas suman `1064.441 s`.
- cierre de cola: antes de reanudar el último movimiento se alcanza
  `481/481` y carga `0`. Durante los `45 s` post-runner entran nuevos deltas y
  se llega a `492` encolados/terminados; el shutdown corta el commit final, con
  último estado periódico `computed=491`, `committed=490` y una última
  terminación inmediatamente antes de `shutdown_join`.
- progreso: se completan `12` pasos. En el paso `13`, el backpressure pausa
  `112.728 s`, reanuda el destino original y solo quedan unos `24.4 s` hasta
  el timeout, insuficientes para la meta de `40 s`.
- publicación: `1137` publicaciones, intervalo medio `1.379 s` y máximo
  `12.677 s`; mejora frente a `prueba_51` (`1.424/24.426 s`).
- fiduciales: `48` asociaciones (`44 fid=2`, `4 fid=1`), `39 OK` y `5`
  supresiones de misma visita. La visita a `fid=1` sí se conserva; el último
  estado fiducial expone `4` anchors, `290` KFs optimizados y `289`
  propagados. La vuelta final no alcanza `fid=2`, por lo que no puede validar
  su asociación final.
- fusión: `203 FUSION_CANDIDATE`; `20226` pares recibidos,
  `12183` fusionados y `8043` rechazados; estado final publicado de `1926`
  tracks y `6869` miembros raw omitidos. La covisibilidad termina con `202`
  aristas `SERVER_LOOP_GEOMETRIC`.
- decisiones restantes: `190 REJECT`, `96 LOOP_OPTIMIZATION_CANDIDATE` fuera
  de 3P y `3 ALREADY_CONFIRMED_COVISIBILITY`.
- interpretación: las tres correcciones hacen lo previsto y eliminan los dos
  fallos internos concretos de `prueba_51`: trabajo sin pose y acumulación de
  commits. No bastan para el criterio temporal; el servidor sigue admitiendo
  demasiados pares distintos y el worker espera demasiado el mutex global.
- conclusión: `NO CONSEGUIDA` para la prueba larga y `PARCIAL` para 3P. La
  estabilidad, la publicación, la fusión y el control de carga mejoran, pero
  la misión no termina dentro de `1500 s` ni valida la vuelta final a `fid=2`.
- siguiente paso recomendado: debatir una nueva iteración antes de modificar.
  Debe reducir trabajo admitido mediante revisiones/caché e incrementalidad y
  estrechar el alcance de `live_state_mutex_`; aumentar solo el timeout o
  cambiar `10/3` ocultaría la causa.

## 2026-07-29 - Incrementalidad, covisibilidad nativa y fast path

- objetivo intentado: evitar redescubrir relaciones ya confirmadas y procesar
  solo asociaciones KF-MapPoint nuevas de snapshots.
- acuerdo funcional: importar toda covisibilidad ORB-SLAM3 positiva como
  confirmada antes de BoW, sin distinguir intra/inter-dron; un KF sin cambios
  no se reprocesa; si solo añade MapPoints, solo esos puntos se comparan con
  vecinos confirmados; una relación desconocida casi alineada puede confirmarse
  con matches estrictos distribuidos y expansión guiada.
- archivos funcionales: `raw_map_types.hpp`, `raw_map_database.cpp`,
  `covisibility_database.hpp/.cpp`, `subcloud_loop_verifier.hpp/.cpp`,
  `global_map_server.cpp` y tests de `orbslam3_multi`.
- arquitectura de carga: admisión física acotada por `high=10`, candidatos
  listos conservados en `throttled_loop_candidates_` y contabilizados en
  `mapping_load`; los candidatos sin pose permanecen separados.
- build: `orbslam3_multi`, `orbslam3_server` y `simulacion_dron`, código `0`.
- pruebas deterministas: covisibilidad, fused landmarks/fast path y tail
  anchor, código `0`.
- conclusión de implementación: `CONSEGUIDA`; faltaba validar la ruta larga.

## 2026-07-29 - `prueba_54`

- objetivo: primera regresión larga de la ruta incremental y fast path.
- resultado: interrumpida de forma diagnóstica en el paso 2.
- evidencia: el fast path incremental alcanzó `30312.359 ms`,
  `mapping_load=98` y `104` workers.
- causa: matching cartesiano repetido contra vecinos del mismo submapa.
- corrección posterior: identidad raw directa para el mismo submapa y hash
  espacial 3D para evitar el producto cartesiano.
- conclusión: `NO CONSEGUIDA`.

## 2026-07-29 - `prueba_55`

- objetivo: revalidar tras eliminar el matching cuadrático.
- resultado: timeout `124` en el paso 2.
- evidencia positiva: el máximo temprano del incremental bajó a
  `900.256 ms`.
- evidencia negativa: al obtener anchors se admitieron `122` trabajos de una
  vez y `mapping_load` llegó a `111`; el flag pausaba movimiento, pero no
  limitaba la admisión.
- corrección posterior: presupuesto de admisión, conservación de no admitidos,
  reintento de fast path por revisión y cancelación del BoW pendiente cuando la
  relación ya queda confirmada.
- conclusión: `NO CONSEGUIDA`.

## 2026-07-29 - `prueba_56`

- objetivo: validar el presupuesto de admisión y el fast path post-anchor.
- resultado: interrumpida tras obtener evidencia diagnóstica suficiente.
- evidencia positiva: carga física máxima `10`, `14` confirmaciones fast path
  y `13` trabajos BoW cancelados; ejemplos `64 -> 117` y `136 -> 240` pares
  strict/expanded.
- evidencia negativa: `104` trabajos listos aplazados se contaban como
  `deferred_without_pose` y no participaban en el flag.
- corrección posterior: separar `throttled_ready` de candidatos realmente sin
  pose, incluirlo en `mapping_load` y drenarlo internamente por capacidad.
- conclusión: `PARCIAL`.

## 2026-07-29 - `prueba_57`

- objetivo: validar la contabilidad completa de `throttled_ready`.
- resultado: interrumpida a petición del usuario mientras avanzaba por la ruta,
  porque cambió la política de actuación del runner.
- evidencia previa a la interrupción: la primera ola drenó
  `127/127` trabajos y la misión avanzó más allá del paso 2; la carga física se
  mantuvo alrededor de `10`.
- motivo de interrupción: ya no se quería cancelar el goal activo ni fabricar
  una maniobra corta con GT. La nueva regla exige terminar el movimiento actual
  y bloquear únicamente el siguiente.
- conclusión: `BLOQUEADA` como prueba de la política antigua; no evalúa la
  política finalmente acordada.

## 2026-07-30 - `prueba_58`

- objetivo: validar la ruta larga con backpressure como puerta entre
  trayectorias, incrementalidad, fast path, fusión y fiduciales.
- cambio del runner: no cancela ni sustituye goals activos; los pasos `wait`
  transcurren; antes de cada nuevo movimiento espera a `flag=false`; se retiró
  todo el control GT usado para frenado.
- build: `orbslam3_multi`, `orbslam3_server` y `simulacion_dron`, código `0`.
- tests deterministas: tres ejecutables de `orbslam3_multi`, código `0`.
- ejecución: segundo arranque Gazebo correcto tras el reintento automático;
  runner `0`, `SIM-DONE success=true`, simulación `0` y `14/14` pasos.
- política validada: `18` goals enviados y `18` resultados `success=true`;
  `6` esperas y `6` aperturas de puerta; `0` cancelaciones, goals de frenado,
  reenvíos, pausas de `wait` o suscripciones GT del runner.
- esperas de puerta: `106.219`, `49.248`, `145.042`, `129.829`, `112.907` y
  `137.435 s`; total `680.680 s`. Un `wait` de `8 s` transcurre con el flag
  activo antes de bloquear el movimiento siguiente.
- duración: escenario `1048.045 s`; no hay timeout ni caída del servidor.
- carga: pico `mapping_load=103`; al shutdown quedan `mapping_load=15`,
  `464` encolados, `455` calculados, `454` committed, `5 throttled_ready` y
  `26 deferred_without_pose`. Los `45 s` post-escenario no drenan la última
  ola.
- tiempos de `455` workers: `lock_wait_ms` medio/máximo
  `1446.648/12965.362`; `prepare_ms` `51.429/94.411`; `compute_ms`
  `445.124/853.017`; total `1962.156/13457.358`.
- incremental/fast path: `579` resúmenes, `326` pares incrementales, una
  confirmación alineada, `155` matches estrictos y `210` expandidos.
- decisiones: `91 FUSION_CANDIDATE`, `199 LOOP_OPTIMIZATION_CANDIDATE`,
  `187 REJECT` y `4 ALREADY_CONFIRMED_COVISIBILITY`.
- fusión final observada: `1114` tracks, `3286` miembros raw omitidos y
  `43245` puntos publicados.
- publicación fused: `1031` mensajes, hueco medio/máximo
  `1.075/13.240 s`.
- fiduciales: `46` asociaciones aceptadas; en `fid=1` se crean dos tareas con
  errores `2.741528 m` y `14.946024 m`, ambas se aplican y cierran; después se
  observan errores `0.213408`, `0.081457` y `0.077787 m`.
- evidencia ausente: el último movimiento llega de nuevo a `fid=2`, pero no se
  publica una nueva asociación fiducial de KF durante esa visita.
- errores: solo las dos salidas `255` de Gazebo, una del primer startup y otra
  del cleanup posterior a `SIM-DONE`; no hay crash/OOM del servidor.
- conclusión: `PARCIAL`. La nueva política del runner queda `CONSEGUIDA` y la
  ruta completa por primera vez termina dentro del timeout; 3P sigue parcial
  porque el coste continúa alto, la cola no queda vacía y falta evidencia de
  asociación en la vuelta final a `fid=2`.
- siguiente paso recomendado: no retocar la puerta. Debatir por separado la
  reducción adicional del volumen BoW/worker y cómo garantizar evidencia
  fiducial en la visita final cuando ORB-SLAM3 no crea un KF nuevo.

## 2026-08-03 21:04 - `prueba_59` - repetición visual sin cambios

- objetivo intentado: repetir
  `prueba_tipica_rodeo_edificio_dos_fiduciales.yaml` para que el usuario
  observase otra ejecución completa en RViz2, sin modificar código, launch,
  YAML ni parámetros.
- archivos funcionales modificados: ninguno.
- paquetes compilados: ninguno; se reutilizó el build validado por
  `prueba_58`.
- prueba Gazebo: `prueba_59`, runner `0`, `SIM-DONE success=true`, simulación
  `0`, `14/14` pasos y `18/18` goals con `success=true`.
- patrones de reducción: `SIM-`, `SCENARIO-RUNNER`, `F1P-WORK-*`,
  `F1P-BACKPRESSURE`, fusión, fiduciales, `ERROR`, `FATAL`, `Killed` y
  `bad_alloc`.
- puerta entre movimientos: cinco esperas de `127.634`, `70.775`, `172.292`,
  `145.173` y `168.976 s`; total `684.850 s`. Ningún goal activo se cancela o
  sustituye.
- duración y carga: escenario `1052.076 s`, pico `mapping_load=114`; los
  `416` trabajos encolados terminan calculados y committed, y la carga llega a
  cero antes del cierre. La carga física se limita a `9 queued + 1 active`,
  pero `throttled_ready` alcanza `104`. Quedan `43 deferred_without_pose`: no
  cuentan en `mapping_load`, pero sí representan candidatos sin resolver porque
  sus submapas carecen de pose world.
- tiempos de worker: `lock_wait_ms` medio/máximo `1500.009/22611.083`,
  `prepare_ms` `46.487/99.792`, `compute_ms` `417.993/883.822` y total
  `1987.843/23065.878` sobre `416` trabajos. Acumulan `826.943 s`: `624.004 s`
  (`75.5 %`) esperando el mutex y `173.885 s` (`21.0 %`) en geometría.
- verificaciones worker: `184` intra-dron (`389.839 s`, media `2.119 s`) y
  `232` inter-dron (`437.104 s`, media `1.884 s`). Sus decisiones son
  `108 FUSION_CANDIDATE`, `130 LOOP_OPTIMIZATION_CANDIDATE`, `176 REJECT` y
  `2 ALREADY_CONFIRMED_COVISIBILITY`.
- fusiones a nivel KF: `210` operaciones, pero `116` pares únicos de KFs.
  Intra-dron hay `64/64` pares únicos, todos verificados por worker. Inter-dron
  hay `146` operaciones sobre `52` pares únicos: `44` confirmaciones del worker,
  `8` confirmaciones fast path nuevas y `94` actualizaciones incrementales de
  pares ya confirmados.
- tiempos de las confirmaciones que acaban en fusión: intra-dron `64`,
  `136.754 s` acumulados, media/máximo `2.137/23.066 s`; inter-dron worker `44`,
  `99.047 s`, media/máximo `2.251/9.894 s`. La aplicación real en
  `FusedLandmarkManager` es pequeña: intra `0.171 s` total, `2.670 ms` de media;
  inter `0.350 s`, `2.397 ms` de media. El fast/incremental completo consume
  `50.459 s` en `563` ingestas, con máximo síncrono de `5.178 s`.
- pares de MapPoints: intra-dron recibe `7852`, fusiona `1749` y rechaza `6103`
  exclusivamente por `same_raw_mappoint`; inter-dron recibe y fusiona
  `5918/5918`. El estado final contiene `1530` tracks, `5456` miembros raw y
  `1103` tracks multi-dron.
- publicación final: `1530` tracks, `5456` miembros raw omitidos y `32611`
  puntos, coherente con `36537 - 5456 + 1530 = 32611`; `978` publicaciones
  fused con hueco medio/máximo `1.134/22.009 s`.
- tracking: excluyendo la inicialización, `dron_1` pierde tracking tres veces
  (`4.001`, `6.041` y `10.046 s` aproximados) y avanza de epoch `0` a `4` con
  cuatro eventos de reset; `dron_2` lo pierde una vez (`4.297 s`) y pasa de
  epoch `0` a `1`. Todos los episodios ocurren después de `fid=1`.
- submapas y nube tras `fid=1`: RawDB termina con `6` submapas, `535` KFs y
  `46668` MapPoints; solo `2` submapas están anclados y `4` se omiten. La nube
  usa `36537` puntos raw anclados, por lo que `10131` MapPoints raw quedan fuera.
  Desde la segunda optimización hay `293` publicaciones y el conteo cambia
  `29189 -> 32611`, pero desde el inicio del movimiento final queda fijo. En ese
  último tramo RawDB aún añade `51` KFs y `6148` MapPoints sin que cambien
  puntos, tracks ni miembros publicados.
- fiduciales: `56` asociaciones (`18` con `fid=1`, `38` con `fid=2`); las dos
  tareas de `fid=1` se aplican, aceptan y cierran. Tras el movimiento final no
  aparece una nueva asociación de KF con `fid=2`.
- evidencia negativa o ausente: el coste sigue dominado por las puertas y la
  espera del mutex; la vuelta final al fiducial 2 mantiene la carencia de
  evidencia ya vista en `prueba_58`.
- revisión visual del usuario: ambos submapas anteriores se optimizan bien al
  llegar a `fid=1`, pero después no aprecia puntos nuevos ni cambios de pose y
  toda la ejecución resulta demasiado lenta. El log matiza que sí hubo cambios
  de conteo antes de las pérdidas, pero confirma una nube totalmente congelada
  durante el movimiento final. Las poses no vuelven a optimizarse porque las
  `46` decisiones `LOOP_OPTIMIZATION_CANDIDATE` posteriores a `fid=1` pertenecen
  a `3Q` y `3P` no las aplica.
- errores: la salida `255` de Gazebo ocurre durante el cleanup posterior a
  `SIM-DONE`; no hay crash, OOM ni caída del servidor.
- conclusión: `PARCIAL`. La repetición visual y la política del runner se
  ejecutan correctamente, pero `mapping_load=0` no equivale a trabajo semántico
  resuelto: quedan candidatos diferidos y cuatro submapas sin anchor. El tiempo
  es excesivo, la nube se congela en el regreso y falta la asociación fiducial
  final.
- siguiente paso recomendado: antes de modificar, debatir por separado la
  pérdida/reanclaje de epochs y el coste del camino común de verificación. La
  fusión de tracks en sí no es el cuello de botella.

## 2026-08-04 - Acuerdo de rediseño de rendimiento pendiente

- objetivo intentado: analizar atajos previos a validacion geometrica, el uso
  de `live_state_mutex_` y el trabajo esperado tras full snapshots, y dejar
  especificada la siguiente correccion sin modificar funcionalidad.
- archivos funcionales modificados: ninguno.
- paquetes compilados: ninguno.
- pruebas Gazebo/replay: ninguna; se conserva `prueba_59` como evidencia base y
  `prueba_tipica_rodeo_edificio_dos_fiduciales.yaml` como regresion acordada.
- auditoria positiva: covisibilidad ORB se importa antes de loops; una arista
  confirmada se comprueba en detector, admision y verificador; KFs sin cambios
  no se redispatchan como nuevos; asociaciones nuevas usan ruta incremental.
- auditoria negativa: `loop_work_keys_` solo deduplica en vuelo y se borra tras
  cualquier commit; no existe cache negativa por revisiones;
  `FindUnknownAlignedOverlaps` excluye el propio submapa; callbacks completos
  mantienen `live_state_mutex_` durante operaciones amplias.
- acuerdo funcional pendiente: captura-calculo-commit con revisiones; prioridad
  a commits e importacion/cancelacion ORB; snapshot materialmente diferencial y
  no-op; cache negativa canonica separada de `CovisibilityDatabase`; fast path
  intra-submapa por IDs raw compartidos distribuidos; metricas especificas.
- limites: no eliminar sincronizacion, no crear un thread por tarea, no cachear
  falta temporal de pose/anchor como rechazo, no cambiar grafos, optimizacion,
  poses ni politica de backpressure del runner.
- resultado de build y pruebas: no ejecutados porque solo se modificaron MDs.
- conclusión: acuerdo documental `CONSEGUIDO`; estado funcional 3P permanece
  `PARCIAL` y la autorizacion de codigo/configuracion queda `PENDIENTE`.
- siguiente paso recomendado: una orden posterior equivalente a
  `implementalo segun lo acordado` autoriza este alcance cerrado; despues deben
  realizarse build, tests locales y la regresion larga acordada.

## 2026-08-04 11:24 - Rediseño de rendimiento implementado

- objetivo intentado: ejecutar el acuerdo cerrado de captura-calculo-commit,
  revisiones raw, snapshots diferenciales/no-op, cache negativa canonica y
  fast path intra-submapa sin iniciar `3Q`.
- archivos funcionales modificados: `raw_map_types.hpp`,
  `raw_map_database.hpp/.cpp`, `loop_verification_result.hpp`,
  `subcloud_loop_verifier.hpp/.cpp`, `loop_decision_manager.cpp`,
  `global_map_server.cpp`, `CMakeLists.txt` y tests de `orbslam3_multi`.
  Se añadieron `LoopPairAttemptDatabase` y su test.
- comportamiento implementado: el worker captura estado y revisiones bajo
  mutex, calcula fuera y valida el commit; la nube se construye/publica fuera
  del mutex; se importan aristas ORB y se cancela trabajo ya confirmado antes
  del fallback; los snapshots sin cambios materiales omiten geometria/nube;
  los rechazos geometricos definitivos se cachean por par/revisiones; el fast
  path admite soporte distribuido por `RawMapPointId` compartidos y expande
  unicamente IDs distintos.
- paquetes compilados: `orbslam3_multi`, `orbslam3_server` y
  `simulacion_dron`, dos builds con codigo `0`; solo aparecen warnings previos
  de `optimization_manager.cpp`.
- tests: `test_covisibility_database`, `test_loop_pair_attempt_database`,
  `test_fused_landmark_manager` y `test_global_pose_store_tail_anchor`, todos
  con codigo `0`.
- conclusion: `PARCIAL`. El bloque compila y sus tests deterministas pasan;
  quedaba pendiente la regresion Gazebo larga.

## 2026-08-04 11:27 - `prueba_60` - YAML relativo no evaluable

- objetivo intentado: ejecutar la ruta larga de dos fiduciales tras el
  rediseño de rendimiento.
- prueba Gazebo: `prueba_60`; `scenario_runner_node` termina con codigo `1`
  antes de recorrer el escenario porque no puede abrir la ruta YAML relativa.
- patrones de reduccion: runner, backpressure, worker, `F1P-*`, fiduciales y
  errores graves; el log completo solo se uso como entrada del reductor.
- evidencia positiva: Gazebo y el backend arrancan; aparecen las nuevas
  metricas de captura/calculo/commit.
- evidencia negativa: no se ejecuta la ruta y no se evalua rendimiento 3P.
- conclusion: `NO CONSEGUIDA` operativa; intento independiente equivalente a
  la clase de fallo de `prueba_52`.
- siguiente paso: repetir con ruta YAML absoluta. Antes de repetir se corrigio
  mecanicamente la distincion entre nube cacheada valida de cero puntos y
  ausencia de cache; el segundo build paso con codigo `0`.

## 2026-08-04 11:28 - `prueba_61` - gate bloqueado por revisiones obsoletas

- objetivo intentado: validar el rediseño con la ruta YAML absoluta y observar
  que el backpressure drenase sin repetir trabajo redundante.
- prueba Gazebo: `prueba_61`, detenida tras la observacion del usuario de que
  ambos drones permanecian en la primera visita a `fid=2`; runner y launch
  quedaron verificados como terminados.
- patrones de reduccion: `SCENARIO-RUNNER-*`, `F1P-BACKPRESSURE`,
  `F1P-WORK-STATS`, `F1P-WORK-COMMIT-SKIP`, `stale_revision_*`,
  `F1P-INGEST-CAPTURE-COMPUTE-COMMIT`, deltas raw y score. El log completo no
  se abrio.
- evidencia del runner: ambos goals del paso 2 terminan `success=true` en
  `40 s`; el `wait` de `8 s` del paso 3 termina; el paso 4 queda en
  `SCENARIO-RUNNER-MOVE-GATE-WAIT` durante `635.830 s` y falla. No hay un goal
  de movimiento colgado.
- evidencia de carga: backpressure se activa en `mapping_load=10`, sube a
  `59-72` y nunca vuelve a `false`. Al final se observan `3315` encolados,
  `3306` iniciados, `3305` calculados, contador `committed=3305`,
  `throttled_ready=62` y `mapping_load=72`; `committed` tambien cuenta retiros
  obsoletos, no solo decisiones geometricas aplicadas.
- causa confirmada en codigo: `RawMapDatabase` compara el mensaje completo de
  cada `OrbMapPoint`; cualquier diferencia incrementa una unica
  `submap.geometry_revision` y copia esa revision a todos los KFs del submapa.
  Deltas recurrentes con `kfs=0` y MPs actualizados invalidan asi todos los
  pares capturados, aunque el cambio no afecte al KF concreto.
- ciclo observado: cada commit obsoleto se salta y se reencola inmediatamente
  mediante `stale_revision_retry`; el siguiente delta MP-only vuelve a cambiar
  la revision. Una llegada final contiene `kfs=0`, `mps=1474` actualizados y
  `mutex_capture_ms=392.999`, seguida de rechazos
  `stale_revision_before_capture` y nuevo drenaje de la misma carga.
- conclusion: `NO CONSEGUIDA`. La inmovilidad es la politica correcta del gate
  frente a un flag que nunca libera; el defecto esta en la granularidad de
  revision y el reencolado autosostenido del backend.
- siguiente paso recomendado: separar cambios geometricos de metadatos de MP,
  invalidar solo KFs afectados y evitar reencolado inmediato repetido para el
  mismo par/revision. Mantener la captura diferencial como segunda deuda, pues
  copiar el estado alcanza aproximadamente `393 ms` en esta prueba.

## 2026-08-04 12:57 - Flujo incremental jerarquico acordado

- objetivo: convertir el diagnostico de `prueba_61` y la propuesta del usuario
  en un contrato 3P claro, sin modificar codigo ni ejecutar pruebas.
- acuerdo: cada KF/MP nuevo se procesa una vez por revision material; se importa
  ORB, se resuelve primero identidad raw/track y matching espacial, despues BoW
  y solo al final RANSAC para relaciones debiles o desconocidas.
- responsabilidades separadas: `RawMapDatabase` conserva raw/revisiones;
  `FusedLandmarkManager`, identidad MP; `CovisibilityDatabase`, evidencia
  positiva KF-KF; y el estado canonico del par conserva rechazos/pending por
  etapa y revisiones.
- fuerza de arista: toda arista ORB positiva se almacena, pero solo soporte
  absoluto/relativo y cobertura 2D/3D fuertes permiten saltar RANSAC. Un punto
  compartido o una arista debil no bastan.
- orden MP: radio espacial, misma `RawMapPointId`, mismo fused track, Hamming,
  unicidad y cobertura. No se guardan negativos MP-MP combinatorios.
- BoW/cache: una revision visual se procesa una vez; solo pares puntuados
  explicitamente pueden quedar `BOW_REJECTED`. RANSAC rechazado se recuerda por
  revisiones de geometria/pose.
- concurrencia: maximo un trabajo por par; una revision nueva reemplaza/agrupa
  el pendiente; un resultado obsoleto se descarta sin retry inmediato; la
  captura no copia completas las bases.
- snapshots: revisiones separadas de apariencia, asociaciones, geometria, pose
  y metadatos; los metadatos no invalidan geometria y solo cambian los KFs que
  observan un MP geometrico modificado.
- frontera 3Q: error bajo fusiona; error alto conserva loop, medicion e inliers
  como `OPTIMIZATION_PENDING`. 3P no optimiza ni fusiona antes del apply 3Q.
- archivos documentales: `subfase_3P.md`, especificacion, implementacion,
  testing y criterios.
- build/tests/simulacion: no ejecutados; cambio exclusivamente documental.
- conclusion: acuerdo documental `CONSEGUIDO`; implementacion funcional
  `PENDIENTE` y 3P permanece `PARCIAL`.

## 2026-08-04 13:25 - `prueba_62` - correccion jerarquica validada automaticamente

- objetivo: implementar el acuerdo posterior a `prueba_61`, compilar y repetir
  `prueba_tipica_rodeo_edificio_dos_fiduciales.yaml` con ruta absoluta.
- codigo: revisiones `pose/associations/appearance/geometry/metadata` por KF;
  invalidacion de MPs solo sobre KFs observadores; snapshot raw parcial;
  aristas fuertes por soporte/cobertura; identidad raw/track previa a Hamming;
  memoria canonica por etapa; un trabajo por par y stale sin retry inmediato.
  Los pares ya identicos no se entregan a fusion y error alto queda
  `OPTIMIZATION_PENDING` sin iniciar 3Q.
- build: `orbslam3_multi orbslam3_server simulacion_dron`, codigo `0`. Solo
  persisten tres warnings previos de `optimization_manager.cpp`.
- tests: `test_covisibility_database`, `test_loop_pair_attempt_database`,
  `test_fused_landmark_manager` y `test_global_pose_store_tail_anchor`, todos
  codigo `0`.
- escenario: `14/14` pasos, `18/18` goals `success=true`, runner/simulacion
  codigo `0`, duracion aproximada `606.627 s`. Frente a `prueba_59`
  (`1052.076 s`, `684.850 s` de gate), las tres esperas efectivas son
  `146.324`, `62.410` y `30.525 s`, unos `239.259 s` en total.
- carga: `428/428/428/428` trabajos enqueued/started/computed/committed,
  `mapping_load` final `0`, pico `148` y `18 deferred_without_pose`. Las `97`
  revisiones stale se descartan con `captured_state_changed`; no existe
  `stale_revision_retry` ni crecimiento autosostenido. Solo se coalesce una
  revision adicional.
- tiempos worker: `lock_wait_ms` p50/p95 `0.012/164.485 ms`, media
  `264.388 ms`; `total_ms` p50/p95 `453.869/886.371 ms`. Dos esperas de lock
  superiores a `22 s` sesgan la media y coinciden con procesamiento amplio de
  snapshots/reconciliacion, no con RANSAC bajo mutex.
- ingesta: `81` bloques materiales con captura media/max
  `60.554/308.922 ms`, calculo fuera del mutex `2254.847/10716.317 ms`, commit
  `3.448/48.751 ms` y cero resultados stale del fast path/BoW. Otras `81`
  llegadas cambian solo metadata y crean cero trabajo de loop.
- memoria y fast path: `142` rechazos geometricos definitivos almacenados y
  cero hits en esta ruta; `513` queries alineadas, `117` confirmaciones y
  `122170` identidades raw resueltas sin descriptor. Se registran `112`
  `OPTIMIZATION_PENDING`, sin fusion ni tarea de optimizacion de loop.
- salida: estado final con `12372` aristas (`295` geometricas), `1652` tracks,
  `4425` miembros raw omitidos y `34725` puntos publicados. RawDB termina con
  `5` submapas; `3` estan anclados y dos epochs transitorios del dron 1 quedan
  fuera. El dron 1 reinicia tres veces durante la segunda mitad.
- fiduciales: una tarea de `fid=1` se crea, aplica y acepta con error final
  cero. La vuelta final a `fid=2` produce observaciones de ambos drones con
  errores `0.155`, `0.168` y `0.057 m`, todos bajo umbral.
- errores: ninguno grave durante la prueba. El exit `255` de Gazebo es posterior
  a `SIM-DONE` y pertenece al cleanup.
- conclusion: `PARCIAL` automatica. La regresion que bloqueaba `prueba_61` esta
  corregida y el rendimiento mejora materialmente, pero el contrato exige
  confirmacion visual RViz2 antes de declarar 3P conseguida. Riesgo residual:
  picos aislados de mutex y submapas transitorios sin anchor.
- artefactos: `codex/archivos_auxiliares/logs/prueba_62.log` conservado sin
  lectura directa y `prueba_62.reduced.log` generado por reductores tematicos.

## 2026-08-04 20:09 - `prueba_63` - repeticion visual sin cambios

- objetivo: repetir exactamente la ruta de `prueba_62` para inspeccion del
  usuario en RViz2, sin modificar codigo, launch, YAML, parametros ni
  configuracion y sin ejecutar un nuevo build.
- escenario: `14/14` pasos, `18/18` goals `success=true`, runner y wrapper con
  codigo `0`. Desde el inicio del paso 1 hasta `SCENARIO-RUNNER-DONE`
  transcurren aproximadamente `581.840 s`.
- carga 3P: cinco gates esperan `48.547`, `7.807`, `81.781`, `27.627` y
  `48.745 s`, unos `214.507 s` en total. Se completan `340/340` trabajos,
  `mapping_load` termina en `0`, el pico es `63` y quedan
  `30 deferred_without_pose`.
- salida automatica: la ultima nube reducida observada contiene `35553` puntos,
  con `2` submapas anclados y `3` sin anchor. No aparecen fallos del runner,
  servidor, RANSAC ni memoria; el `exit 255` de Gazebo ocurre tras `SIM-DONE`
  durante cleanup.
- fiducial: se crea `task_id=1` para `fid=1` con error
  `30.071874 m / 3.035596 rad`. El solver reduce el target a cero, pero 3J
  rechaza el resultado con `useful=false`, `reason=delta_too_large`; no hay
  apply de 3K. Esta repeticion no reproduce la optimizacion aceptada de
  `prueba_62`.
- patrones de reduccion: runner/SIM, backpressure y worker, publicacion final,
  tareas fiduciales, `F1J-OPT`, `F1K-OPT` y errores graves. El log completo no
  se abrio y solo alimento el reductor.
- evidencia visual revisada: el usuario considera que los tiempos ya no son
  malos, pero no observa optimizacion al llegar a `fid=1` ni en `fid=2`.
  El reducido explica `fid=1` por el rechazo `delta_too_large`. La primera
  visita a `fid=2` crea los anchors y sus revisits quedan bajo umbral; en la
  vuelta final no aparece una nueva asociacion KF-fiducial, por lo que no puede
  crearse una tarea.
- conclusion: `PARCIAL`. El rendimiento 3P queda visualmente aceptable; el
  pendiente es retirar el veto por magnitud de correccion y decidir por
  separado si la vuelta final debe garantizar una nueva observacion de
  `fid=2`.
- artefactos: `codex/archivos_auxiliares/logs/prueba_63.log` conservado sin
  lectura directa y `prueba_63.reduced.log` generado.

## 2026-08-04 - Acuerdo post-`prueba_63`

- optimizacion: retirar de dry-run y apply todos los vetos por magnitud de
  traslacion o yaw. Los deltas permanecen como metricas diagnosticas; se
  conservan coste, mejora, error final, poses finitas, hard fiducials,
  preservacion de anchors, validacion post-apply y rollback.
- fiduciales: se conserva la regla normal. El primer fiducial ancla; cualquier
  observacion posterior cuyo KF supere los umbrales de error crea optimizacion,
  independientemente de cuanto deban moverse los KFs. No se añade una ruta
  especial para `fid=2`.
- backpressure: se reutiliza exclusivamente
  `/global_mapping/backpressure_active`; no se crea otro flag. Su valor es la
  carga 3P con histeresis `10/3` OR una o mas optimizaciones ejecutables/en
  vuelo.
- runner: conserva la politica validada; el goal activo termina y el siguiente
  queda bloqueado hasta que el mismo flag vuelva a `false`.
- frontera futura: las optimizaciones por loop deberan participar en el mismo
  contador cuando 3Q las implemente; este cambio no inicia 3Q.
- prueba propuesta: build de `orbslam3_multi`, `orbslam3_server` y
  `simulacion_dron`, tests de optimizacion/backpressure y repeticion visual de
  la ruta larga como `prueba_64`.
- estado: preparacion cerrada, implementacion pendiente de autorizacion
  explicita.

## 2026-08-04 21:05 - `prueba_64` - delta libre y backpressure combinado

- objetivo intentado: retirar todo veto de optimizacion por magnitud de
  traslacion/yaw, usar el unico backpressure como OR de carga 3P y
  optimizaciones en vuelo, y repetir la ruta larga.
- archivos funcionales: `optimization_manager.hpp/.cpp`,
  `global_map_server.cpp`, `global_orb_map_server.launch.py` y
  `test_global_pose_store_tail_anchor.cpp`.
- cambio: desaparecen `delta_too_large`, `partial_delta_too_large` y sus tres
  parametros. `max_delta_t/max_delta_yaw` quedan solo como diagnostico. El
  topic existente combina el latch 3P `10/3` con
  `optimization_jobs_in_flight>0`; no se crea otro flag ni cambia el runner.
- build: primer intento codigo `2` por `No space left on device`; segundo
  intento de `orbslam3_multi orbslam3_server simulacion_dron`, codigo `0`.
- tests: `test_global_pose_store_tail_anchor` incluye dry-run/apply por encima
  de los antiguos limites; tambien pasan covisibility, loop-pair cache y fused
  landmark, todos codigo `0`.
- escenario: `14/14` pasos, `18/18` goals, runner/wrapper codigo `0` y duracion
  aproximada `689.249 s`. Tres gates esperan `100.091`, `180.672` y
  `41.342 s`, en total `322.105 s`.
- optimizacion: `fid=1` crea `task_id=1` con `1.406622 m / 0.211183 rad`; el
  dry-run llega a cero y queda `useful=true`. El apply mueve `18` KFs,
  propaga `123`, conserva el hard fiducial y se acepta con error real cero e
  `internal_max_after=0.290937 m`. El ciclo en vuelo dura `4.502 s` y termina
  en cero; no aparece ningun rechazo por delta.
- backpressure: las transiciones 3P siguen activando en `10` y liberando en
  `3/1/2`. La optimizacion se agenda mientras la causa 3P ya estaba activa, de
  modo que esta ejecucion valida el contador compartido pero no una transicion
  aislada causada solo por optimizacion.
- rendimiento: `401` workers encolados, `399` iniciados/terminados y dos
  pendientes de inicio al shutdown. Hay `80` stale antes de captura y `84`
  commits stale. `lock_wait_ms` medio/maximo `498.392/50968.234`, y el tiempo
  worker medio/maximo `856.432/51196.533 ms`.
- ingesta/nube: `78` bloques, captura media/maxima
  `78.606/585.934 ms`, calculo externo medio/maximo
  `2846.638/31588.373 ms`; la nube final publica `31375` puntos,
  `1657` tracks y omite `5865` miembros raw.
- carga final: el ultimo estado periodico mide `mapping_load=10` y el ultimo
  cambio del topic permanece `active=true`. Por los contadores quedan dos
  trabajos sin iniciar y ocho resultados terminados sin commit al cierre.
- fiduciales/tracking: no aparece una nueva asociacion a `fid=2` durante la
  vuelta final, por lo que no puede crearse otra tarea. RawDB termina con seis
  submapas, tres anclados; `dron_1` cambia de epoch tres veces y `dron_2` una.
- revision visual posterior: el usuario observa KFs de tramos posteriores antes
  que KFs anteriores y no ve optimizacion del dron antihorario en `fid=1`.
  El antihorario es `dron_2`: entra en perdida/reinicializacion antes de
  `fid=1`, cambia de `epoch=1` a `epoch=2` y `fid=1` pasa a ser el primer
  anchor del submapa nuevo (`kf=245`). No existe por tanto una ventana
  fiducial previa `fid=2 -> fid=1` dentro de ese submapa y no se crea tarea.
  La unica optimizacion de esta ejecucion corresponde a `dron_1`.
- diagnostico del orden visual: `orb_map_delta` contiene KFs nuevos o
  modificados, no un prefijo cronologico. En `dron_2 epoch=1` se recibe primero
  un delta con KFs `156..166` y despues otro que vuelve a incluir desde `153`.
  Los full snapshots recuperan ademas bloques antiguos no incorporados, y los
  submapas sin anchor permanecen ocultos hasta que el primer anchor hace
  publicable el bloque acumulado. El servidor ingiere el orden recibido y no
  impone orden cronologico de publicacion. No es movimiento causado por la
  optimizacion, sino llegada, reconciliacion y habilitacion tardias.
- evidencia reducida adicional:
  `codex/archivos_auxiliares/logs/prueba_64.visual_review.log`. El log completo
  no se abrio.
- errores: el unico error grave es `gazebo exit 255` posterior a `SIM-DONE`,
  durante cleanup. El log completo se conserva sin lectura directa y solo
  alimenta reductores/sublogs.
- conclusion: `PARCIAL`. La eliminacion de los vetos y el ciclo de
  backpressure de optimizacion quedan funcionalmente conseguidos, pero esta
  repeticion es unos `107.409 s` mas lenta que `prueba_63`, no drena la carga
  final, vuelve a carecer de observacion final de `fid=2`, publica KFs por
  bloques sin garantia cronologica y pierde la continuidad fiducial de
  `dron_2` al cambiar de epoch antes de `fid=1`.
- siguiente paso recomendado: preparar por separado una correccion de ingesta
  y publicacion ordenada/reconciliable, y decidir como tratar la continuidad
  entre epochs tras una perdida de tracking. No iniciar `3Q` implicitamente ni
  intentar relacionar submapas reseteados sin acuerdo funcional.

## 2026-08-04 - acuerdo de publicacion reactiva y diagnostico de KFs

- acuerdo: cada delta material se compromete primero en las bases de datos y
  solicita una publicacion inmediata de `/global_sparse_cloud`, sin esperar a
  BoW, subnubes, RANSAC o fusion;
- acuerdo: la nube es una vista completa derivada de `RawMapDatabase`,
  `GlobalPoseStore` y `FusedLandmarkManager`, no un append parcial ni una copia
  visual mutable;
- acuerdo: un commit de fusion vuelve a publicar la nube para retirar miembros
  raw duplicados y mostrar el representante vigente; un commit de optimizacion
  actualiza nube y KFs desde `GlobalPoseStore`;
- acuerdo: `3U` publica `/global_keyframes` como `MarkerArray`, con frustums
  world identificados por `(drone_id, map_epoch, local_kf_id)` y color
  determinista por submapa;
- concurrencia: worker fijo, revisiones y coalescing; no thread por delta ni
  salida stale posterior a una revision nueva;
- limite: submapas sin anchor no se publican en `world`; aparecen al anclarse;
- reparto documental: base en `3F`, snapshots en `3G`, apply en `3K/3L`,
  fusion/orden en `3P`, RViz2 en `3U` y limites de rendimiento en `3W`;
- estado: acuerdo documentado, sin cambios de codigo, build ni simulacion. La
  autorizacion funcional permanece pendiente.

## 2026-08-04 22:20 - `prueba_65` - RViz2 no arranca por entorno Snap

- objetivo intentado: validar la publicacion reactiva, `/global_keyframes` y el
  layout RViz2 con la ruta larga y `rawdb_record_enabled:=false`.
- resultado: wrapper codigo `1`; el escenario no llega a iniciarse. RViz2 carga
  `libpthread.so.0` desde `/snap/core20` y falla por
  `__libc_pthread_init@GLIBC_PRIVATE`.
- diagnostico: el detector de startup interpreta la caida de RViz2 como muerte
  de Gazebo porque la linea contiene la ruta del workspace. Los tres exit `255`
  de Gazebo son posteriores al cleanup provocado por ese falso positivo.
- conclusion: `NO EVALUA` la funcionalidad visual ni 3P. Se conserva como
  intento independiente; se filtra `/snap/` de `LD_LIBRARY_PATH` para RViz2.

## 2026-08-04 22:23 - `prueba_66` - saneado parcial insuficiente

- objetivo intentado: repetir la misma ruta tras filtrar
  `LD_LIBRARY_PATH` solo para RViz2.
- resultado: wrapper codigo `1` en los tres intentos de startup; el escenario
  tampoco se inicia y la prueba no evalua el backend.
- diagnostico: otras variables graficas/Snap siguen introduciendo bibliotecas
  `core20`. La correccion pasa a construir un entorno completo sin variables
  `SNAP`/`VSCODE_` ni segmentos `/snap/`, preservando ROS, `DISPLAY` y XDG.
- conclusion: `NO EVALUA`; fallo operativo conservado, no regresion de 3P.

## 2026-08-04 22:42 - `prueba_67` - ruta completa y starvation visual

- objetivo: ejecutar la ruta larga con RViz2 ya estable y validar nube/KFs
  reactivos, fusiones y optimizaciones.
- escenario: `14/14` pasos, `18/18` goals, runner/wrapper codigo `0`, primer
  intento Gazebo y espera final de `45 s`; duracion aproximada `779.072 s`.
- optimizacion: tres tareas aceptadas. `dron_2/fid1` corrige
  `23.555548 m / 2.188053 rad`, `dron_1/fid1` corrige
  `1.150523 m / 0.032418 rad` y la vuelta final de `dron_2/fid2` corrige
  `0.372784 m / 0.007032 rad`; las tres llegan a error cero sin mover hard
  fiducials.
- 3P: `451` enqueued, `450` calculados/committed, carga final `0`, pico `98`.
  El worker promedia `374.151 ms` de compute, pero el lock alcanza
  `35.367 s`. Se obtienen `44 FUSION_CANDIDATE`, `107`
  `LOOP_OPTIMIZATION_CANDIDATE` y `299` rechazos.
- publicacion: `738` solicitudes, solo `90` commits y `191` builds descartados
  porque una solicitud nueva invalidaba el resultado en curso. El hueco medio
  es `8.909 s` y el maximo `71.210 s`; la revision visible queda en `680`
  mientras la solicitada llega a `738`. La optimizacion final de fid2 se acepta
  despues de la ultima publicacion y no llega a RViz2.
- salida visible: revision `680` con `39706` puntos y `509` KFs; RawDB termina
  con cuatro submapas, `531` KFs y `43000` MPs.
- errores: ninguno grave. RViz2 permanece vivo y termina limpiamente; el exit
  de Gazebo ocurre durante cleanup posterior a `SIM-DONE`.
- conclusion: `PARCIAL`. Funcionalidad, ruta y optimizaciones pasan, pero el
  descarte estricto produce starvation y oculta el estado final.

## 2026-08-04 22:44 - correccion monotona y build

- cambio: el unico worker publica toda captura cuya revision sea posterior a
  la ya publicada. Las solicitudes recibidas durante el build quedan para la
  siguiente captura coalescida; no invalidan trabajo util. Shutdown drena hasta
  alcanzar la ultima revision solicitada.
- limite: una revision nunca puede sobrescribir otra igual o posterior; no se
  crea un thread por delta ni se cambia BoW, RANSAC, fusion u optimizacion.
- build: `orbslam3_multi orbslam3_server simulacion_dron`, codigo `0`.
- tests: covisibility, loop-pair cache, fused landmarks y tail anchor, todos
  codigo `0`.

## 2026-08-04 22:58 - `prueba_68` - publicacion reactiva monotona validada

- escenario: misma ruta y parametros que `prueba_67`; `14/14` pasos,
  `18/18` goals, runner/wrapper codigo `0`, primer intento y espera final
  completa. Duracion aproximada `675.095 s`.
- publicacion: `731` solicitudes, `292` commits, `605` solicitudes coalescidas,
  cero descartes y convergencia final `731/731`. El hueco medio/maximo baja a
  `2.513/28.392 s`; la salida final publica `41287` puntos y `542` KFs.
- tiempos visuales: captura bajo mutex media/maxima `871.604/17929.371 ms`,
  build de nube `653.941/2135.096 ms`, markers `2.708/8.708 ms` y publish
  `1.171/4.187 ms`. La starvation queda corregida, pero persisten picos de
  contencion capaces de retrasar una actualizacion.
- optimizacion: tres tareas creadas, aplicadas y aceptadas. Task 1
  `dron_2/fid1` corrige `29.107584 m / 2.042908 rad`, task 2
  `dron_1/fid1` corrige `2.161569 m / 0.220525 rad` y task 3
  `dron_2/fid2` corrige `2.578971 m / 0.447986 rad`; todas terminan en cero,
  conservan hard fiducials y publican el estado aceptado. La task 3 se hace
  visible unos `2.144 s` despues de su accept.
- 3P: `415` trabajos terminados; decisiones `63` fusion, `69` error alto y
  `283` rechazo. Compute medio/maximo `278.062/929.536 ms`; lock
  `583.689/23708.727 ms`. Se almacenan `99` negativos definitivos y no hay hits
  en esta ruta. Ingesta: `119` bloques, calculo externo medio/maximo
  `2063.983/28220.482 ms`, commit `5.602/236.935 ms` y `383` no-op metadata.
- backpressure: siete activaciones y seis liberaciones observadas; los seis
  ciclos cerrados suman `375.431 s`, maximo `125.546 s`. La ultima muestra
  periodica marca carga `10`, pero despues terminan esos diez jobs y se drenan
  commits de fusion hasta la revision visual final; no existe un
  `WORK-STATS` final tras ese drenaje.
- estado final: RawDB `6` submapas, `571` KFs y `48568` MPs. Tres submapas
  estan anclados; se omiten `29` KFs sin pose world. Hay `1598` tracks y
  `5230` miembros raw omitidos: `44919 - 5230 + 1598 = 41287` puntos.
- tracking: `dron_1` abre varios epochs y registra cuatro muestras
  `tracking_state=3`; tres de seis submapas quedan sin anchor. `dron_2`
  conserva epoch 0 y completa la optimizacion final de fid2.
- errores: sin crash, OOM, fatal ni procesos muertos. Solo aparece un aviso
  unico de cola `/gazebo/default/pose/local/info` durante el cierre; RViz2 y
  `global_map_server` terminan limpiamente.
- conclusion: `CONSEGUIDA` para la publicacion reactiva monotona, KFs por
  submapa, layout y visibilidad de optimizaciones. La conclusion agregada de
  3P permanece `PARCIAL` por picos de mutex/latencia visual, resets y submapas
  sin anchor; falta la valoracion visual humana de esta ejecucion.
- artefacto: `prueba_68.log` conservado sin lectura directa; todo el diagnostico
  se obtuvo mediante reducciones tematicas.

## 2026-08-05 - `prueba_69` - revision visual humana y diagnostico

- objetivo: repetir exactamente la ruta de `prueba_68`, sin build ni cambios
  funcionales, para inspeccionar `/global_sparse_cloud` y
  `/global_keyframes` en RViz2.
- resultado operativo: `14/14` pasos, `18/18` goals, runner/wrapper codigo `0`,
  primer intento Gazebo y espera final de `45 s` completa.
- publicacion: `933` solicitudes, `827` coalescidas, `276` commits, cero stale
  y cierre `933/933`. La latencia request-commit es
  `2.150/9.074/32.493 s` de media/p95/maximo. Publicar cuesta milisegundos,
  pero la captura bajo `live_state_mutex_` promedia `1108.821 ms` y alcanza
  `30051.676 ms`; el build de nube promedia `689.712 ms`.
- causa de los bloques visuales: el callback de full snapshot conserva el
  mutex durante insercion y reconciliacion amplias. Dos casos miden
  aproximadamente `19.6 s` y `29.9 s` entre
  `F1G-FULL-SNAPSHOT-ARRIVAL` y `F1G-RAWDB-INSERT-FULL`; las capturas visuales
  asociadas esperan `19.345 s` y `30.052 s`. El worker reconstruye ademas la
  vista completa y coalesce estados intermedios, por lo que no existe una
  imagen RViz2 por delta.
- fid1: ambos goals llegan a las `1785924155.513` y el siguiente move queda
  inicialmente bloqueado por carga 3P. El gate abre y envia el barrido lateral
  a las `1785924281.750`; la tarea fiducial se crea despues, a las
  `1785924287.052`, cuando llega el KF que permite reconocer el revisit. El
  goal ya activo termina por contrato y no se cancela.
- apply/visualizacion: task 1 aplica a las `1785924323.196`, se acepta a las
  `1785924324.160`, el contador de optimizacion baja y el mismo backpressure se
  libera un milisegundo despues. El siguiente goal sale a las
  `1785924324.205`, mientras el primer commit visual posterior llega a las
  `1785924325.413`. El flag protege el trabajo de optimizacion, no la revision
  visual que debe representarlo.
- puntos transitorios: el check post-apply registra
  `fallback_submap_after=33`. Esos MapPoints no se proyectan desde un KF
  utilizable, sino con `world_T_local * p_local_raw`; por eso pueden quedar en
  la colocacion rigida anterior sin un frustum asociado. Deltas o snapshots
  posteriores pueden aportar KFs/asociaciones y hacer que una reconstruccion
  posterior los proyecte desde `GlobalPoseStore`. El log no conserva las
  coordenadas/IDs vistos por el usuario, por lo que demuestra el mecanismo y
  su presencia, no la identidad visual exacta de cada punto.
- conclusion: `PARCIAL`. Se conserva convergencia monotona y estado final, pero
  la validacion humana invalida el cierre automatico de la extension visual:
  no hay frescura acotada por delta, el movimiento puede adelantarse al commit
  visual del apply y existe una ruta de fallback capaz de mostrar puntos sin
  KF coherente.
- siguiente paso recomendado: preparar, antes de tocar codigo, captura raw/pose
  breve sin insercion completa bajo el mutex, una barrera de publicacion para
  mantener el mismo backpressure hasta el commit post-apply y una politica que
  no publique como definitivos MPs sin KF utilizable tras una correccion.
- artefacto: `prueba_69.log` se conserva sin lectura directa; el diagnostico se
  obtuvo exclusivamente mediante reducciones tematicas.

## 2026-08-05 - Fusion integrada en la tarea secundaria unica

- objetivo intentado: integrar la rama de fusion en la misma `LoopTask` que
  comienza en BoW y termina al escribir las bases derivadas;
- archivos modificados: coordinacion en `global_map_server.cpp` y contratos de
  `FusedLandmarkManager`, covisibilidad, scores y publicacion;
- comportamiento: una decision `FUSION` hace commit de tracks/covisibilidad y
  solicita reconstruccion; una optimizacion aceptada puede continuar con la
  fusion compatible dentro del mismo `task_id`;
- build: `orbslam3_multi orbslam3_server simulacion_dron`, codigo `0`;
- test focalizado: `test_fused_landmark_manager`, `PASS`;
- pruebas live: `prueba_75` y `prueba_76` terminan el escenario y mantienen
  publicaciones principales mientras existe backlog secundario;
- evidencia positiva: aparecen commits de fusion seguidos por solicitudes de
  publicacion, sin escribir `RawMapDatabase` ni esperar RViz2;
- evidencia negativa o ausente: no se realizo una confirmacion visual humana
  nueva de la calidad de los tracks fusionados en la prueba 76;
- conclusion: `PARCIAL`. La integracion runtime esta completada, pero la
  calidad global de asociaciones/duplicados y el cierre visual siguen abiertos;
- siguiente paso recomendado: cubrir en `3V` casos positivos y negativos de
  fusion con conteo de tracks y revision visual del resultado.

## 2026-08-16 21:44 - Reimplementacion 3P y `prueba_159` - crash transitivo tardio

- objetivo intentado: reimplementar el contrato acordado de fusion, score por
  evidencia/visibilidad, covisibilidad server, commit con rollback, builder
  incremental y visual 3P; validarlo con la trayectoria tipica de dos
  fiduciales.
- archivos modificados: `raw_map_types.hpp`, `raw_map_database.*`,
  `loop_pipeline.*`, `landmark_score_manager.*`, `covisibility_database.*`,
  `global_map_builder.*`, `sparse_global_backend.*`, nuevos
  `fused_landmark_types.hpp`/`fused_landmark_manager.*`, servidor y grafo web.
- paquetes compilados: `orbslam3_multi orbslam3_server simulacion_dron`.
- resultado de build: codigo 0, tres paquetes finalizados; aviso Drake no
  bloqueante.
- pruebas automaticas previas: dominio 9/9, servidor funcional 4/4 y contrato
  web 1/1. El `colcon test` indiscriminado se descarto porque linters historicos
  recorren `legacy2`; no representa un fallo runtime.
- prueba Gazebo: `prueba_159`, YAML
  `prueba_tipica_rodeo_edificio_dos_fiduciales.yaml`, runner y herramienta
  codigo 0, 411 s, guardia de recursos inactiva.
- patrones de reduccion: `F3P-FUSION`, `F3O-LOOP-DONE`, lifecycle secundario,
  builder, backpressure, `out_of_range`, `map::at`, abortos y procesos.
- evidencia positiva anterior al fallo: 24 intentos, 21 commits, tres stale y
  cero rollback; 370 tracks creados, 382 actualizados, 16 retirados y 868
  miembros raw ocultos. El principal siguio publicando mientras fusionaba.
- evidencia negativa: al iniciar la `LoopTask` 3043 el servidor aborto con
  `std::out_of_range: map::at`; el runner continuo y por eso su `success=true`
  no valida esta ejecucion. El patron causal fue un track ya presente en
  `touched_tracks` que una fusion posterior del mismo patch retiro.
- conclusion: `NO CONSEGUIDA`. Se conserva como intento fallido y demuestra
  que el resultado del runner debe contrastarse con la vida del servidor.
- siguiente paso recomendado: retirar el track absorbido de `touched_tracks`,
  comprobar referencias locales, añadir regresion touch+merge+retire y una
  barrera de excepciones por tarea antes de repetir la prueba completa.

## 2026-08-17 00:44 - `prueba_160` - correccion y validacion automatica 3P

- objetivo intentado: corregir el crash 159 sin cambiar el contrato funcional y
  repetir exactamente la trayectoria tipica hasta el cierre.
- correccion: `PrepareFusion()` elimina de `touched_tracks` todo track
  absorbido y valida que las referencias locales sigan vivas; nueva regresion
  crea dos tracks, refuerza el segundo y lo retira dentro del mismo patch. El
  worker secundario convierte excepciones futuras en fallo duro observable y
  completa lifecycle/cola en vez de abortar el nodo.
- paquetes compilados: `orbslam3_multi orbslam3_server simulacion_dron`.
- resultado de build: codigo 0, 3/3 paquetes, 2026-08-16
  21:46:30-21:46:57.
- pruebas automaticas: `orbslam3_multi` 9/9, servidor 4/4 y
  `pipeline_flow_contract` 1/1; incluye la regresion exacta del fallo 159.
- prueba Gazebo: `prueba_160`, mismo YAML, startup 15 s, timeout 900 s,
  post-wait 20 s y monitor de recursos; scenario, herramienta y simulacion
  codigo 0, `success=true`, 411 s.
- patrones de reduccion: `F3P-FUSION`, publicaciones builder, shutdown/errores,
  backpressure, commits fiduciales y proceso del servidor. El log completo se
  conservo sin lectura directa.
- fusion: 62 intentos, 56 commits, cinco stale por dependencias cambiadas y un
  rollback correcto por cambio de revision de score. En los commits hubo 3083
  pares, tracks `(creados=795, actualizados=1900, retirados=76)`, 2050 miembros
  raw ocultos, score `+5932/-76` con 3979 raw dirty y covisibilidad
  `+92/~27`.
- presupuesto/rendimiento: 125 regiones agotaron el presupuesto negativo
  sparse sin perder fusion/positivos. Prepare medio/maximo sobre intentos
  `55.169/107.757 ms`; commit medio/maximo sobre intentos
  `8.581/39.767 ms`. En los 56 commits aceptados la media/maximo fue
  `7.396/19.347 ms` y 46 superaron el objetivo inicial de 5 ms.
- builder: 383 publicaciones, 228 con tracks recalculados, 20600 recalculos
  acumulados y maximo 410. La ultima publicacion recalcula 87 tracks, usa
  `fusion_revision=56` y publica 23727 puntos/461 KFs.
- cola/vida: 486 principales; 1116 secundarias, 136 stale, 980 committed,
  `pending=0`, `hard_failed=0`, `max_active=1`. No aparecen
  `SECONDARY-EXCEPTION`, `map::at`, aborto ni muerte del servidor; el proceso
  termina limpiamente.
- recursos: 325 muestras, MemAvailable minimo 6568.8 MiB, PSS maximo servidor
  229.7 MiB, PSS maximo grupo 1131.6 MiB, PSI full 0 y guardia inactiva.
- evidencia visual: bridge listo con 23 nodos/38 aristas y contrato web
  superado. El usuario confirma posteriormente que RViz2 y el grafo web se
  vieron muy bien, sin anomalias visuales que cambien los conteos tecnicos.
- error no funcional: Gazebo devuelve 255 durante cleanup despues de
  `SIM-DONE`; servidor, RViz2, bridge y nodos ROS terminan limpiamente.
- conclusion: `PARCIAL`. Las invariantes y criterios funcionales automaticos
  quedan conseguidos, pero el commit supera con frecuencia el objetivo de
  5 ms y el presupuesto negativo se agota a menudo. La validacion visual queda
  conseguida con la observacion posterior del usuario.
- siguiente paso recomendado: decidir si el objetivo diagnostico del commit se
  eleva a 20 ms y si el presupuesto sparse negativo se ajusta ahora o mas
  adelante.

## 2026-08-17 01:39 - `prueba_161` - retry fresco y visibilidad completa

- objetivo intentado: eliminar los presupuestos temporales 3P, reintentar toda
  fusion stale/rollback con una `LoopTask` BAJA fresca y comprobar que la
  optimizacion fiducial y el resto del runtime no regresan.
- cambios funcionales: `FusedLandmarkManager` recorre toda contradiccion
  elegible sin reloj; `SecondaryTaskQueue::PushLoop(task, true)` atraviesa solo
  el ledger completado; `SecondaryWorkerLoop()` completa primero la tarea y
  despues crea el retry con revisiones nuevas. El grafo añade
  `SecondaryWorker --retry / LOW--> SecondaryTaskQueue`.
- archivos principales: tipos/manager/backend de fusion en `orbslam3_multi`,
  cola/servidor y tests en `orbslam3_server`, bridge/grafo/test web en
  `simulacion_dron`, contratos y docs 3P.
- build: `orbslam3_multi orbslam3_server simulacion_dron`, codigo 0, 3/3.
- tests: `orbslam3_multi` 9/9, `test_secondary_queue` 1/1,
  `pipeline_flow_contract` 1/1 y pytest fuente 9/9. Las suites globales de
  server/simulacion conservan fallos de lint preexistentes en `legacy2` y
  codigo ajeno; no se modificaron.
- prueba: YAML `prueba_tipica_rodeo_edificio_dos_fiduciales.yaml`, startup
  15 s, timeout 900 s, post-wait 20 s y recursos. Scenario/herramienta codigo
  0, `success=true`, monitor 413 s.
- reduccion: `F3P-FUSION/RETRY`, lifecycle de retries, `F3J/F3K/F3L`,
  backpressure, builder final, shutdown y errores; log completo no leido.
- fusion/retry: 27 intentos, ocho commits y 19 stale, incluidos cuatro
  rollback. Hay exactamente 19 `F3P-FUSION-RETRY`, todos `created=1` y
  `enqueued=1`; los 19 task IDs aparecen en `START` y `DONE`. No hay skip ni
  duplicado pendiente.
- visibilidad/score aceptado: 56/56 regiones completas entre intentos. Los
  commits suman 348 pares, tracks `(258,84,2)`, 575 miembros ocultos, score
  `+694/-396`, covisibilidad `+10/~0`, 11/11 regiones y 203187 proyecciones.
- cardinalidad revisada: RawMapDatabase termina con 62718 MapPoints. De ellos,
  se estiman 25249 publicables antes de fusion; 575 miembros raw quedan
  representados por 256 tracks vigentes, reduccion neta 319 y salida final
  24930 puntos. Los otros raw pertenecen sobre todo a submapas diferidos o
  puntos bad/no publicables, no son descartes causados por fusion.
- variacion de score: los commits aplican 694 evidencias `+0.04` y 396
  negativas `-0.01/-0.03`, con 1035 actualizaciones raw contadas por commit.
  La telemetria 161 no separa ambos tipos negativos ni registra delta final
  por ID tras clamp, de modo que el incremento teorico medio queda acotado
  entre `+0.0153` y `+0.0230` por actualizacion; el maximo por evidencia es
  `+0.04` y la penalizacion maxima `-0.03`. No puede afirmarse el maximo
  acumulado por landmark sin añadir telemetria especifica.
- rendimiento: prepare aceptado media/maximo `633.852/1087.130 ms`, visibilidad
  `611.492/1037.230 ms` y commit `5.362/10.353 ms`. Ya no existe objetivo de
  5 ms ni warning de 20 ms; el coste se conserva como telemetria.
- optimizacion: cuatro propuestas convergen, validan `accept_full` y hacen
  commit atomico con error final cero; mueven 5, 120, 116 y 2 KFs.
- cola/estado final: 484 principales; 1162 secundarias, 162 stale, 1000
  committed, `pending=0`, `hard_failed=0`, `max_active=1`. Publicacion final:
  24930 puntos, 450 KFs y `fusion_revision=8`.
- recursos: MemAvailable minimo 6644.4 MiB, RSS/PSS maximo del servidor
  246.9/225.6 MiB, PSS grupo 1116.0 MiB, PSI full 0 y guard inactivo.
- evidencia negativa: el prepare es mucho mas caro al evaluar todos los
  negativos, pero backpressure y worker drenan antes del cierre. El unico
  `ERROR` es Gazebo 255 despues de `SIM-DONE` durante cleanup.
- interpretacion de rendimiento: correcto para esta carga de dos drones; las
  cuatro activaciones de backpressure proceden solo de optimizacion activa, no
  de superar el watermark secundario. No extrapola todavia a ocho drones.
- evidencia visual revisada: el usuario da por concluida la subfase y acepta
  su funcionalidad. Solicita como pulido final reorganizar el layout desktop
  del grafo web para separar mejor vertices, aristas y labels; no identifica
  un fallo de datos, lifecycle ni topologia.
- conclusion revisada: `CONSEGUIDA` funcional y visualmente. El rediseño de
  layout queda como ajuste de legibilidad posterior al cierre, sin cambiar la
  semantica 23/39.
- siguiente paso: acordar y aplicar el layout desktop inspirado en la captura;
  despues preparar 3Q.

## 2026-08-17 11:15 - Pulido de layout web posterior al cierre

- objetivo: recolocar exclusivamente los vertices desktop del grafo 3P para
  mejorar lectura de nodos, aristas y labels segun la referencia del usuario.
- cambio: `graph_definition.js` organiza el espacio logico en tres bandas y
  separa las curvas opuestas `dequeue/start` y `retry / LOW`; tras la primera
  revision del usuario se ampliaron horizontal y verticalmente todas las
  columnas desktop y se desvio `score evidence` alrededor del anchor. Conserva
  23 nodos, 39 aristas, categorias, eventos y posiciones movil.
- validacion: contrato web fuente 9/9 antes y despues del ajuste de espaciado;
  capturas headless 1440x900 y 390x844; bridge aislado listo con topologia
  23/39 y navegador abierto en puerto 8765.
- exclusiones cumplidas: sin build necesario por install symlink, sin Gazebo,
  RViz2 ni simulacion.
- conclusion: cambio aplicado; queda abierto para revision visual directa del
  usuario y no altera la conclusion `CONSEGUIDA` de 3P.
