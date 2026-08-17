# 00 - Contexto de compactacion

## Estado Vivo

```text
Estado: 3B-3P CONSEGUIDAS; IMPLEMENTACION 3Q EN CURSO
Objetivo vigente: implementar, compilar y validar integralmente la subfase 3Q segun el acuerdo cerrado
Preparacion: CERRADA
Acuerdo cerrado: si
Autorizacion funcional: CONCEDIDA
Prueba acordada: bateria Gazebo natural multi-escenario sin offset artificial; informar por prueba si hubo optimizacion loop y sus resultados. Revision RViz2/web del usuario en cuatro casos representativos; tests deterministas y logs para todos
Dudas abiertas: ninguna
Alcance acordado 3Q: convertir `OptimizationEvidence` en una rama relativa dentro de la misma `LoopTask` BAJA y unificar el optimizador covisible para loop y fiducial; construir ventanas conjuntas mediante fiduciales hard, tramos temporales, fusiones/loops previos, dependencias blandas y covisibilidad fuerte; mover la componente segun constraints; reutilizar lifecycle/builder/solver/validator/store/fusion 3P; commit breve y dirty sets. Una dependencia soft se sigue e incluye cuando conecta con un submapa delimitado por dos fiduciales hard
Exclusiones acordadas 3Q: sin nueva cola/worker/solver duplicado, sin GT, sin modificar raw, sin publicar desde secundario, sin preemption, sin offset artificial de pruebas y sin excluir loops por ser inter/intra dron o submapa
Criterios acordados 3Q: constraints loop relativas y fiduciales absolutas sobre el grafo comun; hard fiducials inmoviles; fusiones previas como relaciones soft medibles; covisibilidad confirmada en optimizaciones loop y fiducial; densidad base de controles 30 por ciento ampliable por constraints covisibles; dos apoyos independientes; accept completo provisional; fusion posterior opcional sin invalidar una buena pose; stale/rollback reencola BAJA fresca; cero escrituras en reject/stale; continuidad de KFs tardios; accepts positivos reproducibles automaticos/live/visuales. Al entrar en optimizacion dentro de una LoopTask BAJA se activa stop_drones y se mantiene hasta terminar validacion, commit y fusion directa de la rama
Riesgos aceptados 3Q: falso cierre en zonas repetitivas mitigado por dos queries/ambiguedad; degradacion medible de fusion previa soft; sobrerigidez por covisibilidad mitigada por pesos configurables; invalidacion concurrente resuelta con stale/retry; commit multi-base con rollback; coste de ventanas/fusion y posibilidad de que una prueba Gazebo natural no produzca error alto, que se informara sin falsear el resultado
Conclusion agregada 3H-3L: CONSEGUIDA; tests, replays 149/150, live 151 y confirmacion visual del usuario validan continuidad RViz2, color por submapa y lifecycle web
Conclusion agregada 3M: CONSEGUIDA; patch MEDIA canonico/versionado y encadenado a loops validados
Conclusion agregada 3N: CONSEGUIDA; indice BoW/regiones/ledger causal y cola drenada validados
Conclusion agregada 3O: CONSEGUIDA; propagacion rigida, reanchor por primer fiducial directo, carga secundaria y visualizacion RViz2/grafo web validadas; 3P/3Q comprobaran integralmente sus evidencias de fusion/optimizacion
Conclusion agregada 3P: CONSEGUIDA por cierre explicito del usuario; fallo 159 conservado, prueba 160 valida correccion/visual y prueba 161 valida retry fresco, visibilidad completa, optimizaciones, drenaje y recursos; pulido layout no reabre funcionalidad
Conclusion agregada 3B: CONSEGUIDA; criterios automaticos y visuales confirmados
Conclusion agregada 3C: CONSEGUIDA; raw, FIFO/worker principal, replay, backpressure, web y ausencia de publishers RViz2 validados
Conclusion agregada 3D: CONSEGUIDA; usuario confirma comportamiento web esperado tras prueba90 y da por bueno el resultado para continuar
Conclusion agregada 3E: CONSEGUIDA; usuario acepta la evidencia tecnica de 2 anchors/61 poses/2 hard y no solicita cambios visuales; first anchor se observara de nuevo en la siguiente prueba
Conclusion agregada 3F: CONSEGUIDA; color por epoch corregido, probado y confirmado visualmente por el usuario en live 151
Conclusion agregada 3G: CONSEGUIDA; semantica snapshot, rendimiento, carga real con tres drones y restauracion visual validados
Trabajo activo: implementacion funcional 3Q autorizada; codigo base y tests deterministas completados, bateria Gazebo y auditoria funcional en curso
Ultimo hito: build covisible 2026-08-17 13:40:43-13:41:22 exit 0; tests `test_fiducial_optimization` 8/8, `test_sparse_global_backend` 9/9, `test_loop_pipeline` 8/8, `test_secondary_queue` 4/4 y contrato web 9/9 passed
Politica operativa acordada: compilar solo con build_selected_packages.sh; ejecutar Gazebo/RViz2/web mediante launches llamados por run_simulation.sh; crear helper de apertura de una pestaña del grafo invocado por el launch; no usar comandos ad hoc para escribir fuera de src
Archivos criticos localizados: `orbslam3_multi/include/orbslam3_multi/{loop_pipeline,pose_graph_problem,pose_graph_builder,optimization_manager,optimization_validator,global_pose_store,sparse_global_backend}.hpp`; sus CPP activos; `orbslam3_server/src/global_map_server.cpp`; `simulacion_dron/web/pipeline_flow/graph_definition.js`; tests C++ de optimizacion/loop/backend y contrato web
Plan vigente: auditar APIs exactas; implementar problema/grafo/solver/validador covisible comun; integrar OptimizationEvidence, lifecycle stop_drones, commit/fusion/retry y visual; añadir tests; compilar; ejecutar y reducir la bateria Gazebo; documentar cada resultado y cerrar con conclusion
Build siguiente: CONSEGUIDO, 2026-08-17 13:18:38-13:18:53; exit 0, 3/3 paquetes (`orbslam3_multi`, `orbslam3_server`, `simulacion_dron`); log `codex/archivos_auxiliares/colcon_build.log`; unico aviso visible Drake no bloqueante
Prueba vigente: prueba 164 completada `success=true`, scenario exit 0, herramienta exit 0 y 60 s de drenaje; recursos estables (`guard_triggered=false`, server RSS max 235.5 MiB, group RSS max 1602.8 MiB, PSI memoria 0). Log completo preservado y no leido: `codex/archivos_auxiliares/logs/prueba_164.log`
Siguiente accion exacta: reducir prueba 164 por lifecycle 3Q, same-submap, fiducial, colas/backpressure, commits/fusion y cierre; analizar solo artefactos reducidos
Ultima actualizacion: 2026-08-17
```

## Acuerdo 3B Ejecutado

- `orbslam3_server/legacy2/` conservara el estado actual de `src/`, `include/`
  y `launch/`, junto con copias renombradas de `CMakeLists.txt` y `package.xml`
  para que Colcon no lo descubra como otro paquete.
- `orbslam3_multi/legacy2/` conservara el estado actual de `src/` e `include/`,
  incluidos tests y ejecutables auxiliares, y copias renombradas de sus archivos
  de paquete. La carpeta `legacy/` existente no se movera ni duplicara.
- Cada snapshot tendra un README/manifiesto que explique origen, fecha,
  contenido y prohibicion de compilarlo desde los targets activos.
- El `orbslam3_multi` activo quedara como paquete `ament_cmake` vacio y
  compilable, sin biblioteca placeholder ni clases de las subfases 3C-3P.
- El `orbslam3_server` activo quedara como un nodo ROS 2 minimo: inicializa,
  permanece vivo y termina limpiamente, sin subscriptions, publishers, timers,
  mapa, fiduciales, loops, backpressure ni ground truth.
- El launch publico del servidor se reducira al nodo vacio. El launch de
  simulacion solo se adaptara donde sea necesario para no pasar argumentos
  obsoletos al servidor.
- En `codex/contexto/paquetes/orbslam3_server/legacy2/` y
  `codex/contexto/paquetes/orbslam3_multi/legacy2/` se archivara una copia de
  los MD actuales. En la raiz quedaran exclusivamente resúmenes y documentos
  reescritos para los archivos activos: resumen del paquete vacio en
  `orbslam3_multi`, y resumen, nodo vacio y launch minimo en el servidor.
- No se modificaran `ORB_SLAM3`, `orbslam3_ros2`, `orbslam3_msgs`, algoritmos ni
  contratos de subfases posteriores; tampoco se mantendra un fallback runtime
  al servidor anterior.
- Validacion ejecutada: build de los tres paquetes afectados y prueba 77 con
  un YAML nuevo de espera corta, sin RViz2/web, comprobando arranque limpio del
  servidor, ausencia de salidas sparse globales y cierre sin crash.

## Implementacion 3B Completada

- Los arboles `src/include/launch` anteriores y sus metadatos quedaron
  congelados en ambos `legacy2`; los cambios no confirmados que habia en disco
  forman parte de la instantanea.
- Los MD anteriores se archivaron en los dos `legacy2` documentales y la raiz
  describe unicamente el estado activo.
- `orbslam3_multi` es ahora un paquete `ament_cmake` sin targets ni codigo.
- `orbslam3_server` compila solo un nodo `rclcpp` vacio y su launch declara solo
  `use_sim_time`.
- `multi_dron.launch.py` ya no declara ni reenvia parametros del servidor
  congelado; conserva controles independientes para RViz2 y visualizador web.
- Se creo `tray_prueba_77.yaml` con un unico `wait` de 5 s y sin movimientos.
- Comprobaciones estaticas: no existen metadatos Colcon activos dentro de
  `legacy2`, no quedan referencias runtime antiguas y ambos launches parsean.
- Build 2026-08-10 13:19:50-13:19:57: comando seleccionado sobre
  `orbslam3_multi orbslam3_server simulacion_dron`, exit code 0, tres paquetes
  finalizados. Log preservado:
  `codex/archivos_auxiliares/logs/colcon_build_3B_incremental.log`. Unica
  advertencia visible: `CMAKE_PREFIX_PATH` contiene `/opt/drake/share/drake`
  inexistente; no afecta al resultado.
- Prueba 77, intento 1, 2026-08-10 13:20:28: YAML
  `codex/archivos_auxiliares/trayectorias/tray_prueba_77.yaml`, launch acordado,
  startup 15 s, timeout 60 s y post-wait 3 s. `scenario_runner_node` devolvio 1,
  la herramienta devolvio 1 y `success=false`; el launch fue cerrado por la
  herramienta. Log completo no leido:
  `codex/archivos_auxiliares/logs/prueba_77_intento_1.log`.
- Diagnostico reducido del intento 1: el servidor emitio
  `[F3B-EMPTY-SERVER-INIT]` y termino limpiamente. El fallo fue exclusivamente
  `Could not load scenario YAML ... bad file`: la ruta relativa se resolvio
  desde el workspace padre. La correccion es mecanica y conserva el acuerdo:
  repetir con ruta absoluta. El error 255 de Gazebo aparece durante el cierre
  forzado posterior al fallo del runner, no como causa inicial.
- Prueba 77, intento 2, 2026-08-10 13:21:57: mismo launch y tiempos, YAML
  absoluto. `scenario_runner_node` devolvio 0, la herramienta devolvio 0 y
  `success=true`. Log completo no leido:
  `codex/archivos_auxiliares/logs/prueba_77.log`.
- Analisis reducido del intento 2: marcador de inicio unico, scenario de wait
  completado, servidor terminado limpio y ausencia de actividad sparse global
  del servidor. Los deltas visibles son de wrappers locales. El exit 255 de
  Gazebo ocurre tras `SIM-DONE`, durante cleanup.
- Auditoria post-build: el install incremental aun contiene symlinks obsoletos
  a cabeceras/tests/corrector/launch antiguo. No se ejecutaron, pero deben
  limpiarse como artefactos generados para que el install refleje el runtime
  activo vacio.
- Rebuild limpio 2026-08-10 13:31:52-13:31:59: se eliminaron solo build/install
  generados de ambos paquetes y se recompilaron junto con `simulacion_dron`;
  exit code 0, tres paquetes finalizados. Log vigente:
  `codex/archivos_auxiliares/logs/colcon_build.log`; el build incremental
  anterior queda en `colcon_build_3B_incremental.log`.
- Auditoria final de install: solo se instala el ejecutable/launch activo del
  servidor; `orbslam3_multi` no instala codigo ni targets. Colcon lista una sola
  instancia de cada paquete y no descubre ningun `legacy2`.

## Ampliacion Visual 3B Ejecutada

- Snapshot completado en
  `simulacion_dron/legacy2/pipeline_flow_visualizer/`: bridge, web completa,
  launch y metadatos CMake/package anteriores.
- Documentacion anterior copiada a
  `codex/contexto/paquetes/simulacion_dron/legacy2/`.
- No se ha creado publisher, subscription ni arista wrapper-servidor.
- Bridge activo: primera conexion SSE empieza en el presente; reconexion usa
  `Last-Event-ID`; un cursor expirado produce `state_reset`; `/health` expone
  modo live y secuencia.
- Frontend activo: dos nodos (`wrappers`, `server`), cero aristas, drenaje por
  `requestAnimationFrame`, conteo de gaps y estado debug consultable.
- Test directo `test_pipeline_flow_contract.py`: `4 passed`.
- Se creo `tray_prueba_78.yaml`: espera de tracking, dos goals simultaneos a
  fiducial 2 y espera visual de 30 s.
- Se creo `CONTRATO_VISUAL_INCREMENTAL.md` y se sincronizaron los contratos
  principales 3C-3X, extensiones 3F/3K y subdocumentos de 3O/3P/3Q/3S.
- 3U pasa a auditoria/hardening final; no es creadora tardia del visualizador.
- Documentacion activa de `simulacion_dron` describe solo la base 3B y enlaza
  el snapshot legacy2.
- Build 2026-08-10 14:02:39-14:02:45: tres paquetes finalizados, exit code 0.
  Log completo no leido: `codex/archivos_auxiliares/logs/colcon_build.log`.
  Aviso no bloqueante: prefijo Drake inexistente.
- Test integrado CMake `pipeline_flow_contract`: 1/1 passed, 0 failed, 1.46 s.
- Validacion runtime aislada: el primer arranque revelo que
  `pipeline_flow_bridge.py` no conservaba permiso ejecutable; se corrigio de
  forma mecanica y el rebuild de `simulacion_dron` termino con exit code 0.
- `/health` en puerto temporal 8878 devolvio `status=ready`, `mode=live`,
  capacidad 512, secuencia 0 y `replay_on_first_connect=false`.
- Capturas headless de escritorio 1440x900 y movil 390x844 verificaron dos
  nodos, cero aristas, SSE conectado, cero eventos/gaps y ausencia de solapes.
- Prueba 78 preparada para ejecucion: launch
  `ros2 launch simulacion_dron multi_dron.launch.py`, YAML absoluto
  `tray_prueba_78.yaml`, startup 15 s, timeout global 360 s y post-wait 10 s;
  se mantienen activos los defaults de RViz2 y visualizador web.
- Prueba 78 ejecutada desde 14:12:44: `scenario_runner_node` devolvio 0, la
  herramienta marco `success=true` y termino con exit code 0. Durante la
  ejecucion integrada `/health` siguio en `ready/live`, secuencia 0 y sin
  replay inicial. Log completo preservado y no leido:
  `codex/archivos_auxiliares/logs/prueba_78.log`.
- Analisis reducido de prueba 78: bridge `mode=live topology=2_nodes_0_edges`,
  RViz2 iniciado y cerrado limpio, servidor vacio iniciado, ambos goals
  paralelos correctos en 22 s, espera visual de 30 s y escenario completo.
  No aparecen `global_sparse_map` ni `global_keyframes`; auditoria estatica del
  servidor activo confirma ausencia de publishers, subscriptions y timers.
  El exit 255 de Gazebo es posterior a `SIM-DONE` y pertenece al cleanup.
- Conclusion tecnica: criterios automaticos conseguidos. El usuario confirmó
  posteriormente grafo sin conexiones/actividad y RViz2 sin resultados
  globales; conclusión final 3B: CONSEGUIDA.
- Verificacion final tras cierre documental: CTest
  `pipeline_flow_contract` repetido sobre el build final, 1/1 passed en 2.03 s.
- Auditoria de procesos: no quedan procesos de Gazebo, RViz2, bridge, servidor
  ni launch asociados a la prueba 78.

## Acuerdo Funcional Vigente

### Flujo principal

- El servidor encola deltas/full snapshots; un unico PrimaryWorker procesa una
  entrada completa hasta publish ROS antes de iniciar la siguiente.
- El servidor orquesta y delega el trabajo en clases de `orbslam3_multi`; no
  implementa algoritmos de mapa, loop, fusion u optimizacion.
- `RawMapDatabase` conserva exclusivamente el estado ORB-SLAM3 crudo y devuelve
  un `ChangeSet`/resultado de insercion; no llama directamente a otras bases.
- `GlobalPoseStore` registra en principal los KFs de submapas anclados. Score y
  covisibilidad se actualizan mediante `DatabaseUpdateTask` secundaria HIGH.
- El primer fiducial de `(drone_id, map_epoch)` crea el anchor. Una revisit
  valida puede crear una tarea fiducial prioritaria.
- Los KFs nuevos de un submapa anclado reciben pose world inmediatamente desde
  el ultimo anchor/campo de correccion aceptado, sin esperar al worker
  secundario.
- `GlobalMapBuilder` conserva nube/mensaje de KFs e indices incrementales;
  acumula IDs dirty de raw, poses, scores y fusion y solo recalcula afectados.
- `GlobalMapBuilder` se ejecuta solo dentro de la siguiente tarea principal.
  Un commit secundario notifica dirty sets, pero no despierta, reconstruye ni
  publica por si mismo.
- El flujo principal nunca espera BoW, matching, RANSAC, fusion, grafo, solver,
  HTML ni al visualizador JavaScript.

### Flujo secundario

- Existe un unico worker persistente y una cola con prioridades MAX fiducial,
  HIGH `DatabaseUpdateTask` y NORMAL `LoopTask`.
- Una tarea activa nunca se interrumpe: se ejecuta de inicio a fin.
- Al terminar la activa se elige fiducial, despues update de bases y despues
  loop; FIFO se conserva dentro de cada prioridad.
- Cada KF nuevo/materialmente modificado elegible crea su propia `LoopTask`;
  nunca existe una tarea loop agregada por delta.
- Cada `LoopTask` abarca BoW, filtros baratos, matching/RANSAC cuando proceda,
  decision entre fusion u optimizacion, ejecucion elegida y commit final.
- Fusion y optimizacion por loop no son tareas separadas ni tienen prioridades
  diferentes: son desenlaces de la misma `LoopTask`.
- Toda tarea calcula sobre snapshots privados fuera de locks y termina tras un
  commit breve, validado y atomico en las bases derivadas correspondientes.
- Una tarea secundaria no publica, no despierta al principal y no espera
  `publication_ack`; solo compromete bases y notifica cambios dirty.
- `RawMapDatabase` nunca se modifica por fusion u optimizacion.

### Backpressure

- El flag es OR de high watermark principal/secundario y optimizacion
  fiducial/loop activa, con histeresis.
- El goal activo termina normalmente; mientras el flag siga activo no se envia
  el siguiente goal.
- No se solicitan snapshots periodicos nuevos; los ya en vuelo se conservan y
  al liberar se pide como maximo uno fresco por dron.

### Sincronizacion

- Las lecturas largas usan snapshots inmutables/versionados, no contenedores
  live protegidos durante todo el calculo.
- Los mutex solo protegen cambios breves de cola o commits de estado.
- Las revisiones capturadas se validan antes del commit; un resultado obsoleto
  se descarta o reprograma de forma acotada, sin retry inmediato infinito.
- La publicacion de KFs y nube usa una misma revision coherente y nunca observa
  un lote de poses parcialmente escrito.

### Visualizador JavaScript

- `3B` es propietaria de la infraestructura web JavaScript independiente de
  RViz2; cada subfase añade sus nodos, aristas y eventos reales, y `3U` audita
  y endurece el resultado completo.
- Los eventos se emiten mediante instrumentacion minima a una cola acotada y no
  bloqueante; perder telemetria es preferible a frenar el pipeline.
- Los eventos transportan metadatos, IDs, revisiones, cantidades, tiempos y
  resultado, nunca nubes, descriptores o payloads pesados completos.
- Principal y fiducial se conservan completos; loops y `DatabaseUpdateTask`
  pueden descartarse solo como flujos completos. Un loop descartado que decide
  optimizar se muestra obligatoriamente desde `OPTIMIZATION_DECIDED`.
- La UI ilumina nodos/aristas en vivo y muestra tooltips de responsabilidad y
  datos transferidos. Debe iniciarse con la simulacion, pero su fallo no afecta
  a ROS, RViz2 ni al mapa.
- Es tiempo real de observacion humana, no tiempo real duro.

### Exclusiones

- No usar GT salvo como origen del fiducial simulado, debug o metricas externas.
- No iniciar fases posteriores ni cambiar BoW/RANSAC/optimizacion fuera de la
  reorganizacion necesaria para cumplir este contrato.
- No ampliar el alcance funcional ni cambiar criterios algoritmicos sin cerrar
  un nuevo acuerdo.

## Nuevo orden de pipeline

- Fase 1: control del dron, marcada como realizada en el nuevo pipeline.
- Fase 2: separacion fisica de paquetes, pendiente hasta cerrar Fase 3.
- Fase 3: mapa sparse global; corresponde a la antigua Fase 1 y es la fase
  actual de esta conversacion.
- Fase 4: fiducial real sin GT funcional.
- Fase 5: pose global de cada dron sin ground truth.
- Fase 6: tareas y trayectorias de mapeo.
- Fase 7: GUI 3D propia de operacion.
- Fase 8: nube densa global multi-dron.
- Fase 9: mejoras avanzadas futura; queda como placeholder y sus subfases se
  realizaran cuando se avance a esa fase.
- `PIPELINE_MAESTRO.md`, `AGENTS.md`, `CODEX_INDEX.yaml`, los cuatro MDs de
  arranque/estado, ADR_0004/ADR_0006 y referencias operativas auxiliares ya
  fueron sincronizados el 2026-08-09.
- Busqueda amplia posterior: no quedan referencias operativas a
  `fase_1_sparse_global`, `pipeline_fase_1_RESUMEN` como sparse antiguo,
  `--fase 1L` ni nombres `f1/F1` en los artefactos existentes de
  `codex/archivos_auxiliares/html`, `logs` y `repeticiones`.
- Se corrigieron referencias conceptuales residuales `1B/1C/1G/1L/1N/1O` a
  `3B/3C/3G/3L/3N/3O` en arquitectura, topics, reglas, mapa de codigo,
  pruebas clave, ADR_0003, docs de paquetes y YAMLs de trayectoria.
- Las ocurrencias restantes `F1*`/`f1*` son marcadores, parametros o comandos
  legacy del runtime/historial, o pertenecen a la Fase 1 nueva de control del
  dron; no son identificadores del pipeline sparse activo.

## Investigacion propuesta de problemas runtime

Observaciones aportadas por el usuario mediante un MD temporal:

- el visualizador web parece representar eventos antiguos y puede quedarse
  varios segundos por detras del backend;
- se observaron tareas de loop para KFs de submapas aun no anclados;
- KFs y MapPoints de submapas anclados tardaron demasiado en aparecer en
  RViz2;
- el worker acumulo backlog elevado y se desconoce la edad real de las tareas;
- una correccion aparentemente aceptada del dron antihorario parecio
  desaparecer al final de una ejecucion;
- otra ejecucion no produjo esa optimizacion, indicando no determinismo que
  puede depender de datos, tracking, revisiones o scheduling.

Resultados del diagnostico:

- loops pre-anchor confirmados: `31/66` starts en prueba 75 y `38/84` en 76;
- backlog confirmado: 75 encola `561` y conserva `496` en el ultimo start; 76
  encola `489`, pico `429`, ultimo valor `414`;
- callback bajo mutex: p95 `5.159/4.251 s`, maximo `12.829/17.881 s` en 75/76;
- request->commit RViz2: maximo `20.283/27.951 s`; la captura live domina y el
  publish ROS consume pocos milisegundos;
- perdida raw de optimizacion no confirmada: en 75 hay tres commits de poses
  aceptados posteriores al primer commit fiducial y no hay rollback;
- diferencia de entrada confirmada: 75 tiene `13` observaciones de `fid=1` y
  76 tiene cero;
- replay web confirmado por diseno: cliente `400 x 110 ms`, pulsos `520 ms` y
  reconexion SSE desde cero con hasta `512` eventos antiguos;
- no se puede separar espera de mutex/copia ni medir edad causal exacta de
  tareas con la instrumentacion actual.

No autorizado todavia: cambiar thresholds, solver, scheduling, semantica raw,
numero de threads o arquitectura. Toda correccion funcional requiere acuerdo
nuevo.

## Prueba Acordada Para La Implementacion Posterior

1. Build de `orbslam3_multi`, `orbslam3_server` y `simulacion_dron`.
2. Tests deterministas de ingesta/poses, prioridad de cola, commits atomicos,
   fusion y publicacion coherente.
3. Prueba larga `prueba_tipica_rodeo_edificio_dos_fiduciales.yaml` con
   `multi_dron.launch.py`, RViz2 y visualizador JavaScript activos.
4. Validar que el flujo principal sigue recibiendo/publicando mientras una
   tarea secundaria esta activa; que la prioridad observada es
   `tarea activa -> fiducial pendiente -> loops pendientes`; y que cada commit
   aceptado aparece despues en RViz2 sin bloquear el cierre de la tarea.
5. Validar mediante el diagrama los eventos y tooltips sin que su desconexion o
   saturacion afecte a la simulacion.

## Puertas Permanentes

- Leer fisicamente este archivo antes del resto del contexto en un chat nuevo o
  tras compactacion; el resumen automatico no lo sustituye.
- Una primera orden de ejecutar inicia preparacion. Solo una orden posterior al
  acuerdo cerrado concede autorizacion funcional.
- Tras cada cambio, build, test, simulacion o diagnostico de una tarea larga,
  reemplazar este checkpoint inmediatamente.
- Los logs completos solo alimentan reductores; nunca se abren directamente.
- Cada prueba conserva su propia evidencia y conclusion historica.

## Archivos Relevantes

```text
codex/pipeline/fase_3_sparse_global/subfases/subfase_3F_publicacion_reactiva.md
codex/pipeline/fase_3_sparse_global/subfases/subfase_3K_worker_secundario.md
codex/pipeline/fase_3_sparse_global/subfases/subfase_3C.md a subfase_3H.md
codex/pipeline/fase_3_sparse_global/subfases/subfase_3I.md a subfase_3Q_*.md
codex/pipeline/fase_3_sparse_global/subfases/subfase_3S_*.md a subfase_3W.md
codex/pipeline/fase_3_sparse_global/subfases/subfase_3X.md
codex/pipeline/fase_3_sparse_global/subfases/subfase_3P.md
codex/pipeline/fase_3_sparse_global/subfases/subfase_3Q_*.md
codex/pipeline/fase_3_sparse_global/subfases/subfase_3U.md
codex/pipeline/fase_3_sparse_global/pipeline_fase_3_RESUMEN.md
codex/pipeline/fase_3_sparse_global/historial/por_subfase/historial_general_RESUMEN.md
codex/pipeline/fase_3_sparse_global/historial/por_subfase/historial_3P_RESUMEN.md
```
