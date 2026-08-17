# Historial pruebas tipicas

> Extraido mecanicamente de `historial_fase_3.md`. Leer este archivo antes de abrir otros historiales de subfase.

## 2026-07-09 — Prueba tipica larga posterior a 3H — Rodeo del edificio con dos fiduciales

- objetivo:
  - ejecutar la prueba larga comentada por el usuario antes de avanzar a `3I`;
  - ambos drones rodean la casa en sentidos contrarios, pasan por fiducial 2, fiducial 1 y vuelven a fiducial 2 mirando hacia el edificio;
  - validar que la trayectoria queda disponible como prueba tipica y que los logs aportan evidencia para las subfases de optimizacion fiducial.
- archivos creados/modificados:
  - `codex/archivos_auxiliares/trayectorias/prueba_tipica_rodeo_edificio_dos_fiduciales.yaml`;
  - `codex/archivos_auxiliares/trayectorias/tray_prueba_3.yaml`;
  - `orbslam3_server/launch/global_orb_map_server.launch.py`;
  - `simulacion_dron/launch/multi_dron.launch.py`;
  - `codex/contexto/pruebas_clave/pruebas_tipicas.md`;
  - `codex/contexto/paquetes/simulacion_dron/simulacion_dron.md`;
  - `codex/contexto/paquetes/simulacion_dron/launches.md`;
  - `codex/contexto/paquetes/orbslam3_server/launches.md`;
  - `codex/contexto/07_ULTIMA_SESION.md`;
  - `codex/pipeline/fase_3_sparse_global/historial/historial_fase_3.md`.
- cambios de launch:
  - `global_orb_map_server.launch.py` conserva configurados los fiduciales legacy `ids=[1,2]`, `x=[0,0]`, `y=[9,-9]`, `z=[1,1]`, `yaw=[0,0]`, `radius=[2,2]`;
  - `multi_dron.launch.py` permite pasar `rawdb_record_enabled` y `rawdb_record_path` al servidor global.
- compilacion:
  - no se ejecuto `build_selected_packages.sh` porque los cambios fueron launch/YAML/documentacion y los launch instalados son symlinks a `src`;
  - se valido sintaxis con `python3 -m py_compile simulacion_dron/launch/multi_dron.launch.py orbslam3_server/launch/global_orb_map_server.launch.py`;
  - se valido YAML con `yaml.safe_load`.
- simulacion ejecutada:

  ```bash
  ./codex/herramientas/run_simulation.sh --prueba 3 --launch "ros2 launch simulacion_dron multi_dron.launch.py rawdb_record_enabled:=false" --post-scenario-wait-sec 30 --startup-wait-sec 20 --timeout-sec 1200 --max-gazebo-retries 1
  ```

- resultado:
  - `SCENARIO-RUNNER-DONE scenario='prueba_tipica_rodeo_edificio_dos_fiduciales' success=true`;
  - `SIM-DONE prueba=3 success=true`;
  - `SIM-EXIT-CODE 0`;
  - se genero `codex/archivos_auxiliares/logs/prueba_3.log`;
  - se genero `codex/archivos_auxiliares/logs/prueba_3.reduced.log`;
  - no se genero `.record` nuevo porque se ejecuto con `rawdb_record_enabled:=false`.
- patrones de reduccion:

  ```text
  SCENARIO-RUNNER|SIM-DONE|SIM-EXIT-CODE|F1E-FID|F1H-FID|F1F-GLOBALMAP|F1G-FULL|ERROR|FATAL|Segmentation fault|Killed
  ```

- evidencia positiva:
  - inicio en fiducial 2 con revisits `[F1H-FID-OK]` y errores bajos;
  - paso por fiducial 1 con `[F1E-FID-KF-ASSOC] fid=1`;
  - creacion de tareas fiduciales con `[F1H-FID-TASK-CREATED]`, por ejemplo `task_id=1 fid=1 drone_id=1 kf=211 error_t=0.445654` y `task_id=2 fid=1 drone_id=2 kf=158 error_t=22.743950`;
  - vuelta al fiducial 2 con revisits `[F1H-FID-OK]` en epochs nuevos, por ejemplo `drone_id=2 epoch=2 kf=245 error_t=0.001731` y `drone_id=1 epoch=3 kf=314 error_t=0.025037`;
  - estadistica final: `total=41 pending=41 confirmed_ok=65 high_error=41 duplicates=0 no_pose=0 revisits=106`.
- evidencia negativa o ausente:
  - no aparecieron `FATAL`, `Segmentation fault`, `Killed` ni `std::bad_alloc`;
  - aparece `gazebo ... exit code 255` despues de `SIM-DONE`, durante cleanup, patron ya clasificado como no bloqueante;
  - el log reducido no conserva todas las lineas de `TASK_CREATED`, por lo que la evidencia detallada de fiducial 1 se consulto tambien en `prueba_3.log`.
- conclusion: `PRUEBA LARGA CONSEGUIDA`.
- siguiente paso recomendado:
  - ejecutar `subfase_3I.md`, usando `tray_prueba_3.yaml` como prueba de regresion larga cuando interese forzar tareas fiduciales reales.

## 2026-07-23 — Prueba tipica corta 3I — Fiducial 2 a fiducial 1 por dos lados

- objetivo:
  - crear una variante corta de la prueba de rodeo;
  - hacer que ambos drones vayan al fiducial 2, recorran lados opuestos hasta
    fiducial 1 y se paren alli;
  - observar los errores de ambos drones al llegar al fiducial 1 y conservar
    HTML/TSV para repetir la optimizacion sin repetir Gazebo.
- archivos creados/modificados:
  - `codex/archivos_auxiliares/trayectorias/prueba_tipica_fiducial_2_a_1_dos_lados.yaml`;
  - `codex/contexto/pruebas_clave/pruebas_tipicas.md`;
  - `codex/contexto/paquetes/simulacion_dron/00_summary.md`;
  - `codex/contexto/paquetes/simulacion_dron/launches.md`.
- simulacion ejecutada:

  ```bash
  ./codex/herramientas/run_simulation.sh --prueba 27 --yaml /home/chenfu/Gazebo/src/codex/archivos_auxiliares/trayectorias/prueba_tipica_fiducial_2_a_1_dos_lados.yaml --startup-wait-sec 2 --post-scenario-wait-sec 60 --launch "ros2 launch simulacion_dron multi_dron.launch.py pose_graph_vertex_selection_ratio:=0.30 pose_graph_use_covisibility_edges:=false pose_graph_fiducial_neighborhood_vertex_ratio:=0.20 loop_bow_min_mappoints:=1000000 f1l_debug_animation_enabled:=true f1l_graph_dump_enabled:=true f1l_gt_kf_debug_enabled:=true f1l_gt_kf_debug_max_dt_sec:=1.0"
  ```

- resultado:
  - `SCENARIO-RUNNER-DONE scenario='prueba_tipica_fiducial_2_a_1_dos_lados' success=true`;
  - `SIM-DONE prueba=27 success=true`;
  - `SIM-EXIT-CODE 0`;
  - todos los pasos del escenario terminan, incluido `ambos_al_fiducial_1`.
- evidencia fiducial:
  - `drone_1`: `fid=1`, `kf=226`, `error_t=0.158574`,
    `error_rot=0.025352`, `error_yaw=0.021771`, `decision=OK`;
  - `drone_2`: `fid=1`, `kf=203`, `error_t=28.937918`,
    `error_rot=2.908612`, `error_yaw=2.905030`,
    `decision=TASK_CREATED`, `task_id=2`;
  - las observaciones posteriores de `drone_2` en fiducial 1 quedan como
    `TASK_DUPLICATE`, con errores de `29.9-30.5 m`.
- evidencia grafo/optimizacion:
  - `task_id=2`: `window_keyframes=130`, `vertices=44`, `edges=43`,
    `coverage_complete=true`;
  - dry-run live: target `28.937918 m -> 0`, yaw `2.905030 -> 0`,
    coste `69777420.092643 -> 512473.703446`;
  - dump live:
    `codex/archivos_auxiliares/repeticiones/f3i_window_task_2.tsv` y
    `codex/archivos_auxiliares/repeticiones/f3l_graph_task_2.tsv`;
  - HTML live:
    `codex/archivos_auxiliares/html/f3l_debug_animation_task_2.html`;
  - replay offline sobre el TSV: `[F1L-OFFLINE-LOAD] success=true`,
    `[F1L-OFFLINE-HTML] success=true`,
    `codex/archivos_auxiliares/html/f3l_offline_graph_task_2_prueba_27_3d.html`;
  - metrica offline de ventana: `mean_before=7.18291`,
    `mean_after=5.0564`, `max_before=26.0489`, `max_after=11.059`,
    `worsened_kfs=75`.
- evidencia negativa:
  - se crea antes una tarea pequeña en fiducial 2 para `drone_2`, pero el
    builder falla por `previous_fiducial_anchor_missing`; queda como deuda de
    diagnóstico;
  - el apply live de `task_id=2` se rechaza por
    `global_map_check_failed`, con `invalid_pose_skipped_before=153` y
    `invalid_pose_skipped_after=408`; esta causa concreta se corrigió después
    en `3K` con publicación por cobertura de KFs corregidos;
  - hay `gazebo ... exit code 255` durante cleanup despues de `SIM-DONE`,
    patron ya clasificado como no bloqueante.
- conclusion: `PRUEBA TIPICA CONSEGUIDA COMO ESCENARIO Y DIAGNOSTICO`;
  optimizacion `PARCIAL` porque el candidato corrige el target pero no supera
  el guard de mapa global.
- siguiente paso recomendado:
  - usar los dumps actuales de `prueba_31` para ajustar pesos, vecindades
    protegidas y propagacion offline; despues repetir la prueba observando
    RViz2 junto al HTML 3D.

## 2026-07-27 — Prueba típica de aislamiento con un dron

- trayectoria:
  `prueba_rodeo_antihorario_un_dron_fid2_fid1_fid2.yaml`;
- recorrido: `drone_2` fiducial 2 -> fiducial 1 -> fiducial 2 en sentido
  antihorario; `drone_1` permanece parado;
- ejecución: `prueba_41`, escenario completo, `SIM-EXIT-CODE 0`, un único
  `map_epoch`;
- propósito: aislar dos optimizaciones fiduciales consecutivas y comparar
  HTML 3D con RViz2;
- resultado: loop interno ORB-SLAM3 ausente, task 1 mejora la ventana y task 2
  corrige el fiducial pero empeora la ventana;
- validación visual del 2026-07-28: el usuario confirma que ambos applies, los
  KFs posteriores y los MapPoints se ven perfectamente en RViz2, sin retorno
  de la primera ventana a poses antiguas;
- conclusión: escenario y aplicación/publicación `CONSEGUIDOS`; la métrica GT
  debug de task 2 queda como observación no bloqueante.

## 2026-07-28 - Prueba tipica larga del runtime historico

- trayectoria:
  `prueba_tipica_rodeo_edificio_dos_fiduciales.yaml`;
- ejecución: `prueba_42`, escenario completo, `SIM-EXIT-CODE 0`;
- reejecución `prueba_43`: valida controles fiduciales para KFs llegados
  durante el solver (`task 2`: cuatro controles, siete KFs refinados, uno
  derivado), HTML post-commit y nube estable con `invalid_pose_skipped=0`;
- reejecución `prueba_44`: valida nueve KFs tardíos dentro de la ventana
  original; los nueve se refinan, el HTML contiene los 95 KFs reales y la nube
  permanece estable con `invalid_pose_skipped=0`;
- proposito: comprobar la aplicacion de dos llegadas con error alto y la
  reconciliacion de KFs incorporados durante el solver;
- tareas principales: `task_id=1`, `(drone=1, epoch=0)`, 63 KFs; y
  `task_id=2`, `(drone=2, epoch=0)`, 91 KFs;
- resultado final: tres tareas aplicadas/aceptadas, sin poses inválidas,
  `scheduled=3`, `completed=3`, `active=0`;
- limitación: no hubo solape de ventanas (`waited=0`) y el dron 1 perdió
  tracking, regresando al fiducial 2 como `epoch=3`;
- conclusion: prueba tipica util para apply y continuidad larga `PARCIAL`.

## 2026-08-05 - `prueba_73` - Primer runtime del worker unico

- objetivo intentado: validar la separacion principal/secundaria con la prueba
  larga de dos fiduciales y el visualizador activo;
- resultado operativo: scenario `success=true`, `SIM-EXIT-CODE 0`;
- worker: una sola tarea activa y cola limitada inicialmente a `10` loops;
- admision acumulada: `2741` solicitadas, `325` enqueued, `357` coalesced y
  `2059` dropped;
- evidencia positiva: el escenario y la publicacion principal continuaron
  mientras el worker procesaba tareas;
- evidencia negativa: el limite convertia la cola en perdida masiva de KFs y
  no representaba el contrato de procesar uno a uno el trabajo material;
- conclusion: `PARCIAL`;
- correccion derivada: admitir solo KFs materiales y aumentar el limite duro a
  `4096`, manteniendo telemetria y sin usar backpressure para pausar la ruta.

## 2026-08-05 - `prueba_74` - Timeout y loop absoluto incorrecto

- objetivo intentado: repetir la ruta con la admision corregida y validar el
  drenaje del worker;
- resultado operativo: `SIM-EXIT-CODE 124`; una tarea para `query=1:0:221`
  tarda `144224.952 ms` y el escenario excede el timeout;
- evidencia funcional: se aceptan commits de poses por loop. El caso principal
  contra `candidate=2:3:192` mueve `31` KFs del grafo y propaga la correccion a
  `92` KFs;
- diagnostico: la transformacion relativa query-candidate se estaba usando
  como objetivo world absoluto del query;
- conclusion: `NO CONSEGUIDA`;
- correccion derivada: arista `LOOP_RELATIVE`, candidato fijo, error relativo y
  soporte geometrico previo.

## 2026-08-05 - `prueba_75` - Prioridad correcta y segundo fallo causal

- objetivo intentado: validar el grafo relativo, el worker unico, la prioridad
  fiducial y el diagrama mejorado;
- resultado operativo: scenario `success=true`, `SIM-EXIT-CODE 0`, sin crash;
- worker: terminaron las `66` tareas loop que llegaron a empezar y `2` tareas
  fiduciales, siempre una sola activa; se encolaron `561` loops, el maximo
  observado fue `498` y el ultimo inicio aun mostraba `496` pendientes;
- prioridad: una tarea fiducial pendiente empieza al terminar la tarea activa,
  antes que los loops pendientes;
- flujo principal: al menos `103` publicaciones en el artefacto tematico aun
  con backlog secundario;
- 3Q positivo: loop inter-submapa `query=2:1:57`, `candidate=1:0:13`, con `7`
  KFs optimizados y `224` propagados;
- evidencia negativa: tambien se acepta `query=2:1:62` contra el KF posterior
  `candidate=2:1:75`, gap `13`, y se propaga a `248` KFs. Este segundo caso
  explica el movimiento injustificado observado por el usuario;
- revision diagnostica del 2026-08-09: la primera optimizacion fiducial de
  `drone_id=2, epoch=1` se acepta a `1785964351.197`, pero despues se aceptan
  el loop inter-submapa (`7+224` poses), el loop intra-submapa (`6+248`) y una
  segunda optimizacion fiducial (`27+77`). No hay rollback ni evidencia de que
  raw sobrescriba el commit: el resultado visible cambia por commits de poses
  posteriores y por publicacion tardia;
- publicacion: el primer snapshot que necesariamente incluye el primer commit
  fiducial aparece unos `13.1 s` despues; el segundo, unos `4.1 s` despues;
- conclusion: `PARCIAL`;
- correccion derivada: descartar candidatos posteriores del mismo submapa y
  suprimir optimizacion de `near_same_submap` de error alto.

## 2026-08-05 - `prueba_76` - Regresion causal conseguida, flujos aun parciales

- objetivo intentado: repetir exactamente la ruta despues de la segunda
  correccion y comprobar logs de worker, poses, publicacion y bridge;
- resultado operativo: scenario `success=true`, `SIM-EXIT-CODE 0`;
- worker: terminaron las `84` tareas loop que llegaron a empezar, una sola
  activa; se encolaron `489`, el pico fue `429` y el ultimo inicio observado
  aun tenia `414` loops pendientes. `429` no era backlog final demostrado;
- filtro: `skipped_noncausal_same_submap` aparece con valores positivos desde
  queries tempranas y evita usar KFs posteriores del mismo submapa;
- 3Q: cuatro calculos sin soportes previos suficientes; no aparece ningun
  `F1Q-LOOP-OPT-COMMIT`, por lo que no se escriben poses por loops;
- flujo principal: `144` publicaciones hasta la espera post-run, incluso con
  backlog secundario;
- visualizador: bridge `READY`, transporte ROS/SSE disponible y cierre limpio;
- errores: un `gazebo process has died`, exit `255`, aparece despues de
  `SIM-DONE` durante cleanup; el wrapper termina con codigo `0`;
- evidencia ausente: esta ejecucion no genero una tarea fiducial de error alto
  ni tuvo confirmacion humana nueva en RViz2/navegador;
- revision diagnostica del 2026-08-09:
  - `38/84` loops empezaron antes del primer anchor de su submapa y consumieron
    unos `61 s` de worker acumulados;
  - `prueba_75` tuvo `13` observaciones de `fid=1`; `prueba_76`, cero. Por eso
    no podia crear la tarea fiducial equivalente;
  - el tramo del callback posterior al marcador raw y previo al unlock mide
    media `728 ms`, p95 `4.251 s`, p99 `13.270 s` y maximo `17.881 s`;
  - request a commit de RViz2: media `2.837 s`, p95 `12.971 s`, maximo
    `27.951 s`; la captura bajo `live_state_mutex_` domina el retraso;
- conclusion revisada: `CONSEGUIDA` solo como regresion del movimiento espurio
  por loop; `PARCIAL` para separacion temporal del flujo principal, deteccion
  fiducial reproducible, backlog y visualizador live.
