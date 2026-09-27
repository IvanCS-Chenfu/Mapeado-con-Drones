# Contexto de compactacion

## Repeticion 8.5.6 lenta hacia fiducial

Trabajo activo: si.
Preparacion: CERRADA
Acuerdo cerrado: si
Autorizacion funcional: CONCEDIDA
Prueba acordada: detener `run_08`, reducir exclusivamente la velocidad de la
aproximacion GT simultanea de D1/D2 al fiducial 2 y repetir la prueba integrada
8.5.6. La separacion posterior y toda la autonomia del servidor se conservan.
Dudas abiertas: ninguna.

Estado: `run_08` fue interrumpida a peticion del usuario; sus grupos de
procesos y nodos ROS se cerraron y `ros2 node list` quedo vacio. En el
escenario `autonomous_gt_integrated_8_5_6.yaml`, `tx/ty/tz/tyaw` son las
duraciones Pol3 por eje.

Ejecucion `run_09`: interrumpida por el usuario durante la fase autonoma;
no es evidencia final. El primer intento Gazebo murio temprano y el retry
arranco correctamente; el escenario completo su bootstrap. La aproximacion
al fiducial se aplico en `32/32/32/16 s`; separacion y seguridad quedaron sin
cambios. La observacion del usuario es que task_server sigue introduciendo
esperas largas entre ordenes y los drones no encadenan movimiento.

Diagnostico/correccion autorizados: examinar todas las transiciones de
task_server, aislar el bloqueo real y corregirlo sin relajar los vetos
FREE/UNKNOWN/OCCUPIED/RESERVED ni eliminar depth, STOP, riesgo visual o
inflacion. Compilar y repetir hasta que ambos drones reciban ordenes de forma
continua. ROS quedo vacio tras cerrar el grupo residual. Siguiente accion:
analizar exclusivamente los logs reducidos y los workers/timers del servidor;
despues aplicar la correccion minima y validar con builds y nuevas simulaciones.

## Checkpoint vigente

Peticion actual: preparar la prueba integrada 8.5.6 con dos drones.
Preparacion: CERRADA
Acuerdo cerrado: si
Autorizacion funcional: CONCEDIDA
Prueba acordada: D1 y D2 llegan con GT al fiducial 2 y se separan con GT a `(-2,-10,1)` y `(2,-10,1.3)` antes de abrir `/mission/set_coverage_execution_enabled`. A partir de ahí el servidor moderno decide tareas, secciones, poses, planes D*, reservas, capturas y replanificaciones; no hay goals manuales, triggers de prueba ni barrido legacy. Se restauran depth inspection/evidence/stop, visual risk, inflación adicional 1 y planificación estricta FREE. La prueba conserva los STOP y registra reasignación/reselección posterior, sin relajar UNKNOWN/OCCUPIED/RESERVED.
Dudas abiertas: ninguna.

Plan: añadir en task_server un Trigger y parametros de prueba apagados por defecto, usar el dispatcher normal con LOOK_AND_CAPTURE, contar fuentes FREE/direct_free/occupied y detener unicamente su continuacion al aplicar las fuentes. Ajustar launch, escenario y capturador/procesador 8.4-A; compilar task_server y simulacion_dron; ejecutar la repeticion con D1, GT, depth inspection/evidence activos, depth stop y tracking risk apagados; reducir el log por F6C/F6D/F6F/F8A y documentar el resultado.

Trabajo activo: si, implementacion y ejecucion 8.4-A. No hay nodos de
simulacion activos.

Cambios completados: se crearon `autonomous_gt_view_unknown_8_4_a.yaml`
(perfil y escenario), y el capturador pasivo
`Pruebas/Capítulo 8/scripts/capturar_8_4_a_view_unknown.py`. El escenario
realiza el bootstrap GT al fiducial 2, espera 12 s y abre el handoff normal;
el capturador congela before cuando el task state solicita depth para una pose
UNKNOWN y after con el primer crecimiento FREE. YAML, `py_compile` y
`git diff --check`: PASS. La documentacion de `simulacion_dron` refleja
`autonomous_handoff` y ambos perfiles de Capitulo 8.

Build `simulacion_dron`: PASS, codigo 0. Aviso conocido sin impacto: ruta Drake inexistente en `CMAKE_PREFIX_PATH`.

Intentos de simulacion `c8_4_a_view_unknown_run_01`: DOS PREARRANQUES sin nodos ni datos. El primero omitio `--mission-profile`; el segundo uso una ruta instalada inexistente porque el install actual no expone el YAML nuevo como archivo independiente. Correccion mecanica final: usar la ruta fuente valida `simulacion/simulacion_dron/config/mission_profiles/autonomous_gt_view_unknown_8_4_a.yaml`, admitida por launch y scenario runner.

Ejecucion efectiva `c8_4_a_view_unknown_run_01`: el wrapper, Gazebo, scenario runner y capturador cerraron; no quedan procesos activos. El codigo/success del wrapper se verificara exclusivamente mediante el log reducido. Log bruto preservado en `codex/archivos_auxiliares/logs/prueba_c8_4_a_view_unknown_run_01.log`; artefactos del capturador en `Pruebas/Capítulo 8/8_4_A_view_unknown/run_01/`.

Siguiente accion: reducir por `SIM-DONE|SCENARIO-RUNNER|F6C|F6D|F6F|F6G|F6H|LOOK|vista_unknown|VIEW_UNKNOWN|ERROR|FATAL`, analizar el reducido y contrastar los snapshots before/after.

Ejecucion `run_01`: PARCIAL. Wrapper/escenario PASS y D1 ejecuto el primer
`LOOK_AND_CAPTURE` (`command=...:2:1`, type 2), termino `look_completed` con
una observacion depth y el servidor escribio/aplico una fuente
`kind=vista_unknown`. No hubo coverage asociado a ese command. Sin embargo el
capturador eligio el primer incremento FREE posterior a la solicitud, antes de
la materializacion correlacionada; despues la autonomia despacho una orden
distinta `VIEW_ADVANCE` que activo coverage. `run_01` no se usa para concluir
8.4-A.

Correccion mecanica: el capturador de `run_02` guarda 15 s de historia de
VoxelMap y yaw tras solicitar UNKNOWN. El procesador seleccionara el primer
mapa con sello posterior a `[F6F-DEPTH-SOURCES-APPLIED]` del primer command
type 2, y medira el giro firmado entre aceptacion y aplicacion.

Ejecucion efectiva `c8_4_a_view_unknown_run_02`: wrapper, Gazebo, scenario runner y capturador cerraron; no quedan procesos activos. Log bruto preservado en `codex/archivos_auxiliares/logs/prueba_c8_4_a_view_unknown_run_02.log`; el capturador dejo baseline, 11 snapshots de historia VoxelMap y telemetria temporal de yaw en `Pruebas/Capítulo 8/8_4_A_view_unknown/run_02/`.

Siguiente accion: reducir por los marcadores de scenario/F6C/F6D/F6F/F6G/F6H, ejecutar `procesar_8_4_a_view_unknown.py` sobre el reducido y comparar los snapshots correlacionados.

Ejecucion `run_02`: NO CONSEGUIDA. Wrapper/escenario PASS y primer
`LOOK_AND_CAPTURE` correlacionado `...:2:2` completo `look_completed`, depth=1
y `kind=vista_unknown`; no tuvo claims de coverage. El clock relativo de
NavigationState se alineo con VoxelMap mediante la historia temporal: yaw
`90.00 -> 56.93 deg`, delta `-33.07 deg`, no aproximadamente `-90 deg`.
La mirada tuvo evidencia sparse simultanea, por lo que el total VoxelMap no
aísla un delta OCCUPIED atribuible solo al depth. No repetir sin nueva decision
funcional sobre como imponer la direccion derecha de 90 grados.

Trabajo activo: si. No hay nodos de simulacion activos.

Cambios de la repeticion determinista: task_server incorpora el Trigger opt-in `/mission/test_view_unknown_right`, calcula el objetivo visual a `yaw - 90 grados` y lo despacha como `LOOK_AND_CAPTURE` normal. La continuacion se suprime solo para ese workflow tras materializar las fuentes. El marcador F6F ahora expone `free`, `direct_free` y `occupied`. Launch, escenario y procesadores 8.4-A ya usan el Trigger y el handoff general queda apagado. `git diff --check`, YAML y `py_compile`: PASS.

Build task_server: resultado no confirmado por salida truncada del ejecutor y binario instalado ausente; se repetira aisladamente y, si falla, se reducira su log especifico. Build simulacion_dron: PASS, codigo 0; persiste solo el aviso conocido de Drake.

Build task_server repetido: PASS, codigo 0, paquete instalado. Build simulacion_dron: PASS, codigo 0. En ambos casos solo aparece el aviso conocido de ruta Drake inexistente en `CMAKE_PREFIX_PATH`.

Prueba siguiente: `c8_4_a_view_unknown_run_03`, un dron, perfil `autonomous_gt_view_unknown_8_4_a`, GUI de Gazebo opcional para observacion, `phase6_test_view_unknown_right_enabled=true`, depth inspection/evidence activos, depth stop y visual risk apagados. El capturador pasivo guardara baseline, historia VoxelMap y yaw; la duracion posterior debe cubrir giro, captura y materializacion.

Intento de lanzamiento `run_03` previo: INVALIDO antes de crear nodos, Gazebo o datos. El shell activo `set -u` antes de cargar `/opt/ros/iron/setup.bash`; este setup accede a una variable opcional sin definir. Correccion mecanica: cargar los setups antes de habilitar `nounset`; se conserva exactamente la misma prueba y parametros.

Ejecucion efectiva `c8_4_a_view_unknown_run_03`: capturador, scenario runner, Gazebo y wrapper cerraron; no quedan procesos asociados. El capturador termino con `anchored_ready=true`, baseline disponible y 56 snapshots de historia VoxelMap. El codigo/success del wrapper y la evidencia tecnica se determinaran exclusivamente desde el log reducido. Log bruto preservado en `codex/archivos_auxiliares/logs/prueba_c8_4_a_view_unknown_run_03.log`; artefactos en `Pruebas/Capítulo 8/8_4_A_view_unknown/run_03/`.

Resultado `run_03`: CONSEGUIDA. Wrapper/escenario PASS (`success=true`). El Trigger determinista despacho `LOOK_AND_CAPTURE` `autonomous:test_view_unknown_right:1:1` con objetivo `yaw 90 -> 0 grados`; NavigationState midio `-90.0009 grados`. El resultado fue `look_completed`, una observacion depth y `kind=vista_unknown`. Fuentes del comando: `free=1`, `direct_free=0`, `occupied=0`; coverage sin cambio. El VoxelMap global paso FREE `6073 -> 9331` y OCCUPIED `556 -> 671`, pero ese incremento agregado de OCCUPIED coincide con materializacion sparse y no se atribuye al depth: el contador de fuente correlacionada es cero. La continuacion de prueba se suprimio tras aplicar la fuente, sin `VIEW_ADVANCE` posterior.

Documentacion y artefactos actualizados: resumen task_server, summary/launches de simulacion y `Pruebas/Capítulo 8/8_4_A_view_unknown/{notas.md,run_03/summary.md}`.

Repeticion visual autorizada: `run_04` reproduce exactamente `run_03`, con `launch_gazebo_gui=true` y 120 s posteriores al escenario para foto/video. Conserva D1, GT, Trigger determinista a la derecha, depth activo, depth stop y TRACKING_RISK apagados. El capturador registra tambien la evidencia, pero `run_03` sigue siendo la evidencia final vigente salvo que la nueva repeticion revele una divergencia material.

Ejecucion `run_04`: INTERRUMPIDA a peticion del usuario porque se inicio solo con Gazebo y no con la GUI del servidor. No se usa como evidencia. Se cerraron manualmente wrapper, capturador y Gazebo; no quedan procesos.

Repeticion visual `run_05`: CONSEGUIDA. Wrapper/escenario PASS (`success=true`) con Gazebo, GUI de mision y GUI global del servidor activas. Confirmo `LOOK_AND_CAPTURE` como `vista_unknown`, giro `-90.0039 grados`, `free=1`, `direct_free=0`, `occupied=0` y coverage sin cambio. Es una confirmacion visual; `run_03` conserva la referencia tecnica primaria.

Plan 8.4-B: perfil y escenario monodron anclan D1 y lo llevan con GT a `(-2,-8,1,90 grados)` antes de abrir el handoff autonomo normal. Los capturadores pasivos registran TaskState/intervalos U, VoxelMap y navegacion; el procesador correlaciona el primer `MOVE_AND_CAPTURE` con F6F/F6H y el cambio de progreso. Validar YAML/scripts, compilar `simulacion_dron`, ejecutar con Gazebo, GUI de mision y GUI global, reducir por F6C/F6D/F6F/F6G/F6H/F6N y documentar cada resultado.

Cambios 8.4-B completados: `autonomous_gt_view_wall_8_4_b.yaml` (perfil y escenario) y los scripts pasivos `capturar_8_4_b_view_wall.py`/`procesar_8_4_b_view_wall.py`. No se modifico la logica de `task_server`, cobertura ni depth.

Validacion 8.4-B: YAML PASS, `py_compile` PASS y `git diff --check` PASS.

Build `simulacion_dron`: PASS, codigo 0. Aviso conocido sin impacto: ruta Drake inexistente en `CMAKE_PREFIX_PATH`.

Prueba siguiente: `c8_4_b_view_wall_run_01`, un dron, perfil/escenario 8.4-B, D1 GT a `(-2,-8,1,90 grados)`, depth inspection/evidence activos, handoff autonomo normal y las GUIs de Gazebo, mision y servidor. El capturador pasivo durara 390 s; el escenario conservara 240 s posteriores para observacion.

Ejecucion `run_01`: NO CONSEGUIDA. Wrapper/escenario PASS, D1 llego a `(-2,-8,1,90 grados)`, el handoff normal asigno `map_section_level_0_AB` y la tarea quedo RUNNING, pero no aparecio `MOVE_AND_CAPTURE`, depth ni claims. Causa: `phase6_execute_facade_sweeps=false` deja inactivo `RunFacadeWorker` aunque `execution_enabled=true`.

Ejecucion `run_02`: INTERRUMPIDA. Con `phase6_execute_facade_sweeps=true`, `RunFacadeWorker` tomo depth de su candidato propio (`pose=(-0.62,-8.88,1.12)`, no la pose solicitada), lo rechazo como pared por `normal_not_valid` y entro en el ciclo inspeccionar/revisar. No es la prueba pedida y no se usa como evidencia. Se cerraron los procesos restantes.

Aclaracion funcional revisada del usuario: D1 se ancla primero. Despues `MOVE_AND_CAPTURE`, y no un movimiento GT previo, debe elegir el objetivo fijo `(-2,+8,1,90 grados)`, desplazarse hasta esa pose y tomar alli depth. El flujo debe conservar contexto de tarea, workflow y seccion U para permitir `VIEW_WALL`/`VIEW_ADVANCE`, fuentes directas y claims, pero el selector automatico no puede sustituir ese objetivo.

Plan revisado 8.4-B: Trigger opt-in implementado en `task_server`, parametros expuestos por `multi_dron.launch.py` y escenario adaptado. Fija la seccion U del objetivo de prueba, planifica solo por FREE y despacha `MOVE_AND_CAPTURE` correlacionado; no hay segundo movimiento GT ni selector alternativo. Validacion YAML/Python/diff: PASS. Builds: `task_server` PASS, codigo 0 (solo avisos preexistentes de API ROS deprecada y ruta Drake ausente); `simulacion_dron` PASS, codigo 0. Para desbloquear la compilacion se limpiaron exclusivamente logs generados y `build/servidor/task_server`; quedan unos 124 MiB. Ejecuciones 8.4-B: `run_03` INVALIDA por ruta relativa del profile; `run_04` INTERRUMPIDA a peticion del usuario para cerrar todos los nodos; `run_05` NO CONSEGUIDA. D1 se anclo y el Trigger fue llamado, pero la dependencia residual de `SelectFacadeTaskCandidate` devolvio `test_view_wall_fixed_section_unavailable`, por lo que no armo ni despacho el comando. Correccion mecanica aplicada: el Trigger construye `FacadeCandidate` directamente desde el objetivo fijo y su seccion U, sin selector automatico. Validacion del cambio: `git diff --check` PASS y build `task_server` PASS, codigo 0. `run_06` armo correctamente `(-2,8,1,90 grados)`, pero no despacho: el planificador rechazo su celda con `goal_occupied_or_inflated`. `run_07` repitio el flujo sin cambios funcionales para `(-6,-9,1,90 grados)`: anclaje y Trigger PASS, pero su celda tambien fue rechazada con `goal_occupied_or_inflated`, sin enviar movimiento. El usuario observo voxeles OCCUPIED donde no los esperaba y pidio una repeticion identica para descartar un artefacto transitorio. Se cerro la ejecucion y se confirmo `ros2 node list` vacio. Prueba siguiente autorizada: `run_09`, misma configuracion que `run_07` y `run_08`. `run_08` reprodujo el rechazo `goal_occupied_or_inflated` para `(-6,-9,1,90 grados)`, sin movimiento ni captura; se cerraron los nodos y ROS quedo vacio.

Trabajo activo: si, corrección funcional y repetición 8.5.6 autorizadas.
Preparacion: CERRADA
Acuerdo cerrado: si
Autorizacion funcional: CONCEDIDA
Prueba acordada: cuando, tras depth, el objetivo sea FREE pero no transitable por inflación, conservar la política estricta FREE y remitir el workflow a `TRAJECTORY_PLANNING` para que el fallback existente elija el waypoint FREE alcanzable más lejano de una ruta exploratoria. Debe ejecutar `VIEW_ADVANCE` y volver a seleccionar después; no cruzará UNKNOWN, OCCUPIED ni RESERVED. Se compila `task_server` y se repite 8.5.6 sin cambiar escenario ni parámetros de seguridad.
Dudas abiertas: ninguna.

Cierre 8.5.6: `run_01` fue NO CONSEGUIDA por `malloc(): unaligned fastbin chunk detected`; se identificó estado mutable compartido entre callback groups y se serializó en `map_callback_group_`. El build de `task_server` fue PASS. `run_02` es PARCIAL: ambos drones se anclaron, recibieron tareas y completaron depth; D2 realizó tres ciclos D*/reserva/movimiento, una `vista_pared`, dos `view_advance` y coverage de `22/192 = 11,46 %`, sin abortos. D1 materializó una `vista_unknown` pero no recibió continuación ni se movió post-handoff. La ejecución no prueba movimiento autónomo de ambos drones ni conflicto de reservas; datos, figuras y limitaciones constan en `Pruebas/Capítulo 8/8_5_6_autonomia_integrada/`. Diagnóstico posterior de D1: tras materializar FREE, el recheck emitió `state=free action=reselect`, lo que prueba que la celda no era transitable bajo perfil fachada inflado `(3,3,2)` y margen adicional 1. El selector guarda el ratio rechazado pero no lo consume, por lo que reencola silenciosamente el mismo candidato no transitable y mantiene D1 RUNNING sin comando. No es un STOP ni pérdida de pose. 8.4-B/run_14 sigue como evidencia vigente de 8.4-B y 8.2-B.


## Estado 8.3

- Cambio funcional ya implementado: `scenario_runner_node` acepta
  `autonomous_handoff` por YAML (default `true`). El perfil 8.3 lo fija a
  `false` para conservar evidencia sparse sin abrir tareas, planificador, D* ni
  movimiento autonomo. Build de `simulacion_dron` y prueba contractual:
  PASS (`10 passed`).
- `run_01`: PARCIAL. Wrapper/escenario PASS, pero el capturador escribio el
  conteo final de KFs en todas las muestras; no es valido como serie temporal.
- `run_02`: PARCIAL. Wrapper/escenario PASS, handoff omitido y depth apagado.
  Snapshots validos A/B/C: revision `0 -> 2 -> 684`; FREE `0 -> 21 -> 11568`;
  OCCUPIED `0 -> 0 -> 598`; KFs con evidencia `0 -> 0 -> 46`. B precede al
  primer delta de KF y por ello no satisface por si solo el criterio completo
  de 8.3.
- Figura temporal creada solo con `run_02`:
  `Pruebas/Capítulo 8/8_3_voxel_sparse/run_02/figures/voxel_free_occupied_temporal.png` (tambien se conserva SVG).
  CSV asociado: `processed/voxel_counts_temporal.csv`. `OCCUPIED` procede de
  MapPoints sparse cualificados; `FREE` agrega rayos sparse/RANSAC y volumen
  fisico libre asociado al movimiento/KFs, que `VoxelMap` no subtotaliza.
- `run_03`: INTERRUMPIDA por el usuario antes de iniciar correctamente, tras
  agotar el espacio para el log del wrapper. Sus datos no se usan.

Siguiente accion: esperar la decision del usuario sobre repetir 8.3 o continuar
con la siguiente prueba del capitulo 8.


## Cierre vigente 8.5.6 / run_03

Trabajo activo: no.
Preparacion: CERRADA
Acuerdo cerrado: si
Autorizacion funcional: CONCEDIDA y consumida
Prueba acordada: completada.
Dudas abiertas: ninguna para el criterio integrado; queda una cobertura dirigida opcional de `action=prefix_free`.

Resultado: CONSEGUIDA. Se corrigió en `task_server` la revalidación de un
objetivo `FREE` no transitable: conserva `TRAJECTORY_PLANNING`, marca
`fallback_free_advance` y permite que el planificador derive el prefijo FREE
estricto alcanzable más lejano para ejecutar `VIEW_ADVANCE`, sin cruzar
`UNKNOWN`, `OCCUPIED` ni `RESERVED`. Build `task_server`: PASS, código 0.

`c8_5_6_autonomia_integrada_run_03`: wrapper y escenario `success=true`; D1 y
D2 se movieron bajo decisión autónoma, con 14 y 4 commits D* respectivamente,
26 observaciones depth, 18 commits de reserva y máximo de 573 vóxeles
reservados. Ambos materializaron `DEPTH_OCCUPIED`: coverage U final D1
`13/192 = 6,77 %` y D2 `15/192 = 7,81 %`. Sin `malloc`, `FATAL` ni caída de
`task_server`; la salida tardía de la GUI Gazebo fue posterior a `SIM-DONE`.

Límite conocido: `run_03` tomó el fallback ya existente de objetivo `UNKNOWN`;
no forzó directamente el nuevo marcador `action=prefix_free`. Las evidencias,
figuras y resumen vigentes están en
`Pruebas/Capítulo 8/8_5_6_autonomia_integrada/run_03/`. No quedan nodos de
simulación activos.

Aclaración posterior de `run_03`: las primeras órdenes sí terminaron con
`depth=1`, pero solo crearon `depth_free`. No certifica ausencia de depth ni
un fallo de movimiento: para emitir `depth_occupied`, la integración exige
`normal_valid`, normal normalizable e incidencia frontal de como máximo
30 grados. La captura posterior que sí reunió esas condiciones activó coverage.
Diagnóstico preciso añadido: la primera `vista_pared` de D1 en
`run_03` tenía depth válido pero `normal=false`, soporte 138 y confianza 0,548
(< 0,700). La captura posterior que activó coverage tuvo normal válida, soporte
205 y confianza 0,704. La normal proviene de consistencia angular local de la
nube estéreo, no del yaw; las capturas `vista_unknown` solo producen FREE por
contrato aunque la normal sea válida.


## Ajuste autorizado de evidencia depth

Trabajo activo: si, ajuste parametrizable de la clasificación depth.
Preparacion: CERRADA
Acuerdo cerrado: si
Autorizacion funcional: CONCEDIDA
Prueba acordada: no se solicita nueva simulación todavía; compilar y validar los
parámetros.
Dudas abiertas: ninguna.

Alcance autorizado: reducir el umbral de confianza de normal de `0,70` a
`0,50` y ampliar la incidencia directa máxima de `30°` a `45°` en el flujo
Fase 6, manteniendo soporte local, textura, discontinuidades, rango y los
vetos de planificación. El umbral se expondrá de extremo a extremo como
parámetro de `orbslam3`; los defaults de launch Fase 6 aplicarán ambos valores
para las siguientes pruebas.

Ajuste depth completado. `DepthObservationParameters` y `StereoSlamNode`
exponen `depth_normal_min_confidence` con default propio 0,70 y validación
[0,1]. El launch Fase 6 lo propaga con default 0,50;
`phase6_depth_direct_surface_max_incidence_deg` queda en 45 grados. Se
conservan soporte, textura, discontinuidad, rango y planificación estricta.
Builds PASS: `orbslam3`, `dron_individual` y `simulacion_dron`, códigos 0.
El test `test_depth_observation_processor` PASS. No se ejecutó una simulación
posterior: `run_03` sigue siendo evidencia con umbrales 0,70/30 grados.
Trabajo activo: no.


## Repetición 8.5.6 con depth relajado

Trabajo activo: si, ejecución `c8_5_6_autonomia_integrada_run_04`.
Preparacion: CERRADA
Acuerdo cerrado: si
Autorizacion funcional: CONCEDIDA
Prueba acordada: repetir 8.5.6, dos drones y autonomía normal, con los defaults
Fase 6 `depth_normal_min_confidence=0,50` e incidencia máxima 45 grados.
Comparar depth normal válida, `DEPTH_OCCUPIED`, claims y coverage con `run_03`
(0,70/30 grados), sin reinterpretar su evidencia.
Dudas abiertas: ninguna.

Siguiente acción exacta: cerrar el daemon ROS, verificar lista vacía, iniciar el
capturador pasivo de `run_04` y ejecutar `run_simulation.sh` con el mismo
perfil/escenario 8.5.6 y GUI de Gazebo/multidron.

## Diagnóstico posterior de run_04

Trabajo activo: no. c8_5_6_autonomia_integrada_run_04 es PARCIAL y no modifica la conclusión CONSEGUIDA de run_03. El escenario marcó fallo de handoff por superar 10 s, pero task_server siguió vivo y posteriormente asignó ambas tareas. D1 perdió su único item POINT_SELECTION: fue consumido antes de F6H-FACADE-INITIALIZED, y RunPointSelectionWorker retornó al no tener runtime sin reencolarlo. Por eso quedó RUNNING y quieto. D2 validó los umbrales relajados: 8 depth, 3 reservas y claims DEPTH_OCCUPIED, pero sus transiciones incluyen la latencia serializada integración -> materialización -> recheck -> D* -> reserva. No hubo malloc ni FATAL; no quedan nodos. Corrección pendiente, que exige nuevo acuerdo funcional: reintentar/diferir POINT_SELECTION hasta runtime inicializado y hacer tolerante/reintentable el handoff del scenario runner.

## Corrección autorizada posterior a run_04

Trabajo activo: si.
Preparacion: CERRADA
Acuerdo cerrado: si
Autorizacion funcional: CONCEDIDA
Prueba acordada: corregir dos defectos sin alterar la política de vuelo: (1) si
POINT_SELECTION se consume antes de que exista el runtime de fachada, diferirlo
sin pérdida hasta que se inicialice; (2) el scenario runner reintentará el
handoff de cobertura ante respuesta tardía, antes de declararlo fallido. Compilar
`task_server` y `simulacion_dron`; repetir 8.5.6 con dos drones, autonomía
normal y los umbrales depth 0,50/45 grados. Éxito: handoff confirmado, ambos
drones reciben comandos y no hay `malloc` ni `FATAL`.
Dudas abiertas: ninguna.

Siguiente acción exacta: inspeccionar los timers de workers y el método
EnableAutonomousExecution, aplicar las correcciones y validar con builds.

Checkpoint de build: cambios aplicados en `task_server_node.cpp` y
`scenario_runner_node.cpp`. Se retrasa `POINT_SELECTION` hasta el `TaskReport`
aceptado, se inicializa/difiere el runtime sin pérdida y el handoff intenta tres
solicitudes idempotentes de 30 s. Siguiente acción: compilar `task_server` y
`simulacion_dron` con `build_selected_packages.sh`; después ejecutar el test
contractual y registrar resultado antes de simular.

Checkpoint de build: PASS. `task_server` compiló y encadenó correctamente al
build de `simulacion_dron`; `simulacion_dron` terminó con código 0. Solo persiste
el aviso conocido de ruta Drake inexistente. Documentación de ambos componentes
actualizada. Siguiente acción: ejecutar `pytest` contractual de simulación,
registrar su resultado y, si pasa, cerrar nodos ROS e iniciar run_05 de 8.5.6.

Prueba siguiente: `c8_5_6_autonomia_integrada_run_05`, perfil
`autonomous_gt_integrated_8_5_6.yaml`, `num_drones=2`, navegación GT durante
bootstrap, autonomía normal posterior, GUI Gazebo/multidrón y parámetros depth
`normal_min_confidence=0,50`, incidencia `45°`. Se mantendrán STOP, riesgo
visual, inflación adicional 1 y corredores FREE estrictos. Timeout wrapper y
capturador: 600 s; espera posterior al escenario: 420 s. El log se reducirá por
handoff/reintentos, F6B/F6K/F6C/F6D/F6F/F6H/F6I/F6J, `malloc` y `FATAL`.
Siguiente acción: comprobar `ros2 node list` vacío e iniciar capturador y wrapper.

Repetición solicitada: `run_05` completó el escenario y verificó handoff y
primeros ciclos de ambos drones, pero se interrumpió su ventana posterior para
reiniciar. `run_06` se interrumpió por petición del usuario antes de concluir;
ninguna se usará como evidencia final. Se confirmó ROS vacío tras cerrar el
capturador residual. Siguiente acción: iniciar `c8_5_6_autonomia_integrada_run_07`
con la misma configuración validada y capturador aislado en `run_07`.

Checkpoint de correccion: la evidencia de `run_05` muestra lotes del
`VoxelMapWorker` de hasta 55 transacciones y 75.507 celdas FREE en un unico
callback serializado; bloquearon durante segundos ingress, seleccion, depth y
despacho. Se implemento en `task_server` un presupuesto de una transaccion por
tick, prioridad para depth independiente y la metrica `pending/elapsed_ms` en
`F6G-VOXEL-DELTAS-APPLIED`; cada transaccion/keyframe sigue atomica y las de
un mismo keyframe no se adelantan. Se agrego una prueba de prioridad depth.
Siguiente accion: compilar `task_server`, ejecutar su test de evidencia y
repetir 8.5.6 con el mismo contrato de seguridad.

Checkpoint de build: `task_server` no alcanzo resultado de compilacion porque
colcon no pudo escribir `log/servidor/.../events.log` (`No space left on
device`). Siguiente accion: medir espacio y limpiar solo artefactos generados
de build/log permitidos; despues repetir el mismo build antes de ejecutar test
o simulacion.

Checkpoint de reconstruccion: `mission_msgs` reconstruido PASS, codigo 0. La
limpieza de `build/servidor` rompio enlaces symlink-install de dependencias,
por lo que se reconstruyen en orden `mission_msgs -> orbslam3_msgs -> task_lib
-> task_server`. Siguiente accion: compilar `orbslam3_msgs`.

Checkpoint de reconstruccion: `orbslam3_msgs` reconstruido PASS, codigo 0.
Siguiente accion: compilar `task_lib`, tercera y ultima dependencia directa
antes de repetir el build de `task_server`.

Checkpoint de reconstruccion: `task_lib` reconstruido PASS, codigo 0.
Dependencias directas restauradas. Siguiente accion: compilar `task_server`,
ejecutar `test_evidence_pipeline` y luego validar la latencia con una nueva
simulacion 8.5.6.

Checkpoint de build: `task_server` PASS, codigo 0. Avisos conocidos sin
impacto: API ROS deprecada para servicios y ruta Drake inexistente. La
correccion compilo. Siguiente accion: ejecutar CTest dirigido de
`test_evidence_pipeline`; si pasa, actualizar la documentacion de task_server
y preparar una repeticion integrada con ROS vacio.

Checkpoint de test: `test_evidence_pipeline` fallo solo en una expectativa
literal del nuevo test: el source_id sparse contiene tres sufijos de identidad,
por lo que el valor correcto es `sparse_ray_free:1:1:1`. La prioridad depth
paso. Correccion mecanica aplicada; siguiente accion: reconstruir `task_server`
y repetir el mismo CTest.

Checkpoint de build: `task_server` recompilado PASS, codigo 0, tras la
correccion mecanica del test. Siguiente accion: repetir CTest
`test_evidence_pipeline` y, si pasa, actualizar documentacion y lanzar la
repeticion integrada de latencia.

Checkpoint de test: `test_evidence_pipeline` PASS (8/8) y `git diff --check`
sin errores despues de corregir su expectativa. Documentacion de `task_server`
actualizada con presupuesto, prioridad y metrica de materializacion. Para
simular hay que restaurar cuatro paquetes cuyo symlink-install se perdio al
limpiar `build/servidor`: `orbslam3_multi`, `orbslam3_server`,
`multidron_gui_lib` y `multidron_gui`. Siguiente accion: reconstruirlos en
orden y entonces lanzar la repeticion integrada.

Checkpoint de reconstruccion: `orbslam3_multi` PASS, codigo 0. Siguiente
accion: compilar `orbslam3_server`, despues `multidron_gui_lib` y
`multidron_gui`, para restaurar el launch integrado tras la limpieza del build.

Checkpoint de reconstruccion: `orbslam3_server` PASS, codigo 0. Siguiente
accion: compilar `multidron_gui_lib` y despues `multidron_gui`; con ello el
launch integrado volvera a disponer de todos sus binarios instalados.

Checkpoint de reconstruccion: `multidron_gui_lib` PASS, codigo 0. Siguiente
accion: compilar `multidron_gui`, comprobar el install de los binarios
integrados y lanzar la prueba de latencia.

Checkpoint de reconstruccion: `multidron_gui` PASS, codigo 0. Todos los
paquetes que perdieron enlaces symlink-install al limpiar `build/servidor`
fueron restaurados: mission_msgs, orbslam3_msgs, task_lib, task_server,
orbslam3_multi, orbslam3_server, multidron_gui_lib y multidron_gui. Prueba
siguiente: `c8_5_6_autonomia_integrada_run_10`, misma configuracion que
run_09, dos drones y aproximacion GT lenta al fiducial. Criterio de correccion:
F6G debe procesar una transaccion por tick, depth no debe esperar tras rafagas
sparse ajenas y ambos drones deben encadenar comandos sin pausas de varios
segundos atribuibles al servidor.

Checkpoint de diagnostico `run_10`: INTERRUMPIDA por Codex, no es evidencia
funcional. Tras el anclaje, `scenario_runner` no envió los goals de separación
ni abrió el handoff porque `gate_mapping_backpressure=true` esperaba
indefinidamente `/global_mapping/backpressure_active=false`; la señal medida
seguía `true` durante el mapeo continuo. Las poses quedaron en `(0,-10,1)` y
`(0,-10,1.3)` y todas las tareas seguían PENDING. Es un bloqueo del bootstrap,
no una decisión del servidor autónomo. La corrección acotada fija
`gate_mapping_backpressure: false` exclusivamente en el escenario 8.5.6,
por lo que el bootstrap GT puede completarse sin modificar los vetos de la
fase autónoma.

La primera corrección de `task_server` sí quedó medida durante `run_10`: cada
`F6G-VOXEL-DELTAS-APPLIED` procesó `transactions=1`, con cola que drenó a cero
y `elapsed_ms` observado aproximadamente entre 9 y 82 ms. Sustituye los lotes
anteriores de hasta 55 transacciones/75.507 celdas FREE que bloqueaban durante
segundos el callback serializado. Pendiente: `run_11` debe atravesar ahora el
handoff y demostrar encadenamiento de comandos/depth para ambos drones.

Checkpoint `run_11`: PARCIAL e interrumpida para corregir. El bypass de
backpressure funciono: la separación GT se envió/completó en 12 s y el handoff
asignó D1/D2 de inmediato. D2 encadenó tres `LOOK_AND_CAPTURE`; tras la segunda
captura, sus fuentes se aplicaron en ~1,1 s y recibió otra orden ~1,6 s después.
D1 quedó asignado pero no despachó porque `RunPointSelectionWorker` rechazaba
un objetivo `FREE` no transitable por inflación y lo reencolaba directamente,
sin invocar el fallback `prefix_free` ya presente en el planificador.

Corrección aplicada: un objetivo `FREE` inflado conserva el contexto, marca
`fallback_free_advance`, entra en `TRAJECTORY_PLANNING` y deja
`F6K-POINT-SELECTION-FREE-INFLATED`; el planificador hallará el prefijo FREE
estricto más lejano y emitirá `VIEW_ADVANCE`. El caso OCCUPIED continúa
rechazándose y nunca se cruzan UNKNOWN/OCCUPIED/RESERVED. ROS se cerró y el
daemon quedó vacío. Siguiente acción: compilar `task_server` y repetir 8.5.6
para validar el primer comando de D1 y continuidad de ambos drones.

Checkpoint de build posterior: `task_server` PASS (42,7 s) con la corrección
FREE inflado -> `prefix_free`; `simulacion_dron` PASS (74 s) con el flag
`gate_mapping_backpressure=false` por defecto. Solo aparece el aviso conocido
de ruta Drake inexistente. Siguiente prueba: `run_12`, ROS limpio, dos drones,
misma 8.5.6; debe registrar `F6K-POINT-SELECTION-FREE-INFLATED` y un
`VIEW_ADVANCE`/movimiento de D1, además de continuidad para ambos.

Diagnóstico refinado `run_12`: D1 no estaba inactiva; su sección AB fue
asignada y su primer candidato llegó a `TRAJECTORY_PLANNING`, pero no tenía
ningún prefijo FREE transitable. El fallo adicional era que
`rejected_inspection_target_ratios` no participaba en
`SelectFacadeCoverageCandidate`, de modo que la misma sección/candidato se
seleccionaba una y otra vez. Corrección aplicada: `task_lib` acepta secciones
excluidas sin contabilizarlas como cobertura; task_server excluye la sección
solo cuando falla el prefijo, emite `F6I-PREFIX-FREE-UNAVAILABLE` y reselecciona
otro punto. Sigue vigente la prohibición de cruzar UNKNOWN/OCCUPIED/RESERVED.
Siguiente acción: compilar `task_lib` y `task_server`, ejecutar el test nuevo y
repetir la prueba integrada para observar movimiento de ambos drones.

Checkpoint de validación: `task_lib` PASS, `task_server` PASS y
`test_facade_coverage` PASS (7/7), incluido
`ExcludesBlockedSectionWithoutClaimingCoverage`. Siguiente prueba: `run_13`;
D1 debe registrar una exclusión de sección y no repetirla, y ambos drones deben
seguir recibiendo ciclos autónomos o tareas nuevas sin backpressure de runner.

## Corrección de prioridad depth por continuación

Trabajo activo: si. Preparacion: CERRADA. Acuerdo cerrado: si.
Autorizacion funcional: CONCEDIDA.

Se corrigió la extracción de transacciones de `EvidenceDatabase`: el worker ya
no prioriza genéricamente toda fuente `depth_*`. Recibe las `source_id` exactas
que figuran en `pending_depth_continuations_` y adelanta su transacción junto a
la cadena previa del mismo keyframe, sin saltar el orden de esa identidad. La
carga sparse ajena conserva el presupuesto normal de una transacción por tick.
Así una continuación de D1/D2 no puede quedar detrás de cientos de muestras
pasivas que también llevan el prefijo `depth_`. El marcador F6G expone ahora
`continuation_sources`.

Validación: build `task_server` PASS (45,3 s; solo avisos conocidos) y CTest
`test_evidence_pipeline` PASS (incluye prioridad exacta y cadena causal).
Siguiente acción: repetir 8.5.6 con ROS vacío; confirmar que cada
`F6F-DEPTH-SOURCES-WRITTEN` propio se aplica en el siguiente tick o tras una
cadena corta del mismo keyframe, y que ambos drones encadenan comandos.

## Diagnóstico de flush de navegación

`run_14` confirmó que la prioridad por `source_id` funciona cuando la pose del
keyframe ya está disponible: D2 materializó un `VIEW_ADVANCE` en 0,34 s. Aun
así, su primera fuente y la primera de D1 esperaron 2,84 s y 10,64 s porque el
worker hacía `FlushVoxelChanges` tras cada transacción pasiva: cada flush
recalculaba navegación e inflación sobre decenas de miles de celdas, retrasando
las respuestas/push de `GetGlobalKeyFramePose` que vuelven materializable la
fuente depth. La cola llegó a más de 900 transacciones. `run_14` fue
interrumpida y no es evidencia final.

Corrección en curso: mantener la materialización individual y la prioridad de
continuación, pero agrupar el refresh/publicación de navegación de fondo cada
500 ms. Cuando se materializa una `source_id` esperada por D1/D2, forzar flush
inmediato, de modo que coverage, reservas y STOP conservan su visibilidad y
seguridad. Build `task_server` PASS (43,4 s; solo avisos conocidos) y CTest `test_evidence_pipeline` PASS. Siguiente acción: repetir 8.5.6.

## Cierre de continuidad depth 8.5.6

Trabajo activo: no. Preparacion: CERRADA. Acuerdo cerrado: si.
Autorizacion funcional: CONCEDIDA y consumida. Dudas abiertas: ninguna.

`run_15` validó la corrección de rendimiento: prioridad por `source_id` de
continuación más batching de `RefreshNavigation` de fondo a 500 ms y flush
inmediato al materializar la fuente esperada. D1 alcanzó el comando 13 y D2 el
15, con planes, reservas, depth y claims en ambos; D1 aplicó su primera fuente
en 0,17 s. El primer ciclo de D2 esperó pose global varios segundos, pero no
quedó bloqueado. Capturador: 46 TaskState, 317 VoxelMap, 1035 NavigationState,
revisión 2287 y 469 reservas máximas. No hubo `malloc`/`FATAL`; ROS quedó vacío.
Build `task_server` PASS, CTest `test_evidence_pipeline` PASS. La conclusión
primaria integrada de 8.5.6 se mantiene en `run_03`; `run_15` es la validación
vigente de continuidad bajo carga.

## Repetición visual run_16

Trabajo activo: no. Preparacion: CERRADA. Acuerdo cerrado: si.
Autorizacion funcional: CONCEDIDA y consumida. Dudas abiertas: ninguna.

`run_16` reprodujo exactamente la configuración validada de `run_15` con GUI
de Gazebo y GUI multidrón para vídeo y fotos. El usuario confirmó que la toma
fue correcta y pidió su cierre. Antes de la interrupción ordenada, D1 aceptó
comandos hasta `...:3:15` y D2 hasta `...:3:20`, con depth, fuentes aplicadas,
reservas D* y avances en ambos. No aparecieron `malloc` ni `FATAL`. El grupo de
simulación se cerró y `ros2 node list` quedó vacío. Es confirmación visual, no
sustituye la evidencia técnica primaria de `run_03` ni la validación de
continuidad de `run_15`.
