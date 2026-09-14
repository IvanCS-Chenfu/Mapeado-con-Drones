# Historial 6H

## 2026-09-10 - Coverage superficial inicial

- objetivo intentado: filtrar score sparse, calcular coverage superficial
  accesible reversible y publicarlo en tarjetas F7.
- archivos modificados: `task_lib`, `task_server`, replicas `mission_msgs`,
  `multidron_gui_lib` y escenario 633.
- builds: `mission_msgs` Servidor/Dron, `task_lib`, `task_server`,
  `task_manager`, `multidron_gui_lib` y `multidron_gui`, todos correctos.
- CTest: `task_lib` 8/8, `task_server` 7/7 y GUI 9/9 correctos.
- prueba Gazebo: 633, GT, una GUI F7, sin RViz ni GUI legacy; ambos goals al
  fiducial 2 terminaron correctamente.
- evidencia: `F6H-COVERAGE` paso por `0.000`, `0.019`, `0.000`, `0.167` y
  `0.103`, demostrando actualizacion y regresion sobre mapa real.
- limitacion: no hay aun creacion/desbloqueo de ramas ni solicitud automática
  del candidato a D*.
- conclusion: PARCIAL.

## 2026-09-10 - Portales exclusivos y candidato D* automatico

- objetivo intentado: completar el tramo acordado de lineage: clasificar
  portales exclusivos, materializar ramas bloqueadas y enviar el mejor
  candidato de coverage al planificador sin ordenar vuelo.
- archivos modificados: `task_lib` incorpora `BranchPortalAnalyzer` y su test;
  `mission_msgs/TaskState` replica lineage, portales y resolución; `task_server`
  materializa y reconcilia `BRANCH_ENTRY`, y publica rutas candidatas por
  `/mission/planned_routes`.
- builds y tests: `mission_msgs` Servidor/Dron, `task_lib`, `task_server`,
  `task_manager`, `multidron_gui_lib` y `multidron_gui` correctos. CTest:
  `task_lib` 9/9, `task_server` 7/7 y GUI 9/9.
- pruebas previas: 634 no valida porque se movió antes del anclaje; 635 no
  conseguida al usar por error `(-10, 0, Z)` como fiducial 2. Ambas se
  conservan como intentos, sin reutilizarse como evidencia funcional.
- prueba Gazebo valida: 636, GT, Gazebo y una única GUI F7, sin RViz ni GUI
  legacy. Ambos drones llegaron primero a `(0, -10, Z)`, esperaron anclaje y
  realizaron después un desplazamiento corto local; `SIM-DONE success=true`.
- evidencia: coverage para las tareas AB y BC, cambios de voxel hasta revisión
  187 y rutas D* visibles en GUI. Ejemplos: `dstar_1_3` en 3.493 ms,
  `dstar_2_4` en 3.024 ms y `dstar_1_10` en 2.714 ms; la GUI recibió
  `GUI-PLANNED-ROUTE-UPDATE` con 2-3 waypoints. Los rechazos de candidatos
  ocupados o inflados se informaron explícitamente y no produjeron vuelo.
- limitación: la geometría de 636 no cruzó un portal exclusivo. La creación,
  desbloqueo y sustitución de ramas están cubiertas sintéticamente, pero falta
  evidencia integrada de su lifecycle completo. El modelo de desbloqueo actual
  usa un perímetro local como aproximación, no una reconstrucción volumétrica
  completa de la rama.
- conclusión: PARCIAL.

## 2026-09-10 - Alcance de ramas en la casa exterior

- decisión: no crear una simulación adicional ni forzar movimientos para
  fabricar ramas. La escena vigente muestra una casa que se mapea desde fuera.
- interpretación: el interior `UNKNOWN` separado por una barrera `OCCUPIED`
  es inaccesible desde el componente actual; no es una rama ni debe impedir el
  cierre del comportamiento exterior validado.
- alcance futuro: las ramas siguen siendo una capacidad condicional para una
  expansión conectada real, aplicable a pasillos, cuevas, patios, edificios
  abiertos o recintos conectados. No se presupone que todos los entornos sean
  edificios cerrados.
- consecuencia: 636 conserva su valor para coverage-proxy y candidato D*, pero
  no se interpreta como prueba de `BLOCKED_BRANCH`, desbloqueo o `SUPERSEDED`.
- conclusión: PARCIAL por lifecycle de rama aplazado, no por un fallo de la
  simulación exterior.

## 2026-09-10 - Metas de exploracion exclusivamente UNKNOWN

- objetivo intentado: corregir la repeticion de paredes de la prueba extendida
  649, evitando que el lado FREE de un portal pueda ser destino de exploracion.
- archivos modificados: `task_server_node.cpp`, contrato 6H/6I/6G/6D, resumen
  de `task_server` y escenario `tray_prueba_650.yaml`.
- implementacion: `PlanCoverageCandidate` conserva el lado FREE como prueba de
  entrada alcanzable, pero propone solo `unknown_key`. Registra `novelty_m` y
  prefiere la separacion configurable de 5 m; si no existe, el mas separado
  disponible antes de ganancia, viaje y standoff.
- build y CTest: `task_server` compilo correctamente; CTest final 7/7. Un
  primer CTest solo fallo por sangria uncrustify y se corrigio sin alterar la
  funcion.
- prueba Gazebo: 650, GT, Gazebo y GUI F7 unica, sin RViz ni GUI legacy;
  anclaje en fiducial 2 `(0,-10,Z)`, D1 automatico y D2 hover. El primer
  intento de Gazebo murio durante startup y el helper relanzo el intento
  recuperado, que finalizo con `SIM-DONE success=true`.
- evidencia: las 339 selecciones registradas son `source=unknown_portal`, sin
  ninguna meta `free_portal`. D1 despacho, entre otros, `dstar_1_1`,
  `dstar_1_5`, `dstar_1_14`, `dstar_1_28` y `dstar_1_38`.
- limitacion: D* descarta muchos portales por `goal_occupied_or_inflated`; el
  proxy de progreso no es coverage volumetrico. La GUI estuvo activa, pero no
  hubo nueva valoracion visual humana antes del cierre automatico.
- conclusion: PARCIAL. Se valida la regla de destino UNKNOWN y el fallback de
  novedad; queda optimizar portales inflados y construir coverage volumetrico.

## 2026-09-11 - Coverage volumetrico y compuerta de ejecucion

- implementacion: se reemplazo el proxy superficial por una rejilla implicita
  volumetrica 6-conexa. `FREE` y frontera `OCCUPIED` resuelven el componente
  alcanzable; UNKNOWN aislado por barrera cerrada queda fuera reversiblemente.
  Las metas son UNKNOWN estrictos, con banda lejana de 5 m, completion 0.99,
  analisis cada 15 s y `BLOCKED` ante inconsistencia.
- verificacion: `task_lib` 9/9, `task_server` 7/7, GUI 9/9 y contratos de
  simulacion 3/3 correctos; `git diff --check` correcto.
- pruebas 655--657: 655 es invalida por YAML relativo. 656/657 llegan al
  anclaje GT, pero los callbacks seriales de nube/mapa retrasan el servicio de
  compuerta mas de 60 s. En 657, tras vencer el cliente, se registra gate,
  coverage y una meta UNKNOWN a 5 m; D* devuelve `no_safe_escape` en 0.450 ms.
- conclusion: PARCIAL. La politica volumetrica esta implementada; falta aislar
  la compuerta del backlog de VoxelMapWorker para validar el flujo integrado.

## 2026-09-11 - Compuerta concurrente de VoxelMapWorker

- correccion: `task_server` separa grupos de callbacks de mapa/control con dos
  hilos. Solo mapa escribe voxel, navegacion, tareas y action; control responde
  la compuerta y deja una intencion atomica aplicada posteriormente por mapa.
- pruebas: 658 detecto que las respuestas asíncronas de poses globales seguian
  en el grupo por defecto; se movieron al grupo mapa. Build posterior correcto
  y CTest `task_server` 7/7.
- prueba 659: `SIM-DONE success=true`. La compuerta respondio en unos 4 ms,
  mientras el lote de 934 cambios tardo 34.141 s. Coverage eligio UNKNOWN a
  5 m y D* rechazo `no_safe_escape` en 0.286 ms.
- conclusion: CONSEGUIDA para aislamiento de control. Coverage/lifecycle de
  rama siguen parciales por su alcance propio.

## 2026-09-11 - Reintento recuperable de no_safe_escape

- objetivo intentado: evitar que la ausencia temporal de FREE local convierta
  una tarea de coverage en `BLOCKED` o libere el dron.
- implementacion: `SafeEscapeRetry` conserva target UNKNOWN, banda y deadline;
  `coverage_no_safe_escape_retry_sec=15` no añade ese motivo a
  `rejected_targets`. Si el target cambia de estado, se limpia la espera y se
  vuelve al selector normal.
- builds y tests: `task_server` correcto; CTest 7/7. `simulacion_dron`
  correcto y contratos `pipeline_flow`, `mission_flow` y
  `system_architecture`, 3/3.
- prueba Gazebo: 660, GT, fiducial 2, Gazebo y GUI F7 única. Finalizó con
  `SIM-DONE success=true` y sin procesos residuales.
- evidencia: D1 recibe `no_safe_escape` para `16:-34:5` a
  `1789084972.708752`, conserva la espera y reintenta exactamente el mismo
  target a `1789085016.896440`. No aparece `F6H-COVERAGE-BLOCKED`. Cuando la
  evidencia invalida la meta, `F6H-CANDIDATE-RETRY-CLEAR` permite escoger otra;
  otro `no_safe_escape` vuelve a esperar sin bloquear.
- limitacion medida: el deadline es mínimo, no una garantía de wall-clock. El
  primer reintento llegó 44.188 s después porque el grupo mapa estaba ocupado;
  la planificación conserva serialización segura con VoxelMap. Resolver esa
  puntualidad exigiría dividir los lotes largos del worker, fuera de este
  cambio.
- conclusion: CONSEGUIDA para la semántica recuperable; PARCIAL solo para la
  puntualidad exacta de 15 s bajo backlog de mapa.

## 2026-09-11 - Intento prolongado de movimiento automático D2

- objetivo intentado: confirmar una ruta D* aceptada, su ejecución física y
  crecimiento de coverage tras anclar D2 con GT en fiducial 2.
- prueba Gazebo: 661, una GUI F7, D2 único ejecutor y ventana de 220 s tras el
  gate. `SIM-DONE success=true`, sin procesos residuales.
- evidencia: BC pasó de `0.002` (91/36864) a `0.003` (126/36863) y generó
  FREE por KF/voxel. El target `18:-48:6` recibió `no_safe_escape` repetido;
  nunca aparece `F6H-CANDIDATE-SELECT` para D2.
- interpretación: no se valida vuelo autónomo. La política de espera funciona,
  pero el escape solo admite FREE confirmado y el volumen físico libre actual
  no satisface el perfil con clearance de 1 m alrededor del dron.
- conclusión: NO CONSEGUIDA para movimiento/coverage autónomo; requiere un
  acuerdo sobre la evidencia o modelo de escape, sin reinterpretar como fallo
  de D* ni de la compuerta.

## 2026-09-11 - Meta UNKNOWN a 2 m y ejecución interrumpida

- cambio: `coverage_unknown_goal_min_distance_m` queda en 2.0 tanto en launch
  como en el default de `task_server`; el destino sigue siendo UNKNOWN estricto.
- prueba 665: D1 llegó con GT al fiducial 2; el selector eligió UNKNOWN de
  2.00--2.01 m. El coverage de AB pasó de 0.015 (553/36854) a 0.027
  (1005/36843) durante la ventana observada.
- cierre: el usuario solicitó detener Gazebo mientras seguía activo. La prueba
  se conserva como evidencia parcial; no valida exploración sostenida,
  porcentaje final ni cierre de tarea.
