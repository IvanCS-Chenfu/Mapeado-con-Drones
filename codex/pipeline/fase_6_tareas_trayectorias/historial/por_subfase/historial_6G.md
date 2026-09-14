# Historial - 6G

## 2026-09-08 - Planificador D* Lite y contrato de plan previsto

- Se incorporo `DStarLitePlanner` a `task_lib`, con estado incremental por
  dron, 26 vecinos, coste diagonal, `UNKNOWN` penalizado, inflacion, diagonales
  seguras y simplificacion LOS.
- `ReversibleVoxelMap` expone `VoxelChange` y `TakeChanges()` para reparar sin
  reconstruir el snapshot completo.
- `task_server` conserva una instancia por dron, atiende `/mission/plan_route`
  y publica `/mission/planned_routes` como `TrajectoryPlan` transient-local.
  Esta salida sigue siendo una ruta XYZ prevista, no una orden al ejecutor.
- El grafo web incorpora `PlanningWorker`, `PlannedRoute` y los eventos
  `DSTAR_PLAN`, `DSTAR_REPLAN` y `DSTAR_REJECT` con motivo, revision, latencia,
  expansiones e ID correlable.
- Se limito la busqueda a una ventana voxelizada inicio-objetivo dentro del
  volumen duro y se rechazo el objetivo fuera de horizonte. Corrige la prueba
  613, en la que explorar todo el volumen agotaba el presupuesto de computo.
- La correccion de jitter subvoxel estabilizo la ventana de busqueda. La prueba
  614 conserva que el segundo plan sustituyo el visual, pero no se clasifico
  como repair por ese jitter.

## Validacion

- Builds de `mission_msgs`, `task_lib`, `task_server`, `multidron_gui_lib`,
  `multidron_gui` y la replica Dron de `mission_msgs`: correctos.
- CTest dirigido: `task_lib` 7/7, `task_server` 7/7 y GUI 9/9 correctos. El
  GTest D* incluye muro con unico hueco, pasillo bloqueado y retirada de
  obstaculo, ademas de ruta UNKNOWN, reparacion, diagonal y horizonte.
- Prueba 615: Gazebo con fuente GT, una sola GUI F7 y sin RViz2. Plan inicial
  `dstar_1_1`: `expanded=607`; replan incremental `dstar_1_2`:
  `expanded=0`. Ambos tuvieron dos waypoints, revision de mapa 0 y fueron
  recibidos como `Plan previsto` por la GUI. El cierre SIGINT posterior a
  `SIM-DONE success=true` explica el exit 255 de Gazebo.
- Prueba 616: Gazebo/GUI arrancaron y cerraron limpios, guarda de recursos
  inactiva, pero el escenario solo esperaba. No se emitieron solicitudes porque
  la autorizacion externa para el diagnostico ROS fue rechazada; no se usa como
  evidencia de plan/replan ni para ocultar la prueba 615.

Conclusion: CONSEGUIDA para D* Lite incremental y waypoints XYZ previstos. La
trayectoria continua, ejecucion, yaw/pitch, reservas, frontiers y avoidance
entre drones permanecen fuera de 6G.

## 2026-09-08 - Reapertura de distancia y prueba con voxeles ORB

- Se retiro `planning_max_distance_m`: D* solo limita por volumen duro y
  presupuesto de expansiones; 6I limitara los tramos ejecutables. Un GTest
  valida una ruta de 20 m.
- Prueba 617: la GUI materializo hasta 5.221 MapPoints y 2.006 voxeles, pero
  el plan del dron 1 a `(-9,-4,1.0)` se rechazo. El segundo no respondió antes
  del cierre de la ventana.
- Prueba 618 repitio el mapa (hasta 5.384 MapPoints y 2.562 voxeles) y el
  diagnostico preciso confirmo `start_occupied_or_inflated`, con cero
  expansiones. No se considera un fallo de distancia ni del objetivo.

Pendiente funcional: decidir si la futura evidencia `FREE` de 6I debe ser el
requisito para arrancar D* desde una zona ocupada por sparse, o si se autoriza
una salida local controlada con otra regla de seguridad.

## 2026-09-08 - Escape local sobre FREE confirmado

- Acuerdo funcional: el volumen `FREE` representa el cuerpo fisico recorrido,
  sin margen. D* conserva el margen y solo admite un escape desde un inicio
  inflado si este pertenece a `FREE` confirmado.
- `DStarLitePlanner` realiza una busqueda local por celdas FREE hasta el primer
  voxel con clearance completo. No relaja la inflacion ni atraviesa OCCUPIED.
  Informa `no_safe_escape` cuando no hay salida y
  `unreachable_current_map` cuando no existe ruta al objetivo actual.
- Los GTests de `task_lib` validan tanto escape permitido como rechazo sin
  componente FREE de salida. Build y CTest finales: `task_lib` 7/7 y
  `task_server` 7/7.
- Prueba 620: tras observar MapPoints/OCCUPIED/FREE frente al fiducial 2, el
  plan de D1 a `(-9,-4,1.0)` fue rechazado como
  `goal_occupied_or_inflated` (`map_revision=255`, cero expansiones). Esto
  preserva la seguridad del objetivo exacto pedido. La llamada de D2 no
  respondio antes de finalizar la ventana y fue interrumpida controladamente.

Conclusion vigente: el escape y sus motivos estan implementados y testeados,
pero la demostracion integrada de un desvio D* visual queda PARCIAL: requiere
un objetivo con clearance o acotar la respuesta en el mapa denso, mediante un
nuevo acuerdo de prueba.

## 2026-09-08 - Prueba 621 de objetivo cercano

- Se preparo una seleccion manual de banco de pruebas: explorar candidatos
  cercanos a `(-9,-4,1.0)` y conservar el primer `PlanRoute` aceptado para la
  GUI F7. Esta responsabilidad no se incorpora a D*: en producto corresponde
  al planificador de tareas/vista de 6H/6M.
- Gazebo y GUI F7 arrancaron con GT y materializaron evidencia FREE/KF. La
  primera consulta a `(-8.5,-4,1.0)` no devolvio antes de la interrupcion
  controlada. El reducido no contiene marcador de entrada ni salida del
  servicio `PlanRoute`, asi que no permite atribuirlo al algoritmo.

Conclusion: PARCIAL. La busqueda de un objetivo cercano no llego a ejecutar un
plan; la siguiente repeticion debe comprobar primero el descubrimiento del
servicio ROS y aplicar un timeout forzado al cliente.

## 2026-09-08 - Prueba 622 con servicio verificado

- La comprobacion previa mostro `/mission/plan_route` en el grafo ROS desde el
  mismo entorno cliente de la simulacion.
- D1 envio correctamente `PlanRoute` para `(-8.5,-4,1.0)`. El cliente no
  recibio respuesta antes de un timeout forzado de 35 s. El reducido conserva
  actividad `F6I-FREE-KF`, pero no un `F6G-PLAN` ni un `DSTAR_REJECT`.
- El helper se interrumpio para liberar la ejecucion; su limpieza termino con
  `SIM-EXIT-CODE 0`. No se evaluaron mas candidatos, pues la misma solicitud
  seguia sin terminar.

Conclusion: PARCIAL. La comunicacion ROS y el envio de la solicitud estan
validados; el planificador necesita un presupuesto temporal/expansiones que
devuelva resultado o rechazo acotado antes de usarlo para seleccionar puntos
de observacion en un mapa denso.

## 2026-09-08 - Prueba 623: calculo D* confirmado

- Se anadio el marcador diagnostico `F6G-PLAN-START`, sin modificar coste,
  margen, distancia ni seguridad. Build de `task_server` correcto y CTest 7/7.
- La prueba 623 recibio la solicitud de D1 a `(-8.5,-4,1.0)` y registro la
  entrada efectiva al callback con `map_revision=263`. No produjo
  `F6G-PLAN`, `DSTAR_REJECT` ni respuesta antes de que el escenario terminara
  unos 20 s despues.
- Esto descarta descubrimiento ROS y cola del executor: el tiempo se consume
  dentro de D* sobre el mapa denso. La limpieza del launch termino con
  `SIM-EXIT-CODE 0` tras la interrupcion controlada.

Conclusion: PARCIAL. Se debe anadir un presupuesto de tiempo monotono al
planner, que devuelva `planning_time_budget_exhausted` con expansiones medidas.
No limita distancia ni relaja seguridad; permite que el selector futuro de
viewpoints pruebe un candidato cercano alternativo.

## 2026-09-09 - Snapshot navegable y medicion 624/625

- El acuerdo sustituye la propuesta de presupuesto temporal por optimizar antes
  la entrada de D*: `VoxelMapWorker` deriva por perfil las celdas navegables y
  `PlanningWorker` solo recibe snapshot/deltas. `g/rhs` no se comparten porque
  siguen dependiendo del objetivo; las reservas dinamicas quedan en interfaz
  hasta 6J.
- La 624 demostro que el cuello estaba antes de D*: su `F6D-NAV-UPDATE` tardo
  16.060 ms para 24 cambios raw y 1.056 celdas, dejando la solicitud de ruta en
  espera. La correccion calcula transitabilidad una vez por celda y compone
  despues las aristas con valores cacheados.
- Build correcto y CTest: `task_lib` 7/7, `task_server` 7/7. La 625, con
  Gazebo, GT y una sola GUI F7, bajo el mismo perfil `x500_depth`, registro
  110.169 ms para 24 cambios/1.122 celdas; el lote mayor fue 2.633 s para 988
  cambios/34.703 celdas.
- La solicitud real de D1 a `(-8.5,-4,1.0)` devolvio al cliente en 2.35 s. El
  servidor emitio `F6G-PLAN-START` y rechazo correctamente
  `goal_occupied_or_inflated` con `expanded=0`: no inicio una busqueda D* para
  un destino inseguro. La simulacion se cerro de forma controlada y sin nodos
  residuales.

Conclusion vigente: la optimizacion de la entrada incremental de D* esta
CONSEGUIDA y conserva la seguridad. Falta una prueba visual de ruta/desvio con
un objetivo que el selector futuro haya verificado con clearance; no se anade
todavia presupuesto temporal.

## 2026-09-09 - Intentos visuales 626 y 627

- La 626 exploro ocho candidatos rechazados inmediatamente como ocupados o
  inflados. `(-2,-8,4)` fue el primer destino aceptado: cinco waypoints,
  corredor de 23 voxeles, 50.120 expansiones y 5.294 s. El plan acabo durante
  el cierre posterior de la ventana GUI, asi que no se acepta como evidencia
  visual aunque la ruta calculada sea valida.
- La 627 repitio con ventana ampliada y el destino solicitado por el usuario,
  `(-10,5,2)`. Este no se clasifico como ocupado: D* inicio busqueda en
  `map_revision=275` y devolvio `expansion_budget_exhausted` tras 250.001
  expansiones y 28.238 s. No se modificaron limite, margen ni algoritmo sin
  un nuevo acuerdo funcional. El cierre controlado termino sin procesos
  residuales.

Conclusion: no hay aun una observacion GUI aceptable de ruta en el entorno
remoto `(-10,5,2)`. Debe acordarse ampliar el presupuesto de expansiones para
esa prueba o elegir un destino seguro de menor coste.

## 2026-09-09 - Guia jerarquica, cola instrumentada y prueba 628/629

- Se instrumentaron `queue_pops`, `stale_queue_pops` y expansiones reales. Las
  entradas obsoletas se invalidan por generacion antes de consumir el limite de
  expansiones.
- `DStarLitePlanner` usa `heuristic_weight=1.2` y una guia de macro-voxeles por
  perfil. El intento fino sigue un corredor inicial de 8 voxeles, ampliable de
  4 en 4 hasta la ventana completa; la guia es asesora y la seguridad sigue en
  el mapa fino. La salida local FREE no depende de esa guia.
- `task_server` propaga deltas finos/gruesos y registra los nuevos parametros y
  metricas en `F6D-NAV-UPDATE`, `F6G-PLAN` y `F6G-PLAN-REJECT`. Los grafos web
  conservan solo eventos esenciales y GUI F7 solo recibe la ruta final.
- Build de `task_lib` y `task_server` correctos; CTest finales 7/7 y 7/7.
  Se anadieron aserciones de macro-delta, factor grueso y contadores de cola.
- Prueba 628: `scenario_runner_node` no pudo abrir el YAML por ruta relativa;
  no inicio ningun goal ni PlanRoute y termino con `SIM-EXIT-CODE 1`.
- Prueba 629: GT, Gazebo y una unica GUI F7. D1 acepto `dstar_1_1` hacia
  `(-10,5,2)` en revision 218, con 10 waypoints y corredor de 71 voxeles.
  `F6G-PLAN` midio 8.569 expansiones reales, 90.011 pops, 81.442 obsoletos,
  factor 4, corredor 8 sin ampliaciones y 11.811794 s. Gazebo/GUI siguieron
  activos 64 s desde `F6G-PLAN-START`, superando los 50 s acordados, y el
  cierre posterior no dejo procesos residuales.

Conclusion: CONSEGUIDA. La ruta visual remota queda demostrada y la mejora es
clara frente a 627 (250.001 expansiones y 28.238 s), pero el alto numero de
pops obsoletos identifica una optimizacion futura: cola indexada/decrease-key.
No forma parte de este acuerdo ni modifica aun el comportamiento del planner.

## 2026-09-09 - Heap indexado y comparativa 630/631

- La cola de D* se sustituyo por un heap binario indexado por `VoxelKey`: cada
  estado mantiene una unica entrada activa, su prioridad se actualiza en sitio
  y se elimina cuando `g == rhs`. La politica de costes, guia, seguridad y
  eventos/GUI no cambio.
- Build de `task_lib` y `task_server` correctos; CTest completos 7/7 y 7/7.
  El GTest de guia exige ahora cero `stale_queue_pops` en un plan inicial.
- La 630 acepto una ruta, pero el cliente heredo `errexit` y no realizo la
  espera de 50 s ni preservo el marcador `F6G-PLAN`; no se usa como medicion.
- La 631 repitio GT, Gazebo, GUI F7 y `(-10,5,2)` con espera garantizada. D1
  acepto seis waypoints: `expanded=11.480`, `queue_pops=11.480`,
  `stale_queue_pops=0`, factor 4, corredor 8 sin ampliacion y 17.236664 s.
  GUI recibio el plan; la simulacion se mantuvo mas de 50 s desde
  `F6G-PLAN-START` y cerro sin procesos residuales.

Conclusion: CONSEGUIDA para eliminar duplicados de cola. No se afirma mejora
temporal frente a 629: esa ejecucion tenia otro mapa ORB, 8.569 expansiones y
corredor 71, mientras 631 tuvo 11.480 y 88. Para una comparacion de tiempo
estricta hace falta un snapshot voxel fijo o replay determinista.
