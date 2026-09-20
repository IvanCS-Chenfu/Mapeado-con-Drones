# `scenario_runner_node.cpp`

## Rol

El parametro `mission_profile` lee el mismo perfil que el launch y obtiene de
`trajectory_file` el YAML que debe ejecutar. La ruta puede ser absoluta o
relativa al perfil. Si contiene `mission_mode: autonomous`, el runner llama automaticamente a
`/mission/set_coverage_execution_enabled` con `true` despues de completar todos
los pasos de esa trayectoria. En `trajectory` no realiza ese handoff. La
transferencia se produce por finalizacion del ultimo goal, sin exigir una
deteccion de fiducial.

Ejecuta escenarios YAML y envía lotes de goals a
`/dron_X/AccionTrayectoria`. Desde 3C su gate de backpressure está activo. En
4B incorpora el paso generico `wait_for_bool` para esperar readiness externo
sin acoplar el runner al spawner.

Cada goal `move` admite `navigation_source: GT|ORB` sin distinguir mayusculas.
Antes de enviar la action, el runner llama al servicio namespaced
correspondiente. Si el campo falta, hereda `navigation_source` del perfil; un
goal explicito conserva precedencia para cambiar GT/ORB. Los
marcadores `[SCENARIO-RUNNER-NAV-SOURCE]` y
`[SCENARIO-RUNNER-NAV-SOURCE-ERROR]` hacen observable la preparacion.

El paso `wait_for_navigation_pose` valida llegada mediante
`/<drone>/orbslam/navigation_state`. Exige pose local, continuidad y velocidad
validas, tolerancias XYZ/yaw y permanencia `hold_sec`. Recibe `drone_id`,
`target`, `yaw_deg`, `position_tolerance_m`, `yaw_tolerance_deg`, `hold_sec` y
`timeout_sec`; emite marcadores `POSE-GATE-WAIT/DONE/TIMEOUT/ERROR`.

El paso `plan_route` solicita `/mission/plan_route` con `drone_id`, `task_id`,
`target: [x,y,z]`, `dispatch_execution` y `timeout_sec`. Cuando el ultimo vale
`true`, sirve para pruebas de integración: el servidor genera y ejecuta la
ruta mediante su runtime normal, no mediante un goal directo a
`AccionTrayectoria`. Sus marcadores son `PLAN-DONE`, `PLAN-REJECT` y
`PLAN-TIMEOUT`.

Cada invocacion emite una sola solicitud ROS y espera el mismo futuro hasta su
timeout; no reenvia una ruta si D* tarda mas de un segundo en responder. El
unico reintento permitido es el rechazo transitorio literal por pose canonica
no autorizada del paso relativo. `plan_route_relative_until_visual_risk` lee la
pose global autorizada despues de cada terminal, suma su offset de 2 m y
continua desde esa pose real. Se detiene con exito solo despues de la
reorientacion visual local reportada por el dron o falla al agotar el limite
explicito de pasos.

`wait_for_plan_terminal` consume el ultimo `trajectory_id` despachado por
dron desde `/mission/planned_routes` con QoS reliable + transient-local. Espera
su terminal normal; si la ruta original termina por `visual_risk` o por el
marcador canonico `TRACKING_RISK`, observa `VisualRiskEvent` y espera el
terminal local `REORIENTATION_COMPLETED` correlacionado con la trayectoria de
origen antes de continuar. No espera un plan `visual_risk_reorient`, que ya no
existe en el servidor. Un
STOP no visual en curso (`STOP solicitado` o `replaced_by_stop`) no es aun
terminal: espera el mensaje posterior `STOP completado`, que confirma que el
dron ya esta estable. Un terminal no visual distinto se declara fallo del
escenario en vez de lanzar el siguiente paso sobre una trayectoria aun activa.
Sus marcadores son
`EXECUTION-WAIT`, `EXECUTION-DONE`, `REORIENT-DONE`,
`STOP-DONE`, `STOP-FAILED`, `EXECUTION-STOP-NONVISUAL`,
`REORIENT-FAILED` y `EXECUTION-TIMEOUT`.

```text
simulacion_dron/src/control_tray/scenario_runner_node.cpp
rg -n "navigation_source|PrepareNavigationSource|plan_route|wait_for_plan_terminal|PLAN-DONE|EXECUTION-WAIT|wait_for_navigation_pose|POSE-GATE|wait_for_bool|READY-WAIT|mapping_backpressure|MOVE-GATE-WAIT" simulacion/simulacion_dron/src/control_tray/scenario_runner_node.cpp
```

`wait_for_bool` recibe `topic`, `expected` y `timeout_sec`. Crea una
suscripcion reliable + transient-local, por lo que recibe el ultimo estado
aunque el publicador lo haya emitido antes de arrancar el escenario. Marcadores:

```text
[SCENARIO-RUNNER-READY-WAIT]
[SCENARIO-RUNNER-READY]
[SCENARIO-RUNNER-READY-TIMEOUT]
```

Fase 5B añade `expect_rejected` por goal. Un rechazo esperado se registra con
`[SCENARIO-RUNNER-GOAL-REJECTED-EXPECTED]` y permite continuar; una aceptación
inesperada cancela/falla el escenario. Funciona en envío secuencial y
simultáneo y permite validar que un absoluto sin global sea rechazado antes de
los goals relativos.

Topic:

```text
/global_mapping/backpressure_active
std_msgs/msg/Bool
QoS reliable + transient_local
```

El callback actualiza un flag atómico. Un lote ya enviado termina normalmente;
los pasos `wait` no se bloquean; antes del siguiente lote `move`, el runner
espera a `false` y envía una sola vez los destinos originales. La espera del
gate ocurre antes de crear los goals y no consume su timeout.

`gate_mapping_backpressure` es `true` por defecto, tanto como parámetro ROS
como cuando falta en el YAML. Un escenario puede declararlo `false` para una
toma visual no formal: el runner no retiene los siguientes goals, pero el
servidor conserva la detección, publicación y telemetría de backpressure. El
bypass queda explícito en `[SCENARIO-RUNNER-MOVE-GATE-BYPASS]`.

Marcadores:

```text
[SCENARIO-RUNNER-BACKPRESSURE]
[SCENARIO-RUNNER-MOVE-GATE-WAIT]
[SCENARIO-RUNNER-MOVE-GATE-CLEAR]
```

En prueba 85 el gate esperó 67.956 s y se liberó al bajar la cola a 2; después
los tres lotes de dos drones finalizaron con seis resultados correctos.

## Servicio booleano de escenario

El paso `call_set_bool` recibe `service`, `value` y `timeout_sec`. Reintenta
`std_srvs/SetBool` hasta recibir `success=true`, permitiendo esperar gates sin
codificar su logica en el runner. Sus marcadores son
`[SCENARIO-RUNNER-SERVICE-WAIT]`, `NOT-READY`, `DONE` y `TIMEOUT`.

320R2/321 lo usan para llamar a
`/dron_1/control/activate_orb_shadow` entre la aproximacion GT y el nuevo goal
ORB. En 321 se abre antes `control/set_trajectory_active=false` y un
`wait_for_bool` transient-local espera
`control/orb_authority_confirmed=true`; el runner no interpreta tracking,
anchor ni estacionariedad.

## Yaw relativo en escenarios

`yaw_deg` se convierte a radianes y `absoluto_yaw` decide si representa una
orientacion objetivo o un incremento desde la orientacion actual. La
trayectoria tipica de Fase 4 usa seis incrementos relativos alrededor de
`±180°` para forzar giros cortos; los targets XYZ permanecen absolutos.

`codex/archivos_auxiliares/trayectorias/tray_prueba_155.yaml` construye el caso
dirigido A fiducial 2 -> B anchor por loop en fachada norte -> A fiducial 1.
Las pruebas 157/158 lo usan para verificar que el apoyo loop sea posterior al
primer hard y que una optimizacion del padre propague rigidamente el hijo.
