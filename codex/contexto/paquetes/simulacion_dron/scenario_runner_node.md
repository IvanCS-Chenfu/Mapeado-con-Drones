# `scenario_runner_node.cpp`

## Rol

Ejecuta escenarios YAML y envía lotes de goals a
`/dron_X/AccionTrayectoria`. Desde 3C su gate de backpressure está activo. En
4B incorpora el paso generico `wait_for_bool` para esperar readiness externo
sin acoplar el runner al spawner.

```text
simulacion_dron/src/control_tray/scenario_runner_node.cpp
rg -n "wait_for_bool|READY-WAIT|mapping_backpressure|MOVE-GATE-WAIT" simulacion/simulacion_dron/src/control_tray/scenario_runner_node.cpp
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

Marcadores:

```text
[SCENARIO-RUNNER-BACKPRESSURE]
[SCENARIO-RUNNER-MOVE-GATE-WAIT]
[SCENARIO-RUNNER-MOVE-GATE-CLEAR]
```

En prueba 85 el gate esperó 67.956 s y se liberó al bajar la cola a 2; después
los tres lotes de dos drones finalizaron con seis resultados correctos.

## Yaw relativo en escenarios

`yaw_deg` se convierte a radianes y `absoluto_yaw` decide si representa una
orientacion objetivo o un incremento desde la orientacion actual. La
trayectoria tipica de Fase 4 usa seis incrementos relativos alrededor de
`±180°` para forzar giros cortos; los targets XYZ permanecen absolutos.

`codex/archivos_auxiliares/trayectorias/tray_prueba_155.yaml` construye el caso
dirigido A fiducial 2 -> B anchor por loop en fachada norte -> A fiducial 1.
Las pruebas 157/158 lo usan para verificar que el apoyo loop sea posterior al
primer hard y que una optimizacion del padre propague rigidamente el hijo.
