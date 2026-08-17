# `scenario_runner_node.cpp`

## Rol

Ejecuta escenarios YAML y envía lotes de goals a
`/dron_X/AccionTrayectoria`. Desde 3C su gate de backpressure está activo.

```text
simulacion_dron/src/control_tray/scenario_runner_node.cpp
rg -n "mapping_backpressure|MOVE-GATE-WAIT|MOVE-GATE-CLEAR" simulacion_dron/src/control_tray
```

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

`codex/archivos_auxiliares/trayectorias/tray_prueba_155.yaml` construye el caso
dirigido A fiducial 2 -> B anchor por loop en fachada norte -> A fiducial 1.
Las pruebas 157/158 lo usan para verificar que el apoyo loop sea posterior al
primer hard y que una optimizacion del padre propague rigidamente el hijo.
