# Uso de herramientas de Codex

## Regla principal de logs

Tanto en build como en simulación:

```text
log completo -> reductor -> reducido/sublog -> agente
```

El log completo puede conservarse en `codex/archivos_auxiliares/`, pero ningún agente lo abre directamente. Si el reducido no basta, se genera otro reducido o sublog temático con patrones más precisos.

## Artefactos de colcon

```text
build/dron       install/dron       log/dron
build/servidor   install/servidor   log/servidor
build/simulacion install/simulacion log/simulacion
```

No crear `build/install/log` dentro de `src/<grupo>`.

## `build_selected_packages.sh`

Compila paquetes seleccionados de un grupo usando bases separadas. Para el cierre de Fase 2 se prefiere un paquete por invocación y orden topológico real.

Ejemplo:

```bash
./codex/herramientas/build_selected_packages.sh --group dron orbslam3_msgs
./codex/herramientas/build_selected_packages.sh --group servidor orbslam3_server
./codex/herramientas/build_selected_packages.sh --group simulacion simulacion_dron
```

Si falla, ejecutar `reduce_build_log.sh`. El diagnóstico usa únicamente el reducido; si falta contexto, volver a reducir con patrones específicos.

## `run_simulation.sh`

Ejecuta una prueba, conserva el log completo y limpia el grupo de procesos. La secuencia de entorno acordada es:

```text
/opt/ros/<distro>/setup.bash
install/dron/local_setup.bash
install/servidor/local_setup.bash
install/simulacion/local_setup.bash
```

Después se ejecuta el launch sin volver a cargar perfiles personales. La corrección de Fase 2 conserva `setsid` y usa `bash -c`, no `bash -lc`, salvo evidencia real en contra.

Ejemplo:

```bash
./codex/herramientas/run_simulation.sh   --prueba 198   --launch "ros2 launch simulacion_dron multi_dron.launch.py"
```

`--monitor-resources` puede generar CSV/resumen de recursos. Esos resúmenes sí pueden analizarse directamente; el log completo de simulación no.

## `reduce_simulation_log.sh`

Genera un reducido por patrones. Si sigue siendo grande o insuficiente, usar `split_simulation_log.sh` o repetir la reducción con patrones más concretos.

Nunca usar el log completo como fallback.

## ORBvoc

El runtime normal usa el vocabulario completo `ORBvoc.txt`. En clon limpio debe existir bootstrap/preflight reproducible a partir del recurso versionado. Un vocabulario compacto puede usarse solo de forma explícita para pruebas diagnósticas/performance; no sustituye silenciosamente al normal.

## Flujo de una subfase

```text
plan -> cambios autorizados -> build -> reducción si falla -> diagnóstico reducido
     -> pruebas -> reducción/sublogs -> análisis -> documentación/historial
```

Los scripts no deciden `CONSEGUIDA/PARCIAL/NO CONSEGUIDA/BLOQUEADA`; la conclusión se toma contra el contrato de la subfase y la evidencia reducida.
