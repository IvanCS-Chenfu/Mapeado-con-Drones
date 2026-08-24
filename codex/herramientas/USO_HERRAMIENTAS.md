# Uso de herramientas de Codex

Este archivo explica cómo usar las herramientas de `src/codex/herramientas/` para automatizar compilación, simulación y reducción de logs durante una fase del pipeline.

La idea general es que los scripts hagan solo trabajo mecánico:

```text
compilar
reducir logs de build
lanzar simulaciones
reducir logs de simulación
```

El análisis real lo hacen los agentes:

```text
diagnosticador_build
analizador_simulacion_logs
curador_documentacion
```

---

## Uso dentro de una subfase automatica

Cuando el usuario pida ejecutar una subfase completa, las herramientas se usan dentro de este contrato:

```text
planificador_fase
  lee pipeline, subfase e historial
  define build, pruebas, YAMLs, patrones y criterio de exito

implementador_fase
  modifica codigo/YAMLs permitidos
  ejecuta build_selected_packages.sh

diagnosticador_build
  solo actua si el build falla y el log fue reducido

implementador_fase
  corrige el primer error real y recompila

implementador_fase
  ejecuta run_simulation.sh para cada prueba requerida
  ejecuta reduce_simulation_log.sh para cada prueba

analizador_simulacion_logs
  compara logs reducidos contra el criterio de exito

curador_documentacion
  registra resultado, evidencia y conclusion
```

Los scripts no deciden si una subfase esta conseguida. Una simulacion con `SIM-EXIT-CODE 0` solo indica que la herramienta termino correctamente. La subfase solo queda `CONSEGUIDA` si el log reducido contiene la evidencia tecnica definida por la subfase.

Conclusiones permitidas:

```text
CONSEGUIDA
NO CONSEGUIDA
PARCIAL
BLOQUEADA
```

Si faltan markers obligatorios o aparece un error grave, el historial debe decir `NO CONSEGUIDA` o `PARCIAL` aunque los scripts hayan devuelto `0`.

---

## Estructura esperada

Desde la raíz de `src/`:

```text
src/
  codex/
    herramientas/
      build_selected_packages.sh
      bootstrap_orbvoc.sh
      check_workspace_architecture.py
      reduce_build_log.sh
      run_simulation.sh
      reduce_simulation_log.sh
      USO_HERRAMIENTAS.md

    archivos_auxiliares/
      logs/
        colcon_build.log
        colcon_build.reduced.log
        prueba_1.log
        prueba_1.reduced.log
        prueba_2.log
        prueba_2.reduced.log
      trayectorias/
        tray_prueba_1.yaml
        tray_prueba_2.yaml
      html/
        f3l_debug_animation_task_1.html
      repeticiones/
        rawdb_prueba_1.record
        f3l_graph_task_1.tsv
```

Los scripts se ejecutan normalmente desde `src/`:

```bash
./codex/herramientas/<script>.sh ...
```

## Permisos operativos preaprobados

El usuario autoriza a Codex a ejecutar directamente, sin pedir confirmación por chat:

1. `colcon build` a través de `build_selected_packages.sh`, aunque escriba en `build/`, `install/` y `log/`.
2. Limpiezas mínimas de artefactos generados dentro de `build/`, `install/` o `log/` si bloquean una compilación o simulación.
3. Simulaciones con `run_simulation.sh`, incluyendo generación de logs, cierre de Gazebo y reintentos automáticos.

Estas autorizaciones no permiten modificar código fuera de `src/`.
Si el sandbox externo exige escalado, Codex debe pedirlo directamente al ejecutar la herramienta.

---

## 1. `build_selected_packages.sh`

### Función

Compila los paquetes ROS 2 indicados por el agente o por el usuario.

El script guarda todo el log de compilación en:

```text
codex/archivos_auxiliares/logs/colcon_build.log
```

Si ya existía un `colcon_build.log`, se sobrescribe. Al compilar
`dron_individual`, el vocabulario ORB completo se prepara fuera de `src/`.

### Uso

```bash
./codex/herramientas/build_selected_packages.sh --group dron paquete
./codex/herramientas/build_selected_packages.sh --group servidor paquete
./codex/herramientas/build_selected_packages.sh --group simulacion paquete
```

### Grupos de Fase 2

```bash
./codex/herramientas/build_selected_packages.sh --group dron dron_individual
./codex/herramientas/build_selected_packages.sh --group servidor orbslam3_server
./codex/herramientas/build_selected_packages.sh --group simulacion simulacion_dron
```

Cada invocación selecciona exactamente un paquete y escribe en
`build/install/log/<grupo>`. Simulación carga los prefijos de Dron y Servidor.
No usar un build global de `src/`: existen dos copias de `orbslam3_msgs`.

### Paquetes pesados

Si el build incluye paquetes pesados, no agruparlos todos en una sola llamada salvo necesidad clara.

Regla recomendada:

```bash
./codex/herramientas/build_selected_packages.sh --group dron orbslam3_msgs
./codex/herramientas/build_selected_packages.sh --group dron orbslam3
./codex/herramientas/build_selected_packages.sh --group servidor orbslam3_multi
./codex/herramientas/build_selected_packages.sh --group servidor orbslam3_server
./codex/herramientas/build_selected_packages.sh --group dron dron_individual
./codex/herramientas/build_selected_packages.sh --group simulacion simulacion_dron
```

Motivo: `orbslam3`, `dron_individual` y `simulacion_dron` pueden consumir bastante CPU, memoria y disco durante `colcon build`. Compilarlos uno a uno reduce el riesgo de bloqueo del ordenador. Si falla uno, reducir el log y diagnosticar ese paquete antes de seguir.

### Resultado

Si compila correctamente:

```text
exit code = 0
```

Si hay error de compilación:

```text
exit code != 0
```

En caso de error, se debe llamar a:

```bash
./codex/herramientas/reduce_build_log.sh
```

---

## 2. `reduce_build_log.sh`

### Función

Reduce el log de compilación para dejar solo los errores importantes.

Lee:

```text
codex/archivos_auxiliares/logs/colcon_build.log
```

Y escribe:

```text
codex/archivos_auxiliares/logs/colcon_build.reduced.log
```

El archivo `colcon_build.log` queda como log completo y nunca se lee
directamente. Si el reducido no contiene contexto suficiente, se regenera con
patrones más precisos.

### Uso

```bash
./codex/herramientas/reduce_build_log.sh
```

### Cuándo usarlo

Solo si `build_selected_packages.sh` devuelve un código distinto de `0`.

### Flujo esperado

```text
build_selected_packages.sh falla
  ↓
reduce_build_log.sh genera colcon_build.reduced.log con errores clave
  ↓
diagnosticador_build lee colcon_build.reduced.log
  ↓
si falta contexto, se amplía o regenera el reducido
  ↓
diagnosticador_build indica qué corregir
  ↓
implementador_fase corrige
  ↓
build_selected_packages.sh se ejecuta de nuevo
```

### Qué debe hacer el agente después

El agente `diagnosticador_build` debe leer:

```text
codex/archivos_auxiliares/logs/colcon_build.reduced.log
```

y decirle a `implementador_fase`:

```text
archivo afectado
función afectada
primer error real
corrección mínima
qué no tocar
```

---

## `bootstrap_orbvoc.sh`

Prepara el `ORBvoc.txt` completo desde el tarball versionado de ORB-SLAM3:

```bash
./codex/herramientas/bootstrap_orbvoc.sh
./codex/herramientas/bootstrap_orbvoc.sh --check
```

La salida normal vive en `build/dron/_phase2_resources/ORBvoc.txt` y CMake la
instala en el share de `dron_individual`. El script no genera el recurso dentro
de `src/`.

## `check_workspace_architecture.py`

```bash
python3 codex/herramientas/check_workspace_architecture.py --check all
```

Comprueba `layout`, `interfaces`, `dependencies`, `config`, `paths`,
`visualizers` y `docs`. Se ejecuta al cerrar toda fase que cambie paquetes,
interfaces, YAML, launch o arquitectura.

## 3. `run_simulation.sh`

### Función

Ejecuta una prueba de simulación.

El script:

```text
1. hace source de install/setup.bash;
2. lanza el launch principal de la simulación;
3. espera a que arranque ROS/Gazebo;
4. detecta si Gazebo murió al arrancar;
5. si Gazebo falla, mata restos de Gazebo y reintenta;
6. ejecuta el nodo scenario_runner_node con el YAML de trayectoria;
7. espera a que scenario_runner_node termine;
8. espera unos segundos extra tras la última acción;
9. cierra el grupo completo del launch con señales tipo Ctrl+C;
10. guarda el log completo de la prueba.
```

Para un launch de replay que deliberadamente no contiene Gazebo, usar
`--without-gazebo`. La herramienta omite solo el healthcheck, reintentos y
`killall` de Gazebo; conserva launch, runner, timeout, log y cleanup.

Para pruebas de rendimiento, `--monitor-resources` inicia
`monitor_simulation_resources.sh` sobre el grupo completo del launch. Produce:

```text
codex/archivos_auxiliares/logs/prueba_X.resources.csv
codex/archivos_auxiliares/logs/prueba_X.resources.summary
```

El CSV registra cada segundo RAM disponible/usada, swap, PSI de memoria/CPU/IO,
CPU/iowait, swap-in/out y RSS/CPU del grupo. Tambien registra PSS del grupo y
desgloses RSS/PSS del servidor, ORB, Gazebo, RViz2 y web; para ORB separa
`Pss_Anon`, `Pss_File` y `Pss_Shmem`. El resumen conserva minimos y maximos.

PSS es la metrica preferida para comparar componentes porque reparte memoria
compartida entre procesos; RSS se conserva para detectar picos y compatibilidad
con pruebas anteriores. Las medias de una zona estable se calculan sobre el CSV
reducido, nunca leyendo el log completo de simulacion.

La guarda predeterminada detiene la prueba tras tres muestras consecutivas con
menos de 1024 MiB disponibles o `memory PSI full avg10 >= 20`. El objetivo es
proteger la sesión gráfica; una prueba abortada por la guarda no se considera
correcta aunque el pipeline hubiera avanzado funcionalmente.

Para cerrar el launch, la herramienta intenta lanzarlo en un grupo de procesos propio con `setsid`.
Al terminar o fallar, envia `SIGINT` al grupo completo, espera, luego `SIGTERM` y finalmente `SIGKILL` si quedan procesos vivos.
Esto busca reproducir el comportamiento de pulsar `Ctrl+C` en una terminal y cerrar tambien los procesos hijos del launch.

### YAML esperado

Para la prueba `x`, el YAML debe estar en:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_x.yaml
```

Ejemplos:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml
codex/archivos_auxiliares/trayectorias/tray_prueba_2.yaml
```

### Log generado

Para la prueba `x`, el log se guarda en:

```text
codex/archivos_auxiliares/logs/prueba_x.log
```

Si ya existía, se sobrescribe.

### Uso

```bash
./codex/herramientas/run_simulation.sh \
  --prueba 1 \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20 \
  --monitor-resources
```

Opciones de monitorización:

```text
--resource-sample-sec SEC
--resource-min-available-mib MIB
--resource-max-memory-psi-full PCT
--resource-guard-consecutive N
```

Marcadores principales:

```text
[SIM-RESOURCE-MONITOR-START]
[SIM-RESOURCE-GUARD]
[SIM-RESOURCE-SUMMARY]
```

### Generar vocabulario ORB compacto

`generate_compact_orb_vocabulary.py` trunca de forma determinista un
vocabulario DBoW2 de texto al nivel indicado, remapea padres y valida el arbol
resultante. El vocabulario 3G se genero asi:

```bash
./codex/herramientas/generate_compact_orb_vocabulary.py \
  dron_individual/config/orbslam/vocabulary/ORBvoc.txt \
  dron_individual/config/orbslam/vocabulary/ORBvoc_L5.txt \
  --depth 5
```

No regenerarlo durante cada build. El archivo versionado es la entrada runtime;
el script sirve para reproducibilidad o para evaluar otra profundidad.

El script buscará automáticamente:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml
```

Y guardará el log en:

```text
codex/archivos_auxiliares/logs/prueba_1.log
```

### Reintentos automáticos si falla Gazebo

`run_simulation.sh` detecta fallos tempranos de Gazebo durante el arranque, por ejemplo:

```text
process has died ... gazebo
exit code 255
gzserver ... error
gzclient ... error
```

Cuando ocurre, la herramienta:

```text
1. cierra el grupo completo del launch activo;
2. ejecuta killall -9 gzserver gzclient gazebo;
3. espera unos segundos;
4. vuelve a lanzar la simulación.
```

Opciones:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba 1 \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20 \
  --max-gazebo-retries 2 \
  --gazebo-retry-wait-sec 5
```

Logs esperados si ocurre un retry:

```text
SIM-GAZEBO-DETECTED
SIM-RETRY
SIM-GAZEBO-KILL
SIM-RETRY-WAIT
SIM-ATTEMPT-START
```

### Replay sin Gazebo

```bash
./codex/herramientas/run_simulation.sh \
  --prueba 95 \
  --launch "ros2 launch simulacion_dron f3f_replay.launch.py rawdb_replay_path:=/ruta/al.record" \
  --yaml /ruta/al/tray_replay.yaml \
  --without-gazebo
```

El log declara `[EXPECT_GAZEBO] false`. No usar esta opcion para ocultar una
caida de Gazebo en una prueba live.

### Ejemplo con dos pruebas

```bash
./codex/herramientas/run_simulation.sh \
  --prueba 1 \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20

./codex/herramientas/run_simulation.sh \
  --prueba 2 \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20
```

Esto generará:

```text
codex/archivos_auxiliares/logs/prueba_1.log
codex/archivos_auxiliares/logs/prueba_2.log
```

---

## 4. `reduce_simulation_log.sh`

### Función

Reduce el log de una prueba de simulación para dejar solo las líneas importantes para validar la fase.

Lee:

```text
codex/archivos_auxiliares/logs/prueba_x.log
```

Y escribe:

```text
codex/archivos_auxiliares/logs/prueba_x.reduced.log
```

El archivo `prueba_x.log` queda como log completo y nunca se lee directamente.
Si faltan marcadores, se regenera el reducido o se crea un sublog temático.

Si el reducido sigue siendo grande, no abrirlo entero por defecto. Crear sublogs
por marcador o tema según:

```text
codex/contexto/09_LOGS_Y_SUBLOGS.md
```

Comando:

```bash
./codex/herramientas/split_simulation_log.sh --prueba X --fase 3L
```

Ejemplos útiles:

```text
prueba_x.scenario.log
prueba_x.errors.log
prueba_x.F3L.log
prueba_x.gt_window.log
prueba_x.index.md
```

### Uso

```bash
./codex/herramientas/reduce_simulation_log.sh \
  --prueba 1 \
  --patterns "SCENARIO-RUNNER|LOCAL-OPT|LOCAL-POSE-GRAPH|LOOP-SUBCLOUD|FID|FUSED|ERROR|FATAL"
```

### Ejemplo de reducción con patrones de Fase 3

```bash
./codex/herramientas/reduce_simulation_log.sh \
  --prueba 1 \
  --patterns "SCENARIO-RUNNER|LOCAL_LOOP_OPT_TASK|LOCAL-POSE-GRAPH|LOCAL-OPT-APPLY|LOOP-SUBCLOUD|FID-DEBT|FUSED-SIMPLE-SUMMARY|ERROR|FATAL|Segmentation fault|Killed|std::bad_alloc"
```

### Cuándo usarlo

Después de ejecutar todas las simulaciones de una fase.

Ejemplo:

```text
run_simulation.sh prueba 1
run_simulation.sh prueba 2
  ↓
reduce_simulation_log.sh prueba 1
reduce_simulation_log.sh prueba 2
  ↓
analizador_simulacion_logs lee prueba_1.reduced.log y prueba_2.reduced.log
  ↓
si faltan marcadores, regenera el reducido o crea sublogs temáticos
```

---

## Flujo completo de una fase

Ejemplo general:

```text
planificador_fase lee pipeline, estado actual e historial
  ↓
implementador_fase modifica código y YAMLs de prueba
  ↓
build_selected_packages.sh compila paquetes seleccionados
  ↓
si build falla:
    reduce_build_log.sh
    diagnosticador_build lee colcon_build.reduced.log
    si falta contexto, regenera colcon_build.reduced.log
    implementador_fase corrige
    volver a build_selected_packages.sh
  ↓
si build pasa:
    run_simulation.sh prueba 1
    run_simulation.sh prueba 2
    ...
  ↓
reduce_simulation_log.sh prueba 1
reduce_simulation_log.sh prueba 2
  ↓
analizador_simulacion_logs valida resultados
  ↓
curador_documentacion actualiza documentación e historial
```

El historial debe registrar como minimo:
- fase/subfase;
- archivos modificados;
- paquetes compilados;
- resultado de build;
- pruebas ejecutadas;
- patrones de reduccion usados;
- evidencia positiva;
- evidencia negativa o ausente;
- conclusion exacta;
- siguiente paso recomendado.

---

## Ejemplo completo para una fase con dos pruebas

### 1. Compilar

```bash
./codex/herramientas/build_selected_packages.sh \
  orbslam3_msgs orbslam3_multi orbslam3_server
```

### 2. Si falla el build

```bash
./codex/herramientas/reduce_build_log.sh
```

Después, el agente `diagnosticador_build` lee:

```text
codex/archivos_auxiliares/logs/colcon_build.reduced.log
```

### 3. Si compila, ejecutar pruebas

```bash
./codex/herramientas/run_simulation.sh \
  --prueba 1 \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20

./codex/herramientas/run_simulation.sh \
  --prueba 2 \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20
```

### 4. Reducir logs de pruebas

```bash
./codex/herramientas/reduce_simulation_log.sh \
  --prueba 1 \
  --patterns "SCENARIO-RUNNER|LOCAL-OPT|LOCAL-POSE-GRAPH|LOOP-SUBCLOUD|FID|FUSED|ERROR|FATAL"

./codex/herramientas/reduce_simulation_log.sh \
  --prueba 2 \
  --patterns "SCENARIO-RUNNER|LOCAL-OPT|LOCAL-POSE-GRAPH|LOOP-SUBCLOUD|FID|FUSED|ERROR|FATAL"
```

### 5. Analizar

El agente `analizador_simulacion_logs` lee:

```text
codex/archivos_auxiliares/logs/prueba_1.reduced.log
codex/archivos_auxiliares/logs/prueba_2.reduced.log
```

Si faltan marcadores obligatorios o hay duda sobre patrones de `grep`, consulta
`prueba_1.log` o `prueba_2.log` completos antes de repetir simulaciones.

Y concluye si la fase ha funcionado.

---

## Reglas importantes

- `colcon_build.log` siempre se sobrescribe con el build completo.
- `colcon_build.reduced.log` se regenera desde `colcon_build.log`.
- `prueba_x.log` siempre se sobrescribe con la simulación completa.
- `prueba_x.reduced.log` se regenera desde `prueba_x.log`.
- No se guardan diagnósticos manuales en archivos separados por ahora.
- El análisis lo hacen los agentes, no los scripts.
- Los scripts solo ejecutan y reducen información.
- Los YAMLs de trayectoria deben llamarse `tray_prueba_x.yaml`.
- Los logs de simulación completos deben llamarse `prueba_x.log`.
- Los logs de simulación reducidos deben llamarse `prueba_x.reduced.log`.
