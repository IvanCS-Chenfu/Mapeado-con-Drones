#!/usr/bin/env bash
set -uo pipefail

# run_simulation.sh
# Ejecuta una prueba de simulación y guarda TODO el log en:
#   src/codex/archivos_auxiliares/logs/prueba_X.log
#
# El YAML de trayectoria se busca por defecto en:
#   src/codex/archivos_auxiliares/trayectorias/tray_prueba_X.yaml
#
# Uso mínimo:
#   ./codex/herramientas/run_simulation.sh \
#     --prueba 1 \
#     --launch "ros2 launch simulacion_dron <launch>.launch.py"
#
# Uso con opciones:
#   ./codex/herramientas/run_simulation.sh \
#     --prueba 1 \
#     --launch "ros2 launch simulacion_dron <launch>.launch.py" \
#     --post-scenario-wait-sec 20 \
#     --startup-wait-sec 15 \
#     --timeout-sec 900 \
#     --max-gazebo-retries 2
# Para launches de replay que no incluyen Gazebo, añadir --without-gazebo.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CODEX_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
SRC_DIR="$(cd "$CODEX_DIR/.." && pwd)"
WS_DIR="$(cd "$SRC_DIR/.." && pwd)"
AUX_DIR="$CODEX_DIR/archivos_auxiliares"
LOG_DIR="$AUX_DIR/logs"
TRAY_DIR="$AUX_DIR/trayectorias"

PRUEBA=""
LAUNCH_CMD=""
YAML_FILE=""
POST_WAIT_SEC="20"
STARTUP_WAIT_SEC="15"
TIMEOUT_SEC="900"
MAX_GAZEBO_RETRIES="2"
GAZEBO_RETRY_WAIT_SEC="5"
EXPECT_GAZEBO=true
MONITOR_RESOURCES=false
RESOURCE_SAMPLE_SEC="1"
RESOURCE_MIN_AVAILABLE_MIB="1024"
RESOURCE_MAX_MEMORY_PSI_FULL_AVG10="20"
RESOURCE_GUARD_CONSECUTIVE="3"
SCENARIO_PACKAGE="simulacion_dron"
SCENARIO_EXECUTABLE="scenario_runner_node"
SCENARIO_PARAM_NAME="scenario_file"

strip_workspace_paths() {
  local variable="$1"
  local current="${!variable:-}"
  local filtered=""
  local entry
  local -a entries=()

  IFS=':' read -r -a entries <<< "$current"
  for entry in "${entries[@]}"; do
    [ -z "$entry" ] && continue
    case "$entry" in
      "$WS_DIR/install"|"$WS_DIR/install/"*|"$WS_DIR/build"|"$WS_DIR/build/"*|/opt/ros/*)
        continue
        ;;
    esac
    if [ -z "$filtered" ]; then
      filtered="$entry"
    else
      filtered="$filtered:$entry"
    fi
  done
  printf -v "$variable" '%s' "$filtered"
  export "$variable"
}

source_runtime_setup() {
  local setup_file="$1"
  if [ ! -f "$setup_file" ]; then
    log "[SIM-ERROR] No existe el setup requerido: $setup_file"
    return 1
  fi
  set +u
  # shellcheck disable=SC1090
  source "$setup_file"
  set -u
  log "[SIM-INFO] Sourced $setup_file"
}

usage() {
  cat <<USAGE
Uso:
  $0 --prueba N --launch "ros2 launch paquete launch.py" [opciones]

Opciones:
  --prueba N                         Número de prueba. Genera prueba_N.log y usa tray_prueba_N.yaml.
  --launch CMD                       Comando launch principal entre comillas.
  --yaml FILE                        YAML de trayectoria. Por defecto: codex/archivos_auxiliares/trayectorias/tray_prueba_N.yaml.
  --post-scenario-wait-sec SEC       Espera tras terminar scenario_runner_node. Defecto: 20.
  --startup-wait-sec SEC             Espera inicial tras lanzar simulación. Defecto: 15.
  --timeout-sec SEC                  Timeout para scenario_runner_node. Defecto: 900.
  --max-gazebo-retries N             Reintentos si Gazebo muere al arrancar. Defecto: 2.
  --gazebo-retry-wait-sec SEC        Espera tras matar Gazebo antes de reintentar. Defecto: 5.
  --without-gazebo                   Replay sin Gazebo: omite healthcheck, reintentos y killall de Gazebo.
  --monitor-resources                Registra RAM/swap/CPU/PSI/RSS y activa guarda anti-bloqueo.
  --resource-sample-sec SEC          Periodo de muestreo. Defecto: 1.
  --resource-min-available-mib MIB   Guarda si MemAvailable cae bajo el umbral. Defecto: 1024.
  --resource-max-memory-psi-full PCT Guarda si memory PSI full avg10 supera el umbral. Defecto: 20.
  --resource-guard-consecutive N     Muestras consecutivas para disparar la guarda. Defecto: 3.
  --scenario-package PKG             Paquete del nodo de escenarios. Defecto: simulacion_dron.
  --scenario-executable EXE          Ejecutable del nodo de escenarios. Defecto: scenario_runner_node.
  --scenario-param-name NAME         Parámetro ROS del YAML. Defecto: scenario_file.
USAGE
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --prueba)
      PRUEBA="${2:-}"; shift 2 ;;
    --launch)
      LAUNCH_CMD="${2:-}"; shift 2 ;;
    --yaml)
      YAML_FILE="${2:-}"; shift 2 ;;
    --post-scenario-wait-sec)
      POST_WAIT_SEC="${2:-}"; shift 2 ;;
    --startup-wait-sec)
      STARTUP_WAIT_SEC="${2:-}"; shift 2 ;;
    --timeout-sec)
      TIMEOUT_SEC="${2:-}"; shift 2 ;;
    --max-gazebo-retries)
      MAX_GAZEBO_RETRIES="${2:-}"; shift 2 ;;
    --gazebo-retry-wait-sec)
      GAZEBO_RETRY_WAIT_SEC="${2:-}"; shift 2 ;;
    --without-gazebo)
      EXPECT_GAZEBO=false; shift ;;
    --monitor-resources)
      MONITOR_RESOURCES=true; shift ;;
    --resource-sample-sec)
      RESOURCE_SAMPLE_SEC="${2:-}"; shift 2 ;;
    --resource-min-available-mib)
      RESOURCE_MIN_AVAILABLE_MIB="${2:-}"; shift 2 ;;
    --resource-max-memory-psi-full)
      RESOURCE_MAX_MEMORY_PSI_FULL_AVG10="${2:-}"; shift 2 ;;
    --resource-guard-consecutive)
      RESOURCE_GUARD_CONSECUTIVE="${2:-}"; shift 2 ;;
    --scenario-package)
      SCENARIO_PACKAGE="${2:-}"; shift 2 ;;
    --scenario-executable)
      SCENARIO_EXECUTABLE="${2:-}"; shift 2 ;;
    --scenario-param-name)
      SCENARIO_PARAM_NAME="${2:-}"; shift 2 ;;
    -h|--help)
      usage; exit 0 ;;
    *)
      echo "[SIM-ERROR] Argumento desconocido: $1" >&2
      usage
      exit 2 ;;
  esac
done

mkdir -p "$LOG_DIR" "$TRAY_DIR"

if [ -z "$PRUEBA" ]; then
  echo "[SIM-ERROR] Falta --prueba N" >&2
  usage
  exit 2
fi

if [ -z "$LAUNCH_CMD" ]; then
  echo "[SIM-ERROR] Falta --launch \"ros2 launch ...\"" >&2
  usage
  exit 2
fi

if [ -z "$YAML_FILE" ]; then
  YAML_FILE="$TRAY_DIR/tray_prueba_${PRUEBA}.yaml"
  if [ ! -f "$YAML_FILE" ] && [ -f "$AUX_DIR/tray_prueba_${PRUEBA}.yaml" ]; then
    YAML_FILE="$AUX_DIR/tray_prueba_${PRUEBA}.yaml"
  fi
fi

if [ ! -f "$YAML_FILE" ]; then
  echo "[SIM-ERROR] No existe el YAML de trayectoria: $YAML_FILE" >&2
  exit 2
fi

LOG_FILE="$LOG_DIR/prueba_${PRUEBA}.log"
: > "$LOG_FILE"

log() {
  echo "$@" | tee -a "$LOG_FILE"
}

LAUNCH_PID=""
LAUNCH_USES_PROCESS_GROUP=false
RESOURCE_MONITOR_PID=""
RESOURCE_SUMMARY_EMITTED=false
RESOURCE_CSV_FILE="$LOG_DIR/prueba_${PRUEBA}.resources.csv"
RESOURCE_SUMMARY_FILE="$LOG_DIR/prueba_${PRUEBA}.resources.summary"
RESOURCE_STOP_FILE="${TMPDIR:-/tmp}/codex_sim_${PRUEBA}_$$_resources.stop"
RESOURCE_GUARD_FILE="${TMPDIR:-/tmp}/codex_sim_${PRUEBA}_$$_resources.guard"
SCENARIO_PID=""
CURRENT_ATTEMPT=0
CURRENT_ATTEMPT_LOG_START_LINE=1

launch_is_alive() {
  if [ -z "${LAUNCH_PID:-}" ]; then
    return 1
  fi

  if [ "$LAUNCH_USES_PROCESS_GROUP" = true ]; then
    kill -0 "-$LAUNCH_PID" 2>/dev/null
  else
    kill -0 "$LAUNCH_PID" 2>/dev/null
  fi
}

signal_launch_tree() {
  local signal_name="$1"

  if [ -z "${LAUNCH_PID:-}" ]; then
    return 0
  fi

  if [ "$LAUNCH_USES_PROCESS_GROUP" = true ]; then
    kill "-$signal_name" "-$LAUNCH_PID" 2>/dev/null || true
  else
    kill "-$signal_name" "$LAUNCH_PID" 2>/dev/null || true
  fi
}

start_resource_monitor() {
  if [ "$MONITOR_RESOURCES" != true ]; then
    return 0
  fi
  rm -f "$RESOURCE_STOP_FILE" "$RESOURCE_GUARD_FILE"
  RESOURCE_SUMMARY_EMITTED=false
  "$SCRIPT_DIR/monitor_simulation_resources.sh" \
    --pgid "$LAUNCH_PID" \
    --csv "$RESOURCE_CSV_FILE" \
    --summary "$RESOURCE_SUMMARY_FILE" \
    --stop-file "$RESOURCE_STOP_FILE" \
    --guard-file "$RESOURCE_GUARD_FILE" \
    --sample-sec "$RESOURCE_SAMPLE_SEC" \
    --min-available-mib "$RESOURCE_MIN_AVAILABLE_MIB" \
    --max-memory-psi-full-avg10 "$RESOURCE_MAX_MEMORY_PSI_FULL_AVG10" \
    --guard-consecutive "$RESOURCE_GUARD_CONSECUTIVE" &
  RESOURCE_MONITOR_PID=$!
  log "[SIM-RESOURCE-MONITOR-START] pid=$RESOURCE_MONITOR_PID csv=$RESOURCE_CSV_FILE"
}

stop_resource_monitor() {
  if [ "$MONITOR_RESOURCES" != true ]; then
    return 0
  fi
  if [ "$RESOURCE_SUMMARY_EMITTED" = true ] && [ -z "${RESOURCE_MONITOR_PID:-}" ]; then
    return 0
  fi
  touch "$RESOURCE_STOP_FILE"
  if [ -n "${RESOURCE_MONITOR_PID:-}" ]; then
    wait "$RESOURCE_MONITOR_PID" 2>/dev/null || true
  fi
  RESOURCE_MONITOR_PID=""
  if [ -f "$RESOURCE_SUMMARY_FILE" ]; then
    while IFS= read -r line; do
      log "[SIM-RESOURCE-SUMMARY] $line"
    done < "$RESOURCE_SUMMARY_FILE"
  fi
  RESOURCE_SUMMARY_EMITTED=true
  rm -f "$RESOURCE_STOP_FILE" "$RESOURCE_GUARD_FILE"
}

resource_guard_triggered() {
  [ "$MONITOR_RESOURCES" = true ] && [ -f "$RESOURCE_GUARD_FILE" ]
}

log_resource_guard() {
  if [ -f "$RESOURCE_GUARD_FILE" ]; then
    log "[SIM-RESOURCE-GUARD] $(tr '\n' ' ' < "$RESOURCE_GUARD_FILE")"
  else
    log "[SIM-RESOURCE-GUARD] reason=unknown"
  fi
}

wait_with_resource_guard() {
  local duration="$1"
  local elapsed=0
  while [ "$elapsed" -lt "$duration" ]; do
    if resource_guard_triggered; then
      log_resource_guard
      return 125
    fi
    sleep 1
    elapsed=$((elapsed + 1))
  done
  return 0
}

terminate_scenario() {
  if [ -z "${SCENARIO_PID:-}" ]; then
    return 0
  fi
  if kill -0 "$SCENARIO_PID" 2>/dev/null; then
    kill -TERM "$SCENARIO_PID" 2>/dev/null || true
    pkill -TERM -P "$SCENARIO_PID" 2>/dev/null || true
    sleep 1
    kill -KILL "$SCENARIO_PID" 2>/dev/null || true
    pkill -KILL -P "$SCENARIO_PID" 2>/dev/null || true
  fi
  wait "$SCENARIO_PID" 2>/dev/null || true
  SCENARIO_PID=""
}

terminate_launch() {
  if launch_is_alive; then
    log "[SIM-CLEANUP] Enviando SIGINT launch PID=$LAUNCH_PID process_group=$LAUNCH_USES_PROCESS_GROUP"
    signal_launch_tree INT
    sleep 5

    if launch_is_alive; then
      log "[SIM-CLEANUP] Enviando SIGTERM launch PID=$LAUNCH_PID process_group=$LAUNCH_USES_PROCESS_GROUP"
      signal_launch_tree TERM
      sleep 3
    fi

    if launch_is_alive; then
      log "[SIM-CLEANUP] Forzando SIGKILL launch PID=$LAUNCH_PID process_group=$LAUNCH_USES_PROCESS_GROUP"
      signal_launch_tree KILL
      sleep 1
    fi
  fi

  LAUNCH_PID=""
  LAUNCH_USES_PROCESS_GROUP=false
}

kill_gazebo_processes() {
  log "[SIM-GAZEBO-KILL] killall -9 gzserver gzclient gazebo"
  killall -9 gzserver gzclient gazebo >> "$LOG_FILE" 2>&1 || true
}

gazebo_failed_during_startup() {
  if [ "$EXPECT_GAZEBO" != true ]; then
    return 1
  fi

  if [ -n "${LAUNCH_PID:-}" ] &&
      ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    log "[SIM-GAZEBO-DETECTED] reason=launch_process_exited_early attempt=$CURRENT_ATTEMPT pid=$LAUNCH_PID"
    return 0
  fi

  if tail -n +"$CURRENT_ATTEMPT_LOG_START_LINE" "$LOG_FILE" |
      grep -Ein "(gazebo|gzserver|gzclient).*process has died|process has died.*(gazebo|gzserver|gzclient)|exit code 255|gzserver: .*error|gzclient: .*error" >/dev/null 2>&1; then
    log "[SIM-GAZEBO-DETECTED] reason=log_pattern attempt=$CURRENT_ATTEMPT"
    return 0
  fi

  return 1
}

run_one_attempt() {
  CURRENT_ATTEMPT="$1"
  CURRENT_ATTEMPT_LOG_START_LINE=$(( $(wc -l < "$LOG_FILE") + 1 ))

  log "[SIM-ATTEMPT-START] attempt=$CURRENT_ATTEMPT max_gazebo_retries=$MAX_GAZEBO_RETRIES"

  log "[SIM-LAUNCH-START] $LAUNCH_CMD"
  # setsid crea un grupo de procesos propio. El cleanup puede enviar SIGINT
  # al grupo completo, que se parece mas a pulsar Ctrl+C en una terminal.
  if command -v setsid >/dev/null 2>&1; then
    setsid bash -lc "$LAUNCH_CMD" >> "$LOG_FILE" 2>&1 &
    LAUNCH_USES_PROCESS_GROUP=true
  else
    bash -lc "$LAUNCH_CMD" >> "$LOG_FILE" 2>&1 &
    LAUNCH_USES_PROCESS_GROUP=false
  fi
  LAUNCH_PID=$!
  log "[SIM-LAUNCH-PID] $LAUNCH_PID process_group=$LAUNCH_USES_PROCESS_GROUP"
  start_resource_monitor

  log "[SIM-WAIT-STARTUP] seconds=$STARTUP_WAIT_SEC"
  if ! wait_with_resource_guard "$STARTUP_WAIT_SEC"; then
    terminate_launch
    stop_resource_monitor
    return 125
  fi

  if gazebo_failed_during_startup; then
    log "[SIM-RETRY] reason=gazebo_died_early attempt=$CURRENT_ATTEMPT"
    terminate_launch
    stop_resource_monitor
    if [ "$EXPECT_GAZEBO" = true ]; then
      kill_gazebo_processes
    fi
    log "[SIM-RETRY-WAIT] seconds=$GAZEBO_RETRY_WAIT_SEC"
    sleep "$GAZEBO_RETRY_WAIT_SEC"
    return 100
  fi

  SCENARIO_CMD=(
    ros2 run "$SCENARIO_PACKAGE" "$SCENARIO_EXECUTABLE"
    --ros-args
    -p "${SCENARIO_PARAM_NAME}:=${YAML_FILE}"
  )

  log "[SIM-SCENARIO-START] ${SCENARIO_CMD[*]}"
  set +e
  if command -v timeout >/dev/null 2>&1; then
    timeout "$TIMEOUT_SEC" "${SCENARIO_CMD[@]}" >> "$LOG_FILE" 2>&1 &
  else
    "${SCENARIO_CMD[@]}" >> "$LOG_FILE" 2>&1 &
  fi
  SCENARIO_PID=$!
  scenario_status=""
  while kill -0 "$SCENARIO_PID" 2>/dev/null; do
    if resource_guard_triggered; then
      log_resource_guard
      terminate_scenario
      scenario_status=125
      break
    fi
    sleep 1
  done
  if [ -z "$scenario_status" ]; then
    wait "$SCENARIO_PID"
    scenario_status=$?
    SCENARIO_PID=""
  fi
  set -e
  log "[SIM-SCENARIO-EXIT-CODE] $scenario_status"

  log "[SIM-POST-SCENARIO-WAIT] seconds=$POST_WAIT_SEC"
  if ! wait_with_resource_guard "$POST_WAIT_SEC"; then
    return 125
  fi

  if [ "$scenario_status" -ne 0 ]; then
    log "[SIM-ERROR] scenario_runner_node terminó con código $scenario_status"
    return "$scenario_status"
  fi

  log "[SIM-DONE] prueba=$PRUEBA success=true"
  return 0
}

cleanup() {
  local exit_code=$?
  terminate_scenario
  terminate_launch
  stop_resource_monitor
  log "[SIM-EXIT-CODE] $exit_code"
}
trap cleanup EXIT

{
  echo "[SIM-START] $(date '+%Y-%m-%d %H:%M:%S')"
  echo "[SRC_DIR] $SRC_DIR"
  echo "[WS_DIR] $WS_DIR"
  echo "[PRUEBA] $PRUEBA"
  echo "[LOG_FILE] $LOG_FILE"
  echo "[YAML_FILE] $YAML_FILE"
  echo "[LAUNCH_CMD] $LAUNCH_CMD"
  echo "[SCENARIO_NODE] ros2 run $SCENARIO_PACKAGE $SCENARIO_EXECUTABLE"
  echo "[STARTUP_WAIT_SEC] $STARTUP_WAIT_SEC"
  echo "[POST_SCENARIO_WAIT_SEC] $POST_WAIT_SEC"
  echo "[TIMEOUT_SEC] $TIMEOUT_SEC"
  echo "[MAX_GAZEBO_RETRIES] $MAX_GAZEBO_RETRIES"
  echo "[GAZEBO_RETRY_WAIT_SEC] $GAZEBO_RETRY_WAIT_SEC"
  echo "[EXPECT_GAZEBO] $EXPECT_GAZEBO"
  echo "[MONITOR_RESOURCES] $MONITOR_RESOURCES"
  echo "[RESOURCE_SAMPLE_SEC] $RESOURCE_SAMPLE_SEC"
  echo "[RESOURCE_MIN_AVAILABLE_MIB] $RESOURCE_MIN_AVAILABLE_MIB"
  echo "[RESOURCE_MAX_MEMORY_PSI_FULL_AVG10] $RESOURCE_MAX_MEMORY_PSI_FULL_AVG10"
  echo "[RESOURCE_GUARD_CONSECUTIVE] $RESOURCE_GUARD_CONSECUTIVE"
  echo
} | tee -a "$LOG_FILE"

cd "$WS_DIR" || {
  log "[SIM-ERROR] No se pudo entrar en WS_DIR=$WS_DIR"
  exit 2
}

for variable in AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH; do
  strip_workspace_paths "$variable"
done

source_runtime_setup "/opt/ros/${ROS_DISTRO:-iron}/setup.bash" || exit 2
source_runtime_setup "$WS_DIR/install/dron/local_setup.bash" || exit 2
source_runtime_setup "$WS_DIR/install/servidor/local_setup.bash" || exit 2
source_runtime_setup "$WS_DIR/install/simulacion/local_setup.bash" || exit 2

attempt=0
while [ "$attempt" -le "$MAX_GAZEBO_RETRIES" ]; do
  set +e
  run_one_attempt "$attempt"
  attempt_status=$?
  set -e

  if [ "$attempt_status" -eq 0 ]; then
    exit 0
  fi

  if [ "$attempt_status" -ne 100 ]; then
    exit "$attempt_status"
  fi

  attempt=$((attempt + 1))
done

log "[SIM-ERROR] Gazebo fallo durante el arranque tras $MAX_GAZEBO_RETRIES reintentos"
exit 1
