#!/usr/bin/env bash
set -uo pipefail

# Compila exactamente un paquete dentro de uno de los tres grupos de Fase 2.
# El log completo se conserva para las herramientas de reduccion.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CODEX_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
SRC_DIR="$(cd "$CODEX_DIR/.." && pwd)"
WS_DIR="$(cd "$SRC_DIR/.." && pwd)"
AUX_DIR="$CODEX_DIR/archivos_auxiliares"
LOG_DIR="$AUX_DIR/logs"
BUILD_LOG="$LOG_DIR/colcon_build.log"

usage() {
  echo "Uso: $0 --group <dron|servidor|simulacion> <paquete>"
}

if [ "$#" -ne 3 ] || [ "$1" != "--group" ]; then
  usage
  exit 2
fi

GROUP="$2"
PACKAGE="$3"
case "$GROUP" in
  dron|servidor|simulacion) ;;
  *)
    echo "[BUILD-ERROR] Grupo no valido: $GROUP"
    usage
    exit 2
    ;;
esac

GROUP_SRC_DIR="$SRC_DIR/$GROUP"
BUILD_BASE="$WS_DIR/build/$GROUP"
INSTALL_BASE="$WS_DIR/install/$GROUP"
COLCON_LOG_BASE="$WS_DIR/log/$GROUP"
ORB_VOCABULARY_FILE="$BUILD_BASE/_phase2_resources/ORBvoc.txt"

mkdir -p "$LOG_DIR"
: > "$BUILD_LOG"

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

source_setup() {
  local setup_file="$1"
  if [ -f "$setup_file" ]; then
    set +u
    # shellcheck disable=SC1090
    source "$setup_file"
    set -u
    echo "[BUILD-INFO] Sourced $setup_file" | tee -a "$BUILD_LOG"
  fi
}

{
  echo "[BUILD-START] $(date '+%Y-%m-%d %H:%M:%S')"
  echo "[BUILD-SCRIPT] $0"
  echo "[SRC_DIR] $SRC_DIR"
  echo "[WS_DIR] $WS_DIR"
  echo "[BUILD-GROUP] $GROUP"
  echo "[PACKAGES] $PACKAGE"
  echo "[BUILD-BASE] $BUILD_BASE"
  echo "[INSTALL-BASE] $INSTALL_BASE"
  echo "[COLCON-LOG-BASE] $COLCON_LOG_BASE"
} | tee -a "$BUILD_LOG"

if ! colcon list --base-paths "$GROUP_SRC_DIR" --names-only | grep -Fqx "$PACKAGE"; then
  echo "[BUILD-ERROR] $PACKAGE no pertenece al grupo $GROUP" | tee -a "$BUILD_LOG"
  exit 2
fi

for variable in AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH; do
  strip_workspace_paths "$variable"
done

ROS_SETUP="/opt/ros/${ROS_DISTRO:-iron}/setup.bash"
if [ ! -f "$ROS_SETUP" ]; then
  echo "[BUILD-ERROR] No existe el setup ROS 2: $ROS_SETUP" | tee -a "$BUILD_LOG"
  exit 2
fi
source_setup "$ROS_SETUP"

case "$GROUP" in
  dron)
    source_setup "$WS_DIR/install/dron/local_setup.bash"
    ;;
  servidor)
    source_setup "$WS_DIR/install/servidor/local_setup.bash"
    ;;
  simulacion)
    source_setup "$WS_DIR/install/dron/local_setup.bash"
    source_setup "$WS_DIR/install/servidor/local_setup.bash"
    source_setup "$WS_DIR/install/simulacion/local_setup.bash"
    ;;
esac

export CMAKE_BUILD_PARALLEL_LEVEL=1
export MAKEFLAGS=-j1

BUILD_CMD=(
  colcon --log-base "$COLCON_LOG_BASE" build
  --base-paths "$GROUP_SRC_DIR"
  --build-base "$BUILD_BASE"
  --install-base "$INSTALL_BASE"
  --packages-select "$PACKAGE"
  --executor sequential
  --symlink-install
  --cmake-target-skip-unavailable
)

if [ "$GROUP" = "dron" ] && [ "$PACKAGE" = "dron_individual" ]; then
  "$SCRIPT_DIR/bootstrap_orbvoc.sh" --prepare "$ORB_VOCABULARY_FILE" || {
    echo "[BUILD-ERROR] no se pudo preparar ORBvoc.txt completo" | tee -a "$BUILD_LOG"
    exit 2
  }
  BUILD_CMD+=(--cmake-args "-DORB_VOCABULARY_FILE=$ORB_VOCABULARY_FILE")
fi

printf '[BUILD-CMD]'
printf ' %q' "${BUILD_CMD[@]}"
printf '\n'
printf '[BUILD-CMD]' >> "$BUILD_LOG"
printf ' %q' "${BUILD_CMD[@]}" >> "$BUILD_LOG"
printf '\n\n' >> "$BUILD_LOG"

cd "$WS_DIR" || {
  echo "[BUILD-ERROR] No se pudo entrar en WS_DIR=$WS_DIR" | tee -a "$BUILD_LOG"
  exit 2
}

set +e
"${BUILD_CMD[@]}" 2>&1 | tee -a "$BUILD_LOG"
status=${PIPESTATUS[0]}
set -e

{
  echo
  echo "[BUILD-END] $(date '+%Y-%m-%d %H:%M:%S')"
  echo "[BUILD-EXIT-CODE] $status"
} | tee -a "$BUILD_LOG"

exit "$status"
