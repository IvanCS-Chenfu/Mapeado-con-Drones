#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
WS_DIR="$(cd "$SRC_DIR/.." && pwd)"
ARCHIVE="$SRC_DIR/dron/ORB_SLAM3/Vocabulary/ORBvoc.txt.tar.gz"
DEFAULT_TARGET="$WS_DIR/build/dron/_phase2_resources/ORBvoc.txt"
MIN_BYTES=100000000
MODE="${1:---prepare}"
TARGET="${2:-$DEFAULT_TARGET}"

is_full_vocab() {
  [ -f "$1" ] && [ "$(stat -c '%s' "$1")" -ge "$MIN_BYTES" ]
}

case "$MODE" in
  --check)
    if is_full_vocab "$TARGET"; then
      echo "[ORBVOC-CHECK][PASS] $TARGET"
      exit 0
    fi
    echo "[ORBVOC-CHECK][FAIL] falta vocabulario completo: $TARGET" >&2
    exit 1
    ;;
  --prepare) ;;
  *)
    echo "Uso: $0 [--prepare|--check] [ruta_destino]" >&2
    exit 2
    ;;
esac

if is_full_vocab "$TARGET"; then
  echo "[ORBVOC][PASS] vocabulario completo ya preparado: $TARGET"
  exit 0
fi
if [ ! -f "$ARCHIVE" ]; then
  echo "[ORBVOC][ERROR] falta recurso versionado: $ARCHIVE" >&2
  exit 2
fi

member="$(tar -tzf "$ARCHIVE" | awk '/(^|\/)ORBvoc\.txt$/ {print; exit}')"
if [ -z "$member" ]; then
  echo "[ORBVOC][ERROR] el tarball no contiene ORBvoc.txt" >&2
  exit 2
fi

mkdir -p "$(dirname "$TARGET")"
tmp_target="${TARGET}.tmp.$$"
trap 'rm -f "${tmp_target:-}"' EXIT
echo "[ORBVOC] extrayendo recurso completo fuera de src/"
tar -xOf "$ARCHIVE" "$member" > "$tmp_target"
if ! is_full_vocab "$tmp_target"; then
  echo "[ORBVOC][ERROR] el recurso extraido no parece el vocabulario completo" >&2
  exit 2
fi
mv "$tmp_target" "$TARGET"
trap - EXIT
echo "[ORBVOC][PASS] $TARGET"
