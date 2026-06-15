#!/usr/bin/env bash
# RoboTaksi Dashboard — evrensel başlatıcı (Linux + macOS)
set -e

PORT="${1:-8080}"
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ── Python bul (3.9+) ─────────────────────────────────────────────
for candidate in python3 python3.12 python3.11 python3.10 python3.9; do
  if command -v "$candidate" &>/dev/null; then
    ver=$("$candidate" -c "import sys; print(sys.version_info >= (3,9))" 2>/dev/null)
    if [ "$ver" = "True" ]; then
      PYTHON="$candidate"
      break
    fi
  fi
done

if [ -z "$PYTHON" ]; then
  echo "HATA: Python 3.9+ bulunamadı. Lütfen kurun."
  exit 1
fi

# ── Bağımlılıkları kur ────────────────────────────────────────────
if ! "$PYTHON" -c "import fastapi, uvicorn, yaml" 2>/dev/null; then
  echo "Bağımlılıklar kuruluyor…"
  "$PYTHON" -m pip install -r "$DIR/requirements.txt" --quiet \
    --break-system-packages 2>/dev/null || \
  "$PYTHON" -m pip install -r "$DIR/requirements.txt" --quiet
fi

echo ""
echo "╔════════════════════════════════════════╗"
echo "║   RoboTaksi Dashboard                  ║"
echo "║   http://localhost:${PORT}                ║"
echo "╚════════════════════════════════════════╝"
echo ""

cd "$DIR"
"$PYTHON" -m uvicorn backend:app --host 0.0.0.0 --port "$PORT" --log-level warning
