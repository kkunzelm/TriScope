#!/usr/bin/env bash
# Spannt zwei verbundene Pty-Endpunkte auf:
#   /tmp/lstep-app  -> in der GUI als Port auswählen
#   /tmp/lstep-sim  -> der Simulator hört darauf
# Anschließend wird lstep-sim.py auf dem sim-Endpunkt gestartet.

set -euo pipefail

APP_LINK=/tmp/lstep-app
SIM_LINK=/tmp/lstep-sim
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

rm -f "$APP_LINK" "$SIM_LINK"

socat -d -d \
    "pty,raw,echo=0,link=$APP_LINK" \
    "pty,raw,echo=0,link=$SIM_LINK" \
    >/tmp/lstep-socat.log 2>&1 &
SOCAT_PID=$!

cleanup() {
    kill "$SOCAT_PID" 2>/dev/null || true
    rm -f "$APP_LINK" "$SIM_LINK"
}
trap cleanup EXIT INT TERM

# Auf die Symlinks warten (max 2 s)
for _ in {1..20}; do
    if [[ -e "$APP_LINK" && -e "$SIM_LINK" ]]; then
        break
    fi
    sleep 0.1
done

if [[ ! -e "$APP_LINK" || ! -e "$SIM_LINK" ]]; then
    echo "socat hat keine Ptys angelegt. Log:" >&2
    cat /tmp/lstep-socat.log >&2
    exit 1
fi

echo "================================================================"
echo " GUI-Port  : $APP_LINK"
echo " Sim-Port  : $SIM_LINK   (Simulator hängt sich gleich daran)"
echo " socat-Log : /tmp/lstep-socat.log"
echo " Beenden mit Strg-C"
echo "================================================================"

# Simulator im Vordergrund laufen lassen, damit der Trap beim Beenden greift
exec python3 "$SCRIPT_DIR/lstep-sim.py" "$SIM_LINK"
