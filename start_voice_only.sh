#!/bin/bash
# Launch voice-only mode on the Raspberry Pi.
#
# Run by hand:      ./start_voice_only.sh
# Run at boot:      install robot-voice.service (see that file's header).
#
# Waits for two things that are not ready when systemd says "started":
#   * the Arduino's USB serial device
#   * a Bluetooth audio sink, if BT_MAC is configured
#
# Both are polled rather than slept through, so a healthy boot is fast and a
# broken one says which piece is missing instead of failing obscurely.
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$REPO"

# Pull config defaults from .env if present, without clobbering anything
# systemd already put in the environment.
if [ -f "$REPO/.env" ]; then
    set -a
    # shellcheck disable=SC1091
    . "$REPO/.env"
    set +a
fi

export ROBOT_SERIAL_PORT="${ROBOT_SERIAL_PORT:-/dev/ttyUSB0}"
BT_MAC="${BT_MAC:-}"
BT_WAIT_S="${BT_WAIT_S:-20}"
SERIAL_WAIT_S="${SERIAL_WAIT_S:-30}"

log() { printf '%s %s\n' "$(date -u +%H:%M:%S)" "$*" >&2; }

# --- Arduino ---------------------------------------------------------------
for _ in $(seq 1 "$SERIAL_WAIT_S"); do
    [ -e "$ROBOT_SERIAL_PORT" ] && break
    sleep 1
done
if [ ! -e "$ROBOT_SERIAL_PORT" ]; then
    log "ERROR: $ROBOT_SERIAL_PORT never appeared — is the Arduino plugged in?"
    exit 1
fi
if [ ! -r "$ROBOT_SERIAL_PORT" ] || [ ! -w "$ROBOT_SERIAL_PORT" ]; then
    log "ERROR: no read/write access to $ROBOT_SERIAL_PORT."
    log "       Add the user to the dialout group:  sudo usermod -aG dialout $(whoami)"
    exit 1
fi
log "serial ready: $ROBOT_SERIAL_PORT"

# --- Bluetooth audio -------------------------------------------------------
# The unit already slept 5 s; this polls for the sink actually appearing so a
# slow pairing does not silently start the robot deaf. Non-fatal: without the
# buds the robot still drives, it just cannot hear — and that shows up in
# logs.json as an audio error rather than as a mystery.
if [ -n "$BT_MAC" ]; then
    BT_ID="${BT_MAC//:/_}"
    connected=0
    for _ in $(seq 1 "$BT_WAIT_S"); do
        if command -v pactl >/dev/null 2>&1 && \
           pactl list short sources 2>/dev/null | grep -q "$BT_ID"; then
            connected=1
            break
        fi
        # Nudge it — buds often need one explicit connect after a cold boot.
        command -v bluetoothctl >/dev/null 2>&1 && \
            bluetoothctl connect "$BT_MAC" >/dev/null 2>&1
        sleep 1
    done
    if [ "$connected" = "1" ]; then
        log "bluetooth audio ready: $BT_MAC"
        # Prefer mSBC (16 kHz) over CVSD (8 kHz) when the buds offer it.
        if pactl list cards 2>/dev/null | grep -q 'headset-head-unit-msbc'; then
            pactl set-card-profile "bluez_card.$BT_ID" headset-head-unit-msbc \
                >/dev/null 2>&1 && log "profile: mSBC (16 kHz)"
        else
            log "WARN: mSBC unavailable — running 8 kHz CVSD, recognition will suffer"
        fi
    else
        log "WARN: no Bluetooth audio source for $BT_MAC after ${BT_WAIT_S}s"
    fi
fi

exec python3 run_robot.py --voice-only
