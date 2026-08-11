#!/bin/bash
# Launch voice-only mode on the Raspberry Pi.
#
# Run by hand:      ./start_voice_only.sh
# Run at boot:      install robot-voice.service (see that file's header).
#
# Assumes: this repo cloned on the Pi, python3 with the repo's
# requirements installed, mic + speaker configured, and OPENAI_API_KEY
# exported (the systemd service loads it from /etc/robot.env).
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# On the Pi the Arduino shows up as /dev/ttyUSB0 (FTDI) — override by
# exporting ROBOT_SERIAL_PORT before launch if yours differs.
export ROBOT_SERIAL_PORT="${ROBOT_SERIAL_PORT:-/dev/ttyUSB0}"

# USB enumeration can lag behind boot: wait up to 30 s for the Arduino
# to appear before starting, instead of crash-looping.
for _ in $(seq 1 30); do
    [ -e "$ROBOT_SERIAL_PORT" ] && break
    sleep 1
done
if [ ! -e "$ROBOT_SERIAL_PORT" ]; then
    echo "ERROR: $ROBOT_SERIAL_PORT never appeared — is the Arduino plugged in?" >&2
    exit 1
fi

cd "$REPO"
exec python3 run_robot.py --voice-only
