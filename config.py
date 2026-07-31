"""Serial link, motion calibration, and vision service defaults."""

import os

# USB serial from Arduino (often /dev/ttyUSB0 or /dev/ttyACM0)
SERIAL_PORT = os.environ.get("ROBOT_SERIAL_PORT", "/dev/ttyUSB0")
BAUD_RATE = int(os.environ.get("ROBOT_SERIAL_BAUD", "115200"))

# Heartbeat: must be faster than Arduino WATCHDOG_MS (firmware uses 500 ms)
PING_INTERVAL_S = 0.25

# Optional read timeout for non-blocking serial reads (seconds)
SERIAL_TIMEOUT = 0.05

# ---------------------------------------------------------------------------
# Encoder calibration — tune these on flat ground before running missions.
# ---------------------------------------------------------------------------
# How to calibrate TICKS_PER_CM:
#   1. Mark a start line on the floor.
#   2. Run: drivetrain.straight_m(1.0)
#   3. Measure actual distance traveled in cm.
#   4. New value = current TICKS_PER_CM * (100 / measured_cm)
TICKS_PER_CM: float = float(os.environ.get("ROBOT_TICKS_PER_CM", "25.0"))

# How to calibrate TICKS_PER_DEGREE:
#   1. Run: drivetrain.right(360)
#   2. Count actual degrees rotated.
#   3. New value = current TICKS_PER_DEGREE * (360 / measured_degrees)
TICKS_PER_DEGREE: float = float(os.environ.get("ROBOT_TICKS_PER_DEGREE", "3.5"))

# Acceptable sync-error ratio between left and right encoders (0.0–1.0).
# A move producing more skew than this triggers a warning log.
# 0.10 = allow up to 10% difference between wheels.
ENCODER_SYNC_WARN_RATIO: float = float(os.environ.get("ROBOT_SYNC_WARN_RATIO", "0.10"))

# The firmware emits ENC: telemetry every 100 ms and zeroes both counters at
# the start of each encoder-counted move.  After a move ACKs, wait at least one
# telemetry interval before reading the counts, otherwise the last line
# received is a mid-move sample and the reported travel is short.
ENCODER_SETTLE_S: float = float(os.environ.get("ROBOT_ENCODER_SETTLE_S", "0.15"))

# ---------------------------------------------------------------------------
# Connection / handshake settings
# ---------------------------------------------------------------------------
# How long to wait after a DTR reset before draining the buffer and retrying
# the handshake. The UNO bootloader takes ~1.5 s; 2.5 s gives real margin.
# Only used in the DTR-reset fallback path — the fast-path ping skips this.
ARDUINO_DRAIN_WAIT_S: float = float(os.environ.get("ROBOT_DRAIN_WAIT_S", "2.5"))

# Number of times to retry the initial handshake "S" command before giving up.
HANDSHAKE_RETRIES: int = int(os.environ.get("ROBOT_HANDSHAKE_RETRIES", "3"))

# Per-attempt timeout for the handshake ping (seconds).
HANDSHAKE_TIMEOUT_S: float = float(os.environ.get("ROBOT_HANDSHAKE_TIMEOUT_S", "2.0"))

# ---------------------------------------------------------------------------
# Motion defaults
# ---------------------------------------------------------------------------
DEFAULT_TURN_DEGREES = float(os.environ.get("ROBOT_DEFAULT_TURN_DEG", "90"))
DEFAULT_MOVE_METERS = float(os.environ.get("ROBOT_DEFAULT_MOVE_M", "1.0"))

# Vision HTTP API (run Vision service: uvicorn Vision.app:app --host 0.0.0.0 --port 8080)
VISION_SERVICE_URL = os.environ.get(
    "VISION_SERVICE_URL",
    "http://127.0.0.1:8080/detect_objects:frame",
)

# Comma-separated class names; if any appear, guardian may stop the robot while moving
VISION_HALT_OBJECTS = [
    s.strip()
    for s in os.environ.get("VISION_HALT_OBJECTS", "").split(",")
    if s.strip()
]

# Guardian poll rate (Hz) while robot reports motion
VISION_GUARD_HZ = float(os.environ.get("VISION_GUARD_HZ", "4.0"))
