"""Serial link, motion calibration, and vision service defaults."""

import os

# USB serial from Arduino (often /dev/ttyUSB0 or /dev/ttyACM0)
SERIAL_PORT = os.environ.get("ROBOT_SERIAL_PORT", "/dev/ttyUSB0")
BAUD_RATE = int(os.environ.get("ROBOT_SERIAL_BAUD", "115200"))

# Heartbeat: must be faster than Arduino WATCHDOG_MS (firmware uses 500 ms)
PING_INTERVAL_S = 0.25

# Optional read timeout for non-blocking serial reads (seconds)
SERIAL_TIMEOUT = 0.05

# --- Motion heuristics (tune on hardware) ---
# Rough translation from "meters" in voice commands to drive duration
APPROX_METERS_PER_SECOND = float(os.environ.get("ROBOT_METERS_PER_SECOND", "0.2"))
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
