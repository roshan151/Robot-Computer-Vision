# Robot Computer Vision

Raspberry Pi–based robot with **computer vision**, **voice + LLM planning**, and an **Arduino** handling real-time **drivetrain** control (PID, encoders) over **USB serial**. The Pi sends high-level intent only (`F` / `B` / `L` / `R` / `S`, speed, heartbeat); the Arduino runs the low-level control loop.

---

## Repository layout

| Path | Purpose |
|------|--------|
| `sketch_drivetrain/` | Arduino sketch — upload to the board (motors, encoders, serial protocol) |
| `config.py` | Serial port, baud, motion/vision heuristics, environment variable defaults |
| `arduino_bridge.py` / `serial_protocol.py` | PySerial, line protocol, heartbeat `PING` |
| `drivetrain_client.py` | `SerialDrivetrain` — timed moves and intent methods |
| `movement_adapter.py` / `movement_context.py` | Voice/planner → `SerialDrivetrain`; movement history / backtrack |
| `vision_client.py` | Camera capture (Picamera2 or OpenCV) + HTTP calls to the vision API |
| `coordinator.py` | Optional **vision guardian** — stop if selected YOLO classes appear while moving |
| `voice_session.py` | Speech → OpenAI (JSON plan) → movement + vision steps |
| `brain_loop.py` | Simple ~10 Hz OpenCV loop → intent commands (no voice) |
| `run_robot.py` | Main entry: full stack or `--brain-only` |
| `prompts_and_glossary.py` | LLM system prompt and command → method mapping for voice |
| `deprecated/` | Older Pi-GPIO `drivetrain`, legacy vision/voice files (reference only) |
| `deprecated/Vision/` | FastAPI + YOLO `POST /detect_objects:frame` service |

---

## Hardware (summary)

- **Raspberry Pi 4B** — vision, planning, serial to Arduino  
- **Camera** — high-resolution module (Picamera2) or USB / OpenCV  
- **Drivetrain** — 2× DC motors, **L298** (or similar), **separate motor battery**; **common ground** with logic  
- **Arduino** — motor PWM/direction, quadrature encoders, watchdog on serial  
- Optional: TF-Luna distance sensor (your older design mentioned obstacle override — wire through planning/firmware as you prefer)

Longer power/wiring notes from your build are still valid; keep motor supply separate from logic where applicable.

---

## Prerequisites

- **Raspberry Pi**: Python 3.10+ recommended  
- **Arduino**: Uno-class or compatible (interrupt-capable encoder pins per sketch)  
- USB cable **Arduino ↔ Pi** (serial; often `/dev/ttyACM0` or `/dev/ttyUSB0`)  
- **API keys**: `OPENAI_API_KEY` or `OPENAI_API_KEY_ROBIN` in `.env` for voice mode  

---

## Installation (Raspberry Pi)

From the **repository root** (this folder):

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

**Pi-only camera** (when using Picamera2):

```bash
# Follow Raspberry Pi OS docs for picamera2 / libcamera
```

**Vision server** (YOLO API used by `vision_client.py`):

```bash
pip install -r requirements-vision.txt
```

---

## Configuration

### Environment variables

All can be set in a `.env` file (loaded by `voice_session`) or exported in the shell.

| Variable | Purpose |
|----------|--------|
| `ROBOT_SERIAL_PORT` | Default `/dev/ttyUSB0` — set to `/dev/ttyACM0` if needed |
| `ROBOT_SERIAL_BAUD` | Must match Arduino `Serial.begin(...)` (default `115200`) |
| `ROBOT_METERS_PER_SECOND` | Scales spoken “meters” into drive duration |
| `VISION_SERVICE_URL` | Full URL to `POST .../detect_objects:frame` (see below) |
| `VISION_HALT_OBJECTS` | Comma list of YOLO class names; guardian stops the robot if seen while moving |
| `VISION_GUARD_HZ` | Guardian polling rate |
| `OPENAI_API_KEY` / `OPENAI_API_KEY_ROBIN` | Voice + planning |

### Vision API URL

Default in `config.py` is `http://127.0.0.1:8080/detect_objects:frame`.  
If the vision service runs on another machine, set `VISION_SERVICE_URL` to that host.

---

## Arduino firmware

1. Open `sketch_drivetrain/sketch_drivetrain.ino` in the Arduino IDE.  
2. Adjust **pin defines** at the top for your motor driver and encoders.  
3. Select board/port, **Upload**.  
4. Optional: Serial Monitor at the same baud as `ROBOT_SERIAL_BAUD` to see `ACK` / `ENC:` lines.

You can test upload and serial **without motors connected**; encoder lines use internal pull-ups (counts may be noisy when floating).

---

## Running the vision API (YOLO)

From the repository root:

```bash
cd deprecated
uvicorn Vision.app:app --host 0.0.0.0 --port 8080
```

The Pi’s `vision_client.py` posts JSON: `{ "image": "<base64>", "objects": ["plant", ...] }` and expects `{ "response": true/false }`.

For a **remote** PC running Docker/YOLO, point `VISION_SERVICE_URL` at that host.

---

## Running the robot stack

Always run these commands from the **repository root** so imports resolve:

```bash
source .venv/bin/activate
python run_robot.py
```

Options:

| Flag | Meaning |
|------|--------|
| `--no-guardian` | Do not start the vision guardian (ignore `VISION_HALT_OBJECTS` for stopping) |
| `--brain-only` | Only `brain_loop.py` — OpenCV / stub decisions → Arduino (no voice/GPT) |

Shortcut equivalent:

```bash
python brain_loop.py
```

Legacy entry name `voice_controls_v2.py` may still exist under `deprecated/`; prefer `python run_robot.py`.

---

## How to modify the codebase

### Serial protocol / Arduino

- **Firmware**: `sketch_drivetrain/sketch_drivetrain.ino` — watchdog, PID, `F`/`B`/`L`/`R`/`S`, `V:<0–255>`, `PING`.  
- **Pi**: `serial_protocol.py`, `arduino_bridge.py` — keep Pi and firmware in sync.

### Motion tuning (Pi)

- **`config.py`**: `APPROX_METERS_PER_SECOND`, defaults for turn/move.  
- **`drivetrain_client.py`**: duration-based `straight` / `reverse` / turns — tune times vs real-world distance.

### Voice commands / LLM output

- **`prompts_and_glossary.py`**: `movement_prompt`, `commands` — maps phrases like `forward` → method `straight`.  
- **`voice_session.py`**: `JSON_SUFFIX` — schema for GPT (`steps` array). Change prompt + parser together if you add new step types.

### Vision

- **Detection classes**: YOLO names in `deprecated/Vision/service.py` (`computer_vision.detect_plant`).  
- **Guardian**: `coordinator.py` + `VISION_HALT_OBJECTS` — stop when listed classes appear during motion.

### Adding a new feature

1. Prefer **intent on the Pi**, **real-time on Arduino**.  
2. Extend serial protocol only if needed; use `\n`-terminated lines.  
3. Update `serial_protocol.py` and firmware together.

---

## Robot images

**Top view:**  
![Top](https://github.com/user-attachments/assets/65aa1004-ad71-4d3f-be6d-fdf788f3cd46)

**Side view:**  
![Side](https://github.com/user-attachments/assets/65aa1004-c084-4d42-8a80-22c9ce910a82)

**Front view:**  
![Front](https://github.com/user-attachments/assets/65aa1004-b8d2-4fd7-ae23-dbf517e464cc)

---

## Resources

- Speech recognition overview: [Real-time speech-to-text on Raspberry Pi](https://atsss.medium.com/real-time-speech-to-text-on-raspberry-pi-and-python-4be8c347a8fc)  
- Legacy TTS option: [pyttsx3](https://pypi.org/project/pyttsx3/) (current stack may use Nix TTS if configured in `voice_session.py`)
