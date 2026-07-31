# Robot Computer Vision

Raspberry Pi–based robot with **computer vision**, **voice + LLM planning**, and an **Arduino** handling real-time **drivetrain** control (PID, encoders) over **USB serial**. The Pi sends high-level intent only (`F` / `B` / `L` / `R` / `S`, speed, heartbeat); the Arduino runs the low-level control loop.

---

## Repository layout

| Path | Purpose |
|------|--------|
| `sketches/drivetrain.ino` | Arduino sketch — upload to the board (motors, encoders, serial protocol) |
| `sketches/test-*.ino` | Standalone diagnostic sketches (motors only, encoders only, debug) |
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

1. Open `sketches/drivetrain.ino` in the Arduino IDE.  
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


## Debugging Arduino
Check if anything is occupying the port: lsof /dev/cu.usbserial-A5069RR4

## Electrical Schematics

### Bill of Materials

Motors: Uses 2 DC motors, Specs -  12v, 130 RPM, Geared motors
Battery pack: (Used to power DRV and thr 2 motors) 3S Lipo Battery, 50C, 2200mAh, 11.1V
Motor Controller: DRV 8833
Microcontroller: Arduino used to control motors (Powered by Raspberry Pi through USB B port).
Raspberry Pi 4B: Used for computer vision and voice controls, Powered by PiSugar battery.

| Component | Qty | Value | Purpose |
|-----------|-----|-------|---------|
| Ceramic Capacitor | 1 | 100nF (0.1µF) | DRV8833 VCC bypass |
| Ceramic Capacitor | 1 | 100nF (0.1µF) | Left motor output filter |
| Ceramic Capacitor | 1 | 100nF (0.1µF) | Right motor output filter |
| Electrolytic | 1 | 1000µF | Buck converter output bulk |

### Power path
```
LiPo 3S (11.1 V nominal, 12.6 V charged)
   ├─ (+) ──→ Buck Converter IN+
   └─ (−) ──→ STAR POINT ──→ Buck Converter IN−

Buck Converter OUT+ (set to 10 V) ──→ DRV8833 VCC   ← powers motors AND chip
Buck Converter OUT− ──→ Star ground
1000 µF electrolytic across Buck OUT+ / OUT−
```

> **Important:** this DRV8833 module has **no separate VM pin** — the pin
> labelled VCC is the single supply for both the motors and the chip itself.
> The DRV8833's recommended maximum is **10.8 V**, so a 10 V buck setting
> leaves almost no headroom; **setting the buck to ~9 V is safer** (the 12 V
> motors just run a little slower). Never connect raw LiPo voltage to VCC.

### STAR Ground Physical Layout
```
                    ┌─── 16 AWG wire ──→ DRV8833 GND pin
                    │
LiPo (−) ──STAR POINT
                    │
                    ├─── 16 AWG wire ──→ Arduino GND pin (Arduino itself is USB-powered)
                    │
                    └─── 16 AWG wire ──→ Buck Converter GND IN
                    (encoder grounds also return here)
```

### DRV8833 module wiring

Module pinout — side 1: `IN1 IN2 VCC GND IN3 IN4`, side 2: `EEP OUT1 OUT2 OUT3 OUT4 ULT`

```
Arduino            DRV8833 module      Motors
─────────          ──────────────      ──────
Pin 5 (PWM) ──→    IN1        OUT1 ──→ Motor L (+)
Pin 6 (PWM) ──→    IN2        OUT2 ──→ Motor L (−)
Pin 9 (PWM) ──→    IN3        OUT3 ──→ Motor R (+)
Pin 10 (PWM) ─→    IN4        OUT4 ──→ Motor R (−)
Pin 7       ──→    EEP        (sleep — firmware drives it HIGH; do NOT use the 3.3V pin)
Star GND    ──→    GND
                   VCC  ←── Buck converter output
                   ULT  ←── fault output, currently unconnected
```

- **EEP (sleep):** wired to Arduino **pin D7**, which the firmware drives HIGH
  at boot so the chip is always awake. Do **not** wire EEP to the 3.3 V pin:
  that rail comes from a tiny regulator tied to the USB-serial chip, and the
  wire acts as a pipe for motor switching noise straight into the serial link.
- **ULT (fault):** open-drain output that goes LOW on overcurrent or
  overheating. Unconnected for now; wiring it to a spare Arduino input would
  let the firmware report driver faults.

### Encoder wiring

| Encoder pin | Connect to |
|-------------|-----------|
| VCC | Arduino 5V |
| GND | Star GND (same node as Arduino GND) |
| Left Hall A | D2 (interrupt) |
| Left Hall B | D4 (unused by firmware) |
| Right Hall A | D3 (interrupt) |
| Right Hall B | D8 (unused by firmware) |

The firmware counts only the A channels (D2/D3) and infers direction from the
commanded move, so the B channels are currently informational.

### Capacitors: What, Where, How

| Capacitor | Type | Location | Why |
|-----------|------|----------|-----|
| #1 (100nF) | Ceramic disc | **DRV8833 VCC → GND** | High-frequency bypass for the driver's supply pin |
| #2 (100nF) | Ceramic disc | **DRV8833 OUT1 → OUT2** | Suppress left motor brush noise |
| #3 (100nF) | Ceramic disc | **DRV8833 OUT3 → OUT4** | Suppress right motor brush noise |
| #4 (1000µF) | Electrolytic | **Buck Vout → Buck GND** | Bulk storage on the motor rail (VCC) for current spikes |

### Reading the BOOT code (reset diagnosis)

Every time the Arduino starts it prints `BOOT:<hex>`. The hex digit says
**why** the chip restarted — this is the primary tool for diagnosing
mid-drive resets:

| Code | Cause | What it means here |
|------|-------|--------------------|
| `1` | Power-on | The Arduino's 5 V vanished completely → **USB power was interrupted** (cable snag, loose connector, Mac port overcurrent shutdown) |
| `2` | Reset pin | Normal right after opening the serial port (DTR pulse). Mid-drive: electrical noise reached the RESET pin |
| `4` | Brown-out | The 5 V rail sagged → something wired to the Arduino's 5 V (encoders, EEP) is dragging it down, or ground bounce |
| `8` | Watchdog | Not used by this firmware |

Bits can combine (e.g. `3` = power-on + reset pin).

---