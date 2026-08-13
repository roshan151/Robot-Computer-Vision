# Robot Computer Vision

Raspberry Pi–based robot with **computer vision**, **voice + LLM planning**, and an **Arduino** handling real-time **drivetrain** control (PID, encoders) over **USB serial**. The Pi sends high-level intent only (`F` / `B` / `L` / `R` / `S`, speed, heartbeat); the Arduino runs the low-level control loop.

---

## Repository layout

| Path | Purpose |
|------|--------|
| `sketches/drivetrain/drivetrain.ino` | Arduino sketch — upload to the board (motors, encoders, serial protocol). Its header comment is the authoritative pinout. |
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

1. Open `sketches/drivetrain/drivetrain.ino` in the Arduino IDE.  
2. Adjust **pin defines** at the top for your motor driver and encoders. The
   header comment of that file is the authoritative pinout — see
   [Motor driver wiring](#motor-driver-wiring-2-drv8871) and
   [Motor ↔ encoder pairing](#motor--encoder-pairing-invariant) for the
   as-built harness and the invariant it has to satisfy.  
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
Motor Controller: **2× DRV8871 breakout** (one per motor, ILIM ≈ 2.1 A via 30k).
Earlier revisions used a single dual-channel DRV8833; the sections below that
still say "DRV8833" have not been re-verified against the current build.
**Note the voltage warning in [Power path](#power-path) is a DRV8833 limit
(10.8 V max) and does not apply to the DRV8871** — but do not raise the buck
on that basis alone; the firmware header specifies 8–9 V, and the motors and
encoders have their own limits. Re-verify before changing the supply.
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

### Motor driver wiring (2× DRV8871)

> **Superseded.** This section previously described a single dual-channel
> DRV8833 with IN1–IN4 and an EEP pin on **D7**. The build now uses **two
> single-channel DRV8871 breakouts**, one per motor, and **D7 is the right
> encoder's B channel**. Wiring anything to D7 as an enable line will break
> quadrature decoding on the right wheel. The authoritative pinout is the
> header comment of `sketches/drivetrain/drivetrain.ino`.

Each DRV8871 board has its own `IN1` / `IN2` inputs and its own `OUT1` / `OUT2`
motor terminals. Per the driver's truth table (mirrored in `motorWrite()`):

| IN1 | IN2 | Result |
|-----|-----|--------|
| PWM | LOW | forward |
| LOW | PWM | reverse |
| LOW | LOW | coast (auto-sleep) |
| HIGH | HIGH | brake |

**As built:**

```
Arduino          DRV8871 board          Wheel
─────────        ─────────────          ─────
D5  (PWM) ──→    LEFT  board IN1        Motor 2  = LEFT wheel
D6  (PWM) ──→    LEFT  board IN2
D9  (PWM) ──→    RIGHT board IN2  ⚠     Motor 1  = RIGHT wheel
D10 (PWM) ──→    RIGHT board IN1  ⚠
Star GND  ──→    both boards POWER−     (own wire each, no ground ring)
                 both boards POWER+ ←── Buck OUT+ (8–9 V)
```

⚠ **The right board's IN1/IN2 are deliberately reversed** relative to the
firmware's naming (D9 is `IN1_R`, "forward", but lands on the board's IN2).

This is not a mistake, and it must not be "corrected". The two motors are
mirror-mounted, so one side has to be inverted for a `F` command to drive both
wheels the same way down the floor. The firmware has **no motor-invert flag**
— only `ENC_L_INVERT` / `ENC_R_INVERT`, which invert *encoders*, not motors —
so the inversion has to live in the wiring. Swapping a board's IN1/IN2 is
exactly equivalent to swapping its OUT1/OUT2 motor leads, because `motorWrite()`
is symmetric in the two pins; either is fine, but only one may be applied.

If you ever rebuild the harness to match the firmware header literally
(D9→IN1, D10→IN2 on the right board), you must then swap that motor's OUT1/OUT2
leads instead, or the robot will spin in place on every `F`.

### Encoder wiring

2× TSINY-8370 dual-channel Hall encoders. Colours: **yellow = A**, **white = B**,
blue = Vcc, green = GND.

| Encoder wire | Connect to | Notes |
|--------------|-----------|-------|
| blue (Vcc) | Arduino **5V** | not the 9 V rail — the internal 10k pull-up ties Vout to Vcc, so Vcc must equal the logic voltage |
| green (GND) | Star GND | same node as Arduino GND |
| LEFT yellow (A) | **D2** | INT0, rising-edge interrupt |
| LEFT white (B) | **D4** | sampled inside the ISR |
| RIGHT yellow (A) | **D3** | INT1, rising-edge interrupt |
| RIGHT white (B) | **D7** | sampled inside the ISR |

**Both channels are load-bearing.** The firmware is true quadrature: the ISR
fires on A's rising edge and samples B at that instant, so counts are
*hardware-signed* and register real motion — a wheel rolling backwards down a
slope counts down. Direction is **not** inferred from the commanded move.
(Earlier revisions of this README listed right B on D8 and called the B
channels "informational". Both statements are obsolete.)

Consequences of getting A/B backwards on one side: the decoded sign inverts,
signed progress clamps to zero at `drivetrain.ino:673`, the move never reaches
its tick target, and it ends in `TIMEOUT` after 15 s. Swap the two wires rather
than reaching for `ENC_x_INVERT` — a physically correct A/B pairing also fixes
the edge timing, which the invert flag does not.

`ENC_L_INVERT` / `ENC_R_INVERT` are for the *other* problem: correctly paired
A/B, but the motor mounted mirror-image so forward rotation counts down.

**As built: `ENC_R_INVERT 1`.** The right motor is mirror-mounted, so forward
travel spins it the opposite way and its (correctly paired) A/B decodes
negative. Confirmed by hand-spin — rolling both wheels forward gave L=+242,
R=−187 before the flag was set.

> **Both faults negate the sign, so counts alone cannot tell them apart.**
> This bit us once: the harness originally had right yellow→D7 / white→D3
> (A/B swapped) *and* the mirror-mounted motor, and the two errors cancelled.
> `ΔR` read positive, everything looked plausible, and the real inversion only
> appeared once the wire colours were corrected. Distinguish by inspecting the
> **wire colours against the convention** (yellow = A → interrupt pin D2/D3),
> not by the sign of the counts. Colours correct + counts negative → set the
> invert flag. Colours swapped → fix the wires first, then re-test.

### Motor ↔ encoder pairing (invariant)

**Each motor's driver channel and its encoder channel must name the same
wheel.** The left board (D5/D6) must be paired with the left encoder (D2/D4);
the right board (D9/D10) with the right encoder (D3/D7).

This is easy to violate and expensive to diagnose, because straight moves hide
it. `wheelSigns()` maps `F` to (+1, +1) and `B` to (−1, −1) — symmetric, so
crossed channels look fine. `L` is (−1, +1) and `R` is (+1, −1) — antisymmetric,
so crossed channels mirror every turn.

The subtler damage is to the sync trim. The loop at `drivetrain.ino:610`
compares `enc_left` against `enc_right` and applies its correction to channels
L and R. If the pairing is crossed, the correction lands on the wheel it was
not computed for and the loop becomes **positive feedback**: it speeds up the
wheel already ahead until it saturates at `SYNC_AUTHORITY_PCT`. The signature
is a left/right error pinned at a constant magnitude across every move —
consistently one-sided, and never converging.

**Symptom → cause:**

| Symptom | Cause |
|---------|-------|
| `F` drives straight, but `R` turns left | motor channels crossed |
| Sync error stuck at a constant %, always same side | motor↔encoder pairing crossed |
| Sync error alternates sides, varying size | genuine drift — this is what `SYNC_KP` is for |
| Move ends in `TIMEOUT`, one side counts down, other side's count runs far past target | that side's sign is inverted — A/B swapped **or** a mirror-mounted motor without its `ENC_x_INVERT` |
| Neither encoder moves | power, encoder wiring, or serial |

**Verify after any harness change**, before calibrating anything:

1. `tests/test_encoders.py`; hand-roll each wheel in the robot's forward
   direction. Both must count **up** — fix with `ENC_x_INVERT` and re-flash.
2. In the same test, roll the **left** wheel only. `enc_left` must be the
   counter that moves. If `enc_right` moves instead, the pairing is crossed.
3. Command `F` briefly. Both wheels must drive the robot forward, not spin it.
4. Command `R`. The robot must turn right.

Only once all four pass are the calibration numbers meaningful.

### Calibration

Two constants in `config.py`, both overridable by environment variable so you
can calibrate without editing code:

| Constant | Env var | Governs |
|----------|---------|---------|
| `TICKS_PER_CM` | `ROBOT_TICKS_PER_CM` | straight moves |
| `TICKS_PER_DEGREE` | `ROBOT_TICKS_PER_DEGREE` | tank turns |

Both scale the **target**, not the measurement. The firmware ends a move when
*both* wheels reach the target (`drivetrain.ino:714`), so neither constant can
cause a move to stop early — short travel is always traction or a lost encoder
channel, never calibration.

**Distance.** On flat ground, on the surface you will actually run on, at your
normal mission speed (slip changes with PWM, so calibrating at 100 % and
running at 70 % gives a wrong constant):

1. Mark a start line. Run `drivetrain.straight_m(1.0)`.
2. Measure actual travel in cm.
3. `new = current × (100 / measured_cm)`
4. Repeat three times and average — a single trial is noise.

**Turn.** Use a full rotation, not 90°: a 5° reading error at 90° is inside
your measurement precision, while the same relative error at 360° shows up as
20° and is actually readable.

1. Mark the floor under the robot's centre; tape a pointer to the chassis.
2. Run `drivetrain.right(360)`.
3. `new = current × (360 / measured_degrees)`
4. Verify with 4× `right(90)` — the robot should return to its start heading.

`TICKS_PER_DEGREE` is the least portable constant in the tree: a tank turn
scrubs both wheels sideways, so carpet and hardwood genuinely need different
values. Distance is closed-loop on ticks and therefore battery-independent,
but slip is not — calibrate at mid-charge.

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
| `4` | Brown-out | The 5 V rail sagged → something wired to the Arduino's 5 V (both encoders' blue leads) is dragging it down, or ground bounce |
| `8` | Watchdog | Not used by this firmware |

Bits can combine (e.g. `3` = power-on + reset pin).

---

Wire the white B wires: left → D4, right → D7 (blue → 5V, green → GND, yellow → D2/D3 as before).
Flash (./flash.sh or IDE) — confirm the drv8871-v4-quad stamp.
Calibrate polarity: run tests/test_encoders.py, roll each wheel in the robot's forward direction by hand. Both must count up. A side counting down → set its ENC_x_INVERT to 1, re-flash, re-check.
Then test_bot_movements.py — with working, signed encoders this should be the first honest closed-loop run the bot has ever had.
