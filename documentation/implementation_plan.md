# Implementation plan — silent voice agent

## Locked design

| Decision | Value |
|---|---|
| Robot speech | **None** during normal operation. Audio only on failure. |
| `YES` gesture | forward 0.1 m, back 0.1 m |
| `NO` / `DIDN'T UNDERSTAND` gesture | left 30°, right 60°, left 30° (same gesture for both) |
| Thinking | no gesture |
| Socket drop / failure | audio alert to operator → log → restart voice session |
| Odometry drift from gestures | accepted, not mitigated |
| Vision | **stubs only** — no camera on this build |
| Phone UI text stream | Phase 4 |

The zero-echo property survives because **error audio only plays when the session is
already dead.** Order is non-negotiable: tear down session → stop motors → play audio →
restart. Never play audio while the socket is live.

---

## 0. Blocker found in the existing code — read this first

`ArduinoBridge.move()` holds `self._cmd_lock` for the **entire** blocking move
(`arduino_bridge.py` L205, up to `MOVE_TIMEOUT_S`). `stop()` goes through
`_transact()`, which takes the same lock (L254).

**Consequence: any stop issued from another thread blocks until the move finishes.**

Two things follow:

1. **`VisionGuardian` is already broken.** `coordinator.py` L60 calls `self._move.stop()`
   from the guardian thread while the main thread is inside `straight_m`. It blocks on
   `_cmd_lock`, and by the time it acquires it the move is over and the `S` is a no-op.
   The halt path has never worked mid-move. Latent safety bug, independent of this
   project.
2. **`cancel_all()` cannot use `dt.stop()`.** Same trap.

### The fix: out-of-band emergency stop

`_raw_send()` takes only `_write_lock`, not `_cmd_lock` (L326–330) — and the heartbeat
thread already calls it concurrently during moves, so the pattern is proven safe.

Firmware cooperates: `drivetrain.ino` L409–411, on `S` → `if (move_active)
finishMove("STOP")`. The blocked `move()` receives a `D` frame with status `STOP`,
raises `RuntimeError`, and the executor catches it as a cancellation. Clean.

Dedupe is a **single-slot** window (`last_seq`, L387/408) — only the immediately
preceding command is deduplicated. So the e-stop just needs a seq differing from the
in-flight move's. Reserve a band:

```python
# drivetrain/arduino_bridge.py
def emergency_stop(self) -> None:
    """Out-of-band brake. Bypasses _cmd_lock so it works DURING a blocking move."""
    self._estop_seq = 240 + ((self._estop_seq - 239) % 16)   # 240..255, rotating
    self._raw_send(f"S,{self._estop_seq}")
```

and cap the normal counter to 0–239 in `_next_seq()`.

**Task 0.1 — do this before anything else, and fix the guardian while you're in there.**
Test: start a 3 m move, call `emergency_stop()` at t=1 s, assert the move raises within
~200 ms and encoder counts are partial.

---

## Phase 0 — Motion executor + gestures

No network, no audio, no camera. Fully testable today.

### `motion_executor.py`

```python
@dataclass
class MotionEvent:
    job_id: int
    op: str                       # straight | reverse | left | right
    value: float
    status: str                   # done | cancelled | failed
    detail: str = ""

class MotionExecutor:
    def __init__(self, move: ArduinoMovement, history: MovementHistory) -> None: ...
    def start(self) -> None: ...
    def submit(self, op: str, value: float, *, gesture: bool = False) -> int: ...
    def cancel_all(self, reason: str = "") -> list[int]: ...
    def drain(self, timeout: float | None = None) -> bool: ...
    def status(self) -> dict: ...          # moving, queue_depth, last_event
    def events(self) -> "queue.Queue[MotionEvent]": ...
    def close(self) -> None: ...
```

- One worker thread. It is the **only** thread that calls `move()`. This also fixes the
  latent serial race noted above as a side effect.
- `submit()` returns a `job_id` immediately — never blocks.
- `cancel_all()`: clear pending queue → set cancel flag → `bridge.emergency_stop()`
  from the *calling* thread. Deliberately the one concurrent serial touch, and safe
  because it goes via `_write_lock`.
- `gesture=True` jobs **bypass `MovementHistory`**. `apply_voice_word()` pushes every
  move onto `_stack` (`movement_adapter.py` L110); route gestures through it and
  `origin()` will replay your entire conversation backwards. One-line fix, guaranteed
  bug if missed.

### `gestures.py`

```python
YES     = [("straight", 0.1), ("reverse", 0.1)]
NO      = [("left", 30), ("right", 60), ("left", 30)]
UNCLEAR = NO                      # same gesture, per design

class Gesturer:
    def __init__(self, executor: MotionExecutor) -> None: ...
    def play(self, name: str) -> None:   # non-blocking; drops if already gesturing
```

- **Drop, don't queue.** If a gesture is in flight, discard the new one. Queueing turns
  a burst into a 20-second dance.
- Gestures yield to real motion: if `executor.status()["moving"]` is from a non-gesture
  job, skip the gesture entirely. The robot shouldn't nod while driving.

### Gate: measure before proceeding

```python
t = time.time(); dt.right(30); print(time.time() - t)
```

×3 gives your `NO` latency. **Under ~1.2 s: proceed. Over ~2.5 s: the gesture channel is
too slow to be the primary response and Phase 4 becomes mandatory, not optional.**
Record the number in this file before moving on.

**Phase 0 tests**

- `test_cancel_midmove` — 3 m move, cancel at 1 s, partial encoder count.
- `test_gesture_not_in_history` — play 10 gestures, `history.stack` is empty.
- `test_gesture_drop` — 5 rapid `play("YES")`, exactly one executes.
- `test_gesture_yields` — gesture during a 3 m drive is skipped, drive uninterrupted.

---

## Phase 1 — Failure handling, error audio, restart

This is where the design actually lives or dies. Build it before the voice layer so the
voice layer is born supervised.

### `error_audio.py`

**Pre-render the clips; do not call Nix TTS at failure time.** The TTS may be the thing
that failed, and its init is slow. Render once at build time, ship WAVs, play with a
dumb player.

```python
# tools/render_error_clips.py  — build-time, uses Nix TTS
CLIPS = {
    "conn_lost":   "Connection lost. Reconnecting.",
    "conn_failed": "Cannot reach the server. Restarting.",
    "drivetrain":  "Drivetrain error. Restarting.",
    "auth":        "API key rejected.",
    "mic_lost":    "Microphone disconnected.",
}
```

```python
class ErrorAudio:
    def play(self, clip: str) -> None: ...     # blocking, short, best-effort
```

Fallback chain: WAV → 400 Hz sine synthesised inline with numpy → `logger.error` only.
Never raise from this module. It runs when everything else is broken.

**The nasty one:** if the buds disconnect, the buds *are* the output device — you cannot
play "microphone disconnected" through them. `ErrorAudio` must enumerate devices at play
time and fall back to the Pi's onboard sink (3.5 mm / HDMI). If neither exists, log and
move on. Accept that some failures are silent.

### `supervisor.py`

Two restart tiers:

| Tier | Handles | Mechanism |
|---|---|---|
| **In-process reconnect** | transient websocket drop | exponential backoff 1→2→4→8 s, max 4 attempts |
| **Process restart** | everything else | `sys.exit(1)` → systemd `Restart=on-failure` |

**Every path stops the motors first.** Critical subtlety: the firmware link watchdog
(`drivetrain.ino` L570, `finishMove("LINK")`) only fires when the *host process dies*.
An in-process websocket loss leaves the process alive and the heartbeat running — the
watchdog will not save you. Websocket loss is a **new** failure mode with no existing
safety net.

```
on failure:
  1. executor.cancel_all(reason)      # motors first, always
  2. backend.close()                  # session dead => audio is now safe
  3. error_audio.play(clip)
  4. reconnect (tier 1) or sys.exit(1) (tier 2)
```

### Failure taxonomy

| Failure | Detected by | Clip | Action |
|---|---|---|---|
| Websocket dropped | send/recv exc, missed keepalive | `conn_lost` | tier 1 |
| Reconnect exhausted | attempt counter | `conn_failed` | tier 2 |
| Auth / quota | 401 / 429 at connect | `auth` | tier 2, longer `RestartSec` |
| Audio device gone | sounddevice exc | `mic_lost` | tier 2 |
| Serial / Arduino lost | bridge exc | `drivetrain` | tier 2 |
| Bad tool call from model | schema validation | *(none)* | gesture `UNCLEAR`, continue |

### `robot-voice.service` changes

```ini
Restart=on-failure
RestartSec=3
StartLimitIntervalSec=120
StartLimitBurst=5          # stop hammering; surface a real fault instead
```

Also fix the stale header: `User=pi` / `/home/pi/...` vs your actual `roshan151`.

**Phase 1 tests**

- Kill the network mid-session → motors stop, `conn_lost` plays, reconnect succeeds.
- Block the API host → `conn_failed`, exit 1, systemd restarts.
- Unplug the Arduino mid-move → `drivetrain`, clean exit.
- Disconnect the buds → error audio comes out of the onboard sink.

---

## Phase 2 — Voice backend + live session

### `voice_backend.py`

```python
class VoiceBackend(Protocol):
    def connect(self) -> None: ...
    def send_audio(self, pcm16: bytes) -> None: ...
    def send_frame(self, jpeg: bytes) -> None: ...   # Phase 5 stub, no-op
    def on_text(self, cb: Callable[[str, bool], None]) -> None: ...   # (text, final)
    def on_tool_call(self, cb: Callable[[str, dict], None]) -> None: ...
    def send_tool_result(self, call_id: str, result: dict) -> None: ...
    def close(self) -> None: ...
```

`GeminiLiveBackend` first, `response_modalities=["TEXT"]`. Text output roughly halves
cost and removes the TTS path entirely — which is what preserves zero-echo.

Audio in: 16 kHz PCM16 mono. Run `check_bt_audio.sh` first; if the buds sit at 8 kHz
CVSD, upsample at the edge and expect degraded recognition on similar-sounding numbers.

### `robot_tools.py`

Keep the surface small — every tool is context re-read each turn.

| Tool | Returns | Blocking |
|---|---|---|
| `drive(meters)` | `{job_id, status:"started"}` | no |
| `turn(degrees)` | `{job_id, status:"started"}` | no |
| `stop()` | `{status:"stopped", cancelled:[...]}` | immediate, pre-empts |
| `get_status()` | moving, queue depth, last event, position estimate | immediate |
| `return_to_origin(steps=None)` | `{job_id}` | no |
| `answer(value)` | `{ok:true}` | fires `YES` / `NO` gesture |
| `look()` / `find_object(name)` | `{"error":"no camera on this build"}` | **stub** |

`answer(value: "yes" | "no" | "unclear")` is how the model reaches the gesture channel.
System prompt must state plainly: *you have no voice; the only way to reply is `answer`
or a movement tool; if you did not understand, call `answer("unclear")`.*

Clamp `drive` to a max metres **in the tool layer, not the prompt.**

### `live_session.py`

Glue: duplex audio thread → backend → tool dispatch → executor. Feeds `MotionEvent`s
back as tool results so the model knows a move finished. Owns the supervisor loop.

### `vision_stub.py`

```python
class NullVision:
    """Placeholder until a camera exists. Same surface as RobotVision."""
    available = False
    def start(self): pass
    def stop(self): pass
    def capture_file(self, path=None): raise VisionUnavailable()
    def detect_objects(self, objects): raise VisionUnavailable()
    def scan_for_objects(self, objects, move): raise VisionUnavailable()
```

Guardian stays disabled (`--voice-only` already does this). `RobotVision` and
`VisionGuardian` untouched — Phase 5 swaps `NullVision` out and nothing else changes.

**Phase 2 tests**

- "move forward two meters" → `drive(2.0)` → `MotionEvent(done)` → tool result returned.
- "are you there?" → `answer("yes")` → `YES` gesture.
- Garbled input → `answer("unclear")` → `NO` gesture.
- "stop" mid-drive → motors halt < 500 ms (via `emergency_stop`).

---

## Phase 3 — Local keyword spotter (safety)

`openWakeWord` on the raw mic stream, bypassing the model entirely. Round-tripping an
emergency stop through a cloud model at 800 ms is not a safety mechanism.

`keyword_spotter.py` → on hit → `executor.cancel_all("wake word")`. ~5–10% of one Pi 4B
core. Independent of the websocket, so it works while disconnected — which is exactly
when you need it.

---

## Phase 4 — Phone UI text stream

FastAPI + websocket, one static page. Robot pushes model text as it streams. Makes the
robot's output channel actually usable; the gestures become eyes-free confirmation
rather than the sole channel.

Add a **heartbeat indicator** — it's the only honest signal that the socket is alive.
Without it, "no gesture" is ambiguous between *thinking*, *no*, and *dead*, and you've
explicitly assigned no gesture to thinking.

`phone_ui.py` — `/` static page, `/ws` stream, bound to the Pi's LAN address.

---

## Phase 5 — Vision (deferred)

Swap `NullVision` → `RobotVision`, un-stub `look()` / `find_object()`, re-enable
`VisionGuardian` (**after** the Phase 0 guardian fix), add `send_frame()` at ~1 fps.
No other module changes.

---

## Build order summary

| Phase | Deliverable | Needs |
|---|---|---|
| **0.1** | `emergency_stop()` + guardian fix | nothing |
| **0.2** | `MotionExecutor`, `Gesturer`, latency measurement | 0.1 |
| **1** | `ErrorAudio`, `supervisor`, systemd hardening | 0.2 |
| **2** | `VoiceBackend`, `robot_tools`, `live_session`, `vision_stub` | 1 |
| **3** | `keyword_spotter` | 2 |
| **4** | `phone_ui` | 2 |
| **5** | Vision | camera |
