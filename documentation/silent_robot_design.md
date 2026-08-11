# Design review — silent robot, kinesthetic yes/no

Proposal: robot never speaks. Listens only. Responds by moving.
- **Yes** = forward 0.1 m, back 0.1 m
- **No** = left 30°, right 60°, left 30° (net zero)

---

## 1. The good part: this does kill Phase 0

No speaker output means **no echo path**. Every problem in `phase0_audio.md` — AEC,
clock drift, uplink gating, self-transcription loops, ERLE targets — evaporates. Not
mitigated, *structurally absent*. That's a real architectural win and the instinct
behind it is correct.

It also removes the barge-in requirement, because there's nothing to barge in on.
(You still want the local "stop" keyword spotter for **safety** — that requirement is
independent and doesn't go away.)

## 2. Will it work with the JBL buds? Mostly yes — check one thing

| Concern | Verdict |
|---|---|
| Echo | **Gone.** No playback. |
| Sample rate | **Check this.** See below. |
| Latency | HFP adds ~20–30 ms codec delay, now on the input path only. Acceptable. |
| WiFi/BT radio contention on Pi 4B | Still real, but you've halved the BT traffic. Monitor for dropouts. |
| Mic quality | **Better than a robot-mounted mic.** See below. |

### Check for mSBC before accepting 8 kHz

You cannot escape HFP — A2DP is sink-only and carries no microphone. So the mic forces
a headset profile regardless of the fact that you no longer need playback.

But HFP has two codecs:

- **CVSD** — 8 kHz narrowband. This is what your notes pin
  (`headset-head-unit-cvsd`).
- **mSBC** — 16 kHz wideband ("HD Voice"), available on HFP 1.6+.

16 kHz is exactly what the Live API wants for input. Check whether the buds negotiate it:

```bash
pactl list cards | grep -A40 bluez_card
# look for a profile named headset-head-unit-msbc
sudo -u '#1000' XDG_RUNTIME_DIR=/run/user/1000 \
  pactl set-card-profile bluez_card.B4_84_D5_B9_14_23 headset-head-unit-msbc
```

If it's there, switch and you're at native rate. If it isn't, you're upsampling 8 kHz
and every consonant above 4 kHz is already gone — expect noticeably worse recognition
on similar-sounding commands ("fifteen"/"sixteen", "left"/"left thirty").

### An upside worth naming

Earbuds are worn by *you*, not bolted to the robot. That means the mic sits at your
mouth: high SNR, no drivetrain noise, no room reverb, no distance falloff. For
recognition accuracy this beats a robot-mounted array. The buds stop being a compromise
and start being a reasonable operator-headset design — the RC-pilot model.

## 3. The bigger consequence: text-only output changes the architecture

Both providers support audio-in / text-out (`response_modalities=["TEXT"]` on Gemini
Live; `modalities: ["text"]` on OpenAI Realtime). Two effects:

**Cost drops sharply.** Output audio is the expensive half — $64/M vs $32/M on OpenAI,
and assistant audio bills at 1200 tok/min vs 600 tok/min inbound. Dropping it takes you
from ~$0.05/min to ~$0.02/min on `gpt-realtime-2.1`.

**But it removes most of the reason to use a Live API at all.** The native-audio models'
headline feature is natural speech *output*. If the robot never speaks, what you're
actually buying is streaming STT plus tool-calling — and a cascaded
**Deepgram/AssemblyAI streaming STT → Gemini Flash with tools** pipeline does that for
~$0.01/min with comparable latency and no websocket-audio complexity.

The case for keeping a Live API narrows to: robust understanding of noisy/accented
audio, prosody, and built-in turn detection. With an 8 kHz CVSD link, that advantage
narrows further — you're feeding a native-audio model degraded audio.

**Recommendation:** if the robot never speaks, re-evaluate cascaded. It is cheaper,
simpler, and the thing you gave up was the half you weren't using.

---

## 4. Where the gesture protocol breaks

The "never speaks" half is sound. The "responds by moving" half has five problems, in
descending severity.

### 4.1 Latency — probably fatal, measure it first

A "no" is **three** encoder-counted moves. Each carries a serial command round trip,
physical motion, and `ENCODER_SETTLE_S` (0.15 s) before the count can be read. A "yes"
is two.

Spoken "yes" is ~300 ms. If a gestural "no" takes 3–5 s you've made the response channel
**ten times slower than the question channel**, and conversation dies on that alone.

**Measure before building anything else:**

```python
t = time.time(); dt.right(30); print(time.time() - t)
```

Multiply by 3. If it's under ~1.2 s total, the design is viable. Over ~2.5 s, it isn't.
This single number decides the design — get it before writing any of it.

### 4.2 One bit is not enough states

You need at least four, and silence currently means all of them:

| Meaning | Currently signalled by |
|---|---|
| Yes | forward/back |
| No | left/right/left |
| Didn't understand you | *nothing* |
| Working on it / thinking | *nothing* |
| Websocket dropped, I'm deaf | *nothing* |

The last one matters most: **a disconnected robot is indistinguishable from a robot
saying "no."** That's not a UX wrinkle, it's a safety property. You need a distinct
"alive and listening" signal that isn't the absence of motion.

### 4.3 Gesture motion corrupts odometry

`left 30 → right 60 → left 30` is net zero on paper. On carpet it isn't. At
`TICKS_PER_DEGREE = 3.5`, a 30° turn is 105 ticks, and each turn contributes slip and
quantisation error that **compounds across three turns**. `MovementHistory` and
`origin()` are dead-reckoning; every "no" injects heading error into your position
estimate. Twenty exchanges in, "return to origin" is meaningfully wrong.

Turns are the worst possible gesture primitive for this — turning is where wheel slip is
largest. If you keep motion gestures, prefer the forward/back pattern for both and
distinguish them some other way (magnitude, repetition count).

### 4.4 Concrete bug: gestures must bypass `MovementHistory`

`MovementHistory.apply_voice_word()` pushes every move onto `_stack` (movement_adapter.py
L110). Route gestures through it and `origin()` will dutifully try to **undo your
conversation**, replaying every yes and no backwards.

Gestures must go straight to `ArduinoMovement`, never through `MovementHistory`. Easy to
miss, guaranteed to bite.

### 4.5 The actuator is also the display

- The robot can't gesture while driving somewhere — the channels collide.
- "Yes" = drive forward 0.1 m. In a tight space, **agreeing with you is a collision
  risk.** The gesture vocabulary has to be pre-empted by the VisionGuardian, which means
  a halt can silently truncate a "yes" into a "maybe."
- Hundreds of motor start/stops per session, purely for signalling. Drivetrain wear and
  battery for zero locomotion.

---

## 5. Keep the idea, change the channel

The valuable insight is **"robot produces no speech."** Keep it. The part to reconsider
is using *motion* as the output channel. Better options, all echo-free:

| Channel | Latency | Bandwidth | Cost | Notes |
|---|---|---|---|---|
| **Text to a phone/browser UI** | ~0 | Full | $0 | Robot streams responses to a webpage. "What do you see?" becomes answerable. Strictly the best option. |
| **LED ring** | ~0 | ~4–8 states | ~$5 | Perfect for listening / thinking / yes / no / **disconnected**. Solves §4.2 outright. |
| **Non-speech tones** | ~0 | Few states | ~$3 | The R2-D2 model. Reintroduces a *tiny* echo path, but a 100 ms beep is trivially gated — you know exactly when it fires and it isn't speech, so VAD rarely trips. |
| **Motion gestures** | 1.5–5 s | 1 bit | wear | Keep for emphasis when the operator isn't looking at a screen. Not as the primary channel. |

**Suggested target design:**

- Robot never speaks. (Phase 0 stays dead.)
- Buds as operator headset — switch to mSBC if available.
- Audio in → streaming STT → LLM with tools → **text out to phone UI**.
- LED ring for instant state, including a heartbeat that proves the socket is alive.
- Motion gestures optional, for eyes-free acknowledgment only, bypassing
  `MovementHistory`.
- Local "stop" keyword spotter regardless — safety requirement, unchanged.
