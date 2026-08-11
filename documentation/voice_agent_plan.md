# Free-form voice agent — design plan

Target: continuous spoken conversation with the robot, where the model calls tools
(`drive`, `turn`, `stop`, `look`, `find`) instead of emitting a movement-JSON plan.
Audio session runs on the Pi. Moves are async and interruptible.

---

## 1. Why the current design can't get there

`voice_session.py` is a **command parser**, not an agent. Six structural blockers:

| # | Blocker | Where | Effect |
|---|---------|-------|--------|
| 1 | Half-duplex turn loop | `listen_once()` | `adjust_for_ambient_noise` + "Speak your command" every turn. ~1–2 s of dead air per exchange. No barge-in. |
| 2 | Batch STT | `recognize_google(audio)` L157 | Nothing is sent until `listen()` detects end-of-phrase. Adds 0.5–1.5 s and can't stream partials. |
| 3 | Schema-locked output | `response_format={"type":"json_object"}` + `JSON_SUFFIX` | Every utterance must become a movement plan. "How far did you go?" has no valid answer. |
| 4 | Blocking serial execution | `_dispatch_step` → `straight_m` | Encoder moves block up to `MOVE_TIMEOUT_S` (20 s). Mic is dead the whole time — the robot cannot hear "stop". |
| 5 | Blocking TTS | `speak()` → `sd.wait()` | Can't be interrupted mid-sentence. |
| 6 | No state feedback | `messages` only accumulates raw JSON | Model never learns whether a move succeeded, actual encoder distance, or what the camera saw. It's planning blind. |

The `VisionGuardian` is currently the *only* thing that can halt a move in flight, and
it fires on a hardcoded class list — not on your voice.

**Verdict:** keep the glossary path as a deterministic offline fallback; build the
conversational path as a parallel stack.

---

## 2. Provider comparison

Prices and latencies as of Aug 2026. **Latency figures across published benchmarks
disagree badly** (see the Gemini row) — treat the table as a shortlist, then measure
on your own Pi and network before committing.

| Option | Audio in / out | ~Cost per conversation-minute¹ | Reported response latency² | Notes |
|---|---|---|---|---|
| **Gemini Live** (`gemini-3.1-flash-live`, native audio) | 25 tok/s both directions | **~$0.04** (Google quotes ~$0.0368/min effective) | 320 ms p50 / 780 ms p95 in one report; **2.98 s** end-to-end TTFT in another | Free tier for prototyping. 90.8% on ComplexFuncBench Audio (spoken → correct function call). Native video-frame input. |
| **OpenAI Realtime** `gpt-realtime-mini` | $10 / $20 per M | **~$0.015 base**, ~$0.04–0.08 real | ~0.8 s | Cheapest cloud native-audio option. |
| **OpenAI Realtime** `gpt-realtime-2.1` | $32 / $64 per M ($0.40 cached in) | **~$0.05 base**, **$0.15–0.30 real uncached** | ~0.82 s (gpt-realtime-1.5) | Best reasoning, worst cost profile. Cached-input hit rate is the whole ballgame. |
| **Cascaded** (Deepgram Nova-3 → Gemini Flash → Cartesia Sonic) | per-service | ~$0.01–0.03 | 450–850 ms best case (150 STT + 200 LLM + 40 TTS + network) | Cheapest, most control, three vendors to babysit, you implement VAD/barge-in yourself. |
| **Fully local** (whisper.cpp + local LLM + Piper/Nix) | — | $0 | 2–5 s on Pi 5; LLM dominates | Only viable if you offload the LLM to a LAN box. Not conversational on-Pi. |

¹ Assumes 30 s user speech + 30 s robot speech per minute. **The number that will
actually bite you is context replay**: realtime APIs re-send accumulated audio context
each turn, so uncached cost grows superlinearly with session length. Budget for
session truncation/summarization after ~5 minutes regardless of provider.

² "Time from user stops speaking → first audio out."

**Recommendation:** start on **Gemini Live** (free tier, native audio, strongest
published spoken-function-calling score, native video frames — which matters for your
vision path). Put it behind a `VoiceBackend` protocol so OpenAI Realtime is a swap, not
a rewrite. You already have `OPENAI_API_KEY_ROBIN` wired, so the fallback is cheap.

---

## 3. Target architecture

```
  mic ──► AEC ──► LiveSession (websocket, native audio)
                        │  ▲
             tool calls │  │ tool results + async completion events
                        ▼  │
                    RobotTools
                        │
        ┌───────────────┼────────────────┐
        ▼               ▼                ▼
  MotionExecutor    RobotVision     MovementHistory
  (worker thread,   (frames to      (origin/backtrack)
   cancellable)      session)
        │
   SerialDrivetrain ──► Arduino
        ▲
  VisionGuardian (unchanged, independent halt path)
```

### New modules

**`motion_executor.py`** — the load-bearing piece. Single worker thread owning the
drivetrain, fed by a queue.

- `submit(op) -> job_id`, returns immediately.
- `cancel_all()` sets a cancel flag *and* calls `dt.stop()` — must preempt the queue,
  not wait for it.
- Publishes completion events (`{job_id, status, actual_meters, encoder_skew}`) on an
  out-queue.
- Guarantees exactly one thread touches the serial port. Today `VisionGuardian` calls
  `move.stop()` from its own thread while the main thread is mid-`straight_m` — that's
  a latent framing race on the serial link. Routing everything through the executor
  fixes it as a side effect.

**`robot_tools.py`** — tool schemas + dispatch. Proposed surface:

| Tool | Returns | Blocking? |
|---|---|---|
| `drive(meters)` | `{job_id, status:"started"}` | no |
| `turn(degrees)` | `{job_id, status:"started"}` | no |
| `stop()` | `{status:"stopped", cancelled:[job_ids]}` | immediate, preempts |
| `get_status()` | position estimate, moving?, queue depth, last encoder reading | immediate |
| `look()` | scene description from a fresh frame | ~1 s |
| `find_object(name)` | bool + bearing | uses `scan_for_objects` |
| `return_to_origin(steps=None)` | job_id | no |

Keep the tool count small. Every tool is context the model re-reads each turn.

**`voice_backend.py`** — protocol: `connect()`, `send_audio(pcm)`, `send_frame(jpg)`,
`on_tool_call(cb)`, `send_tool_result()`, `interrupt()`, `close()`.
Implementations: `GeminiLiveBackend`, `OpenAIRealtimeBackend`.

**`live_session.py`** — glue. Duplex audio streams, routes tool calls to `RobotTools`,
pumps async completion events back in as tool results so the model can volunteer
"I've finished the two meters."

---

## 4. Phased build

**Phase 0 — Pi audio (do this first, it's the biggest risk and it's provider-agnostic)**

Full-duplex audio on a Pi is where these projects die. Full treatment in
**[`phase0_audio.md`](phase0_audio.md)**. Summary:

- Drop the Bluetooth HFP path — 8 kHz narrowband, and BT shares the Pi 4B radio with
  the websocket.
- Gate the uplink while TTS plays, and recover barge-in with a **local keyword spotter**
  exempt from the gate. You need that spotter for the safety stop anyway.
- Buy hardware AEC (ReSpeaker Mic Array v2.0 / USB speakerphone). The 2-Mic and 4-Mic
  HATs have no AEC.
- Software AEC only with a single clock domain, or it drifts out of lock in minutes.

Deliverable: acceptance tests T1–T5 in that doc pass, motors running. Don't move on
until they do.

**Phase 1 — `MotionExecutor`.** No voice involved. Test: submit a 3 m drive, call
`cancel_all()` at t=1 s, assert the Arduino braked and the encoder count is partial.

**Phase 2 — `RobotTools`** over the executor, exercised from a text REPL. Validates
schemas and the feedback loop before adding audio's failure modes.

**Phase 3 — `GeminiLiveBackend`** + `live_session.py`. Measure real TTFT on your Pi.

**Phase 4 — vision in-conversation.** Push a frame to the Live session on `look()` and
optionally at 1 fps while moving. Keep the FastAPI detector for `VisionGuardian` — it's
deterministic and fast, and you don't want the halt path depending on a websocket.

**Phase 5 — safety.**

- Websocket drop / heartbeat loss → `cancel_all()` + motors off. Non-negotiable.
- `stop` must be reachable without the model: keep a local wake-word or keyword-spot
  on "stop" that bypasses the LLM entirely. Round-tripping an emergency stop through a
  cloud model at 800 ms is not a safety mechanism.
- Cap `drive` at a sane max meters in the tool layer, not in the prompt.
- Keep `run_robot.py --glossary` working offline.

---

## 5. Open questions

- Is `Recognizer.recognize_google` usage anywhere else? (Only `listen_once`; safe to
  leave the whole path intact as fallback.)
- Nix TTS: keep for the offline fallback path only — native-audio providers do their
  own synthesis and mixing two voices will feel incoherent.
- Session length policy: at what turn count do you truncate audio context? Pick a
  number in Phase 3 once you can see the token meter.
