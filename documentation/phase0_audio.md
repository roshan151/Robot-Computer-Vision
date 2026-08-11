# Phase 0 — full-duplex audio on the Pi 4B

The problem in one line: the speaker feeds the mic, the provider's VAD hears the
robot's own voice, and the agent interrupts itself in a loop.

---

## 0. Start by killing the Bluetooth path

Current setup (`documentation/tf-luna-and-bluetooth.txt`) is JBL buds pinned to
`headset-head-unit-cvsd`. For a realtime voice agent this is disqualifying on three
counts:

- **HFP/CVSD is 8 kHz narrowband mono.** Gemini Live and OpenAI Realtime both want
  16 kHz PCM in. Upsampling 8 kHz recovers nothing — you've thrown away every
  consonant above 4 kHz before the model ever sees it.
- **Pi 4B shares one radio between WiFi and Bluetooth.** Your realtime websocket rides
  the same antenna as the audio. Under load you get both dropouts and BT audio stutter.
- **HFP adds 100–200 ms** round trip on top of a latency budget you're already fighting
  for.

Go USB. Keep the buds for debugging over SSH if you like.

---

## 1. Three failure modes, often conflated

| Mode | Symptom | Real cause |
|---|---|---|
| **False barge-in** | Robot stops mid-sentence for no reason | Provider VAD triggers on the robot's own voice arriving through the mic |
| **Self-transcription loop** | Robot answers itself, escalating | Its own speech is transcribed as user turn; model responds; repeat |
| **Howling** | Rising squeal | Acoustic gain loop; volume + proximity |

Modes 1 and 2 are the ones that kill projects. Both are solved by the same thing:
**the upstream audio stream must not contain the robot's own voice.**

---

## 2. Solution tiers, in order of what I'd actually do

### Tier A — Gate the uplink + local keyword spotter (do this regardless)

The cheapest robust fix isn't AEC at all. While the robot is speaking, **stop sending
mic audio upstream**. Re-open the gate after the playback buffer drains plus ~150 ms of
room-reverb tail.

That costs you barge-in — unacceptable for a robot you need to shout "stop" at. So
recover barge-in with a **local wake-word/keyword spotter that runs on the raw mic
stream and is exempt from the gate**:

```
mic ──┬──► [gate: open unless TTS playing] ──► provider websocket
      └──► [openWakeWord: "stop" / "hey rover"] ──► cancel_all() + kill playback + open gate
```

This is the piece worth internalising: **you already need that local keyword spotter for
safety.** Routing an emergency stop through a cloud model at 800 ms is not a safety
mechanism. Building it for safety buys you barge-in for free, and vice versa. One
component, two requirements, no AEC tuning.

- `openWakeWord` (Apache-2.0, ONNX) runs a custom word in roughly 5–10% of one Pi 4B
  core. `porcupine` is lighter but the free tier is licence-restricted.
- **Pick a wake word the robot itself never says** — "hey rover", not "stop" — or the
  spotter self-triggers off the speaker. If you want literal "stop" to work, suppress
  the spotter for any TTS utterance whose text contains it.
- Gate on *playback buffer state*, not on "I called speak()". With native-audio
  providers the audio arrives in chunks over the socket; track when the last chunk has
  actually drained from the ALSA buffer.

Ship this first. It works on any hardware, including the buds.

### Tier B — Hardware AEC (the real fix; ~$50–70)

An echo canceller needs a **reference signal** of what's being played, tightly
clock-aligned with the capture. The clean way to get that is a device that does both
directions on one chip.

| Device | AEC? | Notes |
|---|---|---|
| **ReSpeaker Mic Array v2.0** (XMOS XVF-3000, USB) | **Yes, on-chip** | AEC + beamforming + dereverb + noise suppression, all on the XMOS chip. Zero Pi CPU. Seeed's own docs note it removes the need for PulseAudio AEC, which cost 50–60% CPU on a Pi. ~$70. |
| **USB conference speakerphone** (Anker PowerConf, Jabra Speak 410/510) | **Yes** | Underrated for robots: one USB device, mic + speaker + hardware AEC + single clock domain, all solved. $40–60 used. |
| ReSpeaker 2-Mic Pi HAT | **No** | Common misconception. It's 2 mics + a WM8960 codec. No AEC block. |
| ReSpeaker 4-Mic Array (HAT) | **No** | Beamforming only, no playback reference. |

If you buy one thing for this project, buy from row 1 or 2. Phase 0 mostly evaporates.

### Tier C — Software AEC (only if you refuse to buy hardware)

PipeWire `module-echo-cancel` with `aec_method=webrtc` (WebRTC AEC3 — the same
canceller Chrome ships, so the algorithm is not the weak link).

Note Pi OS Bookworm defaults to **PipeWire**, not PulseAudio; your notes are written
against Pulse, so check which you're actually on before debugging the wrong stack.

Three conditions, all mandatory:

1. **One clock domain.** Capture and playback must share a crystal. A USB mic plus a
   separate 3.5 mm or USB speaker means two independent oscillators; they drift, the
   echo-path delay wanders, and AEC3 loses lock — typically fine for 30 s, broken by
   5 minutes. This is the single most common reason software AEC "doesn't work on Pi."
   Use one USB codec that does mic-in *and* line-out.
2. **Low, stable buffer latency.** Small fixed ALSA period size. Also pin
   `node.force-quantum` — variable quantum causes the crackling that's widely reported
   against this module.
3. **A linear echo path.** A clipping speaker makes the echo nonlinear and AEC
   mathematically cannot cancel it. Keep volume off the rails. This is why "it works
   quiet, breaks loud" is such a common report.

### Tier D — Physical (free, do it anyway)

- Point the mic away from the speaker; put the chassis between them if you can.
- **Mount the speaker on rubber grommets.** A robot chassis is an excellent sound
  conductor — mechanical coupling through the frame bypasses every acoustic fix above
  and it's easy to miss because it doesn't look like an acoustic problem.
- Directional/cardioid mic over omni.
- Lower the volume. Half the battle.

---

## 3. Acceptance tests — don't leave Phase 0 without these

Write these as scripts; you'll rerun them every time you change hardware.

**T1 — Self-trigger rate.** Empty room. Play 30 s of TTS. Count upstream VAD triggers.
**Target: 0.** This is the gating test.

**T2 — ERLE.** Play known audio, record the mic, compute
`10 * log10(power_before_AEC / power_after_AEC)`. **Target: > 25 dB.** Only meaningful
for Tier B/C; skip if you went pure-gating.

**T3 — Barge-in latency.** While the robot is speaking, say the wake word at normal
volume from 1 m. Measure wall-clock to motors halted. **Target: < 500 ms.**
Measure this with the motors actually running — motor and gearbox noise is a real
signal-to-noise problem the desk test won't show you.

**T4 — Drift soak.** 10 minutes of continuous alternating speech. An AEC that passes T2
at 30 s and fails at 5 minutes is telling you about clock drift, not about the
algorithm. This is the test that catches Tier C's failure mode, and it's the one people
skip.

**T5 — Motors on.** Rerun T1 and T3 while driving. Drivetrain noise floor changes
everything, and your mic is bolted to the noise source.

---

## 4. Recommended path

1. Order a USB speakerphone or ReSpeaker Mic Array v2.0. **(Tier B)**
2. While it ships, build the gate + `openWakeWord` spotter — you need it for the safety
   stop anyway, and it's provider-agnostic. **(Tier A)**
3. Rubber-mount the speaker. **(Tier D)**
4. Run T1–T5. Only then start Phase 1.

Skip Tier C entirely unless the hardware budget is zero — and if you do go there, the
one-clock-domain rule is not optional.
