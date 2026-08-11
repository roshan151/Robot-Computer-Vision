"""
Voice → movement, with an Arduino drivetrain backend.

Audio goes to Gemini in a single request that returns the movement plan
directly. Voice activity detection and endpointing stay on-device; only the
understanding is remote.

Optional: Picamera2 / OpenCV (no camera on this build), Nix TTS.
"""

from __future__ import annotations

import logging
import os
import sys
import time
from typing import Any, Dict, List, Optional

import config
import robot_log
from audio_cues import cues
import audio_gate
from gemini_client import (
    GeminiError,
    GeminiRateLimited,
    GeminiVoicePlanner,
    to_legacy_steps,
)
from gestures import SyncGesturer

from prompts_and_glossary import (
    audio_movement_prompt,
    commands as glossary_commands,
)

from movement_adapter import ArduinoMovement, MovementHistory
from vision_client import RobotVision

logger = logging.getLogger(__name__)

VISION_STEP_KEYS = {"capture", "record", "detect", "scan", "terminate"}
ORIGIN_KEYS = set(glossary_commands.get("origin", [])) | {"origin"}
TERMINATE_WORDS = {t.lower() for t in glossary_commands.get("terminate", [])} | {"terminate"}


class _NoSpeech(Exception):
    """Nobody spoke inside the listen window.

    Deliberately not a RuntimeError: silence is the idle state, not a fault,
    and conflating the two is what produced the old 'Speak your command' /
    'No speech heard' loop every ten seconds.
    """


class VoiceRobotSession:
    def __init__(
        self,
        move: ArduinoMovement,
        history: MovementHistory,
        vision: Optional[RobotVision],
        model: Optional[str] = None,
    ) -> None:
        self.move = move
        self.history = history
        self.vision = vision
        # Gemini is the only backend: it takes the microphone audio and the
        # planning prompt in a single request. config validates the toggle at
        # import, so reaching here means it is "gemini".
        self._planner = GeminiVoicePlanner(
            model=model or config.GEMINI_MODEL,
            system_prompt=audio_movement_prompt,
        )
        # The robot's only way to answer a question. Deliberately NOT routed
        # through MovementHistory — see SyncGesturer.
        self._gestures = SyncGesturer(move)
        self._nix = None
        self._sd = None
        self._sr = None
        self._recognizer = None
        self._mic = None
        self._calibrated = False
        self._noise_dropped = 0
        self._cues = cues()
        self._init_audio()

    def _init_audio(self) -> None:
        try:
            import sounddevice as sd  # type: ignore

            self._sd = sd
        except Exception as e:
            logger.warning("sounddevice unavailable: %s", e)
        try:
            import speech_recognition as sr  # type: ignore

            self._sr = sr
            self._recognizer = sr.Recognizer()
            self._mic = sr.Microphone()
        except Exception as e:
            logger.warning("speech_recognition unavailable: %s", e)
        nix_dir = config.NIX_TTS_DIR
        if nix_dir and os.path.isdir(nix_dir):
            try:
                if nix_dir not in sys.path:
                    sys.path.insert(0, nix_dir)
                from nix.models.TTS import NixTTSInference  # type: ignore

                model_dir = config.NIX_TTS_MODEL or nix_dir
                self._nix = NixTTSInference(model_dir=model_dir)
                self._samplerate = 22050
            except Exception as e:
                logger.warning("Nix TTS unavailable: %s", e)

    def speak(self, text: str) -> None:
        """Report something to the operator.

        The robot is silent by design, and it runs headless — so by default
        this neither prints nor speaks. The text goes to logs.json, which is
        where you would look for it anyway.

        Printing was actively harmful: with no screen attached it produced
        nothing but a growing journal, and the message it emitted most often
        was a prompt to talk that the operator could not see.

        Set ROBOT_SPEECH=1 to restore spoken output via Nix TTS — useful at a
        desk, but note it puts the robot's voice back into the microphone.
        """
        robot_log.event("voice.say", text=text)
        if not config.ROBOT_SPEECH_ENABLED:
            return
        if self._nix is not None and self._sd is not None:
            try:
                c, c_length, _phoneme = self._nix.tokenize(text)
                xw = self._nix.vocalize(c, c_length)
                self._sd.play(xw[0, 0], self._samplerate)
                self._sd.wait()
            except Exception as e:
                robot_log.event("audio.error", logging.WARNING,
                                stage="tts", err=f"{type(e).__name__}: {e}")

    def plan(self, wav_bytes: bytes) -> Dict[str, Any]:
        """Audio in, plan out — one request, no separate transcription step."""
        return self._planner.plan(wav_bytes)

    def calibrate(self) -> None:
        """Measure the ambient noise floor once, at startup.

        This used to run on every turn, costing LISTEN_CALIBRATE_S of dead air
        before each command. Once is enough because dynamic_energy_threshold
        keeps adapting the threshold as the session runs.
        """
        if not self._mic or not self._recognizer or self._calibrated:
            return
        try:
            with self._mic as source:
                self._recognizer.adjust_for_ambient_noise(
                    source, duration=config.LISTEN_CALIBRATE_S
                )
            self._recognizer.dynamic_energy_threshold = True
            # Set BEFORE the optional tuning below: if that throws, we must not
            # fall through to re-calibrating on every single turn, which would
            # silently add a second of dead air per command.
            self._calibrated = True

            # Silence that ends an utterance. The operator waits through this
            # on every command before the request is even sent, so it is felt
            # as latency exactly like the API call is. It also trims trailing
            # silence off the upload.
            self._recognizer.pause_threshold = config.LISTEN_PAUSE_S
            self._recognizer.non_speaking_duration = min(
                getattr(self._recognizer, "non_speaking_duration",
                        config.LISTEN_PAUSE_S),
                config.LISTEN_PAUSE_S,
            )
            robot_log.event("voice.calibrate",
                            threshold=round(self._recognizer.energy_threshold, 1),
                            pause_s=config.LISTEN_PAUSE_S)
        except Exception as e:
            robot_log.event("audio.error", logging.WARNING,
                            stage="calibrate", err=f"{type(e).__name__}: {e}")

    def listen_once(self) -> bytes:
        """Wait for one spoken command; return it as WAV bytes.

        Note what stays local: `Recognizer.listen()` is doing voice activity
        detection and endpointing, deciding when the utterance began and ended.
        Only the *transcription* moved to Gemini — endpointing on-device is
        what keeps a silent room from uploading anything at all.

        Two ordering rules matter here:

        1. The ready cue plays BEFORE the capture stream opens. Playing it with
           the stream live would put the tone in the buffer that listen() then
           reads, and the robot would hear its own beep as the start of the
           command.
        2. Ambient calibration happens once in calibrate(), not per turn.

        A silence timeout is not an error and not a prompt — the caller re-arms
        without any output at all.
        """
        if not self._sr or not self._mic or not self._recognizer:
            raise RuntimeError("microphone / speech_recognition not available")

        self.calibrate()

        # dynamic_energy_threshold keeps adapting downward in a quiet room,
        # with no lower bound, until the microphone wakes on nothing. Clamp it
        # back up before every listen — one line, and it removes most of the
        # spurious triggers at source rather than filtering them later.
        if self._recognizer.energy_threshold < config.LISTEN_MIN_ENERGY:
            self._recognizer.energy_threshold = config.LISTEN_MIN_ENERGY

        self._cues.ready()          # outside the `with` block, deliberately

        with self._mic as source:
            try:
                audio = self._recognizer.listen(
                    source,
                    timeout=config.LISTEN_TIMEOUT_S or None,
                    phrase_time_limit=config.LISTEN_PHRASE_LIMIT_S,
                )
            except self._sr.WaitTimeoutError as e:
                raise _NoSpeech("no speech within listen window") from e

        # 16 kHz mono PCM WAV — exactly what Gemini wants, and it downsamples
        # to 16 kbps mono anyway, so sending more is wasted Pi WiFi upload.
        return audio.get_wav_data(
            convert_rate=config.GEMINI_AUDIO_RATE, convert_width=2
        )

    def _dispatch_step(self, step: Dict[str, Any]) -> bool:
        """
        Execute one planner step. Returns True if the voice session should exit
        (terminate / shut down vision session).
        """
        word, value = next(iter(step.items()))
        word_l = word.lower()

        if word_l in TERMINATE_WORDS:
            self._vision_step("terminate", None)
            return True

        if word_l in glossary_commands["movement"]:
            self.history.apply_voice_word(word_l, value)
            return False

        if word_l in ORIGIN_KEYS:
            self.history.origin(value)
            self.history.clear()
            return False

        if word_l in VISION_STEP_KEYS:
            self._vision_step(word_l, value)
            return False

        raise ValueError(f"unknown step key: {word}")

    def _vision_step(self, key: str, value: Any) -> None:
        if self.vision is None:
            self.speak("Vision is not available on this session.")
            return

        if key == "terminate":
            self.vision.stop()
            self.speak("Camera stopped.")
            return

        if self._vision_needs_start():
            self.vision.start()

        if key == "capture":
            path = self.vision.capture_file()
            self.speak(f"Saved image {path}")
            return

        if key == "record":
            self.speak("Video recording is not implemented in the robust stack yet.")
            return

        if key == "detect":
            objects = value if isinstance(value, list) else ["plant", "plants", "leaf", "leaves"]
            found = self.vision.detect_objects(objects)
            self.speak("Objects found." if found else "Objects not found.")
            return

        if key == "scan":
            objects = value if isinstance(value, list) else ["plant", "plants"]
            self.vision.scan_for_objects(objects, self.move)
            self.speak("Scan complete.")
            return

    def _vision_needs_start(self) -> bool:
        return self.vision._picam is None and self.vision._cv2 is None  # type: ignore[attr-defined]

    def run(self) -> None:
        listening = True
        while listening:
            try:
                wav = self.listen_once()
            except _NoSpeech:
                # Nobody spoke. This is the normal idle state, not a fault:
                # re-arm the microphone with no cue, no print, no prompt. The
                # throttled event is purely so the log can distinguish "idle"
                # from "wedged" — silence alone looks identical to a hang.
                robot_log.event_throttled(
                    "voice.idle", key="idle", window_s=600.0,
                    waiting_s=config.LISTEN_TIMEOUT_S,
                )
                continue
            except Exception as e:
                robot_log.event("audio.error", logging.ERROR,
                                stage="listen", err=f"{type(e).__name__}: {e}")
                self._cues.unclear()
                continue

            # Local gate FIRST. Recognizer.listen() fires on energy, not
            # speech, so clicks, bumps and fan noise all reach here. Uploading
            # them is what exhausts the quota — and none of them could ever
            # have produced a command.
            gate = audio_gate.check(wav)
            if not gate.accepted:
                self._noise_dropped += 1
                robot_log.event_throttled(
                    "voice.noise", key="gate", window_s=60.0,
                    reason=gate.reason, dropped_total=self._noise_dropped,
                    **gate.as_dict(),
                )
                continue

            # One request: audio + prompt + schema -> plan. No separate
            # transcription hop, so nothing can be lost between two models.
            t0 = time.monotonic()
            try:
                result = self.plan(wav)
            except GeminiRateLimited as e:
                # Do NOT retry and do NOT loop straight back into listening —
                # that is what turns one 429 into a hundred. Sit out the
                # cooldown with the microphone closed.
                wait = max(1.0, min(e.retry_after or config.GEMINI_COOLDOWN_S,
                                    config.GEMINI_COOLDOWN_MAX_S))
                robot_log.event("voice.throttled", logging.WARNING,
                                err=str(e), sleeping_s=round(wait, 1),
                                budget=self._planner.budget.status(),
                                noise_dropped=self._noise_dropped)
                self._cues.error()
                time.sleep(wait)
                continue
            except GeminiError as e:
                robot_log.event("voice.unclear", logging.WARNING,
                                stage="plan", err=str(e),
                                audio_kb=round(len(wav) / 1024, 1))
                self._cues.unclear()
                continue
            except Exception as e:
                robot_log.event("audio.error", logging.ERROR,
                                stage="plan", err=f"{type(e).__name__}: {e}")
                self._cues.unclear()
                continue

            heard = result.get("heard", "")
            answer = result.get("answer", "none")
            steps = to_legacy_steps(result.get("steps", []))

            # `heard` is the model's own transcript. On a robot with no screen
            # this is the difference between "it ignored me" and "it heard
            # something else" — the single most useful line in logs.json.
            robot_log.event(
                "voice.heard", text=heard, answer=answer, steps=len(steps),
                rtt_s=round(time.monotonic() - t0, 2),
                audio_kb=round(len(wav) / 1024, 1),
            )

            if answer == "unclear":
                # The model heard audio but could not make a command of it. It
                # deliberately returns no steps rather than guessing: a wrong
                # move on a floor with obstacles beats no move.
                #
                # "no" and "didn't understand you" share the head-shake by
                # design — one bit of output, and the distinction is not worth
                # a second gesture the operator would have to learn.
                self._cues.unclear()
                self._gestures.play("unclear")
                continue

            if answer in ("yes", "no"):
                # The robot's entire reply. Blocking on this thread is correct:
                # the planner never returns an answer and steps together, so a
                # gesture can never overlap a movement, and there is nothing
                # else for the loop to do while it plays.
                self._gestures.play(answer)
                continue

            for step in steps:
                try:
                    if self._dispatch_step(step):
                        listening = False
                        break
                except Exception as e:
                    robot_log.event("move.failed", logging.ERROR,
                                    step=step, err=f"{type(e).__name__}: {e}")

            time.sleep(0.3)

        if self.vision:
            self.vision.stop()


def run_voice_session(
    move: Optional[ArduinoMovement] = None,
    vision: Optional[RobotVision] = None,
    start_guardian: bool = True,
) -> None:
    import config
    from coordinator import RobotCoordinator
    from movement_context import MovementContext

    ctx = MovementContext()
    move = move or ArduinoMovement(ctx=ctx)
    hist = MovementHistory(move)
    vision = vision or RobotVision()
    coord = None
    if start_guardian and config.VISION_HALT_OBJECTS:
        vision.start()
        coord = RobotCoordinator(move, vision, ctx)
        coord.start_guardian()
    try:
        session = VoiceRobotSession(move, hist, vision)
        session.run()
    finally:
        if coord:
            coord.stop_guardian()
        move.close()
        vision.stop()
