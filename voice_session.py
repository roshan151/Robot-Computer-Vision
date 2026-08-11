"""
Voice → GPT → movement + vision, with Arduino drivetrain backend.
Optional: Picamera2 / OpenCV, Nix TTS, Google STT.
"""

from __future__ import annotations

import json
import logging
import os
import re
import sys
import time
from typing import Any, Dict, List, Optional

import config
import robot_log
from audio_cues import cues

from prompts_and_glossary import commands as glossary_commands, movement_prompt

from movement_adapter import ArduinoMovement, MovementHistory
from vision_client import RobotVision

logger = logging.getLogger(__name__)

VISION_STEP_KEYS = {"capture", "record", "detect", "scan", "terminate"}
ORIGIN_KEYS = set(glossary_commands.get("origin", [])) | {"origin"}
TERMINATE_WORDS = {t.lower() for t in glossary_commands.get("terminate", [])} | {"terminate"}
JSON_SUFFIX = """

Respond with ONLY valid JSON (no markdown fences). Schema:
{"steps":[{"forward":2},{"left":90}]}

Rules:
- Each element of "steps" is one object with exactly one key.
- Movement keys: forward, reverse, left, right, stop, origin.
  - forward/reverse: numeric meters.
  - left/right: numeric degrees.
  - stop and origin: use null as value (JSON null).
- Vision keys: capture, record, detect, scan, terminate — use null except detect/scan take an array of object names, e.g. ["plant","plants"].
- For scan, value is the same array style as detect.
"""


class _NoSpeech(Exception):
    """Nobody spoke inside the listen window.

    Deliberately not a RuntimeError: silence is the idle state, not a fault,
    and conflating the two is what produced the old 'Speak your command' /
    'No speech heard' loop every ten seconds.
    """


def _strip_code_fence(text: str) -> str:
    text = text.strip()
    m = re.match(r"^```(?:json)?\s*([\s\S]*?)```$", text, re.IGNORECASE)
    if m:
        return m.group(1).strip()
    return text


def parse_gpt_steps(content: str) -> List[Dict[str, Any]]:
    raw = _strip_code_fence(content)
    data = json.loads(raw)
    steps = data.get("steps")
    if not isinstance(steps, list):
        raise ValueError("JSON must contain a 'steps' array")
    out: List[Dict[str, Any]] = []
    for item in steps:
        if not isinstance(item, dict) or not item:
            raise ValueError("each step must be a non-empty object")
        if len(item) > 1:
            logger.warning("multi-key step from model, using first key only: %s", item)
        k = next(iter(item))
        out.append({k: item[k]})
    return out


class VoiceRobotSession:
    def __init__(
        self,
        move: ArduinoMovement,
        history: MovementHistory,
        vision: Optional[RobotVision],
        openai_model: Optional[str] = None,
    ) -> None:
        self.move = move
        self.history = history
        self.vision = vision
        # config loads .env once at import and is the single source of truth
        # for both the model name and the credential.
        self.openai_model = openai_model or config.OPENAI_MODEL
        self.openai_api_key = config.OPENAI_API_KEY
        self._nix = None
        self._sd = None
        self._sr = None
        self._recognizer = None
        self._mic = None
        self._calibrated = False
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

    def query_gpt(self, messages: list) -> str:
        # require() names the variable and the three places it can live, which
        # is a better first line in logs.json than a vendor 401 three frames in.
        api_key = self.openai_api_key or config.require("OPENAI_API_KEY")
        import openai

        client = openai.OpenAI(api_key=api_key)
        response = client.chat.completions.create(
            model=self.openai_model,
            messages=messages,
            temperature=0.3,
            max_tokens=800,
            response_format={"type": "json_object"},
        )
        return response.choices[0].message.content.strip()

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
            self._calibrated = True
            robot_log.event("voice.calibrate",
                            threshold=round(self._recognizer.energy_threshold, 1))
        except Exception as e:
            robot_log.event("audio.error", logging.WARNING,
                            stage="calibrate", err=f"{type(e).__name__}: {e}")

    def listen_once(self) -> str:
        """Wait for one spoken command. Blocks silently until speech arrives.

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

        try:
            text = str(self._recognizer.recognize_google(audio)).lower()
        except self._sr.UnknownValueError as e:
            raise RuntimeError("could not understand audio") from e
        robot_log.event("voice.heard", text=text)
        return text

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
        system = movement_prompt + JSON_SUFFIX
        messages: List[Dict[str, str]] = [{"role": "system", "content": system}]
        listening = True
        while listening:
            try:
                speech = self.listen_once()
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
            except RuntimeError as e:
                # Heard something, could not transcribe it. Same answer the
                # robot gives for "no": one gesture, no speech.
                robot_log.event("voice.unclear", logging.WARNING, err=str(e))
                self._cues.unclear()
                continue
            except Exception as e:
                robot_log.event("audio.error", logging.ERROR,
                                stage="listen", err=f"{type(e).__name__}: {e}")
                self._cues.unclear()
                continue

            messages.append({"role": "user", "content": speech})
            try:
                raw = self.query_gpt(messages)
                steps = parse_gpt_steps(raw)
            except Exception as e:
                logger.exception("GPT parse failed: %s", e)
                self.speak("I could not plan that command.")
                messages.append({"role": "assistant", "content": json.dumps({"error": str(e)})})
                continue

            messages.append({"role": "assistant", "content": raw})

            for step in steps:
                try:
                    if self._dispatch_step(step):
                        listening = False
                        break
                except Exception as e:
                    logger.exception("step failed: %s", e)
                    self.speak(f"Failed on step {step}: {e}")

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
