"""
One-call voice planner: microphone audio in, movement steps out.

What this replaces
------------------
    mic -> Google Web Speech -> text -> OpenAI -> JSON     (two round trips)
    mic -> Gemini Flash (audio + schema) -> JSON           (one)

Gemini is natively multimodal, so the audio and the planning prompt go in the
same request and structured JSON comes back directly. Beyond dropping a network
hop and a vendor, this fixes a real quality problem: the old prompt had to
instruct the model to *guess* at speech-recognition errors ("'turn write' is
probably 'turn right'") because all it ever saw was a transcript. Gemini hears
the audio, so homophones are resolved from sound rather than from apology.

Schema shape
------------
The wire format is typed rather than the old `{"forward": 2}` single-key
objects. Structured output cannot constrain arbitrary key *names*, but it can
constrain an enum:

    {"action": "forward", "value": 2.0}

`to_legacy_steps()` converts back to the single-key form the existing dispatcher
expects, so nothing downstream changes.

The response also carries `heard` — the model's own transcript. On a robot with
no screen that is the difference between "it ignored me" and "it heard 'move
forward too meters'".
"""

from __future__ import annotations

import json
import logging
import re
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

import config
import robot_log

logger = logging.getLogger(__name__)

# Actions the planner may emit. Kept in step with prompts_and_glossary and
# voice_session's dispatcher — an enum here is what stops the model inventing
# verbs the robot cannot perform.
MOVEMENT_ACTIONS = ("forward", "reverse", "left", "right", "stop", "origin")
VISION_ACTIONS = ("capture", "record", "detect", "scan", "terminate")
ACTIONS = MOVEMENT_ACTIONS + VISION_ACTIONS

# Actions whose value is a list of object names rather than a number.
OBJECT_ACTIONS = ("detect", "scan")
# Actions that take no argument at all.
NULLARY_ACTIONS = ("stop", "capture", "record", "terminate")

RESPONSE_SCHEMA: Dict[str, Any] = {
    "type": "object",
    "properties": {
        "heard": {
            "type": "string",
            "description": "Verbatim transcript of the spoken command.",
        },
        "answer": {
            "type": "string",
            "enum": ["yes", "no", "unclear", "none"],
            "description": (
                "The robot's whole reply, performed as a gesture. "
                "'yes' nods; 'no' shakes; 'unclear' shakes identically to 'no' "
                "and means the speech could not be understood — never use it as "
                "a soft refusal, and never when the command might have been "
                "'stop'. 'none' means a command is being executed instead. "
                "Set anything other than 'none' only with an empty steps list."
            ),
        },
        "steps": {
            "type": "array",
            "description": "Ordered actions to execute. Empty if none apply.",
            "items": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": list(ACTIONS)},
                    "value": {
                        "type": "number",
                        "description": "Metres for forward/reverse, degrees for "
                                       "left/right, step count for origin.",
                    },
                    "objects": {
                        "type": "array",
                        "items": {"type": "string"},
                        "description": "Object names for detect/scan.",
                    },
                },
                "required": ["action"],
            },
        },
    },
    "required": ["heard", "steps"],
}


class GeminiError(RuntimeError):
    """The planner could not produce a usable result."""


class GeminiRateLimited(GeminiError):
    """Quota exhausted (HTTP 429), or the local budget refused the request.

    Separate from GeminiError because the right response is different: a
    malformed reply means shake the head and listen again, but hammering a
    rate-limited endpoint just deepens the hole.
    """

    def __init__(self, message: str, retry_after: float = 0.0) -> None:
        super().__init__(message)
        self.retry_after = retry_after


def _looks_rate_limited(exc: Exception) -> bool:
    text = f"{type(exc).__name__} {exc}".lower()
    return ("429" in text or "resource_exhausted" in text
            or "rate limit" in text or "quota" in text)


def _retry_after_from(exc: Exception) -> float:
    """Pull a server-suggested delay out of the error if it offers one."""
    m = re.search(r"retry[- _]?after[\"'\s:]+(\d+(?:\.\d+)?)", str(exc), re.I)
    if m:
        return float(m.group(1))
    m = re.search(r"retryDelay[\"'\s:]+(\d+(?:\.\d+)?)s", str(exc))
    return float(m.group(1)) if m else 0.0


class RequestBudget:
    """Token bucket + minimum spacing + post-429 cooldown.

    The speech gate is what should keep request volume sane. This is the
    backstop that makes it impossible to exceed the quota even if the gate is
    misconfigured or the room does something unexpected — a robot that quietly
    burns its daily quota overnight is a worse failure than one that declines
    a command.
    """

    def __init__(self, max_rpm: Optional[int] = None,
                 min_interval_s: Optional[float] = None) -> None:
        self.max_rpm = config.GEMINI_MAX_RPM if max_rpm is None else max_rpm
        self.min_interval_s = (config.GEMINI_MIN_INTERVAL_S
                               if min_interval_s is None else min_interval_s)
        self._lock = threading.Lock()
        self._times: List[float] = []
        self._cooldown_until = 0.0
        self._cooldown_s = config.GEMINI_COOLDOWN_S
        self.rejected = 0

    def check(self) -> None:
        """Raise GeminiRateLimited if this request must not be sent."""
        now = time.monotonic()
        with self._lock:
            if now < self._cooldown_until:
                self.rejected += 1
                raise GeminiRateLimited(
                    f"in cooldown for another {self._cooldown_until - now:.0f}s "
                    f"after a 429",
                    retry_after=self._cooldown_until - now,
                )

            self._times = [t for t in self._times if now - t < 60.0]
            if len(self._times) >= self.max_rpm:
                wait = 60.0 - (now - self._times[0])
                self.rejected += 1
                raise GeminiRateLimited(
                    f"local budget: {self.max_rpm} requests/min reached",
                    retry_after=wait,
                )
            if self._times and now - self._times[-1] < self.min_interval_s:
                wait = self.min_interval_s - (now - self._times[-1])
                self.rejected += 1
                raise GeminiRateLimited(
                    f"local budget: minimum {self.min_interval_s}s between "
                    f"requests", retry_after=wait,
                )
            self._times.append(now)

    def penalise(self, retry_after: float = 0.0) -> float:
        """Record a server 429 and start (or extend) the cooldown."""
        with self._lock:
            wait = max(retry_after, self._cooldown_s)
            self._cooldown_until = time.monotonic() + wait
            self._cooldown_s = min(self._cooldown_s * 2,
                                   config.GEMINI_COOLDOWN_MAX_S)
            return wait

    def succeeded(self) -> None:
        """A good response resets the escalating cooldown."""
        with self._lock:
            self._cooldown_s = config.GEMINI_COOLDOWN_S

    def status(self) -> Dict[str, Any]:
        now = time.monotonic()
        with self._lock:
            recent = len([t for t in self._times if now - t < 60.0])
            return {
                "rpm_used": recent,
                "rpm_max": self.max_rpm,
                "rejected": self.rejected,
                "cooldown_s": max(0.0, round(self._cooldown_until - now, 1)),
            }


def to_legacy_steps(steps: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """Typed steps -> the single-key dicts `_dispatch_step` already understands.

    Keeping the old shape at the boundary means the dispatcher, glossary and
    MovementHistory are all untouched by this change.
    """
    out: List[Dict[str, Any]] = []
    for step in steps:
        if not isinstance(step, dict):
            continue
        action = str(step.get("action", "")).lower()
        if action not in ACTIONS:
            logger.warning("dropping unknown action %r", action)
            continue
        if action in OBJECT_ACTIONS:
            out.append({action: step.get("objects") or None})
        elif action in NULLARY_ACTIONS:
            out.append({action: None})
        else:
            out.append({action: step.get("value")})
    return out


class GeminiVoicePlanner:
    """Audio -> movement plan in a single request."""

    def __init__(
        self,
        api_key: Optional[str] = None,
        model: Optional[str] = None,
        system_prompt: str = "",
        history_turns: Optional[int] = None,
    ) -> None:
        self.model = model or config.GEMINI_MODEL
        self.system_prompt = system_prompt
        self.history_turns = (
            config.GEMINI_HISTORY_TURNS if history_turns is None else history_turns
        )
        self._api_key = api_key
        self._client = None
        self._surface: Optional[str] = None
        self.budget = RequestBudget()
        # Rolling text context. Prior audio is deliberately NOT resent: the
        # transcript carries everything the planner needs and costs a fraction
        # of the tokens.
        self._history: List[Tuple[str, str]] = []

    # ------------------------------------------------------------------ #

    def _connect(self):
        if self._client is not None:
            return self._client
        key = self._api_key or config.require("GEMINI_API_KEY")
        try:
            from google import genai  # type: ignore
        except ImportError as e:
            raise GeminiError(
                "google-genai is not installed. pip install google-genai"
            ) from e

        self._client = genai.Client(api_key=key)
        # The Interactions API is the current surface; generate_content is the
        # legacy one. Installed SDK versions vary widely on a Pi, so pick
        # whichever this one actually has rather than pinning a version.
        self._surface = "interactions" if hasattr(self._client, "interactions") \
            else "generate_content"
        robot_log.event("voice.connect", backend="gemini", model=self.model,
                        surface=self._surface)
        return self._client

    def _prompt_text(self) -> str:
        parts = [self.system_prompt]
        if self._history:
            recent = self._history[-self.history_turns:]
            parts.append(
                "\nEarlier in this session (already executed, for context only):\n"
                + "\n".join(f"- said: {h}\n  did: {p}" for h, p in recent)
            )
        parts.append(
            "\nThe attached audio is the operator's latest spoken command. "
            "Transcribe it into `heard`, then plan only that command."
        )
        return "\n".join(parts)

    def plan(self, wav_bytes: bytes, mime_type: str = "audio/wav") -> Dict[str, Any]:
        """Send audio, get `{heard, answer, steps}` back. One request.

        `wav_bytes` should be 16 kHz mono PCM WAV — Gemini downsamples to
        16 kbps and mixes to mono anyway, so sending more is wasted upload on
        a Pi's WiFi.
        """
        size_kb = len(wav_bytes) / 1024
        if size_kb > config.GEMINI_MAX_AUDIO_KB:
            raise GeminiError(
                f"audio is {size_kb:.0f} kB, over the "
                f"{config.GEMINI_MAX_AUDIO_KB} kB inline limit"
            )

        # Checked before connecting, so a refused request costs nothing.
        self.budget.check()

        client = self._connect()
        prompt = self._prompt_text()

        try:
            if self._surface == "interactions":
                raw = self._call_interactions(client, prompt, wav_bytes, mime_type)
            else:
                raw = self._call_generate_content(client, prompt, wav_bytes, mime_type)
        except GeminiError:
            raise
        except Exception as e:
            if _looks_rate_limited(e):
                wait = self.budget.penalise(_retry_after_from(e))
                raise GeminiRateLimited(
                    f"server refused: {type(e).__name__}: {e}", retry_after=wait
                ) from e
            raise GeminiError(f"{type(e).__name__}: {e}") from e

        self.budget.succeeded()
        result = self._parse(raw)
        heard = result.get("heard", "")
        if heard:
            self._history.append((heard, json.dumps(result.get("steps", []))))
            del self._history[: -self.history_turns or None]
        return result

    # ------------------------------------------------------------------ #

    def _call_interactions(self, client, prompt: str, audio: bytes, mime: str) -> str:
        import base64

        interaction = client.interactions.create(
            model=self.model,
            input=[
                {"type": "text", "text": prompt},
                {
                    "type": "audio",
                    "data": base64.b64encode(audio).decode("utf-8"),
                    "mime_type": mime,
                },
            ],
            response_format=RESPONSE_SCHEMA,
        )
        return interaction.output_text

    def _call_generate_content(self, client, prompt: str, audio: bytes, mime: str) -> str:
        from google.genai import types  # type: ignore

        response = client.models.generate_content(
            model=self.model,
            contents=[
                types.Part.from_text(text=prompt),
                types.Part.from_bytes(data=audio, mime_type=mime),
            ],
            config=types.GenerateContentConfig(
                response_mime_type="application/json",
                response_schema=RESPONSE_SCHEMA,
                temperature=config.GEMINI_TEMPERATURE,
            ),
        )
        return response.text

    @staticmethod
    def _parse(raw: str) -> Dict[str, Any]:
        if not raw or not raw.strip():
            raise GeminiError("empty response")
        text = raw.strip()
        # Structured output should never fence, but a stray ```json costs one
        # command if unhandled and nothing to strip defensively.
        if text.startswith("```"):
            text = text.split("\n", 1)[-1].rsplit("```", 1)[0].strip()
        try:
            data = json.loads(text)
        except json.JSONDecodeError as e:
            raise GeminiError(f"response was not JSON: {e}") from e
        if not isinstance(data, dict):
            raise GeminiError("response was not a JSON object")
        data.setdefault("heard", "")
        data.setdefault("answer", "none")
        steps = data.get("steps")
        data["steps"] = steps if isinstance(steps, list) else []
        return data

    def reset(self) -> None:
        self._history.clear()
