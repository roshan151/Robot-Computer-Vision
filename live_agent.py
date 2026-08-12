"""
Gemini Live agent: continuous audio in, tool calls out.

This replaces the turn-based loop entirely. There is no push-to-listen, no
per-utterance upload, no ambient calibration, no local speech gate and no
request budget — all of that existed to decide *when* to spend a request, and
a streaming session has no discrete requests to spend.

What the model gets is a live microphone and the robot's actual functions. It
decides when the operator has finished speaking (server-side VAD) and calls
`drive`, `turn`, `stop` or `answer` directly.

Two tasks run concurrently:

    _pump_audio   microphone -> session, forever
    _pump_events  session -> tool dispatch, forever

They are cancelled together. If either dies the session is torn down, the
error tone plays, and the supervisor reconnects — in that order, because the
tone must not play into a live microphone.

Audio format
------------
16 kHz signed 16-bit mono PCM upstream, which is what the Live API expects and
what the Pi's capture path already produces.
"""

from __future__ import annotations

import asyncio
import logging
import time
from typing import Any, Dict, Optional

import config
import robot_log
from audio_cues import cues
from robot_tools import RobotTools, declarations

logger = logging.getLogger(__name__)

SEND_SAMPLE_RATE = 16000     # uplink: what the Live API expects
RECV_SAMPLE_RATE = 24000     # downlink: what it returns (only used if played)
CHUNK_FRAMES = 1024          # ~64 ms at 16 kHz


class LiveAgentError(RuntimeError):
    """The Live session could not be established or has failed."""


class LiveConfigError(LiveAgentError):
    """The session was rejected for how it was configured.

    Kept separate from ordinary failures because retrying is pointless and
    actively harmful: the same config will be refused every time, and each
    attempt brakes the motors and writes another identical error. One bad
    setting should produce one clear line, not a log full of them.
    """


def _is_config_rejection(exc: BaseException) -> bool:
    text = f"{type(exc).__name__} {exc}".lower()
    return (
        "1007" in text                       # websocket policy violation
        or "not supported by the model" in text
        or "invalid_argument" in text
        or "response modalities" in text
    )


class LiveAgent:
    """One connected session. Construct, `run()`, and it returns when done."""

    def __init__(self, tools: RobotTools, model: Optional[str] = None) -> None:
        self.model = model or config.GEMINI_LIVE_MODEL
        self._tools = tools
        self._cues = cues()
        self._audio_q: "asyncio.Queue[bytes]" = asyncio.Queue(maxsize=64)
        self._stream = None
        self._dropped = 0
        self._audio_out_bytes = 0
        self._out = None

    # ------------------------------------------------------------------ #

    def _config(self) -> Dict[str, Any]:
        from google.genai import types  # type: ignore

        # AUDIO, because the native-audio Live models are speech-to-speech and
        # reject TEXT outright ("1007 ... response modalities (TEXT) is not
        # supported by the model").
        #
        # The robot is still silent. response_modalities controls what the
        # model GENERATES, not what we render — and nothing here plays the
        # returned PCM. It is read off the socket and dropped, so no speaker
        # ever emits it and the microphone never hears it.
        #
        # The transcriptions are what make that free: rather than losing the
        # model's words, we get them as text for logs.json, plus a transcript
        # of what it heard the operator say. On a robot with no screen, that
        # pair is the difference between "it ignored me" and "it misheard me".
        cfg: Dict[str, Any] = {
            "response_modalities": [config.LIVE_RESPONSE_MODALITY],
            "system_instruction": config.LIVE_SYSTEM_PROMPT,
            "tools": [{"function_declarations": declarations()}],
        }
        try:
            cfg["input_audio_transcription"] = types.AudioTranscriptionConfig()
            cfg["output_audio_transcription"] = types.AudioTranscriptionConfig()
        except AttributeError:
            # Older SDK without the config type — the agent still works, the
            # log is just quieter about what was said.
            logger.debug("SDK has no AudioTranscriptionConfig; skipping")
        return cfg

    # ------------------------------------------------------------------ #

    def _open_microphone(self) -> None:
        import sounddevice as sd  # type: ignore

        loop = asyncio.get_running_loop()

        def callback(indata, frames, time_info, status):  # runs on PortAudio's thread
            if status:
                logger.debug("audio input status: %s", status)
            # call_soon_threadsafe because this fires on PortAudio's own
            # thread, not the event loop.
            try:
                loop.call_soon_threadsafe(self._offer, bytes(indata))
            except RuntimeError:
                pass                     # loop closing

        self._stream = sd.RawInputStream(
            samplerate=SEND_SAMPLE_RATE,
            blocksize=CHUNK_FRAMES,
            device=config.AUDIO_INPUT_DEVICE or None,
            dtype="int16",
            channels=1,
            callback=callback,
        )
        self._stream.start()

    def _offer(self, chunk: bytes) -> None:
        """Enqueue a chunk, dropping the oldest if we have fallen behind.

        Dropping beats blocking: if the uplink stalls, the newest audio is what
        matters and an unbounded queue would just grow until the robot was
        acting on minute-old speech.
        """
        try:
            self._audio_q.put_nowait(chunk)
        except asyncio.QueueFull:
            self._dropped += 1
            try:
                self._audio_q.get_nowait()
                self._audio_q.put_nowait(chunk)
            except (asyncio.QueueEmpty, asyncio.QueueFull):
                pass
            robot_log.event_throttled(
                "audio.error", key="uplink-lag", window_s=30.0,
                level=logging.WARNING, stage="mic-queue",
                err="uplink behind, dropping audio", dropped=self._dropped,
            )

    def _close_microphone(self) -> None:
        if self._out is not None:
            try:
                self._out.stop()
                self._out.close()
            except Exception:
                pass
            self._out = None
        if self._stream is not None:
            try:
                self._stream.stop()
                self._stream.close()
            except Exception:
                pass
            self._stream = None

    # ------------------------------------------------------------------ #

    async def _pump_audio(self, session) -> None:
        from google.genai import types  # type: ignore

        while True:
            chunk = await self._audio_q.get()
            blob = types.Blob(data=chunk,
                              mime_type=f"audio/pcm;rate={SEND_SAMPLE_RATE}")
            # send_realtime_input is the current name; older SDKs used
            # send(input=..., end_of_turn=False). Try the modern one first.
            if hasattr(session, "send_realtime_input"):
                await session.send_realtime_input(audio=blob)
            else:
                await session.send(input=blob)

    async def _pump_events(self, session) -> None:
        async for response in session.receive():
            calls = self._extract_tool_calls(response)
            if calls:
                await self._handle_tool_calls(session, calls)
                continue

            # Audio comes back because the model is speech-to-speech, but the
            # robot is silent by design: read it off the socket and drop it.
            # Nothing plays it, so nothing re-enters the microphone.
            self._drain_audio(response)
            self._log_transcripts(response)

    def _drain_audio(self, response) -> None:
        """Discard generated speech. Counted, so 'silent' stays a deliberate
        choice rather than something we stopped noticing."""
        data = getattr(response, "data", None)
        if not data:
            return
        self._audio_out_bytes += len(data)

        if config.LIVE_PLAY_AUDIO:
            # Off by default, and it should stay off: the microphone is open
            # for the whole session, so anything played here is streamed
            # straight back to the model as if the operator had said it.
            self._play(data)
            return

        robot_log.event_throttled(
            "voice.say", key="discarded", window_s=300.0,
            text="(model speech discarded — robot answers by moving)",
            bytes_dropped=self._audio_out_bytes,
        )

    def _play(self, pcm: bytes) -> None:
        try:
            import numpy as np  # type: ignore
            import sounddevice as sd  # type: ignore

            if self._out is None:
                self._out = sd.OutputStream(
                    samplerate=RECV_SAMPLE_RATE, channels=1, dtype="int16")
                self._out.start()
            self._out.write(np.frombuffer(pcm, dtype="<i2"))
        except Exception as e:
            robot_log.event_throttled(
                "audio.error", key="playback", window_s=60.0,
                level=logging.WARNING, stage="live-playback",
                err=f"{type(e).__name__}: {e}")

    def _log_transcripts(self, response) -> None:
        """What the model heard, and what it would have said.

        The only window into the conversation on a robot with no screen.
        """
        server = getattr(response, "server_content", None)
        if server is None:
            return
        heard = getattr(server, "input_transcription", None)
        said = getattr(server, "output_transcription", None)

        text = getattr(heard, "text", None)
        if text and text.strip():
            robot_log.event("voice.heard", text=text.strip()[:400])

        text = getattr(said, "text", None)
        if text and text.strip():
            robot_log.event("voice.say", text=text.strip()[:400], spoken=False)

    @staticmethod
    def _extract_tool_calls(response) -> list:
        """Tool calls have lived in two places across SDK versions.

        Checking both costs nothing and avoids a silent no-op where the robot
        hears commands, decides what to do, and never does it.
        """
        tc = getattr(response, "tool_call", None)
        if tc is None:
            server = getattr(response, "server_content", None)
            tc = getattr(server, "tool_call", None) if server else None
        return list(getattr(tc, "function_calls", []) or []) if tc else []

    async def _handle_tool_calls(self, session, calls) -> None:
        from google.genai import types  # type: ignore

        responses = []
        for call in calls:
            name = getattr(call, "name", "")
            args = dict(getattr(call, "args", {}) or {})
            t0 = time.monotonic()
            result = await self._tools.dispatch(name, args)
            logger.info("tool %s(%s) -> %s in %.2fs",
                        name, args, result, time.monotonic() - t0)
            responses.append(types.FunctionResponse(
                id=getattr(call, "id", None), name=name,
                response={"output": result},
            ))

        if hasattr(session, "send_tool_response"):
            await session.send_tool_response(function_responses=responses)
        else:
            await session.send(input={"tool_response": {
                "function_responses": [
                    {"id": r.id, "name": r.name, "response": r.response}
                    for r in responses
                ]
            }})

    # ------------------------------------------------------------------ #

    async def run(self) -> None:
        """Connect and serve until the session ends or a task fails."""
        try:
            from google import genai  # type: ignore
        except ImportError as e:
            raise LiveAgentError(
                "google-genai is not installed. pip install -U google-genai"
            ) from e

        client = genai.Client(api_key=config.require("GEMINI_API_KEY"))

        try:
            connection = client.aio.live.connect(
                model=self.model, config=self._config())
        except Exception as e:
            if _is_config_rejection(e):
                raise LiveConfigError(str(e)) from e
            raise

        async with connection as session:
            robot_log.event("voice.connect", backend="gemini-live",
                            model=self.model,
                            modality=config.LIVE_RESPONSE_MODALITY)

            # Tone BEFORE the microphone opens, so it cannot be streamed to the
            # model as if the operator had made the sound.
            self._cues.started()

            self._open_microphone()
            try:
                # Deliberately not asyncio.TaskGroup: the Pi runs Python 3.10
                # and TaskGroup / except* are 3.11+. gather with
                # FIRST_EXCEPTION gives the same shape — whichever pump dies
                # first takes the session down, and the other is cancelled.
                tasks = [
                    asyncio.create_task(self._pump_audio(session),
                                        name="mic-uplink"),
                    asyncio.create_task(self._pump_events(session),
                                        name="events"),
                ]
                done, pending = await asyncio.wait(
                    tasks, return_when=asyncio.FIRST_EXCEPTION)
                for task in pending:
                    task.cancel()
                if pending:
                    await asyncio.gather(*pending, return_exceptions=True)
                for task in done:
                    task.result()          # re-raise the real failure
            finally:
                self._close_microphone()


async def _run_supervised(tools: RobotTools) -> None:
    """Reconnect on failure, with backoff. Motors stop first, every time."""
    backoff = config.LIVE_RECONNECT_BACKOFF_S
    while True:
        agent = LiveAgent(tools)
        try:
            await agent.run()
            robot_log.event("session.stop", reason="live session closed")
            return
        except asyncio.CancelledError:
            raise
        except Exception as e:
            _on_failure(tools, agent, e)
            if isinstance(e, LiveConfigError) or _is_config_rejection(e):
                # Retrying a rejected configuration just produces the same
                # error forever, braking the motors on every attempt. Say what
                # is wrong once, in terms that name the fix, and stop.
                robot_log.event(
                    "fatal", logging.CRITICAL,
                    cause="live session configuration rejected",
                    model=agent.model,
                    modality=config.LIVE_RESPONSE_MODALITY,
                    err=str(e)[:300],
                    fix=("native-audio Live models are speech-to-speech and "
                         "only accept AUDIO. Set ROBOT_LIVE_MODALITY=AUDIO — "
                         "the robot stays silent because the returned audio is "
                         "discarded, not played."),
                )
                raise

        await asyncio.sleep(backoff)
        backoff = min(backoff * 2, config.LIVE_RECONNECT_MAX_S)


def _on_failure(tools: RobotTools, agent: LiveAgent, exc: BaseException) -> None:
    """Order matters: brake, tear down, THEN make a noise."""
    try:
        tools._move.emergency_stop()
    except Exception:
        pass
    robot_log.event("voice.drop", logging.ERROR,
                    err=f"{type(exc).__name__}: {exc}")
    # Safe to play now: the session is gone, so nothing is listening.
    agent._cues.error()


def run_live_agent(move, history=None) -> None:
    """Blocking entrypoint used by run_robot.py."""
    tools = RobotTools(move, history)
    try:
        asyncio.run(_run_supervised(tools))
    except KeyboardInterrupt:
        robot_log.event("session.stop", reason="keyboard interrupt")
    finally:
        try:
            move.emergency_stop()
        except Exception:
            pass
