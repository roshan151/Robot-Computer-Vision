"""
Gemini one-call planner: schema, step conversion, and failure behaviour.

No network. The SDK is stubbed, so these run anywhere and assert the parts
that are ours: what we send, what we do with what comes back, and — most
importantly — what happens when the model returns something unusable.

Run:  python tests/test_gemini_planner.py
"""

from __future__ import annotations

import json
import sys
import types
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

import config  # noqa: E402
import gemini_client as gc  # noqa: E402
from gemini_client import (  # noqa: E402
    ACTIONS,
    RESPONSE_SCHEMA,
    GeminiError,
    GeminiVoicePlanner,
    RequestBudget,
    to_legacy_steps,
)

WAV = b"RIFF" + b"\0" * 2000


class FakeInteractions:
    def __init__(self, payload, record) -> None:
        self._payload = payload
        self._record = record

    def create(self, **kwargs):
        self._record.append(kwargs)
        if isinstance(self._payload, Exception):
            raise self._payload
        text = self._payload if isinstance(self._payload, str) \
            else json.dumps(self._payload)
        return types.SimpleNamespace(output_text=text)


def _planner(payload, prompt="SYSTEM PROMPT"):
    record = []
    p = GeminiVoicePlanner(api_key="fake-key", system_prompt=prompt)
    p._client = types.SimpleNamespace(interactions=FakeInteractions(payload, record))
    p._surface = "interactions"
    # These tests are about request SHAPE, not pacing — the budget has its own
    # suite in test_noise_and_budget.py.
    p.budget = RequestBudget(max_rpm=10_000, min_interval_s=0)
    return p, record


# --------------------------------------------------------------------------- #
# Schema
# --------------------------------------------------------------------------- #

def test_schema_constrains_actions_to_an_enum() -> None:
    """Structured output cannot constrain arbitrary key names, but it can
    constrain an enum — which is what stops the model inventing verbs."""
    item = RESPONSE_SCHEMA["properties"]["steps"]["items"]
    assert item["properties"]["action"]["enum"] == list(ACTIONS)
    assert "forward" in ACTIONS and "stop" in ACTIONS
    assert item["required"] == ["action"], "value must stay optional for stop"


def test_schema_requires_a_transcript() -> None:
    """`heard` is the only window into what the robot understood."""
    assert "heard" in RESPONSE_SCHEMA["required"]
    assert "steps" in RESPONSE_SCHEMA["required"]


def test_schema_actions_match_the_dispatcher() -> None:
    """An action the dispatcher cannot route is a command silently dropped."""
    import voice_session as vs
    from prompts_and_glossary import commands as glossary

    routable = (set(glossary["movement"]) | vs.ORIGIN_KEYS
                | vs.VISION_STEP_KEYS | vs.TERMINATE_WORDS)
    unroutable = [a for a in ACTIONS if a not in routable]
    assert not unroutable, f"planner can emit unroutable actions: {unroutable}"


# --------------------------------------------------------------------------- #
# Step conversion
# --------------------------------------------------------------------------- #

def test_typed_steps_become_legacy_single_key_dicts() -> None:
    got = to_legacy_steps([
        {"action": "forward", "value": 2.0},
        {"action": "left", "value": 90},
    ])
    assert got == [{"forward": 2.0}, {"left": 90}]


def test_nullary_actions_carry_none() -> None:
    for action in ("stop", "capture", "record", "terminate"):
        assert to_legacy_steps([{"action": action, "value": 5}]) == [{action: None}], \
            f"{action} should ignore any value"


def test_object_actions_carry_the_list() -> None:
    got = to_legacy_steps([{"action": "detect", "objects": ["plant", "plants"]}])
    assert got == [{"detect": ["plant", "plants"]}]
    assert to_legacy_steps([{"action": "scan"}]) == [{"scan": None}]


def test_origin_keeps_its_value() -> None:
    assert to_legacy_steps([{"action": "origin", "value": -1}]) == [{"origin": -1}]


def test_unknown_actions_are_dropped_not_raised() -> None:
    """One bad step must not discard the whole plan."""
    got = to_legacy_steps([
        {"action": "forward", "value": 1},
        {"action": "teleport", "value": 99},
        {"action": "left", "value": 45},
    ])
    assert got == [{"forward": 1}, {"left": 45}]


def test_malformed_steps_are_survivable() -> None:
    assert to_legacy_steps(["not a dict", None, {}, {"action": "stop"}]) == [{"stop": None}]


# --------------------------------------------------------------------------- #
# The request
# --------------------------------------------------------------------------- #

def test_audio_and_prompt_go_in_one_request() -> None:
    """The whole point: no separate transcription call."""
    p, record = _planner({"heard": "go forward", "answer": "none",
                          "steps": [{"action": "forward", "value": 1}]})
    p.plan(WAV)

    assert len(record) == 1, f"expected exactly one API call, got {len(record)}"
    kinds = [part["type"] for part in record[0]["input"]]
    assert "audio" in kinds and "text" in kinds, kinds
    assert record[0]["response_format"] is RESPONSE_SCHEMA


def test_prompt_carries_history_but_never_audio() -> None:
    """Resending prior audio would cost 32 tokens/sec for nothing — the
    transcript already carries what the planner needs."""
    p, record = _planner({"heard": "turn left", "answer": "none",
                          "steps": [{"action": "left", "value": 90}]})
    p.plan(WAV)
    p.plan(WAV)

    second = record[1]
    assert sum(1 for part in second["input"] if part["type"] == "audio") == 1, \
        "more than one audio part — history audio is being resent"
    text = next(part["text"] for part in second["input"] if part["type"] == "text")
    assert "turn left" in text, "prior transcript missing from context"


def test_oversized_audio_is_refused_before_upload() -> None:
    p, record = _planner({"heard": "", "steps": []})
    huge = b"\0" * (config.GEMINI_MAX_AUDIO_KB * 1024 + 1)
    try:
        p.plan(huge)
    except GeminiError as e:
        assert "inline limit" in str(e)
    else:
        raise AssertionError("oversized audio was uploaded anyway")
    assert record == [], "a doomed request was still sent"


# --------------------------------------------------------------------------- #
# Failure behaviour
# --------------------------------------------------------------------------- #

def test_non_json_response_raises_geminierror() -> None:
    p, _ = _planner("I'm afraid I can't do that")
    try:
        p.plan(WAV)
    except GeminiError as e:
        assert "not JSON" in str(e)
    else:
        raise AssertionError("garbage was accepted as a plan")


def test_empty_response_raises_geminierror() -> None:
    p, _ = _planner("   ")
    try:
        p.plan(WAV)
    except GeminiError:
        pass
    else:
        raise AssertionError("empty response accepted")


def test_fenced_json_still_parses() -> None:
    """Structured output should never fence, but an unhandled stray ```json
    would cost a command for no reason."""
    p, _ = _planner('```json\n{"heard":"stop","answer":"none",'
                    '"steps":[{"action":"stop"}]}\n```')
    result = p.plan(WAV)
    assert to_legacy_steps(result["steps"]) == [{"stop": None}]


def test_transport_errors_become_geminierror() -> None:
    """The caller catches GeminiError to play the 'unclear' cue; a raw
    exception type escaping would skip that and look like a crash."""
    p, _ = _planner(ConnectionError("network unreachable"))
    try:
        p.plan(WAV)
    except GeminiError as e:
        assert "ConnectionError" in str(e)
    else:
        raise AssertionError("transport error was not wrapped")


def test_missing_fields_are_defaulted() -> None:
    p, _ = _planner({"steps": [{"action": "forward", "value": 1}]})
    result = p.plan(WAV)
    assert result["heard"] == ""
    assert result["answer"] == "none"


def test_unclear_answer_yields_no_steps() -> None:
    p, _ = _planner({"heard": "mmfph", "answer": "unclear", "steps": []})
    result = p.plan(WAV)
    assert result["answer"] == "unclear"
    assert to_legacy_steps(result["steps"]) == []


# --------------------------------------------------------------------------- #
# Wiring
# --------------------------------------------------------------------------- #

def test_openai_and_google_stt_are_gone() -> None:
    src = (REPO / "voice_session.py").read_text()
    for gone in ("openai", "recognize_google", "query_gpt", "parse_gpt_steps"):
        assert gone not in src, f"{gone} still referenced in voice_session.py"
    reqs = (REPO / "requirements.txt").read_text()
    assert "openai" not in reqs, "openai still in requirements.txt"
    assert "google-genai" in reqs


def test_prompt_drops_the_homophone_hack() -> None:
    """That instruction only existed because the model saw a transcript.
    Gemini hears the audio, so it is not just unnecessary — it is misleading."""
    from prompts_and_glossary import audio_movement_prompt

    assert "turn write" not in audio_movement_prompt
    assert "AUDIO" in audio_movement_prompt
    assert "heard" in audio_movement_prompt


def test_yes_no_actually_gesture() -> None:
    """A yes/no answer must move the robot. It is the only reply channel —
    logging it and standing still is indistinguishable from being ignored."""
    import ast

    tree = ast.parse((REPO / "voice_session.py").read_text())
    fn = next(n for n in ast.walk(tree)
              if isinstance(n, ast.FunctionDef) and n.name == "run")
    src = ast.dump(fn)
    assert "_gestures" in src, "run() never plays a gesture"
    assert "gesture.skip" not in src, "yes/no is still being skipped"


def test_unclear_shares_the_no_gesture() -> None:
    """Locked design: 'no' and 'didn't understand' are the same head-shake."""
    from gestures import NO, UNCLEAR

    assert UNCLEAR is NO


def test_prompt_teaches_unclear() -> None:
    """The model can only emit 'unclear' reliably if the prompt says when."""
    from prompts_and_glossary import audio_movement_prompt as P

    assert '"unclear"' in P, "unclear is not described in the prompt"
    assert "inaudible" in P, "no trigger conditions given for unclear"
    # The model must know the gesture is shared with "no" — the operator sees
    # one thing for two different situations, and it cannot reason about that
    # unless told. Phrasing is free; the fact is not.
    lowered = P.lower()
    assert ("identical" in lowered or "same head-shake" in lowered
            or "same as" in lowered), \
        "prompt does not tell the model that no and unclear look the same"
    # A rule alone triggers less reliably than a rule plus a worked example.
    assert 'answer "unclear"' in P, "no worked example of answering unclear"


def test_prompt_biases_ambiguous_audio_toward_stop() -> None:
    """A missed stop is the one failure with a physical cost."""
    from prompts_and_glossary import audio_movement_prompt as P

    lowered = P.lower()
    assert "never answer \"unclear\" when the command might have been \"stop\"" \
        in lowered or ("unclear" in lowered and "might have been" in lowered), \
        "prompt does not bias ambiguous audio toward stopping"


def test_prompt_gesture_geometry_matches_config() -> None:
    """The prompt quotes the gesture shape. If config drifts, it starts lying
    to the model about what its own answers do."""
    from prompts_and_glossary import audio_movement_prompt as P

    assert f"{config.GESTURE_YES_METERS} m" in P, \
        f"prompt does not match GESTURE_YES_METERS={config.GESTURE_YES_METERS}"
    assert f"{config.GESTURE_NO_DEGREES:.0f} deg" in P, \
        f"prompt does not match GESTURE_NO_DEGREES={config.GESTURE_NO_DEGREES}"
    assert f"{config.GESTURE_NO_DEGREES * 2:.0f} deg" in P


def test_schema_description_matches_the_prompt() -> None:
    desc = RESPONSE_SCHEMA["properties"]["answer"]["description"].lower()
    assert "unclear" in desc and "stop" in desc, \
        "schema description must cover unclear and the stop bias"
    assert "empty steps" in desc, \
        "schema must say an answer comes with no steps"


def test_sync_gesturer_bypasses_history() -> None:
    """origin() walks the history stack; conversational nods in it would make
    'return to origin' replay the conversation."""
    from gestures import SyncGesturer

    calls = []

    class FakeMove:
        def straight(self, v): calls.append(("straight", v))
        def reverse(self, v): calls.append(("reverse", v))
        def left(self, v): calls.append(("left", v))
        def right(self, v): calls.append(("right", v))

    g = SyncGesturer(FakeMove())
    assert g.play("yes") is True
    assert calls == [("straight", 0.1), ("reverse", 0.1)]

    calls.clear()
    g.play("no")
    assert calls == [("left", 30.0), ("right", 60.0), ("left", 30.0)]


def test_gesture_failure_does_not_kill_the_loop() -> None:
    from gestures import SyncGesturer

    class BrokenMove:
        def left(self, v): raise RuntimeError("serial gone")
        def straight(self, v): pass
        def reverse(self, v): pass
        def right(self, v): pass

    assert SyncGesturer(BrokenMove()).play("no") is False


def test_session_uses_gemini_planner() -> None:
    src = (REPO / "voice_session.py").read_text()
    assert "GeminiVoicePlanner" in src
    assert "get_wav_data" in src, "audio is not being converted for upload"
    assert f"config.GEMINI_AUDIO_RATE" in src, "upload rate not from config"


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = 0
    for fn in fns:
        try:
            fn()
            print(f"  PASS  {fn.__name__}")
        except Exception as e:                       # noqa: BLE001
            failed += 1
            print(f"  FAIL  {fn.__name__}: {type(e).__name__}: {e}")
    print(f"\n{len(fns) - failed}/{len(fns)} passed")
    sys.exit(1 if failed else 0)
