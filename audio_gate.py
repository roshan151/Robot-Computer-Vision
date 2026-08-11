"""REMOVED — replaced by the Gemini Live agent.

The turn-based path (push-to-listen, per-utterance upload, ambient calibration,
local speech gate, request budget) no longer exists. Its job was deciding WHEN
to spend a request; a streaming session has no discrete requests to spend.

See live_agent.py and robot_tools.py.
Recover the old implementation from git history or
documentation/reverted-2026-08-11.patch.
"""

raise ImportError(__doc__)
