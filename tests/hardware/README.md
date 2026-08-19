# Hardware diagnostics

Scripts, not pytest tests — each needs a robot on the other end of the serial
port, and each is run by hand. They are named `check_*` rather than `test_*`
precisely so `pytest` does not try to collect them.

| Script | What it tells you |
|---|---|
| `check_protocol.py` | Frame encode/decode and checksums, no robot needed |
| `check_encoders.py` | **Run this after any harness change.** Hand-roll each wheel forward: both counters must go *up*, and rolling the left wheel must move `enc_left`. It is the only way to tell a swapped A/B pair from a mirror-mounted motor |
| `check_movements.py` | Closed-loop moves end to end |
| `check_timed.py` | Timing and sync-error behaviour across a run |

```bash
python tests/hardware/check_encoders.py
```

Order matters: encoders first. Calibration numbers mean nothing until
`check_encoders.py` passes all four of its checks.
