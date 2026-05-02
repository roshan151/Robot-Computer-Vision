"""Line-oriented protocol over UART (newline-delimited)."""

from __future__ import annotations

from typing import Optional, Tuple

ACK = "ACK"
ERR = "ERR"
PING = "PING"


def enc_line(left: int, right: int) -> str:
    return f"ENC:{left},{right}"


def parse_enc_line(line: str) -> Optional[Tuple[int, int]]:
    line = line.strip()
    if not line.startswith("ENC:"):
        return None
    rest = line[4:]
    parts = rest.split(",", 1)
    if len(parts) != 2:
        return None
    try:
        return int(parts[0]), int(parts[1])
    except ValueError:
        return None
