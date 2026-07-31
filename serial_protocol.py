"""Line-oriented protocol over UART (newline-delimited)."""

from __future__ import annotations

from typing import Optional, Tuple

ACK  = "ACK"
ERR  = "ERR"
BOOT = "BOOT"   # Arduino just (re)started — "BOOT:<hex>" carries the reset cause

# AVR MCUSR reset-cause bits, reported by the firmware as "BOOT:<hex>".
# Plain-English meaning of each bit — this is the key diagnostic for
# distinguishing power problems from noise problems:
_RESET_CAUSE_BITS = (
    (0x1, "power-on (the Arduino's 5V supply dropped completely — "
          "USB power was interrupted: check the USB cable, connector, and port)"),
    (0x2, "external reset pin (normal when the serial port is opened via DTR; "
          "mid-move it means electrical noise reached the RESET pin)"),
    (0x4, "brown-out (the Arduino's 5V rail sagged below ~2.7V — "
          "something is dragging down or coupling into the 5V supply)"),
    (0x8, "watchdog flag (usually a bootloader side effect — ignore unless it appears alone)"),
)


def is_boot_line(line: str) -> bool:
    """True for both the legacy bare "BOOT" and the new "BOOT:<hex>" form."""
    return line == BOOT or line.startswith(BOOT + ":")


def describe_boot(line: str) -> str:
    """Human-readable reset cause from a BOOT line."""
    if ":" not in line:
        return "reset cause unknown (firmware without cause reporting)"
    try:
        flags = int(line.split(":", 1)[1], 16)
    except ValueError:
        return f"reset cause unparseable: {line!r}"
    causes = [text for bit, text in _RESET_CAUSE_BITS if flags & bit]
    if not causes:
        return "no cause flags set (unexpected — possibly an old bootloader)"
    return "; ".join(causes)


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
