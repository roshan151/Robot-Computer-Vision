"""
Raw serial connection diagnostic — bypasses ArduinoBridge threading entirely.
Run this to see exactly what the Arduino sends and whether it ACKs.

Usage:
  python test_connection.py
"""

import serial
import time

PORT = "/dev/cu.usbserial-A5069RR4"
BAUD = 115200


def try_connect(label: str, dtr_reset: bool, wait_s: float) -> bool:
    print(f"\n{'─'*55}")
    print(f"TEST: {label}")
    print(f"  DTR reset: {dtr_reset}   wait: {wait_s}s")

    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    print("  Port opened OK")

    if dtr_reset:
        ser.setDTR(False)
        time.sleep(0.1)
        ser.setDTR(True)

    print(f"  Waiting {wait_s}s for Arduino to boot...")
    time.sleep(wait_s)

    # Show everything that arrived during the wait
    waiting = ser.in_waiting
    if waiting:
        raw = ser.read(waiting)
        print(f"  Bytes in buffer after wait: {raw!r}")
    else:
        print("  Buffer empty after wait")

    ser.reset_input_buffer()

    print("  Sending: b'S\\n'")
    ser.write(b"S\n")
    ser.flush()

    print("  Waiting up to 3s for any response...")
    deadline = time.time() + 3.0
    got_ack = False
    while time.time() < deadline:
        line = ser.readline()
        if line:
            print(f"    RX: {line!r}")
            if b"ACK" in line:
                got_ack = True
                print("    ^^^ GOT ACK — handshake would succeed")

    if not got_ack:
        print("  TIMEOUT — no ACK received in 3s")

    ser.close()
    return got_ack


# ── Test 1: what our bridge currently does ────────────────────────────────
result1 = try_connect(
    label="Current bridge behaviour (DTR reset + 2.5s wait)",
    dtr_reset=True,
    wait_s=2.5,
)

time.sleep(1.0)

# ── Test 2: skip DTR reset entirely ──────────────────────────────────────
result2 = try_connect(
    label="No DTR reset (Arduino already running)",
    dtr_reset=False,
    wait_s=0.3,
)

time.sleep(1.0)

# ── Test 3: longer wait in case bootloader is slow ────────────────────────
result3 = try_connect(
    label="DTR reset + 5s wait (slow bootloader fallback)",
    dtr_reset=True,
    wait_s=5.0,
)

print(f"\n{'='*55}")
print(f"Results:")
print(f"  DTR + 2.5s : {'PASS' if result1 else 'FAIL'}")
print(f"  No DTR     : {'PASS' if result2 else 'FAIL'}")
print(f"  DTR + 5.0s : {'PASS' if result3 else 'FAIL'}")
