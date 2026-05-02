"""
Manual movement test script for the robot drivetrain.

Examples:
  python test_bot_movements.py
  python test_bot_movements.py --sequence square --repeat 2 --speed 55
  python test_bot_movements.py --port /dev/ttyACM0 --baud 115200
"""

from __future__ import annotations

import argparse
import time
from typing import TYPE_CHECKING, Callable, Dict, List, Tuple

if TYPE_CHECKING:
    from drivetrain_client import SerialDrivetrain

Step = Tuple[str, float]

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Test bot movement commands over serial.")
    parser.add_argument(
        "--port",
        default="/dev/cu.usbserial-A5069RR4",
        help="Serial port override (default from ROBOT_SERIAL_PORT/config).",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Baud rate override (default from ROBOT_SERIAL_BAUD/config).",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=60.0,
        help="Movement speed percent (0-100). Default: 60.",
    )
    parser.add_argument(
        "--move-duration",
        type=float,
        default=3,
        help="Seconds for forward/reverse steps. Default: 3.",
    )
    parser.add_argument(
        "--turn-angle",
        type=float,
        default=90.0,
        help="Degrees for left/right steps. Default: 90.",
    )
    parser.add_argument(
        "--pause",
        type=float,
        default=0.25,
        help="Pause in seconds between steps. Default: 0.4.",
    )
    parser.add_argument(
        "--repeat",
        type=int,
        default=1,
        help="How many times to repeat selected sequence. Default: 1.",
    )
    parser.add_argument(
        "--sequence",
        choices=["basic", "square", "spin"],
        default="basic",
        help="Predefined movement sequence. Default: basic.",
    )
    return parser.parse_args()


def build_sequence(name: str, move_duration: float) -> List[Step]:
    if name == "square":
        return [
            ("forward", move_duration),
            ("right", 0),
            ("forward", move_duration),
            ("right", 0),
            ("forward", move_duration),
            ("right", 0),
            ("forward", move_duration),
            ("right", 0),
        ]
    if name == "spin":
        return [("left", 0), ("right", 0), ("left", 0), ("right", 0), ("stop", 0)]
    return [
        ("forward", move_duration),
        ("reverse", move_duration),
        ("left", 0),
        ("right", 0),
        ("stop", 0),
    ]


def execute_sequence(
    drivetrain: "SerialDrivetrain",
    sequence: List[Step],
    speed: float,
    turn_angle: float,
    pause_s: float,
    repeat: int,
) -> None:
    actions: Dict[str, Callable[[float, float], None]] = {
        "forward": lambda duration, spd: drivetrain.straight(duration=duration, speed=spd),
        "reverse": lambda duration, spd: drivetrain.reverse(duration=duration, speed=spd),
        "left":    lambda _duration, spd: drivetrain.left(angle=turn_angle, speed=spd),
        "right":   lambda _duration, spd: drivetrain.right(angle=turn_angle, speed=spd),
        "stop":    lambda _duration, _spd: drivetrain.stop(),
    }

    def label(step_name: str, duration: float) -> str:
        if step_name in ("forward", "reverse"):
            return f"duration={duration:.2f}s"
        if step_name in ("left", "right"):
            return f"angle={turn_angle:.1f}°"
        return "—"

    for cycle in range(1, repeat + 1):
        print(f"\n=== Cycle {cycle}/{repeat} ===")
        for step_name, duration in sequence:
            print(f"-> {step_name} ({label(step_name, duration)}, speed={speed:.1f}%)")

            before = drivetrain.get_encoder_status()
            t0 = time.monotonic()
            try:
                actions[step_name](duration, speed)
                status = "ok"
            except Exception as exc:
                status = f"error: {exc}"
            elapsed = time.monotonic() - t0
            after = drivetrain.get_encoder_status()

            dL = after["motor1_count"] - before["motor1_count"]
            dR = after["motor2_count"] - before["motor2_count"]
            print(
                f"   {status}  elapsed={elapsed:.2f}s  "
                f"ΔL={dL} ΔR={dR}  sync_error={dL - dR}"
            )

            if status != "ok":
                # Bail out of the cycle on the first error rather than
                # continuing to push commands into a broken connection.
                raise RuntimeError(f"step {step_name!r} failed: {status}")

            if pause_s > 0:
                time.sleep(pause_s)


def main() -> None:
    args = parse_args()
    repeat = max(1, args.repeat)
    from drivetrain_client import SerialDrivetrain

    try:
        if args.port is None and args.baud is None:
            drivetrain = SerialDrivetrain()
        elif args.port is None:
            drivetrain = SerialDrivetrain(baud=args.baud)
        elif args.baud is None:
            drivetrain = SerialDrivetrain(port=args.port)
        else:
            drivetrain = SerialDrivetrain(port=args.port, baud=args.baud)
    except TimeoutError:
        print("\nERROR: Timed out waiting for ACK from Arduino.")
        # print("Try these fixes:")
        # print("  1) Use the correct serial port (macOS is usually /dev/cu.usb*).")
        # print("  2) Close Arduino Serial Monitor / any app using the same port.")
        # print("  3) Press Arduino reset once, then rerun this script.")
        # print("  4) Confirm firmware baud is 115200 and sketch is uploaded.")
        raise SystemExit(2)
    except Exception as exc:
        print(f"\nERROR: Could not connect to drivetrain: {exc}")
        raise SystemExit(2)

    sequence = build_sequence(args.sequence, args.move_duration)

    print("Connected to drivetrain.")
    print(
        "Running sequence:",
        ", ".join(name for name, _ in sequence),
        f"(repeat={repeat}, speed={args.speed:.1f}%, turn_angle={args.turn_angle:.1f})",
    )

    try:
        execute_sequence(
            drivetrain=drivetrain,
            sequence=sequence,
            speed=args.speed,
            turn_angle=args.turn_angle,
            pause_s=max(0.0, args.pause),
            repeat=repeat,
        )
    finally:
        # close() is idempotent and already issues a stop() with a short
        # timeout, so we don't call drivetrain.stop() separately.
        drivetrain.close()
        print("\nStopped and closed drivetrain connection.")


if __name__ == "__main__":
    main()
