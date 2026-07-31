"""
Low-level Serial bridge to Arduino drivetrain — v3.

Reliability fixes vs v2:
  - _send() holds _lock for the ENTIRE transaction (write + wait).
    Previously only the write was locked, which meant two callers could
    race on the ACK queue and one could drain the other's reply.
  - Handshake retries: if the first "S" ping times out the bridge retries
    up to HANDSHAKE_RETRIES times before raising — handles sluggish USB
    enumeration and occasional OS buffer hiccups.
  - Reset-wait duration is now taken from config.ARDUINO_DRAIN_WAIT_S
    so it can be tuned without touching code.
  - Read buffer capped at 256 bytes to prevent unbounded growth if the
    Arduino sends a very long line or framing goes wrong.
  - is_connected property for callers to gate on before sending commands.
  - Closed-state guard at the top of _send() for cleaner error messages.
"""

from __future__ import annotations

import logging
import queue
import threading
import time
from typing import Callable, Optional

import serial

import config
from serial_protocol import (
    ACK,
    describe_boot,
    describe_err,
    is_boot_line,
    is_err_line,
    parse_enc_line,
)

logger = logging.getLogger(__name__)

# How long (seconds) to wait for an ACK on a *move* command.
# Set generously — the Arduino will ACK as soon as ticks are done.
MOVE_ACK_TIMEOUT = 30.0
# How long to wait for ACK on fast commands (S, V:, F, B, L, R)
CMD_ACK_TIMEOUT  = 2.0

_READ_BUF_MAX = 256  # bytes; lines longer than this are discarded


class ArduinoBridge:
    def __init__(
        self,
        port: str = config.SERIAL_PORT,
        baud: int = config.BAUD_RATE,
        timeout: float = config.SERIAL_TIMEOUT,
        on_encoder: Optional[Callable[[int, int], None]] = None,
        on_error:   Optional[Callable[[], None]]         = None,
    ) -> None:
        self._on_encoder = on_encoder
        self._on_error   = on_error

        # _lock serialises the ENTIRE send-wait cycle so two callers can
        # never interleave their write and ACK consumption.
        self._lock = threading.Lock()

        self._cmd_q        : queue.Queue[str] = queue.Queue()
        self._awaiting_ack = threading.Event()
        self._reader_stop  = threading.Event()
        self._closed       = False
        self._reader: Optional[threading.Thread] = None

        logger.info("Opening serial port %s @ %d baud", port, baud)
        try:
            self._ser = serial.Serial(port, baud, timeout=timeout)
        except serial.SerialException as exc:
            raise RuntimeError(
                f"Cannot open serial port {port!r}: {exc}\n"
                "Check the port name (macOS: /dev/cu.usb*, Linux: /dev/ttyUSB0 or /dev/ttyACM0) "
                "and that nothing else (Arduino IDE Serial Monitor, etc.) has it open."
            ) from exc

        # Two-phase connect: try fast first, then fall back to DTR reset.
        # Reader thread is started before _connect() so we can call _send().
        self._reader = threading.Thread(
            target=self._read_loop, daemon=True, name="arduino-reader"
        )
        self._reader.start()

        try:
            self._connect()
        except Exception as exc:
            logger.error("Arduino handshake failed: %s", exc)
            self.close()
            raise

    # ------------------------------------------------------------------ #
    # Public properties
    # ------------------------------------------------------------------ #

    @property
    def is_connected(self) -> bool:
        """True if the bridge is open and the reader thread is alive."""
        return (
            not self._closed
            and self._reader is not None
            and self._reader.is_alive()
        )

    # ------------------------------------------------------------------ #
    # Connection setup
    # ------------------------------------------------------------------ #

    # ---- drain helper ---------------------------------------------------- #

    def _drain(self, wait_s: float = 0.0) -> None:
        """Sleep, then flush OS + PySerial input buffers."""
        if wait_s > 0:
            time.sleep(wait_s)
        try:
            self._ser.reset_input_buffer()
        except Exception as exc:
            logger.warning("Could not drain input buffer: %s", exc)
        self._drain_queue()

    # ---- DTR reset ------------------------------------------------------- #

    def _dtr_reset(self) -> None:
        """Pulse DTR low→high to reboot the Arduino."""
        try:
            self._ser.setDTR(False)
            time.sleep(0.05)
            self._ser.setDTR(True)
            logger.debug("DTR reset pulse sent")
        except Exception as exc:
            logger.warning("DTR reset not available on this adapter: %s", exc)

    # ---- ping helper ----------------------------------------------------- #

    def _ping_once(self, timeout_s: float) -> str:
        """
        Send "S" and return the first response token: "ACK", "BOOT:<cause>",
        "ERR", or "TIMEOUT".  Never raises.

        BOOT means the Arduino just booted (valid during connect, invalid
        during moves — callers decide how to interpret it).
        """
        self._drain()
        self._awaiting_ack.set()
        try:
            payload = b"S\n"
            self._ser.write(payload)
            self._ser.flush()
            deadline = time.monotonic() + timeout_s
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return "TIMEOUT"
                try:
                    token = self._cmd_q.get(timeout=min(0.1, remaining))
                    return token  # ACK, BOOT, or ERR
                except queue.Empty:
                    continue
        except Exception as exc:
            logger.warning("_ping_once write error: %s", exc)
            return "TIMEOUT"
        finally:
            self._awaiting_ack.clear()

    # ---- main connect logic ---------------------------------------------- #

    def _connect(self) -> None:
        """
        Adaptive handshake with BOOT awareness.

        The Arduino sends "BOOT" from setup() (not "ACK") so the Pi can
        distinguish a hardware reset from a completed command.  During the
        connect phase BOOT is as good as ACK — it means the Arduino is alive
        and ready.  During moves, BOOT means the Arduino reset mid-command
        (motor power spike) and _wait_ack raises immediately.

        Connection strategy:
          1. Drain briefly (catches any BOOT from macOS auto-reset on port open).
          2. Ping with a short timeout.
             - ACK  → Arduino was already running, connected.
             - BOOT → Arduino just (re)booted, connected.
             - TIMEOUT → proceed to Phase 2.
          3. DTR reset, wait for bootloader, drain, retry up to HANDSHAKE_RETRIES.
        """
        port    = self._ser.port
        baud    = self._ser.baudrate
        retries = config.HANDSHAKE_RETRIES
        timeout = config.HANDSHAKE_TIMEOUT_S
        total   = retries + 1   # 1 fast-path attempt + retries reset-path attempts

        logger.info("Connecting on %s @ %d baud …", port, baud)

        # ── Phase 1: fast ping (Arduino already running or auto-reset) ────
        self._drain(wait_s=0.2)
        result = self._ping_once(timeout_s=timeout)
        if result == ACK or is_boot_line(result):
            logger.info("Connected on %s (attempt 1/%d, response=%s)", port, total, result)
            return

        logger.info("Fast-path ping got %r — doing DTR reset and retrying", result)

        # ── Phase 2: DTR reset + longer wait ──────────────────────────────
        self._dtr_reset()
        # Wait for UNO bootloader (~1.5 s) + margin; BOOT arrives in this
        # window and is consumed by drain() so it doesn't confuse later pings.
        self._drain(wait_s=config.ARDUINO_DRAIN_WAIT_S)

        last_result = result
        for attempt in range(2, total + 1):
            result = self._ping_once(timeout_s=timeout)
            if result == ACK or is_boot_line(result):
                logger.info(
                    "Connected on %s (attempt %d/%d, response=%s, after DTR reset)",
                    port, attempt, total, result,
                )
                return
            last_result = result
            logger.warning("Handshake attempt %d/%d: got %r", attempt, total, result)
            if attempt < total:
                time.sleep(0.5)

        raise RuntimeError(
            f"Arduino did not respond after {total} handshake attempts "
            f"(last response: {last_result!r}). "
            f"Check USB cable, port {port!r}, and that the firmware is flashed."
        )

    # ------------------------------------------------------------------ #
    # Lifecycle
    # ------------------------------------------------------------------ #

    def close(self) -> None:
        """
        Idempotent shutdown: stop motors, kill reader thread, close port.

        Safe to call from a KeyboardInterrupt handler.  Uses a raw fire-and-
        forget write for the emergency stop so it never blocks on _wait_ack
        (which is the method that was being interrupted, meaning _lock may
        still be held and _awaiting_ack may still be set when we arrive here).
        """
        if self._closed:
            return
        self._closed = True
        logger.debug("Closing ArduinoBridge")

        # Clear the ack flag so _dispatch_line stops routing to the queue.
        # This must happen before the fire-and-forget stop write so that any
        # ENC: lines or leftover ACKs from the interrupted move don't confuse
        # a future open() in the same process.
        self._awaiting_ack.clear()
        self._drain_queue()

        # Fire-and-forget emergency stop: write raw bytes without waiting for
        # ACK.  This is intentionally not _send() — that calls _wait_ack()
        # which blocks on queue.get() and can be interrupted again.  A plain
        # serial.write() is not interruptible.
        #
        # Send "S" twice with a short gap — the first may arrive while the
        # Arduino is still processing the last move's ACK.  The second
        # guarantees the stop lands cleanly.  Then wait 150 ms so the
        # firmware's immediateStop() has time to zero the PWM outputs before
        # we close the port — without this wait the motors keep spinning
        # because the Arduino processes the stop command after the port closes.
        #
        # The sleep is wrapped in a KeyboardInterrupt shield so a second ^C
        # during teardown cannot skip the wait and leave motors running.
        try:
            self._ser.write(b"S\n")
            self._ser.flush()
        except Exception:
            pass

        # Shield the inter-command gap and settle wait from KeyboardInterrupt.
        # signal.setitimer is not available on Windows but works on macOS/Linux.
        import signal as _signal

        def _no_op(sig, frame): pass  # absorb any SIGINT during teardown

        old_handler = _signal.getsignal(_signal.SIGINT)
        try:
            _signal.signal(_signal.SIGINT, _no_op)
            try:
                self._ser.write(b"S\n")   # second stop for reliability
                self._ser.flush()
                time.sleep(0.15)          # let firmware zero PWM outputs
            except Exception:
                pass
        finally:
            _signal.signal(_signal.SIGINT, old_handler)

        self._reader_stop.set()
        if self._reader is not None:
            self._reader.join(timeout=1.5)
            if self._reader.is_alive():
                logger.warning("Reader thread did not exit cleanly within 1.5 s")

        try:
            self._ser.close()
            logger.debug("Serial port closed")
        except Exception as exc:
            logger.warning("Error closing serial port: %s", exc)

    # ------------------------------------------------------------------ #
    # Reader thread
    # ------------------------------------------------------------------ #

    def _read_loop(self) -> None:
        buf = bytearray()
        while not self._reader_stop.is_set():
            try:
                chunk = self._ser.read(64)
            except Exception as exc:
                if not self._reader_stop.is_set():
                    logger.error("Serial read error: %s", exc)
                break

            if not chunk:
                continue

            for b in chunk:
                if b in (0x0A, 0x0D):  # LF or CR — end of line
                    if buf:
                        try:
                            line = buf.decode("utf-8", errors="replace").strip()
                        except Exception:
                            line = ""
                        buf.clear()
                        if line:
                            self._dispatch_line(line)
                else:
                    if len(buf) < _READ_BUF_MAX:
                        buf.append(b)
                    else:
                        # Line exceeded max — framing error; reset.
                        logger.warning(
                            "RX line exceeded %d bytes (first 32: %r); discarding",
                            _READ_BUF_MAX,
                            bytes(buf[:32]),
                        )
                        buf.clear()

    def _dispatch_line(self, line: str) -> None:
        logger.debug("RX: %r  awaiting_ack=%s", line, self._awaiting_ack.is_set())

        if is_boot_line(line):
            if self._awaiting_ack.is_set():
                # Arduino reset while we were waiting for a command's ACK.
                # Push the full BOOT line into the queue so _wait_ack can
                # fail fast (with the reset cause) instead of blocking for
                # the full timeout (up to 30 s for moves).
                self._cmd_q.put(line)
            else:
                # Unexpected reset during idle — log prominently with cause.
                logger.error(
                    "Arduino reset while idle — %s", describe_boot(line)
                )
                if self._on_error:
                    self._on_error()
            return

        if line == ACK or is_err_line(line):
            if self._awaiting_ack.is_set():
                self._cmd_q.put(line)
            elif is_err_line(line):
                logger.error("Arduino reported ERR (unsolicited) — %s",
                             describe_err(line))
                if self._on_error:
                    self._on_error()
            return

        if line.startswith("WARN:JUNK,"):
            logger.warning(
                "Arduino received corrupted serial input (%s) — "
                "commands still ran; electrical noise on the serial line",
                line.split(",", 1)[1],
            )
            return

        if line.startswith("WARN:NOISE,"):
            # The firmware repaired a corrupted encoder reading and kept
            # driving.  Surface it — repeated warnings mean the encoder
            # wiring is picking up noise even though the move survived.
            logger.warning(
                "Encoder noise repaired mid-move (raw counts %s) — "
                "move continued; check encoder wire routing if frequent",
                line.split(",", 1)[1],
            )
            return

        enc = parse_enc_line(line)
        if enc is not None and self._on_encoder:
            self._on_encoder(enc[0], enc[1])

    # ------------------------------------------------------------------ #
    # Internal send helpers
    # ------------------------------------------------------------------ #

    def _drain_queue(self) -> None:
        """Discard stale ACK/ERR entries from the command queue."""
        while True:
            try:
                self._cmd_q.get_nowait()
            except queue.Empty:
                break

    def _wait_ack(self, timeout_s: float) -> None:
        """Block until ACK arrives. Raises on ERR, BOOT, or timeout."""
        deadline = time.monotonic() + timeout_s
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            try:
                line = self._cmd_q.get(timeout=min(0.1, remaining))
            except queue.Empty:
                continue
            if line == ACK:
                return
            if is_err_line(line):
                raise RuntimeError(f"Arduino returned ERR — {describe_err(line)}")
            if is_boot_line(line):
                raise RuntimeError(
                    f"Arduino reset mid-command — {describe_boot(line)}"
                )
        raise TimeoutError(f"No ACK from Arduino within {timeout_s:.1f} s")

    def _send(self, cmd: str, ack_timeout: float = CMD_ACK_TIMEOUT) -> None:
        """
        Send a command and block until ACK (or raise on ERR / timeout).

        Holds _lock for the ENTIRE transaction so concurrent callers cannot
        interleave their writes and ACK reads.

        Note: _wait_ack blocks on queue.get() which releases the GIL, so
        the reader thread can still call _dispatch_line and push into the
        queue even while this method holds _lock.
        """
        if self._closed:
            raise RuntimeError("ArduinoBridge is closed — call constructor again to reconnect")

        payload = (cmd.strip() + "\n").encode("utf-8")

        with self._lock:
            # Drain stale queue entries BEFORE arming the flag so old junk
            # doesn't satisfy our wait. Under the lock, no other _send() is
            # active, so the only things in the queue are leftovers from
            # before this call.
            self._drain_queue()
            self._awaiting_ack.set()
            time.sleep(0.01)
            self._drain_queue()
            try:
                logger.debug("TX: %r", cmd)
                self._ser.write(payload)
                self._ser.flush()
            except Exception:
                self._awaiting_ack.clear()
                raise
            try:
                self._wait_ack(ack_timeout)
            finally:
                self._awaiting_ack.clear()

    # ------------------------------------------------------------------ #
    # Public API — intent commands (open-loop, Arduino ACKs immediately)
    # ------------------------------------------------------------------ #

    def forward(self)  -> None: self._send("F")
    def backward(self) -> None: self._send("B")
    def left(self)     -> None: self._send("L")
    def right(self)    -> None: self._send("R")

    def stop(self) -> None:
        """Send stop with a short timeout — safe to call during teardown."""
        try:
            self._send("S", ack_timeout=0.5)
        except Exception:
            logger.warning("stop(): no ACK received (already disconnected?)")

    def set_speed_pwm(self, value: int) -> None:
        v = max(0, min(255, int(value)))
        self._send(f"V:{v}")

    # ------------------------------------------------------------------ #
    # Public API — encoder-counted move (blocks until Arduino ACKs done)
    # ------------------------------------------------------------------ #

    def move(self, direction: str, speed_pwm: int, ticks: int) -> None:
        """
        Send M:<dir>,<speed>,<ticks> and block until the Arduino signals
        that the encoder target has been reached (ACK), or raise on
        timeout (stall) or ERR.

        direction : 'F' | 'B' | 'L' | 'R'
        speed_pwm : 0-255 PWM duty cycle
        ticks     : encoder ticks to travel (average of both wheels)

        The Arduino's PID loop keeps both wheels in sync during straight
        moves. No time.sleep() is needed on the Pi side.
        """
        if direction not in ("F", "B", "L", "R"):
            raise ValueError(f"Invalid direction: {direction!r}")
        if not self.is_connected:
            raise RuntimeError("Arduino is not connected")

        spd  = max(0, min(255, int(speed_pwm)))
        tkns = max(1, int(ticks))
        cmd  = f"M:{direction},{spd},{tkns}"
        logger.debug("move -> %r (ack_timeout=%.1f s)", cmd, MOVE_ACK_TIMEOUT)
        self._send(cmd, ack_timeout=MOVE_ACK_TIMEOUT)
