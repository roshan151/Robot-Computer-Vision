"""
Low-level Serial bridge to Arduino drivetrain — v2.

Key change: motion commands use the new "M:<dir>,<speed>,<ticks>" protocol.
The Arduino auto-stops and sends ACK when the encoder target is reached.
The Pi blocks in send_raw() the whole time — no time.sleep() needed for moves.
"""

from __future__ import annotations

import logging
import queue
import threading
import time
from typing import Callable, Optional

import serial

import config
from serial_protocol import ACK, ERR, parse_enc_line

logger = logging.getLogger(__name__)

# How long (seconds) to wait for an ACK on a *move* command.
# Set generously — the Arduino will ACK as soon as ticks are done.
MOVE_ACK_TIMEOUT = 30.0
# How long to wait for ACK on fast commands (S, V:, F, B, L, R)
CMD_ACK_TIMEOUT  = 2.0


class ArduinoBridge:
    def __init__(
        self,
        port: str = config.SERIAL_PORT,
        baud: int = config.BAUD_RATE,
        timeout: float = config.SERIAL_TIMEOUT,
        on_encoder: Optional[Callable[[int, int], None]] = None,
        on_error:   Optional[Callable[[], None]]         = None,
    ) -> None:
        self._ser        = serial.Serial(port, baud, timeout=timeout)
        self._lock       = threading.Lock()
        self._on_encoder = on_encoder
        self._on_error   = on_error

        self._cmd_q        : queue.Queue[str] = queue.Queue()
        self._awaiting_ack = threading.Event()
        self._reader_stop  = threading.Event()
        self._closed       = False
        # _reader is created later, after the Arduino reset handshake.
        # We assign None up front so close() is safe even if init fails.
        self._reader: Optional[threading.Thread] = None

        # ---- Connect handshake ----
        # On macOS + FTDI, simply opening the port does NOT reset the Arduino,
        # so any state from a previous session (mid-move motors, dirty line
        # buffer, encoder counts, etc.) leaks into this one. Force a reset
        # via a DTR pulse, wait for the bootloader to finish, then drain the
        # OS input buffer BEFORE the reader thread starts — that way no stale
        # boot ACK or junk byte can be misattributed to a later command.
        self._reset_arduino_and_drain()

        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

        # Sanity-ping the Arduino. If this fails we cannot trust anything
        # else, so close out cleanly and surface the error to the caller.
        try:
            self._send("S", ack_timeout=2.0)
        except Exception as exc:
            logger.error("Arduino handshake ping failed: %s", exc)
            self.close()
            raise

    def _reset_arduino_and_drain(self) -> None:
        """
        Pulse DTR low→high to reboot the Arduino, then wait long enough for
        its bootloader + setup() to finish and discard everything that came
        across the wire during that window. Must be called before the reader
        thread starts so we don't race it on reset_input_buffer().
        """
        try:
            self._ser.setDTR(False)
            time.sleep(0.05)
            self._ser.setDTR(True)
        except Exception as exc:
            # Some adapters don't expose DTR; not fatal, the ping below will
            # still tell us whether the link is healthy.
            logger.warning("DTR reset failed (continuing): %s", exc)

        # Bootloader is ~1.5 s on a UNO; give it real margin.
        time.sleep(2.5)

        # Drain whatever arrived: boot ACK, ENC: telemetry, partial lines.
        try:
            self._ser.reset_input_buffer()
        except Exception:
            pass

    # ------------------------------------------------------------------ #
    # Lifecycle
    # ------------------------------------------------------------------ #

    def close(self) -> None:
        # Idempotent: safe to call multiple times.
        if self._closed:
            return
        self._closed = True
        try:
            self.stop()
        except Exception:
            pass
        self._reader_stop.set()
        if self._reader is not None:
            try:
                self._reader.join(timeout=1.0)
            except Exception:
                pass
        try:
            self._ser.close()
        except Exception:
            pass

    # ------------------------------------------------------------------ #
    # Reader thread
    # ------------------------------------------------------------------ #

    def _read_loop(self) -> None:
        buf = bytearray()
        while not self._reader_stop.is_set():
            chunk = self._ser.read(64)
            if not chunk:
                continue
            for b in chunk:
                if b in (0x0A, 0x0D):
                    if buf:
                        try:
                            line = buf.decode("utf-8", errors="replace").strip()
                        except Exception:
                            line = ""
                        buf.clear()
                        if line:
                            self._dispatch_line(line)
                else:
                    if len(buf) < 128:
                        buf.append(b)

    def _dispatch_line(self, line: str) -> None:
        logger.debug("RX: %r  awaiting_ack=%s", line, self._awaiting_ack.is_set())

        if line in (ACK, ERR):
            if self._awaiting_ack.is_set():
                self._cmd_q.put(line)
            elif line == ERR and self._on_error:
                self._on_error()
            return

        enc = parse_enc_line(line)
        if enc and self._on_encoder:
            self._on_encoder(enc[0], enc[1])

    # ------------------------------------------------------------------ #
    # Internal send helpers
    # ------------------------------------------------------------------ #

    def _drain_queue(self) -> None:
        """Discard any queued ACK/ERR lines."""
        while True:
            try:
                self._cmd_q.get_nowait()
            except queue.Empty:
                break

    def _send(self, cmd: str, ack_timeout: float = CMD_ACK_TIMEOUT) -> None:
        """
        Send a command and block until ACK (or raise on ERR / timeout).
        Thread-safe via self._lock.

        Note: we do NOT call self._ser.reset_input_buffer() here. The reader
        thread is consuming the same port concurrently; flushing the OS-level
        input buffer can race with the reader and discard a real ACK that
        just landed. Draining the cmd queue before setting awaiting_ack is
        sufficient: any old ACK/ERR is removed, and ENC: lines are routed
        away from the queue by _dispatch_line.
        """
        payload = (cmd.strip() + "\n").encode("utf-8")

        # Order matters: set the flag FIRST so any line received between
        # drain and write goes into the queue (and not silently dropped).
        self._awaiting_ack.set()
        self._drain_queue()
        try:
            with self._lock:
                self._ser.write(payload)
                self._ser.flush()
            self._wait_ack(ack_timeout)
        finally:
            self._awaiting_ack.clear()

    def _wait_ack(self, timeout_s: float) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            try:
                line = self._cmd_q.get(timeout=min(0.1, remaining))
            except queue.Empty:
                continue
            if line == ACK:
                return
            if line == ERR:
                raise RuntimeError("Arduino returned ERR")
        raise TimeoutError("no ACK from Arduino")

    # ------------------------------------------------------------------ #
    # Public API — intent commands (open-loop, immediate ACK)
    # ------------------------------------------------------------------ #

    def forward(self)  -> None: self._send("F")
    def backward(self) -> None: self._send("B")
    def left(self)     -> None: self._send("L")
    def right(self)    -> None: self._send("R")

    def stop(self) -> None:
        # Use a short timeout: stop should be near-instant, and we don't
        # want teardown to hang if the Arduino has already gone away.
        try:
            self._send("S", ack_timeout=0.5)
        except Exception:
            logger.warning("stop(): no ACK (already disconnected?)")

    def set_speed_pwm(self, value: int) -> None:
        v = max(0, min(255, int(value)))
        self._send(f"V:{v}")

    # ------------------------------------------------------------------ #
    # Public API — encoder-counted move (robust, blocking until done)
    # ------------------------------------------------------------------ #

    def move(self, direction: str, speed_pwm: int, ticks: int) -> None:
        """
        Send M:<dir>,<speed>,<ticks> and block until Arduino ACKs completion.

        direction : 'F' | 'B' | 'L' | 'R'
        speed_pwm : 0-255
        ticks     : encoder ticks to travel (avg of both wheels)

        The Arduino stops the motors and sends ACK when ticks are reached.
        No time.sleep() needed on the Pi side.
        """
        if direction not in ("F", "B", "L", "R"):
            raise ValueError(f"Invalid direction: {direction!r}")
        spd = max(0, min(255, int(speed_pwm)))
        cmd = f"M:{direction},{spd},{int(ticks)}"
        logger.debug("move -> %r (timeout=%.1fs)", cmd, MOVE_ACK_TIMEOUT)
        self._send(cmd, ack_timeout=MOVE_ACK_TIMEOUT)