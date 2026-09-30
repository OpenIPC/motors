"""Transparent two-port forwarder with logging.

Bytes are forwarded as soon as read() returns them, never held back for
framing, so the timing the PTZ board sees differs from a direct wire only by
the USB round trip (FTDI latency timer, 16 ms by default; see set_latency()).
"""

from __future__ import annotations

import os
import selectors
import signal
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, TextIO

import serial

from .log import C2P, P2C, LogWriter


def open_port(path: str, baud: int) -> serial.Serial:
    return serial.Serial(
        path, baud,
        bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0, write_timeout=1,
        xonxoff=False, rtscts=False, dsrdtr=False,
        exclusive=True,
    )


def set_latency(path: str, ms: int) -> str:
    """Best effort: lower the FTDI latency timer through sysfs.

    Returns a short status string for the capture header."""
    tty = Path(os.path.realpath(path)).name
    node = Path("/sys/bus/usb-serial/devices") / tty / "latency_timer"
    try:
        node.write_text(f"{ms}\n")
        return f"{node.read_text().strip()}ms"
    except OSError as e:
        try:
            return f"{node.read_text().strip()}ms (set failed: {e.strerror})"
        except OSError:
            return "n/a"


@dataclass
class Stats:
    bytes: dict = field(default_factory=lambda: {C2P: 0, P2C: 0})
    reads: dict = field(default_factory=lambda: {C2P: 0, P2C: 0})


class Stopper:
    """Turns SIGINT/SIGTERM into a flag the loop polls."""

    def __init__(self) -> None:
        self.stop = False

    def install(self) -> None:
        signal.signal(signal.SIGINT, self._set)
        signal.signal(signal.SIGTERM, self._set)

    def _set(self, *_: object) -> None:
        self.stop = True


def run(
    cam: serial.Serial,
    ptz: serial.Serial,
    writer: LogWriter,
    stopper: Stopper,
    on_data: Callable[[int, str, bytes], None] | None = None,
    marks: TextIO | None = None,
    duration: float | None = None,
) -> Stats:
    """Forward cam<->ptz until stopped. `marks` lines become mark records."""
    stats = Stats()
    sel = selectors.DefaultSelector()
    sel.register(cam.fileno(), selectors.EVENT_READ, (cam, ptz, C2P))
    sel.register(ptz.fileno(), selectors.EVENT_READ, (ptz, cam, P2C))
    if marks is not None:
        sel.register(marks.fileno(), selectors.EVENT_READ, None)
    t0 = time.monotonic_ns()
    last_flush = t0
    try:
        while not stopper.stop:
            now = time.monotonic_ns()
            if duration is not None and now - t0 >= duration * 1e9:
                break
            for key, _ in sel.select(timeout=0.2):
                t = time.monotonic_ns() - t0
                if key.data is None:
                    line = marks.readline()
                    if not line:
                        sel.unregister(key.fileobj)
                        continue
                    writer.mark(t, line.rstrip("\n"))
                    continue
                src, dst, d = key.data
                data = src.read(4096)
                if not data:
                    continue
                dst.write(data)
                writer.data(t, d, data)
                stats.bytes[d] += len(data)
                stats.reads[d] += 1
                if on_data:
                    on_data(t, d, data)
            if now - last_flush > 1e9:
                writer.flush()
                last_flush = now
    finally:
        sel.close()
        writer.flush()
    return stats


def stdin_marks() -> TextIO | None:
    return sys.stdin if sys.stdin.isatty() else None
