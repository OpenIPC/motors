"""Transparent two-port forwarder with logging.

Bytes are forwarded as soon as read() returns them, never held back for
framing, so the timing the PTZ board sees differs from a direct wire only by
the USB round trip (FTDI latency timer, 16 ms by default; see set_latency()).
"""

from __future__ import annotations

import os
import selectors
import signal
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

import serial

from .framing import CAM_RULES, Framer
from .log import C2P, H2P, P2C, LogWriter

# A probe is held back while a camera frame is half-forwarded, unless the
# camera has been quiet this long (it stopped mid-frame).
PROBE_HOLD_NS = 20_000_000


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
    bytes: dict = field(default_factory=lambda: {C2P: 0, P2C: 0, H2P: 0})
    reads: dict = field(default_factory=lambda: {C2P: 0, P2C: 0, H2P: 0})


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
    control: int | None = None,
    duration: float | None = None,
    on_mark: Callable[[int, str], None] | None = None,
) -> Stats:
    """Forward cam<->ptz until stopped.

    `control` is a readable fd (normally stdin). Each line read from it is
    either `!<hex>`, sent to the PTZ board and logged as h2p (a probe), or
    free text, logged as a mark. Probes are only written between camera
    frames, so they never splice into one on the PTZ wire."""
    stats = Stats()
    sel = selectors.DefaultSelector()
    sel.register(cam.fileno(), selectors.EVENT_READ, (cam, ptz, C2P))
    sel.register(ptz.fileno(), selectors.EVENT_READ, (ptz, cam, P2C))
    if control is not None:
        sel.register(control, selectors.EVENT_READ, None)
    pending = b""
    probes: list[bytes] = []
    cam_framer = Framer(CAM_RULES)
    last_c2p = 0
    t0 = time.monotonic_ns()
    last_flush = t0

    def send_probes(t: int) -> None:
        if not probes or (cam_framer.pending() and t - last_c2p < PROBE_HOLD_NS):
            return
        for probe in probes:
            ptz.write(probe)
            writer.data(t, H2P, probe)
            stats.bytes[H2P] += len(probe)
            stats.reads[H2P] += 1
            if on_data:
                on_data(t, H2P, probe)
        probes.clear()
    try:
        while not stopper.stop:
            now = time.monotonic_ns()
            if duration is not None and now - t0 >= duration * 1e9:
                break
            for key, _ in sel.select(timeout=0.2):
                t = time.monotonic_ns() - t0
                if key.data is None:
                    chunk = os.read(control, 4096)
                    if not chunk:
                        sel.unregister(control)
                        continue
                    pending += chunk
                    *lines, pending = pending.split(b"\n")
                    for raw in lines:
                        line = raw.decode(errors="replace").strip()
                        if not line:
                            continue
                        probe = parse_probe(line)
                        if probe:
                            probes.append(probe)
                        else:
                            writer.mark(t, line)
                            if on_mark:
                                on_mark(t, line)
                    send_probes(t)
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
                if d == C2P:
                    cam_framer.feed(t, data)
                    last_c2p = t
                    send_probes(t)
            send_probes(time.monotonic_ns() - t0)
            if now - last_flush > 1e9:
                writer.flush()
                last_flush = now
    finally:
        sel.close()
        writer.flush()
    return stats


def parse_probe(line: str) -> bytes | None:
    if not line.startswith("!"):
        return None
    try:
        return bytes.fromhex(line[1:])
    except ValueError:
        return None
