"""Act as the camera: send frames to the PTZ board and log what it answers.

Only the PTZ port is opened. Stop the bridge first; with the lab wiring the
camera then has no path to the PTZ board, so the host is the only sender.
"""

from __future__ import annotations

import math
import selectors
import time
from typing import Callable

import serial

from .bridge import Stats, Stopper
from .log import C2P, H2P, P2C, LogWriter


def schedule_replay(records, speed: float = 1.0) -> list[tuple[int, bytes]]:
    """(offset_ns, bytes) for every record that went to the PTZ board, camera
    traffic and bridge probes alike, in capture order, relative to the first."""
    if not (speed > 0 and math.isfinite(speed)):
        raise ValueError(f"speed must be finite and > 0, got {speed}")
    sent = [r for r in records if r.d in (C2P, H2P)]
    if not sent:
        return []
    base = sent[0].t
    return [(int((r.t - base) / speed), r.data) for r in sent]


def schedule_frames(frames: list[bytes], rate: float, duration: float) -> list[tuple[int, bytes]]:
    """Cycle through `frames` at `rate` frames/s for `duration` seconds."""
    if not all(v > 0 and math.isfinite(v) for v in (rate, duration)):
        raise ValueError(f"rate and duration must be finite and > 0, got {rate}, {duration}")
    if not frames:
        raise ValueError("no frames to send")
    period = int(1e9 / rate)
    count = max(1, int(duration * rate))
    return [(i * period, frames[i % len(frames)]) for i in range(count)]


def run(
    ptz: serial.Serial,
    schedule: list[tuple[int, bytes]],
    writer: LogWriter,
    stopper: Stopper,
    on_data: Callable[[int, str, bytes], None] | None = None,
    tail: float = 0.5,
    on_abort: bytes = b"",
) -> Stats:
    """Send `schedule`, then keep listening for `tail` seconds.

    If stopped (SIGINT/SIGTERM) before the whole schedule went out, write
    `on_abort` before returning: a replayed move whose stop frame was still
    pending would otherwise keep the motor running."""
    stats = Stats()
    sel = selectors.DefaultSelector()
    sel.register(ptz.fileno(), selectors.EVENT_READ)
    t0 = time.monotonic_ns()
    i = 0
    end = None
    # Bytes still owed to the board's current frame: its parser takes any A5
    # or C5 as the start of an 8-byte frame, with no timeout (PROTOCOL.md).
    owed = 0

    def track(data: bytes) -> None:
        nonlocal owed
        for b in data:
            if owed:
                owed -= 1
            elif b in (0xA5, 0xC5):
                owed = 7
    try:
        while not stopper.stop:
            now = time.monotonic_ns() - t0
            while i < len(schedule) and schedule[i][0] <= now:
                data = schedule[i][1]
                ptz.write(data)
                track(data)
                t = time.monotonic_ns() - t0  # when it was written, not when it was due
                writer.data(t, C2P, data)
                stats.bytes[C2P] += len(data)
                stats.reads[C2P] += 1
                if on_data:
                    on_data(t, C2P, data)
                i += 1
            if i >= len(schedule):
                if end is None:
                    end = now + int(tail * 1e9)
                elif now >= end:
                    break
            wait_ns = (schedule[i][0] - now) if i < len(schedule) else (end - now)
            for _ in sel.select(timeout=max(0.0, min(wait_ns / 1e9, 0.2))):
                data = ptz.read(4096)
                if data:
                    t = time.monotonic_ns() - t0
                    writer.data(t, P2C, data)
                    stats.bytes[P2C] += len(data)
                    stats.reads[P2C] += 1
                    if on_data:
                        on_data(t, P2C, data)
    finally:
        if on_abort and i < len(schedule):
            # A partial frame on the wire would swallow the abort frame, so
            # complete it first (with zero bytes, which start nothing).
            pad = bytes(owed)
            ptz.write(pad + on_abort)
            t = time.monotonic_ns() - t0
            writer.mark(t, f"stopped with {len(schedule) - i} writes pending; sent on-abort frame"
                           + (f" after {len(pad)} padding bytes" if pad else ""))
            writer.data(t, C2P, pad + on_abort)
        sel.close()
        writer.flush()
    return stats
