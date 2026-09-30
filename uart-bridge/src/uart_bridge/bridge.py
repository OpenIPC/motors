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

from .framing import CAM_RULES, SYNC_BYTES, Frame, Framer
from .log import C2M, C2P, C2T, H2P, P2C, T2C, LogWriter

# A probe is held back while a camera frame is half-forwarded. The rest of a
# frame can lag by the USB latency timer (up to 16 ms) plus scheduling, so
# only a silence far beyond that, 20 frame periods, counts as the camera
# having stopped mid-frame; the probe is then sent and the stall is logged.
CAMERA_STALL_NS = 1_000_000_000

# The XM board's parser takes A5 or C5 as the start of an 8-byte frame and
# has no inter-byte timeout, so a partial frame on its wire swallows the next
# command. Camera frames are 0.7 ms long with ~49 ms between them: a read that
# follows this much silence starts a frame. Until one does, camera bytes are
# logged but not forwarded, and on exit the frame in flight is completed.
FRAME_GAP_NS = 5_000_000
FINISH_FRAME_S = 0.1


class PtyPort:
    """The master side of a pseudo terminal, with the slice of the
    serial.Serial interface the bridge uses. A program under test opens
    `self.path` as if it were the camera's UART."""

    def __init__(self) -> None:
        import tty
        self.master, self._slave = os.openpty()
        tty.setraw(self._slave)  # keep the slave open: no EIO when the tool closes it
        os.set_blocking(self.master, False)
        self.path = os.ttyname(self._slave)

    def fileno(self) -> int:
        return self.master

    def read(self, n: int) -> bytes:
        try:
            return os.read(self.master, n)
        except BlockingIOError:
            return b""

    WRITE_DEADLINE_S = 0.05

    def write(self, data: bytes) -> int:
        """Write what the pty takes within WRITE_DEADLINE_S and return that
        count. With no tool reading the slave the queue fills up; blocking
        there would stall the whole bridge, so the rest is dropped."""
        view = memoryview(data)
        end = time.monotonic() + self.WRITE_DEADLINE_S
        while view and time.monotonic() < end:
            try:
                view = view[os.write(self.master, view):]
            except BlockingIOError:
                time.sleep(0.001)
        return len(data) - len(view)

    @property
    def in_waiting(self) -> int:
        return 0

    def reset_input_buffer(self) -> None:
        while self.read(4096):
            pass

    def close(self) -> None:
        os.close(self.master)
        os.close(self._slave)


def trailing_commands(data: bytes) -> int | None:
    """Offset of a run of whole frames that ends `data` and contains at
    least one checkable command (C5 ... 5C), or None.

    Used while joining the camera stream mid-frame: a read that merges the
    tail of one frame with complete frames behind it has no timing to tell
    where the tail ends, but a C5 frame carries its own 5C end byte. A5
    frames do not, so a tail of A5 frames alone is not trusted."""
    for p in range(1, len(data) - 7):
        rest = data[p:]
        if len(rest) % 8:
            continue
        chunks = [rest[i:i + 8] for i in range(0, len(rest), 8)]
        if all(c[0] in SYNC_BYTES and (c[0] != 0xC5 or c[7] == 0x5C) for c in chunks) \
                and any(c[0] == 0xC5 for c in chunks):
            return p
    return None


def is_url(path: str) -> bool:
    return "://" in path


def open_port(path: str, baud: int) -> serial.Serial:
    """A serial device, or a pyserial URL such as socket://host:port (e.g. a
    remote xm-uart -l relay in front of another camera's lens board)."""
    if is_url(path):
        return serial.serial_for_url(path, baudrate=baud, timeout=0, write_timeout=1)
    return serial.Serial(
        path, baud,
        bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0, write_timeout=1,
        xonxoff=False, rtscts=False, dsrdtr=False,
        exclusive=True,
    )


@dataclass
class Drained:
    bytes: int
    last_ns: int | None  # time.monotonic_ns() of the last byte seen, None if the line was quiet


def drain_stale(port: serial.Serial, settle_s: float = 0.03) -> Drained:
    """Discard input that piled up before we opened the port.

    While nothing reads a port, the kernel and the FTDI chip keep buffering
    what the board sends; on open that backlog (kilobytes, and garbled once
    the buffers wrap) arrives in a burst and the bridge would forward it to
    the other board. Flush, then keep discarding for `settle_s` so bytes
    still in flight from the adapter are dropped too. The time of the last
    byte seen tells run() whether the camera line was quiet when it started."""
    dropped = port.in_waiting
    last = time.monotonic_ns() if dropped else None
    port.reset_input_buffer()
    end = time.monotonic() + settle_s
    while time.monotonic() < end:
        n = len(port.read(4096))
        if n:
            dropped += n
            last = time.monotonic_ns()
        time.sleep(0.002)
    return Drained(dropped, last)


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
    bytes: dict = field(default_factory=lambda: dict.fromkeys((C2P, P2C, H2P, C2M, C2T, T2C), 0))
    reads: dict = field(default_factory=lambda: dict.fromkeys((C2P, P2C, H2P, C2M, C2T, T2C), 0))


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
    cam_last_ns: int | None = None,
    mute_cam: bool = False,
    tee: serial.Serial | None = None,
) -> Stats:
    """Forward cam<->ptz until stopped.

    `cam_last_ns` is when the camera line last carried a byte before the
    bridge started (from drain_stale); None means it was quiet.

    With `tee`, every whole frame the camera sends to its board is also
    written to the tee (logged c2t), and what the tee answers is logged as
    t2c but not passed to the camera. The tee gets frames only, never a
    partial one or junk such as the boot console.

    With `mute_cam`, camera bytes are logged but not forwarded: the PTZ board
    hears nothing from the camera (probes still go through). Used to tell
    what the board does on its own from what the camera makes it do.

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
    if tee is not None:
        sel.register(tee.fileno(), selectors.EVENT_READ, (tee, None, T2C))
    pending = b""
    probes: list[bytes] = []
    cam_framer = Framer(CAM_RULES)
    last_c2p = 0
    cam_synced = False
    t0 = time.monotonic_ns()
    # When the camera last sent anything, relative to t0.
    prev_c2p_read = (cam_last_ns - t0) if cam_last_ns is not None else -FRAME_GAP_NS
    last_flush = t0

    def send_probes(t: int) -> None:
        if not probes:
            return
        if cam_framer.pending():
            if t - last_c2p < CAMERA_STALL_NS:
                return
            note = f"camera stalled mid-frame ({cam_framer.pending().hex(' ')}); probe sent anyway"
            writer.mark(t, note)
            if on_mark:
                on_mark(t, note)
        for probe in probes:
            ptz.write(probe)
            writer.data(t, H2P, probe)
            stats.bytes[H2P] += len(probe)
            stats.reads[H2P] += 1
            if on_data:
                on_data(t, H2P, probe)
        probes.clear()
    def to_tee(t: int, items) -> None:
        if tee is None:
            return
        for item in items:
            if isinstance(item, Frame):
                tee.write(item.data)
                writer.data(t, C2T, item.data)
                stats.bytes[C2T] += len(item.data)
                stats.reads[C2T] += 1
                if on_data:
                    on_data(t, C2T, item.data)

    def finish_frame() -> None:
        """Forward the rest of a camera frame already partly on the PTZ wire."""
        nonlocal last_c2p
        end = time.monotonic() + FINISH_FRAME_S
        while cam_framer.pending() and time.monotonic() < end:
            data = cam.read(8 - len(cam_framer.pending()))
            if not data:
                time.sleep(0.001)
                continue
            t = time.monotonic_ns() - t0
            ptz.write(data)
            writer.data(t, C2P, data)
            stats.bytes[C2P] += len(data)
            stats.reads[C2P] += 1
            to_tee(t, cam_framer.feed(t, data))
            last_c2p = t
        if cam_framer.pending():
            writer.mark(time.monotonic_ns() - t0,
                        f"exited with a partial camera frame on the PTZ wire: {cam_framer.pending().hex(' ')}")

    try:
        while not stopper.stop:
            now = time.monotonic_ns()
            if duration is not None and now - t0 >= duration * 1e9:
                break
            for key, _ in sel.select(timeout=0.05):
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
                if d == T2C:  # the second board's replies: logged, not passed on
                    writer.data(t, T2C, data)
                    stats.bytes[T2C] += len(data)
                    stats.reads[T2C] += 1
                    if on_data:
                        on_data(t, T2C, data)
                    continue
                if d == C2P and mute_cam:
                    # Never reached the board: log it as c2m, so replay and
                    # diff do not take it for board traffic.
                    writer.data(t, C2M, data)
                    stats.bytes[C2M] += len(data)
                    stats.reads[C2M] += 1
                    if on_data:
                        on_data(t, C2M, data)
                    continue
                if d == C2P and not cam_synced:
                    gap = t - prev_c2p_read
                    prev_c2p_read = t
                    if gap >= FRAME_GAP_NS and data[0] in SYNC_BYTES:
                        cam_synced = True
                    else:
                        p = trailing_commands(data)
                        skipped = data if p is None else data[:p]
                        note = f"joined camera mid-frame, not forwarded: {skipped.hex(' ')}"
                        writer.mark(t, note)
                        if on_mark:
                            on_mark(t, note)
                        if p is None:
                            continue
                        data = data[p:]
                        cam_synced = True
                written = dst.write(data)
                if written is not None and written < len(data):
                    note = f"{d}: {len(data) - written} bytes dropped, nobody reading the pty"
                    writer.mark(t, note)
                    if on_mark:
                        on_mark(t, note)
                writer.data(t, d, data)
                stats.bytes[d] += len(data)
                stats.reads[d] += 1
                if on_data:
                    on_data(t, d, data)
                if d == C2P:
                    to_tee(t, cam_framer.feed(t, data))
                    last_c2p = t
                    send_probes(t)
            send_probes(time.monotonic_ns() - t0)
            if now - last_flush > 1e9:
                writer.flush()
                last_flush = now
        finish_frame()
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
