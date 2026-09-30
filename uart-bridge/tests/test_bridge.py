import io
import os
import threading
import time

from uart_bridge import analysis, bridge, inject
from uart_bridge.framing import Frame, Framer, Junk
from uart_bridge.log import C2P, P2C, LogWriter, read_log

IDLE = [bytes.fromhex(h) for h in (
    "a5319ef75cb15996", "a5319ef75cb159ad", "a5309ef45eb45d58", "a5339ef1246be1ae",
)]


def test_framer_resyncs_and_splits_across_reads():
    f = Framer()
    out = f.feed(1, b"\x00\x11" + IDLE[0][:5])
    assert out == [Junk(1, b"\x00\x11")]
    out = f.feed(2, IDLE[0][5:] + IDLE[1])
    assert out == [Frame(2, IDLE[0]), Frame(2, IDLE[1])]
    assert f.pending() == b""


def test_framer_keeps_stride_when_sync_byte_is_inside_frame():
    frame = bytes.fromhex("a5019e07a501d9a5")
    f = Framer()
    assert f.feed(0, frame + IDLE[0]) == [Frame(0, frame), Frame(0, IDLE[0])]


def test_log_round_trip(tmp_path):
    p = tmp_path / "c.jsonl"
    with open(p, "w") as fp:
        w = LogWriter(fp, {"mode": "test"})
        w.data(10, C2P, IDLE[0])
        w.mark(20, "pan left")
        w.data(30, P2C, b"\x01\x02")
    header, recs = read_log(p)
    assert header["mode"] == "test" and header["version"] == 1
    assert [(r.t, r.d, r.data, r.note) for r in recs] == [
        (10, C2P, IDLE[0], ""), (20, "mark", b"", "pan left"), (30, P2C, b"\x01\x02", "")]


def pty_port():
    master, slave = os.openpty()
    return master, os.ttyname(slave), slave


def read_exact(fd, n, timeout=2.0):
    buf = b""
    end = time.monotonic() + timeout
    while len(buf) < n and time.monotonic() < end:
        try:
            buf += os.read(fd, n - len(buf))
        except BlockingIOError:
            time.sleep(0.01)
    return buf


def test_bridge_forwards_both_ways_and_logs(tmp_path):
    cam_m, cam_path, cam_s = pty_port()
    ptz_m, ptz_path, ptz_s = pty_port()
    cam = bridge.open_port(cam_path, 115200)
    ptz = bridge.open_port(ptz_path, 115200)
    log = io.StringIO()
    stopper = bridge.Stopper()
    th = threading.Thread(target=bridge.run, args=(cam, ptz, LogWriter(log, {}), stopper))
    th.start()
    try:
        payload = b"".join(IDLE)
        os.write(cam_m, payload)
        assert read_exact(ptz_m, len(payload)) == payload
        os.write(ptz_m, b"\x5a\x01\x02")
        assert read_exact(cam_m, 3) == b"\x5a\x01\x02"
    finally:
        stopper.stop = True
        th.join(2)
    p = tmp_path / "b.jsonl"
    p.write_text(log.getvalue())
    _, recs = read_log(p)
    assert b"".join(r.data for r in recs if r.d == C2P) == payload
    assert b"".join(r.data for r in recs if r.d == P2C) == b"\x5a\x01\x02"
    for fd in (cam_m, cam_s, ptz_m, ptz_s):
        os.close(fd)


def test_inject_sends_schedule_and_logs_replies():
    ptz_m, ptz_path, ptz_s = pty_port()
    ptz = bridge.open_port(ptz_path, 115200)
    log = io.StringIO()
    sched = inject.schedule_frames(IDLE[:2], rate=50, duration=0.1)
    assert [o for o, _ in sched] == [0, 20_000_000, 40_000_000, 60_000_000, 80_000_000]
    th = threading.Thread(target=inject.run,
                          args=(ptz, sched, LogWriter(log, {}), bridge.Stopper()),
                          kwargs={"tail": 0.3})
    th.start()
    got = read_exact(ptz_m, 8 * len(sched))
    os.write(ptz_m, b"\x77")
    th.join(2)
    assert got == b"".join(f for _, f in sched)
    assert '"d":"p2c","x":"77"' in log.getvalue()
    os.close(ptz_m)
    os.close(ptz_s)


def events_from(frames, t0=0, step=50_000_000):
    return [analysis.Event(t0 + i * step, C2P, "frame", f) for i, f in enumerate(frames)]


def idle_stream(bodies, per=20):
    out = []
    for n, body in enumerate(bodies):
        for i in range(per):
            out.append(bytes.fromhex(body) + bytes([(n * per + i) & 0xFF]))
    return out


BODIES = ["a5319ef75cb159", "a5309ef45eb45d", "a5339ef1246be1", "a5329ef65eb65d",
          "a53d9efb247d99", "a53c9ef82670ed"]


def test_diff_identical_and_phase_shifted_idle_captures_match():
    a = events_from(idle_stream(BODIES))
    assert analysis.diff(a, a) == []
    # B starts 7 frames later and one body earlier-ending: only edges differ.
    b = events_from(idle_stream(BODIES[:-1])[7:])
    assert analysis.diff(a, b) == []


def test_diff_reports_inserted_command_and_timing():
    a = events_from(idle_stream(BODIES))
    moved = BODIES[:3] + ["a5339e00000000"] + BODIES[3:]
    b = events_from(idle_stream(moved))
    divs = analysis.diff(a, b)
    assert [(d.op, [s.key for s in d.b]) for d in divs] == [("insert", ["a5 33 9e 00 00 00 00"])]
    slow = idle_stream(BODIES[:2]) + idle_stream(BODIES[2:3], per=30) + idle_stream(BODIES[3:])
    divs = analysis.diff(a, events_from(slow))
    assert [(d.op, d.a[0].count, d.b[0].count) for d in divs] == [("timing", 20, 30)]


def test_diff_totally_different_captures_diverge():
    a = events_from(idle_stream(BODIES[:2]))
    b = events_from(idle_stream(["a5009e00000000"]))
    assert analysis.diff(a, b)


def test_counter_unscramble_is_monotonic_on_captured_sequence():
    from uart_bridge import codec
    seq = "2e 29 28 2b 2a 35 34 37 36 31 30 33 32 3d 3c 3f 3e 39 38 3b 3a 05 04 07 06 01 00 03 02 0d 0c 0f 0e 09 08 0b 0a 15 14 17 16 11 10 13 12 1d 1c 1f 1e 19 18 1b 1a 65 64 67 66 61 60 63 62"
    ns = [codec.counter(bytes([0xA5, int(b, 16)])) for b in seq.split()]
    assert ns == list(range(0x0B, 0x0B + len(ns)))
