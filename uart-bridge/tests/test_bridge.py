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
    """Read up to n bytes, giving up after `timeout` s (returns what arrived)."""
    os.set_blocking(fd, False)
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
        time.sleep(0.02)  # the bridge joins the camera stream after a quiet gap
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


def with_command(frames, at, cmd):
    return frames[:at] + [cmd] + frames[at:]


def test_diff_idle_captures_match_whatever_the_counter_phase():
    a = events_from(idle_stream(BODIES))
    assert analysis.diff(a, a) == []
    b = events_from(idle_stream(BODIES[:-1])[7:])  # other start, other end
    assert analysis.diff(a, b) == []


def test_diff_strict_a5_sees_counter_edges_unless_ignored():
    a = events_from(idle_stream(BODIES))
    b = events_from(idle_stream(BODIES[:-1])[7:])
    assert analysis.diff(a, b, strict_a5=True)
    assert analysis.diff(a, b, strict_a5=True, ignore_edges=True) == []


def test_diff_reports_inserted_command_mid_stream_and_at_edges():
    idle = idle_stream(BODIES)
    a = events_from(idle)
    for at in (60, 0, len(idle)):
        divs = analysis.diff(a, events_from(with_command(idle, at, ZOOM_IN)))
        inserted = [s.key for d in divs if d.op == "insert" for s in d.b]
        assert ZOOM_IN.hex(" ") in inserted and not any(d.a for d in divs), at


def test_diff_reports_run_length_change():
    stop = bytes.fromhex("c50100000000015c")
    a = events_from(idle_stream(BODIES[:2]) + [ZOOM_IN] + idle_stream(BODIES[2:3]) + [stop]
                    + idle_stream(BODIES[3:]))
    b = events_from(idle_stream(BODIES[:2]) + [ZOOM_IN] + idle_stream(BODIES[2:3], per=30) + [stop]
                    + idle_stream(BODIES[3:]))
    ops = [(d.op, d.detail) for d in analysis.diff(a, b)]
    assert ("count", "x20 vs x30") in ops


def test_diff_reports_same_frames_sent_slower():
    stop = bytes.fromhex("c50100000000015c")
    frames = idle_stream(BODIES[:2]) + [ZOOM_IN] + idle_stream(BODIES[2:3]) + [stop] \
        + idle_stream(BODIES[3:])
    a = events_from(frames)
    b = events_from(frames, step=60_000_000)  # 20 % slower cadence, same keys and counts
    assert [(d.op, d.detail) for d in analysis.diff(a, b)] == [("span", "950 ms vs 1140 ms")]
    # The stop command 300 ms late, everything else identical.
    late = events_from(frames)
    i = frames.index(stop)
    for ev in late[i:]:
        ev.t += 300_000_000
    assert [d.op for d in analysis.diff(a, late)] == ["gap"]


def test_diff_totally_different_captures_diverge():
    a = events_from(idle_stream(BODIES[:2]))
    b = events_from([ZOOM_IN] * 5)
    assert analysis.diff(a, b)


def test_unknown_reply_bytes_do_not_depend_on_read_split():
    from uart_bridge.log import Record
    unknown = bytes.fromhex("7a0102030405")
    one = [Record(0, P2C, unknown + REPLY)]
    split = [Record(0, P2C, unknown[:2]), Record(1, P2C, unknown[2:4]), Record(2, P2C, unknown[4:] + REPLY)]
    assert analysis.diff(analysis.events(one), analysis.events(split)) == []


def test_trailing_partial_frame_keeps_its_own_direction_time():
    from uart_bridge.log import Record
    evs = analysis.events([Record(5, C2P, IDLE[0][:3]), Record(900, P2C, REPLY)])
    junk = [e for e in evs if e.kind == "junk"]
    assert [(e.d, e.t) for e in junk] == [(C2P, 5)]


def test_printer_flush_reports_final_repeat_count():
    from uart_bridge.cli import Printer
    out = io.StringIO()
    p = Printer(out=out)
    for i in range(3):
        p.data(i, C2P, IDLE[0])
    p.flush()
    assert "(last frame x3)" in out.getvalue()


def test_invalid_replay_speed_and_rate_rejected():
    import pytest
    from uart_bridge.cli import build_parser
    for argv in (["inject", "--replay", "x", "--speed", "0"], ["inject", "--frame", "a5", "--rate", "-1"]):
        with pytest.raises(SystemExit):
            build_parser().parse_args(argv)
    with pytest.raises(ValueError):
        inject.schedule_replay([], speed=0)


def test_counter_unscramble_is_monotonic_on_captured_sequence():
    from uart_bridge import codec
    seq = "2e 29 28 2b 2a 35 34 37 36 31 30 33 32 3d 3c 3f 3e 39 38 3b 3a 05 04 07 06 01 00 03 02 0d 0c 0f 0e 09 08 0b 0a 15 14 17 16 11 10 13 12 1d 1c 1f 1e 19 18 1b 1a 65 64 67 66 61 60 63 62"
    ns = [codec.counter(bytes([0xA5, int(b, 16)])) for b in seq.split()]
    assert ns == list(range(0x0B, 0x0B + len(ns)))


REPLY = bytes.fromhex("ef01000904032f2e58312e3320")  # captured: "X1.3 "
ZOOM_IN = bytes.fromhex("c50100200000215c")


def test_ptz_reply_framed_across_byte_sized_reads():
    from uart_bridge.framing import PTZ_RULES
    f = Framer(PTZ_RULES)
    out = []
    for i, b in enumerate(REPLY + REPLY[:3]):
        out += f.feed(i, bytes([b]))
    assert out == [Frame(len(REPLY) - 1, REPLY)]
    assert f.pending() == REPLY[:3]


def test_cam_framer_takes_pelco_and_a5_and_rejects_bad_end_byte():
    f = Framer()
    bad = bytes.fromhex("c5010020000021ff")
    assert f.feed(0, ZOOM_IN + IDLE[0] + bad + IDLE[1]) == [
        Frame(0, ZOOM_IN), Frame(0, IDLE[0]), Junk(0, bad), Frame(0, IDLE[1])]


def test_codec_decodes_pelco_and_replies():
    from uart_bridge import codec
    d = codec.decode(ZOOM_IN)
    assert d.ok and d.fields["actions"] == ["zoom+"] and d.text == "pelco addr=1 zoom+"
    assert codec.decode(bytes.fromhex("c50100000000015c")).text == "pelco addr=1 stop"
    assert "ck=01!=21" in codec.decode(bytes.fromhex("c50100200000015c")).text
    # Captured from the stock firmware (e1-stock): sum % 256, iris bits, presets.
    assert codec.decode(bytes.fromhex("c50100800000815c")).text == "pelco addr=1 focus-near"
    assert codec.decode(bytes.fromhex("c50104000000055c")).text == "pelco addr=1 iris-close"
    assert codec.decode(bytes.fromhex("c50100030005095c")).text == "pelco addr=1 set-preset 00 05"
    assert codec.decode(bytes.fromhex("c50100070005 0d5c".replace(" ", ""))).fields["extended"] == "goto-preset"
    assert codec.decode(bytes.fromhex("ef01000a04032f2e202020202020")).text == "reply type=00 zoom report blank"
    r = codec.decode(REPLY)
    assert r.fields["zoom"] == "X1.3" and r.fields["type"] == 0
    assert codec.decode(bytes.fromhex("ef01020101")).text == "reply night"


def test_bridge_probe_waits_for_camera_frame_boundary():
    cam_m, cam_path, cam_s = pty_port()
    ptz_m, ptz_path, ptz_s = pty_port()
    ctl_r, ctl_w = os.pipe()
    cam = bridge.open_port(cam_path, 115200)
    ptz = bridge.open_port(ptz_path, 115200)
    log = io.StringIO()
    stopper = bridge.Stopper()
    th = threading.Thread(target=bridge.run, args=(cam, ptz, LogWriter(log, {}), stopper),
                          kwargs={"control": ctl_r})
    th.start()
    try:
        time.sleep(0.02)
        os.write(cam_m, IDLE[0][:3])          # camera frame in flight
        assert read_exact(ptz_m, 3) == IDLE[0][:3]
        os.write(ctl_w, b"pan test\n!" + ZOOM_IN.hex().encode() + b"\n")
        time.sleep(0.08)                      # rest of the frame lags well past 20 ms
        os.write(cam_m, IDLE[0][3:])          # camera finishes its frame
        got = read_exact(ptz_m, 5 + len(ZOOM_IN))
        assert got == IDLE[0][3:] + ZOOM_IN   # probe after, not inside, the frame
    finally:
        stopper.stop = True
        th.join(2)
    assert '"d":"mark","note":"pan test"' in log.getvalue()
    assert f'"d":"h2p","x":"{ZOOM_IN.hex()}"' in log.getvalue()
    for fd in (cam_m, cam_s, ptz_m, ptz_s, ctl_r, ctl_w):
        os.close(fd)


def test_replay_includes_bridge_probes_in_capture_order():
    from uart_bridge.log import H2P, Record
    recs = [Record(0, C2P, IDLE[0]), Record(10, H2P, ZOOM_IN), Record(20, P2C, REPLY),
            Record(50, C2P, IDLE[1])]
    assert inject.schedule_replay(recs) == [(0, IDLE[0]), (10, ZOOM_IN), (50, IDLE[1])]


def test_diff_treats_probe_and_camera_command_to_ptz_alike():
    from uart_bridge.log import H2P
    idle = idle_stream(BODIES)
    a = events_from(with_command(idle, 60, ZOOM_IN))
    b = events_from(with_command(idle, 60, ZOOM_IN))
    b[60].d = H2P                          # same bytes, sent by the host as a probe
    assert analysis.diff(a, b) == []


def test_capture_never_overwritten(tmp_path):
    import pytest
    from uart_bridge.cli import open_log
    p = tmp_path / "c.jsonl"
    open_log(p, "bridge")[1].close()
    with pytest.raises(SystemExit):
        open_log(p, "bridge")


def test_decode_shows_trailing_incomplete_frame(tmp_path, capsys):
    from uart_bridge.cli import main
    p = tmp_path / "c.jsonl"
    with open(p, "w") as fp:
        w = LogWriter(fp, {"mode": "test"})
        w.data(0, C2P, IDLE[0] + IDLE[1][:3])
    main(["decode", str(p)])
    assert "incomplete a5 31 9e" in capsys.readouterr().out


def test_nonfinite_rates_rejected():
    import pytest
    from uart_bridge.cli import build_parser
    for argv in (["inject", "--frame", "a5", "--rate", "inf"], ["inject", "--frame", "a5", "--duration", "nan"]):
        with pytest.raises(SystemExit):
            build_parser().parse_args(argv)
    with pytest.raises(ValueError):
        inject.schedule_frames([IDLE[0]], rate=float("inf"), duration=1)


def test_stale_input_is_drained_not_forwarded():
    cam_m, cam_path, cam_s = pty_port()
    ptz_m, ptz_path, ptz_s = pty_port()
    cam = bridge.open_port(cam_path, 115200)
    ptz = bridge.open_port(ptz_path, 115200)
    os.write(cam_m, b"\xa4\xda\xc2\xc2" + IDLE[0] * 50)   # backlog from before the bridge ran
    time.sleep(0.02)
    assert bridge.drain_stale(cam).bytes == 4 + 8 * 50
    stopper = bridge.Stopper()
    th = threading.Thread(target=bridge.run, args=(cam, ptz, LogWriter(io.StringIO(), {}), stopper))
    th.start()
    try:
        time.sleep(0.02)
        os.write(cam_m, IDLE[1])
        assert read_exact(ptz_m, 64, timeout=0.5) == IDLE[1]   # only live traffic reaches the board
    finally:
        stopper.stop = True
        th.join(2)
    for fd in (cam_m, cam_s, ptz_m, ptz_s):
        os.close(fd)


def run_bridge_thread(cam, ptz, log=None, **kw):
    stopper = bridge.Stopper()
    th = threading.Thread(target=bridge.run, args=(cam, ptz, LogWriter(log or io.StringIO(), {}), stopper),
                          kwargs=kw)
    th.start()
    return stopper, th


def test_pty_camera_side_reaches_ptz_board():
    ptz_m, ptz_path, ptz_s = pty_port()
    cam = bridge.PtyPort()
    ptz = bridge.open_port(ptz_path, 115200)
    stopper, th = run_bridge_thread(cam, ptz)
    try:
        time.sleep(0.02)
        tool = os.open(cam.path, os.O_RDWR | os.O_NOCTTY)   # the program under test
        os.write(tool, ZOOM_IN)
        assert read_exact(ptz_m, 8) == ZOOM_IN
        os.write(ptz_m, REPLY)
        os.set_blocking(tool, False)
        assert read_exact(tool, len(REPLY)) == REPLY
        os.close(tool)
    finally:
        stopper.stop = True
        th.join(2)
    cam.close()
    for fd in (ptz_m, ptz_s):
        os.close(fd)


def test_bridge_joins_camera_stream_only_at_a_frame_start():
    cam_m, cam_path, cam_s = pty_port()
    ptz_m, ptz_path, ptz_s = pty_port()
    cam = bridge.open_port(cam_path, 115200)
    ptz = bridge.open_port(ptz_path, 115200)
    log = io.StringIO()
    stopper, th = run_bridge_thread(cam, ptz, log)
    try:
        time.sleep(0.02)
        os.write(cam_m, IDLE[0][3:])           # tail of a frame already in flight (contains no gap)
        time.sleep(0.001)
        os.write(cam_m, IDLE[0][5:])           # still mid-stream, no quiet gap before it
        time.sleep(0.03)                        # the inter-frame gap
        os.write(cam_m, IDLE[1])
        assert read_exact(ptz_m, 32, timeout=0.5) == IDLE[1]
    finally:
        stopper.stop = True
        th.join(2)
    assert "joined camera mid-frame" in log.getvalue()
    for fd in (cam_m, cam_s, ptz_m, ptz_s):
        os.close(fd)


def test_bridge_completes_frame_in_flight_on_exit():
    cam_m, cam_path, cam_s = pty_port()
    ptz_m, ptz_path, ptz_s = pty_port()
    cam = bridge.open_port(cam_path, 115200)
    ptz = bridge.open_port(ptz_path, 115200)
    stopper, th = run_bridge_thread(cam, ptz)
    try:
        time.sleep(0.02)
        os.write(cam_m, IDLE[0][:3])
        assert read_exact(ptz_m, 3) == IDLE[0][:3]
        stopper.stop = True                      # stop while the frame is half forwarded
        time.sleep(0.08)                         # loop has left select, finish window is open
        os.write(cam_m, IDLE[0][3:] + IDLE[1])   # rest of the frame, then a whole new one
        th.join(2)
        assert read_exact(ptz_m, 16, timeout=0.3) == IDLE[0][3:]   # completed, nothing more
    finally:
        stopper.stop = True
        th.join(2)
    for fd in (cam_m, cam_s, ptz_m, ptz_s):
        os.close(fd)


def test_quiet_camera_line_first_frame_is_forwarded_immediately():
    cam = bridge.PtyPort()
    ptz_m, ptz_path, ptz_s = pty_port()
    ptz = bridge.open_port(ptz_path, 115200)
    drained = bridge.drain_stale(cam)
    assert drained.last_ns is None
    stopper, th = run_bridge_thread(cam, ptz, cam_last_ns=drained.last_ns)
    try:
        tool = os.open(cam.path, os.O_RDWR | os.O_NOCTTY)
        os.write(tool, IDLE[0])                 # right away, like xm-uart's init[]
        assert read_exact(ptz_m, 8) == IDLE[0]
        os.close(tool)
    finally:
        stopper.stop = True
        th.join(2)
    cam.close()
    for fd in (ptz_m, ptz_s):
        os.close(fd)


def test_joining_mid_frame_keeps_a_command_in_the_same_read():
    assert bridge.trailing_commands(IDLE[0][3:] + ZOOM_IN) == 5
    assert bridge.trailing_commands(IDLE[0][3:] + IDLE[1]) is None          # A5 alone: not trusted
    assert bridge.trailing_commands(IDLE[0][3:] + ZOOM_IN + IDLE[1]) == 5
    assert bridge.trailing_commands(IDLE[0][3:]) is None
    cam_m, cam_path, cam_s = pty_port()
    ptz_m, ptz_path, ptz_s = pty_port()
    cam = bridge.open_port(cam_path, 115200)
    ptz = bridge.open_port(ptz_path, 115200)
    stopper, th = run_bridge_thread(cam, ptz, cam_last_ns=time.monotonic_ns())  # camera was busy
    try:
        os.write(cam_m, IDLE[0][3:] + ZOOM_IN)   # tail + a command, one read, no gap
        assert read_exact(ptz_m, 16, timeout=0.5) == ZOOM_IN
    finally:
        stopper.stop = True
        th.join(2)
    for fd in (cam_m, cam_s, ptz_m, ptz_s):
        os.close(fd)


def test_pty_write_gives_up_when_nobody_reads():
    cam = bridge.PtyPort()
    t = time.monotonic()
    n = cam.write(b"\xef" * 1_000_000)
    assert n < 1_000_000 and time.monotonic() - t < 0.5
    cam.close()


def test_inject_sends_on_abort_when_stopped_early():
    ptz_m, ptz_path, ptz_s = pty_port()
    ptz = bridge.open_port(ptz_path, 115200)
    stop = bytes.fromhex("c50100000000015c")
    stopper = bridge.Stopper()
    sched = [(0, ZOOM_IN), (5_000_000_000, stop)]      # stop due in 5 s
    th = threading.Thread(target=inject.run, args=(ptz, sched, LogWriter(io.StringIO(), {}), stopper),
                          kwargs={"on_abort": stop})
    th.start()
    assert read_exact(ptz_m, 8) == ZOOM_IN
    stopper.stop = True                                # interrupted before the stop was due
    th.join(2)
    assert read_exact(ptz_m, 8, timeout=0.5) == stop
    os.close(ptz_m)
    os.close(ptz_s)


def test_on_abort_completes_a_partial_frame_first():
    ptz_m, ptz_path, ptz_s = pty_port()
    ptz = bridge.open_port(ptz_path, 115200)
    stop = bytes.fromhex("c50100000000015c")
    stopper = bridge.Stopper()
    sched = [(0, ZOOM_IN + b"\xa5"), (5_000_000_000, stop)]   # a lone A5 left on the wire
    th = threading.Thread(target=inject.run, args=(ptz, sched, LogWriter(io.StringIO(), {}), stopper),
                          kwargs={"on_abort": stop})
    th.start()
    assert read_exact(ptz_m, 9) == ZOOM_IN + b"\xa5"
    stopper.stop = True
    th.join(2)
    assert read_exact(ptz_m, 15, timeout=0.5) == bytes(7) + stop   # frame completed, then stop
    os.close(ptz_m)
    os.close(ptz_s)


def test_on_abort_hex_is_validated_by_the_parser():
    import pytest
    from uart_bridge.cli import build_parser
    with pytest.raises(SystemExit):
        build_parser().parse_args(["inject", "--frame", "c5", "--on-abort", "zz"])
    assert build_parser().parse_args(["inject", "--frame", "c5", "--on-abort", "c501"]).on_abort == b"\xc5\x01"


class FailingPort:
    """A port whose writes fail after the first one, each with a distinct
    exception instance (kept in .errors); readable fd from a pipe."""

    def __init__(self, ok_writes=1, on_fail=None):
        self.r, self.w = os.pipe()
        self.ok_writes = ok_writes
        self.errors = []
        self.on_fail = on_fail  # called at the first failure, e.g. to break the log too

    def fileno(self):
        return self.r

    def read(self, n):
        return b""

    def write(self, data):
        if self.ok_writes:
            self.ok_writes -= 1
            return len(data)
        e = OSError(5, f"Input/output error #{len(self.errors) + 1}")
        self.errors.append(e)
        if self.on_fail:
            self.on_fail()
        raise e

    def close(self):
        os.close(self.r)
        os.close(self.w)


class FailingLog(io.StringIO):
    """A capture file that stops accepting writes once .broken is set."""

    broken = False

    def write(self, s):
        if self.broken:
            raise OSError(28, "No space left on device")
        return super().write(s)


STOP_FRAME = bytes.fromhex("c50100000000015c")
FAILING_SCHEDULE = [(0, ZOOM_IN), (1_000_000, ZOOM_IN), (5_000_000_000, STOP_FRAME)]


def test_failed_port_propagates_the_run_error_not_the_abort_error():
    import pytest
    port = FailingPort()
    log = io.StringIO()
    with pytest.raises(OSError) as err:
        inject.run(port, FAILING_SCHEDULE, LogWriter(log, {}), bridge.Stopper(), on_abort=STOP_FRAME)
    assert len(port.errors) == 2                      # the scheduled write, then the abort write
    assert err.value is port.errors[0]                # the run's error, not the abort's
    assert "on-abort write failed" in log.getvalue()  # cleanup ran and was logged
    port.close()


def test_broken_log_during_cleanup_does_not_mask_the_run_error():
    import pytest
    log = FailingLog()
    writer = LogWriter(log, {})

    def break_log():
        log.broken = True                             # the capture file fails too, from here on

    port = FailingPort(on_fail=break_log)
    with pytest.raises(OSError) as err:
        inject.run(port, FAILING_SCHEDULE, writer, bridge.Stopper(), on_abort=STOP_FRAME)
    assert err.value is port.errors[0]
    port.close()


def test_failed_abort_after_clean_stop_is_raised():
    import pytest
    port = FailingPort(ok_writes=0)                   # every write fails
    log = FailingLog()
    writer = LogWriter(log, {})
    log.broken = True                                 # and so does the log
    stopper = bridge.Stopper()
    stopper.stop = True                               # stopped by a signal before any write
    with pytest.raises(OSError) as err:
        inject.run(port, [(5_000_000_000, ZOOM_IN)], writer, stopper, on_abort=STOP_FRAME)
    assert err.value is port.errors[0]                # the abort write's error, not the log's
    port.close()


def test_mute_cam_logs_but_does_not_forward_camera_bytes():
    cam_m, cam_path, cam_s = pty_port()
    ptz_m, ptz_path, ptz_s = pty_port()
    cam = bridge.open_port(cam_path, 115200)
    ptz = bridge.open_port(ptz_path, 115200)
    log = io.StringIO()
    stopper, th = run_bridge_thread(cam, ptz, log, mute_cam=True)
    try:
        time.sleep(0.02)
        os.write(cam_m, ZOOM_IN)
        assert read_exact(ptz_m, 8, timeout=0.3) == b""           # board hears nothing
        os.write(ptz_m, REPLY)
        assert read_exact(cam_m, len(REPLY)) == REPLY              # board -> camera still flows
    finally:
        stopper.stop = True
        th.join(2)
    assert f'"d":"c2p","x":"{ZOOM_IN.hex()}"' in log.getvalue()   # but it is logged
    for fd in (cam_m, cam_s, ptz_m, ptz_s):
        os.close(fd)


def test_read_log_reads_gzipped_captures(tmp_path):
    import gzip
    p = tmp_path / "c.jsonl.gz"
    with gzip.open(p, "wt") as fp:
        w = LogWriter(fp, {"mode": "test"})
        w.data(10, C2P, IDLE[0])
    header, recs = read_log(p)
    assert header["mode"] == "test" and recs[0].data == IDLE[0]


def test_cli_gz_capture_round_trips(tmp_path):
    import gzip
    from uart_bridge.cli import open_log
    p = tmp_path / "cap.jsonl.gz"
    path, fp = open_log(p, "bridge")
    with fp:
        LogWriter(fp, {"mode": "bridge"}).data(5, C2P, ZOOM_IN)
    with gzip.open(p, "rb") as g:                      # really compressed
        assert g.read(1) == b"{"
    header, recs = read_log(p)
    assert header["mode"] == "bridge" and recs[0].data == ZOOM_IN
