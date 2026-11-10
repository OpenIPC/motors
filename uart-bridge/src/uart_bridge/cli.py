"""uart-bridge command line."""

from __future__ import annotations

import argparse
import datetime as _dt
import gzip
import math
import os
import sys
from pathlib import Path

from . import analysis, codec
from .framing import Frame
from .log import C2M, C2P, C2T, H2P, MARK, P2C, T2C, LogWriter, read_log

DEFAULT_CAM = "/dev/ttyUSB0"
DEFAULT_PTZ = "/dev/ttyUSB1"
ARROW = {C2P: "cam->ptz", P2C: "ptz->cam", H2P: "host->ptz", C2M: "cam-x-ptz",
         C2T: "cam->tee", T2C: "tee->", analysis.TO_PTZ: "->ptz"}


def fmt_t(t: int) -> str:
    return f"{t / 1e9:10.3f}"


class Printer:
    """Prints frames as they arrive. With `collapse`, a frame is printed only
    when its key() differs from the previous frame in the same direction; the
    repeat count of that previous frame is shown on the same line."""

    def __init__(self, collapse: bool = True, out=None):
        self.collapse = collapse
        self.out = out or sys.stdout
        self.framers = analysis.framers()
        self.last_key: dict[str, str] = {}
        self.repeat: dict[str, int] = {}
        self.last_t = 0
        self.dir_t: dict[str, int] = {}

    def data(self, t: int, d: str, data: bytes) -> None:
        self.dir_t[d] = t
        for item in self.framers[d].feed(t, data):
            if isinstance(item, Frame):
                self.frame(item.t, d, item.data)
            else:
                self.line(item.t, d, f"junk {item.data.hex(' ')}")

    def frame(self, t: int, d: str, data: bytes) -> None:
        k = codec.key(data, strict_a5=True)  # show A5 clock/gain changes, not focus-value jitter
        if self.collapse and k == self.last_key.get(d):
            self.repeat[d] += 1
            return
        prev = f"  (prev x{self.repeat[d] + 1})" if self.collapse and d in self.last_key else ""
        self.last_key[d], self.repeat[d] = k, 0
        self.last_t = t
        self.line(t, d, f"{data.hex(' ')}  {codec.decode(data).text}{prev}")

    def flush(self) -> None:
        """Print what is still pending at the end: repeat counts of the last
        frame in each direction, and bytes of a frame that never completed."""
        for d, n in self.repeat.items():
            if self.collapse and n:
                self.line(self.last_t, d, f"(last frame x{n + 1})")
                self.repeat[d] = 0
        for d, f in self.framers.items():
            if f.pending():
                self.line(self.dir_t.get(d, 0), d, f"incomplete {f.pending().hex(' ')}")
                f.buf.clear()

    def mark(self, t: int, note: str) -> None:
        print(f"{fmt_t(t)}  ---- {note}", file=self.out)

    def line(self, t: int, d: str, text: str) -> None:
        print(f"{fmt_t(t)}  {ARROW[d]}  {text}", file=self.out, flush=True)


def default_log(mode: str) -> Path:
    stamp = _dt.datetime.now().strftime("%Y%m%d-%H%M%S-%f")
    return Path("captures") / f"{stamp}-{mode}.jsonl"


def open_log(path: Path | None, mode: str):
    """Create the capture file; never overwrite an existing capture."""
    path = path or default_log(mode)
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        # A .gz name gets a real gzip file, since read_log() decides by the suffix.
        if str(path).endswith(".gz"):
            return path, gzip.open(path, "xt")
        return path, open(path, "x")
    except FileExistsError:
        raise SystemExit(f"{path} already exists; not overwriting a capture")


def print_stats(stats, path: Path) -> None:
    print(f"\nlog: {path}", file=sys.stderr)
    for d in (C2P, P2C, H2P, C2M, C2T, T2C):
        if d in (H2P, C2M, C2T, T2C) and not stats.reads[d]:
            continue
        print(f"  {ARROW[d]}: {stats.bytes[d]} bytes in {stats.reads[d]} reads", file=sys.stderr)


def cmd_bridge(a: argparse.Namespace) -> int:
    from . import bridge

    latency = {}
    if a.latency:
        latency = {p: bridge.set_latency(p, a.latency) for p in (a.cam, a.ptz)
                   if p != "pty" and not bridge.is_url(p)}
    if a.cam == "pty":
        cam = bridge.PtyPort()
        a.cam = cam.path
    else:
        cam = bridge.open_port(a.cam, a.baud)
    ptz = bridge.open_port(a.ptz, a.ptz_baud or a.baud)
    tee = bridge.open_port(a.tee, a.ptz_baud or a.baud) if a.tee else None
    cam_drain, ptz_drain = bridge.drain_stale(cam), bridge.drain_stale(ptz)
    stale = {"cam": cam_drain.bytes, "ptz": ptz_drain.bytes}
    path, fp = open_log(a.log, "bridge")
    with fp:
        writer = LogWriter(fp, {
            "mode": "bridge", "cam": a.cam, "ptz": a.ptz,
            "baud": a.baud, "ptz_baud": a.ptz_baud or a.baud, "latency": latency,
            "stale_dropped": stale, "mute_cam": a.mute_cam, "tee": a.tee or "",
            "note": a.note or "",
        })
        stopper = bridge.Stopper()
        stopper.install()
        printer = None if a.quiet else Printer(collapse=not a.all)
        control = None if a.no_stdin else sys.stdin.fileno()
        if isinstance(cam, bridge.PtyPort):
            # Only now: anything the tool writes before this point would have
            # been discarded by drain_stale() as pre-bridge input.
            print(f"camera side is a pty: point the tool under test at {cam.path}",
                  file=sys.stderr, flush=True)
        print(f"bridging {a.cam} <-> {a.ptz}, logging to {path}"
              + ("; a line on stdin adds a mark, !<hex> sends a probe to the PTZ board"
                 if control is not None else ""), file=sys.stderr)
        stats = bridge.run(cam, ptz, writer, stopper,
                           on_data=printer.data if printer else None,
                           control=control, duration=a.duration,
                           on_mark=printer.mark if printer else None,
                           cam_last_ns=cam_drain.last_ns, mute_cam=a.mute_cam, tee=tee)
        if printer:
            printer.flush()
    cam.close()
    ptz.close()
    if tee is not None:
        tee.close()
    print_stats(stats, path)
    return 0


def cmd_inject(a: argparse.Namespace) -> int:
    from . import bridge, inject

    if a.replay:
        _, records = read_log(a.replay)
        schedule = inject.schedule_replay(records, a.speed)
        source = {"replay": str(a.replay), "speed": a.speed}
    else:
        frames = [bytes.fromhex(h) for h in a.frame]
        schedule = inject.schedule_frames(frames, a.rate, a.duration)
        source = {"frames": [f.hex() for f in frames], "rate": a.rate, "duration": a.duration}
    if not schedule:
        print("nothing to send", file=sys.stderr)
        return 1
    latency = bridge.set_latency(a.ptz, a.latency) if a.latency else ""
    ptz = bridge.open_port(a.ptz, a.ptz_baud)
    stale = bridge.drain_stale(ptz).bytes
    path, fp = open_log(a.log, "inject")
    with fp:
        writer = LogWriter(fp, {"mode": "inject", "ptz": a.ptz, "ptz_baud": a.ptz_baud,
                                "latency": latency, "stale_dropped": stale, **source})
        stopper = bridge.Stopper()
        stopper.install()
        printer = None if a.quiet else Printer(collapse=not a.all)
        print(f"injecting {len(schedule)} writes into {a.ptz}, logging to {path}", file=sys.stderr)
        stats = inject.run(ptz, schedule, writer, stopper,
                           on_data=printer.data if printer else None, tail=a.tail,
                           on_abort=a.on_abort or b"")
        if printer:
            printer.flush()
    ptz.close()
    print_stats(stats, path)
    return 0


def counts(s: analysis.Summary) -> str:
    """Frames and junk bytes per direction: c2p and p2c always, h2p and the
    muted c2m whenever they carry anything, so noise in them is not hidden."""
    dirs = [d for d in (C2P, P2C, H2P, C2M, C2T, T2C)
            if d in (C2P, P2C) or s.frames[d] or s.junk_bytes[d]]
    return ("frames " + " ".join(f"{d}={s.frames[d]}" for d in dirs)
            + "  junk bytes " + " ".join(f"{d}={s.junk_bytes[d]}" for d in dirs))


def cmd_decode(a: argparse.Namespace) -> int:
    header, records = read_log(a.file)
    print(f"# {header.get('mode', '?')} {header.get('wall', '')} git={header.get('git', '')} "
          f"{header.get('note', '')}".rstrip())
    printer = Printer(collapse=not a.all)
    for r in records:
        if r.d == MARK:
            printer.mark(r.t, r.note)
        elif a.dir in ("both", r.d):
            printer.data(r.t, r.d, r.data)
    printer.flush()
    s = analysis.summarize(analysis.events(records))
    lo, mean, hi = s.c2p_period_ms
    print(f"# {s.duration_s:.1f}s  {counts(s)}  "
          f"c2p rate={s.c2p_rate:.2f}/s period min/mean/max={lo:.1f}/{mean:.1f}/{hi:.1f} ms")
    return 0


def zoom_reports(records, d: str) -> list[tuple[int, float]]:
    """(t, zoom) for every zoom report in direction d."""
    out = []
    for ev in analysis.events([r for r in records if r.d == d]):
        if ev.kind == "frame":
            z = codec.decode(ev.data).fields.get("zoom")
            if z and z.startswith("X"):
                out.append((ev.t, float(z[1:])))
    return out


def settled(reports: list[tuple[int, float]], gap_s: float = 1.0) -> list[tuple[int, float]]:
    """Where each movement ended: the last report before a pause of gap_s."""
    out = []
    for i, (t, z) in enumerate(reports):
        if i + 1 == len(reports) or reports[i + 1][0] - t > gap_s * 1e9:
            out.append((t, z))
    return out


def cmd_boards(a: argparse.Namespace) -> int:
    """Compare two lens boards driven by the same traffic: the vendor board
    (p2c) against the tee'd board (t2c) of one bridge capture, or against the
    p2c of a second capture (e.g. an inject replay)."""
    _, ra = read_log(a.a)
    first = zoom_reports(ra, P2C)
    if a.b:
        _, rb = read_log(a.b)
        second, label = zoom_reports(rb, P2C), str(a.b)
    else:
        second, label = zoom_reports(ra, T2C), "tee (t2c)"
    sa, sb = settled(first), settled(second)
    print(f"A = {a.a} p2c: {len(first)} reports, {len(sa)} settled positions")
    print(f"B = {label}: {len(second)} reports, {len(sb)} settled positions")
    worst = 0.0
    for i in range(max(len(sa), len(sb))):
        x = sa[i] if i < len(sa) else None
        y = sb[i] if i < len(sb) else None
        diff = abs(x[1] - y[1]) if x and y else None
        if diff is not None:
            worst = max(worst, diff)
        print(f"  #{i + 1:<3} A {fmt_t(x[0]) + f'  X{x[1]:.1f}' if x else '       -':>18}   "
              f"B {fmt_t(y[0]) + f'  X{y[1]:.1f}' if y else '       -':>18}   "
              + ("missing" if diff is None else f"diff {diff:.1f}"))
    if not sa or not sb:
        print("MISMATCH: no zoom positions from " + ("either board" if not sa and not sb else
              "board A" if not sa else "board B") + "; nothing was compared")
        return 1
    mismatch = len(sa) != len(sb) or worst > a.tolerance + 1e-9
    print(f"{'MISMATCH' if mismatch else 'match'}: max difference {worst:.1f} (tolerance {a.tolerance})")
    return 1 if mismatch else 0


def cmd_diff(a: argparse.Namespace) -> int:
    _, ra = read_log(a.a)
    _, rb = read_log(a.b)
    ea, eb = analysis.events(ra), analysis.events(rb)
    for name, evs in (("A", ea), ("B", eb)):
        s = analysis.summarize(evs)
        print(f"{name}: {s.duration_s:.1f}s {counts(s)} rate={s.c2p_rate:.2f}/s")
    divs = analysis.diff(ea, eb, tolerance=a.tolerance, time_tolerance_ms=a.time_tolerance,
                         strict_a5=a.strict_a5, ignore_edges=a.ignore_edges)
    for dv in divs:
        print(f"\n{ARROW[dv.d]} {dv.op} {dv.detail}".rstrip())
        for side, segs in (("A", dv.a), ("B", dv.b)):
            for s in segs:
                print(f"  {side} {fmt_t(s.t)} x{s.count:<4} {s.key}")
    print(f"\n{len(divs)} divergence(s)")
    return 1 if divs else 0


def positive(text: str) -> float:
    v = float(text)
    if not (v > 0 and math.isfinite(v)):
        raise argparse.ArgumentTypeError(f"must be finite and > 0, got {text}")
    return v


def hex_bytes(text: str) -> bytes:
    try:
        return bytes.fromhex(text)
    except ValueError:
        raise argparse.ArgumentTypeError(f"not hex bytes: {text!r}")


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="uart-bridge", description=__doc__)
    sub = p.add_subparsers(dest="cmd", required=True)

    b = sub.add_parser("bridge", help="forward camera <-> PTZ board and log both directions")
    b.add_argument("--cam", default=DEFAULT_CAM,
                   help=f"camera-side port (default {DEFAULT_CAM}); 'pty' creates a pseudo terminal "
                        "so a local program under test plays the camera")
    b.add_argument("--ptz", default=DEFAULT_PTZ, help=f"PTZ-board-side port (default {DEFAULT_PTZ})")
    b.add_argument("--baud", type=int, default=115200)
    b.add_argument("--ptz-baud", type=int, help="PTZ side baud if it differs from --baud")
    b.add_argument("--log", type=Path, help="capture file (default captures/<time>-bridge.jsonl)")
    b.add_argument("--duration", type=positive, help="stop after N seconds")
    b.add_argument("--note", help="free text stored in the capture header")
    b.add_argument("--latency", type=int, default=1,
                   help="FTDI latency timer in ms, set via sysfs (0 = leave as is; default 1)")
    b.add_argument("--quiet", action="store_true", help="no live output")
    b.add_argument("--all", action="store_true", help="print every frame, not only changes")
    b.add_argument("--tee", metavar="URL",
                   help="also send every whole camera frame to this port or URL, e.g. "
                        "socket://host:9000 (xm-uart -l on another camera); its replies are logged as t2c")
    b.add_argument("--mute-cam", action="store_true",
                   help="log camera bytes but do not forward them to the PTZ board")
    b.add_argument("--no-stdin", action="store_true",
                   help="ignore stdin (no marks, no probes)")
    b.set_defaults(func=cmd_bridge)

    i = sub.add_parser("inject", help="act as the camera: send frames to the PTZ board")
    i.add_argument("--ptz", default=DEFAULT_PTZ)
    i.add_argument("--ptz-baud", type=int, default=115200)
    src = i.add_mutually_exclusive_group(required=True)
    src.add_argument("--replay", type=Path, help="replay the cam->ptz stream of a capture")
    src.add_argument("--frame", action="append", help="hex frame to send; repeat to cycle several")
    i.add_argument("--speed", type=positive, default=1.0, help="replay speed factor")
    i.add_argument("--rate", type=positive, default=20.0, help="frames/s for --frame")
    i.add_argument("--duration", type=positive, default=1.0, help="seconds to send --frame for")
    i.add_argument("--tail", type=float, default=0.5, help="seconds to keep listening afterwards")
    i.add_argument("--on-abort", metavar="HEX", type=hex_bytes,
                   help="bytes to send if interrupted before the schedule is done, "
                        "e.g. the XM stop frame c50100000000015c")
    i.add_argument("--log", type=Path)
    i.add_argument("--latency", type=int, default=1)
    i.add_argument("--quiet", action="store_true")
    i.add_argument("--all", action="store_true")
    i.set_defaults(func=cmd_inject)

    d = sub.add_parser("decode", help="print an annotated timeline of a capture")
    d.add_argument("file", type=Path)
    d.add_argument("--all", action="store_true", help="print every frame, not only changes")
    d.add_argument("--dir", choices=("both", C2P, P2C, H2P, C2M, C2T, T2C), default="both")
    d.set_defaults(func=cmd_decode)

    bo = sub.add_parser("boards", help="compare the zoom positions two lens boards settled at")
    bo.add_argument("a", type=Path, help="capture (its p2c is board A; its t2c is B unless B is given)")
    bo.add_argument("b", type=Path, nargs="?", help="second capture whose p2c is board B")
    bo.add_argument("--tolerance", type=float, default=0.1, help="allowed zoom difference (default 0.1)")
    bo.set_defaults(func=cmd_boards)

    f = sub.add_parser("diff", help="compare two captures; exit 1 on divergence")
    f.add_argument("a", type=Path)
    f.add_argument("b", type=Path)
    f.add_argument("--tolerance", type=int, default=3,
                   help="allowed repeat-count difference per segment, in frames")
    f.add_argument("--time-tolerance", type=positive, default=150.0,
                   help="allowed difference in segment span and spacing, ms (default 150)")
    f.add_argument("--strict-a5", action="store_true",
                   help="compare A5 frames' seconds, day/night and gain, not just their presence and cadence")
    f.add_argument("--ignore-edges", action="store_true",
                   help="drop unmatched runs at the capture ends (for --strict-a5)")
    f.set_defaults(func=cmd_diff)
    return p


def main(argv: list[str] | None = None) -> int:
    a = build_parser().parse_args(argv)
    try:
        rc = a.func(a)
        sys.stdout.flush()  # a closed pipe can also surface here, at the final flush
        return rc
    except BrokenPipeError:
        # Reader went away (`| head`); keep the interpreter from complaining
        # again while it flushes stdout at exit.
        os.dup2(os.open(os.devnull, os.O_WRONLY), sys.stdout.fileno())
        return 0


if __name__ == "__main__":
    sys.exit(main())
