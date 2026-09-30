#!/usr/bin/env python3
"""Reproduce the experiments behind xm-uart/PROTOCOL.md.

Run on the lab host that has the camera board on one USB-UART and the lens
board on the other, from the uart-bridge directory:

    uv run scripts/xm_uart_audit.py stock   --camera 10.0.0.5 --python-dvr ~/git/python-dvr
    uv run scripts/xm_uart_audit.py accept
    uv run scripts/xm_uart_audit.py tool    ../xm-uart/xm-uart-motors-host
    uv run scripts/xm_uart_audit.py focus   --camera 10.0.0.5
    uv run scripts/xm_uart_audit.py restore --zoom 1.2
    uv run scripts/xm_uart_audit.py refocus --camera 10.0.0.5

stock   bridges camera<->board and drives every DVRIP PTZ command (E1)
accept  host as camera: which frame variants the board acts on (E3)
tool    a program under test through `bridge --cam pty`, scripted keys (E2)
focus   RTSP sharpness (ffmpeg blurdetect) for regions of the current image
restore zoom out/in with the board's own reports until it reads --zoom
refocus hill-climb focus on RTSP sharpness (experiments leave focus drifted)
focusdir which focus bit moves focus nearer: sweeps with near and far targets

Everything moves the lens; `accept` and `tool` return the zoom to where it
started, `stock` leaves it roughly there, `restore` fixes the rest.
Captures go to captures/<experiment>-*.jsonl.
"""

from __future__ import annotations

import argparse
import json
import re
import signal
import subprocess
import sys
import time
from pathlib import Path

UV = "uv"
HERE = Path(__file__).resolve().parent.parent  # uart-bridge/
CAPTURES = HERE / "captures"
STOP = "c50100000000015c"
REPORT_S = 0.3  # a zoom pulse this long always yields a report (they come every ~225 ms)


def frame(c1=0, c2=0, d1=0, d2=0, sync=0xC5, addr=1, ck=None, trailer=b"\x5c") -> str:
    b = bytes([sync, addr, c1, c2, d1, d2])
    return (b + bytes([sum(b[1:]) % 256 if ck is None else ck]) + trailer).hex()


ZOOM_IN, ZOOM_OUT = frame(c2=0x20), frame(c2=0x40)


def uart_bridge(*args, **kw):
    return subprocess.run([UV, "run", "uart-bridge", *args], cwd=HERE, **kw)


def zooms(log: Path) -> list[float]:
    raw = bytes.fromhex("".join(json.loads(line)["x"] for line in open(log)
                                if json.loads(line).get("d") == "p2c"))
    return [float(z) for z in re.findall(rb"X(\d+\.\d)", raw)]


def inject(name: str, records: list[tuple[float, str]], tail: float = 1.2) -> list[float]:
    """Host as camera: send (seconds, hex) records, return the zoom reports."""
    CAPTURES.mkdir(exist_ok=True)
    src, out = CAPTURES / f"{name}.src.jsonl", CAPTURES / f"{name}.jsonl"
    with open(src, "w") as f:
        f.write(json.dumps({"type": "header", "version": 1, "mode": "synthetic", "note": name}) + "\n")
        for t, x in records:
            f.write(json.dumps({"t": int(t * 1e9), "d": "c2p", "x": x}) + "\n")
    out.unlink(missing_ok=True)
    # --on-abort: if this run is interrupted before its stop frame is due,
    # inject still sends one, so no motor is left running.
    uart_bridge("inject", "--replay", str(src), "--tail", str(tail), "--quiet", "--log", str(out),
                "--on-abort", STOP, capture_output=True, check=True)
    return zooms(out)


def pulse(name: str, cmd: str, hold: float = 0.35) -> list[float]:
    return inject(name, [(0, cmd), (hold, STOP)])


class Bridge:
    """`uart-bridge bridge` in the background, with marks and probes on stdin."""

    def __init__(self, log: str, note: str, pty: bool = False):
        self.log = HERE / log
        self.log.unlink(missing_ok=True)  # the bridge never overwrites a capture
        args = [UV, "run", "uart-bridge", "bridge", "--quiet", "--note", note, "--log", log]
        if pty:
            args[4:4] = ["--cam", "pty"]
        self.p = subprocess.Popen(args, cwd=HERE, stdin=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        self.path = None
        for line in self.p.stderr:
            m = re.search(r"(/dev/pts/\d+)|logging to", line)
            if m:
                self.path = m.group(1)
                if not pty or self.path:
                    break
        if self.p.poll() is not None or (pty and not self.path):
            raise SystemExit(f"uart-bridge bridge did not start (exit {self.p.poll()})")

    def send(self, line: str) -> None:
        self.p.stdin.write(line + "\n")
        self.p.stdin.flush()

    def close(self) -> None:
        self.p.stdin.close()
        self.p.send_signal(signal.SIGTERM)
        self.p.wait()


def cmd_stock(a) -> None:
    sys.path.insert(0, str(Path(a.python_dvr).expanduser()))
    from dvrip import DVRIPCam  # python-dvr

    br = Bridge("captures/e1-stock.jsonl", "E1 stock firmware PTZ via DVRIP")
    time.sleep(3)
    cam = DVRIPCam(a.camera, user=a.user, password=a.password)
    if not cam.login():
        raise SystemExit("DVRIP login failed")
    print("Uart.PTZ:", json.dumps(cam.get_info("Uart.PTZ")))

    def param(preset):
        return {"AUX": {"Number": 0, "Status": "On"}, "Channel": 0, "MenuOpts": "Enter",
                "POINT": {"bottom": 0, "left": 0, "right": 0, "top": 0}, "Pattern": "SetBegin",
                "Preset": preset, "Step": 5, "Tour": 0}

    def step(cmd, hold=0.5):
        # python-dvr's ptz_step convention: Preset 65535 starts, -1 stops.
        # The board keeps moving until it gets a stop, so stop no matter what.
        br.send(f"{cmd} start")
        cam.set_command("OPPTZControl", {"Command": cmd, "Parameter": param(65535)})
        try:
            time.sleep(hold)
        finally:
            br.send(f"{cmd} stop")
            cam.set_command("OPPTZControl", {"Command": cmd, "Parameter": param(-1)})
        time.sleep(1.5)

    try:
        for c in ("ZoomTile", "ZoomWide", "FocusNear", "FocusFar", "IrisSmall", "IrisLarge",
                  "DirectionLeft", "DirectionRight", "DirectionUp", "DirectionDown",
                  "DirectionLeftUp", "DirectionRightDown"):
            step(c)
        for c in ("SetPreset", "GotoPreset", "ClearPreset"):
            br.send(f"{c} {a.preset}")
            cam.ptz(c, preset=a.preset)
            time.sleep(2.5)
    finally:
        cam.close()
        time.sleep(1)
        br.close()
    print("capture: captures/e1-stock.jsonl  (uv run uart-bridge decode ...)")


def cmd_accept(a) -> None:
    def case(name, cmd):
        z = pulse(f"e3-{name}", cmd)
        print(f"{name:20s} {cmd:18s} {'ACTED ' + str(z) if z else 'ignored'}", flush=True)
        if z:
            pulse(f"e3-{name}-back", ZOOM_OUT)

    for name, cmd in [
        ("reference", ZOOM_IN),
        ("sync-ff", frame(c2=0x20, sync=0xFF)),
        ("sync-a0", frame(c2=0x20, sync=0xA0)),
        ("cksum-00", frame(c2=0x20, ck=0x00)),
        ("cksum-mod100", frame(c2=0x20, d1=0x40, d2=0x40, ck=(1 + 0x20 + 0x40 + 0x40) % 100)),
        ("cksum-mod256", frame(c2=0x20, d1=0x40, d2=0x40)),
        ("addr-0", frame(c2=0x20, addr=0)),
        ("addr-ff", frame(c2=0x20, addr=0xFF)),
        ("trailer-00", frame(c2=0x20, trailer=b"\x00")),
    ]:
        case(name, cmd)
    # The parser trap: a lone sync byte swallows the next command.
    for name, first in (("lone-a5", "a5"), ("lone-c5", "c5"), ("lone-5a", "5a"),
                        ("whole-a5-frame", "a57b9ef0efeee0f4")):
        z = inject(f"e3e-{name}-then-zoom", [(0, first), (0.2, ZOOM_IN), (0.55, STOP)])
        print(f"{name + ' + zoom':20s} {'ACTED ' + str(z) if z else 'command lost'}", flush=True)
        if z:
            pulse(f"e3e-{name}-back", ZOOM_OUT)
        else:
            inject(f"e3e-{name}-flush", [(0, STOP), (0.1, STOP)], tail=0.3)  # realign the parser


def cmd_tool(a) -> None:
    binary = str(Path(a.binary).resolve())
    br = Bridge("captures/e2-tool.jsonl", f"E2 {binary} via pty", pty=True)
    print("pty:", br.path)
    tool = subprocess.Popen([binary, "-d", br.path], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT)

    def key(k, what):
        br.send(f"key {k!r}: {what}")
        tool.stdin.write(k.encode())
        tool.stdin.flush()

    time.sleep(1)
    for k, hold, what in (("+", 0.5, "zoom in"), ("-", 0.5, "zoom out"), ("z", 0.4, "focus near"),
                          ("x", 0.4, "focus far"), ("h", 0.3, "pan left"), ("l", 0.3, "pan right"),
                          ("j", 0.3, "tilt down"), ("k", 0.3, "tilt up")):
        key(k, what)
        time.sleep(hold)
        key(" ", "stop")
        time.sleep(1.2)
    key("+", "zoom in, then SIGINT mid-move")
    time.sleep(0.4)
    tool.send_signal(signal.SIGINT)
    try:
        out = tool.communicate(timeout=3)[0].decode(errors="replace")
    except subprocess.TimeoutExpired:
        tool.kill()
        out = tool.communicate()[0].decode(errors="replace")
        print("tool ignored SIGINT")
    time.sleep(1)
    back = subprocess.Popen([binary, "-d", br.path], stdin=subprocess.PIPE, stdout=subprocess.DEVNULL)
    br.send("zoom back")
    time.sleep(0.3)
    for k, pause in ((b"-", 0.4), (b"q", 0)):
        back.stdin.write(k)
        back.stdin.flush()
        time.sleep(pause)
    back.wait(timeout=3)
    time.sleep(0.5)
    br.close()
    print(out[-2000:])
    print("capture: captures/e2-tool.jsonl; compare: uv run uart-bridge diff "
          "../xm-uart/captures/e2-xmuart-new.jsonl captures/e2-tool.jsonl")


# 2592x1944 frame, w:h:x:y; adjust to the scene in front of the camera
REGIONS = {"near": "700:750:130:230", "far": "330:900:640:340", "mid": "600:250:1450:520"}


def cmd_focus(a) -> None:
    url = (f"rtsp://{a.camera}:554/user={a.user}&password={a.password}"
           "&channel=1&stream=0.sdp?real_stream")
    CAPTURES.mkdir(exist_ok=True)
    snap = CAPTURES / f"focus-{time.strftime('%H%M%S')}.jpg"
    subprocess.run(["ffmpeg", "-loglevel", "error", "-rtsp_transport", "tcp", "-i", url,
                    "-frames:v", "1", "-y", str(snap)], check=True, timeout=30)
    for name, crop in REGIONS.items():
        r = subprocess.run(["ffmpeg", "-hide_banner", "-i", str(snap), "-vf", f"crop={crop},blurdetect",
                            "-f", "null", "-"], capture_output=True, text=True)
        m = re.search(r"blur mean: ([0-9.]+)", r.stderr)
        print(f"{name:5s} blur={m.group(1) if m else '?'}  (lower is sharper)")
    print("snapshot:", snap)


def sharpness_blur(a) -> float:
    """Whole-frame blurdetect of one RTSP frame (lower is sharper)."""
    url = (f"rtsp://{a.camera}:554/user={a.user}&password={a.password}"
           "&channel=1&stream=0.sdp?real_stream")
    r = subprocess.run(["ffmpeg", "-hide_banner", "-rtsp_transport", "tcp", "-i", url, "-frames:v", "1",
                        "-vf", "blurdetect", "-f", "null", "-"], capture_output=True, text=True, timeout=30)
    return float(re.search(r"blur mean: ([0-9.]+)", r.stderr).group(1))


def cmd_refocus(a) -> None:
    """Contrast hill-climb with focus pulses: move while the image sharpens,
    reverse and halve the pulse when it gets worse (like an41908a's AF)."""
    near, far = frame(c2=0x80), frame(c1=0x01)
    best = sharpness_blur(a)
    print(f"start blur={best:.2f}", flush=True)
    direction, hold = near, 0.4
    while hold >= 0.05:
        pulse("refocus", direction, hold=hold)
        time.sleep(0.5)
        now = sharpness_blur(a)
        print(f"{'near' if direction == near else 'far '} {hold:.2f}s blur={now:.2f}", flush=True)
        if now < best:
            best = now
            continue
        # Worse: undo this step, then continue the other way with smaller
        # pulses. The undo never lands exactly where the step started
        # (backlash, uneven motor speed), so measure again instead of
        # trusting the old value.
        direction = far if direction == near else near
        pulse("refocus", direction, hold=hold)
        time.sleep(0.5)
        best = sharpness_blur(a)
        print(f"undo -> blur={best:.2f}", flush=True)
        hold /= 2
    print(f"done blur={sharpness_blur(a):.2f}")


def resync_camera_osd() -> list[float]:
    """The camera shows the last zoom report *it* received. Everything sent
    by inject bypassed it, so nudge the zoom in and back through the bridge
    to hand it a current report. Returns the reports seen, since the nudge
    itself moves the lens."""
    br = Bridge("captures/osd-resync.jsonl", "resync camera OSD zoom")
    try:
        for cmd in (ZOOM_IN, ZOOM_OUT):
            time.sleep(1.0)
            br.send("!" + cmd)
            time.sleep(REPORT_S)  # long enough that the board reports it
            br.send("!" + STOP)
        time.sleep(1.5)
    finally:
        br.close()
    return zooms(br.log)


class CaptureError(Exception):
    pass


def gray_frames(a, n: int = 2) -> tuple[int, int, list[bytes]]:
    """n consecutive RTSP frames as 8-bit greyscale, straight from ffmpeg."""
    url = (f"rtsp://{a.camera}:554/user={a.user}&password={a.password}"
           "&channel=1&stream=0.sdp?real_stream")
    probe = subprocess.run(["ffprobe", "-v", "error", "-rtsp_transport", "tcp", "-select_streams", "v:0",
                            "-show_entries", "stream=width,height", "-of", "csv=p=0", url],
                           capture_output=True, text=True, timeout=30)
    try:
        w, h = (int(v) for v in probe.stdout.strip().split(",")[:2])
    except ValueError:
        raise CaptureError(f"ffprobe gave no frame size (exit {probe.returncode}): {probe.stderr.strip()}")
    cap = subprocess.run(["ffmpeg", "-loglevel", "error", "-rtsp_transport", "tcp", "-i", url,
                          "-frames:v", str(n), "-f", "rawvideo", "-pix_fmt", "gray", "-"],
                         capture_output=True, timeout=30)
    if cap.returncode != 0 or len(cap.stdout) < n * w * h:
        raise CaptureError(f"ffmpeg returned {len(cap.stdout) // (w * h)} of {n} frames "
                           f"(exit {cap.returncode}): {cap.stderr.decode(errors='replace').strip()}")
    return w, h, [cap.stdout[i * w * h:(i + 1) * w * h] for i in range(n)]


def tenengrad(frame: bytes, w: int, x: int, y: int, r: int) -> float:
    """Mean squared central-difference gradient in a (2r)^2 box: higher is sharper."""
    total = 0
    for yy in range(y - r + 1, y + r - 1):
        row, up, down = yy * w, (yy - 1) * w, (yy + 1) * w
        for xx in range(x - r + 1, x + r - 1):
            gx = frame[row + xx + 1] - frame[row + xx - 1]
            gy = frame[down + xx] - frame[up + xx]
            total += gx * gx + gy * gy
    return total / (2 * r - 2) ** 2


def peak_position(values: list[float]) -> float | None:
    """Centroid of the part of a sharpness curve above 80% of its maximum;
    None for a flat curve (no texture, blank or covered target)."""
    top = max(values)
    if top <= 0 or top - min(values) < 0.05 * top:
        return None
    pts = [(i + 1, v / top - 0.8) for i, v in enumerate(values) if v / top >= 0.8]
    return sum(i * wgt for i, wgt in pts) / sum(wgt for _, wgt in pts)


def parse_target(text: str) -> tuple[str, tuple[int, int, int]]:
    try:
        name, xyr = text.split("=")
        x, y, r = (int(v) for v in xyr.split(","))
    except ValueError:
        raise SystemExit(f"target {text!r}: expected NAME=X,Y,R")
    if r < 3:
        raise SystemExit(f"target {name}: radius {r} too small, need >= 3")
    return name, (x, y, r)


def cmd_focusdir(a) -> None:
    """Which focus bit moves focus nearer. Sweep focus across its range in
    small steps (one direction per sweep, backlash taken up first) and find
    where each target is sharpest: sweeping nearer, far targets peak before
    near ones. The sweep is repeated in reverse and again forwards, so a
    physical effect has to flip sign with the direction."""
    parsed = [parse_target(t) for t in a.near + a.far]
    names = [n for n, _ in parsed]
    if len(set(names)) != len(names):
        raise SystemExit(f"target names must be unique: {names}")
    near = dict(parsed[:len(a.near)])
    far = dict(parsed[len(a.near):])
    targets = {**near, **far}
    # Pre-flight, before the lens moves: the camera delivers frames and every
    # target box fits inside them.
    try:
        w, h, _ = gray_frames(a, 1)
    except CaptureError as e:
        raise SystemExit(f"camera capture failed: {e}")
    for k, (x, y, r) in targets.items():
        if not (r <= x < w - r and r <= y < h - r):
            raise SystemExit(f"target {k} ({x},{y}) r={r} does not fit a {w}x{h} frame")
    bit80, bit01 = frame(c2=0x80), frame(c1=0x01)
    pulse("focusdir-away", bit01, hold=a.away)
    time.sleep(0.3)
    verdicts = []
    for name, move in (("cmd2 0x80", bit80), ("cmd1 0x01", bit01), ("cmd2 0x80", bit80)):
        pulse("focusdir-takeup", move, hold=0.15)
        time.sleep(0.3)
        curves = {k: [] for k in targets}
        for _ in range(a.steps):
            pulse("focusdir-step", move, hold=a.step)
            time.sleep(0.3)
            try:
                w, _, frames = gray_frames(a)
            except CaptureError as e:
                raise SystemExit(f"camera capture failed mid-sweep ({e}); focus is off, "
                                 "run `refocus` once the camera is back")
            for k, (x, y, r) in targets.items():
                curves[k].append(sum(tenengrad(f, w, x, y, r) for f in frames) / len(frames))
        peaks = {k: peak_position(v) for k, v in curves.items()}
        flat = [k for k, v in peaks.items() if v is None]
        if flat:
            print(f"sweep {name}: inconclusive, no sharpness change on {', '.join(flat)}", flush=True)
            verdicts.append(None)
            continue
        offset = (sum(peaks[k] for k in near) / len(near)) - (sum(peaks[k] for k in far) / len(far))
        print(f"sweep {name}: " + "  ".join(f"{k}={v:.2f}" for k, v in peaks.items())
              + f"  near-far={offset:+.2f} steps", flush=True)
        # near peaking later means this sweep moves focus nearer
        verdicts.append(("0x80" if move == bit80 else "0x01") if offset > 0 else
                        ("0x01" if move == bit80 else "0x80"))
    if None not in verdicts and len(set(verdicts)) == 1:
        bit = verdicts[0]
        print(f"focus NEARER = {'cmd2 0x80' if bit == '0x80' else 'cmd1 0x01'} (all three sweeps agree)")
    else:
        print(f"inconclusive: sweeps disagree {verdicts}")
    pulse("focusdir-back", bit01, hold=a.away)  # roughly back; use `refocus` to finish


def cmd_restore(a) -> None:
    """Zoom to --zoom using the board's own reports. Exits non-zero if the
    target cannot be reached (end stop) or the board stops answering."""

    def confirm_silence(i, cmd, last):
        # A short pulse can end without a report, so silence is only an end
        # stop once a report-length pulse is silent too and the board still
        # answers in the other direction.
        z = pulse(f"restore-{i}-confirm", cmd, hold=REPORT_S)
        if z:
            return z
        other = ZOOM_IN if cmd == ZOOM_OUT else ZOOM_OUT
        if pulse(f"restore-{i}-other", other, hold=REPORT_S):
            pulse(f"restore-{i}-back", cmd, hold=REPORT_S + 0.2)  # undo, and a bit more
            end = "wide" if cmd == ZOOM_OUT else "tele"
            raise SystemExit(f"zoom {end} end stop reached near X{last}; X{a.zoom} is out of range")
        raise SystemExit("the board stopped sending zoom reports in both directions")

    # Where are we? Probe toward tele; at the tele end stop that is silent,
    # so probe toward wide instead and start from that report.
    z = pulse("restore-probe", ZOOM_IN, hold=REPORT_S) or \
        pulse("restore-probe-wide", ZOOM_OUT, hold=REPORT_S)
    if not z:
        raise SystemExit("no zoom reports from the board in either direction")
    for i in range(40):
        now = z[-1]
        if abs(now - a.zoom) < 0.05:
            # Only done once the camera has seen a report on target: the
            # resync nudge moves the lens too, so measure what it left.
            z = resync_camera_osd()
            if not z:
                raise SystemExit("no zoom reports during the camera OSD resync")
            if abs(z[-1] - a.zoom) < 0.05:
                print("zoom", z[-1], "(camera OSD in sync)")
                return
            print("resync left zoom at", z[-1], "- correcting")
            continue
        cmd = ZOOM_OUT if now > a.zoom else ZOOM_IN
        z = pulse(f"restore-{i}", cmd, hold=min(0.4, 0.1 + abs(now - a.zoom) / 4))
        if not z:
            z = confirm_silence(i, cmd, now)
    raise SystemExit(f"gave up at X{z[-1]}")


def positive_int(text: str) -> int:
    v = int(text)
    if v <= 0:
        raise argparse.ArgumentTypeError(f"must be > 0, got {v}")
    return v


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)
    for name, fn in (("stock", cmd_stock), ("focus", cmd_focus), ("refocus", cmd_refocus),
                     ("focusdir", cmd_focusdir)):
        s = sub.add_parser(name)
        s.add_argument("--camera", required=True)
        s.add_argument("--user", default="admin")
        s.add_argument("--password", default="")
        s.set_defaults(func=fn)
    sub.choices["stock"].add_argument("--python-dvr", default="~/git/python-dvr")
    sub.choices["stock"].add_argument("--preset", type=int, default=5)
    fd = sub.choices["focusdir"]
    # Defaults: the 85H50AI rig at X2.4. The chair occludes the doorway and the
    # far door is seen through it, so their depth order is certain.
    fd.add_argument("--near", action="append", metavar="NAME=X,Y,R",
                    default=None, help="near target box centre and half-size (repeatable)")
    fd.add_argument("--far", action="append", metavar="NAME=X,Y,R", default=None)
    fd.add_argument("--steps", type=positive_int, default=30)
    fd.add_argument("--step", type=float, default=0.1, help="focus pulse per step, s")
    fd.add_argument("--away", type=float, default=1.5, help="initial defocus, s")
    sub.add_parser("accept").set_defaults(func=cmd_accept)
    t = sub.add_parser("tool")
    t.add_argument("binary")
    t.set_defaults(func=cmd_tool)
    r = sub.add_parser("restore")
    r.add_argument("--zoom", type=float, default=1.2)
    r.set_defaults(func=cmd_restore)
    a = p.parse_args()
    if a.cmd == "focusdir":
        a.near = a.near or ["chair_mesh=200,300,90"]
        a.far = a.far or ["room_door=660,640,70", "door_leaf=920,560,70", "star=1820,540,70"]
    a.func(a)


if __name__ == "__main__":
    main()
