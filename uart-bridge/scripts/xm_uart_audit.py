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
    uart_bridge("inject", "--replay", str(src), "--tail", str(tail), "--quiet", "--log", str(out),
                capture_output=True, check=True)
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
            time.sleep(0.12)
            br.send("!" + STOP)
        time.sleep(1.5)
    finally:
        br.close()
    return zooms(br.log)


def cmd_restore(a) -> None:
    z = pulse("restore-probe", ZOOM_IN, hold=0.1)
    for i in range(40):
        if not z:
            raise SystemExit("no zoom reports from the board")
        now = z[-1]
        if abs(now - a.zoom) < 0.05:
            # Only done once the camera has seen a report on target: the
            # resync nudge moves the lens too, so measure what it left.
            z = resync_camera_osd()
            if z and abs(z[-1] - a.zoom) < 0.05:
                print("zoom", z[-1], "(camera OSD in sync)")
                return
            print("resync left zoom at", z[-1] if z else "?", "- correcting")
            continue
        wide = now > a.zoom
        z = pulse(f"restore-{i}", ZOOM_OUT if wide else ZOOM_IN,
                  hold=min(0.4, 0.1 + abs(now - a.zoom) / 4))
    print("gave up at", z[-1] if z else "?")


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)
    for name, fn in (("stock", cmd_stock), ("focus", cmd_focus), ("refocus", cmd_refocus)):
        s = sub.add_parser(name)
        s.add_argument("--camera", required=True)
        s.add_argument("--user", default="admin")
        s.add_argument("--password", default="")
        s.set_defaults(func=fn)
    sub.choices["stock"].add_argument("--python-dvr", default="~/git/python-dvr")
    sub.choices["stock"].add_argument("--preset", type=int, default=5)
    sub.add_parser("accept").set_defaults(func=cmd_accept)
    t = sub.add_parser("tool")
    t.add_argument("binary")
    t.set_defaults(func=cmd_tool)
    r = sub.add_parser("restore")
    r.add_argument("--zoom", type=float, default=1.2)
    r.set_defaults(func=cmd_restore)
    a = p.parse_args()
    a.func(a)


if __name__ == "__main__":
    main()
