#!/usr/bin/env python3
"""Zoom tracking inside the XM lens board: the experiments behind the
"Zoom tracking inside the board" section of xm-uart/PROTOCOL.md.

Run on the lab host from the uart-bridge directory. Name each board to test
with --board NAME PTZ RTSP, where PTZ is `-` for the board wired to this host
(the bridge's PTZ port) or the URL of an `xm-uart -l` relay on another camera:

    uv run scripts/xm_tracking.py --board vendor - 'rtsp://CAM1:554/...' \\
        --board openipc socket://CAM2:9000 rtsp://root:PASS@CAM2/stream=0  all

Boards are driven in parallel, one thread each. The camera must not move the
lens itself meanwhile (stock firmware idle; majestic's lens driver off).

zoom      full zoom-in and zoom-out on video: tracking while the zoom runs (T1)
offset    after a zoom-only move and the settle, how far focus is from sharp (T2)
settle    how long focus keeps moving after the stop; do frames disturb it (T3)
combined  frames with zoom AND focus bits set (T4)
interleave a focus frame during a zoom, and a zoom frame during a focus move (T5)
carry     is a manual focus offset kept through a zoom (T6)
backlash  focus reversal slack and the smallest pulse that moves focus (T7)
stockzoom the host's board only: the stock firmware's own zoom over DVRIP with
          the camera's A5 stream live, against the same zoom injected with the
          camera cut off (needs --camera and --python-dvr)

Results go to captures/tracking-<experiment>.json, video to captures/*.mkv (the
uncommitted working area); the published results are in xm-uart/captures/.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import xm_uart_audit as A  # noqa: E402

NEARER, FARTHER = A.frame(c2=0x80), A.frame(c1=0x01)  # measured: PROTOCOL.md, focus direction
W, H = 640, 360
WIDE_S = 6.5    # zoom-out time that reaches the wide stop from anywhere (full range is 5.6 s)
SETTLE_S = 8.0  # wait after a zoom stop before measuring (the settle is measured by `settle`)
# zoom-in time from the wide stop to each ratio (the same on both boards measured)
LEVELS = {"X2.0": 1.6, "X3.0": 2.8, "X4.0": 4.1, "X5.0": 6.5}


class Board:
    def __init__(self, name: str, ptz: str, rtsp: str):
        self.name, self.ptz, self.rtsp = name, (None if ptz == "-" else ptz), rtsp

    def pulse(self, tag: str, cmd: str, hold: float) -> list[float]:
        return A.pulse(f"trk-{self.name}-{tag}", cmd, hold=hold, ptz=self.ptz)

    def inject(self, tag: str, records, tail: float = 1.0) -> list[float]:
        return A.inject(f"trk-{self.name}-{tag}", records, tail=tail, ptz=self.ptz)

    def sharpness(self) -> float:
        return round(A.center_sharpness(argparse.Namespace(rtsp=self.rtsp)), 1)

    def to_level(self, hold: float) -> float | None:
        """Wide stop, then zoom in for `hold` s and wait out the settle."""
        self.pulse("wide", A.ZOOM_OUT, WIDE_S)
        time.sleep(1.5)
        z = self.pulse("level", A.ZOOM_IN, hold)
        time.sleep(SETTLE_S)
        return z[-1] if z else None

    def focus_offset(self, away: float = 1.5, steps: int = 30) -> dict:
        a = argparse.Namespace(rtsp=self.rtsp, ptz=self.ptz, away=away, steps=steps)
        try:
            curve, offset = A.focus_offset(a, tag=f"trk-{self.name}-fo", verbose=False)
        except (SystemExit, A.CaptureError) as e:
            return {"error": str(e)}
        return {"offset_s": offset, "best": max(curve), "curve": curve}

    def video(self, tag: str, lead: float, records, seconds: float) -> tuple[list, list[float]]:
        """Record `seconds` of video; `lead` s in, inject `records`. Returns the
        centre-sharpness timeline (t_s, value) at 5 fps and the zoom reports."""
        out = A.CAPTURES / f"trk-{self.name}-{tag}.mkv"
        rec = subprocess.Popen(["ffmpeg", "-loglevel", "error", "-rtsp_transport", "tcp", "-i", self.rtsp,
                                "-t", str(seconds), "-c", "copy", "-y", str(out)])
        time.sleep(lead)
        z = self.inject(tag, records, tail=max(1.0, seconds - lead - records[-1][0] - 1))
        rec.wait()
        return timeline(out), z


def timeline(path: Path, fps: int = 5) -> list[tuple[float, float]]:
    raw = subprocess.run(["ffmpeg", "-loglevel", "error", "-i", str(path), "-vf",
                          f"fps={fps},scale={W}:{H},format=gray", "-f", "rawvideo", "-"],
                         capture_output=True).stdout
    return [(i / fps, round(A.tenengrad(raw[i * W * H:(i + 1) * W * H], W, W // 2, H // 2, 120), 1))
            for i in range(len(raw) // (W * H))]


def say(b: Board, *msg) -> None:
    print(f"{b.name:8s}", *msg, flush=True)


def exp_zoom(b: Board) -> dict:
    b.pulse("wide", A.ZOOM_OUT, WIDE_S)
    time.sleep(3)
    out = {}
    for label, cmd in (("in", A.ZOOM_IN), ("out", A.ZOOM_OUT)):
        sharp, z = b.video(f"zoom-{label}", 2.0, [(0, cmd), (7.0, A.STOP)], 20)
        out[label] = {"cmd_at": 2.0, "stop_at": 9.0, "sharp": sharp, "reports": z}
        say(b, f"zoom {label}: {z[0] if z else '?'} -> {z[-1] if z else '?'} in {len(z)} reports, "
               f"sharpness every 1 s:", " ".join(f"{s:.0f}" for _, s in sharp[::5]))
    return out


def exp_offset(b: Board) -> dict:
    out = {}
    for label, hold in LEVELS.items():
        reached = b.to_level(hold)
        # sharpness where the board left focus, measured before the sweep moves it:
        # the offset below leans on an assumed backlash, this does not
        settled = b.sharpness()
        r = out[label] = {"reached": reached, "settled": settled, **b.focus_offset()}
        say(b, label, "reached", r["reached"], "settled", settled, "best", r.get("best"),
            "offset", r.get("offset_s"), r.get("error", ""))
    return out


def exp_settle(b: Board) -> dict:
    during = {
        "plain": [],
        "stop frames every 0.2 s for 3 s": [(1.0 + 0.2 * i, A.STOP) for i in range(15)],
        "focus nearer 0.2 s then farther 0.2 s, at +1 s": [(1.0, NEARER), (1.2, A.STOP),
                                                          (1.4, FARTHER), (1.6, A.STOP)],
    }
    out = {}
    for level in ("X3.0", "X5.0"):
        hold = LEVELS[level]
        for name, extra in during.items():
            b.pulse("wide", A.ZOOM_OUT, WIDE_S)
            time.sleep(SETTLE_S)
            seq = [(0, A.ZOOM_IN), (hold, A.STOP)] + [(hold + t, f) for t, f in extra]
            sharp, z = b.video(f"settle-{level}-{name.split()[0]}", 2.0, seq, 2.0 + hold + 16)
            stop = 2.0 + hold
            out[f"{level} {name}"] = {"stop_at": stop, "sharp": sharp, "reports": z}
            say(b, level, f"{name:46s} after stop, every 1 s:",
                " ".join(f"{s:.0f}" for t, s in sharp if t >= stop and round((t - stop) * 5) % 5 == 0))
    return out


def exp_combined(b: Board) -> dict:
    variants = {
        "tele only (c2 20)": A.frame(c2=0x20),
        "tele + nearer (c2 A0)": A.frame(c2=0xA0),
        "tele + farther (c1 01, c2 20)": A.frame(c1=0x01, c2=0x20),
        "wide only (c2 40)": A.frame(c2=0x40),
        "wide + nearer (c2 C0)": A.frame(c2=0xC0),
        "wide + farther (c1 01, c2 40)": A.frame(c1=0x01, c2=0x40),
    }
    out = {}
    for name, fr in variants.items():
        start = b.to_level(LEVELS["X2.0"])
        s0 = b.sharpness()
        # resent every 50 ms for 1.5 s, the way a held button would
        z = b.inject("combined", [(i * 0.05, fr) for i in range(30)] + [(1.5, A.STOP)])
        s1 = b.sharpness()
        time.sleep(7.0)
        s2 = b.sharpness()
        out[name] = {"start": start, "reports": z, "sharp_before": s0, "sharp_after": s1, "sharp_settled": s2}
        say(b, f"{name:30s} from X{start}: reports {z}  sharpness {s0} -> {s1} -> {s2}")
    return out


def exp_interleave(b: Board) -> dict:
    out = {}
    for name, seq in (("zoom, focus frame at 0.8 s, stop at 2.0 s",
                       [(0, A.ZOOM_IN), (0.8, NEARER), (2.0, A.STOP)]),
                      ("focus, zoom frame at 0.8 s, stop at 2.0 s",
                       [(0, NEARER), (0.8, A.ZOOM_IN), (2.0, A.STOP)])):
        start = b.to_level(LEVELS["X2.0"])
        z = b.inject("interleave", seq)
        out[name] = {"start": start, "reports": z}
        say(b, f"{name}: from X{start}, reports {z}")
    return out


def exp_carry(b: Board) -> dict:
    """Is a manual focus offset kept through a zoom? Reference offset at X3.0,
    then reach X3.0 again after a 1 s farther nudge, made either at the wide
    stop or mid-range at X2.0. A carried nudge moves the offset by ~+0.5 s or
    more (1 s, less whatever of it went into the gear slack). The sharpness
    before and after the nudge shows it moved the lens."""
    b.to_level(LEVELS["X3.0"])
    out = {"reference": b.focus_offset()}
    for where in ("wide stop", "X2.0"):
        if where == "wide stop":
            b.pulse("wide", A.ZOOM_OUT, WIDE_S)
            time.sleep(SETTLE_S)
            zoom_in = LEVELS["X3.0"]
        else:
            b.to_level(LEVELS["X2.0"])
            zoom_in = 1.2  # X2.0 -> X3.0
        before = b.sharpness()
        b.pulse("nudge", FARTHER, 1.0)
        time.sleep(1.0)
        after = b.sharpness()
        z = b.pulse("x3", A.ZOOM_IN, zoom_in)
        time.sleep(SETTLE_S)
        out[f"nudged at {where}"] = {"sharp_before_nudge": before, "sharp_after_nudge": after,
                                     "reached": z[-1] if z else None, **b.focus_offset()}
    say(b, "X3.0 offset: reference", out["reference"].get("offset_s"),
        *(f"| nudged 1 s farther at {w} (sharpness {out[f'nudged at {w}']['sharp_before_nudge']} -> "
          f"{out[f'nudged at {w}']['sharp_after_nudge']}): X{out[f'nudged at {w}']['reached']} "
          f"{out[f'nudged at {w}'].get('offset_s', out[f'nudged at {w}'].get('error'))}"
          for w in ("wide stop", "X2.0")))
    return out


def sweep(b: Board, cmd: str, n: int, step: float) -> list[float]:
    curve = []
    for _ in range(n):
        b.pulse("step", cmd, step)
        time.sleep(0.3)
        curve.append(b.sharpness())
    return curve


def exp_backlash(b: Board) -> dict:
    """At X3.0: from ~0.5 s farther than the tracked focus, sweep nearer in
    `step` pulses through the peak, then straight back farther for twice as
    long. The peak's shift between the two passes is the reversal slack; a
    peak still resolved at the smallest step says that pulse moves the lens."""
    out = {}
    for step in (0.05, 0.03):
        n, m = int(1.6 / step), int(3.2 / step)
        b.to_level(LEVELS["X3.0"])
        b.pulse("away", FARTHER, 0.8)
        b.pulse("takeup", NEARER, 0.3)
        fwd = sweep(b, NEARER, n, step)
        back = sweep(b, FARTHER, m, step)
        pf = max(range(n), key=fwd.__getitem__)
        pb = max(range(m), key=back.__getitem__)
        # after nearer step i the lens is at i+1; after farther step k at n-(k+1-s)
        # once s steps of slack are taken up, so the peak pf+1 = n-(pb+1-s)
        slack = round((pf + pb + 2 - n) * step, 3)
        edge = pb == m - 1 or pf in (0, n - 1)
        out[f"{step}"] = {"nearer_sweep": fwd, "farther_sweep": back, "peak_nearer_step": pf,
                          "peak_farther_step": pb, "backlash_s": None if edge else slack}
        say(b, f"{step} s steps: nearer peak {max(fwd):.0f} at {pf}, farther peak {max(back):.0f} at {pb}: "
               + ("a peak at the sweep's edge, no result" if edge else f"reversal slack {slack:+.2f} s"))
        b.pulse("return", NEARER, step * (pf + 1 - n + m))  # the slack, then back to the peak
    return out


def exp_stockzoom(b: Board) -> dict:
    """Does the board close an autofocus loop on the camera's A5 stream? The same
    zoom, alternately through the stock firmware (camera bridged, A5 live) and
    injected by the host (camera cut off); settled sharpness against the best."""
    if b.ptz is not None:
        return {"skipped": "only the board wired to this host has the stock camera on it"}
    out = []
    for rep in range(2):
        for level in ("X2.0", "X4.0"):
            hold = LEVELS[level]
            for mode in ("stock, A5 live", "host, no A5"):
                if mode.startswith("stock"):
                    br = A.Bridge(f"captures/trk-{b.name}-stockzoom.jsonl", "stock zoom, A5 live")
                    try:
                        time.sleep(2)
                        cam = A.Dvrip(STOCK, br)
                        cam.step("ZoomWide", hold=WIDE_S, settle=1.5)
                        cam.step("ZoomTile", hold=hold, settle=SETTLE_S)
                        cam.close()
                        settled = b.sharpness()
                    finally:
                        br.close()
                    z = A.zooms(br.log)
                else:
                    z = [b.to_level(hold)]
                    settled = b.sharpness()
                r = {"level": level, "mode": mode, "rep": rep, "reached": z[-1] if z else None,
                     "settled": settled, **b.focus_offset()}
                if r.get("best"):
                    r["settled_of_best"] = round(settled / r["best"], 2)
                out.append(r)
                say(b, f"{level} {mode:15s} reached X{r['reached']} settled {settled} of best "
                       f"{r.get('best')}: {r.get('settled_of_best')}")
    return out


STOCK = None  # argparse.Namespace for Dvrip: camera, user, password, python_dvr

EXPERIMENTS = {"zoom": exp_zoom, "offset": exp_offset, "settle": exp_settle, "combined": exp_combined,
               "interleave": exp_interleave, "carry": exp_carry, "backlash": exp_backlash,
               "stockzoom": exp_stockzoom}


def run(boards: list[Board], name: str) -> None:
    results: dict = {}

    def one(b: Board) -> None:
        try:
            results[b.name] = EXPERIMENTS[name](b)
        except Exception as e:  # keep the other board's results
            results[b.name] = {"error": f"{type(e).__name__}: {e}"}
            say(b, "FAILED", results[b.name]["error"])

    threads = [threading.Thread(target=one, args=(b,), name=b.name) for b in boards]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    A.CAPTURES.mkdir(exist_ok=True)
    out = A.CAPTURES / f"tracking-{name}.json"
    out.write_text(json.dumps(results))
    print("saved", out, flush=True)


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--board", nargs=3, action="append", required=True, metavar=("NAME", "PTZ", "RTSP"),
                   help="a lens board: PTZ is - for this host's board, or an xm-uart -l relay URL")
    p.add_argument("--camera", help="stockzoom: the stock XM camera's IP (DVRIP)")
    p.add_argument("--user", default="admin")
    p.add_argument("--password", default="")
    p.add_argument("--python-dvr", default="~/git/python-dvr")
    p.add_argument("experiment", nargs="+", choices=[*EXPERIMENTS, "all"])
    a = p.parse_args()
    global STOCK
    STOCK = argparse.Namespace(camera=a.camera, user=a.user, password=a.password, python_dvr=a.python_dvr)
    if "stockzoom" in a.experiment and not a.camera:
        p.error("stockzoom needs --camera")
    boards = [Board(*spec) for spec in a.board]
    if len({b.name for b in boards}) != len(boards):
        p.error("board names must differ (they name the captures)")
    everything = [e for e in EXPERIMENTS if e != "stockzoom" or a.camera]
    for name in (everything if "all" in a.experiment else a.experiment):
        run(boards, name)
    for b in boards:  # leave every lens in a known state
        b.pulse("end-wide", A.ZOOM_OUT, WIDE_S)
    print("done; the lenses are left wide (`xm_uart_audit.py restore` zooms the host's board back)", flush=True)


if __name__ == "__main__":
    main()
