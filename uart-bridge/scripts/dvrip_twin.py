#!/usr/bin/env python3
"""Drive a stock XM camera and an OpenIPC sibling with the same DVRIP PTZ
commands, side by side, and compare what their lenses do.

The stock firmware zooms and leaves focus to the lens board; OpenIPC's majestic
(netip with OPPTZControl, majestic-af behind it) zooms the same way and then
fine-focuses. Both are driven through python-dvr, in lockstep, so the only
difference between the two runs is the camera.

Run on the lab host that has the stock camera's lens board on the bridge (its
zoom comes from the board's own reports), from the uart-bridge directory:

    uv run scripts/dvrip_twin.py --stock 10.0.0.5 --openipc 10.0.0.6 \\
        --openipc-http root:PASS [--reference]

Per zoom level, from the wide stop: zoom in with one held DVRIP move, record
both cameras' video until well after any autofocus has finished, then compare
the zoom reached, how sharp each settled and when. A manual focus nudge ends the
run: OpenIPC must not refocus after it. --reference adds a focus sweep per
level on each camera, so each settled sharpness can be put against that
camera's own best (absolute sharpness is not comparable between cameras).
Sharpness is measured at the frame centre, where zoom goes; aim both cameras
at something textured there, or point --stock-roi / --openipc-roi at it.

The OpenIPC camera needs majestic with netip PTZ and netip.enabled, netip.user
and netip.password (the sofia hash of the DVRIP password) set; see TWIN.md.
Results: captures/dvrip-twin-<time>.json and a table. Exits non-zero when the
zoom differs by more than 0.2 at any level (see ZOOM_TOLERANCE), or a camera
fails.
"""

from __future__ import annotations

import argparse
import base64
import json
import statistics
import sys
import threading
import time
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import xm_tracking as T  # noqa: E402
import xm_uart_audit as A  # noqa: E402

# The same held DVRIP move does not zoom both cameras exactly alike. The stock
# firmware puts the start frame on its lens wire 0.02-0.22 s after the request
# and answers only then, and python-dvr starts its hold on the answer, so the
# stock lens moves 0.10-0.15 s longer than asked (measured on the bridge: 1.6 s
# held 1.70 s, 2.8 s held 2.95 s, 4.1 s held 4.20 s); majestic answers at once.
# At ~0.74 x per second that is ~0.1, and the reports' 0.1 resolution can add as
# much again.
ZOOM_TOLERANCE = 0.2
OBSERVE_S = 30.0  # video per level: the board's ~10 s settle, then majestic-af's pass
QUIET_S = 12.0    # after the zoom-out, before the next zoom-in


class Camera:
    """One camera: its DVRIP control, its video, and where its zoom and
    autofocus state come from."""

    def __init__(self, name: str, dvrip: A.Dvrip, rtsp: str, roi=None):
        self.name, self.dvrip, self.rtsp, self.roi = name, dvrip, rtsp, roi

    def zoom(self) -> float | None:
        raise NotImplementedError

    def af_status(self) -> str | None:
        return None  # the stock firmware has no camera-side autofocus

    def sharpness(self) -> float:
        return round(A.center_sharpness(argparse.Namespace(rtsp=self.rtsp, roi=self.roi)), 1)

    def close(self) -> None:
        self.dvrip.close()


class StockCamera(Camera):
    def __init__(self, name, dvrip, rtsp, bridge: A.Bridge | None, roi=None):
        super().__init__(name, dvrip, rtsp, roi)
        self.bridge = bridge

    def zoom(self):
        if not self.bridge:
            return None
        z = A.zooms(self.bridge.log)
        return z[-1] if z else None


class OpenIpcCamera(Camera):
    def __init__(self, name, dvrip, rtsp, host: str, http_auth: str, roi=None):
        super().__init__(name, dvrip, rtsp, roi)
        self.host = host
        self.auth = "Basic " + base64.b64encode(http_auth.encode()).decode()

    def http(self, path: str) -> str:
        req = urllib.request.Request(f"http://{self.host}{path}", headers={"Authorization": self.auth})
        return urllib.request.urlopen(req, timeout=5).read().decode().strip()

    def zoom(self):
        # "mag=2.2 age_ms=17", or "unknown" before the first zoom after boot
        word = self.http("/zoom").split()[0]
        return float(word.split("=")[1]) if word.startswith("mag=") else None

    def af_status(self):
        return self.http("/autofocus/status")


def settled(tl: list[tuple[float, float]], window_s: float = 2.0) -> float | None:
    """Median sharpness over the last `window_s` of a timeline."""
    if not tl:
        return None
    end = tl[-1][0]
    return round(statistics.median(s for t, s in tl if t >= end - window_s), 1)


def settle_time(tl: list[tuple[float, float]], since: float, tol: float = 0.05) -> float | None:
    """Seconds after `since` from which the timeline stays within `tol` of its
    final value: when the lens stopped changing the picture."""
    final = settled(tl)
    if final is None:
        return None
    after = [(t, s) for t, s in tl if t >= since]
    for i, (t, _) in enumerate(after):
        if all(abs(s - final) <= tol * final for _, s in after[i:]):
            return round(t - since, 1)
    return None


def wait_af(cam: Camera, before: str | None, t_stop: float, limit: float) -> tuple[str | None, float | None]:
    """The camera's next finished autofocus pass after `before`, and how long
    after `t_stop` it finished; (status, None) if none came within `limit` s."""
    if before is None and cam.af_status() is None:
        return None, None
    while time.monotonic() - t_stop < limit:
        s = cam.af_status()
        if s != before and (s.startswith("done") or s.startswith("failed")):
            return s, round(time.monotonic() - t_stop, 1)
        time.sleep(0.5)
    return cam.af_status(), None


def level_run(cam: Camera, level: str, hold: float, gate: threading.Barrier, stamp: str) -> dict:
    """Wide stop, then one held zoom-in to `level`, recorded."""
    cam.dvrip.mark(f"{level}: to the wide stop")
    gate.wait()
    cam.dvrip.step("ZoomWide", hold=T.WIDE_S, settle=0)
    t_wide = time.monotonic()
    wait_af(cam, cam.af_status(), t_wide, 40)  # majestic-af books a pass after a zoom-out too
    time.sleep(max(0.0, t_wide + QUIET_S - time.monotonic()))
    before = cam.af_status()
    out = A.CAPTURES / f"dvrip-twin-{stamp}-{cam.name}-{level}.mkv"
    rec = T.Recording(cam.rtsp, out, OBSERVE_S, cam.name)
    try:
        # Both recordings are running before either zoom starts (an RTSP stream
        # can take a second or more to start): the zooms start together, each
        # at least 2 s into its own video.
        gate.wait()
        time.sleep(max(0.0, rec.t0 + 2.0 - time.monotonic()))
        cmd_at = rec.elapsed()
        cam.dvrip.mark(f"{level}: zoom in {hold} s")
        cam.dvrip.step("ZoomTile", hold=hold, settle=0)
        stop_at, t_stop = rec.elapsed(), time.monotonic()
        # Only as long as the recording still runs: a pass that finishes after
        # the video ends has no settled picture to show for it.
        af, af_s = wait_af(cam, before, t_stop, max(0.0, OBSERVE_S - stop_at))
        tl = rec.finish(cam.roi)
    finally:
        rec.close()
    # final_sharp is the full-size centre sharpness, the metric the reference
    # sweep's best is in; the timeline (smaller, for timing) is not comparable.
    return {"cmd_at": cmd_at, "stop_at": stop_at, "zoom": cam.zoom(), "af": af, "af_done_s": af_s,
            "settled": settled(tl), "settle_s": settle_time(tl, stop_at), "final_sharp": cam.sharpness(),
            "video": out.name, "sharp": tl}


NUDGE_S = 1.5          # well past the 0.45-0.7 s gear slack a reversal takes up first
NUDGE_MIN_CHANGE = 0.1  # a nudge that changes sharpness less than this proves nothing


def manual_focus_run(cam: Camera, gate: threading.Barrier) -> dict:
    """A focus nudge by hand: the picture changes, and no autofocus pass may
    follow it (it would undo what the operator just did). The status is polled
    through the whole 15 s, so a pass that starts and ends between two
    samples is still seen. A pass still running from the last zoom is waited
    out first, so its finishing is not taken for a refocus."""
    t = time.monotonic()
    while cam.af_status() == "running" and time.monotonic() - t < 40:
        time.sleep(0.5)
    before = cam.af_status()
    gate.wait()
    s0 = cam.sharpness()
    cam.dvrip.mark("manual focus nudge")
    cam.dvrip.step("FocusNear", hold=NUDGE_S, settle=0)
    seen = set()
    t_end = time.monotonic() + 15
    while time.monotonic() < t_end:
        st = cam.af_status()
        if st is not None:
            seen.add(st)
        time.sleep(0.5)
    s1 = cam.sharpness()
    return {"sharp_before": s0, "sharp_after": s1, "statuses": sorted(seen),
            "refocused": before is not None and any(st != before for st in seen),
            "moved": bool(s0) and abs(s1 - s0) >= NUDGE_MIN_CHANGE * s0}


def reference_stock(cam: StockCamera, levels: dict) -> dict:
    """Best focus per level on the host's own board: zoom by inject, then a
    focus sweep (xm_tracking's offset experiment). The bridge must be closed."""
    board = T.Board(cam.name, "-", cam.rtsp, cam.roi)
    return {lv: {"reached": board.to_level(hold), **board.focus_offset()} for lv, hold in levels.items()}


def reference_openipc(cam: OpenIpcCamera, levels: dict) -> dict:
    """Best focus per level through majestic: zoom over DVRIP, then a sweep
    of 100 ms focus steps through /ptz, which the plugin times exactly."""
    out = {}
    for lv, hold in levels.items():
        cam.dvrip.step("ZoomWide", hold=T.WIDE_S, settle=0)
        wait_af(cam, cam.af_status(), time.monotonic(), 40)
        before = cam.af_status()
        cam.dvrip.step("ZoomTile", hold=hold, settle=0)
        # majestic-af's own pass first: a sweep that overlapped it would record
        # the pass's moves as well as its own.
        af, af_s = wait_af(cam, before, time.monotonic(), 40)
        if af_s is None or not str(af).startswith("done"):
            out[lv] = {"reached": cam.zoom(), "error": f"the after-zoom pass did not finish ({af})"}
            continue
        # Far side first, past the crest; the sweep then crosses it going near.
        cam.dvrip.step("FocusFar", hold=1.5, settle=0.5)
        curve = []
        for _ in range(30):
            req = urllib.request.Request(f"http://{cam.host}/ptz?move=near:100", method="POST",
                                         headers={"Authorization": cam.auth})
            urllib.request.urlopen(req, timeout=5).read()
            time.sleep(0.45)
            curve.append(cam.sharpness())
        best = max(range(len(curve)), key=curve.__getitem__)
        if best in (0, len(curve) - 1):
            # The sharpest sample at an end of the sweep: the crest was never
            # crossed, so this is no best focus (as focus_offset decides it too).
            out[lv] = {"reached": cam.zoom(), "error": f"no peak inside the sweep (sharpest at step {best})",
                       "curve": curve}
        else:
            out[lv] = {"reached": cam.zoom(), "best": curve[best], "curve": curve}
    return out


def home(step) -> None:
    """Leave a lens at the wide end and in focus. Into the wide stop, then a
    short zoom back out of it: the XM board re-derives focus from its own curve
    when a zoom LEAVES the wide stop (a focus offset made there is not carried),
    not when it arrives, and the zoom books majestic-af a pass on OpenIPC."""
    step("ZoomWide", hold=T.WIDE_S, settle=1.0)
    step("ZoomTile", hold=0.3, settle=0)


def in_parallel(cams: list[Camera], fn, *args) -> dict:
    """Run fn(cam, *args) on every camera at once; errors are results."""
    results: dict = {}

    def one(cam):
        try:
            results[cam.name] = fn(cam, *args)
        except (Exception, SystemExit) as e:  # keep the other camera's result
            for x in args:  # and do not leave it waiting at a barrier for this one
                if isinstance(x, threading.Barrier):
                    x.abort()
            results[cam.name] = {"error": f"{type(e).__name__}: {e}"}
            print(f"{cam.name}: FAILED {results[cam.name]['error']}", flush=True)

    threads = [threading.Thread(target=one, args=(c,), name=c.name) for c in cams]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    return results


def verdict(report: dict, names: tuple[str, str]) -> tuple[list[str], list[str]]:
    """(failures, flags). Failures: a camera erred, or the zoom reached differs
    by more than ZOOM_TOLERANCE. Flags: the OpenIPC camera settled less sharp,
    relative to its own best, than the stock one; its autofocus did not finish;
    or it refocused after a manual nudge."""
    stock, oip = names
    fails, flags = [], []
    for lv, r in report["levels"].items():
        for n in names:
            if "error" in r.get(n, {}):
                fails.append(f"{lv} {n}: {r[n]['error']}")
        if any("error" in r.get(n, {}) for n in names):
            continue
        a, b = r.get(stock, {}).get("zoom"), r.get(oip, {}).get("zoom")
        if b is None or (a is None and not report.get("no_bridge")):
            fails.append(f"{lv}: no zoom reading ({stock} {a}, {oip} {b})")
        elif a is not None and abs(a - b) > ZOOM_TOLERANCE + 1e-9:
            fails.append(f"{lv}: zoom {stock} X{a} vs {oip} X{b}")
        if r.get(oip, {}).get("af_done_s") is None:
            flags.append(f"{lv}: {oip} after-zoom autofocus not seen to finish ({r[oip].get('af')})")
        elif not str(r[oip].get("af")).startswith("done"):
            fails.append(f"{lv}: {oip} autofocus {r[oip]['af']}")
        for n in names:
            if r.get(n, {}).get("ref_zoom_mismatch"):
                flags.append(f"{lv}: {n} reference swept at X{r[n]['ref_zoom_mismatch']}, the level at "
                             f"X{r[n].get('zoom')}: no % of best")
        ra, rb = r.get(stock, {}).get("of_best"), r.get(oip, {}).get("of_best")
        if ra is not None and rb is not None and rb < ra:
            flags.append(f"{lv}: {oip} settled at {rb:.0%} of its best, {stock} at {ra:.0%}")
    for n, m in report.get("manual", {}).items():
        if "error" in m:
            fails.append(f"manual focus {n}: {m['error']}")
    for n, m in report.get("restore", {}).items():
        if "error" in m:
            fails.append(f"final zoom-out {n}: {m['error']}")
    m = report.get("manual", {}).get(oip, {})
    if m.get("refocused"):
        flags.append(f"{oip} refocused after a manual focus nudge")
    if m and "error" not in m and not m.get("moved"):
        flags.append(f"{oip} manual nudge barely changed the picture: the no-refocus check is inconclusive")
    for n, levels in report.get("reference", {}).items():
        if "error" in levels:
            fails.append(f"reference {n}: {levels['error']}")
            continue
        for lv, rr in levels.items():
            if "error" in rr:
                fails.append(f"reference {lv} {n}: {rr['error']}")
    return fails, flags


def table(report: dict, names: tuple[str, str]) -> str:
    stock, oip = names
    rows = [f"{'level':6s} {'zoom ' + stock:>12s} {'zoom ' + oip:>12s} "
            f"{stock + ' %best':>13s} {oip + ' %best':>13s} {stock + ' settle':>13s} "
            f"{oip + ' settle':>13s} {oip + ' AF':>10s}"]
    for lv, r in report["levels"].items():
        a, b = r.get(stock, {}), r.get(oip, {})
        pct = lambda x: f"{x['of_best']:.0%}" if x.get("of_best") is not None else "-"
        sec = lambda v: f"{v}s" if v is not None else "-"
        rows.append(f"{lv:6s} {str(a.get('zoom')):>12s} {str(b.get('zoom')):>12s} {pct(a):>13s} "
                    f"{pct(b):>13s} {sec(a.get('settle_s')):>13s} {sec(b.get('settle_s')):>13s} "
                    f"{sec(b.get('af_done_s')):>10s}")
    return "\n".join(rows)


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--stock", required=True, metavar="IP", help="the stock XM camera")
    p.add_argument("--stock-user", default="admin")
    p.add_argument("--stock-password", default="")
    p.add_argument("--stock-rtsp", help="default: the XM stock stream URL")
    p.add_argument("--openipc", required=True, metavar="IP", help="the OpenIPC camera (majestic)")
    p.add_argument("--openipc-user", default="admin", help="its netip.user")
    p.add_argument("--openipc-password", default="", help="the password netip.password is the hash of")
    p.add_argument("--openipc-http", required=True, metavar="USER:PASS", help="its web/RTSP login")
    p.add_argument("--openipc-rtsp", help="default: rtsp://USER:PASS@IP/stream=0")
    p.add_argument("--stock-roi", type=A.parse_roi, metavar="X,Y,R",
                   help="where to measure sharpness, as fractions of the frame (default: the centre)")
    p.add_argument("--openipc-roi", type=A.parse_roi, metavar="X,Y,R")
    p.add_argument("--levels", nargs="+", default=list(T.LEVELS), choices=list(T.LEVELS))
    p.add_argument("--reference", action="store_true", help="also sweep focus per level for each camera's best")
    p.add_argument("--no-bridge", action="store_true",
                   help="this host is not wired to the stock board: no stock zoom or reference")
    p.add_argument("--python-dvr", default="~/git/python-dvr")
    a = p.parse_args()

    levels = {lv: T.LEVELS[lv] for lv in a.levels}
    names = ("stock", "openipc")
    stock_rtsp = a.stock_rtsp or A.rtsp_url(argparse.Namespace(rtsp=None, camera=a.stock, user=a.stock_user,
                                                                password=a.stock_password))
    oip_rtsp = a.openipc_rtsp or f"rtsp://{a.openipc_http}@{a.openipc}/stream=0"
    stamp = time.strftime("%Y%m%d-%H%M%S")
    A.CAPTURES.mkdir(exist_ok=True)
    bridge = None if a.no_bridge else A.Bridge(f"captures/dvrip-twin-{stamp}.jsonl",
                                                "dvrip_twin: stock camera's lens traffic")
    report: dict = {"stamp": stamp, "no_bridge": a.no_bridge, "levels": {}, "restore": {}}
    out = A.CAPTURES / f"dvrip-twin-{stamp}.json"
    cams: list[Camera] = []
    try:
        time.sleep(2)
        dv = lambda ip, user, pw, br: A.Dvrip(argparse.Namespace(python_dvr=a.python_dvr, camera=ip,
                                                                  user=user, password=pw), br)
        cams = [StockCamera(names[0], dv(a.stock, a.stock_user, a.stock_password, bridge), stock_rtsp, bridge,
                            a.stock_roi),
                OpenIpcCamera(names[1], dv(a.openipc, a.openipc_user, a.openipc_password, None), oip_rtsp,
                              a.openipc, a.openipc_http, a.openipc_roi)]
        for lv, hold in levels.items():
            gate = threading.Barrier(len(cams), timeout=180)
            report["levels"][lv] = in_parallel(cams, level_run, lv, hold, gate, stamp)
            r = report["levels"][lv]
            if any("error" in r.get(n, {}) for n in names):
                # A camera that failed is in an unknown state: move no lens
                # further except to put it back at the wide stop below.
                report["aborted"] = f"at {lv}"
                print(f"{lv}: a camera failed, stopping the run", flush=True)
                break
            print(f"{lv}: " + "  ".join(f"{n} X{r[n].get('zoom')} settled {r[n].get('settled')} "
                                        f"in {r[n].get('settle_s')}s af {r[n].get('af_done_s')}s"
                                        for n in names), flush=True)
        else:
            report["manual"] = in_parallel(cams, manual_focus_run, threading.Barrier(len(cams), timeout=180))
        # The manual check left both lenses defocused on purpose.
        report["restore"] = in_parallel(cams, lambda cam: home(cam.dvrip.step) or {})
    finally:
        for c in cams:
            c.close()
        if bridge:
            bridge.close()
    out.write_text(json.dumps(report))   # the levels are kept whatever the reference does
    if a.reference and not report.get("aborted"):
        refs: dict = {}
        if not a.no_bridge:
            try:
                refs[names[0]] = reference_stock(cams[0], levels)
            except (Exception, SystemExit) as e:
                refs[names[0]] = {"error": f"{type(e).__name__}: {e}"}
            # The sweeps leave the lens at the last level and off focus.
            try:
                home(lambda cmd, hold, settle: A.pulse("dvrip-twin-home", A.ZOOM_OUT if cmd == "ZoomWide"
                                                       else A.ZOOM_IN, hold=hold))
            except (Exception, SystemExit) as e:
                report["restore"][names[0]] = {"error": f"after the reference: {type(e).__name__}: {e}"}
        ref_cam = None
        try:
            ref_cam = OpenIpcCamera(
                names[1], A.Dvrip(argparse.Namespace(python_dvr=a.python_dvr, camera=a.openipc,
                                                     user=a.openipc_user, password=a.openipc_password)),
                oip_rtsp, a.openipc, a.openipc_http, a.openipc_roi)
            refs[names[1]] = reference_openipc(ref_cam, levels)
        except (Exception, SystemExit) as e:
            refs[names[1]] = {"error": f"{type(e).__name__}: {e}"}
        finally:
            if ref_cam:
                try:   # the sweep ends at its near end
                    home(ref_cam.dvrip.step)
                except (Exception, SystemExit) as e:
                    report["restore"][names[1]] = {"error": f"after the reference: {type(e).__name__}: {e}"}
                ref_cam.close()
        report["reference"] = refs
        # The same metric on both sides of the ratio: centre sharpness at full size.
        # And only where the sweep ran at the level's zoom: the stock reference
        # reaches it by timed pulses on the wire, the run by DVRIP, which holds
        # the stock lens a little longer (see ZOOM_TOLERANCE).
        for lv, r in report["levels"].items():
            for n in names:
                ref = refs.get(n, {}).get(lv, {}) if "error" not in refs.get(n, {}) else {}
                best, final = ref.get("best"), r.get(n, {}).get("final_sharp")
                if not (best and final):
                    continue
                z, rz = r[n].get("zoom"), ref.get("reached")
                if z is None or rz is None or abs(z - rz) > 0.1 + 1e-9:
                    r[n]["ref_zoom_mismatch"] = rz   # None: the sweep's zoom is unknown
                    continue
                r[n]["of_best"] = round(final / best, 2)
        out.write_text(json.dumps(report))
    print(table(report, names))
    fails, flags = verdict(report, names)
    for f in flags:
        print("FLAG", f)
    print("saved", out)
    if fails:
        raise SystemExit("failed: " + "; ".join(fails))


if __name__ == "__main__":
    main()
