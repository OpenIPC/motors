"""scripts/dvrip_twin.py: the DVRIP side against a fake camera, and the pure
helpers that decide what a run measured.

The fake speaks python-dvr's framing (a 20-byte little-endian header, JSON plus
two trailing bytes) and answers login, keep-alive and OPPTZControl the way
majestic's netip does, recording every PTZ request. Skipped when python-dvr is
not where --python-dvr defaults to (or PYTHON_DVR points)."""

import json
import os
import socket
import struct
import sys
import threading
import time
from pathlib import Path

import pytest

SCRIPTS = Path(__file__).resolve().parent.parent / "scripts"
sys.path.insert(0, str(SCRIPTS))
import dvrip_twin as D  # noqa: E402
import xm_uart_audit as A  # noqa: E402

PYTHON_DVR = Path(os.environ.get("PYTHON_DVR", "~/git/python-dvr")).expanduser()


class FakeDvrip:
    def __init__(self):
        self.sock = socket.socket()
        self.sock.bind(("127.0.0.1", 0))
        self.sock.listen(1)
        self.port = self.sock.getsockname()[1]
        self.ptz: list[tuple[float, str, int]] = []
        threading.Thread(target=self.serve, daemon=True).start()

    def serve(self):
        conn, _ = self.sock.accept()
        while True:
            head = b""
            while len(head) < 20:
                chunk = conn.recv(20 - len(head))
                if not chunk:
                    return
                head += chunk
            _, _, session, seq, msg, n = struct.unpack("<BB2xII2xHI", head)
            body = b""
            while len(body) < n:
                body += conn.recv(n - len(body))
            req = json.loads(body[:-2]) if n > 2 else {}
            if msg == 1000:
                reply = {"Ret": 100, "SessionID": "0x00000001", "AliveInterval": 20, "DeviceType ": "IPC"}
            elif msg == 1400:
                op = req["OPPTZControl"]
                self.ptz.append((time.monotonic(), op["Command"], op["Parameter"]["Preset"]))
                ok = op["Command"] in ("ZoomTile", "ZoomWide", "FocusNear", "FocusFar")
                reply = {"Name": "OPPTZControl", "Ret": 100 if ok else 607, "SessionID": "0x00000001"}
            else:
                reply = {"Name": req.get("Name", ""), "Ret": 100, "SessionID": "0x00000001"}
            data = json.dumps(reply).encode() + b"\x0a\x00"
            conn.sendall(struct.pack("<BB2xII2xHI", 255, 1, 1, seq, msg + 1, len(data)) + data)


@pytest.fixture
def camera(monkeypatch):
    if not (PYTHON_DVR / "dvrip.py").exists():
        pytest.skip(f"python-dvr not at {PYTHON_DVR}")
    fake = FakeDvrip()
    sys.path.insert(0, str(PYTHON_DVR))
    import dvrip
    real = dvrip.DVRIPCam.__init__

    def at_fake_port(self, ip, **kw):  # python-dvr has no port argument everywhere
        real(self, ip, **kw)
        self.port = fake.port

    monkeypatch.setattr(dvrip.DVRIPCam, "__init__", at_fake_port)
    cam = A.Dvrip(type("Args", (), {"python_dvr": str(PYTHON_DVR), "camera": "127.0.0.1",
                                    "user": "admin", "password": ""})())
    yield fake, cam
    cam.close()


def test_a_step_is_one_held_move_start_then_stop(camera):
    fake, cam = camera
    cam.step("ZoomTile", hold=0.3, settle=0)
    (t0, c0, p0), (t1, c1, p1) = fake.ptz
    assert (c0, p0) == ("ZoomTile", 65535) and (c1, p1) == ("ZoomTile", -1)
    assert 0.25 <= t1 - t0 <= 1.0
    cam.mark("no bridge: a mark is a no-op")


def test_settled_and_settle_time():
    tl = [(t / 5, 100.0) for t in range(10)] + [(2 + t / 5, 100 + 50 * t) for t in range(10)] + \
         [(4 + t / 5, 600.0) for t in range(20)]
    assert D.settled(tl) == 600.0
    assert D.settle_time(tl, since=2.0) == 2.0      # the picture stopped changing at 4 s
    assert D.settled([]) is None


def test_verdict_fails_on_zoom_and_errors_and_flags_soft_focus():
    names = ("stock", "openipc")
    done = {"af": "done fv=1", "af_done_s": 15.8}
    report = {"levels": {
        "X2.0": {"stock": {"zoom": 2.2, "of_best": 0.4}, "openipc": {"zoom": 2.2, "of_best": 0.95, **done}},
        "X3.0": {"stock": {"zoom": 3.1, "of_best": 1.0}, "openipc": {"zoom": 3.4, "of_best": 0.8, **done}},
        "X4.0": {"stock": {"error": "SystemExit: DVRIP login failed"}, "openipc": {"zoom": 4.0, **done}},
        "X5.0": {"stock": {"zoom": 4.8}, "openipc": {"zoom": 5.0, "af": "failed: x", "af_done_s": 16.0}},
    }, "manual": {"openipc": {"refocused": True, "moved": True}, "stock": {"moved": True}}}
    fails, flags = D.verdict(report, names)
    assert any("X3.0: zoom" in f for f in fails) and any("X4.0 stock" in f for f in fails)
    assert not any("X2.0" in f or "X5.0: zoom" in f for f in fails)   # 0.2 is within tolerance
    assert any("X3.0" in f and "80%" in f for f in flags)
    assert any("X5.0" in f and "autofocus failed" in f for f in fails)
    assert any("refocused" in f for f in flags)
    assert "X2.0" in D.table(report, names)


def test_verdict_does_not_pass_what_it_did_not_measure():
    names = ("stock", "openipc")
    ok = {"af": "done fv=1", "af_done_s": 15.8}
    # a missing zoom reading fails, unless the stock side has no bridge by choice
    r = {"levels": {"X2.0": {"stock": {"zoom": None}, "openipc": {"zoom": 2.2, **ok}}}}
    assert D.verdict(r, names)[0]
    assert not D.verdict({**r, "no_bridge": True}, names)[0]
    r = {"levels": {"X2.0": {"stock": {"zoom": 2.2}, "openipc": {"zoom": None, **ok}}}}
    assert D.verdict(r, names)[0]
    # a stale "done" with no pass seen to finish is not a finished pass
    r = {"levels": {"X2.0": {"stock": {"zoom": 2.2}, "openipc": {"zoom": 2.2, "af": "done", "af_done_s": None}}}}
    assert any("not seen to finish" in f for f in D.verdict(r, names)[1])
    # errors in the manual check or the reference fail the run
    base = {"levels": {"X2.0": {"stock": {"zoom": 2.2}, "openipc": {"zoom": 2.2, **ok}}}}
    assert D.verdict({**base, "manual": {"stock": {"error": "boom"}}}, names)[0]
    assert D.verdict({**base, "restore": {"openipc": {"error": "timed out"}}}, names)[0]
    assert D.verdict({**base, "reference": {"stock": {"X2.0": {"error": "flat"}}}}, names)[0]
    assert D.verdict({**base, "reference": {"openipc": {"error": "HTTP 500"}}}, names)[0]
    # a nudge that did not change the picture makes the no-refocus check inconclusive
    flags = D.verdict({**base, "manual": {"openipc": {"refocused": False, "moved": False}}}, names)[1]
    assert any("inconclusive" in f for f in flags)
    flags = D.verdict({"levels": {"X2.0": {"stock": {"zoom": 2.2, "ref_zoom_mismatch": 2.0},
                                           "openipc": {"zoom": 2.2, **ok}}}}, names)[1]
    assert any("reference swept at X2.0" in f for f in flags)
    assert D.verdict(base, names) == ([], [])


def test_roi_box_stays_inside_the_frame():
    assert A.roi_box(640, 360, None, 120) == (320, 180, 120)
    assert A.roi_box(2592, 1944, (0.25, 0.3, 0.1), 200) == (648, 583, 194)
    x, y, r = A.roi_box(640, 360, (0.02, 0.98, 0.2), 120)   # pushed in from the corner
    assert r + 1 <= x <= 640 - r - 1 and r + 1 <= y <= 360 - r - 1
    assert A.parse_roi("0.5,0.4,0.1") == (0.5, 0.4, 0.1)
    with pytest.raises(Exception):
        A.parse_roi("0.5,1.2,0.1")


def test_levels_and_the_moves_that_reach_them():
    assert list(D.LEVELS)[0] == "X1.0" and "X5.0" in D.LEVELS
    # a zoom-in level: into the wide stop, then the measured zoom-in, which is recorded
    assert D.moves("X3.0") == [("ZoomWide", 6.5), ("ZoomTile", 2.8)]
    # X1.0 is the wide stop itself: settled at X2.0 first, then the zoom-out into it is recorded
    assert D.moves("X1.0") == [("ZoomWide", 6.5), ("ZoomTile", 1.6), ("ZoomWide", 6.5)]
    # a zoom-out level: settled at the tele end, then the zoom-out (full range less the level's
    # zoom-in hold) is recorded; out-X1.0 goes into the wide stop
    assert D.moves("out-X3.0") == [("ZoomWide", 6.5), ("ZoomTile", 6.5), ("ZoomWide", 2.8)]
    assert D.moves("out-X1.0")[-1] == ("ZoomWide", 6.5)
    assert set(D.OUT_LEVELS) == {"out-X4.0", "out-X3.0", "out-X2.0", "out-X1.5", "out-X1.0"}
    assert list(D.IN_LEVELS) == ["X1.0", "X2.0", "X3.0", "X4.0", "X5.0"]   # the default run


def test_board_only_is_the_picture_before_the_pass_starts():
    # stop at 4.0 s: the window is 1.5-2.8 s after it, before majestic-af's pass (3 s)
    tl = [(t / 10, 100.0 if t < 55 else 50.0 if t <= 68 else 200.0) for t in range(0, 120)]
    assert D.board_only(tl, 4.0) == 50.0
    assert D.board_only([(0.0, 1.0)], 4.0) is None
