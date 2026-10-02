"""A camera's lens driven over ONVIF: the same step / mark / close as
xm_uart_audit.Dvrip, so dvrip_twin.py can drive both cameras over either protocol.

Zoom goes through the PTZ service (ContinuousMove at +/-1, then Stop), focus
through the Imaging service (continuous Move at +/-1, then Stop): what the stock
XM firmware offers and what it turns into lens-board frames (xm-uart/PROTOCOL.md,
"Driven over ONVIF"). The SOAP side is onvif-tt's DUT (zeep with its vendored
WSDLs), imported from a checkout the way python-dvr is."""

from __future__ import annotations

import sys
import time
from pathlib import Path

# The twin's command names, as on DVRIP, and what each becomes over ONVIF:
# (service, sign). Positive Imaging speed is nearer on both firmwares.
COMMANDS = {
    "ZoomTile": ("ptz", +1.0),
    "ZoomWide": ("ptz", -1.0),
    "FocusNear": ("imaging", +1.0),
    "FocusFar": ("imaging", -1.0),
}


def load_dut(onvif_tt: str):
    """onvif-tt's DUT and DUTConfig, from a checkout (its src/ on the path)."""
    src = Path(onvif_tt).expanduser() / "src"
    if not src.is_dir():
        raise SystemExit(f"no onvif-tt checkout at {onvif_tt} (looked for {src})")
    sys.path.insert(0, str(src))
    from onvif_tt.runtime.dut import DUT, DUTConfig  # noqa: E402
    return DUT, DUTConfig


class Onvif:
    """`a` carries onvif_tt (checkout path), camera, port, user, password."""

    def __init__(self, a, br=None, dut=None):
        self.br = br
        if dut is None:
            DUT, DUTConfig = load_dut(a.onvif_tt)
            dut = DUT(DUTConfig(host=a.camera, port=a.port, user=a.user, password=a.password))
        self.d = dut
        profiles = self.d.media.GetProfiles() or []
        ptz = [p for p in profiles if getattr(p, "PTZConfiguration", None)]
        if not ptz:
            raise SystemExit(f"{a.camera}: no media profile with a PTZConfiguration")
        self.profile = ptz[0].token
        self.source = self.d.media.GetVideoSources()[0].token

    def _start(self, service: str, sign: float) -> None:
        if service == "ptz":
            req = self.d.ptz.create_type("ContinuousMove")
            req.ProfileToken = self.profile
            req.Velocity = {"Zoom": {"x": sign}}
            self.d.ptz.ContinuousMove(req)
        else:
            req = self.d.imaging.create_type("Move")
            req.VideoSourceToken = self.source
            req.Focus = {"Continuous": {"Speed": sign}}
            self.d.imaging.Move(req)

    def _stop(self, service: str) -> None:
        if service == "ptz":
            self.d.ptz.Stop({"ProfileToken": self.profile, "PanTilt": True, "Zoom": True})
        else:
            self.d.imaging.Stop({"VideoSourceToken": self.source})

    def step(self, cmd: str, hold: float = 0.5, settle: float = 1.5) -> None:
        """`cmd` for `hold` s, then stop, then `settle` s: one held move, as on DVRIP."""
        if cmd not in COMMANDS:
            raise SystemExit(f"no ONVIF move for {cmd}")
        service, sign = COMMANDS[cmd]
        self.mark(f"onvif {cmd} {hold}s")
        # The hold is timed from the request, not from its answer: the stock
        # firmware can answer a second or more after its frame is on the lens
        # wire (traced: 4.07 s of zoom for a 2.8 s hold timed from the answer),
        # while its start and stop frames lag their requests alike.
        t0 = time.monotonic()
        self._start(service, sign)
        try:
            time.sleep(max(0.0, hold - (time.monotonic() - t0)))
        finally:
            self._stop(service)
        time.sleep(settle)

    def mark(self, note: str) -> None:
        if self.br:
            self.br.send(note)

    def close(self) -> None:
        pass
