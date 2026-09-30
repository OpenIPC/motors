"""Turn a capture into frames and compare two captures."""

from __future__ import annotations

import difflib
from dataclasses import dataclass

from . import codec
from .framing import CAM_RULES, PTZ_RULES, Frame, Framer
from .log import C2M, C2P, H2P, MARK, P2C, Record

RULES = {C2P: CAM_RULES, H2P: CAM_RULES, P2C: PTZ_RULES, C2M: CAM_RULES}
DIRECTIONS = (C2P, H2P, P2C, C2M)
# diff compares what the PTZ board received, whoever sent it: camera traffic
# and bridge probes (or inject writes, logged c2p) are one stream.
TO_PTZ = "to-ptz"
# Muted camera output (c2m) never reached the board; it is compared as its own
# stream, not as board traffic.
STREAMS = {TO_PTZ: (C2P, H2P), P2C: (P2C,), C2M: (C2M,)}


def framers() -> dict[str, Framer]:
    return {d: Framer(r) for d, r in RULES.items()}


@dataclass
class Event:
    t: int
    d: str
    kind: str  # "frame", "junk", "mark"
    data: bytes = b""
    note: str = ""


def events(records: list[Record]) -> list[Event]:
    """Frame every direction. Consecutive junk in one direction is merged, so
    unrecognised bytes form the same event however the reads split them."""
    fr = framers()
    out: list[Event] = []
    last: dict[str, Event] = {}  # previous data event per direction
    last_t: dict[str, int] = {}

    def emit(ev: Event) -> None:
        prev = last.get(ev.d)
        if ev.kind == "junk" and prev is not None and prev.kind == "junk":
            prev.data += ev.data
            return
        out.append(ev)
        last[ev.d] = ev

    for r in records:
        if r.d == MARK:
            out.append(Event(r.t, MARK, "mark", note=r.note))
            continue
        last_t[r.d] = r.t
        for item in fr[r.d].feed(r.t, r.data):
            emit(Event(item.t, r.d, "frame" if isinstance(item, Frame) else "junk", item.data))
    for d, f in fr.items():
        if f.pending():
            emit(Event(last_t[d], d, "junk", f.pending()))
    return out


def key_of(ev: Event, strict_a5: bool = False) -> str:
    if ev.kind == "junk":
        return "junk:" + ev.data.hex(" ")
    return codec.key(ev.data, strict_a5)


@dataclass
class Segment:
    key: str
    count: int
    t: int  # first occurrence
    t_last: int = 0

    @property
    def span_ms(self) -> float:
        return (self.t_last - self.t) / 1e6


def segments(evs: list[Event], d: str | tuple[str, ...], strict_a5: bool = False) -> list[Segment]:
    """Run-length encode the frames of one direction (or several, merged in
    capture order) by key()."""
    dirs = (d,) if isinstance(d, str) else d
    out: list[Segment] = []
    for ev in evs:
        if ev.d not in dirs or ev.kind == "mark":
            continue
        k = key_of(ev, strict_a5)
        if out and out[-1].key == k:
            out[-1].count += 1
            out[-1].t_last = ev.t
        else:
            out.append(Segment(k, 1, ev.t, ev.t))
    return out


@dataclass
class Summary:
    frames: dict
    junk_bytes: dict
    duration_s: float
    c2p_rate: float  # of A5 frames, the camera's periodic stream
    c2p_period_ms: tuple[float, float, float]  # min, mean, max between A5 frames


def summarize(evs: list[Event]) -> Summary:
    frames = dict.fromkeys(DIRECTIONS, 0)
    junk = dict.fromkeys(DIRECTIONS, 0)
    ts: list[int] = []
    for ev in evs:
        if ev.kind == "frame":
            frames[ev.d] += 1
            if ev.d == C2P and ev.data[0] == 0xA5:
                ts.append(ev.t)
        elif ev.kind == "junk":
            junk[ev.d] += len(ev.data)
    data_ts = [ev.t for ev in evs if ev.kind != "mark"]
    dur = (data_ts[-1] - data_ts[0]) / 1e9 if len(data_ts) > 1 else 0.0
    gaps = [(b - a) / 1e6 for a, b in zip(ts, ts[1:])]
    period = (min(gaps), sum(gaps) / len(gaps), max(gaps)) if gaps else (0.0, 0.0, 0.0)
    rate = (len(ts) - 1) / ((ts[-1] - ts[0]) / 1e9) if len(ts) > 1 and ts[-1] > ts[0] else 0.0
    return Summary(frames, junk, dur, rate, period)


@dataclass
class Divergence:
    d: str
    op: str  # replace / delete / insert / count / span / gap
    a: list[Segment]
    b: list[Segment]
    detail: str = ""


def diff(evs_a: list[Event], evs_b: list[Event], tolerance: int = 3,
         time_tolerance_ms: float = 150.0, strict_a5: bool = False,
         ignore_edges: bool = False) -> list[Divergence]:
    """Compare two captures direction by direction.

    Segments (runs of frames with the same key) are aligned by key. For each
    aligned pair:
      count  repeat counts differ by more than `tolerance` frames
      span   first-to-last frame time differs by more than `time_tolerance_ms`
      gap    time from the previous aligned segment's last frame differs by
             more than `time_tolerance_ms` (a command sent late or slowly)
    A capture's first and last segment is cut by the capture itself, so its
    count and span are not compared. Unmatched segments are always reported
    unless `ignore_edges`, which drops unmatched runs at either capture end;
    use it only with `strict_a5`, where the counter makes the ends differ.
    """
    out: list[Divergence] = []
    tol_ns = time_tolerance_ms * 1e6
    for d, dirs in STREAMS.items():
        sa, sb = segments(evs_a, dirs, strict_a5), segments(evs_b, dirs, strict_a5)
        sm = difflib.SequenceMatcher(a=[s.key for s in sa], b=[s.key for s in sb], autojunk=False)
        ops = sm.get_opcodes()
        for n, (op, i1, i2, j1, j2) in enumerate(ops):
            if op != "equal":
                if ignore_edges and n in (0, len(ops) - 1) and len(ops) > 1:
                    continue
                out.append(Divergence(d, op, sa[i1:i2], sb[j1:j2]))
                continue
            for k in range(i2 - i1):
                ia, ib = i1 + k, j1 + k
                x, y = sa[ia], sb[ib]
                cut = ia in (0, len(sa) - 1) or ib in (0, len(sb) - 1)
                if not cut and abs(x.count - y.count) > tolerance:
                    out.append(Divergence(d, "count", [x], [y], f"x{x.count} vs x{y.count}"))
                elif not cut and abs(x.span_ms - y.span_ms) * 1e6 > tol_ns:
                    out.append(Divergence(d, "span", [x], [y],
                                          f"{x.span_ms:.0f} ms vs {y.span_ms:.0f} ms"))
                if k > 0:  # previous segment is aligned too
                    # From the previous segment's last frame, which a capture
                    # cut cannot move (it only truncates a segment's start).
                    ga = x.t - sa[ia - 1].t_last
                    gb = y.t - sb[ib - 1].t_last
                    if abs(ga - gb) > tol_ns:
                        out.append(Divergence(d, "gap", [x], [y],
                                              f"{ga / 1e6:.0f} ms vs {gb / 1e6:.0f} ms after previous"))
    return out
