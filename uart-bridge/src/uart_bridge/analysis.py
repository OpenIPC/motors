"""Turn a capture into frames and compare two captures."""

from __future__ import annotations

import difflib
from dataclasses import dataclass

from . import codec
from .framing import Frame, Framer
from .log import C2P, MARK, P2C, Record

# The PTZ board's reply format is not known yet; frame it the same way and let
# anything that does not fit fall out as junk.
FRAMERS = {C2P: Framer, P2C: Framer}


@dataclass
class Event:
    t: int
    d: str
    kind: str  # "frame", "junk", "mark"
    data: bytes = b""
    note: str = ""


def events(records: list[Record]) -> list[Event]:
    framers = {d: f() for d, f in FRAMERS.items()}
    out: list[Event] = []
    for r in records:
        if r.d == MARK:
            out.append(Event(r.t, MARK, "mark", note=r.note))
            continue
        for item in framers[r.d].feed(r.t, r.data):
            kind = "frame" if isinstance(item, Frame) else "junk"
            out.append(Event(item.t, r.d, kind, item.data))
    for d, f in framers.items():
        if f.pending():
            out.append(Event(records[-1].t if records else 0, d, "junk", f.pending()))
    return out


def key_of(ev: Event) -> str:
    if ev.kind == "junk":
        return "junk:" + ev.data.hex(" ")
    if ev.d == C2P:
        return codec.key(ev.data)
    return ev.data.hex(" ")


@dataclass
class Segment:
    key: str
    count: int
    t: int  # first occurrence


def segments(evs: list[Event], d: str) -> list[Segment]:
    """Run-length encode the frames of one direction by key()."""
    out: list[Segment] = []
    for ev in evs:
        if ev.d != d or ev.kind == "mark":
            continue
        k = key_of(ev)
        if out and out[-1].key == k:
            out[-1].count += 1
        else:
            out.append(Segment(k, 1, ev.t))
    return out


@dataclass
class Summary:
    frames: dict
    junk_bytes: dict
    duration_s: float
    c2p_rate: float
    c2p_period_ms: tuple[float, float, float]  # min, mean, max between frame completions


def summarize(evs: list[Event]) -> Summary:
    frames = {C2P: 0, P2C: 0}
    junk = {C2P: 0, P2C: 0}
    ts: list[int] = []
    for ev in evs:
        if ev.kind == "frame":
            frames[ev.d] += 1
            if ev.d == C2P:
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
    op: str  # replace / delete / insert / timing
    a: list[Segment]
    b: list[Segment]


def diff(evs_a: list[Event], evs_b: list[Event], tolerance: int = 3,
         keep_edges: bool = False) -> list[Divergence]:
    """Compare two captures direction by direction.

    Segments are compared by key; a matching segment whose repeat count differs
    by more than `tolerance` frames is a timing divergence. Unmatched runs at
    the very start or end only reflect where each capture was cut, so they are
    dropped unless `keep_edges`.
    """
    out: list[Divergence] = []
    for d in (C2P, P2C):
        sa, sb = segments(evs_a, d), segments(evs_b, d)
        sm = difflib.SequenceMatcher(a=[s.key for s in sa], b=[s.key for s in sb], autojunk=False)
        ops = sm.get_opcodes()
        for n, (op, i1, i2, j1, j2) in enumerate(ops):
            if op == "equal":
                for k in range(i2 - i1):
                    ia, ib = i1 + k, j1 + k
                    x, y = sa[ia], sb[ib]
                    # A capture's first and last segment are cut short by the capture itself.
                    at_edge = ia in (0, len(sa) - 1) or ib in (0, len(sb) - 1)
                    if abs(x.count - y.count) > tolerance and (keep_edges or not at_edge):
                        out.append(Divergence(d, "timing", [x], [y]))
                continue
            # Only an edge next to a match is a capture-cut artefact; a capture
            # with no match at all is a real divergence.
            edge = (n == 0 and len(ops) > 1 and ops[1][0] == "equal") or \
                   (n == len(ops) - 1 and n > 0 and ops[n - 1][0] == "equal")
            if edge and not keep_edges:
                continue
            out.append(Divergence(d, op, sa[i1:i2], sb[j1:j2]))
    return out
