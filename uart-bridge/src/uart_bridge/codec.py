"""Field decoder for the frames framing.py recognises.

The protocol is only partly understood, so this module records what has been
observed and nothing more. Update it as captures confirm fields, and keep
`key()` in sync: diff.py compares captures by `key()`, so it has to drop every
byte that changes without a change in meaning.

Camera -> PTZ, A5 frames (stock firmware sends them at 20/s even when idle):

    byte 0   0xA5 sync
    byte 1   counter ^ 0x25; the counter increments once per second
             (confirmed: consecutive values differ by exactly n ^ (n+1),
             and 0x1a -> 0x65 is the 0x3f -> 0x40 carry)
    byte 2   0x9E, constant
    byte 3-6 change with the counter through a non-linear scramble (not a
             plain XOR with it); not decoded yet
    byte 6   bits 1..0 also flip for single frames between counter steps,
             so they carry live state on top of the scramble
    byte 7   different in every frame, even with bytes 0-6 unchanged;
             not a sum or XOR of bytes 0-6

    The PTZ board does not answer A5 frames, including xm-uart's init[]
    (a5 7b 9e f0 ef ee e0 f4), which is from the same family.

Camera -> PTZ, XM Pelco-D variant (what xm-uart sends; the board answers):

    C5|FF addr cmd1 cmd2 data1 data2 cksum 5C
    cksum = (addr + cmd1 + cmd2 + data1 + data2) % 100
    cmd2: 02 right, 04 left, 08 up, 10 down, 20 zoom tele, 40 zoom wide,
          80 focus far; cmd1: 01 focus near; data1/data2 pan/tilt speed

PTZ -> camera:

    EF 01 type len payload[len]
    type 00: position report, payload ends in ASCII "X<zoom> " (e.g. X1.3);
             sent repeatedly while the zoom moves
    type 02, len 1: 01 night, 00 day (per xm-uart)
"""

from __future__ import annotations

from dataclasses import dataclass

from .framing import SYNC

COUNTER_XOR = 0x25

PELCO_CMD2 = ((0x02, "right"), (0x04, "left"), (0x08, "up"), (0x10, "down"),
              (0x20, "zoom+"), (0x40, "zoom-"), (0x80, "focus-far"))


def counter(frame: bytes) -> int:
    return frame[1] ^ COUNTER_XOR


@dataclass
class Decoded:
    ok: bool
    fields: dict
    text: str


def _a5(frame: bytes) -> Decoded:
    f = {"counter": counter(frame), "magic": frame[2],
         "body": frame[3:7].hex(" "), "tail": frame[7]}
    warn = "" if frame[2] == 0x9E else " !magic"
    return Decoded(True, f, f"xm n={f['counter']:02x} body={f['body']} tail={frame[7]:02x}{warn}")


def _pelco(frame: bytes) -> Decoded:
    addr, c1, c2, d1, d2, ck = frame[1:7]
    ok = (addr + c1 + c2 + d1 + d2) % 100 == ck
    acts = [name for bit, name in PELCO_CMD2 if c2 & bit]
    if c1 & 0x01:
        acts.append("focus-near")
    f = {"addr": addr, "cmd1": c1, "cmd2": c2, "pan_speed": d1, "tilt_speed": d2,
         "actions": acts, "cksum_ok": ok}
    text = (f"pelco addr={addr} {'+'.join(acts) or 'stop'}"
            + (f" pan={d1}" if d1 else "") + (f" tilt={d2}" if d2 else "")
            + ("" if ok else f" !cksum {ck}"))
    return Decoded(ok, f, text)


def _reply(frame: bytes) -> Decoded:
    typ, n = frame[2], frame[3]
    payload = frame[4:4 + n]
    f = {"type": typ, "len": n, "payload": payload.hex(" ")}
    text = f"reply type={typ:02x} {payload.hex(' ')}"
    x = payload.find(b"X")
    if x >= 0 and all(32 <= b < 127 for b in payload[x:]):
        f["zoom"] = payload[x:].decode().strip()
        f["prefix"] = payload[:x].hex(" ")
        text = f"reply type={typ:02x} {f['prefix']} zoom={f['zoom']}"
    elif typ == 0x02 and n == 1:
        f["night"] = payload[0] == 1
        text = f"reply {'night' if payload[0] == 1 else 'day' if payload[0] == 0 else payload.hex()}"
    return Decoded(True, f, text)


def decode(frame: bytes) -> Decoded:
    if len(frame) == 8 and frame[0] == SYNC:
        return _a5(frame)
    if len(frame) == 8 and frame[0] in (0xC5, 0xFF) and frame[7] == 0x5C:
        return _pelco(frame)
    if len(frame) >= 4 and frame[:2] == b"\xef\x01" and len(frame) == 4 + frame[3]:
        return _reply(frame)
    return Decoded(False, {}, "unknown")


def key(frame: bytes) -> str:
    """What a frame means, as far as we know. A5 frames drop the tail byte,
    which changes in every frame; everything else is compared whole."""
    if len(frame) == 8 and frame[0] == SYNC:
        return frame[:7].hex(" ")
    return frame.hex(" ")
