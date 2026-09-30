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
    byte 2   data, not a constant: 9E for hours, then 92 without a reboot;
             churns (9F 9D 93 90 91 96) in the first minute after boot
    byte 3-6 change with the counter through a non-linear scramble (not a
             plain XOR with it); not decoded yet
    byte 6   bits 1..0 also flip for single frames between counter steps,
             so they carry live state on top of the scramble
    byte 7   different in every frame, even with bytes 0-6 unchanged;
             not a sum or XOR of bytes 0-6

    The PTZ board does not answer A5 frames, including xm-uart's init[]
    (a5 7b 9e f0 ef ee e0 f4), which is from the same family.

Camera -> PTZ, XM Pelco-D variant: see xm-uart/PROTOCOL.md for the full
spec and the captures behind it. In short:

    C5 addr cmd1 cmd2 data1 data2 cksum 5C       (fixed 8 bytes)
    cksum = (addr + cmd1 + cmd2 + data1 + data2) % 256 as the stock firmware
            sends it; the board does not check it, nor addr, nor the 5C
    cmd1: 01 focus farther, 02 iris open, 04 iris close
    cmd2: 02 right, 04 left, 08 up, 10 down, 20 zoom tele, 40 zoom wide,
          80 focus nearer; bit 0 set = extended command
    (the focus bits are named by measured direction, which is the reverse
    of their Pelco-D names; see PROTOCOL.md "Focus direction")
          (03 set preset, 05 clear preset, 07 goto preset, 25 zoom speed, ...)

PTZ -> camera:

    EF 01 type len payload[len]
    type 00, len 09: 04 03 2F 2E + ASCII "X<zoom> ", ~every 225 ms while
             the zoom moves; the camera overlays it on the OSD
    type 00, len 0A: 04 03 2F 2E + six spaces, ~6.7 s after the last zoom
             report (reads like "erase the ratio text"; effect unverified)
    type 02, len 1: 01 night, 00 day (per the old xm-uart; not observed)
"""

from __future__ import annotations

from dataclasses import dataclass

from .framing import SYNC

COUNTER_XOR = 0x25

PELCO_CMD2 = ((0x02, "right"), (0x04, "left"), (0x08, "up"), (0x10, "down"),
              (0x20, "zoom+"), (0x40, "zoom-"), (0x80, "focus-near"))
PELCO_CMD1 = ((0x01, "focus-far"), (0x02, "iris-open"), (0x04, "iris-close"))
EXTENDED = {0x03: "set-preset", 0x05: "clear-preset", 0x07: "goto-preset",
            0x25: "zoom-speed", 0x27: "focus-speed", 0x2B: "auto-focus",
            0x4F: "set-zoom-pos", 0x51: "query-pan", 0x53: "query-tilt", 0x55: "query-zoom"}


def checksum(frame: bytes) -> int:
    return sum(frame[1:6]) % 256


def counter(frame: bytes) -> int:
    return frame[1] ^ COUNTER_XOR


@dataclass
class Decoded:
    ok: bool
    fields: dict
    text: str


def _a5(frame: bytes) -> Decoded:
    f = {"counter": counter(frame), "b2": frame[2],
         "body": frame[3:7].hex(" "), "tail": frame[7]}
    return Decoded(True, f, f"xm n={f['counter']:02x} b2={frame[2]:02x} body={f['body']} tail={frame[7]:02x}")


def _pelco(frame: bytes) -> Decoded:
    addr, c1, c2, d1, d2, ck = frame[1:7]
    ok = checksum(frame) == ck
    note = "" if ok else f" ck={ck:02x}!={checksum(frame):02x}"
    if c2 & 0x01:
        name = EXTENDED.get(c2, f"ext-{c2:02x}")
        f = {"addr": addr, "extended": name, "cmd1": c1, "cmd2": c2, "data1": d1, "data2": d2,
             "cksum_ok": ok}
        return Decoded(True, f, f"pelco addr={addr} {name} {d1:02x} {d2:02x}{note}")
    acts = [name for bit, name in PELCO_CMD2 if c2 & bit]
    acts += [name for bit, name in PELCO_CMD1 if c1 & bit]
    f = {"addr": addr, "cmd1": c1, "cmd2": c2, "pan_speed": d1, "tilt_speed": d2,
         "actions": acts, "cksum_ok": ok}
    text = (f"pelco addr={addr} {'+'.join(acts) or 'stop'}"
            + (f" pan={d1}" if d1 else "") + (f" tilt={d2}" if d2 else "") + note)
    return Decoded(True, f, text)


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
    elif x < 0 and payload[:4] == b"\x04\x03\x2f\x2e" and payload[4:].strip(b" ") == b"":
        f["erase"] = True
        text = f"reply type={typ:02x} zoom report blank"
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


def key(frame: bytes, strict_a5: bool = False) -> str:
    """What a frame means, as far as we know.

    An A5 frame's bytes 1-6 follow a per-second counter whose phase depends on
    when the camera booted, and the scramble is not decoded, so by default an
    A5 frame is only its class: diff then checks that the stream is there and
    its cadence, not its content. `strict_a5` compares bytes 0-6 instead (for
    captures whose counters run in step); byte 7 changes every frame and is
    never compared. Other frames are compared whole."""
    if len(frame) == 8 and frame[0] == SYNC:
        return frame[:7].hex(" ") if strict_a5 else "a5"
    return frame.hex(" ")
