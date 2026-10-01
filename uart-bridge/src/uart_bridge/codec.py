"""Field decoder for the frames framing.py recognises.

The protocol is only partly understood, so this module records what has been
observed and nothing more. Update it as captures confirm fields, and keep
`key()` in sync: diff.py compares captures by `key()`, so it has to drop every
byte that changes without a change in meaning.

Camera -> PTZ, A5 frames: the camera's image statistics, 20/s even when idle.
Decoded from the code that sends them, libXmAuto.so xmaf_value_thread_create()
in the stock firmware (000529B2, 2021-03-03), and checked against every
captured frame (decode/encode round-trips byte for byte):

    byte 0   0xA5 sync
    byte 1   ((sec ^ 0x25) & 0x7F) | 0x80 at night; sec is the low byte of
             the camera clock's gettimeofday() seconds
    byte 2   AG[15:8] ^ 0x9A           AG = sensor analog gain, the low 16
                                         bits of ISP_EXP_INFO_S.u32AGain
                                         (22.10 fixed point, 0x400 = 1x)
    byte 3   (k - b2) ^ AG[7:0]  ^ 0x65    with k = (b1 - 1) & 0xFF
    byte 4   (k - b3) ^ FV[31:24] ^ 0x65   FV = XM_AF_ValueGet(): the ISP's
    byte 5   (k - b4) ^ FV[23:16] ^ 0x65    17x15 focus-statistic zones,
    byte 6   (k - b5) ^ FV[15:8]  ^ 0x65    weighted by an AF window table,
    byte 7   (k - b6) ^ FV[7:0]   ^ 0x65    horizontal and vertical blended

    So bytes 3-5 change only when the second (or the gain) does, and the low
    bytes of the focus value change in every frame. A queued command frame
    (C5..., or the C5 02 C5 x y w h 5C person-rectangle frame) is sent in an
    A5 slot instead. The PTZ board does not answer A5 frames, and on the
    85H50AI it does not act on them either (PROTOCOL.md, "The camera's A5
    stream").

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

PTZ -> camera (what the stock firmware's AFCommProc() does with each type):

    EF 01 type len payload[len]
    type 00: OSD text at payload[1..2] (x, y halves); payload 04 03 2F 2E +
             ASCII "X<zoom> ", ~every 225 ms while the zoom moves, then six
             spaces ~6.7 s after the last report to erase it
    type 01: OSD text, a fixed string
    type 02, len 1: IR-cut filter, 00 / 01 (the firmware's "afc_dnc")
    types 04, 05, 06: pan, tilt, zoom position (two bytes), stored by the
             camera as Pelco-D responses FF 01 00 59 / 5B / 54 + the bytes
    types 08, 09: the board's AF-controller and PTZ firmware versions
    (types 01-09 are from the firmware's code; only 00 has been seen)
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
    """An A5 frame's seconds, mod 128 (byte 1 without its night bit)."""
    return (frame[1] & 0x7F) ^ COUNTER_XOR


def a5_values(frame: bytes) -> tuple[int, bool, int, int]:
    """(seconds mod 128, night, analog gain, focus value) of an A5 frame."""
    b = frame
    k = (b[1] - 1) & 0xFF
    gain = ((b[2] ^ 0x9A) << 8) | (b[3] ^ 0x65 ^ ((k - b[2]) & 0xFF))
    fv = 0
    for i in range(4, 8):
        fv = (fv << 8) | (b[i] ^ 0x65 ^ ((k - b[i - 1]) & 0xFF))
    return counter(b), bool(b[1] & 0x80), gain, fv


def a5_frame(sec: int, night: bool, gain: int, fv: int) -> bytes:
    """Build an A5 frame as the stock firmware does (inverse of a5_values)."""
    b = [SYNC, ((sec ^ COUNTER_XOR) & 0x7F) | (0x80 if night else 0), ((gain >> 8) & 0xFF) ^ 0x9A]
    k = (b[1] - 1) & 0xFF
    b.append(((k - b[2]) & 0xFF) ^ (gain & 0xFF) ^ 0x65)
    for shift in (24, 16, 8, 0):
        b.append(((k - b[-1]) & 0xFF) ^ ((fv >> shift) & 0xFF) ^ 0x65)
    return bytes(b)


@dataclass
class Decoded:
    ok: bool
    fields: dict
    text: str


def _a5(frame: bytes) -> Decoded:
    sec, night, gain, fv = a5_values(frame)
    f = {"counter": sec, "night": night, "gain": gain, "fv": fv}
    return Decoded(True, f, f"stats s={sec:3d} {'night' if night else 'day'} gain={gain / 1024:.2f}x fv={fv}")


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

    An A5 frame is the camera's clock and live image statistics, which differ
    between any two captures, so by default it is only its class: diff then
    checks that the stream is there and its cadence, not its content.
    `strict_a5` compares its seconds, day/night and gain (for captures whose
    clocks run in step); the focus value changes in every frame and is never
    compared. Other frames are compared whole."""
    if len(frame) == 8 and frame[0] == SYNC:
        if not strict_a5:
            return "a5"
        sec, night, gain, _ = a5_values(frame)
        return f"a5 s={sec} {'night' if night else 'day'} gain={gain}"
    return frame.hex(" ")
