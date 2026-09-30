"""Field decoder for the camera -> PTZ-board frames.

The protocol is only partly understood, so this module records what has been
observed and nothing more. Update it as captures confirm fields, and keep
`key()` in sync: diff.py compares captures by `key()`, so it has to drop every
byte that changes without a change in meaning.

Observed so far (idle camera, stock firmware, 115200 8N1, 20 frames/s):

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

The same frame family appears as `init[]` in xm-uart/main.c
(a5 7b 9e f0 ef ee e0 f4).
"""

from __future__ import annotations

from dataclasses import dataclass

from .framing import SYNC

COUNTER_XOR = 0x25


def counter(frame: bytes) -> int:
    return frame[1] ^ COUNTER_XOR


@dataclass
class Decoded:
    ok: bool
    fields: dict
    text: str


def decode(frame: bytes) -> Decoded:
    if len(frame) != 8 or frame[0] != SYNC:
        return Decoded(False, {}, "bad frame")
    f = {
        "counter": counter(frame),
        "magic": frame[2],
        "body": frame[3:7].hex(" "),
        "tail": frame[7],
    }
    warn = "" if frame[2] == 0x9E else " !magic"
    text = f"n={counter(frame):02x} body={f['body']} tail={frame[7]:02x}{warn}"
    return Decoded(True, f, text)


def key(frame: bytes) -> str:
    """What a frame means, as far as we know: everything but the tail byte."""
    return frame[:7].hex(" ")
