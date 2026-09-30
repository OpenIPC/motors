"""Reassemble fixed-size sync-prefixed frames from a byte stream.

The camera sends 8-byte frames that start with 0xA5. The sync byte can also
appear inside a frame (the last byte looks random), so once in sync we take
whole frames at a fixed stride and only hunt for a sync byte when a frame does
not start with one. Bytes skipped while hunting are reported as junk.
"""

from __future__ import annotations

from dataclasses import dataclass

SYNC = 0xA5
FRAME_LEN = 8


@dataclass
class Frame:
    t: int  # timestamp of the read that completed the frame
    data: bytes


@dataclass
class Junk:
    t: int
    data: bytes


class Framer:
    def __init__(self, sync: int = SYNC, size: int = FRAME_LEN):
        self.sync = sync
        self.size = size
        self.buf = bytearray()

    def feed(self, t: int, data: bytes) -> list[Frame | Junk]:
        self.buf += data
        out: list[Frame | Junk] = []
        while self.buf:
            if self.buf[0] != self.sync:
                i = self.buf.find(self.sync)
                n = len(self.buf) if i < 0 else i
                out.append(Junk(t, bytes(self.buf[:n])))
                del self.buf[:n]
                continue
            if len(self.buf) < self.size:
                break
            out.append(Frame(t, bytes(self.buf[: self.size])))
            del self.buf[: self.size]
        return out

    def pending(self) -> bytes:
        return bytes(self.buf)
