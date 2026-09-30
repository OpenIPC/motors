"""Reassemble frames from a byte stream.

Each direction has a list of rules. A rule claims a frame by its first
byte(s) and says how long the frame is. Once a frame starts we take it whole,
so a sync byte inside a frame (the A5 tail byte looks random) never splits
it. Bytes no rule claims come out as junk.

    camera -> PTZ   A5 xx 9E xx xx xx xx xx       8 bytes, XM scrambled frame
                    C5|FF addr c1 c2 d1 d2 ck 5C  8 bytes, XM Pelco-D variant
    PTZ -> camera   EF 01 type len payload[len]   status reply
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


class Fixed:
    """Frame of `size` bytes whose first byte is one of `syncs`."""

    def __init__(self, syncs: tuple[int, ...], size: int = FRAME_LEN, end: int | None = None):
        self.syncs = syncs
        self.size = size
        self.end = end

    def starts(self, b: int) -> bool:
        return b in self.syncs

    def length(self, buf: bytearray) -> int | None:
        """Frame length, 0 if buf does not hold this frame, None if more bytes are needed."""
        if self.end is not None and len(buf) >= self.size and buf[self.size - 1] != self.end:
            return 0
        return self.size


class Prefixed:
    """`prefix` + type byte + length byte + payload."""

    def __init__(self, prefix: bytes = b"\xef\x01"):
        self.prefix = prefix

    def starts(self, b: int) -> bool:
        return b == self.prefix[0]

    def length(self, buf: bytearray) -> int | None:
        n = len(self.prefix)
        if len(buf) < n + 2:
            if bytes(buf[:n]) != self.prefix[: min(n, len(buf))]:
                return 0
            return None
        if bytes(buf[:n]) != self.prefix:
            return 0
        return n + 2 + buf[n + 1]


CAM_RULES = (Fixed((SYNC,)), Fixed((0xC5, 0xFF), end=0x5C))
# Bytes the XM board's parser treats as a frame start (see xm-uart/PROTOCOL.md).
SYNC_BYTES = (SYNC, 0xC5)
PTZ_RULES = (Prefixed(),)


class Framer:
    def __init__(self, rules=CAM_RULES):
        self.rules = rules
        self.buf = bytearray()

    def _starts(self, b: int) -> bool:
        return any(r.starts(b) for r in self.rules)

    def feed(self, t: int, data: bytes) -> list[Frame | Junk]:
        self.buf += data
        out: list[Frame | Junk] = []
        junk = bytearray()
        while self.buf:
            rule = next((r for r in self.rules if r.starts(self.buf[0])), None)
            n = rule.length(self.buf) if rule else 0
            if n == 0:
                # Not a frame start: skip to the next byte that could be one.
                i = 1
                while i < len(self.buf) and not self._starts(self.buf[i]):
                    i += 1
                junk += self.buf[:i]
                del self.buf[:i]
                continue
            if junk:
                out.append(Junk(t, bytes(junk)))
                junk.clear()
            if n is None or len(self.buf) < n:
                break
            out.append(Frame(t, bytes(self.buf[:n])))
            del self.buf[:n]
        if junk:
            out.append(Junk(t, bytes(junk)))
        return out

    def pending(self) -> bytes:
        return bytes(self.buf)
