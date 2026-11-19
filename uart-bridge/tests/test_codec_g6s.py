"""The newer lens-link library, libXmAf_Comm.so (GK7205V300 G6S, LibXmAf
ee983c8d, 2025-03-07), against the 85H50AI codec in codec.py.

g6s_af_frame() transcribes XmAf_Send_afValue() and its helper sub_2294() from
the decompile (xm-uart/PROTOCOL.md, "Newer revision"). Nothing here was
captured from a G6S wire; the point is that, for the obfuscated OEMs, the new
code builds exactly the frames the old one did, and the old one is verified
against 14,705 captured frames."""

import random

from uart_bridge import codec

GENERAL, BESTECHER, SSAF, AF_GENERAL = range(4)


def g6s_key(sec: int, night: bool) -> int:
    k = ~(sec ^ 0x5A) & 0xFF  # v22 = ~(tv_sec ^ 0x5A)
    return (k | 0x80) if night else (k & 0x7F)  # bit 7 replaced by day/night


def g6s_af_frame(oem: int, sec: int, night: bool, gain: int, fv: int, flags: int = 0) -> bytes:
    k = g6s_key(sec, night)

    def sub_2294(x: int, prev: int) -> int:
        if oem in (SSAF, AF_GENERAL):
            return (~(x ^ 0x9A) ^ ((~prev) + k)) & 0xFF
        return x

    out, prev = [0xA5, k], k
    for x in ((gain >> 8) & 0xFF, gain & 0xFF, (fv >> 24) & 0xFF, (fv >> 16) & 0xFF, (fv >> 8) & 0xFF, fv & 0xFF):
        prev = sub_2294(x, prev)
        out.append(prev)
    if oem != AF_GENERAL:  # template length 9, the last byte carries two flags
        out.append(flags)
    for i in range(1, 7):  # XmAf_Send_avoidData(buf, 1, 7)
        if out[i] == 0xA5:
            out[i] = 0xA4
    return bytes(out)


def test_key_byte_matches_85h50ai():
    for sec in range(256):
        for night in (False, True):
            assert g6s_key(sec, night) == codec.a5_frame(sec, night, 0, 0)[1]


def test_af_general_is_the_85h50ai_frame():
    rnd = random.Random(1)
    checked = 0
    for _ in range(20000):
        sec, night = rnd.randrange(256), rnd.random() < 0.5
        gain, fv = rnd.randrange(1 << 16), rnd.randrange(1 << 32)
        old = codec.a5_frame(sec, night, gain, fv)
        if 0xA5 in old[1:7]:  # only the new code rewrites those to A4
            continue
        assert g6s_af_frame(AF_GENERAL, sec, night, gain, fv) == old
        assert g6s_af_frame(SSAF, sec, night, gain, fv)[:8] == old
        checked += 1
    assert checked > 19000


def test_general_sends_values_in_the_clear():
    f = g6s_af_frame(GENERAL, 0, False, 0x0123, 0x00045678, flags=3)
    assert f == bytes([0xA5, 0x25, 0x01, 0x23, 0x00, 0x04, 0x56, 0x78, 0x03])
    assert g6s_af_frame(BESTECHER, 0, False, 0x0123, 0x00045678, flags=3) == f


def test_no_stray_sync_in_bytes_1_to_6():
    f = g6s_af_frame(GENERAL, 0, False, 0xA5A5, 0xA5A5A5A5)
    assert f[1:7].count(0xA5) == 0 and f[7] == 0xA5  # byte 7 is not covered
