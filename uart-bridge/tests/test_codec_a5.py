"""A5 frames: the camera's clock, day/night, analog gain and focus value,
scrambled as libXmAuto.so xmaf_value_thread_create() does it (codec.py)."""

from uart_bridge import codec

# Captured from the stock camera (xm-uart/captures/e1-stock.jsonl, a5-burst-record).
CAPTURED = [
    ("a57e9ebaa6b36744", 91, False, 1024, 116791),   # 1x gain: byte 2 reads 9E
    ("a5799ebfdcf82db8", 92, False, 1024, 116886),
]


def test_decodes_captured_frames():
    for hexframe, sec, night, gain, fv in CAPTURED:
        assert codec.a5_values(bytes.fromhex(hexframe)) == (sec, night, gain, fv), hexframe


def test_byte2_is_the_gain_high_byte():
    # "9E for hours, then 92" in the old notes: gain 1x, then 2x
    assert codec.a5_values(codec.a5_frame(10, False, 0x400, 5))[2] == 0x400
    assert codec.a5_frame(10, False, 0x400, 5)[2] == 0x9E
    assert codec.a5_frame(10, False, 0x800, 5)[2] == 0x92


def test_round_trip_and_night_bit():
    for sec in (0, 1, 0x3F, 0x40, 127, 200):
        for night in (False, True):
            for gain in (0x400, 0x167E, 0xFFFF):
                for fv in (0, 116791, 0xFFFFFFFF):
                    f = codec.a5_frame(sec, night, gain, fv)
                    assert codec.a5_values(f) == (sec & 0x7F, night, gain, fv)
                    assert bool(f[1] & 0x80) == night


def test_counter_ignores_the_night_bit():
    day, night = codec.a5_frame(33, False, 0x400, 1), codec.a5_frame(33, True, 0x400, 1)
    assert codec.counter(day) == codec.counter(night) == 33


def test_strict_key_drops_the_focus_value():
    a, b = codec.a5_frame(5, False, 0x400, 100), codec.a5_frame(5, False, 0x400, 999)
    assert codec.key(a, strict_a5=True) == codec.key(b, strict_a5=True)
    assert codec.key(a, strict_a5=True) != codec.key(codec.a5_frame(6, False, 0x400, 100), strict_a5=True)
    assert codec.key(a) == "a5"


def test_decode_text():
    assert codec.decode(codec.a5_frame(91, True, 0x800, 42)).text == "stats s= 91 night gain=2.00x fv=42"
