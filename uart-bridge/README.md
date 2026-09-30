# uart-bridge

A host-side validation harness for UART motor protocols. It is written in Python and does not get cross-compiled.

The host sits between the camera board and the PTZ motor board, with one USB-UART adapter on each board. `uart-bridge`:
- forwards bytes both ways;
- logs every byte with a timestamp;
- decodes frames live;
- can stand in for the camera (`inject`);
- compares captures (`diff`).

The goal is to prove that our motor-control implementation drives the PTZ board the same way the stock firmware does.

```
camera board  TX/RX ── /dev/ttyUSB0 ─┐
                                     host: uart-bridge  ── captures/*.jsonl
PTZ board     TX/RX ── /dev/ttyUSB1 ─┘
```

## Run

```sh
uv run pytest                                   # pty-based tests, no hardware needed
uv run uart-bridge bridge --note "idle"         # forward + log; Ctrl-C to stop
uv run uart-bridge decode captures/<file>.jsonl # annotated timeline (--all: every frame)
uv run uart-bridge diff stock.jsonl ours.jsonl  # exit 1 on divergence
uv run uart-bridge inject --replay stock.jsonl  # host acts as camera, replays c2p + h2p (stop the bridge first)
uv run uart-bridge inject --frame a52e9eea2662efae --rate 20 --duration 2
```

- While `bridge` runs, each line on stdin is stored in the log as a timestamped mark, for example `pan left pressed`. A line of the form `!<hex>` is sent to the PTZ board as a probe and logged as `h2p`. A probe waits for the end of any camera frame in flight, so it never splices into one. If the camera stops mid-frame for 1 s, the probe goes out anyway and the stall is logged as a mark. For example, xm-uart's zoom-in then stop:
  `( sleep 2; echo '!c50100200000215c'; sleep .5; echo '!c50100000000015c'; sleep 2 ) | uv run uart-bridge bridge --duration 6`
- `bridge` and `inject` open the ports exclusively and set the FTDI latency timer to 1 ms through sysfs, using `--latency` (0 leaves it alone). The default is 16 ms, which makes every timestamp late by up to 16 ms.

### Two identical FT232R adapters

Both FT232R adapters report the same serial number (`A5069RR4`), so `/dev/serial/by-id` shows only one of them. When `ttyUSBn` numbering matters, use the by-path names:
- camera: `/dev/serial/by-path/pci-0000:00:14.0-usb-0:3:1.0-port0`
- PTZ board: `/dev/serial/by-path/pci-0000:00:14.0-usb-0:4:1.0-port0`

## Capture format (JSONL)

The first line is a header: `{"type":"header","version":1,"wall":...,"git":...,"mode":...}`, followed by the ports, baud rates and your note. After that there is one record per `read()`:

```
{"t": <ns since start>, "d": "c2p" | "p2c" | "h2p", "x": "<hex>"}
{"t": <ns since start>, "d": "mark", "note": "..."}
```

`c2p` is camera → PTZ board, `p2c` is PTZ board → camera, and `h2p` is a probe the host injected. Record boundaries are read boundaries, not frame boundaries.

## What `diff` compares

The traffic is split into two streams: what the PTZ board received (`c2p` and `h2p` together, since a probe and a camera command reach the board the same way) and what it answered (`p2c`). Each stream is framed and run-length encoded by `codec.key()` into segments. The segment sequences are aligned with difflib, and the diff reports:
- **insert / delete / replace:** segments present in only one capture, including at either end.
- **count:** a segment repeats more than `--tolerance` frames more or less often.
- **span:** a segment lasts more than `--time-tolerance` ms longer or shorter.
- **gap:** a segment starts more than `--time-tolerance` ms earlier or later after the previous one, which catches a command sent late.

A capture's first and last segment is cut by the capture itself, so its count and span are not compared.

By default an `A5` frame is compared only as the class `a5`. Its bytes follow a per-second counter whose phase depends on when the camera booted, and the scramble is not decoded yet. The diff therefore checks that the stream is there and keeps its cadence, not what it carries. `--strict-a5` compares bytes 0–6. Combine it with `--ignore-edges` for two captures whose counters run in step. Other frames (Pelco commands, PTZ replies) are always compared byte for byte.

## Protocol notes (stock firmware, camera → PTZ)

Status is in `src/uart_bridge/codec.py`.
- **Line:** 115200 8N1, 8-byte frames starting with `A5`, sent every 50 ms (20/s) even when idle.
- **Byte 1:** a counter XOR `0x25` that increments once per second.
- **Byte 2:** `9E`.
- **Bytes 3–6:** scrambled together with the counter, not decoded yet. The two low bits of byte 6 flip for single frames.
- **Byte 7:** changes in every frame.

When the camera receives zoom reports, the low byte of its `A5` frames changes every frame, until about 2 s after the zoom stops. That looks like the camera's autofocus loop.

The PTZ board **does not answer `A5` frames**, not even xm-uart's `init[]`. It does answer the XM Pelco-D variant `C5|FF addr c1 c2 d1 d2 ck 5C`, where `ck = sum % 100`. While the zoom moves, it replies with `EF 01 <type> <len> <payload>`. For type 00 the payload is `04 03 2F 2E "X1.6 "`, a zoom-ratio report. An idle camera sends only `A5` frames, so an idle capture has no `p2c` traffic, and that is expected.
