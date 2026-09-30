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
uv run uart-bridge inject --replay stock.jsonl  # host acts as camera (stop the bridge first)
uv run uart-bridge inject --frame a52e9eea2662efae --rate 20 --duration 2
```

- If stdin is a terminal while `bridge` runs, each line you type (then Enter) is stored in the log as a timestamped mark, for example `pan left pressed`.
- `bridge` and `inject` open the ports exclusively and set the FTDI latency timer to 1 ms through sysfs, using `--latency` (0 leaves it alone). The default is 16 ms, which makes every timestamp late by up to 16 ms.

### Two identical FT232R adapters

Both FT232R adapters report the same serial number (`A5069RR4`), so `/dev/serial/by-id` shows only one of them. When `ttyUSBn` numbering matters, use the by-path names:
- camera: `/dev/serial/by-path/pci-0000:00:14.0-usb-0:3:1.0-port0`
- PTZ board: `/dev/serial/by-path/pci-0000:00:14.0-usb-0:4:1.0-port0`

## Capture format (JSONL)

The first line is a header: `{"type":"header","version":1,"wall":...,"git":...,"mode":...}`, followed by the ports, baud rates and your note. After that there is one record per `read()`:

```
{"t": <ns since start>, "d": "c2p" | "p2c", "x": "<hex>"}
{"t": <ns since start>, "d": "mark", "note": "..."}
```

`c2p` is camera → PTZ board. Record boundaries are read boundaries, not frame boundaries.

## What `diff` compares

Each direction is framed and run-length encoded by `codec.key()`. The segment sequences are then aligned with difflib. The diff reports:
- changed, missing or extra segments;
- segments whose repeat count differs by more than `--tolerance` frames, which is a timing difference.

Unmatched runs at the very start or end of a capture only reflect where the capture was cut, so the diff ignores them unless you pass `--keep-edges`.

For now, `key()` is bytes 0–6 of the frame. The body still contains the counter, so two captures only line up when both counters run in step. Once the body scramble is decoded, `key()` should drop the counter.

## Protocol notes (stock firmware, camera → PTZ)

Status is in `src/uart_bridge/codec.py`.
- **Line:** 115200 8N1, 8-byte frames starting with `A5`, sent every 50 ms (20/s) even when idle.
- **Byte 1:** a counter XOR `0x25` that increments once per second.
- **Byte 2:** `9E`.
- **Bytes 3–6:** scrambled together with the counter, not decoded yet. The two low bits of byte 6 flip for single frames.
- **Byte 7:** changes in every frame.

The PTZ board sent nothing back during a 60 s idle bridge run.
