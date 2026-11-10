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

- `bridge --cam pty` creates a pseudo terminal in place of the camera port and prints its path once forwarding has started. A local program under test (for example `xm-uart-motors-host -d /dev/pts/N`) then plays the camera against the real PTZ board, and every byte is logged. `scripts/xm_uart_audit.py tool` does this with a scripted key sequence.
- Ports can also be pyserial URLs such as `socket://host:9000`, for example `inject --ptz socket://cam:9000 --replay stock.jsonl` to replay a recording to another camera's lens board through `xm-uart -l`.
- `bridge --tee URL` also sends every **whole** camera frame to URL, logged as `c2t`, and logs what comes back as `t2c`. It never forwards those replies to the camera. The boot console and other junk are not teed.
- `uart-bridge boards A [B]` compares where two lens boards settled: A's `p2c` against A's `t2c`, or against B's `p2c`. It exits 1 on a mismatch beyond `--tolerance`.
- [TWIN.md](TWIN.md) is the full procedure for comparing two cameras' lens boards this way.
- `bridge --mute-cam` logs the camera's bytes but forwards none of them, so the PTZ board hears nothing from the camera. That separates what the board does on its own from what the camera makes it do; it showed that the lens board re-homes by itself at power-up (see `xm-uart/PROTOCOL.md`, *Power-up behaviour*).
- The bridge never leaves a partial frame on the PTZ wire. The XM board starts an 8-byte frame at any `A5`/`C5` byte and has no inter-byte timeout (see `xm-uart/PROTOCOL.md`), so a stray byte silently eats the next command. Forwarding of camera bytes therefore starts only after a quiet gap on the camera line; bytes before it are logged as a mark, not forwarded. On exit, the bridge finishes forwarding the frame in flight.
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

By default an `A5` frame is compared only as the class `a5`. It carries the camera's clock and live image statistics (decoded in `codec.py`; see `xm-uart/PROTOCOL.md`), which differ between any two captures. The diff therefore checks that the stream is there and keeps its cadence, not what it carries. `--strict-a5` compares the decoded seconds, day/night and gain, never the focus value, which changes every frame. Combine it with `--ignore-edges` for two captures whose clocks run in step. Other frames (Pelco commands, PTZ replies) are always compared byte for byte.

## Protocol

The XM camera ↔ lens board protocol is specified in [`xm-uart/PROTOCOL.md`](../xm-uart/PROTOCOL.md), measured with this tool. `codec.py` decodes what that spec establishes. The recordings behind it are in `xm-uart/captures/`; `e1-stock.jsonl` is the stock firmware reference to diff an implementation against.

## Reproducing the measurements

`scripts/xm_uart_audit.py` re-runs the experiments on a rig:
- `stock`: the stock firmware, driven through DVRIP with python-dvr;
- `accept`: which frame variants the board acts on, and the partial-frame trap;
- `tool <binary>`: a program under test through the pty;
- `focus`: RTSP sharpness;
- `focusdir`: which focus bit moves focus nearer, by sweeping focus past near and far targets whose depth order is known from occlusion;
- `restore` / `refocus`: put the lens back afterwards;
- `sync`: drive two lenses (the second one through `--openipc socket://…`) into the wide end stop;
- `twin`: a live tee of a stock DVRIP session to a second board, with sharpness on both;
- `focusoffset`: where the sharpest focus is relative to a board's tracked focus.

Run `uv run scripts/xm_uart_audit.py -h` for the options.

`scripts/xm_tracking.py` measures the board's own zoom tracking on one or more
boards at once (`--board NAME PTZ RTSP`, repeatable): focus while and after a zoom,
where it settles against the sharpest point, combined and interleaved zoom/focus
frames, backlash, and whether the camera's `A5` stream feeds an autofocus loop. The
results are in `xm-uart/PROTOCOL.md`, "Zoom tracking inside the board".

`scripts/dvrip_twin.py` drives the stock camera and an OpenIPC sibling with the
same DVRIP PTZ commands through python-dvr, in lockstep. At each zoom level it
compares the zoom reached, when each picture stopped changing, how sharp it
settled (with `--reference`, against each camera's own best focus), and when
majestic-af's after-zoom pass finished. It also records where each lens board's own
tracking left focus 1.5–2.8 s after the stop, before majestic-af's pass starts. The
default levels zoom in from the wide stop; `--levels out-X4.0 … out-X1.0` zoom out
from the tele end instead. It also checks that a manual focus nudge is not followed
by an autofocus pass. The OpenIPC camera needs majestic with
DVRIP PTZ (netip `OPPTZControl`); see TWIN.md, "Driving both cameras over DVRIP".
