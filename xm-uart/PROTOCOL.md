# XM camera ↔ lens board UART protocol

This spec covers the serial link between a Xiongmai IP camera main board and its zoom/focus (AF) lens board, which is what `xm-uart` drives.

Everything here was measured, not taken from a datasheet. The camera was cut out of the link with [`uart-bridge`](../uart-bridge/): the camera board's UART and the lens board's UART each went to a USB adapter on a host, and the host forwarded and logged every byte in both directions. The stock camera firmware was then driven through its own DVRIP API (`OPPTZControl` via python-dvr). Physical effects were measured on its RTSP video: sharpness via ffmpeg `blurdetect`, brightness via `signalstats`, and the zoom OSD.

Each statement is marked with how well it's established:
- **verified**: observed in the captures listed at the end.
- **stock**: this is what the stock firmware sends, and the board's reaction was observed.
- **unverified**: inferred, or copied from older code, and not seen on the wire.

Test rig: XM **HI3516EV300_85H50AI** camera, stock firmware V5.00.R02.000529B2, camera setting `Uart.PTZ` = `PELCOD`, 115200 8N1, device address 1, and a lens with motorised zoom and focus but no pan/tilt head.

## Physical layer

| | |
|---|---|
| Levels | TTL UART, full duplex |
| Format | **115200 baud, 8N1**, no flow control (**verified**) |
| Camera side | `/dev/ttyAMA0` on the camera SoC; the system getty must be off on it |

## Traffic at a glance

```
camera ──► board   A5 xx 9E xx xx xx xx xx   8 B, every 50 ms, always (camera heartbeat/state stream)
camera ──► board   C5 01 c1 c2 d1 d2 ck 5C   8 B, once per user action, replaces one A5 slot
board  ──► camera  EF 01 tt ll <ll bytes>    zoom-ratio reports while zooming, then a blank one
```

The board never answers a command directly. The only thing it sends is zoom reports while the zoom moves (**verified**).

## Command frame (camera → board)

```
byte  0     1     2     3     4      5      6      7
      C5    addr  cmd1  cmd2  data1  data2  cksum  5C
```

| Field | Value | Board behaviour |
|---|---|---|
| sync | `C5` | **Must be `C5`.** `FF` (the standard Pelco-D sync, xm-uart's old `AUTO_FOCUS`-off variant) and `A0` are ignored (**verified**). |
| addr | `01` in the stock config | **Ignored.** 0, 1, 2 and FF all work (**verified**). |
| cmd1, cmd2 | bit fields, see below | |
| data1 / data2 | pan / tilt speed, 0–0x3F | **No effect on zoom speed** (00, 3F and 40 behave the same) (**verified**). Stock sends `27` at DVRIP Step 5. |
| cksum | `(addr+cmd1+cmd2+data1+data2) % 256` | This is what the stock firmware sends (**stock**; e.g. `c5 01 00 80 00 00 81 5c`). **The board does not check it**, for zoom or focus frames (**verified**; see *Checksum on focus frames*). |
| trailer | `5C` | Sent by the stock firmware. **Not checked** (**verified**), but the frame is always 8 bytes. |

### Framing rule, and the trap in it (verified)

The board's receiver treats **`A5` or `C5` as the start of an 8-byte frame** and takes the next 7 bytes as the rest of it. **There is no inter-byte timeout:** even a 1 s gap doesn't reset the parser. Other bytes outside a frame are skipped.

A stray or truncated frame therefore swallows the first 7 bytes of the **next** command, which is silently lost:

| Sent | Result (capture) |
|---|---|
| `C5…` zoom-in, then stop | zooms (`e3e-control`) |
| lone `A5`, 0.2 s pause, then zoom-in | **nothing**: the zoom-in completed the `A5` frame (`e3e-lone-a5-then-zoom`) |
| lone `A5`, **1 s** pause, then zoom-in | **nothing** |
| lone `C5`, then zoom-in | **nothing** |
| lone `5A`, then zoom-in | zooms (non-sync byte skipped) |
| complete 8-byte `A5` frame, then zoom-in | zooms |

A sender must write every frame whole, and must never start or stop a stream in the middle of a frame.

### Checksum on focus frames

An earlier xm-uart fix (d9cbf71) concluded that this same 85H50AI board discards focus frames with a wrong checksum. That doesn't reproduce.

Focus pulses were stepped in one direction only, after taking up the backlash, with a stop frame as a null control. Every checksum value moved the focus, including the `01` that the old bug sent. Blur is ffmpeg `blurdetect` on the whole frame (lower is sharper); its values pass through the best focus point and then rise.

| Frame (0.25 s pulse) | Δ blur |
|---|---|
| stop frame (control) | −0.01, −0.01, −0.10, +0.02 |
| cmd2 `80`, ck `81` (stock, % 256) | −1.48, later +4.35 |
| cmd2 `80`, ck `1D` (% 100) | +0.39 (at the sharpest point) |
| cmd2 `80`, ck `01` (old bug) | +1.71 |
| cmd2 `80`, ck `00` | +4.35 |
| cmd1 `01`, ck `02` / `01` / `00` / `81` | −0.30 / −0.97 / −1.40 / −3.70 |

A first pulse after a direction reversal often moves the image very little, because of backlash (see *Timing*). That is a plausible reason focus looked unresponsive before.

### Command bits

The stock firmware's DVRIP command names are the reference here. Pelco-D's own names for the same bits are given alongside because two of them disagree.

| DVRIP command (stock) | cmd1 | cmd2 | data1 | data2 | Pelco-D name of the bit | Effect on this rig |
|---|---|---|---|---|---|---|
| ZoomTile | 00 | **20** | 00 | 00 | zoom tele | zoom in; reports `X…` rising (**verified**) |
| ZoomWide | 00 | **40** | 00 | 00 | zoom wide | zoom out (**verified**) |
| FocusNear | 00 | **80** | 00 | 00 | *focus far* | focus moves (**verified**); direction, see note |
| FocusFar | **01** | 00 | 00 | 00 | *focus near* | focus moves the other way (**verified**) |
| IrisLarge | **02** | 00 | 00 | 00 | iris open | no visible effect (luma unchanged) |
| IrisSmall | **04** | 00 | 00 | 00 | iris close | no visible effect |
| DirectionLeft | 00 | **04** | 27 | 00 | pan left | none, no pan motor |
| DirectionRight | 00 | **02** | 27 | 00 | pan right | none |
| DirectionUp | 00 | **08** | 00 | 27 | tilt up | none, no tilt motor |
| DirectionDown | 00 | **10** | 00 | 27 | tilt down | none |
| diagonals | 00 | OR of the above, e.g. LeftUp `0C`, RightDown `12` | 27 | 27 | | none |
| *(any stop)* | 00 | 00 | 00 | 00 | stop | stops zoom/focus (**verified**) |

**Focus naming.** The stock firmware sends cmd2 `0x80` for "FocusNear", but Pelco-D calls that bit "focus far". The physical direction could not be settled on the test scene. Both directions defocus it symmetrically, and both end stops are equally blurred at wide zoom. So this spec, and `xm-uart`, follow the stock firmware's naming. If a scene with a clear near/far target shows otherwise, swap the two labels in `xm-uart/main.c`, not the bytes the stock firmware is known to send.

### Timing (stock)

- **One frame per action.** A move is a single start frame, and stopping is a single stop frame. Nothing is repeated.
- A command goes out about 150–250 ms after the DVRIP request and takes the place of the next `A5` slot, so the 50 ms cadence is kept.
- The zoom keeps moving until a stop frame arrives. Stock DVRIP sends stop when the UI button is released.
- **Focus has backlash.** After a run of 0.2 s focus steps in one direction, one 0.2 s step back did not return to the previous sharpness (blur 3.24, then 3.98 after the next step, then 3.94 after stepping back). Short reversals partly go into lost motion, so any autofocus built on these commands must re-measure after every reversal.

### Extended commands (cmd2 bit 0 set)

The stock firmware sends Pelco-D extended commands for presets (**stock**). **This board ignores them**: no motion and no reply (**verified**).

| DVRIP | Frame | Board |
|---|---|---|
| SetPreset *n* | `C5 01 00 03 00 n ck 5C` | ignored |
| GotoPreset *n* | `C5 01 00 07 00 n ck 5C` | ignored: the zoom did not return |
| ClearPreset *n* | `C5 01 00 05 00 n ck 5C` | ignored |
| *(not sent by stock)* | zoom speed `25`, query pan `51` / tilt `53` / zoom `55` / `61` | no reply, no effect |

Presets therefore have to be implemented on the camera side, for example as zoom/focus positions tracked from the zoom reports.

## Replies (board → camera)

```
EF 01 type len payload[len]
```

| type | len | payload | Meaning |
|---|---|---|---|
| `00` | `09` | `04 03 2F 2E` + ASCII `"X1.4 "` | **Zoom report**, sent about every 225 ms while the zoom moves plus one more after stop. The camera draws it on the video OSD (bottom right). (**verified**) |
| `00` | `0A` | `04 03 2F 2E` + six spaces | Sent about 6.7 s after the last zoom report (**verified**). It reads like an "erase the ratio text" message, but the camera's OSD went on showing the last ratio in later snapshots, so what the camera does with it is **unverified**. |
| `02` | `01` | `01` night / `00` day | from the old xm-uart; **unverified**, never seen during these tests |

The camera keeps showing the **last report it received**. Commands sent to the board with the camera cut out of the link leave the camera's OSD, and its idea of the zoom, stale: the OSD read X2.5 over a lens that was back at X1.2. It corrects itself at the next zoom movement it sees.

The `04 03 2F 2E` prefix stayed the same at every zoom position seen (X1.1–X3.8). It looks like an OSD position and attribute header rather than lens data.

Replies arrive split across reads, often one byte at a time, so a receiver has to buffer until `4 + len` bytes are present.

## The camera's A5 stream (camera → board)

The camera sends `A5 xx 9E xx xx xx xx xx` every 50 ms from boot, whether or not anything is connected.

| byte | Meaning |
|---|---|
| 0 | `A5` |
| 1 | `counter ^ 0x25`; the counter increments once per second (**verified**: consecutive values differ by exactly `n ^ (n+1)`) |
| 2 | `9E`, constant |
| 3–6 | scrambled together with the counter, not decoded. The two low bits of byte 6 change every frame for about 2 s after the camera receives zoom reports. |
| 7 | different in every frame, even with bytes 0–6 unchanged; not a sum or XOR of bytes 0–6 |

**The board shows no observable reaction to it.** It never answers, zoom and focus stay put, and a defocused lens is **not** refocused while the stream runs (15 s watched). Focus is nevertheless held through a zoom (sharpness unchanged from X2.4 to X3.2), which points to zoom/focus tracking inside the board. Whether the stream carries anything the board uses is open. The old xm-uart sent one such frame (`a5 7b 9e f0 ef ee e0 f4`) as an "init". That frame is from this stream, and the board accepts it as a complete frame and ignores it, so it has been removed.

## Where the old xm-uart went wrong

| Old behaviour | Effect | Fixed to |
|---|---|---|
| checksum `% 100` | different from stock for any sum ≥ 100, e.g. focus `…80 00 00 1D 5C` instead of `…81 5C`; harmless only because the board ignores it | `% 256`, as stock sends it |
| `h` "Pan left" sent cmd2 `02`, `l` "Pan right" sent `04` | pan directions swapped relative to stock and Pelco-D | left `04`, right `02` |
| `z` "Focus near" sent cmd1 `01` | the opposite of what the stock firmware calls FocusNear | `z` sends cmd2 `80`, `x` sends cmd1 `01` (see *Focus naming*) |
| `AUTO_FOCUS` compile switch for sync `C5` / `FF` | `FF` frames are ignored by this board | fixed `C5` |
| replies parsed per `read()`, looking for `'X'` in byte 0 | zoom reports never parsed; printed as scattered hex dumps and fragments like `Magnification: X1` | `EF 01 type len` framing with a buffer; prints `Zoom X1.4`, `Zoom report blank`, day/night |
| stop after `while(1)` was unreachable; the terminal was never restored | Ctrl-C left the lens moving and the terminal without echo | SIGINT/SIGTERM/SIGHUP, `q` and stdin EOF all send stop and restore the terminal (`e2-exit-paths`) |
| `write()` result ignored | a short write would desync the board (see framing rule) | `write_all()` retries until the whole frame is out |
| `init[]` `A5` frame at start | no effect | removed |

## Captures

These are in [`captures/`](captures/), in the `uart-bridge` JSONL format. Replay one with `uart-bridge decode <file>`, or compare an implementation against the stock firmware with `uart-bridge diff captures/e1-stock.jsonl <yours>`.

| File | What |
|---|---|
| `e1-stock.jsonl` | stock firmware driven through every DVRIP PTZ command (zoom, focus, iris, 6 directions, set/goto/clear preset), with marks |
| `e2-xmuart-orig.jsonl` | the old xm-uart through the bridge (pty) against the real board |
| `e2-xmuart-new.jsonl` | the fixed xm-uart, same key script |
| `e2-exit-paths.jsonl` | fixed xm-uart stopped by SIGINT and by `q` in the middle of a zoom |
| `e3e-control.jsonl` | a plain zoom-in and stop (host as camera) |
| `e3e-lone-a5-then-zoom.jsonl` | the parser trap: a lone `A5` byte, then a zoom-in that never happens |
