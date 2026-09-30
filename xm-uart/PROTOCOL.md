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
| Shared with the console | the camera's **U-Boot console goes out on this same UART** at boot ("System startup", "U-Boot 2016.11…", "Starting kernel…"), so the lens board receives it. It is mostly ASCII. In three power cycles the window from power loss to the resumed `A5` stream held about 2074 bytes, with 3–5 non-ASCII bytes of power-up line noise (e.g. `8F 92 C6`) and never an `A5` or `C5` (**verified**, `e13`–`e15`). If a noise byte ever were a sync byte, it would only swallow the first 7 bytes of the resuming `A5` stream: the camera sends no commands at that point. |

## Traffic at a glance

```
camera ──► board   A5 xx xx xx xx xx xx xx   8 B, every 50 ms, always (camera heartbeat/state stream)
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
| FocusNear | 00 | **80** | 00 | 00 | *focus far* | **focus nearer** (**verified**, see *Focus direction*) |
| FocusFar | **01** | 00 | 00 | 00 | *focus near* | **focus farther** (**verified**) |
| IrisLarge | **02** | 00 | 00 | 00 | iris open | no visible effect (luma unchanged) |
| IrisSmall | **04** | 00 | 00 | 00 | iris close | no visible effect |
| DirectionLeft | 00 | **04** | 27 | 00 | pan left | none, no pan motor |
| DirectionRight | 00 | **02** | 27 | 00 | pan right | none |
| DirectionUp | 00 | **08** | 00 | 27 | tilt up | none, no tilt motor |
| DirectionDown | 00 | **10** | 00 | 27 | tilt down | none |
| diagonals | 00 | OR of the above, e.g. LeftUp `0C`, RightDown `12` | 27 | 27 | | none |
| *(any stop)* | 00 | 00 | 00 | 00 | stop | stops zoom/focus (**verified**) |

**Focus direction (verified).** On this board **cmd2 `0x80` moves focus nearer and cmd1 `0x01` moves it farther.** That is what the stock firmware's names say (FocusNear / FocusFar), and the **opposite of the Pelco-D names** for those bits.

How it was measured: at X2.4, focus was swept across its whole range in 0.1 s steps, one direction per sweep, after taking up backlash. At each step, two RTSP frames gave a gradient-energy (Tenengrad) sharpness for several targets. The depth order of the targets is certain from occlusion alone: the chair in the foreground covers part of the doorway, and the far room's door is only visible through that doorway. Three Siemens stars on the wall were measured as well.

| Sweep | far room door / door leaf / stars peak at step | near chair peaks at step | near − far |
|---|---|---|---|
| cmd2 `80` | 15.6 – 15.8 | 16.9 | **+1.2** |
| cmd1 `01` (reverse) | 19.5 – 19.8 | 18.6 | **−1.0** |
| cmd2 `80` (repeat) | 17.4 – 17.5 | 18.4 | **+1.0** |

Sweeping with cmd2 `80`, the distant targets sharpen first and the near chair after them: focus is moving nearer. The offset changes sign when the sweep is reversed, so it isn't a timing artefact. An earlier, independent run gave +0.9, −1.1, +0.9. The far targets peak together: beyond about 3 m everything is close to the hyperfocal distance at this focal length. The chair, at about 1 m, differs roughly five times more in 1/distance. Data: `captures/focus-direction-sweeps.json`. Reproduce with `uart-bridge/scripts/xm_uart_audit.py focusdir`.

### Timing (stock)

- **One frame per action.** A move is a single start frame, and stopping is a single stop frame. Nothing is repeated.
- A command goes out about 150–250 ms after the DVRIP request and takes the place of the next `A5` slot, so the 50 ms cadence is kept.
- The zoom keeps moving until a stop frame arrives. Stock DVRIP sends stop when the UI button is released.
- **Zoom range is X1.0 – X5.0 on this lens.** At an end stop a zoom command produces no motion and **no reports**, so a silent board at the end of travel is normal, not a fault.
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

The camera sends `A5 xx xx xx xx xx xx xx` every 50 ms from boot, whether or not anything is connected.

| byte | Meaning |
|---|---|
| 0 | `A5` |
| 1 | `counter ^ 0x25`; the counter increments once per second (**verified**: consecutive values differ by exactly `n ^ (n+1)`) |
| 2 | **data, not a constant**: `9E` for hours, then `92` with no reboot in between; it cycles through `9F 9D 93 90 91 96` in the first minute after boot, then settles (**verified**, `e13`) |
| 3–6 | scrambled together with the counter, not decoded. The two low bits of byte 6 change every frame for about 2 s after the camera receives zoom reports. |
| 7 | different in every frame, even with bytes 0–6 unchanged; not a sum or XOR of bytes 0–6 |

**The board shows no observable reaction to it.** It never answers, zoom and focus stay put, and a defocused lens is **not** refocused while the stream runs (15 s watched). Focus is nevertheless held through a zoom (sharpness unchanged from X2.4 to X3.2), which points to zoom/focus tracking inside the board. Whether the stream carries anything the board uses is open. The old xm-uart sent one such frame (`a5 7b 9e f0 ef ee e0 f4`) as an "init". That frame is from this stream, and the board accepts it as a complete frame and ignores it, so it has been removed.

## Power-up behaviour

**The lens board re-homes on its own every time it is powered, and then returns the zoom to where it was before power was lost.** The camera sends no `C5` command during boot at all.

Measured by power-cycling the camera over PoE with the bridge logging throughout and RTSP video recorded from the moment it came up. The lens board is powered from the camera, so it loses power as well. Times are from the start of each capture; power returns at ≈17 s.

| Time | Camera → board | Lens (from the video) |
|---|---|---|
| 17.9 – 20 s | U-Boot console text | (no video yet) |
| 24.7 s | `A5` stream resumes, 4.7 s after "Starting kernel"; byte 2 churns for about a minute | (no video yet) |
| 35 s | RTSP video available | heavily defocused |
| 40 – 45 s | | focus sweeps through sharp and out again |
| ≈51 s | | **zoom driven to the wide end** (X1.0) |
| 52 – 64 s | | parked at wide, defocused |
| 66 – 68 s | | refocused at wide |
| ≈70 s | board sends **one** zoom report: the pre-power-off value | **zoom returns to the last position**, then refocuses |
| after 80 s | | stable and sharp |

The restored positions: X3.0 → X3.0 (`e14`), X2.0 → X2.0 (`e15`), and X1.2 → X1.1 (`e13`). Focus comes back sharp even when the lens was deliberately defocused before power-off (blur 12.5 → 4.8), because the board re-derives focus from zoom tracking.

**The homing is done by the board itself.** In `e15` the bridge ran with `--mute-cam`, so the board received nothing from the camera, and the same homing sequence ran and the zoom still returned to its pre-power-off X2.0. The board must therefore store the last zoom position itself.

**Camera traffic affects when the zoom is restored.** With the camera connected, the return and its report came at ≈70 s (68.2 s and 70.5 s in two runs). With the camera muted, it came at 119.2 s. So the board seems to wait for something from the camera's `A5` stream, with a timeout fallback. This rests on one muted run and is **unverified**.

The lens column comes from RTSP frames sampled across each run (a 4×4 contact sheet per run, not committed because it shows the lab room). The field of view gives the zoom: wide at ≈51–64 s, back at the pre-power-off framing by ≈70 s, or 119 s when muted. Tenengrad sharpness on the centre of the frame gives the focus: peaks at 42 s and 51 s, a steady defocused plateau at 52–63 s, and a climb to a stable maximum by ≈80 s.

A replacement for the camera firmware doesn't need to home or restore the lens at boot, because the board does both. It should expect a single zoom report about a minute after power-up, and must not send motion commands in the meantime.

## Second board: network replay (two 85H50AI cameras)

A second 85H50AI camera, running OpenIPC, was fed the reference camera's exact traffic, `A5` stream included, through `bridge --tee` → `xm-uart -l`. It was also fed a recorded stock session through `inject --ptz socket://…`. The procedure is in `uart-bridge/TWIN.md`. Findings:

- **Zoom is identical board to board.** Live, over the stock firmware's commands, both boards settled at X2.2, X3.4/3.5, X2.8 and X1.0, within 0.1 (`twin-live-tee`). Replaying the stored `e1-stock` session from the same starting zoom gave X1.7, X1.2, X1.7 on both, with no difference (`replay-e1-to-openipc`). From a different starting zoom it misses by 0.3: zoom ratio isn't linear in steps.
- **Focus at X1.0 is identical with a healthy lens.** After both boards re-homed and synced, each tracked focus was within ~0.1 s of drive of its camera's sharpest point: +0.02 s and +0.07 s.
- **A degraded lens shows up as unreachable focus.** The OpenIPC unit's original lens, several years in service, was blurred at every zoom. It was sharpest at its far focus stop and still improving there. Its travel matched the healthy lens (≈20 s of drive stop to stop), but its in-focus point lay beyond that travel. Replacing the lens fixed it. So the fault was in the lens, not the board or the protocol.
- **Board limits count steps, lenses don't report back.** A replacement lens that did not move physically still produced normal zoom reports and a "silent" end stop. Only the video showed it: a narrower field of view at "X1.0". Check the picture, not only the reports.
- **The board refocuses after a zoom by itself; the `A5` stream plays no part (verified).** The stock firmware's zoom-in (1.5 s) was recorded with the camera connected, then replayed to the same vendor board from the same start (the wide stop) twice: once with the recorded `A5` frames and once with **only the two `C5` frames**, zoom and stop. The sharpness after the stop climbed the same way in all three runs, about 870 → 980 → 1000 over ~4 s, reaching 998, 1003 and 1006 (`captures/a5-burst-*.jsonl.gz`). 1445 idle `A5` frames teed to the second board also changed nothing.
- **Focus diverges between boards only through their starting focus state.** In the live twin session the zoom matched, but each lens was sharp at some zooms and not others. The vendor lens started that run out of focus at X1.0 (sharpness 10), the OpenIPC lens in focus. Tracking carries a board's focus offset through every zoom move, so start both from a re-home and `sync` (`uart-bridge/TWIN.md`).
- **OpenIPC's majestic must release the lens UART** (`isp.autofocus.enabled: false`). In manual mode it still keeps the tty open and takes half of the board's replies.

## Where the old xm-uart went wrong

| Old behaviour | Effect | Fixed to |
|---|---|---|
| checksum `% 100` | different from stock for any sum ≥ 100, e.g. focus `…80 00 00 1D 5C` instead of `…81 5C`; harmless only because the board ignores it | `% 256`, as stock sends it |
| `h` "Pan left" sent cmd2 `02`, `l` "Pan right" sent `04` | pan directions swapped relative to stock and Pelco-D | left `04`, right `02` |
| `z` "Focus near" sent cmd1 `01` | focused **farther**, the opposite of its label (verified on video) | `z` sends cmd2 `80` (nearer), `x` sends cmd1 `01` (farther); see *Focus direction* |
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
| `e13-powercycle.jsonl.gz` | camera power-cycled over PoE (lens at X1.2); includes the U-Boot console text and the post-boot `A5` churn |
| `e14-powercycle.jsonl.gz` | power cycle from X3.0 defocused; zoom returns to X3.0 at 70.5 s |
| `e15-powercycle-muted.jsonl.gz` | power cycle from X2.0 with `--mute-cam`, so the board hears nothing from the camera; it still homes and returns to X2.0 at 119.2 s. The camera's output is logged as `c2m`, for "muted, never sent to the board". |
| `twin-live-tee.jsonl.gz` | live stock DVRIP session on the reference camera, teed to the OpenIPC board: `p2c` is the reference board, `t2c` the second board; checkpoints carry both cameras' sharpness |
| `replay-e1-to-openipc.jsonl.gz` | `e1-stock` replayed to the OpenIPC board through `xm-uart -l`, starting from the same zoom (X1.2); `boards e1-stock.jsonl replay-e1-to-openipc.jsonl.gz` matches |
| `a5-burst-record.jsonl.gz` | stock zoom-in (DVRIP ZoomTile 1.5 s) from the wide stop, camera connected: the `C5` commands plus the live `A5` stream |
| `a5-burst-with-a5.jsonl.gz`, `a5-burst-c5-only.jsonl.gz` | the recording replayed to the same board from the same start, with and without its `A5` frames; the post-zoom refocus is identical |
| `focus-direction-sweeps.json` | per-step sharpness of the near, far and star targets for the three focus sweeps |
