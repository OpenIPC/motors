# anjoy-motor

Drive the zoom/focus/iris lens motors and the pan/tilt head of Anjoy AF
camera modules (SigmaStar-based boards) over the motor-MCU UART, read back
the MCU's live position reports, and servo the zoom to an absolute optical
multiple.

Verified on an **MTF45-4G_AF** (FW V3.4.5.6), `/dev/ttyS2`, **57600 8N1** —
not 115200: at 115200 the MCU is completely deaf (rx counter stays flat,
zero ACKs); at 57600 it ACKs and streams reports. The vendor rate is also
baked into `comm_server` as the termios constant `B57600|CS8|CREAD|CLOCAL`.

The wire format was recovered from the vendor's own `comm_server` with an
LD_PRELOAD `write()` logger while driving the web PTZ verbs, then verified
standalone: handshake + state stream + a command frame moves the real
motor with every vendor daemon dead.

## The session requirement (the one that killed all early attempts)

The MCU ignores bare command frames. What it demands is a live session:

```c
/* state frame, 16 bytes, sent every ~280 ms, idle and during motion */
51 01 04 78 01 0d 59 56 07 20 2f 00 20 00 00 CK
                                                 CK = sum of bytes 0..14 (INCLUDING 0x51)
```

* bytes 6..9 are live lens telemetry (they ramp while a motor moves),
  byte 10 drifts slowly — a frame frozen from a capture works fine;
* the MCU needs the stream for a few **seconds** before it answers at
  all: prime with ~14 frames (~4 s, vendor cold-start pacing, 300-500 ms
  handshake gaps) and ACKs + reports flow; two frames buy total silence.
* optional one-way boot handshake before priming ( vendor sends it, the
  MCU never replies): `END`×2, `VER`, `EXVER`×2, ASCII + 8-bit payload sum
  (`END` = `45 4e 44 d7`, `VER` = `56 45 52 ed`,
  `EXVER` = `45 58 56 45 52 8a`), plus a poll frame
  `51 01 04 79 91 01 00 61 00` every ~10-30 s;
* without this stream, `tx` on `/proc/tty/driver/ms_uart` climbs but `rx`
  never twitches — that flat rx is the signature of a rejected session.

## Command frames (host → MCU)

7-byte Pelco-D-style frames, address `0x01`,
`CK = (01 + C1 + C2 + D1 + D2) & 0xff`:

| Verb | C1 | C2 | D1 | D2 | Frame |
|---|---|---|---|---|---|
| zoom tele | 00 | 20 | 14 | 14 | `FF 01 00 20 14 14 49` |
| zoom wide | 00 | 40 | 14 | 14 | `FF 01 00 40 14 14 69` |
| focus near | 01 | 00 | 30 | 30 | `FF 01 01 00 30 30 62` |
| focus far | 00 | 80 | 30 | 30 | `FF 01 00 80 30 30 E1` |
| iris open | 02 | 00 | 30 | 30 | `FF 01 02 00 30 30 63` |
| iris close | 04 | 00 | 30 | 30 | `FF 01 04 00 30 30 65` |
| head up | 00 | 08 | 00 | 14 | `FF 01 00 08 00 14 1D` |
| head down | 00 | 10 | 00 | 14 | `FF 01 00 10 00 14 25` |
| head left | 00 | 04 | 14 | 00 | `FF 01 00 04 14 00 19` |
| head right | 00 | 02 | 14 | 00 | `FF 01 00 02 14 00 17` |
| stop | 00 | 00 | 00 | 00 | `FF 01 00 00 00 00 01` |

* D1/D2 are speed bytes; the vendor maps UI speed 1..10 to `5*v+5`
  (`1→0x0a`, `3→0x14`, `10→0x37`). Focus/iris verbs always carry `30 30`.
* C2 direction bits OR for diagonals (vendor emitted `0x0C` = left+up).
* Motion starts on the verb frame and runs until a stop frame (all-zero
  direction fields) — one stop frame is enough, the motor obeyed ~20 ms
  after it left the wire (measured via report cutoff).

## Status reports (MCU → host)

While a motor is actually moving (pushing against an endstop counts), the
MCU continuously sends 17-byte position reports at ~150-300 ms, with each
position typically seen twice:

```
51 01 04 78 01 0A 0A 10 3B ZZ ZZ 21 3B FF FF 3B CK
                               ^^^^^ zoom     ^^^^^ focus
```

* `CK` (byte 16) = 8-bit sum of bytes 0..15.
* **zz (bytes 9-10) decode as DECIMAL DIGITS**: byte 9 = tens (zero is
  transmitted as `0x3B`), byte 10 = units. Scale is **1..30, wide →
  tele**: wide endstop reads `3B 01` (=1), tele endstop `03 00` (=30).
  The old "16-bit big-endian counter" theory is dead — `3B 3B 08` is
  position 8, not `0x3B08`.
* bytes 13-14 = focus position, raw and stable during zoom moves
  (`14 0F` observed). All-`3B` garbage frames appear during link
  shutdown; decode them to zoom > 30 and the range guard drops them.
* single-byte ACKs sprinkle through the stream (`21`, `78`, `3b`…);
  they are not position data.

Decode live with `-j` (one JSON object per report on stdout, TX
diagnostics on stderr — the stream is clean for web-UI consumption):

```
{"zoom_pos":8,"focus_pos":5135,"flags":"040A","raw":"51010478010A0A103B3B08213B140F3B2B"}
```

## Absolute zoom (verb `T`) — verified closed loop

`-m <multiple>` maps the target onto the 4.4×..45× geometric lens curve
(`multiple(pos) = 4.4 * (45/4.4)^((pos-1)/29)`, exact at the endstops,
midpoints uncalibrated), `-p <pos>` aims at a position 1..30 directly.
The loop then:

1. drives fast (speed `0x14`) toward the target — first move is blind
   (reports only come once a motor moves), direction is corrected on the
   first report; coverage ≈ 2 pos/s;
2. stops on the first crossing report (~0 coast observed at `0x14`);
3. settles 700 ms, then runs **one** slow (`0x0A`) closed-loop correction
   approach if the residual delta is nonzero — corrections shorter than
   the report cadence are invisible to the loop, so an earlier design
   blind-pulsed and oscillated (7 → 12 → runaway);
4. always ends with a stop frame, even on the 30 s budget expiry or a
   dead RX link.

Verified live in both directions with exact landings and `exit 0`:
`21→8`, `9→20`, `20→3` (`-p`), `3→15` (`-m 12.5` → position 15), plus a
`-p 1` park at the wide endstop. A miss or a dead RX link exits 1; the
session ends as soon as the loop is done (the 30 s budget is a ceiling,
not a run time), and a stop frame always goes out on the way out.

```
./anjoy-motor -d T -m 12.5 -j       # zoom to ~12.5x
./anjoy-motor -d T -p 8 -j -r 800   # zoom to position 8 on the 1..30 scale
```

## Usage

```
make                                          # arm-openipc-linux-musleabi-gcc, static
./anjoy-motor -d u -t 2 -r 1500 -j           # zoom tele 2 s, stream positions
./anjoy-motor -d s -t 0 -r 1500 -j           # stop; listen for in-flight reports
./anjoy-motor -d T -m 20 -j                  # absolute zoom, closed loop
```

`-r ms` extends the listen window after the move (reports keep flowing
while the motor halts); `-B` inherits the port settings instead of
reconfiguring; `-s` overrides the speed bytes; `-D` picks another UART.
Run with no arguments for the full list.

## Stock-firmware contention — TWO readers on ttyS2

The stock firmware has **two** daemons on this UART, and they are not
symmetric:

* `comm_server` is the writer/session owner. `procman` respawns it within
  seconds if it dies — SIGSTOP `procman` **first**, then SIGSTOP
  `comm_server`. A SIGSTOPped comm_server is tolerated by the system
  indefinitely (hours observed); the lens and RX link care only about
  the byte stream, and the frozen daemon stops transmitting idle stop
  frames that would otherwise cancel your moves every ~2 s.
* `media_server` **also holds the port open and READS it**. It is quiet
  while comm_server is healthy, but once comm_server is dead or frozen it
  wakes up and eats the incoming bytes: a test session saw 2 bytes of a
  +170 report burst; a goto run parsed 3 of ~140 frames. Commands still
  work (TX is unaffected — the motor obeys whoever writes a valid
  session), but closed-loop `T` starves. SIGSTOPping media_server wins
  the RX stream back — for a while: a system watchdog **reboots the
  camera roughly 60 s after media_server freezes** (observed twice).
  Keep exclusive-RX test runs inside that window and `kill -CONT` after.
  The reboot is harmless — the vendor stack comes back cleanly — but
  `/tmp` is tmpfs: write logs to `/mnt/nand` if a window may snap shut.

`killall`/`killall -9 comm_server` matches nothing: these daemons scrub
their argv (empty cmdline), so pidof-style matching on `/proc/*/comm` or
explicit PIDs is required.

On OpenIPC there is no vendor stack on the port and none of this applies —
`anjoy-motor` has the UART to itself.
