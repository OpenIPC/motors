# anjoy-motor

Drive the zoom/focus/iris lens motors and the pan/tilt head of Anjoy AF
camera modules (SigmaStar-based boards) over the motor-MCU UART, and read
back the MCU's live position reports.

Verified on an **MTF45-4G_AF** (FW V3.4.5.6, 2026-01-06), `/dev/ttyS2`.
The wire format was recovered by ptracing the vendor's own `comm_server`
while driving the web PTZ verbs, and by listening on the UART with the
vendor daemons stopped.

## Command frames (host → MCU)

7-byte Pelco-D-style frames, address `0x01`:

```
FF | 01 | C1 | C2 | D1 | D2 | CK        CK = (01 + C1 + C2 + D1 + D2) & 0xff
```

Motion is **one-shot**: a single frame starts the motor and it runs until
a stop frame. The vendor stack follows every motion verb with *two* stop
frames 10 ms apart, and emits an idle stop frame roughly every 2 s.

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

C1 carries focus-near / iris bits, C2 the head directions and zoom — bits
can presumably be OR'd for combined motion (standard Pelco-D semantics,
not yet verified on this MCU).

## Status reports (MCU → host)

The MCU streams unsolicited **17-byte** reports — but only **while a motor
is actually moving**; at rest the port is completely silent:

```
51 01 04 78 01 0A 0A 10 3B ZZ ZZ 21 3B FF FF 3B CK
             |-- unknown --|     |     |         |
             0A 0A 10 3B   zoom   21 3B  focus   3B
```

* `CK` (byte 16) = 8-bit sum of the 16 preceding bytes (verified against
  live frames).
* `ZZ ZZ` (bytes 9-10) = zoom position, a plain **16-bit big-endian
  counter** — verified live counting *down* while zooming wide and *up*
  while zooming tele, ~4-5 reports/s. Each report is emitted **twice** in
  a row. End-stop scale not yet calibrated (values around `0x3Bxx`
  observed mid-range; an earlier partial capture suggested `0x0300` at
  the tele limit — treat the scale as unknown).
* `FF FF` (bytes 13-14) = focus position (earlier session: `14 0F`
  changed to `16 0F` after a focus-near verb; constant during zoom moves).
* Remaining fields (`0A 0A 10`, the `3B` padding, `04`, `01`) not mapped.

Decode with `-j` for one JSON object per report on stdout (TX diagnostics
go to stderr, so the JSON stream is clean for web-UI consumption):

```
{"zoom_pos":15110,"focus_pos":5135,"flags":"040A","raw":"51010478010A0A103B3B06213B140F3B29"}
```

## Usage

```
make                                          # arm-openipc-linux-musleabi-gcc
./anjoy-motor -d u -t 2 -r 5000 -j            # zoom tele 2 s, stream positions
./anjoy-motor -d s -t 0 -r 5000 -j -B         # stop; listen for any in-flight reports
```

`-r ms` is the **total listen window, starting right before the verb** —
the MCU only reports while its motors move, so the window must cover the
hold, not the quiet after the stop. With `-j`, position reports go to
stdout as JSON; everything else is printed to stderr.

Run `./anjoy-motor` with no arguments for the full option list
(`-B` inherits the port settings instead of reconfiguring, `-s`
overrides the speed bytes, `-D` picks another UART).

## Stock-firmware contention

On the stock firmware `comm_server` and `media_server` keep the same
`ttyS2` open. Consequences observed live:

* **RX is stolen** — whichever daemon sits blocked in `read()` wins the
  race for each arriving byte, so a second reader gets almost nothing.
  (The reports the vendor reads are what feed the zoom-OSD on the
  vendor RTSP overlay.) The MCU is not "mute": `/proc/tty/driver/ms_uart`
  shows the kernel rx counter rising during motion regardless of who is
  reading — use that to tell "nothing is moving" from "someone ate it".
* **TX is shared** — the vendor's idle stop frames (~every 2 s) will
  cancel a move you start and hold.

For exclusive control, stop both daemons and listen *immediately* —
`procman` respawns them within seconds (~4-8 s on FW V3.4.5.6) and the
respawned daemon will eat the rest of your window. The verified recipe:

```
killall comm_server media_server
./anjoy-motor -d d -t 2 -r 5000 -j -B         # must finish inside the gap
```

`killall` + a single ~7 s verb/listen reliably yields the reports of the
move you just commanded; everything after the respawn belongs to the
daemon again.

On OpenIPC there is no vendor stack on the port and none of this
applies — `anjoy-motor` has the UART to itself.
