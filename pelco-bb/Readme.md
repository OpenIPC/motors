# pelco-bb — bit-banged Pelco-D sender for debugging an RS-485 PTZ head (Xiongmai HI3516D_N81820)

Sends any bytes at 9600 8N1 (default) on the PTZ port of the Xiongmai
**HI3516D_N81820** board (HiSilicon **Hi3516A/D V100**) by driving the UART1
pads as GPIOs through `/dev/mem`. Use it to find out whether a PTZ head on the
bus can be reached at all, and with which electrical drive, when frames
written to `/dev/ttyAMA1` (majestic's `ptz.head: pelco-d`) do nothing.

## Why it exists

On the lab N81820, with the head and a USB-RS485 adapter on the same pair, the
UART path never reached the bus. `/dev/ttyAMA1` at 9600, with GPIO9_2 (the
stock libdvr's "DE") high or low, gave 0 bytes at the adapter. So did UART2,
UART3 and a forced break. The bus pair sees UART1 TXD and RXD single-ended from
CN2 pins 3/4, and a lone TXD swing never pulls A below B.

Driving both pads as a complementary pair does work. The default mode below
moved the head, and the adapter decoded clean frames.

| Mode | Pads | Bus |
|---|---|---|
| default | TXD pad (GPIO9_5) = data, RXD pad (GPIO9_3) = NOT data | normal polarity, ±3.3 V differential |
| `-i` | TXD pad only, inverted, GPIO9_2 high during the burst | for an A/B pair wired the other way round |

Both modes set the bus-mark levels **before** the pads leave the UART, so the
line never drops into a break at the start. On exit they restore exactly what
they found: the pads (`0x200f007c`, `0x200f0084`), the GPIO9 direction
register and GPIO9_2's level. So UART1 works again, and a board profile that
holds GPIO9_2 high keeps it high. SIGINT, SIGTERM and SIGHUP end the burst at
the next byte and take the same path. The tool then exits 130.

## Timing

The bits are timed by a busy-wait on `CLOCK_MONOTONIC` under `SCHED_FIFO` with
memory locked. Kernel interrupts (ISP/VIU, VPSS, the encoder and the tick)
still delay an edge now and then. Without a guard, about 20 % of frames came
out damaged, with or without majestic running. So each edge's lateness is
measured after the write. A frame with an edge later than 20 % of a bit is
sent again, up to 5 times, after a fresh idle gap counted from the real time
of its last edge, so the head can resync. The head drops the damaged copy on
its checksum. The tool reports `N frame(s) resent, M not delivered` on stderr and
exits 1 if any frame never got a clean copy out. Expect one or two of those
in a few hundred frames.

It only transmits. A Pelco-D head does not answer anyway.

## Usage

```sh
pelco-bb -r 250 ff 01 00 04 3f 00 44    # addr 1: pan left, ~2.5 s of repeats
pelco-bb -r 3   ff 01 00 00 00 00 01    # stop
pelco-bb -r 250 ff 01 00 02 20 00 23    # pan right
pelco-bb -i -r 250 ff 01 00 04 3f 00 44 # same, single inverted pad + GPIO9_2
```

Each byte is one hex argument, 00–ff; anything else is rejected before the
hardware is touched. `-b` takes 50–115200 baud. A Pelco-D frame is
`FF addr cmd1 cmd2 data1 data2 sum`, with `sum` = bytes 2–6 mod 256. Don't run
it while majestic is sending on `ttyAMA1`, because both use the same pads.

## Checking the bus with a USB-RS485 adapter

Put the adapter on the same pair as the head and keep it receiving at 9600 8N1
(`stty -F /dev/ttyUSB0 9600 raw clocal -crtscts`, then `cat` it into a file)
while the camera sends. Then count frames whose checksum is valid:

- **Clean frames, and the head moves:** the drive works.
- **Clean frames, but the head does not move:** something else on the pair
  blocks it. For example, a node wired the other way round biases the idle
  line to space even while it only listens.
- **Nothing at all:** the drive does not reach this pair.
- **A fixed garbled pattern:** the pair is reversed. The Pelco-D stop frame
  `FF 01 00 00 00 00 01` comes out as `00 7f ff ff ff fd`, and
  `UUUUhello\r\n` as `55 55 55 55 2f 35 27 27 21 e5 eb`.

Also replay the same frames from the adapter to the head with the camera
**detached and attached**. That separates "the camera cannot drive" from "the
camera spoils the bus".

## Build

```sh
make                 # pelco-bb-openipc, needs arm-openipc-linux-musleabi-gcc
```

The vendor HiSilicon uClibc toolchain (`arm-linux-uclibceabi-gcc` from
`arm-gcc6.3-linux-uclibceabi`) also works: `arm-linux-uclibceabi-gcc -O2 -static
-o pelco-bb main.c`. Don't use a recent glibc cross-compiler for the camera: its
static binary aborts with `FATAL: kernel too old` on the 4.9 kernel.

## Board constants

The tool is hard-wired to the HI3516D_N81820 / Hi3516A V100 map:

| What | Address / bit |
|---|---|
| IOCONFIG | `0x200f0000`; UART1_RXD pad `0x07c` (1 = UART, 0 = GPIO9_3), UART1_TXD pad `0x084` (1 = UART, 0 = GPIO9_5) |
| GPIO9 (PL061) | `0x201d0000`; data is masked by address bits, so `base + (mask << 2)` writes only the bits in `mask`; direction at `+0x400` |
| GPIO9_2 | the RS-485 DE line in stock `libdvr` (`PtzWrite`); the OpenIPC builder profile holds it high |

For another board, change these constants.
