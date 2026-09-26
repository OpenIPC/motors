# sd2n4g-motor — standalone pan/tilt stepper tool (Zenointel SD-2N-4G)

Drives the SD-2N-4G's two pan/tilt stepper motors **directly over memory-mapped
GPIO** — no vendor kernel modules (`motor.ko`/`gpioStep.ko`), no `hunter` app, no
network/DHIP. Works under **OpenIPC** (where the vendor stack is absent) and on
stock firmware (with, or even without, the vendor app running). Same idea as the
Xiongmai `ms41908_lens.c`, but for a 4-wire stepper over GPIO instead of an SPI
lens chip.

**This camera has no motorized zoom/focus** — the lens is fixed/flat. The firmware
exposes phantom `zoomfocus` RPCs that return success but move nothing (verified by
before/after snapshots showing no FOV or focus change). So this tool is pan/tilt
only.

## Hardware (reverse-engineered from the stock firmware)

SoC Goke **GK7205V510**. GPIO = ARM **PL061**, banks `0x120B0000 + bank*0x1000`
(banks 0–9). Masked data write `*(base + (4<<pin)) = level<<pin`; direction at
`base+0x400`; pinmux (IOCFG) at `0x100C0000`/`0x112C0000`, nibble 0 = GPIO.

| Motor | GPIOs (cfg `"motor"`) | phase-col order (cfg `line=[0,2,1,3]`) | dir inv | full travel |
|---|---|---|---|---|
| **pan**  (cfg "roll",  timer 2) | 3, 4, 72, 73 | **3, 72, 4, 73** | 0 | 1640 steps ≈ 280° (5.86 st/°) |
| **tilt** (cfg "pitch", timer 3) | 69, 59, 58, 57 | **69, 58, 59, 57** | 1 | 580 steps ≈ 90° (6.44 st/°) |

Stepping = `gpioStep.ko`'s **half-step 8-phase** table (verbatim):
`1001, 1000, 1010, 0010, 0110, 0100, 0101, 0001`, one 4-bit write per tick,
`period_µs = 1e6/speed/2`, `speed` ≈ 100–900 (stock uses 150–600). Open-loop step
counting; **no home/PI sensor** is wired on this unit. The tool's half-step count
equals the vendor's `steps` (measured: 400 pan steps ≈ 68° → full ≈ 1640).

The axis is **not self-holding when de-energized** — releasing coils lets the tilt
head droop under gravity (the vendor holds a coil current). A move leaves the last
phase energized (hold); `off` releases both motors.

## Build

```sh
export PATH=/opt/arm-openipc-linux-musleabi_sdk-buildroot/bin:$PATH
make                       # -> sd2n4g-motor-openipc (static musl ARM)
```

## Run (on the camera)

```sh
./sd2n4g-motor                      # interactive jog: l/r pan, u/d tilt (arrows too),
                                    #   +/- speed, o off, p probe, q quit
./sd2n4g-motor -d l -x 400 -s 400   # pan left 400 steps @ speed 400
./sd2n4g-motor -d r -x 400          # pan right 400 steps
./sd2n4g-motor -d u -y 200          # tilt up 200 steps
./sd2n4g-motor -d p                 # probe pin state, no motion (safe with app up)
./sd2n4g-motor -d s                 # de-energize both motors
```

CLI follows the repo PTZ convention: `-d <dir> -s <speed> -x <pan steps>
-y <tilt steps>`, with `l`/`r` = pan, `u`/`d` = tilt, `s` = stop, `p` = probe.

Direct-register writes take effect regardless of what "owns" the GPIO, so on stock
the tool moves the motors even with `hunter` running **as long as no PTZ command is
in flight** (verified). For exclusive control (e.g. contention or OpenIPC bring-up),
stop the vendor owner first — and disarm the watchdog so it doesn't reboot:

```sh
printf V > /dev/watchdog                      # magic-close disarm (if not NOWAYOUT)
killall hunter ; rmmod zoomfocus gpioStep motor   # optional; frees the pins cleanly
```

A reboot restores stock. A full NAND backup exists as the safety net.

## Notes / TODO

- Coil pairing comes from cfg `line=[0,2,1,3]`; if a motor buzzes instead of
  turning on another unit, try the raw GPIO order or the full-step table.
- `-a <deg>` absolute/relative-by-degrees convenience and a soft-limit clamp
  (using the travel figures above) are natural next additions.
- Destined for `~/git/motors` (as `zenointel-sd2n4g/`).
