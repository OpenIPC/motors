# ms41908-lens — standalone MS41908M zoom/focus/iris tool (Xiongmai HI3516D_N81820)

Drives a **Panasonic MS41908M** (an AN41908A clone) varifocal lens driver **directly
over SPI + memory-mapped GPIO** — no vendor kernel modules, no MPP/ISP SDK, no
`Sofia`. Zoom, focus and iris on a 16× motorized lens. Works under **OpenIPC** (where
the vendor stack is absent) and on stock firmware. Same idea as the Zenointel
`sd2n4g-motor` tool, but for an **SPI lens chip** driving two stepper motors + an iris
DAC, instead of GPIO-bit-banged steppers.

The board is the Xiongmai **HI3516D_N81820** (HiSilicon **Hi3516A V100**), lens module
**LENS_LH13_FHD_X16** (16× varifocal; ships with IMX291/IMX323/OV4689 heads). The bus,
register map, GPIO map, pinmux, interrupt setup and per-move handshake are all
reverse-engineered from the stock firmware's `libxmaf.so`
(`xmspi_*` / `ms419x9_init` / `ms419_plsintr_init` / `motor_config_register`).

## The motor needs the ISP pipeline's VD timing

The MS41908M steps on the falling edge of **VD_FZ**, a strobe the tool pulses per move,
**but the chip only latches it while the ISP pipeline is producing vertical-sync
timing.** So a streamer must be running for the motors to actually turn:

- **OpenIPC** — run **majestic**. It drives the pipeline (supplies VD) but never touches
  the MS41908M, so this tool can own the lens IC while video runs. Nothing to stop.
- **stock Xiongmai** — `Sofia` is *both* the streamer and the lens driver, so it fights
  the tool for `/dev/spidev1.0` and the lens GPIOs. Stop it first, then restore:

  ```sh
  killall Sofia dvrHelper
  # ... run the tool ...
  setsid dvrHelper /lib/modules /usr/bin/Sofia 127.0.0.1 9578 &
  ```

SPI *register* access (init, `probe`, iris DAC writes) works with no streamer at all;
only the stepper motion depends on VD.

## Hardware (reverse-engineered from the stock firmware)

| Signal | Wiring | Notes |
|---|---|---|
| **SPI** | `/dev/spidev1.0`, mode `0x0C` (`SPI_LSB_FIRST\|SPI_CS_HIGH`), 8-bit, 5 MHz | one `{addr,lo,hi}` triple per `SPI_IOC_MESSAGE`; read = `{addr\|0x40,0,0}` |
| **EN** (chip select) | GPIO **8_7** | driven HIGH for the duration of each SPI transfer (`xmspi_enable`) |
| **VD_FZ** (move latch) | GPIO **10_5** | pulsed after each ctrl-register write to commit the move |
| **PI focus** | GPIO **0_3** (input) | focus photo-interrupter |
| **PI zoom** | GPIO **0_4** (input) | zoom photo-interrupter |
| motion-done IRQ | block `0x20220400..` | status `+0x14`, clear `+0x1C`; zoom = bit 1, focus = bit 0 |

GPIO is ARM **PL061** (no gpiolib/sysfs on this board) — banks at `0x20140000 +
bank*0x10000`, masked data write `*(base + (1<<(pin+2))) = level<<pin`, direction at
`base+0x400`. The tool drives it via `/dev/mem` mmap.

**Pinmux** (`ms419_plsintr_init`): OpenIPC leaves the SPI1 pads as plain GPIO, so the
tool sets them itself before any SPI — SPI1 SCLK/SDO/SDI → function 1 at IOCONFIG
`0x200F0060/64/68`, and the SPI1 clock gate at `0x201C0000 |= 0x80`. Without this every
SPI read comes back `0`. (Stock Sofia did this at lens init; OpenIPC does not.)

**MS41908M register map** used here: ctrl `zoom = 0x24`, `focus = 0x29`,
value = `(4*step) | 0x0400(start) | (dir<<8)`; iris DAC = reg `0x00`; init writes
`0x20=0x5C02, 0x21=0x0087, 0x22, 0x23, 0x25, 0x27, 0x28, 0x2A, 0x0B` (see `ms419_init`).
Read-back of `0x21==0x0087` / `0x20==0x5C02` confirms the bus is alive.

## Travel limits and homing

`motor_config_register` in `libxmaf` gives this lens **2610** zoom microsteps and **1280**
focus microsteps end-to-end. **No working PI home is wired on the bench units seen** (the
PI reads a constant level because the lens rides an end-stop), so the tool cannot trust
the photo-interrupter for zeroing. Instead:

- **Unhomed** (default): jog is allowed but clamped to a generous backstop (±(MAX+200))
  so a runaway can't accumulate; `p` shows `(UNHOMED)`.
- **Zero it**: gently jog to a mechanical end (you'll hear a soft click at the stop) and
  press **`o`** — that point becomes 0 and exact soft limits `[0,MAX]` turn on.
- **`h`** does a bounded seek into the WIDE + NEAR stops to set 0 automatically.

Once zeroed, jog **refuses to drive past 0 or MAX**, so it can't ram a stop. The motors
are current-limited steppers — a brief stall at an end-stop is harmless, but the soft
limits stop *sustained* ramming. Direction polarity (`WIDE`/`TELE`, `NEAR`/`FAR`) is
board-specific in `libxmaf` (XORed with a per-axis cfg bit), so a label may be flipped
vs the physical lens; the point is the *range* can't be exceeded either way.

## Build

```sh
export PATH=/opt/arm-openipc-linux-musleabi_sdk-buildroot/bin:$PATH
make                       # -> ms41908-lens-openipc (static musl ARM)
```

## Run (on the camera)

```sh
./ms41908-lens             # interactive jog (soft travel limits, keys below)
./ms41908-lens probe       # register dump — drives the SPI bus + EN pin; on stock, stop Sofia first
./ms41908-lens home        # motion proof: bounded PI home-seek on both axes
./ms41908-lens watchpi 15  # read-only PI-pin monitor for 15 s (safe with streamer up)
./ms41908-lens diag 12     # instrument one zoom axis to locate a stepping failure
```

Interactive jog keys:

```
  +/-   zoom (tele / wide)      q/w   iris (open / close)
  x/z   focus (far / near)      o     zero here (enable exact soft limits)
  p     position + PI status    h     home (bounded seek to stops = 0)
  Ctrl-C quit
```

## Notes / caveats

- **`watchpi` is truly read-only** — it only samples the PI input GPIOs — and is safe to
  run while the streamer is up. **`probe` is not**: it configures the SPI1 pads and toggles
  the shared **EN** chip-select around every read, so although it commands no motion it can
  corrupt a concurrent lens transaction. Stop Sofia first before probing on stock firmware
  (on OpenIPC nothing else owns the lens). The jog / `home` / `diag` motion modes likewise
  need exclusive access — keep majestic running on OpenIPC for VD.
- Position is not persisted across invocations — re-zero (`o`/`h`) each run.
- The `home` PI-seek is a motion *proof* (any PI GPIO transition = physical motion); on
  units where the PI never transitions it reports no motion even though the motor stepped
  — use the RTSP/snapshot image to confirm zoom/focus visually instead.
- Builds clean under musl (OpenIPC) and glibc; `_IOC_SIZEBITS` is defined locally because
  musl's `<sys/ioctl.h>` doesn't expose the value `<linux/spi/spidev.h>` needs.
