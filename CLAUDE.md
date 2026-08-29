# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repository is

A collection of **independent, single-file C command-line tools** that drive camera motors (pan/tilt, zoom, focus, iris) on IP-camera SoCs — Xiongmai, HiSilicon, Ingenic T31. There is no shared library and no top-level build. Each directory is a self-contained program targeting one specific hardware/driver combination, and `api/` is a design document for a future daemon that would unify them.

There are no tests — the code only does anything when talking to real motor hardware. The one automated check is `.github/workflows/gcc-compat.yml`, a **required status check** (`GCC Gate`) on `master`. Know precisely what it does and does not cover:

- It cross-compiles **five of the six tools** on GCC 12 and GCC 14. **`an41908a` is excluded**, because it needs the proprietary Hi3516CV500 MPP SDK — so changes under `an41908a/` get no CI coverage whatsoever and must be built by hand. A green gate says nothing about them.
- It is a *compiler* gate, not a *toolchain* gate. It uses Debian's glibc cross-compilers (`arm-linux-gnueabihf`, `mipsel-linux-gnu`), not the musl OpenIPC or vendor toolchains these tools actually ship against. That is deliberate: the defects it exists to catch — implicit declarations, format bugs, int conversions — are language-level and libc-independent, whereas the real toolchains are 100 MB–3.7 GB downloads and the vendor ones are GCC 4.4–6.3, too old to catch what a modern compiler rejects. The gap it leaves is a musl-only build break (a header difference between musl and glibc) passing CI and failing on the real toolchain.
- It proves the tree still compiles. It does not produce a deployable binary, and it says nothing about whether a motor moves.

Because every tool talks to different vendor hardware, **the tools cannot be built or run on the development host** — they are cross-compiled for ARM and executed on a camera over SSH.

## Building

There is no top-level Makefile and nothing builds natively — every target is a
cross-compile. Build per directory.

Directories with a Makefile — `xm-kmotor/`, `xm-uart/`, `an41908a/`:

```sh
cd xm-kmotor && make                    # builds ALL variants (see gotcha below)
cd xm-kmotor && make xm-kmotor-openipc  # single variant — prefer this
make clean
```

The `-orig` / `-openipc` suffix selects the toolchain via a target-specific `CC`:

| Target suffix | `CC` the Makefile invokes | Used by |
|---|---|---|
| `-openipc` | `arm-openipc-linux-musleabi-gcc` | xm-kmotor, xm-uart |
| `-orig` | `arm-xm-linux-gcc` | xm-kmotor (Xiongmai vendor) |
| `-orig` | `arm-hisiv510-linux-gcc` | xm-uart (HiSilicon vendor) |
| `-orig` | `arm-himix200-linux-gcc` | an41908a (HiSilicon vendor) |

**Build gotcha:** the pattern rules (`xm-%: main.o`) share a single `main.o` across variants. GNU make inherits the target-specific `CC` into prerequisites, so plain `make` compiles `main.o` once with the `-orig` toolchain and then links that same object into the `-openipc` binary. `make -n` in `xm-kmotor/` shows it plainly. Build one variant at a time with `make clean` in between.

`camhi-motor/`, `i2c-motor/`, and `ingenic-motor/` have **no Makefile** — compile the single source directly with the right cross-compiler for that SoC (see the arch warning below).

### Getting the toolchains

Nothing here is packaged for a normal distro; both toolchain families come from OpenIPC GitHub releases.

**OpenIPC Buildroot SDKs** (provides the `-openipc` targets) — <https://github.com/OpenIPC/firmware/releases/tag/toolchain>, one `toolchain.<vendor>-<family>.tgz` per SoC family, ~100 MB each, GCC 13.3.0 from Buildroot 2024.02.10:

```sh
curl -LO https://github.com/OpenIPC/firmware/releases/download/toolchain/toolchain.xiongmai-xm530.tgz
tar xzf toolchain.xiongmai-xm530.tgz
cd arm-openipc-linux-musleabi_sdk-buildroot && ./relocate-sdk.sh   # required — paths are baked in
export PATH="$PWD/bin:$PATH"                                       # gives arm-openipc-linux-musleabi-gcc
```

`relocate-sdk.sh` is not optional; the SDK ships with absolute paths from the build machine. There is also a `toolchain-asan` release with sanitizer-enabled variants. To build a toolchain from scratch instead, `OpenIPC/firmware` does it via `make <board>_defconfig && make toolchain` (see `general/toolchain.mk`, which sets `BR2_TOOLCHAIN_BUILDROOT_VENDOR="openipc"`).

**Watch the architecture — not every target is ARM.** `ingenic-motor` targets the Ingenic T31, which is **MIPS little-endian**. `toolchain.ingenic-t31.tgz` unpacks to `mipsel-openipc-linux-musl_sdk-buildroot` and its compiler is `mipsel-openipc-linux-musl-gcc`, *not* an `arm-…-musleabi-` prefix:

```sh
mipsel-openipc-linux-musl-gcc -Wall -o ingenic-motor ingenic-motor/main.c   # compiles clean
```

**HiSilicon vendor toolchains** (provides `-orig` for `xm-uart` and `an41908a`) — <https://github.com/OpenIPC/toolchains>, release `v1`. Each `.tgz` wraps an inner `.tar.bz2`. `arm-himix200-linux` and `arm-hisiv610-linux` exceed GitHub's asset limit and are split into `.part00`/`.part01`/… — `cat` them back together first.

```sh
gh release download v1 -R OpenIPC/toolchains -p 'arm-hisiv510-linux.tgz'
tar xzf arm-hisiv510-linux.tgz && tar xjf arm-hisiv510-linux/arm-hisiv510-linux.tar.bz2
```

**Name mismatch to expect:** the real binaries use the full ABI prefix — `arm-hisiv510-linux-uclibcgnueabi-gcc` — but `xm-uart/Makefile` calls `arm-hisiv510-linux-gcc`. The short alias only exists if you run the bundled `.install` script, which hardcodes `/opt/hisi-linux/x86-arm`. Just symlink the short name onto the real one and put it on `PATH`; the build then succeeds unmodified and yields a uClibc-linked ARM EABI5 binary.

`arm-xm-linux-gcc` (the Xiongmai vendor compiler for `xm-kmotor`'s `-orig` target) is in **neither** repo — build the `-openipc` variant unless you have that toolchain from a firmware dump.

`an41908a/` also needs the proprietary HiSilicon **MPP SDK** (`Hi3516CV500_SDK_V2.0.2.1`) for `hi_type.h`/`mpi_isp.h` and the `-lmpi -lisp …` libraries; set `MPP_DIR` in its Makefile. That SDK is not in either public repo, so this is the one tool you cannot build from public sources alone. Its Makefile also ends with a `sudo cp` to `/mnt/noc/…` — a leftover from one developer's deploy setup; remove or adapt it.

### Compiler-version caveat

These sources predate modern compilers and are only fully warning-clean on the vendor-era ones, so expect noise (unused variables, signedness mismatches) rather than treating it as a regression you introduced. Two issues that were real errors have been fixed — `i2c-motor/main.c` was calling `ioctl()` without `<sys/ioctl.h>`, which GCC 14+ rejects outright as an implicit declaration, and `xm-uart/main.c` had `printf("Usage : %s\n")` with no matching argument. Both now build clean under GCC 14. If you add code here, check it against a current GCC as well as the vendor toolchain, since the vendor compilers (GCC 4.4–6.3) silently accept things newer ones reject.

## Three hardware-access patterns

Understanding which pattern a tool uses explains most of its code:

1. **`ioctl` against a vendor kernel module** (`xm-kmotor`, `camhi-motor`, `ingenic-motor`). The module creates `/dev/motor`; the tool is a thin CLI over `ioctl`. The kernel module ships with vendor firmware and must be `insmod`'d with GPIO-pin and max-step parameters *before* the tool works — see each directory's Readme for the exact `insmod`/`modprobe` line, since pin maps are per-camera-model.

2. **Direct bus access from user space** (`i2c-motor`, `an41908a`). No custom kernel module — the tool drives the motor-driver IC itself over I2C SMBus (`/dev/i2c-2`, MS32006 at addr `0x10`) or SPI (`/dev/spidev2.0`, AN41908A) plus sysfs GPIO. `i2c-motor` also needs `devmem` pinmux writes first (documented in its Readme).

3. **Serial protocol over UART** (`xm-uart`). Speaks a Pelco-D variant to a Xiongmai AF module on `/dev/ttyAMA0` at 115200 8N1; interactive keyboard REPL. Note the `AUTO_FOCUS` compile-time switch changes the sync byte from `0xff` to `0xc5`, and system getty must be disabled on that UART.

## The three `/dev/motor` ioctl ABIs are mutually incompatible

This is the single most important thing to know before touching the ioctl tools. All three open `/dev/motor`, but nothing else is shared:

- **`xm-kmotor`** — encoded ioctl numbers (`0x80184D02` status, `0x80184D03` maxsteps, `0x40184D01` command) over `int[]` arrays. The command lives in `s[0]` as a **direction bitmask**: 1=left, 2=right, 4=up, 8=down, so diagonals are OR'd (6=right-up, 9=left-down); 16=scan, 17=goto, 18=stop, 19=steps, 20=set-position.
- **`camhi-motor`** — different encoded numbers (`0xC0046D01`/`0xC0046D02`), and `s[0]` is a plain **enum**: 0=stop, 1=zoom−, 2=zoom+, 3=focus+, 4=focus−.
- **`ingenic-motor`** — plain small ioctl numbers (`0x1`–`0x7`) over **C structs** (`motor_message`, `motors_steps`, `motor_reset_data`). Note that `x_max_steps`/`y_max_steps` in `motor_message` are **not** in the stock T31 kernel module — they are an OpenIPC extension, so `-S`/`-i` output depends on a patched module. Its `open("/dev/motor", 0)` also relies on the T31 driver ignoring the open mode.

Do not copy an ioctl constant or command value from one tool into another.

## CLI conventions and their traps

All tools share the shape `-d <direction-char> -s <speed> [-x N] [-y N]`, but the letters mean different things depending on whether the tool moves a camera head or a lens:

- **PTZ tools** (`xm-kmotor`, `ingenic-motor`): `u`/`d`/`l`/`r` = up/down/left/right, `s` = stop, `h` = set position, `g` = steps, `t`/`f` = goto/scan (xm-kmotor), `c`/`b` = cruise/go-back (ingenic-motor).
- **Lens tools** (`camhi-motor`, `i2c-motor`): `u`/`d` = zoom in/out, `l`/`r` = focus −/+, `i` = init (i2c-motor only).

Speed ranges also differ per driver and are silently clamped: 10 (xm-kmotor), 100 (camhi/i2c), 900 (ingenic-motor, the kernel module's practical ceiling).

`xm-kmotor` and `ingenic-motor` expose `-j` (position/status) and `-i` (adds max steps) which print **hand-rolled JSON to stdout** — these are consumed directly by a web UI's `ptzclient.cgi`, so changing key names or adding stray output to stdout is a breaking change. `xm-kmotor`'s `-j`/`-i` emit duplicate `"unknown"` keys for undecoded status fields.

## `an41908a/` — the outlier

The only tool with real algorithmic content. It drives the AN41908A lens driver over SPI: register writes are latched by pulsing GPIO `VD_FZ` (24) for focus/zoom and `VD_IS` (19) for iris, with `RSTB` (27) reset on startup. Command sequences (`turn_on`, `send_focus_cmd`, `init_lens`, `init_iris`) are raw register/value byte triples reverse-engineered from vendor firmware — treat the magic bytes as opaque.

On top of that it implements **contrast-based autofocus** in a background pthread (`AF_proc`): it pulls focus-value statistics from the HiSilicon ISP via `HI_MPI_ISP_GetFocusStatistics`, blends horizontal/vertical metrics into a weighted score using the `AFWeight` window map, then hill-climbs — probe a direction, follow increasing contrast, reverse and halve the step on each miss until the step reaches zero. This is why the Makefile links the whole MPP library set. The papers backing this approach are linked in the root `README.md`.

## `api/` — design only, nothing implemented

`api/README.md` proposes a `motors-daemon` that loads per-hardware "driver wrapper" `.so` plugins and exposes a unified PTZ/AF/IRCut API over a UNIX socket. None of it exists yet, and the open questions it raises (one wrapper at a time vs. several; whether protocol logic belongs in the kernel module or the wrapper) are unresolved. Read it before designing anything cross-cutting, but do not treat it as describing current code.
