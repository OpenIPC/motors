# MS32006 Motorized Lens Controller for SigmaStar SSC338Q

High-precision dual stepper motor controller for 5x motorized optical varifocal lenses (e.g. Sony IMX415 + CW-TY207135D14 2.7–13.5mm) on **SigmaStar SSC338Q (Infinity6e)** cameras.

Due to the mechanical sensor mount depth, positions below 2000 steps lie beyond the optical focal plane; the enforced usable range is **6.3 mm to 13.5 mm (approx. 2.3x to 5.0x optical zoom)**.

---

## ⚡ Features

1. **Autonomous SigmaStar SoC RIU Power Rail Initialization**:
   Automatically maps `/dev/mem` and clears register `0x1F223618` (`Bank 0x111B Offset 0x06`, bit 0) before I2C transactions to enable the motor power rail without requiring kernel modules or external scripts.
2. **1/8 Microstepping & Excitation**:
   Configures `Reg 0x00 = 0x01` (Standby release) and `Reg 0x0A = 0x08` (1/8 microstep excitation mode) for quiet, high-precision lens positioning.
3. **Automatic 0 mA Coil Sleep (Cool Operation)**:
   De-energizes stepper coils (`Reg 0x0A = 0x00`, `Reg 0x00 = 0x00`) immediately when moves complete. Eliminates motor heating while relying on the lead screw's mechanical detent holding torque.
4. **Adaptive Parabolic Parfocal Tracking Math**:
   Implements simultaneous dual-axis parabolic focus tracking:
   $$\text{Focus}(z) = -0.000190833 \cdot z^2 + 0.704167 \cdot z + 4045.0$$
   Dynamically scales segment count to the move distance (~1 segment per 100 zoom steps, clamped 1..25) for smooth parfocal tracking on large moves and snappy response on short jogs.
5. **OpenIPC Web UI & CLI Compatibility**:
   - OpenIPC flags: `-d <u|d|r|l|i|s>`, `-s <speed>`, `-p <pps>`, `-n <steps>`, `-x <steps>`, `-j` / `-i` (JSON status matching `ingenic-motor`).
   - Direct Web UI `/cgi-bin/j/ptz.cgi` format: `motor <profile> <h> <v>`.
   - Concurrency protection: Non-blocking file locking (`LOCK_EX | LOCK_NB`) prevents overlapping moves and reports `motor: device busy`.

---

## 🛠️ Build

OpenIPC rootfs images are musl-based, so build with the OpenIPC SigmaStar Buildroot SDK (`arm-openipc-linux-musleabi-gcc`):

```sh
# 1. Download and unpack OpenIPC toolchain for SigmaStar Infinity6e
curl -LO https://github.com/OpenIPC/firmware/releases/download/toolchain/toolchain.sigmastar-infinity6e.tgz
tar xzf toolchain.sigmastar-infinity6e.tgz
cd arm-openipc-linux-musleabi_sdk-buildroot && ./relocate-sdk.sh   # required — paths are baked in
export PATH="$PWD/bin:$PATH"

# 2. Build motor binary
cd i2c-motor/sigmastar-ssc338q
make CC=arm-openipc-linux-musleabi-gcc
```

---

## 🌐 OpenIPC Web UI Integration

To enable the on-screen PTZ overlay controls on the live video stream in OpenIPC Web UI:

1. Enable PTZ in the U-Boot environment:
   ```sh
   fw_setenv ptz 1
   ```
2. Ensure the binary is installed at `/usr/bin/motor`:
   ```sh
   cp motor /usr/bin/motor
   chmod +x /usr/bin/motor
   ```
3. Open the camera Web UI in your browser (`http://<camera-ip>`). The live stream preview will display interactive on-screen controls:
   - **Up / Down**: Smooth simultaneous parfocal optical zoom (IN / OUT).
   - **Left / Right**: Fine focus adjustments (FAR / NEAR).
   - **Center (OK)**: Full mechanical homing & optical autofocus calibration (`motor home`).

> [!NOTE]
> Since movements are synchronous, stop (`-d s`) returns immediately without hardware interaction. Web UI press-and-hold zoom operations degrade gracefully to discrete step jogs.

---

## 🎮 Commands

After a system reboot (or if `/tmp` state is lost), run `motor home` or press the center button in the Web UI to calibrate the physical lens position. Relative moves are gated until calibration is established.

```sh
# 1. Optical Homing & Calibration (Lands at 6.3mm Wide, Focus: 4690)
motor home

# 2. Smooth Parfocal Zoom to any focal length (6.3mm .. 13.5mm)
motor setfocal 8.5      # Sets 8.5mm with auto-focus tracking
motor setfocal 13.5     # 5.0x Maximum Optical Telephoto

# 3. OpenIPC Standard Flags
motor -d u -s 10        # Zoom In (Parfocal, speed=10 -> 800 PPS)
motor -d d -s 10        # Zoom Out (Parfocal, speed=10 -> 800 PPS)
motor -d r -s 5         # Fine Focus Near
motor -d l -s 5         # Fine Focus Far
motor -d i              # Init / Home calibration
motor -d s              # Stop (returns 0)
motor -j                # JSON status output for Web UI & Majestic
motor -i                # Full camera parameters JSON for Web UI initialization
```
