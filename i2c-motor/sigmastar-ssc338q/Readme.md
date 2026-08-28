# MS32006 Motorized Lens Controller for SigmaStar SSC338Q

High-precision dual stepper motor controller for 5x motorized optical varifocal lenses (e.g. Sony IMX415 + CW-TY207135D14 2.7–13.5mm) on **SigmaStar SSC338Q (Infinity6e)** cameras.

---

## ⚡ Features

1. **Autonomous SigmaStar SoC RIU Power Rail Initialization**:
   Automatically maps `/dev/mem` and clears register `0x1F223618` (`Bank 0x111B Offset 0x06 = 0x0000`) before I2C transactions to enable the motor power rail without requiring kernel modules or external scripts.
2. **1/8 Microstepping & Excitation**:
   Properly configures `Reg 0x00 = 0x01` (Standby release) and `Reg 0x0A = 0x08` (1/8 microstep excitation mode) for quiet, high-precision lens positioning.
3. **Automatic 0 mA Coil Sleep (Cool Operation)**:
   De-energizes stepper coils (`Reg 0x0A = 0x00`, `Reg 0x00 = 0x00`) immediately when moves complete. Eliminates motor heating while relying on the lead screw's mechanical detent holding torque.
4. **Parabolic Parfocal Tracking Math**:
   Implements simultaneous dual-axis parabolic focus tracking:
   $$\text{Focus}(z) = -0.000190833 \cdot z^2 + 0.704167 \cdot z + 4045.0$$
5. **OpenIPC Web UI & CLI Flag Compatibility**:
   - Standard flags: `-d u`, `-d d`, `-d r`, `-d l`, `-d i`, `-j` (JSON status).
   - Direct Web UI `/cgi-bin/j/ptz.cgi` format: `motor <profile> <h> <v>`.

---

## 🛠️ Build

```sh
# Cross-compile for SigmaStar ARMv7:
make CC=arm-linux-gnueabihf-gcc
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
3. Open the camera Web UI in your browser (`http://<camera-ip>`). The live stream preview will now display interactive on-screen controls:
   - **Up / Down**: Smooth simultaneous parfocal optical zoom (IN / OUT).
   - **Left / Right**: Fine focus adjustments (FAR / NEAR).
   - **Center (OK)**: Full mechanical homing & optical autofocus calibration.

---

## 🎮 Commands

```sh
# 1. Optical Homing & Calibration (Lands at 6.3mm Wide, Focus: 4690)
motor home

# 2. Smooth Parfocal Zoom to any focal length (6.3mm .. 13.5mm)
motor setfocal 8.5      # Sets 8.5mm with auto-focus tracking
motor setfocal 13.5     # 5.0x Maximum Optical Telephoto

# 3. OpenIPC Standard Flags
motor -d u -s 10        # Zoom In (Parfocal)
motor -d d -s 10        # Zoom Out (Parfocal)
motor -d r -s 5         # Fine Focus Near
motor -d l -s 5         # Fine Focus Far
motor -j                # JSON status output for Web UI & Majestic
motor -i                # Full camera parameters JSON for Web UI initialization
```
