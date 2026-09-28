# Vendor contrast-autofocus algorithm (Xiongmai `libxmaf.so`), reverse-engineered

This documents **how Xiongmai's stock firmware focuses** the LENS_LH13_FHD_X16 (16×
MS41908M lens) on the HI3516D_N81820 (HiSilicon Hi3516A V100). It is a reference for
anyone re-implementing autofocus for this lens (e.g. an OpenIPC/majestic AF plugin): the
vendor's method converges on the sharpest point and then goes silent, where a naive
hill-climb hunts, overshoots, and never settles.

**Source:** `/lib/libxmaf.so` from the stock rootfs (ARM32 LE, `280972` bytes, build
dated 2018-08-28). The library is **not stripped** — every function and global keeps its
name in `.symtab` — but its DWARF only covers libgcc, so the struct field map below was
reconstructed from the machine code (r2 `pdc`/`pdf`; addresses cited are file offsets in
that binary and are there to be re-checked). Contrast this with `an41908a/` in this repo,
whose AF is the *naive* version: "probe a direction, follow increasing contrast, reverse
and halve the step until the step reaches zero." The vendor does something considerably
smarter.

## TL;DR — it is a bracket-and-return state machine, not a hill-climb

```
FCB_Focus_AF  ──▶  find direction  ──▶  COARSE accelerate  ──▶  detect peak
                                                                     │
                        ┌────────────────────────────────────────────┘
                        ▼
              FINE "mini" 3-point bracket  ──▶  return to the RECORDED best position
                        │                                     │
                        └── (2 reversals → give up hunting) ──┘
                                                              ▼
                                                    idle: watch FV, only
                                                    re-focus if the scene changes
```

Five ideas make it work, and each maps to a way a naive port goes wrong:

1. **It never trusts a focus value read right after a move.** Every decision uses an FV
   sampled **3–4 frames after the lens moved**, and the read itself blocks for a fresh
   VD. A port that reads FV one frame after stepping samples the lens mid-transit and
   chases a settling transient. *(This is the single most common bug.)*
2. **It records the best `(position, focus-value)` seen and drives back to it.** It does
   not stop *at* wherever it happened to detect the peak — it returns to the stored best
   position. Miss this and you land one bracket-step past sharp.
3. **It brackets the peak with three samples** and only declares victory when the middle
   sample beats *both* neighbours by more than a hysteresis window. A lone noise spike
   cannot confirm a peak.
4. **Coarse peak detection uses a drop-accumulator that resets on any FV recovery**, so a
   transient dip during the coarse sweep can't trip an early stop.
5. **It is bounded.** After 2 direction reversals in the fine phase it commits to the
   recorded best and *stops*. That bound is why the vendor motor falls silent — a port
   without it dithers around the peak forever (endless clicking, "never settles").

## The focus-value metric (what "sharpness" means here)

`Focus_GetDefinition_Data` refreshes the live FV every frame into the AF struct. The read
path (`XM_AF_GetValue_API`) calls `HI_MPI_ISP_GetVDTimeOut(..., 5000)` — **it blocks for a
fresh vertical sync (up to 5 s) before reading** — then pulls the ISP AF-zone edge
statistics and reduces them to one number roughly as:

```
FV = (29 * h_edge + 35 * v_edge) >> 6,  then / 80   (top-weighted zones)
```

The blocking-on-VD is not incidental: it guarantees the statistics belong to a frame the
lens was stationary for. See §"Settle timing".

## The one struct that matters: `m_tAutoFocusProcCtrl` @ `0x486c8`

The whole search reads and writes this single global. Reconstructed field map (offset →
meaning → the function that proves it):

| Off | Meaning | Evidence |
|----|----|----|
| +0x00 | motor MAX position limit | `Focus_Motor_Move` add-branch clamp |
| +0x04 | motor MIN position limit | `Focus_Motor_Move` sub-branch clamp; set in `AutoFocusVar_Init` from the Fminmax table |
| +0x08 | prevFV (FV two samples ago) | `Focus_FindPeak_Thread` |
| +0x0c | currFV (last FV) | `Focus_FindPeak_Thread` |
| +0x10/14/18 | 3-point FV sample slots s0/s1/s2 | `Focus_FindDirect_Thread`, `Focus_Mini_Thread` |
| +0x20 | one-shot **initial settle** frame counter | decremented at the top of each `*_Process` |
| +0x21 | **per-step settle reload** value | copied into +0x22 |
| +0x22 | **per-step settle** frame countdown | `if (+0x22) { +0x22--; return; }` |
| +0x27 | **step size** (microsteps per move) | `Focus_Motor_Move` |
| +0x28 / +0x29 | count of MIN- / MAX-limit hits | `Focus_Motor_Move` |
| +0x2a | **direction** (0 = subtract / 1 = add) | toggled throughout |
| +0x2c | **recorded best position** | written by `Focus_MaxDef_Record`, returned to by `Focus_Search_Max` |
| +0x30 | **recorded max FV** | `Focus_MaxDef_Record` |
| +0x34 | FindDirect sample index (0..3) | `Focus_FindDirect_Thread` |
| +0x35 | direction-decided / limit-hit flag | `Focus_FindDirect_Thread`, `Focus_Motor_Move` |
| +0x38 | **accumulated FV-drop** | `Focus_FindPeak_Thread` |
| +0x3c | **per-step FV-drop hysteresis margin** | set in `Focus_Acceler_On` |
| +0x40 | **accumulated-drop LIMIT to confirm peak** | `Focus_FindPeak_Thread` |
| +0x48 | running **max slope** seen | `Focus_GetSlope_Max` |
| +0x50 | **slope-peak-detected** flag | `Focus_GetSlope_Max` → `FindPeak` |
| +0x51 | Mini 3-sample count (0..3) | `Focus_Mini_Thread` |
| +0x52 | Mini "still climbing" step count (limit 15) | `Focus_Mini_Thread` |
| +0x53 | Mini **direction-reversal count** (limit 1) | `Focus_Mini_Thread` |
| +0x54 | Mini **bracket window** (hysteresis, 40–50) | `Focus_Mini_On` |
| +0x65 | mini-enabled flag (=1 from init) | gates the normal ending |
| +0x68 | **AF STATE** — the dispatch selector | `Vsy_ZoomFocus_Task` jump table |
| +0x70 | **live FV** (current contrast metric) | `Focus_GetDefinition_Data` |
| +0x7c | AF master-enable (`== 0x5a`) | `AutoFocus_Process` gate |

Motor position lives in a separate struct `*ptMotorCtrl` (@ `0x486bc`): **+0 = actual
current position, +4 = commanded target.** The AF code writes the *target*; a per-VD motor
task slews *actual* toward it.

## State machine

`AutoFocus_Process` runs every VD (frame) while `+0x7c == 0x5a`. It calls
`Vsy_ZoomFocus_Task` (decide) then `Vsy_FocusZoom_Motor_Task` (slew). The decision half:

```c
Focus_GetDefinition_Data();               // read fresh FV into +0x70, EVERY frame
switch (ctrl[0x68]) {                      // 9-case jump table
  case 0: FCB_ZoomFocus_State();           // IDLE / monitor
  case 1: Focus_Start_Process();           // FIND DIRECTION
  case 2: Focus_Acceler_Process();         // COARSE ramp
  case 3: Focus_Mini_Process();            // FINE "mini" creep
  case 4: FCB_ZoomFocus_State();
          Focus_TrigChk_Process();         // DONE / re-trigger monitor
  case 5..8: zoom-track / manual
}
```

Transitions (each sets `+0x68`):

```
FCB_Focus_AF ──usleep──▶ Focus_Start_On           (+0x68 = 1, +0x20 = 2)
Focus_Start_Process:  Focus_FindDirect_Thread();
                      if (+0x35) ▶ Focus_Acceler_On (+0x68 = 2)
Focus_Acceler_Process: MaxDef_Record; FindPeak;
      if peak found              ▶ Focus_Mini_On     (+0x68 = 3)
      elif hit BOTH limits       ▶ Focus_Search_Max ▶ Focus_Mini_On
      else Focus_Motor_Move; Focus_SetUpAcler_Step   // step + accelerate
Focus_Mini_Process:   MaxDef_Record; Mini_Thread;
      if peak confirmed          ▶ Focus_Search_Max
      if a limit was hit         ▶ Focus_Search_Max
Focus_Search_Max:     ptMotorCtrl[+4] = +0x2c;       // command best position
      if (+0x65)                 ▶ Focus_TrigChk_On   (+0x68 = 4)   // normal end
```

## Phase 1 — find direction (`Focus_FindDirect_Thread`)

Samples FV a few times one step apart (index `+0x34`, samples in `+0x10/14/18`) to decide
which way is uphill, sets the direction bit `+0x2a` and the decided flag `+0x35`, then
hands off to the coarse phase. Runs at **2 VD per sample**.

## Phase 2 — coarse accelerate + peak detect

`Focus_Acceler_On` seeds the phase from the **zoom position** (`zpos`; breakpoints 29 / 49
/ 76):

| zpos | init step `+0x27` | drop-limit `+0x40` | hysteresis `+0x3c` |
|---|---|---|---|
| ≤29 (wide) | 1 | 500 | 150 |
| 30–49 | 2 | 490 | 145 |
| 50–76 | 2 | 480 | 140 |
| ≥77 (tele) | 3 | 470 | 135 |

It also sets initial settle `+0x20 = 2` and per-step settle `+0x21 = 2`.

**Acceleration** — after every coarse step `Focus_SetUpAcler_Step` grows the step:

```
step += {1, 2, 2, 3}[zpos-band]           // faster the wider you are
step  = min(step, g_pbZoomInfo[zpos])      // clamp to a per-zoom max-step table
```

**Peak detection** — `Focus_FindPeak_Thread` (reconstructed):

```c
int Focus_FindPeak_Thread(void) {
    ctrl.prevFV = ctrl.currFV;                 // shift history
    ctrl.currFV = ctrl.liveFV;                 // +0x70
    if (ctrl.prevFV == 0) return 0;            // no history yet

    Focus_GetSlope_Max();                      // updates +0x48 / +0x50
    if (ctrl.slopePeakFlag) return 1;          // gradient crested (see below)

    if (ctrl.prevFV <= ctrl.currFV + ctrl.hyst) {  // +0x3c: still rising/flat
        ctrl.dropAccum = 0;                    // RESET — transient dips don't count
        return 0;
    }
    ctrl.dropAccum += (ctrl.prevFV - ctrl.currFV); // +0x38, only while FV keeps falling
    if (ctrl.dropAccum <= ctrl.dropLimit) return 0; // +0x40 (470..500)

    ctrl.dir = !ctrl.dir;                      // passed the peak — turn around
    return 1;                                  // PEAK CONFIRMED
}
```

`Focus_GetSlope_Max`:

```c
slope = (currFV > prevFV) ? (currFV - prevFV) / step : 0;   // step = +0x27
if (slope > 0 && slope >= maxSlope) maxSlope = slope;       // +0x48
if (slope < maxSlope && maxSlope > 5000) ctrl.slopePeakFlag = 1;  // +0x50
```

Two independent triggers end the coarse phase: **(a)** the FV gradient rose above `5000`
and then started falling, or **(b)** the *cumulative* FV drop past the crest exceeded
`470–500` — but only counted while FV keeps falling, with any recovery resetting the
accumulator to 0. Coarse also bails via `Focus_Search_Max` if it hits **both** mechanical
limits without finding a peak.

## Phase 3 — fine "mini" 3-point bracket (`Focus_Mini_Thread`)

`Focus_Mini_On` sets **step `+0x27 = 1` microstep** (2 for lens-ID `0x8216`), per-step
settle `+0x21 = 3`, and the bracket window `+0x54 = {50, 50, 45, 40}` by zoom band. This
is the real min/mid/max convergence — three FV samples one microstep apart:

```c
int Focus_Mini_Thread(void) {
    if (ctrl.settle) { ctrl.settle--; return 0; }   // +0x22 gate
    ctrl.settle = ctrl.settleReload;                // +0x21 = 3  → 4 VD per step

    s[ctrl.n] = ctrl.liveFV; if (ctrl.n <= 2) ctrl.n++;   // collect s0,s1,s2
    if (ctrl.n != 3) { Focus_Motor_Move(); return 0; }

    if (s1 > s2 + W && s1 > s0 + W && s0 > 39 && s2 > 39) {  // W = +0x54
        ctrl.dir = !ctrl.dir;  return 1;            // MID is the peak → confirmed
    }
    if (s0 > s1 && s1 > s2) {                        // overshoot (monotonic down)
        ctrl.n = 0; ctrl.climb = 0;
        ctrl.dir = !ctrl.dir;                        // reverse
        if (++ctrl.reversals > 1) { Focus_Search_Max(); return 0; }  // give up hunting
        Focus_Motor_Move(); return 0;
    }
    ctrl.reversals = 0;                             // still climbing / mixed
    s0 = s1; s1 = s2; ctrl.n = 2;                    // slide window, keep newest two
    if (++ctrl.climb > 15 && zpos > 30) { Focus_Acceler_On(); return 0; } // re-coarse
    Focus_Motor_Move(); return 0;
}
```

Peak is declared **only** when the middle of three samples beats *both* neighbours by more
than the window `W` (40–50) and both neighbours clear a noise floor of 39. After **2
reversals** it stops dithering and commits to the recorded best — the bound that makes the
motor go quiet.

## Return-to-peak (`Focus_MaxDef_Record` / `Focus_Search_Max`)

`Focus_MaxDef_Record` runs *every* active coarse/fine frame:

```c
if (ctrl.recordedMaxFV <= ctrl.liveFV) {   // +0x30 <= +0x70
    ctrl.recordedPos   = ptMotorCtrl[0];   // +0x2c = current ACTUAL position
    ctrl.recordedMaxFV = ctrl.liveFV;      // +0x30
}
```

`Focus_Search_Max`:

```c
ptMotorCtrl[+4] = ctrl.recordedPos;        // command target = best position seen
FocusLine_NumUpdata(...);                  // feed result into the zoom-tracking curve
if (ctrl.+65) Focus_TrigChk_On();          // → monitor state 4
```

It **does not re-measure FV at the peak** — it trusts the recorded `(pos, FV)` pair and
slews there. So the landing accuracy is exactly the sample-grid resolution of the search,
and it depends on `recordedPos` being the *actual* position on a *settled* frame. Record
position/FV at the wrong point in the settle cycle and you store an off-peak position.

## Settle timing — the critical part

Two-level frame gating, everything counted in **VD/frames**:

- Each `*_Process` first burns a one-shot initial delay `+0x20 = 2` frames.
- Then a per-step countdown `+0x22`; at 0 it does the work and reloads `+0x22 = +0x21`.

Work therefore happens once every `+0x21 + 1` frames:

| Phase | reload `+0x21` | frames per motor step |
|---|---|---|
| Find-direction | (`+0x22` = 1) | **2 VD / sample** |
| Coarse | 2 | **3 VD / step** |
| Fine (mini) | 3 | **4 VD / step** |

`Focus_GetDefinition_Data` refreshes `+0x70` every frame, but the Process code only
*consumes* it on the post-settle frame — so the FV that drives every decision is read
**3 (coarse) / 4 (fine) frames after the move**, and the read blocks for a fresh VD. The
lens is stationary when its sharpness is measured. A port that reads FV one frame after
stepping is the classic "focus misses the sharpest point" bug.

## Motor drive model — step-and-settle bursts, not per-VD continuous

- `Focus_Motor_Move` writes only the **target** (`ptMotorCtrl[+4]`), advancing it by
  `+0x27` microsteps, clamped to `[+0x04, +0x00]`; on a limit it toggles direction and
  bumps `+0x28`/`+0x29`.
- `Vsy_FocusZoom_Motor_Task` (every VD): if `actual != target` → `focus_run_nowait`.
- `focus_run_nowait`: `delta = min(target-actual, ptMotorCfg[0x4c]); motor_focus_move(dir, delta)`
  — up to `ptMotorCfg[0x4c]` microsteps **per VD** toward the target.
- `ms9x9_focus_move` loads MS41908 reg `0x29` with `step << 2` pulses + direction bit —
  **one-shot pulse burst per call**, not free-running.

Net: during the *search* the target is nudged 1–3 microsteps every 3–4 VD; the lens
executes that small move in ~1 VD and then sits idle through the settle frames — i.e.
**discrete step-then-wait bursts, and the vendor searches this way too.** Only the
return-to-peak and zoom-tracking (target far from actual) produce a continuous multi-VD
slew. The vendor's silence is not from driving continuously; it is from **converging in a
bounded search and then stopping.** A port that steps every frame overshoots; a port that
never satisfies a convergence test clicks forever.

## Idle re-trigger (`Focus_TrigChk_On` / `_Process`)

Once focused, state 4 just watches FV and only re-runs AF when sharpness drops past a
**zoom-dependent threshold**, confirmed over ~4 frames (`m_tAutoTrigChkCtrl+0x2d = 4`):

| zpos | FV-drop threshold `m_tAutoTrigChkCtrl+4` |
|---|---|
| ≤29 (wide) | 20 |
| 30–49 | 19 |
| 50–76 | 18 |
| ≥77 (tele) | 17 |

This is why stock firmware sits silent until the scene actually changes.

## What `FocusLine_*` is (and is *not*)

`FocusLine_Move / Correct / Middle / NumIs{Min,Mid,Max} / NumUpdata / DirMove /
FocusLineSet` are **not** the contrast-peak search. They are the separate **zoom↔focus
tracking (parfocal) curve**: state `m_tAutoZoomTrackCtrl` (@ `0x48748`), reference points
`focusnum_min/mid/max`, per-sensor tables `g_pbZFLineData_*` (3000 B). They interpolate
where focus *should* sit as zoom moves, so focus tracks during a zoom without a full
re-search; `Focus_Search_Max` calls `FocusLine_NumUpdata` to feed each freshly-found best
focus back into that curve. The peak-convergence bracket is `Focus_Mini_Thread` above.

## Per-unit / per-lens calibration

- `AutoFocusVar_Init` loads `/mnt/mtd/Config/lensoffset.dat` — a **persistent per-unit
  day/night focus offset** ("DayOffsetFocusNow" / "NgtOffsetFocusNow"), applied so a given
  unit's focus zero is corrected for its own mechanical tolerance.
- `FocusData_Init(sensor)` selects per-lens tables: `g_pbFminmax_*` (focus min/max **per
  zoom step** — the search bounds, so it never blind-sweeps the full range),
  `g_pbZFLineData_*` (parfocal curve), `g_pbZoomInfo_*` (per-zoom coarse max-step clamp).
  Variants seen: `_imx291` / `_LH18_291` (arg 1) and `_LH18_4689` (arg 3).

## Porting checklist (what to copy)

1. Read the ISP focus statistic **only after the lens has settled** — gate every FV
   sample 3–4 frames behind the move and read on a fresh VD.
2. **Record `(best_position, max_FV)`** every frame; at the end **drive to
   `best_position`**, don't stop where the peak was detected.
3. Coarse: accelerate the step by zoom band; stop on a slope-crest (slope > threshold then
   falling) **or** a cumulative FV-drop past a limit, with the accumulator **reset on any
   FV recovery**.
4. Fine: 1 microstep per ~4 frames, 3-sample bracket, confirm only when the middle beats
   both neighbours by a window (40–50) above a noise floor.
5. **Bound it**: give up hunting after 2 reversals and commit to the recorded best — this
   is what stops the endless clicking / "never settles".
6. Only continuous-slew on the *return* to best and on zoom-tracking; the search itself is
   step-then-wait.

## Constants (all from this `libxmaf.so`)

- Zoom-position breakpoints: 29 / 49 / 76 (`0x1d` / `0x31` / `0x4c`); re-coarse only if zpos > 30.
- Coarse init step: 1 / 2 / 2 / 3 (by zoom band). Ramp increment: +1 / +2 / +2 / +3 per step, clamped to `g_pbZoomInfo[zpos]`.
- Coarse drop-limit `+0x40`: 500 / 490 / 480 / 470.
- Coarse per-step hysteresis `+0x3c`: 150 / 145 / 140 / 135.
- Slope threshold: 5000 (`0x1388`) — max slope must exceed this before a slope-crest counts.
- Fine step: 1 microstep (2 for lens-ID `0x8216`).
- Fine bracket window `+0x54`: 50 / 50 / 45 / 40.
- Fine sample floor: neighbours' FV must be > 39 (`0x27`).
- Fine max climb before re-coarse: `+0x52` > 15.
- Fine reversal limit: `+0x53` > 1 → commit to recorded best.
- Settle frames: initial `+0x20` = 2; per-step `+0x21` = 2 (coarse) / 3 (fine); find-dir `+0x22` = 1.
- FV: `(29·h + 35·v) >> 6`, ÷ 80, top-weighted zones; `HI_MPI_ISP_GetVDTimeOut(..., 5000)`.
- Idle re-trigger threshold: 20 / 19 / 18 / 17 by zoom band, confirmed over 4 frames.
- AF master-enable byte `+0x7c` = `0x5a`.
- Max microsteps/VD slew: `ptMotorCfg[0x4c]` (runtime config).

## Provenance

Reverse-engineered from `libxmaf.so` (stock HI3516D_N81820 firmware, build 2018-08-28),
carved from the 16 MB SPI-NOR dump (`romfs` squashfs). Key symbols: `AutoFocus_Process`,
`Vsy_ZoomFocus_Task`, `Focus_Start_Process`, `Focus_FindDirect_Thread`,
`Focus_Acceler_On/_Process`, `Focus_SetUpAcler_Step`, `Focus_GetSlope_Max`,
`Focus_FindPeak_Thread`, `Focus_Mini_On/_Process/_Thread`, `Focus_MaxDef_Record`,
`Focus_Search_Max`, `Focus_TrigChk_On/_Process`, `Focus_Motor_Move`, `focus_run_nowait`,
`ms9x9_focus_move`; globals `m_tAutoFocusProcCtrl`, `m_tAutoTrigChkCtrl`,
`m_tAutoZoomTrackCtrl`, `ptMotorCtrl`, `g_pbFminmax_*`, `g_pbZFLineData_*`,
`g_pbZoomInfo_*`. The MS41908 register/GPIO/VD transport is documented in `Readme.md`.
