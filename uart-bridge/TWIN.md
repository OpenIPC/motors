# Comparing two cameras' lens boards (network replay)

This procedure proves that a second camera's lens board, for example one running OpenIPC, does the same as a reference camera's board when both get **the same traffic from the same vendor firmware**. It was first used on a pair of Xiongmai 85H50AI cameras (`xm-uart/PROTOCOL.md`, *Second board*), and it works for any camera whose lens board sits on a UART.

```
                     ┌──────────── bridge --tee ─────────────┐
reference camera ──► │ uart-bridge (lab host) ──► its own board │   p2c: reference board's replies
      (stock fw)     │            │                           │
                     └────────────┼───────────────────────────┘
                                  │ whole frames over TCP (c2t)
                                  ▼
                 second camera: xm-uart -l 9000 ──► its lens board   t2c: second board's replies
```

The reference camera's UART runs through the bridge as usual. Every whole frame it sends to its own lens is also sent over TCP to `xm-uart -l` on the second camera, which writes it to its own lens board and returns that board's replies.
- `uart-bridge boards` compares where the two boards settle.
- RTSP snapshots from both cameras compare what the lenses actually do.

## What you need
- A reference camera wired through `uart-bridge bridge` (see README).
- A second camera you have shell access to, whose lens board is on a UART (`/dev/ttyAMA0` on the 85H50AI), with a network path from the lab host.
- Remote power for both, to re-home the boards. The lab uses defib's MikroTik PoE control (`defib.power.routeros.RouterOSController.power_cycle(port)`); issue one API call at a time per connection.
- RTSP access to both cameras.

## 1. Get the second camera's firmware off the lens

**Only one process may own the lens UART.** On OpenIPC, majestic has its own focus-motor driver (`isp.autofocus`, actuator `pelco-xm`). Setting its mode to `manual` is **not enough**: majestic keeps the tty open, and two readers on one tty split the board's replies between them. In the lab, one zoom produced two reports; xm-uart saw X2.9 and majestic's `/zoom` saw X3.0. Also, a writer that interleaves bytes with the relay's can leave a partial frame, which the XM board's parser will complete with the next command.

```sh
cli -s .isp.autofocus.enabled false      # majestic releases the UART; video keeps running
/etc/init.d/S95majestic restart
ls -l /proc/$(pidof majestic)/fd | grep ttyAMA || echo free
# to undo: cli -d .isp.autofocus.enabled && /etc/init.d/S95majestic restart
```

`/proc/tty/driver/ttyAMA` shows tx and rx byte counts per UART. That's a quick way to see whether anything writes to the board, or whether the board answers at all.

## 2. Install the relay on the second camera

Cross-build `xm-uart` (`make xm-uart-motors-openipc`) and copy it over. OpenIPC's dropbear has no sftp-server, so use `scp -O`. Install it into the overlay, not `/tmp`, so it survives the reboots re-homing needs, and start it at boot with `xm-uart/S99xm-uart-relay`:

```sh
scp -O xm-uart-motors-openipc root@CAM:/usr/bin/xm-uart
scp -O ../xm-uart/S99xm-uart-relay root@CAM:/etc/init.d/ && ssh root@CAM 'chmod +x /etc/init.d/S99xm-uart-relay && /etc/init.d/S99xm-uart-relay start'
```

**Set `CLIENT` in the init script to the uart-bridge host's address** before starting it. The relay is unauthenticated, so `-a CLIENT` is what stops any other host on the network from driving the lens; the script refuses to start without it.

The relay writes only **whole frames** to the board. A partial frame is dropped after 300 ms, and stray bytes are dropped. It sends a stop frame when its client disconnects, and it returns the board's replies to the client. It serves one client at a time and refuses a second. A client that vanishes without closing is noticed within ~11 s through TCP keepalive, and the lens is stopped. A client that sends faster than the UART can transmit loses whole frames, never half of one.

**Check that the second board is really there before trusting anything:**

```sh
uv run scripts/xm_uart_audit.py sync --openipc socket://CAM:9000
```

This drives both zooms to the wide end stop and back-probes each one. Each board must stay silent at the stop, then report when probed in reverse, e.g. `[1.0, 1.2, 1.2]`. A board whose receive line isn't wired fails here.

**This checks the board, not the lens.** The reports come from the board's step counter. A lens that doesn't move at all still passes: its board counted steps and reported X1.0 → X3.2 normally. Only the pictures in step 3 catch that.

## 3. Start both boards from the same state
1. **Re-home both.** Power-cycle both cameras. An XM board homes zoom and focus by itself at power-up, then returns the zoom to its last position; see PROTOCOL.md, *Power-up behaviour*. Wait for each board's single restore report: about 70 s on the reference, and about 120 s on a board that hears nothing from its camera.
2. **`sync`.** Drive both into the wide end stop. At the stop, each board sets focus from its own zoom tracking, so the two are comparable.
3. **Compare snapshots.** At the wide stop, both cameras should frame the scene the same way. **A different field of view at the "same" zoom means one lens isn't where its board thinks it is**: a lens that doesn't move, or a mechanism that doesn't match the board. Stop and fix that first.

## 4. Replay
- **Live (twin):**
  ```sh
  uv run scripts/xm_uart_audit.py twin --camera REF --openipc socket://CAM:9000 --openipc-rtsp rtsp://…
  ```
  It bridges with `--tee`, drives the stock firmware through DVRIP (zoom in and out, focus near and far, back to wide), records the centre sharpness of both cameras at each checkpoint, and ends with `uart-bridge boards` on the capture.
- **From a recording:**
  ```sh
  uart-bridge inject --ptz socket://CAM:9000 --replay stock.jsonl --on-abort c50100000000015c
  uart-bridge boards stock.jsonl replay.jsonl
  ```
  **Start the second lens at the zoom the recording started at.** The same command moves the zoom reading by different amounts from different starting points, because zoom ratio isn't linear in motor steps.

`boards` pairs the positions where each board *settled*, meaning its last report before a pause, and fails if any pair differs by more than `--tolerance` (0.1).

## 5. Focus
```sh
uv run scripts/xm_uart_audit.py focusoffset --camera REF
uv run scripts/xm_uart_audit.py focusoffset --rtsp rtsp://… --ptz socket://CAM:9000
```

This measures how far each board's tracked focus is from that camera's sharpest point. It uses a one-direction sweep after taking up backlash, then drives back. Equal offsets mean equal lenses.

"No peak inside the sweep" means the best focus lies outside the swept range: widen `--away` and `--steps`. If sharpness is still rising when the lens reaches a stop, the lens can't reach focus at all. Then run the same sweep on both lenses from their near stop and compare *where along the travel* each one is sharpest:
- **Same travel, different sharp point:** the fault is optical or mechanical in that lens (the focus group has shifted, or the back-focus has changed).
- **Different travel:** the board's limits or reference differ.

## Pitfalls (all hit in the lab)
- **Two readers on the lens tty.** See step 1.
- **Stale lens state.** Compare only after both boards have re-homed. Earlier autofocus runs, or sweeps, leave focus offsets that tracking preserves through every zoom move.
- **Relay in `/tmp`.** It's gone after the power cycle that re-homing needs. Install it into the overlay.
- **Different scenes.** Absolute sharpness numbers aren't comparable between cameras. Compare trends and the positions of peaks, not values.
- **Killing your own session.** `pkill -f pattern` matches the SSH command line that contains the same pattern. Use `pgrep`/`pkill` on a pattern that isn't in your own command, or kill by PID.
