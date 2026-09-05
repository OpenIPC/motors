# pelcodtui

`pelcodtui` is a standalone ncurses interface for Pelco-D camera controllers.
It sends UART commands directly and uses camera templates to describe autonomous
controller settings.

Pelco-D does not return these settings. The TUI therefore shows the last command
sent and its timestamp. It never presents saved values as confirmed camera state.

<img width="2548" height="1335" alt="image" src="https://github.com/user-attachments/assets/4fb2e6ad-354b-4a01-ae46-66da988db382" />


## Build and run

```sh
make test
./build/pelcodtui --dry-run --profiles-dir ./cameras --state /tmp/pelcodtui-state.conf
```

Camera commands in `--dry-run` mode do not use the hardware lock or update
command history.

Install on a camera with `make DESTDIR=/path/to/rootfs install`. The default
connection for the bundled P35 HiEasy, H07 HiEasy, and legacy P6SLite profiles
is `/dev/ttyAMA0`, 115200 baud, address 1.
The program shares `/tmp/btzoom.lock` with the OpenIPC WebUI and keeps the lock
directory empty for compatibility with `btzoom`. An empty lock older than 60
seconds is treated as stale and may be recovered.

Build and deploy over SSH:

```sh
CAMERA_PASSWORD='your-password' make deploy
```

Override the defaults with `CAMERA_HOST`, `CAMERA_USER`, or `CAMERA_CC`.
The build first looks for an OpenIPC firmware checkout at `../../firmware`,
then falls back to finding `arm-openipc-linux-musleabi-gcc` on `PATH`.
The camera deploy bundles ncurses and its terminal definitions under
`/usr/lib/pelcodtui`. Override their sources with `CAMERA_NCURSES_LIB` and
`CAMERA_TERMINFO_DIR` when needed. By default, deployment gets both paths from
the cross-compiler sysroot reported by `CAMERA_CC -print-sysroot`.
SSH host-key checks remain enabled. Set `CAMERA_INSECURE_SSH=1` only for a
camera whose host key you cannot store or check.
Deployment replaces the binary and installs all shipped templates, but preserves
`/etc/pelcodtui/state.conf`.

The profile comments identify the source manuals and note conflicting ranges.
The PDF archive used to prepare them is local reference material and is not
included in the repository.

Keys: arrows select settings and Enter opens or applies them. Preset control is
the first menu item and TUI pulse settings are the last. WASD moves pan/tilt,
Z/X controls zoom, N/F controls focus, Space sends STOP, and Q exits.

## Safety

Factory reset, preset deletion, and pan/tilt correction require confirmation.
Raw preset control also confirms documented destructive commands in known
camera profiles. Unknown-camera mode keeps unrestricted preset access.
Movement commands are timed and finish with three STOP frames at 10 ms spacing.
Press the same movement key again before it stops to add another pulse, up to
five seconds total.
