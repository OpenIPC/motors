## xm-uart

Interactive driver for the zoom/focus lens board of Xiongmai AF camera modules. It speaks the XM Pelco-D variant over the camera's UART, at 115200 8N1.

The wire protocol is specified in [PROTOCOL.md](PROTOCOL.md). That spec was measured against the stock camera firmware and the real board with [`uart-bridge`](../uart-bridge/), and [`captures/`](captures/) holds the recordings it rests on.

### Usage

```sh
xm-uart-motors-openipc [-d /dev/ttyAMA0]
```

Disable the system getty on the UART first.

| Key | Action |
|---|---|
| `+` / `-` | zoom in / out |
| `z` / `x` | focus near / far (stock firmware naming, see PROTOCOL.md) |
| `h` / `l` (Colemak `i`) | pan left / right |
| `j` / `k` (Colemak `n` / `e`) | tilt down / up |
| Space, Enter | stop |
| `q` | stop and quit |

A move continues until stop. The tool also sends stop whenever it exits: on `q`, Ctrl-C, SIGTERM or SIGHUP, and when stdin closes.

Zoom reports from the board are printed as `Zoom X1.4`.

### Build

```sh
make xm-uart-motors-openipc   # OpenIPC musl toolchain
make xm-uart-motors-orig      # HiSilicon vendor toolchain
make xm-uart-motors-host      # native, e.g. to run under uart-bridge --cam pty
```

### Checking a change against the stock firmware

On a host where the camera board and the lens board each go to a USB-UART adapter:

```sh
cd uart-bridge
uv run uart-bridge bridge --cam pty --log mine.jsonl   # prints the pty path to use
../xm-uart/xm-uart-motors-host -d /dev/pts/N           # in another terminal
uv run uart-bridge diff ../xm-uart/captures/e1-stock.jsonl mine.jsonl
```
