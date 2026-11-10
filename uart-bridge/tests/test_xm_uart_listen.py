"""xm-uart's network relay mode (-l), against a pty standing in for the lens
board's UART. Builds xm-uart/main.c with the host compiler; skipped without one."""

import os
import shutil
import socket
import subprocess
import time
from pathlib import Path

import pytest

MAIN_C = Path(__file__).resolve().parents[2] / "xm-uart" / "main.c"
ZOOM_IN = bytes.fromhex("c50100200000215c")
STOP = bytes.fromhex("c50100000000015c")
A5 = bytes.fromhex("a5319ef75cb15996")
REPLY = bytes.fromhex("ef01000904032f2e58312e3320")


@pytest.fixture(scope="module")
def xm_uart(tmp_path_factory):
    cc = shutil.which("cc") or shutil.which("gcc")
    if not cc:
        pytest.skip("no host C compiler")
    if not MAIN_C.exists():
        pytest.skip("xm-uart/main.c not present (uart-bridge copied on its own)")
    out = tmp_path_factory.mktemp("xm") / "xm-uart"
    subprocess.run([cc, "-O2", "-o", str(out), str(MAIN_C)], check=True)
    return out


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def read_board(fd, n, timeout=1.0):
    os.set_blocking(fd, False)
    buf, end = b"", time.monotonic() + timeout
    while len(buf) < n and time.monotonic() < end:
        try:
            buf += os.read(fd, n - len(buf))
        except BlockingIOError:
            time.sleep(0.005)
    return buf


def start_relay(xm_uart, *extra):
    board, slave = os.openpty()
    port = free_port()
    proc = subprocess.Popen([str(xm_uart), "-d", os.ttyname(slave), "-l", str(port), *extra],
                            stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    for _ in range(100):  # wait for the listener
        try:
            socket.create_connection(("127.0.0.1", port), timeout=0.1).close()
            break
        except OSError:
            time.sleep(0.02)
    read_board(board, 64, timeout=0.3)  # the probe connection's disconnect sends a stop
    return board, slave, port, proc


def stop_relay(board, slave, proc):
    proc.terminate()
    proc.wait(timeout=3)
    os.close(board)
    os.close(slave)


@pytest.fixture
def relay(xm_uart):
    board, slave, port, proc = start_relay(xm_uart, "-a", "127.0.0.1")
    yield board, port, proc
    stop_relay(board, slave, proc)


def connect(port):
    s = socket.create_connection(("127.0.0.1", port), timeout=1)
    s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    time.sleep(0.05)
    return s


def test_whole_and_split_frames_reach_the_board_whole(relay):
    board, port, proc = relay
    s = connect(port)
    s.sendall(A5 + ZOOM_IN[:3])
    time.sleep(0.05)
    s.sendall(ZOOM_IN[3:])
    assert read_board(board, 16) == A5 + ZOOM_IN
    assert proc.poll() is None                       # stdin was /dev/null: still running
    s.close()


def test_stale_partial_and_junk_are_never_written(relay):
    board, port, _ = relay
    s = connect(port)
    s.sendall(b"hello" + ZOOM_IN[:5])                 # junk, then a frame that never completes
    time.sleep(0.5)                                  # longer than the 300 ms partial timeout
    s.sendall(STOP)
    assert read_board(board, 16, timeout=0.5) == STOP  # only the whole frame came through
    s.close()


def test_board_replies_go_back_to_the_client(relay):
    board, port, _ = relay
    s = connect(port)
    os.write(board, REPLY)
    s.settimeout(1)
    got = b""
    while len(got) < len(REPLY):
        got += s.recv(64)
    assert got == REPLY
    s.close()


def test_disconnect_sends_stop_and_second_client_is_refused(relay):
    board, port, _ = relay
    s = connect(port)
    s2 = connect(port)
    s2.settimeout(1)
    assert s2.recv(8) == b""                          # refused: closed at once
    s.sendall(ZOOM_IN)
    assert read_board(board, 8) == ZOOM_IN
    s.close()                                        # sender vanishes mid-move
    assert read_board(board, 8) == STOP


def test_late_tail_does_not_complete_an_expired_frame(relay):
    board, port, _ = relay
    s = connect(port)
    s.sendall(ZOOM_IN[:5])
    time.sleep(0.35)                                 # past the 300 ms partial-frame deadline
    s.sendall(ZOOM_IN[5:] + STOP)                    # the late tail arrives together with a stop
    assert read_board(board, 16, timeout=0.5) == STOP
    s.close()


def test_client_from_another_address_is_refused(xm_uart):
    board, slave, port, proc = start_relay(xm_uart, "-a", "10.255.255.1")
    try:
        s = socket.create_connection(("127.0.0.1", port), timeout=1)
        s.settimeout(1)
        assert s.recv(8) == b""                      # closed at once: not the allowed client
        s.close()
        assert read_board(board, 8, timeout=0.3) == b""
    finally:
        stop_relay(board, slave, proc)


def test_truncated_frame_then_valid_frame(relay):
    board, port, _ = relay
    s = connect(port)
    s.sendall(ZOOM_IN[:5])                           # sender dies mid-frame...
    time.sleep(0.05)                                 # ...and a new one starts well inside 300 ms
    s.sendall(STOP)
    assert read_board(board, 16, timeout=0.5) == STOP
    s.close()


def test_spliced_frame_in_one_send_is_rejected(relay):
    board, port, _ = relay
    s = connect(port)
    s.sendall(ZOOM_IN[:5] + STOP)                    # c5 01 00 20 00 c5 01 00 | 00 00 01 5c
    assert read_board(board, 16, timeout=0.5) == STOP  # resynced on the second C5
    s.close()


def test_flood_reaches_the_board_as_whole_frames_only(relay):
    board, port, _ = relay
    s = connect(port)
    s.sendall((ZOOM_IN + A5) * 500)                  # far more than the pty will buffer unread
    time.sleep(0.5)
    got = read_board(board, 200_000, timeout=1.5)
    s.close()
    got += read_board(board, 64, timeout=0.3)        # the disconnect's stop
    assert got and len(got) % 8 == 0
    for i in range(0, len(got), 8):
        f = got[i:i + 8]
        assert f in (ZOOM_IN, A5, STOP), f"not a whole frame at {i}: {f.hex(' ')}"
