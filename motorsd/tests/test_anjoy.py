import select
#!/usr/bin/env python3

import json
import os
import pathlib
import pty
import socket
import subprocess
import tempfile
import time

ROOT = pathlib.Path(__file__).resolve().parents[1]
DAEMON = ROOT / "build" / "motorsd"
CLIENT = ROOT / "build" / "motorsctl"
MOTORS_CLI = ROOT / "build" / "motors-cli"
ANJOY_DRIVER = ROOT / "build" / "anjoy_driver"
ANJOY_CONF = ROOT / "config" / "anjoy.conf"

def read_exact(fd, n, timeout=2.0):
    deadline = time.monotonic() + timeout
    buf = b""
    while len(buf) < n and time.monotonic() < deadline:
        chunk = os.read(fd, n - len(buf))
        if chunk:
            buf += chunk
    assert len(buf) == n, f"Expected {n} bytes, got {len(buf)}: {buf.hex()}"
    return buf

def test_anjoy_driver():
    with tempfile.TemporaryDirectory(prefix="anjoy-test-") as tempdir:
        temp = pathlib.Path(tempdir)
        master, slave = pty.openpty()
        slave_name = os.ttyname(slave)

        conf_file = temp / "anjoy.conf"
        conf_file.write_text(f"""device={slave_name}
baud=57600
pan_speed=20
tilt_speed=20
keepalive_ms=200
""", encoding="utf-8")

        parent, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        driver = subprocess.Popen(
            [str(ANJOY_DRIVER), "--fd", str(child.fileno()), "--config", str(conf_file)],
            pass_fds=(child.fileno(),), stderr=subprocess.PIPE, text=True,
        )
        child.close()

        try:
            # 1. Startup: initial keepalive (16 bytes) and safe stop (7 bytes)
            # Initial keepalive frame
            keepalive = read_exact(master, 16)
            assert keepalive[:4] == bytes.fromhex("51010478"), f"Bad keepalive: {keepalive.hex()}"
            sum_expected = sum(keepalive[:15]) & 0xff
            assert keepalive[15] == sum_expected, "Bad keepalive checksum"

            # Initial safe stop frame
            stop_frame = read_exact(master, 7)
            assert stop_frame == bytes.fromhex("ff010000000001"), f"Bad stop: {stop_frame.hex()}"

            # 2. Capabilities
            parent.send(json.dumps({"version": 1, "id": "caps", "op": "capabilities"}).encode())
            reply = json.loads(parent.recv(4096))
            assert reply["ok"] is True
            assert reply["name"] == "anjoy"
            assert reply["axes"] == 0x1f
            assert reply["stop_domains"] == [0x1f] * 5

            # 3. Move Pan Left (axis 0, direction left)
            # Pan left: C1=00, C2=04, D1=20 (0x14), D2=00, CK=(01+00+04+14+00)=0x19
            parent.send(json.dumps({"version": 1, "id": "m1", "op": "move", "axis": 0, "direction": "left", "duration_ms": 0}).encode())
            reply = json.loads(parent.recv(4096))
            assert reply["ok"] is True
            pan_left_frame = read_exact(master, 7)
            assert pan_left_frame == bytes.fromhex("ff010004140019"), f"Bad pan left: {pan_left_frame.hex()}"

            # 4. Move Tilt Up (axis 1, direction up)
            # Tilt up: C1=00, C2=08, D1=00, D2=20 (0x14), CK=(01+00+08+00+14)=0x1d
            parent.send(json.dumps({"version": 1, "id": "m2", "op": "move", "axis": 1, "direction": "up", "duration_ms": 0}).encode())
            reply = json.loads(parent.recv(4096))
            assert reply["ok"] is True
            tilt_up_frame = read_exact(master, 7)
            assert tilt_up_frame == bytes.fromhex("ff01000800141d"), f"Bad tilt up: {tilt_up_frame.hex()}"

            # 5. Move Zoom Tele (axis 2, direction tele)
            # Zoom tele: C1=00, C2=20, D1=14, D2=14, CK=(01+00+20+14+14)=0x49
            parent.send(json.dumps({"version": 1, "id": "m3", "op": "move", "axis": 2, "direction": "tele", "duration_ms": 0}).encode())
            reply = json.loads(parent.recv(4096))
            assert reply["ok"] is True
            zoom_tele_frame = read_exact(master, 7)
            assert zoom_tele_frame == bytes.fromhex("ff010020141449"), f"Bad zoom tele: {zoom_tele_frame.hex()}"

            # 6. Move Focus Near (axis 3, direction near)
            # Focus near: C1=01, C2=00, D1=30, D2=30, CK=(01+01+00+30+30)=0x62
            parent.send(json.dumps({"version": 1, "id": "m4", "op": "move", "axis": 3, "direction": "near", "duration_ms": 0}).encode())
            reply = json.loads(parent.recv(4096))
            assert reply["ok"] is True
            focus_near_frame = read_exact(master, 7)
            assert focus_near_frame == bytes.fromhex("ff010100303062"), f"Bad focus near: {focus_near_frame.hex()}"

            # 7. Move Iris Open (axis 4, direction open)
            # Iris open: C1=02, C2=00, D1=30, D2=30, CK=(01+02+00+30+30)=0x63
            parent.send(json.dumps({"version": 1, "id": "m5", "op": "move", "axis": 4, "direction": "open", "duration_ms": 0}).encode())
            reply = json.loads(parent.recv(4096))
            assert reply["ok"] is True
            iris_open_frame = read_exact(master, 7)
            assert iris_open_frame == bytes.fromhex("ff010200303063"), f"Bad iris open: {iris_open_frame.hex()}"

            # 8. Stop
            parent.send(json.dumps({"version": 1, "id": "s1", "op": "stop", "axes": 0x1f}).encode())
            reply = json.loads(parent.recv(4096))
            assert reply["ok"] is True
            stop_frame = read_exact(master, 7)
            assert stop_frame == bytes.fromhex("ff010000000001"), f"Bad stop: {stop_frame.hex()}"

            # 8b. Preset commands (preset.call, preset.set, preset.clear, and invalid preset/ircut validation)
            parent.send(json.dumps({"version": 1, "id": "p1", "op": "command", "name": "preset.call", "value": "5"}).encode())
            reply = json.loads(parent.recv(4096))
            assert reply["ok"] is True
            assert read_exact(master, 7) == bytes.fromhex("ff01000700050d")

            parent.send(json.dumps({"version": 1, "id": "p2", "op": "command", "name": "preset.set", "value": "5"}).encode())
            reply = json.loads(parent.recv(4096))
            assert reply["ok"] is True
            assert read_exact(master, 7) == bytes.fromhex("ff010003000509")

            parent.send(json.dumps({"version": 1, "id": "p3", "op": "command", "name": "preset", "value": "0"}).encode())
            reply = json.loads(parent.recv(4096))
            assert reply["ok"] is False

            parent.send(json.dumps({"version": 1, "id": "ic1", "op": "command", "name": "ircut", "value": "bogus"}).encode())
            reply = json.loads(parent.recv(4096))
            assert reply["ok"] is False

            # 9. Telemetry report from MCU:
            # 15 bytes: 51 01 04 78 01 08 0B 1A ZZ ZZ 21 26 26 26 CK
            # Zoom 25x: byte 8 = 0x02, byte 9 = 0x05
            report = bytearray(bytes.fromhex("5101047801080b1a02052126262600"))
            report[14] = sum(report[:14]) & 0xff
            os.write(master, report)

            # Check if telemetry event received
            parent.settimeout(1.0)
            telemetry_pkt = json.loads(parent.recv(4096))
            assert telemetry_pkt["event"] == "telemetry"
            assert telemetry_pkt["name"] == "zoom_magnification"
            assert abs(telemetry_pkt["value"] - 25.0) < 0.05, f"Unexpected mag: {telemetry_pkt['value']}"

            # Zoom 60x: byte 8 = 0x06, byte 9 = 0x00
            report = bytearray(bytes.fromhex("5101047801080b1a06002126262600"))
            report[14] = sum(report[:14]) & 0xff
            os.write(master, report)
            telemetry_pkt = json.loads(parent.recv(4096))
            assert telemetry_pkt["event"] == "telemetry"
            assert abs(telemetry_pkt["value"] - 60.0) < 0.05, f"Unexpected mag: {telemetry_pkt['value']}"

            # 14-byte sub-1.0 overshoot report: 51 01 04 78 01 07 0B 1A 08 21 26 26 26 CK -> clamped to 1.0
            report = bytearray(bytes.fromhex("5101047801070b1a082126262600"))
            report[13] = sum(report[:13]) & 0xff
            os.write(master, report)
            telemetry_pkt = json.loads(parent.recv(4096))
            assert telemetry_pkt["event"] == "telemetry"
            assert abs(telemetry_pkt["value"] - 1.0) < 0.05, f"Unexpected mag: {telemetry_pkt['value']}"

            # Dummy/uninitialized report (0x26 bytes when moving at endstop): must be ignored
            report = bytearray(bytes.fromhex("5101047801080b1a26262626262600"))
            report[14] = sum(report[:14]) & 0xff
            os.write(master, report)
            parent.settimeout(0.1)
            try:
                unexpected = parent.recv(4096)
                assert False, f"Dummy frame should not produce telemetry: {unexpected}"
            except socket.timeout:
                pass

            # 10. Timed move auto-stop
            parent.send(json.dumps({"version": 1, "id": "t1", "op": "move", "axis": 0, "direction": "right", "duration_ms": 50}).encode())
            reply = json.loads(parent.recv(4096))
            assert reply["ok"] is True
            _ = read_exact(master, 7) # Move frame
            time.sleep(0.1)
            # Expect stop frame and movement_ended event
            stop_timed = read_exact(master, 7)
            assert stop_timed == bytes.fromhex("ff010000000001")
            ended_event = json.loads(parent.recv(4096))
            assert ended_event["event"] == "movement_ended"
            assert ended_event["axes"] == 0x1f

        finally:
            parent.close()
            driver.terminate()
            driver.wait(timeout=2)
            os.close(master)
            os.close(slave)

def test_anjoy_with_daemon():
    with tempfile.TemporaryDirectory(prefix="anjoy-daemon-test-") as tempdir:
        temp = pathlib.Path(tempdir)
        master, slave = pty.openpty()
        slave_name = os.ttyname(slave)

        conf_file = temp / "anjoy.conf"
        conf_file.write_text(f"""device={slave_name}
baud=57600
pan_speed=20
tilt_speed=20
keepalive_ms=300
""", encoding="utf-8")

        sock_path = temp / "motorsd.sock"
        daemon = subprocess.Popen(
            [str(DAEMON), "--socket", str(sock_path), "--driver", str(ANJOY_DRIVER), "--driver-config", str(conf_file)],
            stderr=subprocess.PIPE, text=True,
        )

        time.sleep(0.2)
        try:
            # Drain initial keepalive + stop frame and any boot-time AF cancel frames
            while True:
                r, _, _ = select.select([master], [], [], 0.05)
                if not r: break
                os.read(master, 256)

            # Connect client
            sock = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
            sock.connect(str(sock_path))

            # Query capabilities
            sock.send(json.dumps({"version": 1, "id": "c1", "op": "capabilities"}).encode())
            caps = json.loads(sock.recv(4096))
            assert caps["ok"] is True
            assert caps["driver"] == "anjoy"
            assert "pan" in caps["axes"] and "tilt" in caps["axes"] and "zoom" in caps["axes"]

            # Acquire and move pan
            sock.send(json.dumps({"version": 1, "id": "a1", "op": "acquire", "role": "manual", "axis": "pan", "lease_ms": 2000}).encode())
            acq = json.loads(sock.recv(4096))
            assert acq["ok"] is True

            sock.send(json.dumps({"version": 1, "id": "m1", "op": "move", "axis": "pan", "direction": "left"}).encode())
            m1 = json.loads(sock.recv(4096))
            assert m1["ok"] is True
            pan_f = read_exact(master, 7)
            assert pan_f == bytes.fromhex("ff010004140019")

            sock.send(json.dumps({"version": 1, "id": "s1", "op": "stop", "axis": "pan"}).encode())
            s1 = json.loads(sock.recv(4096))
            assert s1["ok"] is True
            stop_f = read_exact(master, 7)
            assert stop_f == bytes.fromhex("ff010000000001")

            sock.close()

            # Test motors-cli
            res = subprocess.run([str(MOTORS_CLI), "--socket", str(sock_path), "tele", "100"], capture_output=True, text=True)
            assert res.returncode == 0, f"motors-cli failed: {res.stderr}"
            tele_f = read_exact(master, 7)
            assert tele_f == bytes.fromhex("ff010020141449")
            
            # Read until timed auto-stop frame arrives (skipping periodic keepalives if any)
            incoming = b""
            stop_frame = bytes.fromhex("ff010000000001")
            while stop_frame not in incoming:
                chunk = os.read(master, 64)
                if not chunk:
                    break
                incoming += chunk
            assert stop_frame in incoming
        finally:
            daemon.terminate()
            daemon.wait(timeout=2)
            os.close(master)
            os.close(slave)

if __name__ == "__main__":
    print("Running test_anjoy_driver()...")
    test_anjoy_driver()
    print("PASS: test_anjoy_driver()")
    print("Running test_anjoy_with_daemon()...")
    test_anjoy_with_daemon()
    print("PASS: test_anjoy_with_daemon()")
    print("ALL ANJOY TESTS PASSED!")
