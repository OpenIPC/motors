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
DRIVER = ROOT / "build" / "mock_driver"
PELCOD_DRIVER = ROOT / "build" / "pelcod_driver"
H07_PROFILE = ROOT.parent / "pelcodtui" / "cameras" / "h07-hieasy.conf"
LIBMOTORS_CLIENT = ROOT / "build" / "test_libmotors"
LIBMOTORS_PREEMPTION = ROOT / "build" / "test_libmotors_preemption"


class Client:
    def __init__(self, path):
        self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        self.socket.settimeout(1)
        self.socket.connect(str(path))
        self.events = []

    def request(self, request):
        self.socket.send(json.dumps(request, separators=(",", ":")).encode())
        while True:
            message = json.loads(self.socket.recv(32768))
            if "event" in message:
                self.events.append(message)
                continue
            return message

    def close(self):
        self.socket.close()

    def receive(self):
        if self.events:
            return self.events.pop(0)
        return json.loads(self.socket.recv(32768))


def expect(response, ok=True, state=None, error=None):
    assert response["ok"] is ok, response
    if state is not None:
        assert response.get("state") == state, response
    if error is not None:
        assert response.get("error") == error, response


def wait_for(predicate, description, timeout=1.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError(f"timed out waiting for {description}")


def wait_for_daemon(daemon, socket_path, timeout=1.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if socket_path.exists():
            return
        if daemon.poll() is not None:
            raise AssertionError(
                f"daemon exited with {daemon.returncode}: {daemon.stderr.read().strip()}"
            )
        time.sleep(0.01)
    raise AssertionError("timed out waiting for daemon socket")


def read_lines(path):
    if not path.exists():
        return []
    return path.read_text(encoding="utf-8").splitlines()


def run_test():
    with tempfile.TemporaryDirectory(prefix="motorsd-test-") as directory:
        temp = pathlib.Path(directory)
        socket_path = temp / "motorsd.sock"
        trace_path = temp / "driver.trace"
        config_path = temp / "mock.conf"
        config_path.write_text(f"trace={trace_path}\n", encoding="utf-8")

        daemon = subprocess.Popen(
            [str(DAEMON), "--socket", str(socket_path), "--driver", str(DRIVER),
             "--driver-config", str(config_path)],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        af = None
        manual = None
        observer = None
        stopper = None
        try:
            wait_for_daemon(daemon, socket_path)

            result = subprocess.run(
                [str(CLIENT), "--socket", str(socket_path),
                 '{"version":1,"id":"caps","op":"capabilities"}'],
                check=True, capture_output=True, text=True,
            )
            caps = json.loads(result.stdout)
            assert caps["driver"] == "mock", caps
            assert caps["axes"] == ["pan", "tilt", "zoom", "focus"], caps
            assert caps["available"] is True, caps

            described = Client(socket_path)
            description = described.request(
                {"version": 1, "id": "describe", "op": "describe"}
            )
            expect(description)
            assert description["controls"][0]["name"] == "test.level", description
            expect(described.request({"version": 1, "id": "command",
                                      "op": "command", "name": "test.level",
                                      "value": 5}), state="command_sent")
            described.close()
            assert "COMMAND name=test.level value=5" in read_lines(trace_path)

            library_af = subprocess.Popen(
                [str(LIBMOTORS_PREEMPTION), str(socket_path)],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
            )
            assert library_af.stdout.readline().strip() == "READY"
            library_manual = Client(socket_path)
            expect(library_manual.request({"version": 1, "id": "library-preempt",
                                           "op": "acquire", "role": "manual",
                                           "axis": "focus", "lease_ms": 1000}),
                   state="acquired:preempted")
            library_manual.close()
            assert library_af.wait(timeout=2) == 0, library_af.stderr.read()

            sequence = subprocess.run(
                [str(CLIENT), "--socket", str(socket_path), "--wait-event",
                 '{"version":1,"id":"ctl-subscribe","op":"subscribe"}',
                 '{"version":1,"id":"ctl-acquire","op":"acquire",'
                 '"role":"manual","axis":"zoom","lease_ms":1000}',
                 '{"version":1,"id":"ctl-move","op":"move",'
                 '"axis":"zoom","direction":"tele","duration_ms":10}'],
                check=True, capture_output=True, text=True,
            )
            sequence_replies = [json.loads(line) for line in sequence.stdout.splitlines()]
            assert [reply.get("state") for reply in sequence_replies] == [
                "subscribed", "acquired", "start_sent", None
            ], sequence_replies
            assert sequence_replies[-1]["event"] == "movement_ended", sequence_replies

            subprocess.run([str(LIBMOTORS_CLIENT), str(socket_path)], check=True)
            assert "MOVE axis=3 direction=near duration_ms=20" in read_lines(trace_path)

            af = Client(socket_path)
            expect(af.request({"version": 2, "id": "bad-version", "op": "capabilities"}),
                   ok=False, error="unsupported version")
            expect(af.request({"version": 1, "id": "no-timeout", "op": "acquire",
                               "role": "af", "axis": "focus"}),
                   ok=False, error="positive lease_ms required")
            expect(af.request({"version": 1, "id": "af-acquire", "op": "acquire",
                               "role": "af", "axis": "focus", "lease_ms": 5000}),
                   state="acquired")
            expect(af.request({"version": 1, "id": "af-subscribe", "op": "subscribe"}),
                   state="subscribed")
            af_move = af.request({"version": 1, "id": "af-move", "op": "move",
                                  "axis": "focus", "direction": "far"})
            expect(af_move, state="start_sent")
            assert isinstance(af_move.get("driver_completed_mono_ms"), int), af_move

            manual = Client(socket_path)
            observer = Client(socket_path)
            expect(observer.request({"version": 1, "id": "subscribe",
                                     "op": "subscribe"}), state="subscribed")
            expect(manual.request({"version": 1, "id": "lower-priority", "op": "acquire",
                                   "role": "automation", "axis": "focus", "lease_ms": 5000}),
                   ok=False, error="busy:af")
            expect(manual.request({"version": 1, "id": "raw-busy", "op": "raw",
                                   "payload": {"transport": "uart", "bytes": "ff00"}}),
                   ok=False, error="driver owned by another client")
            expect(manual.request({"version": 1, "id": "manual-acquire", "op": "acquire",
                                   "role": "manual", "axis": "focus", "lease_ms": 5000}),
                   state="acquired:preempted")
            revoked = af.receive()
            assert revoked["event"] == "lease_revoked", revoked
            assert revoked["axes"] == 0x08 and revoked["reason"] == "preempted", revoked

            lines = read_lines(trace_path)
            move_index = lines.index("MOVE axis=3 direction=far duration_ms=0")
            preempt_stop_index = lines.index("STOP axes=0x08", move_index + 1)
            assert preempt_stop_index > move_index, lines

            stop_count = read_lines(trace_path).count("STOP axes=0x08")
            expect(manual.request({"version": 1, "id": "timed-move", "op": "move",
                                   "axis": "focus", "direction": "near", "duration_ms": 80}),
                   state="start_sent")
            wait_for(lambda: read_lines(trace_path).count("STOP axes=0x08") > stop_count,
                     "timed movement STOP")
            event = observer.receive()
            assert event["event"] == "movement_ended", event
            assert event["axes"] == 0x08, event
            assert isinstance(event.get("driver_completed_mono_ms"), int), event

            expect(manual.request({"version": 1, "id": "raw", "op": "raw",
                                   "payload": {"transport": "uart", "bytes": "ff00"}}),
                   state="raw_sent")
            expect(manual.request({"version": 1, "id": "continuous", "op": "move",
                                   "axis": "focus", "direction": "far"}),
                   state="start_sent")
            expect(manual.request({"version": 1, "id": "manual-subscribe",
                                   "op": "subscribe"}), state="subscribed")
            stopper = Client(socket_path)
            expect(stopper.request({"version": 1, "id": "normal-stop", "op": "stop",
                                    "axis": "focus"}), state="stop_sent")
            stopped = manual.receive()
            assert stopped["event"] == "movement_ended", stopped
            assert stopped["axes"] == 0x08, stopped
            assert isinstance(stopped.get("driver_completed_mono_ms"), int), stopped
            stopper.close()
            stopper = None
            expect(manual.request({"version": 1, "id": "move-after-stop", "op": "move",
                                   "axis": "focus", "direction": "far"}),
                   state="start_sent")
            stop_count = read_lines(trace_path).count("STOP axes=0x08")
            manual.close()
            manual = None
            wait_for(lambda: read_lines(trace_path).count("STOP axes=0x08") > stop_count,
                     "disconnect STOP")

            stop_count = read_lines(trace_path).count("STOP axes=0x08")
            expect(af.request({"version": 1, "id": "short-lease", "op": "acquire",
                               "role": "af", "axis": "focus", "lease_ms": 80}),
                   state="acquired")
            expect(af.request({"version": 1, "id": "lease-move", "op": "move",
                               "axis": "focus", "direction": "near"}),
                   state="start_sent")
            wait_for(lambda: read_lines(trace_path).count("STOP axes=0x08") > stop_count,
                     "lease expiry STOP")

            expect(af.request({"version": 1, "id": "crash-acquire", "op": "acquire",
                               "role": "af", "axis": "focus", "lease_ms": 5000}),
                   state="acquired")
            crash = af.request({"version": 1, "id": "crash", "op": "raw",
                                "payload": {"crash": True}})
            expect(crash, ok=False, error="driver disconnected")

            request_number = 0

            def driver_available():
                nonlocal request_number
                request_number += 1
                response = af.request({"version": 1, "id": f"recovery-{request_number}",
                                       "op": "capabilities"})
                return response.get("available") is True

            wait_for(driver_available, "driver recovery")
            assert read_lines(trace_path).count("SAFE_START") == 2, read_lines(trace_path)
            expect(af.request({"version": 1, "id": "old-lease", "op": "move",
                               "axis": "focus", "direction": "near"}),
                   ok=False, error="lease required")

            lines = read_lines(trace_path)
            assert 'RAW payload={"transport":"uart","bytes":"ff00"}' in lines, lines
            assert daemon.poll() is None, daemon.stderr.read()
        finally:
            if manual is not None:
                manual.close()
            if af is not None:
                af.close()
            if observer is not None:
                observer.close()
            if stopper is not None:
                stopper.close()
            daemon.terminate()
            try:
                daemon.wait(timeout=2)
            except subprocess.TimeoutExpired:
                daemon.kill()
                daemon.wait(timeout=2)

        assert daemon.returncode == 0, daemon.stderr.read()
        assert read_lines(trace_path)[-1] == "CLOSE", read_lines(trace_path)

        control_trace = temp / "control-loss.trace"
        control_config = temp / "control-loss.conf"
        control_config.write_text(f"trace={control_trace}\n", encoding="utf-8")
        parent, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        driver = subprocess.Popen(
            [str(DRIVER), "--fd", str(child.fileno()), "--config", str(control_config)],
            pass_fds=(child.fileno(),), stderr=subprocess.PIPE, text=True,
        )
        child.close()
        parent.send(json.dumps({"version": 1, "id": "move", "op": "move",
                                "axis": 3, "direction": "near", "duration_ms": 0}).encode())
        expect(json.loads(parent.recv(4096)), state="sent")
        parent.close()
        driver.wait(timeout=1)
        assert driver.returncode == 0, driver.stderr.read()
        control_lines = read_lines(control_trace)
        assert "STOP axes=0x08" in control_lines, control_lines
        assert control_lines[-2:] == ["CONTROL_LOST", "CLOSE"], control_lines

        bad_config = temp / "bad.conf"
        bad_config.write_text("unknown=value\n", encoding="utf-8")
        failed = subprocess.run(
            [str(DAEMON), "--socket", str(socket_path), "--driver", str(DRIVER),
             "--driver-config", str(bad_config)],
            capture_output=True, text=True,
        )
        assert failed.returncode != 0, failed
        assert "driver startup failed" in failed.stderr, failed.stderr

    with tempfile.TemporaryDirectory(prefix="pelcod-profile-test-") as directory:
        temp = pathlib.Path(directory)
        master, slave = pty.openpty()
        profile = temp / "h07.conf"
        profile.write_text(
            H07_PROFILE.read_text(encoding="utf-8").replace(
                "device=/dev/ttyAMA0", f"device={os.ttyname(slave)}"
            ),
            encoding="utf-8",
        )
        parent, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        driver = subprocess.Popen(
            [str(PELCOD_DRIVER), "--fd", str(child.fileno()), "--config", str(profile)],
            pass_fds=(child.fileno(),), stderr=subprocess.PIPE, text=True,
        )
        child.close()

        def read_profile_frames(length):
            data = b""
            deadline = time.monotonic() + 1
            while len(data) < length and time.monotonic() < deadline:
                data += os.read(master, length - len(data))
            assert len(data) == length, data
            return data

        try:
            assert read_profile_frames(7) == bytes.fromhex("ff010000000001")
            parent.send(json.dumps({"version": 1, "id": "describe",
                                    "op": "describe"}).encode())
            description_packet = parent.recv(32768)
            assert len(description_packet) > 4096, len(description_packet)
            description = json.loads(description_packet)
            expect(description)
            assert description["profile"] == "h07-hieasy", description
            assert any(menu["label"] == "Accessories"
                       for menu in description["menus"]), description
            control = next(item for item in description["controls"]
                           if item["name"] == "ir.brightness")
            assert control["name"] == "ir.brightness", control
            assert control["min"] == 1 and control["max"] == 10, control

            parent.send(json.dumps({"version": 1, "id": "ir", "op": "command",
                                    "name": "ir.brightness", "value": 5}).encode())
            expect(json.loads(parent.recv(4096)), state="sent")
            assert read_profile_frames(14) == bytes.fromhex(
                "ff010003007a7e" "ff010003000509"
            )
        finally:
            parent.close()
            driver.wait(timeout=1)
            os.close(slave)
            os.close(master)
        assert driver.returncode == 0, driver.stderr.read()


def test_pelcod_driver():
    with tempfile.TemporaryDirectory(prefix="pelcod-driver-test-") as directory:
        temp = pathlib.Path(directory)
        master, slave = pty.openpty()
        config = temp / "pelcod.conf"
        config.write_text(
            f"device={os.ttyname(slave)}\nbaud=115200\naddress=1\n"
            "pan_speed=32\ntilt_speed=32\nstop_repeat=3\nstop_delay_ms=2\n",
            encoding="utf-8",
        )
        parent, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        driver = subprocess.Popen(
            [str(PELCOD_DRIVER), "--fd", str(child.fileno()), "--config", str(config)],
            pass_fds=(child.fileno(),), stderr=subprocess.PIPE, text=True,
        )
        child.close()

        def read_exact(length):
            data = b""
            deadline = time.monotonic() + 1
            while len(data) < length and time.monotonic() < deadline:
                data += os.read(master, length - len(data))
            assert len(data) == length, data
            return data

        try:
            assert read_exact(21) == bytes.fromhex("ff010000000001") * 3
            os.close(slave)
            slave = -1
            parent.send(json.dumps({"version": 1, "id": "caps",
                                    "op": "capabilities"}).encode())
            caps = json.loads(parent.recv(4096))
            assert caps["name"] == "pelcod", caps
            assert caps["stop_domains"] == [15, 15, 15, 15, 15], caps

            parent.send(json.dumps({"version": 1, "id": "move", "op": "move",
                                    "axis": 3, "direction": "near",
                                    "duration_ms": 20}).encode())
            expect(json.loads(parent.recv(4096)), state="sent")
            assert read_exact(7) == bytes.fromhex("ff010100000002")
            assert read_exact(21) == bytes.fromhex("ff010000000001") * 3
            event = json.loads(parent.recv(4096))
            assert event["event"] == "movement_ended", event
            assert event["axes"] == 15, event

            parent.send(json.dumps({"version": 1, "id": "raw", "op": "raw",
                                    "payload": {"bytes": "ff010007007a82"}}).encode())
            expect(json.loads(parent.recv(4096)), state="sent")
            assert read_exact(7) == bytes.fromhex("ff010007007a82")
        finally:
            parent.close()
            driver.wait(timeout=1)
            if slave >= 0:
                os.close(slave)
            os.close(master)
        assert driver.returncode == 0, driver.stderr.read()


def test_failed_driver_recovery():
    with tempfile.TemporaryDirectory(prefix="motorsd-recovery-test-") as directory:
        temp = pathlib.Path(directory)
        socket_path = temp / "motorsd.sock"
        trace_path = temp / "driver.trace"
        config_path = temp / "mock.conf"
        marker_path = temp / "first-driver-exited"
        attempts_path = temp / "driver-attempts"
        wrapper_path = temp / "driver-wrapper"
        config_path.write_text(f"trace={trace_path}\n", encoding="utf-8")
        wrapper_path.write_text(
            "#!/bin/sh\n"
            f"echo start >> '{attempts_path}'\n"
            f"if [ -e '{marker_path}' ]; then exit 42; fi\n"
            f"'{DRIVER}' \"$@\"\n"
            "result=$?\n"
            f"touch '{marker_path}'\n"
            "exit $result\n",
            encoding="utf-8",
        )
        wrapper_path.chmod(0o755)

        daemon = subprocess.Popen(
            [str(DAEMON), "--socket", str(socket_path), "--driver", str(wrapper_path),
             "--driver-config", str(config_path)],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        client = None
        try:
            wait_for_daemon(daemon, socket_path)
            client = Client(socket_path)
            expect(client.request({"version": 1, "id": "subscribe",
                                   "op": "subscribe"}), state="subscribed")
            expect(client.request({"version": 1, "id": "lease", "op": "acquire",
                                   "role": "af", "axis": "focus", "lease_ms": 5000}),
                   state="acquired")
            expect(client.request({"version": 1, "id": "move", "op": "move",
                                   "axis": "focus", "direction": "near"}),
                   state="start_sent")
            expect(client.request({"version": 1, "id": "crash", "op": "raw",
                                   "payload": {"crash": True}}),
                   ok=False, error="driver disconnected")

            response = client.request({"version": 1, "id": "caps",
                                       "op": "capabilities"})
            expect(response)
            assert response["available"] is False, response
            assert len(read_lines(attempts_path)) == 4, read_lines(attempts_path)
            revoked = client.events.pop(0)
            assert revoked["event"] == "lease_revoked", revoked
            assert revoked["reason"] == "driver_failure", revoked
            expect(client.request({"version": 1, "id": "old-lease", "op": "move",
                                   "axis": "focus", "direction": "near"}),
                   ok=False, error="driver unavailable")
            assert daemon.poll() is None, daemon.stderr.read()
        finally:
            if client is not None:
                client.close()
            daemon.terminate()
            try:
                daemon.wait(timeout=2)
            except subprocess.TimeoutExpired:
                daemon.kill()
                daemon.wait(timeout=2)

        assert daemon.returncode == 0, daemon.stderr.read()
        assert "driver recovery failed after 3 attempts" in daemon.stderr.read()


if __name__ == "__main__":
    run_test()
    test_pelcod_driver()
    test_failed_driver_recovery()
    print("PASS: persistent driver IPC, leases, preemption, crash recovery, and raw access")
