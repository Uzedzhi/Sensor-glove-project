#!/usr/bin/env python3
"""PTY + real driver + real Unix socket; never opens uinput or moves the desktop mouse."""
import errno
import json
import math
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import threading
import time


class Sensor:
    def __init__(self, link):
        self.master, slave = os.openpty()
        target = os.ttyname(slave)
        os.close(slave)
        self.link = Path(link)
        self.link.unlink(missing_ok=True)
        self.link.symlink_to(target)
        self.rotation = (1, 0, 0, 0)
        self.paused = False
        self.clock = 0
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        while not self.stop.is_set():
            with self.lock:
                q, clock, paused = self.rotation, self.clock, self.paused
                if not paused:
                    self.clock += 10
            if not paused:
                line = (f"I ({clock}) bno055: Quaternion W={q[0]:.6f} X={q[1]:.6f} "
                        f"Y={q[2]:.6f} Z={q[3]:.6f} | Cal SYS=0 GYR=3 ACC=2 MAG=1\r\n")
                try:
                    os.write(self.master, line.encode())
                except OSError as error:
                    if error.errno not in (errno.EIO, errno.EBADF):
                        raise
            self.stop.wait(.01)

    def angle(self, axis, degrees):
        q = [math.cos(math.radians(degrees)/2), 0, 0, 0]
        q[axis+1] = math.sin(math.radians(degrees)/2)
        with self.lock:
            self.rotation = q

    def close(self):
        self.stop.set()
        self.thread.join(timeout=1)
        if self.master >= 0:
            os.close(self.master)
            self.master = -1


class Controller:
    def __init__(self, path):
        self.socket = socket.socket(socket.AF_UNIX)
        self.socket.settimeout(1)
        self.socket.connect(str(path))
        self.buffer = b""
        self.next_id = 1

    def receive(self):
        while b"\n" not in self.buffer:
            chunk = self.socket.recv(65536)
            if not chunk:
                raise RuntimeError("Driver closed socket")
            self.buffer += chunk
        line, self.buffer = self.buffer.split(b"\n", 1)
        return json.loads(line)

    def request(self, command, *, expect_ok=True, **fields):
        request = {"version": 1, "id": self.next_id, "command": command, **fields}
        self.next_id += 1
        self.socket.sendall(json.dumps(request).encode()+b"\n")
        reply = self.receive()
        assert reply["id"] == request["id"] and reply["version"] == 1, reply
        assert reply["ok"] == expect_ok, reply
        return reply.get("result") if expect_ok else reply["error"]

    def close(self):
        self.socket.close()


def wait_for(fn, timeout=4):
    until = time.monotonic()+timeout
    last = None
    while time.monotonic() < until:
        try:
            last = fn()
            if last:
                return last
        except (ConnectionError, FileNotFoundError):
            pass
        time.sleep(.02)
    raise AssertionError(f"Condition did not become true: {last}")


def run(driver):
    with tempfile.TemporaryDirectory(prefix="glove-integration-") as directory:
        base = Path(directory)
        sock, profile, link = base/"driver.sock", base/"config.json", base/"sensor"
        sensor = Sensor(link)
        log = open(base/"driver.log", "w+")
        process = None
        client = None

        def start():
            return subprocess.Popen([str(driver), "--dry-run", "--port", str(link), "--socket", str(sock),
                                     "--config", str(profile)], stdout=subprocess.DEVNULL, stderr=log)

        def status():
            return client.request("get_status")

        def ready():
            s = status()
            return s if s["ready"] else None

        try:
            process = start()
            wait_for(sock.exists)
            client = Controller(sock)
            initial = status()
            assert not initial["enabled"] and initial["dry_run"]
            wait_for(ready)
            assert not any("/dev/uinput" in os.readlink(p) for p in Path(f"/proc/{process.pid}/fd").iterdir())
            duplicate = subprocess.run([str(driver), "--dry-run", "--socket", str(sock), "--config", str(profile)],
                                       capture_output=True, text=True, timeout=2)
            assert duplicate.returncode != 0 and "already" in duplicate.stderr, duplicate.stderr
            # Partial request, version and JSON errors.
            client.socket.sendall(b'{"version":1,"id":900,"command":"get_')
            time.sleep(.03)
            client.socket.sendall(b'status"}\n')
            assert client.receive()["id"] == 900
            client.socket.sendall(b'{"version":9,"id":901,"command":"get_status"}\n')
            assert not client.receive()["ok"]
            client.socket.sendall(b'{broken}\n')
            assert not client.receive()["ok"]
            client.request("unknown", expect_ok=False)
            config = client.request("get_config")
            config["deadzone_deg_s"] = 0
            client.request("set_config", config=config)
            assert json.loads(profile.read_text())["deadzone_deg_s"] == 0
            client.request("set_enabled", enabled=True)
            time.sleep(.06)  # Establish post-enable baseline.
            sensor.angle(2, 5)
            moved = wait_for(lambda: (s if (s := status())["total_dx"] < -90 else None))
            assert moved["total_dy"] == 0, moved
            time.sleep(.1)
            assert status()["total_dx"] == moved["total_dx"]  # Held tilt does not keep moving.
            before = moved["total_dx"]
            client.request("recenter")
            time.sleep(.05)
            assert status()["total_dx"] == before
            # Live configuration changes, same process, persisted profile.
            config["x"]["invert"] = False
            result = client.request("set_config", config=config)
            assert result["pid"] == process.pid and result["enabled"]
            time.sleep(.05)
            sensor.angle(2, 10)
            wait_for(lambda: status()["total_dx"] > before+90)
            config["x"]["plane"] = "None"
            config["y"]["plane"] = "XY"
            config["y"]["invert"] = False
            client.request("set_config", config=config)
            time.sleep(.05)
            old = status()
            sensor.angle(2, 15)
            wait_for(lambda: status()["total_dy"] > old["total_dy"]+90)
            assert status()["total_dx"] == old["total_dx"]
            # Save failure must not acknowledge or replace active settings/profile.
            original = profile.read_text()
            if os.geteuid() != 0:
                os.chmod(base, 0o500)
                try:
                    bad = json.loads(original)
                    bad["x"]["gain"] = 77
                    client.request("set_config", config=bad, expect_ok=False)
                    assert client.request("get_config")["x"]["gain"] == config["x"]["gain"]
                    assert profile.read_text() == original
                finally:
                    os.chmod(base, 0o700)
            # A slow/flooding client cannot stall a normal client or 100 Hz telemetry.
            slow = socket.socket(socket.AF_UNIX)
            slow.settimeout(.5)
            slow.connect(str(sock))
            old_count = status()["samples"]
            try:
                slow.sendall(b'{"version":1,"id":1,"command":"get_status"}\n'*3000)
            except OSError:
                pass
            time.sleep(.1)
            assert status()["samples"] > old_count
            slow.close()
            huge = socket.socket(socket.AF_UNIX)
            huge.settimeout(.5)
            huge.connect(str(sock))
            try:
                huge.sendall(b"x"*70000)
                try:
                    assert huge.recv(1) == b""
                except ConnectionResetError:
                    pass
            finally:
                huge.close()
            assert status()["pid"] == process.pid
            # Invalid sensor sample does not replace the valid sample.
            old_rejected = status()["rejected"]
            os.write(sensor.master, b"Quaternion W=NaN X=0 Y=0 Z=0 | Cal SYS=0 GYR=0 ACC=0 MAG=0\n")
            wait_for(lambda: status()["rejected"] > old_rejected)
            with sensor.lock:
                sensor.paused = True
            time.sleep(.35)
            assert not status()["enabled"] and status()["telemetry_state"] == "stale"
            with sensor.lock:
                sensor.paused = False
            wait_for(ready)
            assert not status()["enabled"]
            client.request("set_enabled", enabled=True)
            with sensor.lock:
                sensor.clock = 0
            wait_for(lambda: not status()["enabled"])
            wait_for(ready)
            # Physical-disconnect substitute: replace the PTY behind the same configured symlink.
            sensor.close()
            wait_for(lambda: not status()["serial_connected"])
            sensor = Sensor(link)
            wait_for(ready)
            assert not status()["enabled"]
            client.close()
            client = Controller(sock)
            assert client.request("get_config") == config
            client.request("shutdown")
            process.wait(timeout=2)
            assert process.returncode == 0 and not sock.exists()
            client.close()
            client = None
            process = start()
            wait_for(sock.exists)
            client = Controller(sock)
            assert client.request("get_config") == config
            assert not status()["enabled"]
            process.terminate()
            process.wait(timeout=2)
            assert not sock.exists()
            print("PASS: real driver, PTY, protocol, persistence, 100 Hz stream, pause/restart/reconnect, bounded clients; no uinput")
        except Exception:
            log.flush()
            log.seek(0)
            print(log.read(), file=sys.stderr)
            raise
        finally:
            if client:
                client.close()
            if process and process.poll() is None:
                process.terminate()
                process.wait(timeout=2)
            sensor.close()
            log.close()


if __name__ == "__main__":
    run(Path(sys.argv[1]).resolve())
