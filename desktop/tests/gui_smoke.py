#!/usr/bin/env python3
"""Exercise the specified GUI scenarios on a dedicated Xvfb display, with a dry-run driver."""
import ctypes as C
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time

from integration import Controller, Sensor, wait_for


class XInput:
    def __init__(self):
        self.x = C.CDLL("libX11.so.6")
        self.t = C.CDLL("libXtst.so.6")
        self.x.XOpenDisplay.argtypes = [C.c_char_p]
        self.x.XOpenDisplay.restype = C.c_void_p
        self.display = self.x.XOpenDisplay(None)
        assert self.display, "No X display"
        self.x.XDefaultRootWindow.argtypes = [C.c_void_p]
        self.x.XDefaultRootWindow.restype = C.c_ulong
        self.root = self.x.XDefaultRootWindow(self.display)
        self.x.XQueryTree.argtypes = [C.c_void_p, C.c_ulong, C.POINTER(C.c_ulong), C.POINTER(C.c_ulong),
                                      C.POINTER(C.POINTER(C.c_ulong)), C.POINTER(C.c_uint)]
        self.x.XFetchName.argtypes = [C.c_void_p, C.c_ulong, C.POINTER(C.c_void_p)]
        self.x.XFree.argtypes = [C.c_void_p]
        self.x.XFlush.argtypes = [C.c_void_p]
        self.x.XCloseDisplay.argtypes = [C.c_void_p]
        self.x.XTranslateCoordinates.argtypes = [C.c_void_p, C.c_ulong, C.c_ulong, C.c_int, C.c_int,
                                                 C.POINTER(C.c_int), C.POINTER(C.c_int), C.POINTER(C.c_ulong)]
        self.x.XStringToKeysym.argtypes = [C.c_char_p]
        self.x.XStringToKeysym.restype = C.c_ulong
        self.x.XKeysymToKeycode.argtypes = [C.c_void_p, C.c_ulong]
        self.x.XKeysymToKeycode.restype = C.c_ubyte
        self.t.XTestFakeMotionEvent.argtypes = [C.c_void_p, C.c_int, C.c_int, C.c_int, C.c_ulong]
        self.t.XTestFakeButtonEvent.argtypes = [C.c_void_p, C.c_uint, C.c_int, C.c_ulong]
        self.t.XTestFakeKeyEvent.argtypes = [C.c_void_p, C.c_uint, C.c_int, C.c_ulong]
        self.window = 0

    def find_window(self):
        root, parent = C.c_ulong(), C.c_ulong()
        children, count = C.POINTER(C.c_ulong)(), C.c_uint()
        self.x.XQueryTree(self.display, self.root, C.byref(root), C.byref(parent), C.byref(children), C.byref(count))
        windows = [int(children[i]) for i in range(count.value)]
        if children:
            self.x.XFree(children)
        for window in windows:
            name = C.c_void_p()
            if self.x.XFetchName(self.display, window, C.byref(name)) and name.value:
                text = C.string_at(name).decode(errors="replace")
                self.x.XFree(name)
                if text == "Glove mouse - Control":
                    self.window = window
                    return window
        return None

    def click(self, x, y):
        rx, ry, child = C.c_int(), C.c_int(), C.c_ulong()
        self.x.XTranslateCoordinates(self.display, self.window, self.root, 0, 0, C.byref(rx), C.byref(ry), C.byref(child))
        self.t.XTestFakeMotionEvent(self.display, -1, rx.value+x, ry.value+y, 0)
        self.t.XTestFakeButtonEvent(self.display, 1, 1, 0)
        self.x.XFlush(self.display)
        time.sleep(.06)
        self.t.XTestFakeButtonEvent(self.display, 1, 0, 0)
        self.x.XFlush(self.display)
        time.sleep(.1)

    def key(self, name, down):
        symbol = self.x.XStringToKeysym(name.encode())
        code = self.x.XKeysymToKeycode(self.display, symbol)
        assert code, name
        self.t.XTestFakeKeyEvent(self.display, code, down, 0)
        self.x.XFlush(self.display)
        time.sleep(.04)

    def chord(self, name):
        self.key("Control_L", True)
        self.key(name, True)
        self.key(name, False)
        self.key("Control_L", False)
        time.sleep(.06)

    def field(self, x, y, text):
        self.click(x, y)
        self.chord("a")
        for char in text:
            name = {".": "period", "-": "minus"}.get(char, char)
            self.key(name, True)
            self.key(name, False)

    def capture(self, path):
        subprocess.run(["import", "-window", hex(self.window), str(path)], check=True, timeout=5)


def run(driver, gui, artifacts):
    assert os.environ.get("GLOVE_GUI_TEST") == "1", "Run with dedicated xvfb-run and GLOVE_GUI_TEST=1"
    assert not os.environ.get("WAYLAND_DISPLAY"), "Use isolated Xvfb; unset WAYLAND_DISPLAY"
    artifacts.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="glove-gui-") as directory:
        base = Path(directory)
        sock, profile, link = base/"driver.sock", base/"config.json", base/"sensor"
        log = open(base/"session.log", "w+")
        driver_process = gui_process = sensor = controller = None
        x = XInput()

        def start_gui():
            process = subprocess.Popen([str(gui), "--socket", str(sock)], stdout=log, stderr=log)
            wait_for(x.find_window)
            time.sleep(.25)
            assert process.poll() is None
            return process

        def start_driver():
            process = subprocess.Popen([str(driver), "--socket", str(sock), "--config", str(profile),
                                        "--port", str(link), "--dry-run"], stdout=log, stderr=log)
            wait_for(sock.exists)
            return process

        def status():
            return controller.request("get_status")

        try:
            gui_process = start_gui()
            x.capture(artifacts/"gui-offline.png")
            sensor = Sensor(link)
            driver_process = start_driver()
            controller = Controller(sock)
            wait_for(lambda: status()["ready"])
            time.sleep(1.1)  # Allow GUI reconnect and state refresh.
            x.capture(artifacts/"gui-ready.png")
            x.click(140, 705)  # Enable.
            wait_for(lambda: status()["enabled"])
            time.sleep(.05)
            sensor.angle(2, 5)
            wait_for(lambda: status()["total_dx"] < -90)
            time.sleep(.2)
            x.capture(artifacts/"gui-preview.png")
            x.click(140, 705)  # Pause.
            wait_for(lambda: not status()["enabled"])
            before = status()["total_dx"]
            sensor.angle(2, 10)
            time.sleep(.15)
            assert status()["total_dx"] == before
            x.click(243, 257)  # XZ for X.
            x.click(469, 257)  # Invert X off.
            x.field(600, 257, "30")
            x.click(375, 321)  # Y off.
            x.field(100, 597, "0")
            old_pid = status()["pid"]
            x.click(560, 705)  # Apply.
            wait_for(lambda: (c := controller.request("get_config"))["x"] == {"plane": "XZ", "invert": False, "gain": 30}
                     and c["y"]["plane"] == "None" and c["deadzone_deg_s"] == 0)
            assert status()["pid"] == old_pid
            x.click(345, 705)  # Set reference, no jump.
            assert status()["total_dx"] == before
            x.field(600, 257, "0")
            x.click(560, 705)
            time.sleep(.2)
            assert controller.request("get_config")["x"]["gain"] == 30
            x.capture(artifacts/"gui-validation.png")
            x.field(600, 257, "30")
            x.click(560, 705)
            if os.geteuid() != 0:
                old_profile = profile.read_bytes()
                x.field(600, 257, "31")
                base.chmod(0o500)
                try:
                    x.click(560, 705)
                    time.sleep(.3)
                    assert controller.request("get_config")["x"]["gain"] == 30
                    assert profile.read_bytes() == old_profile
                    x.capture(artifacts/"gui-save-error.png")
                finally:
                    base.chmod(0o700)
                x.field(600, 257, "30")
                x.click(560, 705)
            x.click(140, 705)
            wait_for(lambda: status()["enabled"])
            controller.request("shutdown")
            driver_process.wait(timeout=3)
            time.sleep(.7)
            assert gui_process.poll() is None
            driver_process = start_driver()
            controller.close()
            controller = Controller(sock)
            wait_for(lambda: status()["ready"])
            assert not status()["enabled"] and controller.request("get_config")["x"]["gain"] == 30
            time.sleep(1.1)
            x.click(140, 705)
            wait_for(lambda: status()["enabled"])
            x.chord("q")
            gui_process.wait(timeout=3)
            assert gui_process.returncode == 0 and driver_process.poll() is None and status()["enabled"]
            gui_process = start_gui()
            time.sleep(.3)
            x.click(140, 705)  # Existing enabled state must be displayed as Pause.
            wait_for(lambda: not status()["enabled"])
            x.chord("q")
            gui_process.wait(timeout=3)
            controller.request("shutdown")
            driver_process.wait(timeout=3)
            print("PASS: raylib GUI offline/live, plane/invert/gain/noise edit, apply/error, preview, pause/center, IPC restart, independent close/reopen")
        except Exception:
            log.flush()
            log.seek(0)
            print(log.read(), file=sys.stderr)
            if x.find_window():
                x.capture(artifacts/"gui-failure.png")
            raise
        finally:
            for process in (gui_process, driver_process):
                if process and process.poll() is None:
                    process.terminate()
                    process.wait(timeout=3)
            if controller:
                controller.close()
            if sensor:
                sensor.close()
            x.x.XCloseDisplay(x.display)
            log.close()


if __name__ == "__main__":
    run(Path(sys.argv[1]).resolve(), Path(sys.argv[2]).resolve(), Path(sys.argv[3]).resolve())
