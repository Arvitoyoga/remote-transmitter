#!/usr/bin/env python3
import argparse
import re
import sys
import threading
import time
import tkinter as tk
from serial.tools import list_ports
import serial


def list_ports_():
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found")
        return
    for port in ports:
        print(f"{port.device}\t{port.description}")


def clamp(v, lo=-1.0, hi=1.0):
    return max(lo, min(hi, v))


def parse_line(line):
    text = line.strip()
    if not text:
        return None

    print(f"RAW: {text}")

    # New format: SIM,throttle,roll,pitch,yaw,left,right
    if text.startswith("SIM"):
        try:
            parts = text.split(",")
            if len(parts) < 7:
                return None
            throttle, roll, pitch, yaw, left_index, right_index = map(int, parts[1:7])
            return {
                "type": "sim",
                "left_x": -((yaw / 1984.0) * 2.0 - 1.0),
                "left_y": -((throttle / 1984.0) * 2.0 - 1.0),
                "right_x": -((roll / 1984.0) * 2.0 - 1.0),
                "right_y": -((pitch / 1984.0) * 2.0 - 1.0),
            }
        except Exception:
            return None

    # Current firmware format: L[----o----] R[----o----] | lx=... ly=... | rx=... ry=...
    match = re.search(
        r"L\[(?P<left>[^\]]*)\]\s+R\[(?P<right>[^\]]*)\]\s+\|\s+lx=(?P<lx>[-+0-9.]+)\s+ly=(?P<ly>[-+0-9.]+)\s+\|\s+rx=(?P<rx>[-+0-9.]+)\s+ry=(?P<ry>[-+0-9.]+)",
        text,
    )
    if match:
        return {
            "type": "imu",
            "left_x": -clamp(float(match.group("ly")) / 45.0),
            "left_y": -clamp(float(match.group("lx")) / 45.0),
            "right_x": -clamp(float(match.group("rx")) / 45.0),
            "right_y": -clamp(float(match.group("ry")) / 45.0),
        }

    return None


class JoystickWindow:
    def __init__(self, port, baudrate, timeout):
        self.root = tk.Tk()
        self.root.title("IMU Joystick View")
        self.root.geometry("760x420")
        self.root.configure(bg="#111111")
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial = None
        self.running = True

        self.canvas = tk.Canvas(self.root, width=760, height=420, bg="#111111", highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.status = tk.StringVar(value="Waiting for data...")
        self.label = tk.Label(self.root, textvariable=self.status, fg="white", bg="#111111", font=("Consolas", 10))
        self.label.pack(fill=tk.X, padx=8, pady=4)

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self._draw_placeholder()
        self._connect_serial()

    def _connect_serial(self):
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            self.status.set(f"Listening on {self.serial.port} @ {self.baudrate}")
            self.thread = threading.Thread(target=self._read_loop, daemon=True)
            self.thread.start()
        except Exception as exc:
            self.status.set(f"Serial error: {exc}")
            self.root.after(1000, self._connect_serial)

    def _read_loop(self):
        buffer = ""
        while self.running:
            if self.serial is None:
                break
            try:
                if self.serial.in_waiting:
                    chunk = self.serial.read(self.serial.in_waiting)
                    if not chunk:
                        continue
                    buffer += chunk.decode("utf-8", errors="replace")
                else:
                    time.sleep(0.01)
                    continue

                while "\n" in buffer or "\r" in buffer:
                    if "\n" in buffer:
                        line, buffer = buffer.split("\n", 1)
                    else:
                        line, buffer = buffer.split("\r", 1)
                    text = line.strip()
                    if not text:
                        continue
                    parsed = parse_line(text)
                    if parsed is None:
                        continue
                    self.root.after(0, self._update_view, parsed)
            except Exception as exc:
                self.root.after(0, self._set_status_error, str(exc))

    def _set_status_error(self, message):
        self.status.set(f"Read error: {message}")

    def _update_view(self, data):
        self.status.set(
            f"L(x={data['left_x']:+.2f}, y={data['left_y']:+.2f})   "
            f"R(x={data['right_x']:+.2f}, y={data['right_y']:+.2f})"
        )
        self._draw_joysticks(data)

    def _draw_placeholder(self):
        self._draw_joysticks({"left_x": 0.0, "left_y": 0.0, "right_x": 0.0, "right_y": 0.0})

    def _draw_joysticks(self, data):
        self.canvas.delete("all")
        self.canvas.create_text(380, 20, text="IMU Joysticks", fill="#f0f0f0", font=("Consolas", 14, "bold"))

        self._draw_single_joystick(180, 220, data["left_x"], data["left_y"], "Left")
        self._draw_single_joystick(580, 220, data["right_x"], data["right_y"], "Right")

    def _draw_single_joystick(self, cx, cy, x, y, label):
        radius = 110
        self.canvas.create_oval(cx - radius, cy - radius, cx + radius, cy + radius, outline="#7fdcff", width=2)
        self.canvas.create_line(cx - radius, cy, cx + radius, cy, fill="#3a3a3a", width=1)
        self.canvas.create_line(cx, cy - radius, cx, cy + radius, fill="#3a3a3a", width=1)
        px = cx + int(x * (radius - 20))
        py = cy - int(y * (radius - 20))
        self.canvas.create_oval(px - 12, py - 12, px + 12, py + 12, fill="#ffdd57", outline="#ffb703", width=2)
        self.canvas.create_text(cx, cy + radius + 20, text=label, fill="#f0f0f0", font=("Consolas", 11))

    def run(self):
        self.root.mainloop()

    def on_close(self):
        self.running = False
        try:
            if self.serial is not None:
                self.serial.close()
        except Exception:
            pass
        self.root.destroy()


def main():
    parser = argparse.ArgumentParser(description="Render serial IMU joystick data in a Tkinter window.")
    parser.add_argument("--list", action="store_true")
    parser.add_argument("-p", "--port", required=False)
    parser.add_argument("-b", "--baudrate", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=0.1)
    args = parser.parse_args()

    if args.list:
        list_ports_()
        return

    if not args.port:
        print("Need --port")
        sys.exit(1)

    app = JoystickWindow(args.port, args.baudrate, args.timeout)
    app.run()


if __name__ == "__main__":
    main()
