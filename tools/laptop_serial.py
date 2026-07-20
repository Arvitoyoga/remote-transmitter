#!/usr/bin/env python3
import argparse
import json
import socket
import sys
import time

import serial
from serial.tools import list_ports


def list_serial_ports():
    ports = list(list_ports.comports())
    if not ports:
        print("Tidak ada serial port terdeteksi.")
        return

    for port in ports:
        print(f"{port.device}\t{port.description}")


def parse_hex_bytes(text):
    text = text.replace(",", " ").replace("0x", "")
    return bytes(int(part, 16) for part in text.split())


def open_serial(port, baudrate, timeout):
    return serial.Serial(
        port=port,
        baudrate=baudrate,
        timeout=timeout,
        write_timeout=timeout,
    )


def read_loop(ser):
    print(f"Membaca dari {ser.port} @ {ser.baudrate}. Tekan Ctrl+C untuk stop.")
    while True:
        data = ser.readline()
        if data:
            try:
                print(data.decode("utf-8", errors="replace"), end="")
            except UnicodeDecodeError:
                print(data.hex(" "))


def clamp(value, lo=-1.0, hi=1.0):
    return max(lo, min(hi, value))


def normalize_channel(value):
    value = max(0, min(1984, value))
    return clamp(((value / 1984.0) * 2.0) - 1.0)


def normalize_throttle(value):
    value = max(0, min(1984, value))
    return value / 1984.0


def parse_sim_line(line):
    parts = line.strip().split(",")
    if len(parts) < 7 or parts[0] != "SIM":
        return None

    throttle, roll, pitch, yaw = map(int, parts[1:5])
    button_values = [int(v) for v in parts[5:]]

    if len(button_values) == 2:
        left_buttons = [button_values[0]]
        right_buttons = [button_values[1]]
    else:
        left_buttons = button_values[:6]
        right_buttons = button_values[6:12] if len(button_values) >= 12 else []

    return {
        "throttle": throttle,
        "roll": roll,
        "pitch": pitch,
        "yaw": yaw,
        "left_point": bool(left_buttons[0]) if len(left_buttons) > 0 else False,
        "left_middle": bool(left_buttons[1]) if len(left_buttons) > 1 else False,
        "left_ring": bool(left_buttons[2]) if len(left_buttons) > 2 else False,
        "left_little": bool(left_buttons[3]) if len(left_buttons) > 3 else False,
        "right_point": bool(right_buttons[0]) if len(right_buttons) > 0 else False,
        "right_middle": bool(right_buttons[1]) if len(right_buttons) > 1 else False,
        "right_ring": bool(right_buttons[2]) if len(right_buttons) > 2 else False,
        "right_little": bool(right_buttons[3]) if len(right_buttons) > 3 else False,
        "throttle_norm": normalize_throttle(throttle),
        "roll_norm": normalize_channel(roll),
        "pitch_norm": normalize_channel(pitch),
        "yaw_norm": normalize_channel(yaw),
    }


def map_sim_to_joystick_data(sim_data):
    return {
        "left_x": -normalize_channel(sim_data["yaw"]),
        "left_y": -normalize_channel(sim_data["throttle"]),
        "right_x": -normalize_channel(sim_data["roll"]),
        "right_y": -normalize_channel(sim_data["pitch"]),
        "left_point": sim_data["left_point"],
        "left_middle": sim_data["left_middle"],
        "left_ring": sim_data["left_ring"],
        "left_little": sim_data["left_little"],
        "right_point": sim_data["right_point"],
        "right_middle": sim_data["right_middle"],
        "right_ring": sim_data["right_ring"],
        "right_little": sim_data["right_little"],
    }


class JoystickEmulator:
    def __init__(self):
        self.device = None
        self._backend = None
        self._warned = False
        self._disabled_reason = None

        platform = sys.platform.lower()
        if platform.startswith("linux"):
            try:
                import uinput
            except Exception as exc:  # pragma: no cover - depends on platform package
                raise RuntimeError(f"uinput unavailable: {exc}") from exc

            self._uinput = uinput
            self.device = self._uinput.Device([
                self._uinput.ABS_X,
                self._uinput.ABS_Y,
                self._uinput.ABS_RX,
                self._uinput.ABS_RY,
                self._uinput.BTN_TRIGGER,
                self._uinput.BTN_THUMB,
                self._uinput.BTN_TOP,
                self._uinput.BTN_BASE3,
                self._uinput.BTN_BASE4,
                self._uinput.BTN_BASE5,
                self._uinput.BTN_BASE6,
            ])
            self._backend = "linux-uinput"
            return

        if platform.startswith("win"):
            try:
                import vgamepad
            except Exception as exc:  # pragma: no cover - depends on installed package
                raise RuntimeError(
                    "vgamepad unavailable. Install it with: pip install vgamepad"
                ) from exc

            controller_cls = None
            for candidate in ("VXBox360Controller", "VX360Controller"):
                cls = getattr(vgamepad, candidate, None)
                if cls is not None:
                    controller_cls = cls
                    break

            if controller_cls is None:
                raise RuntimeError("vgamepad API is not available on this installation")

            self.device = controller_cls(1)
            self._backend = "windows-vgamepad"
            return

        raise RuntimeError(f"Unsupported platform for joystick emulation: {platform}")

    def update(self, joystick_data):
        if self.device is None:
            self._warn_once("Joystick emulator is disabled")
            return

        try:
            if self._backend == "linux-uinput":
                self.device.emit(self._uinput.ABS_X, self._scale_axis(joystick_data["left_x"]))
                self.device.emit(self._uinput.ABS_Y, self._scale_axis(joystick_data["left_y"]))
                self.device.emit(self._uinput.ABS_RX, self._scale_axis(joystick_data["right_x"]))
                self.device.emit(self._uinput.ABS_RY, self._scale_axis(joystick_data["right_y"]))
                self.device.emit(self._uinput.BTN_TRIGGER, 1 if joystick_data["left_point"] else 0)
                self.device.emit(self._uinput.BTN_THUMB, 1 if joystick_data["left_middle"] else 0)
                self.device.emit(self._uinput.BTN_TOP, 1 if joystick_data["left_ring"] else 0)
                self.device.emit(self._uinput.BTN_BASE3, 1 if joystick_data["left_little"] else 0)
                self.device.emit(self._uinput.BTN_BASE4, 1 if joystick_data["right_point"] else 0)
                self.device.emit(self._uinput.BTN_BASE5, 1 if joystick_data["right_middle"] else 0)
                self.device.emit(self._uinput.BTN_BASE6, 1 if joystick_data["right_ring"] else 0)
                self.device.syn()
            elif self._backend == "windows-vgamepad":
                if hasattr(self.device, "left_joystick"):
                    self.device.left_joystick(
                        self._scale_axis(joystick_data["left_x"]),
                        self._scale_axis(joystick_data["left_y"]),
                    )
                elif hasattr(self.device, "set_left_stick"):
                    self.device.set_left_stick(
                        self._scale_axis(joystick_data["left_x"]),
                        self._scale_axis(joystick_data["left_y"]),
                    )
                if hasattr(self.device, "right_joystick"):
                    self.device.right_joystick(
                        self._scale_axis(joystick_data["right_x"]),
                        self._scale_axis(joystick_data["right_y"]),
                    )
                elif hasattr(self.device, "set_right_stick"):
                    self.device.set_right_stick(
                        self._scale_axis(joystick_data["right_x"]),
                        self._scale_axis(joystick_data["right_y"]),
                    )
                if hasattr(self.device, "update"):
                    self.device.update()
        except Exception as exc:
            self._warn_once(f"Joystick update failed: {exc}")

    def _warn_once(self, message):
        if not self._warned:
            print(message)
            self._warned = True

    @staticmethod
    def _scale_axis(value):
        return int(round((clamp(value) + 1.0) * 127.5))


def parse_udp_target(target):
    host, port = target.rsplit(":", 1)
    return host, int(port)


def sim_loop(ser, udp_target=None, joystick_emulator=None):
    udp_socket = None
    udp_addr = None
    if udp_target:
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_addr = parse_udp_target(udp_target)

    if joystick_emulator is not None:
        print("Emulasi joystick virtual aktif.")

    print(f"Membaca input simulator dari {ser.port} @ {ser.baudrate}. Ctrl+C untuk stop.")

    while True:
        raw = ser.readline()
        if not raw:
            continue

        line = raw.decode("utf-8", errors="replace").strip()
        data = parse_sim_line(line)
        if data is None:
            continue

        joystick_data = map_sim_to_joystick_data(data)
        print(
            "Lx={left_x:+.3f} Ly={left_y:+.3f} Rx={right_x:+.3f} Ry={right_y:+.3f} "
            "LP={left_point} LM={left_middle} LR={left_ring} LL={left_little} "
            "RP={right_point} RM={right_middle} RR={right_ring} RL={right_little}".format(**joystick_data)
        )

        if joystick_emulator is not None:
            joystick_emulator.update(joystick_data)

        if udp_socket:
            payload = json.dumps(joystick_data, separators=(",", ":")).encode("utf-8")
            udp_socket.sendto(payload, udp_addr)


def interactive_loop(ser):
    print(f"Terhubung ke {ser.port} @ {ser.baudrate}.")
    print("Ketik teks lalu Enter untuk kirim. Ctrl+C untuk stop.")

    while True:
        if ser.in_waiting:
            data = ser.read(ser.in_waiting)
            print(data.decode("utf-8", errors="replace"), end="")

        line = sys.stdin.readline()
        if line:
            ser.write(line.encode("utf-8"))
            ser.flush()
        else:
            time.sleep(0.01)


def main():
    parser = argparse.ArgumentParser(description="Komunikasi serial laptop dengan ESP32.")
    parser.add_argument("--list", action="store_true", help="Tampilkan daftar serial port.")
    parser.add_argument("-p", "--port", help="Serial port, contoh: /dev/ttyUSB0 atau COM5.")
    parser.add_argument("-b", "--baudrate", type=int, default=115200, help="Baudrate serial.")
    parser.add_argument("--timeout", type=float, default=0.1, help="Timeout serial dalam detik.")
    parser.add_argument("--read", action="store_true", help="Mode baca saja.")
    parser.add_argument("--sim", action="store_true", help="Parse format SIM dari firmware remote.")
    parser.add_argument("--udp", help="Kirim data SIM sebagai JSON UDP, contoh: 127.0.0.1:9000.")
    parser.add_argument("--joystick", action="store_true", help="Emulasikan joystick virtual dari data SIM saat tersedia.")
    parser.add_argument("--send", help="Kirim teks sekali lalu keluar.")
    parser.add_argument("--send-hex", help='Kirim byte hex sekali, contoh: "c8 18 16 00".')
    args = parser.parse_args()

    if args.list:
        list_serial_ports()
        return

    if not args.port:
        parser.error("Isi --port, atau pakai --list untuk melihat port yang tersedia.")

    with open_serial(args.port, args.baudrate, args.timeout) as ser:
        if args.send is not None:
            ser.write(args.send.encode("utf-8"))
            ser.flush()
            return

        if args.send_hex is not None:
            ser.write(parse_hex_bytes(args.send_hex))
            ser.flush()
            return

        if args.read:
            read_loop(ser)
            return

        if args.sim:
            joystick_emulator = None
            if args.joystick:
                try:
                    joystick_emulator = JoystickEmulator()
                except Exception as exc:
                    print(f"Joystick emulation unavailable: {exc}")
            sim_loop(ser, args.udp, joystick_emulator)
            return

        interactive_loop(ser)


if __name__ == "__main__":
    main()
