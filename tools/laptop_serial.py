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


def normalize_channel(value):
    value = max(0, min(1984, value))
    return (value / 992.0) - 1.0


def normalize_throttle(value):
    value = max(0, min(1984, value))
    return value / 1984.0


def parse_sim_line(line):
    parts = line.strip().split(",")
    if len(parts) != 7 or parts[0] != "SIM":
        return None

    throttle, roll, pitch, yaw, left_index, right_index = map(int, parts[1:])
    return {
        "throttle": throttle,
        "roll": roll,
        "pitch": pitch,
        "yaw": yaw,
        "left_index": bool(left_index),
        "right_index": bool(right_index),
        "throttle_norm": normalize_throttle(throttle),
        "roll_norm": normalize_channel(roll),
        "pitch_norm": normalize_channel(pitch),
        "yaw_norm": normalize_channel(yaw),
    }


def parse_udp_target(target):
    host, port = target.rsplit(":", 1)
    return host, int(port)


def sim_loop(ser, udp_target=None):
    udp_socket = None
    udp_addr = None
    if udp_target:
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_addr = parse_udp_target(udp_target)

    print(f"Membaca input simulator dari {ser.port} @ {ser.baudrate}. Ctrl+C untuk stop.")

    while True:
        raw = ser.readline()
        if not raw:
            continue

        line = raw.decode("utf-8", errors="replace").strip()
        data = parse_sim_line(line)
        if data is None:
            continue

        print(
            "thr={throttle_norm:.3f} roll={roll_norm:.3f} "
            "pitch={pitch_norm:.3f} yaw={yaw_norm:.3f} "
            "L={left_index} R={right_index}".format(**data)
        )

        if udp_socket:
            payload = json.dumps(data, separators=(",", ":")).encode("utf-8")
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
            sim_loop(ser, args.udp)
            return

        interactive_loop(ser)


if __name__ == "__main__":
    main()
