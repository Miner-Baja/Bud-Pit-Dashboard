#!/usr/bin/env python3
"""
Baja SAE LoRa Telemetry Receiver
=================================
Reads RYLR998 "+RCV=" frames from a USB-connected LoRa module (the pit
receiver) and decodes the binary LoRaPacket struct sent by the car
computer firmware.

Install dependency:
    pip install pyserial

Usage:
    python lora_receiver.py COM5
    python lora_receiver.py /dev/ttyUSB0 --baud 115200 --log telemetry.csv

--- PACKET LAYOUT (must match the firmware's #pragma pack(push, 1) struct) ---
    r1[10]   uint16   ring buffer of RPM1 samples (10 @ 10Hz -> last 1s)
    r2[10]   uint16   ring buffer of RPM2 samples
    v[10]    uint16   ring buffer of speed samples, tenths of mph
    t1       uint8    temp1, offset by +40 (0C = 40)
    t2       uint8    temp2, offset by +40 (always 40 -- no 2nd sensor wired)
    bat      uint16   battery voltage x100
    pct      uint8    battery percent, 0-100
    lat1     int32    "previous fix" latitude  x1e7
    lon1     int32    "previous fix" longitude x1e7
    lat2     int32    "current fix"  latitude  x1e7
    lon2     int32    "current fix"  longitude x1e7
    awd      uint8    4WD relay state (0/1)
    crc16    uint16   Modbus CRC16

NOTE ON THE CRC: the firmware computes crc16_modbus() over only the first
78 bytes of the 84-byte struct (a firmware quirk), which does not cover
the full lat2/lon2/awd fields. This script reproduces that exact
calculation so crc_ok reflects what the transmitter actually intended,
but be aware the check does not protect the tail of the packet.

Also note: the firmware issues "AT+SEND=2,80,<84 raw bytes>" -- the
length argument (80) doesn't match the actual byte count written (84).
That mismatch lives in the firmware/radio layer, not here; this script
decodes whatever byte count the RYLR998's own "+RCV=...,<Length>,..."
header reports and only proceeds if that Length equals PACKET_SIZE (84).
"""

import argparse
import csv
import datetime
import struct

import serial

# 10H + 10H + 10H + B + B + H + B + i + i + i + i + B + H
PACKET_FORMAT = "<10H10H10HBBHBiiiiBH"
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)  # 84 bytes
CRC_COVERED_BYTES = 78  # matches the firmware's (buggy) crc16_modbus(&pkt, 78)


def crc16_modbus(data: bytes) -> int:
    """Reproduces the car computer's crc16_modbus() exactly."""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def read_until(ser: serial.Serial, terminator: bytes) -> bytes:
    """Reads raw bytes until `terminator` is seen (inclusive)."""
    buf = bytearray()
    while True:
        b = ser.read(1)
        if not b:
            raise TimeoutError(f"timed out waiting for {terminator!r}")
        buf += b
        if buf.endswith(terminator):
            return bytes(buf)


def read_rcv_frame(ser: serial.Serial):
    """
    Scans for a "+RCV=" header, then reads Address, Length, exactly
    `Length` raw payload bytes, and the trailing ",RSSI,SNR".

    The payload is read by byte COUNT (not readline()) because the raw
    LoRa payload is binary and may itself contain '\\n', ',', etc.

    Returns (address, payload_bytes, rssi, snr).
    """
    marker = b"+RCV="
    window = bytearray()
    while True:
        b = ser.read(1)
        if not b:
            raise TimeoutError("timed out waiting for +RCV= header")
        window += b
        if len(window) > len(marker):
            del window[0]
        if bytes(window) == marker:
            break

    address = int(read_until(ser, b",")[:-1])
    length = int(read_until(ser, b",")[:-1])
    payload = ser.read(length)
    if len(payload) != length:
        raise TimeoutError(f"expected {length} payload bytes, got {len(payload)}")

    tail = read_until(ser, b"\n").decode(errors="replace").strip()
    parts = tail.split(",")  # parts[0] is '' (leading comma right after payload)
    rssi = int(parts[1]) if len(parts) > 1 and parts[1].lstrip("-").isdigit() else None
    snr = int(parts[2]) if len(parts) > 2 and parts[2].lstrip("-").isdigit() else None

    return address, payload, rssi, snr


def parse_packet(payload: bytes) -> dict:
    if len(payload) != PACKET_SIZE:
        raise ValueError(f"expected {PACKET_SIZE}-byte packet, got {len(payload)}")

    unpacked = struct.unpack(PACKET_FORMAT, payload)
    r1 = unpacked[0:10]
    r2 = unpacked[10:20]
    v = unpacked[20:30]
    (
        t1_raw,
        t2_raw,
        bat_raw,
        pct,
        lat1_raw,
        lon1_raw,
        lat2_raw,
        lon2_raw,
        awd,
        crc_rx,
    ) = unpacked[30:]

    crc_calc = crc16_modbus(payload[:CRC_COVERED_BYTES])

    return {
        "rpm1_samples": list(r1),
        "rpm2_samples": list(r2),
        "speed_mph_samples": [x / 10.0 for x in v],
        "temp1_c": t1_raw - 40,
        "temp2_c": t2_raw - 40,  # placeholder sensor; firmware always sends 40 (0C)
        "battery_voltage": bat_raw / 100.0,
        "battery_pct": pct,
        "lat1": lat1_raw / 1e7,
        "lon1": lon1_raw / 1e7,
        "lat2": lat2_raw / 1e7,
        "lon2": lon2_raw / 1e7,
        "awd_active": bool(awd),
        "crc_received": crc_rx,
        "crc_calculated": crc_calc,
        "crc_ok": crc_rx == crc_calc,
    }


def main():
    ap = argparse.ArgumentParser(
        description="Decode Baja SAE LoRa telemetry from a USB RYLR998 receiver"
    )
    ap.add_argument("port", help="Serial port, e.g. COM5 or /dev/ttyUSB0")
    ap.add_argument(
        "--baud", type=int, default=115200, help="Baud rate (default 115200)"
    )
    ap.add_argument(
        "--timeout",
        type=float,
        default=5.0,
        help="Per-byte serial read timeout, seconds",
    )
    ap.add_argument("--log", help="Optional CSV file to append decoded rows to")
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=args.timeout)
    print(f"[LoRa] Listening on {args.port} @ {args.baud} baud (Ctrl+C to stop)")

    csv_file = csv_writer = None
    if args.log:
        csv_file = open(args.log, "a", newline="")
        csv_writer = csv.writer(csv_file)
        if csv_file.tell() == 0:
            csv_writer.writerow(
                [
                    "timestamp",
                    "address",
                    "rssi",
                    "snr",
                    "crc_ok",
                    "rpm1_last",
                    "rpm2_last",
                    "speed_last_mph",
                    "temp1_c",
                    "battery_voltage",
                    "battery_pct",
                    "lat1",
                    "lon1",
                    "lat2",
                    "lon2",
                    "awd_active",
                ]
            )

    try:
        while True:
            try:
                address, payload, rssi, snr = read_rcv_frame(ser)
            except TimeoutError as e:
                print(f"[LoRa] {e} -- still listening...")
                continue

            if len(payload) != PACKET_SIZE:
                print(
                    f"[LoRa] Skipping frame: {len(payload)} bytes, expected {PACKET_SIZE}"
                )
                continue

            try:
                data = parse_packet(payload)
            except ValueError as e:
                print(f"[LoRa] Parse error: {e}")
                continue

            now = datetime.datetime.now().isoformat(timespec="seconds")
            crc_flag = "OK" if data["crc_ok"] else "BAD"
            print(
                f"[{now}] addr={address} RSSI={rssi} SNR={snr} CRC={crc_flag} | "
                f"RPM1={data['rpm1_samples'][-1]} RPM2={data['rpm2_samples'][-1]} "
                f"SPD={data['speed_mph_samples'][-1]:.1f}mph "
                f"T1={data['temp1_c']}C V={data['battery_voltage']:.2f}V "
                f"PCT={data['battery_pct']}% "
                f"AWD={'ON' if data['awd_active'] else 'off'} "
                f"pos1=({data['lat1']:.6f},{data['lon1']:.6f}) "
                f"pos2=({data['lat2']:.6f},{data['lon2']:.6f})"
            )

            if csv_writer:
                csv_writer.writerow(
                    [
                        now,
                        address,
                        rssi,
                        snr,
                        data["crc_ok"],
                        data["rpm1_samples"][-1],
                        data["rpm2_samples"][-1],
                        data["speed_mph_samples"][-1],
                        data["temp1_c"],
                        data["battery_voltage"],
                        data["battery_pct"],
                        data["lat1"],
                        data["lon1"],
                        data["lat2"],
                        data["lon2"],
                        data["awd_active"],
                    ]
                )
                csv_file.flush()

    except KeyboardInterrupt:
        print("\n[LoRa] Stopped by user")
    finally:
        ser.close()
        if csv_file:
            csv_file.close()


if __name__ == "__main__":
    main()
