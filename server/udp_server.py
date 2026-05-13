import socket
import struct
import asyncio
from dataclasses import dataclass

UDP_IP = "0.0.0.0"
UDP_PORT = 16161
BUFFER_SIZE = 128
PAYLOAD_SIZE = 18
PAYLOAD_FORMAT = "<7hI"
SAMPLE_PRINT_INTERVAL = 60

ACCEL_SCALE = 0
GYRO_SCALE = 0

@dataclass
class MPU6050RawData:
    accel_x: int
    accel_y: int
    accel_z: int
    temp: int
    gyro_x: int
    gyro_y: int
    gyro_z: int
    timestamp: int

@dataclass
class MPU6050FloatData:
    accel_x: float
    accel_y: float
    accel_z: float
    temp: float
    gyro_x: float
    gyro_y: float
    gyro_z: float
    timestamp: int

def parse_payload_to_int16(data: bytes) -> MPU6050RawData:
    if len(data) < PAYLOAD_SIZE:
        raise ValueError(f"payload too short: {len(data)} bytes, expected {PAYLOAD_SIZE}")

    values = struct.unpack(PAYLOAD_FORMAT, data[:PAYLOAD_SIZE])
    return MPU6050RawData(
        accel_x=values[0],
        accel_y=values[1],
        accel_z=values[2],
        temp=values[3],
        gyro_x=values[4],
        gyro_y=values[5],
        gyro_z=values[6],
        timestamp=values[7],
    )


def raw_to_float(raw: MPU6050RawData) -> MPU6050FloatData:
    accel_div = 16384.0 / (1 << ACCEL_SCALE)
    gyro_div = 131.072 / (1 << GYRO_SCALE)

    return MPU6050FloatData(
        accel_x=raw.accel_x / accel_div,
        accel_y=raw.accel_y / accel_div,
        accel_z=raw.accel_z / accel_div,
        temp=(raw.temp / 340.0) + 35.0,
        gyro_x=raw.gyro_x / gyro_div,
        gyro_y=raw.gyro_y / gyro_div,
        gyro_z=raw.gyro_z / gyro_div,
        timestamp=raw.timestamp,
    )


def print_float_data(sample_count: int, readable: MPU6050FloatData) -> None:
    print(
        f"Sample {sample_count}: "
        f"accel=({readable.accel_x:.4f}, {readable.accel_y:.4f}, {readable.accel_z:.4f}) g, "
        f"temp={readable.temp:.2f} C, "
        f"gyro=({readable.gyro_x:.4f}, {readable.gyro_y:.4f}, {readable.gyro_z:.4f}) deg/s, "
        f"timestamp={readable.timestamp}"
    )


def main() -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    sample_count = 0

    print(f"UDP server listening on {UDP_IP}:{UDP_PORT}")

    while True:
        data, addr = sock.recvfrom(BUFFER_SIZE)
        try:
            raw = parse_payload_to_int16(data)
        except ValueError as exc:
            print(f"Received invalid packet from {addr}: {exc}")
            continue

        readable = raw_to_float(raw)
        sample_count += 1

        if sample_count % SAMPLE_PRINT_INTERVAL == 0:
            print_float_data(sample_count, readable)


if __name__ == "__main__":
    main()
