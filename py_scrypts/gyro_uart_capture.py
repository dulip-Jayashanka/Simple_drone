#!/usr/bin/env python3
"""Capture repeated five-second STM32 raw-gyroscope blocks to CSV."""

import argparse
import csv
import re
import sys
from pathlib import Path

import serial


BEGIN_RE = re.compile(
    r"^# GYRO_CAPTURE_BEGIN block=(\d+) samples=(\d+) start_us=(\d+)$"
)
END_RE = re.compile(r"^# GYRO_CAPTURE_END block=(\d+)$")
MCU_HEADER = "block,index,gx_raw,gy_raw,gz_raw"
UINT32_MODULUS = 1 << 32


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Save the STM32 GYRO_CAPTURE_TEST UART stream as a CSV file."
        )
    )
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--output", default="gyro_raw_20min.csv")
    parser.add_argument("--blocks", type=int, default=240)
    parser.add_argument("--sample-rate", type=float, default=500.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if args.blocks < 1 or args.sample_rate <= 0.0:
        print("--blocks and --sample-rate must be positive", file=sys.stderr)
        return 2

    output_path = Path(args.output).expanduser()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    try:
        uart = serial.Serial(
            port=args.port,
            baudrate=args.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1.0,
        )
    except serial.SerialException as exc:
        print(f"Could not open {args.port}: {exc}", file=sys.stderr)
        return 1

    print(f"UART: {args.port} at {args.baud} baud")
    print(f"CSV:  {output_path}")
    print("Reset the STM32 now. Press Ctrl+C to stop safely.\n")

    sample_count = 0
    last_complete_block = 0
    current_block = None
    current_block_start_unwrapped = None
    first_block_start_unwrapped = None
    previous_block_start_raw = None
    wrap_count = 0
    expected_block_samples = None

    try:
        with output_path.open("w", newline="", buffering=1) as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(
                [
                    "block",
                    "index",
                    "captured_time_s",
                    "sensor_elapsed_s",
                    "gx_raw",
                    "gy_raw",
                    "gz_raw",
                ]
            )

            while True:
                raw_line = uart.readline()
                if not raw_line:
                    continue

                line = raw_line.decode("ascii", errors="ignore").strip()
                if not line:
                    continue

                begin_match = BEGIN_RE.match(line)
                if begin_match:
                    current_block = int(begin_match.group(1))
                    expected_block_samples = int(begin_match.group(2))
                    block_start_raw = int(begin_match.group(3))

                    if (
                        previous_block_start_raw is not None
                        and block_start_raw < previous_block_start_raw
                        and previous_block_start_raw - block_start_raw
                        > UINT32_MODULUS // 2
                    ):
                        wrap_count += 1

                    previous_block_start_raw = block_start_raw
                    current_block_start_unwrapped = (
                        block_start_raw + wrap_count * UINT32_MODULUS
                    )

                    if first_block_start_unwrapped is None:
                        first_block_start_unwrapped = (
                            current_block_start_unwrapped
                        )

                    print(
                        f"Receiving block {current_block}/{args.blocks} "
                        f"({expected_block_samples} samples)"
                    )
                    continue

                end_match = END_RE.match(line)
                if end_match:
                    ended_block = int(end_match.group(1))
                    csv_file.flush()
                    last_complete_block = ended_block
                    print(
                        f"Saved block {ended_block}/{args.blocks}; "
                        f"total samples={sample_count}"
                    )

                    if ended_block >= args.blocks:
                        print("All requested gyro blocks received.")
                        break
                    continue

                if line == MCU_HEADER or line.startswith("[") or line.startswith("#"):
                    if line.startswith("["):
                        print(line)
                    continue

                parts = line.split(",")
                if len(parts) != 5:
                    continue

                try:
                    block, index, gx, gy, gz = map(int, parts)
                except ValueError:
                    continue

                if current_block is None or block != current_block:
                    print(f"Ignored row outside its block marker: {line}")
                    continue

                if not all(-32768 <= value <= 32767 for value in (gx, gy, gz)):
                    print(f"Rejected invalid int16 gyro row: {line}")
                    continue

                captured_time_s = sample_count / args.sample_rate
                sensor_elapsed_s = (
                    (
                        current_block_start_unwrapped
                        - first_block_start_unwrapped
                    )
                    / 1_000_000.0
                    + index / args.sample_rate
                )

                writer.writerow(
                    [
                        block,
                        index,
                        f"{captured_time_s:.6f}",
                        f"{sensor_elapsed_s:.6f}",
                        gx,
                        gy,
                        gz,
                    ]
                )
                sample_count += 1

                if sample_count % 100 == 0:
                    csv_file.flush()

    except KeyboardInterrupt:
        print("\nCapture stopped by user.")
    finally:
        uart.close()

    print(f"CSV closed safely: {output_path}")
    print(f"Samples stored: {sample_count}")
    print(f"Last complete block: {last_complete_block}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
