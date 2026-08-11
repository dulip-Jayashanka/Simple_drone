#!/usr/bin/env python3
"""Capture STM32 accelerometer pipeline UART output into a CSV file.

Expected firmware line (fields depend on enabled Make flags):

    [ACCEL PIPE] seq=123 t_us=456000 raw=1,2,3 median=1,2,3 \
    calibrated_g=0.001,0.002,1.000 filtered_g=0.001,0.002,0.999 \
    filtered_ms2=0.010,0.020,9.797 flags=0x00000001

Install the only external dependency with:

    python3 -m pip install pyserial
"""

from __future__ import annotations

import argparse
import csv
import re
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

try:
    import serial
except ImportError:
    print(
        "Missing pyserial. Install it with: python3 -m pip install pyserial",
        file=sys.stderr,
    )
    raise SystemExit(1)


LINE_PREFIX = "[ACCEL PIPE]"
HEADER = [
    "host_time_utc",
    "host_elapsed_s",
    "sequence",
    "timestamp_us",
    "raw_x",
    "raw_y",
    "raw_z",
    "median_x",
    "median_y",
    "median_z",
    "calibrated_x_g",
    "calibrated_y_g",
    "calibrated_z_g",
    "filtered_x_g",
    "filtered_y_g",
    "filtered_z_g",
    "filtered_x_ms2",
    "filtered_y_ms2",
    "filtered_z_ms2",
    "flags",
]

SEQ_RE = re.compile(r"\bseq=(\d+)")
TIMESTAMP_RE = re.compile(r"\bt_us=(\d+)")
FLAGS_RE = re.compile(r"\bflags=(\S+)")
TRIPLET_RES = {
    "raw": re.compile(r"\braw=(-?\d+),(-?\d+),(-?\d+)"),
    "median": re.compile(r"\bmedian=(-?\d+),(-?\d+),(-?\d+)"),
    "calibrated_g": re.compile(
        r"\bcalibrated_g=(-?\d+(?:\.\d+)?),"
        r"(-?\d+(?:\.\d+)?),(-?\d+(?:\.\d+)?)"
    ),
    "filtered_g": re.compile(
        r"\bfiltered_g=(-?\d+(?:\.\d+)?),"
        r"(-?\d+(?:\.\d+)?),(-?\d+(?:\.\d+)?)"
    ),
    "filtered_ms2": re.compile(
        r"\bfiltered_ms2=(-?\d+(?:\.\d+)?),"
        r"(-?\d+(?:\.\d+)?),(-?\d+(?:\.\d+)?)"
    ),
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Store [ACCEL PIPE] UART output in CSV format."
    )
    parser.add_argument(
        "--port",
        default="/dev/ttyUSB0",
        help="serial device (default: /dev/ttyUSB0)",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="UART baud rate (default: 115200)",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("accel_pipeline.csv"),
        help="output CSV path (default: accel_pipeline.csv)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="stop after this many seconds; 0 means unlimited",
    )
    parser.add_argument(
        "--samples",
        type=int,
        default=0,
        help="stop after this many valid rows; 0 means unlimited",
    )
    parser.add_argument(
        "--show-all",
        action="store_true",
        help="also display non-pipeline STM32 messages",
    )
    args = parser.parse_args()

    if args.baud <= 0:
        parser.error("--baud must be greater than zero")
    if args.duration < 0:
        parser.error("--duration cannot be negative")
    if args.samples < 0:
        parser.error("--samples cannot be negative")

    return args


def triplet(line: str, field: str) -> list[str]:
    match = TRIPLET_RES[field].search(line)
    return list(match.groups()) if match else ["", "", ""]


def parse_pipeline_line(line: str, start_monotonic: float) -> list[object] | None:
    if not line.startswith(LINE_PREFIX):
        return None

    sequence_match = SEQ_RE.search(line)
    timestamp_match = TIMESTAMP_RE.search(line)
    flags_match = FLAGS_RE.search(line)

    # sequence and timestamp identify a complete, usable firmware row.
    if sequence_match is None or timestamp_match is None:
        return None

    return [
        datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
        f"{time.monotonic() - start_monotonic:.6f}",
        sequence_match.group(1),
        timestamp_match.group(1),
        *triplet(line, "raw"),
        *triplet(line, "median"),
        *triplet(line, "calibrated_g"),
        *triplet(line, "filtered_g"),
        *triplet(line, "filtered_ms2"),
        flags_match.group(1) if flags_match else "",
    ]


def main() -> int:
    args = parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)

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

    row_count = 0
    rejected_count = 0
    return_code = 0
    start_monotonic = time.monotonic()

    print(f"UART: {args.port} at {args.baud} baud")
    print(f"CSV:  {args.output.resolve()}")
    print("Reset the STM32 if required. Press Ctrl+C to stop.\n")

    try:
        with args.output.open("w", newline="", encoding="utf-8", buffering=1) as file:
            writer = csv.writer(file)
            writer.writerow(HEADER)

            while True:
                if args.duration and time.monotonic() - start_monotonic >= args.duration:
                    break

                raw_bytes = uart.readline()
                if not raw_bytes:
                    continue

                line = raw_bytes.decode("ascii", errors="ignore").strip()
                if not line:
                    continue

                row = parse_pipeline_line(line, start_monotonic)
                if row is None:
                    if line.startswith(LINE_PREFIX):
                        rejected_count += 1
                        print(f"Rejected malformed pipeline line: {line}")
                    elif args.show_all:
                        print(line)
                    continue

                writer.writerow(row)
                row_count += 1

                if row_count == 1 or row_count % 100 == 0:
                    file.flush()
                    print(f"Stored {row_count} rows (latest seq={row[2]})")

                if args.samples and row_count >= args.samples:
                    break

    except KeyboardInterrupt:
        print("\nCapture stopped by user.")
    except (OSError, serial.SerialException) as exc:
        print(f"\nCapture error: {exc}", file=sys.stderr)
        return_code = 1
    finally:
        uart.close()

    print(f"CSV saved: {args.output.resolve()}")
    print(f"Rows stored: {row_count}")
    print(f"Malformed pipeline lines rejected: {rejected_count}")
    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
