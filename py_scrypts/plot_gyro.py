#!/usr/bin/env python3
"""Plot and summarize stationary MPU6500 raw-gyroscope capture data."""

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd


GYRO_COUNTS_PER_DPS = 65.5
AXES = ("x", "y", "z")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plot CSV data produced by gyro_uart_capture.py"
    )
    parser.add_argument("csv_file")
    parser.add_argument(
        "--output-dir",
        help="Plot directory (default: same directory as the CSV)",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Open plot windows after saving the PNG files",
    )
    return parser.parse_args()


def save_axis_plot(df: pd.DataFrame, axis: str, output_dir: Path) -> None:
    column = f"g{axis}_raw"
    plt.figure(figsize=(14, 6))
    plt.plot(df["captured_time_s"] / 60.0, df[column], linewidth=0.45)
    plt.xlabel("Captured gyro data time (min)")
    plt.ylabel(f"Raw gyro {axis.upper()} (counts)")
    plt.title(f"MPU6500 Stationary Raw Gyroscope {axis.upper()}")
    plt.grid(True, alpha=0.35)
    plt.tight_layout()
    path = output_dir / f"gyro_{axis}_raw.png"
    plt.savefig(path, dpi=150)
    print(f"Saved {path}")


def main() -> int:
    args = parse_args()
    csv_path = Path(args.csv_file).expanduser()
    output_dir = (
        Path(args.output_dir).expanduser()
        if args.output_dir
        else csv_path.parent
    )
    output_dir.mkdir(parents=True, exist_ok=True)

    df = pd.read_csv(csv_path)
    required = {
        "block",
        "index",
        "captured_time_s",
        "sensor_elapsed_s",
        "gx_raw",
        "gy_raw",
        "gz_raw",
    }
    missing = required - set(df.columns)
    if missing:
        raise ValueError(f"CSV is missing columns: {sorted(missing)}")
    if df.empty:
        raise ValueError("CSV contains no gyro samples")

    df = df.sort_values(["block", "index"]).reset_index(drop=True)
    for axis in AXES:
        df[f"g{axis}_dps"] = df[f"g{axis}_raw"] / GYRO_COUNTS_PER_DPS

    print("\n========== STATIONARY GYROSCOPE DATA ==========")
    print(f"CSV file:             {csv_path}")
    print(f"Samples:              {len(df)}")
    print(f"Complete/partial blocks: {df['block'].nunique()}")
    print(
        f"Captured data time:   "
        f"{(df['captured_time_s'].iloc[-1] / 60.0):.2f} min"
    )
    print(
        f"Sensor elapsed time:  "
        f"{(df['sensor_elapsed_s'].iloc[-1] / 60.0):.2f} min"
    )

    print("\nAxis statistics (raw counts and deg/s):")
    for axis in AXES:
        raw = df[f"g{axis}_raw"]
        dps = df[f"g{axis}_dps"]
        print(
            f"{axis.upper()}: mean={raw.mean():9.3f} counts "
            f"({dps.mean(): .6f} deg/s), "
            f"std={raw.std():8.3f} counts "
            f"({dps.std():.6f} deg/s), "
            f"min={raw.min()}, max={raw.max()}"
        )

    for axis in AXES:
        save_axis_plot(df, axis, output_dir)

    plt.figure(figsize=(14, 6))
    for axis in AXES:
        plt.plot(
            df["captured_time_s"] / 60.0,
            df[f"g{axis}_dps"],
            label=f"G{axis.upper()}",
            linewidth=0.45,
        )
    plt.xlabel("Captured gyro data time (min)")
    plt.ylabel("Angular rate (deg/s)")
    plt.title("MPU6500 Stationary Raw Gyroscope — X, Y, Z")
    plt.legend()
    plt.grid(True, alpha=0.35)
    plt.tight_layout()
    combined_path = output_dir / "gyro_xyz_raw_dps.png"
    plt.savefig(combined_path, dpi=150)
    print(f"Saved {combined_path}")

    block_means = df.groupby("block", as_index=False).agg(
        sensor_elapsed_s=("sensor_elapsed_s", "first"),
        gx_dps=("gx_dps", "mean"),
        gy_dps=("gy_dps", "mean"),
        gz_dps=("gz_dps", "mean"),
    )

    plt.figure(figsize=(14, 6))
    for axis in AXES:
        plt.plot(
            block_means["sensor_elapsed_s"] / 60.0,
            block_means[f"g{axis}_dps"],
            marker=".",
            markersize=3,
            linewidth=0.8,
            label=f"G{axis.upper()} 5 s mean",
        )
    plt.xlabel("Actual sensor elapsed time including UART gaps (min)")
    plt.ylabel("Five-second mean angular rate (deg/s)")
    plt.title("MPU6500 Stationary Gyroscope Bias Drift")
    plt.legend()
    plt.grid(True, alpha=0.35)
    plt.tight_layout()
    drift_path = output_dir / "gyro_bias_drift_dps.png"
    plt.savefig(drift_path, dpi=150)
    print(f"Saved {drift_path}")

    if args.show:
        plt.show()
    else:
        plt.close("all")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
