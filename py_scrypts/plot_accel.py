import argparse

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


# ---------------------------------------------------------
# Configuration
# ---------------------------------------------------------

SAMPLE_RATE_HZ = 500.0


# ---------------------------------------------------------
# Command-line argument
# ---------------------------------------------------------

parser = argparse.ArgumentParser(
    description="Plot raw MPU6500 accelerometer CSV data"
)

parser.add_argument(
    "csv_file",
    help="CSV file produced from the STM32 UART capture",
)

args = parser.parse_args()


# ---------------------------------------------------------
# Load CSV
# ---------------------------------------------------------

df = pd.read_csv(args.csv_file)

required_columns = {
    "block",
    "index",
    "ax_raw",
    "ay_raw",
    "az_raw",
}

missing = required_columns - set(df.columns)

if missing:
    raise ValueError(
        f"CSV is missing columns: {sorted(missing)}"
    )


# ---------------------------------------------------------
# Sort data
# ---------------------------------------------------------

df = df.sort_values(
    by=["block", "index"]
).reset_index(drop=True)


# ---------------------------------------------------------
# Generate a continuous sample number
# ---------------------------------------------------------

df["sample"] = np.arange(len(df))


# ---------------------------------------------------------
# Generate CAPTURED time
#
# This treats consecutive recorded samples as 500 Hz.
#
# IMPORTANT:
# UART transmission gaps between 5-second blocks are
# intentionally NOT included.
# ---------------------------------------------------------

df["time_s"] = df["sample"] / SAMPLE_RATE_HZ


# ---------------------------------------------------------
# Basic information
# ---------------------------------------------------------

print()
print("========== ACCELEROMETER DATA ==========")

print(f"CSV file:           {args.csv_file}")
print(f"Samples:            {len(df)}")
print(f"Blocks:             {df['block'].nunique()}")
print(f"Nominal sample rate:{SAMPLE_RATE_HZ} Hz")

captured_seconds = len(df) / SAMPLE_RATE_HZ

print(
    f"Captured duration:  "
    f"{captured_seconds:.2f} s "
    f"({captured_seconds / 60.0:.2f} min)"
)

print()


# ---------------------------------------------------------
# X-axis plot
# ---------------------------------------------------------

plt.figure(figsize=(14, 6))

plt.plot(
    df["time_s"],
    df["ax_raw"],
    linewidth=0.6,
)

plt.xlabel("Captured time (s)")
plt.ylabel("Raw accelerometer X (counts)")
plt.title("MPU6500 Raw Accelerometer X")
plt.grid(True)

plt.tight_layout()

plt.savefig(
    "accel_x_raw.png",
    dpi=150,
)

print("Saved accel_x_raw.png")


# ---------------------------------------------------------
# Y-axis plot
# ---------------------------------------------------------

plt.figure(figsize=(14, 6))

plt.plot(
    df["time_s"],
    df["ay_raw"],
    linewidth=0.6,
)

plt.xlabel("Captured time (s)")
plt.ylabel("Raw accelerometer Y (counts)")
plt.title("MPU6500 Raw Accelerometer Y")
plt.grid(True)

plt.tight_layout()

plt.savefig(
    "accel_y_raw.png",
    dpi=150,
)

print("Saved accel_y_raw.png")


# ---------------------------------------------------------
# Z-axis plot
# ---------------------------------------------------------

plt.figure(figsize=(14, 6))

plt.plot(
    df["time_s"],
    df["az_raw"],
    linewidth=0.6,
)

plt.xlabel("Captured time (s)")
plt.ylabel("Raw accelerometer Z (counts)")
plt.title("MPU6500 Raw Accelerometer Z")
plt.grid(True)

plt.tight_layout()

plt.savefig(
    "accel_z_raw.png",
    dpi=150,
)

print("Saved accel_z_raw.png")


# ---------------------------------------------------------
# All axes together
# ---------------------------------------------------------

plt.figure(figsize=(14, 6))

plt.plot(
    df["time_s"],
    df["ax_raw"],
    label="Ax",
    linewidth=0.6,
)

plt.plot(
    df["time_s"],
    df["ay_raw"],
    label="Ay",
    linewidth=0.6,
)

plt.plot(
    df["time_s"],
    df["az_raw"],
    label="Az",
    linewidth=0.6,
)

plt.xlabel("Captured time (s)")
plt.ylabel("Raw accelerometer value (counts)")
plt.title("MPU6500 Raw Accelerometer — X, Y, Z")

plt.legend()
plt.grid(True)

plt.tight_layout()

plt.savefig(
    "accel_xyz_raw.png",
    dpi=150,
)

print("Saved accel_xyz_raw.png")


# ---------------------------------------------------------
# Show plots
# ---------------------------------------------------------

plt.show()