import csv
import serial
import sys
import time

# ---------------------------------------------------------
# Configuration
# ---------------------------------------------------------

SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 115200

OUTPUT_FILE = "accel_raw_20min.csv"

TOTAL_BLOCKS = 240


# ---------------------------------------------------------
# Open UART
# ---------------------------------------------------------

try:
    ser = serial.Serial(
        port=SERIAL_PORT,
        baudrate=BAUD_RATE,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=1.0,
    )

except serial.SerialException as exc:
    print(f"Could not open {SERIAL_PORT}: {exc}")
    sys.exit(1)


print(f"UART opened: {SERIAL_PORT}")
print(f"Baud rate: {BAUD_RATE}")
print(f"Saving to: {OUTPUT_FILE}")
print()
print("Reset the STM32 now.")
print("Press Ctrl+C to stop manually.")
print()


# ---------------------------------------------------------
# Open output CSV
# ---------------------------------------------------------

sample_count = 0
last_block = 0

with open(
    OUTPUT_FILE,
    "w",
    newline="",
    buffering=1,
) as csv_file:

    writer = csv.writer(csv_file)

    # Our clean PC-side CSV header.
    writer.writerow([
        "block",
        "index",
        "ax_raw",
        "ay_raw",
        "az_raw",
    ])

    try:

        while True:

            raw_line = ser.readline()

            if not raw_line:
                continue

            try:
                line = raw_line.decode(
                    "ascii",
                    errors="ignore"
                ).strip()

            except UnicodeDecodeError:
                continue

            if not line:
                continue

            # -------------------------------------------------
            # Show STM32 messages in terminal.
            # -------------------------------------------------

            print(line)

            # -------------------------------------------------
            # Detect end of a capture block.
            # -------------------------------------------------

            if line.startswith(
                "# ACCEL_CAPTURE_END block="
            ):
                try:
                    block_number = int(
                        line.split("=")[1]
                    )

                    last_block = block_number

                    # Make sure completed block is actually
                    # written to disk.
                    csv_file.flush()

                    print(
                        f"\nSaved block "
                        f"{block_number}/{TOTAL_BLOCKS}"
                    )

                    print(
                        f"Total samples saved: "
                        f"{sample_count}\n"
                    )

                    if block_number >= TOTAL_BLOCKS:
                        print(
                            "All 240 blocks received."
                        )
                        break

                except ValueError:
                    pass

                continue

            # -------------------------------------------------
            # Ignore non-data UART messages.
            # -------------------------------------------------

            if line.startswith("#"):
                continue

            if line.startswith("["):
                continue

            if line == (
                "block,index,"
                "ax_raw,ay_raw,az_raw"
            ):
                continue

            # -------------------------------------------------
            # Process acceleration CSV row.
            #
            # Expected:
            #
            # block,index,ax_raw,ay_raw,az_raw
            # -------------------------------------------------

            parts = line.split(",")

            if len(parts) != 5:
                continue

            try:
                block = int(parts[0])
                index = int(parts[1])
                ax = int(parts[2])
                ay = int(parts[3])
                az = int(parts[4])

            except ValueError:
                continue

            # -------------------------------------------------
            # Basic validity check
            # int16 accelerometer values.
            # -------------------------------------------------

            if not (
                -32768 <= ax <= 32767
                and
                -32768 <= ay <= 32767
                and
                -32768 <= az <= 32767
            ):
                print(
                    "Rejected invalid acceleration row:",
                    line
                )
                continue

            # -------------------------------------------------
            # Write immediately into CSV.
            # -------------------------------------------------

            writer.writerow([
                block,
                index,
                ax,
                ay,
                az,
            ])

            sample_count += 1

            # Periodically force Python's buffer to disk.
            if sample_count % 100 == 0:
                csv_file.flush()

    except KeyboardInterrupt:

        print("\nCapture stopped by user.")

    finally:

        csv_file.flush()
        ser.close()


print()
print("UART closed.")
print(f"CSV file: {OUTPUT_FILE}")
print(f"Samples stored: {sample_count}")
print(f"Last complete block: {last_block}")