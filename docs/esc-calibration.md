# ESC endpoint calibration (isolated motor-node build)

This mode is **not** flight firmware. It sends equal PWM pulse widths to all
four ESC signal pins. Never run it with propellers installed. Use a stable
common ground between the motor node and the ESC signal grounds.

The installed ESC model and its calibration method must be confirmed. A 30 A
rating alone does not establish SimonK firmware or 1000–2000 us support.

## Motor-node connections

- M1 PA6; M2 PA7; M3 PB0; M4 PB1.
- USB-UART TX -> PA10 (USART1 RX); USB-UART RX -> PA9 (USART1 TX);
  USB-UART GND -> board GND.
- Serial console: 115200 baud, 8N1. Send plain characters, no special terminator.

## Build (from repository root)

```sh
git switch esc-calibration
cd firmware/motor-node
make clean
make esc-calibration
```

Build output: `build/motor-node.elf`, `build/motor-node.bin`,
`build/motor-node.hex`.

Example flashing with ST-Link (only if `st-flash` is installed):

```sh
st-flash write build/motor-node.bin 0x08000000
```

## Procedure (propellers removed)

1. **ESC main battery disconnected.** Flash and boot the motor node.
2. Console prints LOW (1000 us) at boot. Type `H` to send HIGH (2000 us).
3. Only then connect ESC battery power if the **exact** ESC firmware
   documentation calls for high-throttle power-on calibration.
4. When the ESC confirms high endpoint by its documented tones, type `L`.
5. Listen for documented endpoint-confirmation tones. Disconnect ESC power,
   reboot controller in LOW, and verify ESC startup.
6. `X` disconnects PWM hardware outputs and latches the console off until reset.

The high pulse automatically returns to LOW after 30 seconds; this is a
fallback only, not a replacement for the manual `L` command. If the ESC
does not confirm the endpoint as expected, disconnect ESC battery power.

Calibration mode bypasses the normal FC-to-motor link by design; **do not**
use this firmware for flight. Restore the normal motor-node build afterward.
