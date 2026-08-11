#ifndef STATUS_LED_H
#define STATUS_LED_H

#include <stdbool.h>

/*
 * PHASE 2.1.8: NEW FILE
 *
 * Bare-metal driver for the Blue Pill onboard status LED.
 *
 * The LED is normally connected to PC13 and is active-low:
 *
 *     PC13 LOW  -> LED ON
 *     PC13 HIGH -> LED OFF
 *
 * The application does not need to remember this electrical
 * inversion. The driver translates logical LED operations into
 * the correct PC13 output level.
 */

/*
 * Enable GPIOC and configure PC13 as a 2 MHz general-purpose
 * push-pull output.
 *
 * The LED is left OFF after initialization.
 */
void status_led_init(void);

/*
 * Turn the physical LED ON.
 *
 * Because the LED is active-low, this drives PC13 LOW.
 */
void status_led_on(void);

/*
 * Turn the physical LED OFF.
 *
 * Because the LED is active-low, this drives PC13 HIGH.
 */
void status_led_off(void);

/*
 * Reverse the current LED state.
 */
void status_led_toggle(void);

/*
 * Return true if the LED is currently ON.
 *
 * This checks the PC13 output latch, not the physical light level.
 */
bool status_led_is_on(void);

#endif /* STATUS_LED_H */