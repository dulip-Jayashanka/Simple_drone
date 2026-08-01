#include "status_led.h"

#include <stdint.h>

/*
 * PHASE 2.1.8: NEW FILE
 *
 * Bare-metal PC13 status LED driver for STM32F103C8T6.
 */


/* ================================================================
 * Peripheral base addresses
 * ================================================================ */

#define RCC_BASE_ADDRESS       0x40021000UL
#define GPIOC_BASE_ADDRESS     0x40011000UL


/* ================================================================
 * Register offsets
 * ================================================================ */

/*
 * RCC_APB2ENR is located at:
 *
 * RCC base + 0x18
 */
#define RCC_APB2ENR_OFFSET     0x18UL

/*
 * GPIOx_CRH configures pins 8 through 15.
 */
#define GPIO_CRH_OFFSET        0x04UL

/*
 * GPIOx_ODR contains the current output-latch values.
 */
#define GPIO_ODR_OFFSET        0x0CUL

/*
 * GPIOx_BSRR atomically sets or resets GPIO output bits.
 */
#define GPIO_BSRR_OFFSET       0x10UL


/* ================================================================
 * Memory-mapped register helper
 * ================================================================ */

/*
 * Convert an address into a volatile 32-bit peripheral register.
 *
 * volatile prevents the compiler from removing or combining
 * hardware-register accesses.
 */
#define REG32(address)         (*(volatile uint32_t *)(address))


/* ================================================================
 * Required peripheral registers
 * ================================================================ */

#define RCC_APB2ENR            \
    REG32(RCC_BASE_ADDRESS + RCC_APB2ENR_OFFSET)

#define GPIOC_CRH              \
    REG32(GPIOC_BASE_ADDRESS + GPIO_CRH_OFFSET)

#define GPIOC_ODR              \
    REG32(GPIOC_BASE_ADDRESS + GPIO_ODR_OFFSET)

#define GPIOC_BSRR             \
    REG32(GPIOC_BASE_ADDRESS + GPIO_BSRR_OFFSET)


/* ================================================================
 * RCC configuration
 * ================================================================ */

/*
 * RCC_APB2ENR bit 4:
 *
 * IOPCEN = I/O Port C clock enable
 */
#define RCC_APB2ENR_IOPCEN     (1UL << 4)


/* ================================================================
 * PC13 definitions
 * ================================================================ */

#define STATUS_LED_PIN         13UL
#define STATUS_LED_PIN_MASK    (1UL << STATUS_LED_PIN)

/*
 * GPIOx_BSRR:
 *
 * Bits 15:0  set output bits.
 * Bits 31:16 reset output bits.
 *
 * Therefore:
 *
 * Bit 13 sets PC13 HIGH.
 * Bit 29 resets PC13 LOW.
 */
#define STATUS_LED_SET_MASK    STATUS_LED_PIN_MASK
#define STATUS_LED_RESET_MASK  \
    (STATUS_LED_PIN_MASK << 16UL)


/* ================================================================
 * GPIO configuration-field definitions
 * ================================================================ */

/*
 * PC13 is controlled by GPIOC_CRH because CRH configures
 * pins 8 through 15.
 *
 * Pin position inside CRH:
 *
 *     PC13 -> pin index 13 - 8 = 5
 *
 * Each pin uses four bits:
 *
 *     5 x 4 = bit position 20
 *
 * Therefore, PC13 uses GPIOC_CRH bits 23:20.
 */
#define STATUS_LED_CRH_SHIFT   \
    ((STATUS_LED_PIN - 8UL) * 4UL)

#define STATUS_LED_CRH_MASK    \
    (0xFUL << STATUS_LED_CRH_SHIFT)

/*
 * STM32F1 GPIO output configuration:
 *
 * CNF[1:0]  = 00 -> General-purpose push-pull
 * MODE[1:0] = 10 -> Maximum output speed 2 MHz
 *
 * Four-bit field:
 *
 *     CNF1 CNF0 MODE1 MODE0
 *       0    0     1     0
 *
 *     Binary 0010 = hexadecimal 0x2
 */
#define GPIO_OUTPUT_2MHZ_PUSH_PULL  0x2UL

#define STATUS_LED_CRH_VALUE        \
    (GPIO_OUTPUT_2MHZ_PUSH_PULL << STATUS_LED_CRH_SHIFT)


/* ================================================================
 * Public status-LED functions
 * ================================================================ */

void status_led_init(void)
{
    uint32_t gpio_crh;

    /*
     * Enable the GPIOC peripheral clock.
     *
     * The read-modify-write operation preserves all other
     * APB2 peripheral clock-enable bits.
     */
    RCC_APB2ENR |= RCC_APB2ENR_IOPCEN;

    /*
     * Read RCC_APB2ENR back.
     *
     * This ensures that the clock-enable write has reached the
     * peripheral before GPIOC registers are accessed.
     */
    (void)RCC_APB2ENR;

    /*
     * Prepare the PC13 output latch before changing PC13 into
     * output mode.
     *
     * PC13 HIGH means that the active-low LED is OFF.
     *
     * Preparing the latch first prevents an unwanted LED flash
     * when the pin changes from input mode to output mode.
     */
    GPIOC_BSRR = STATUS_LED_SET_MASK;

    /*
     * Read the complete CRH register so the configurations of
     * PC8-PC12 and PC14-PC15 are preserved.
     */
    gpio_crh = GPIOC_CRH;

    /*
     * Clear only the four-bit PC13 configuration field.
     */
    gpio_crh &= (uint32_t)(~STATUS_LED_CRH_MASK);

    /*
     * Configure PC13 as:
     *
     *     General-purpose push-pull output
     *     Maximum output speed = 2 MHz
     */
    gpio_crh |= STATUS_LED_CRH_VALUE;

    /*
     * Write the completed configuration back to GPIOC_CRH.
     */
    GPIOC_CRH = gpio_crh;
}


void status_led_on(void)
{
    /*
     * The Blue Pill LED is active-low.
     *
     * Writing to BSRR bit 29 resets ODR bit 13,
     * which drives PC13 LOW and turns the LED ON.
     */
    GPIOC_BSRR = STATUS_LED_RESET_MASK;
}


void status_led_off(void)
{
    /*
     * Writing to BSRR bit 13 sets ODR bit 13,
     * which drives PC13 HIGH and turns the LED OFF.
     */
    GPIOC_BSRR = STATUS_LED_SET_MASK;
}


void status_led_toggle(void)
{
    /*
     * Check the PC13 output latch.
     *
     * ODR13 = 0 -> PC13 LOW  -> LED currently ON
     * ODR13 = 1 -> PC13 HIGH -> LED currently OFF
     */
    if ((GPIOC_ODR & STATUS_LED_PIN_MASK) == 0UL)
    {
        /*
         * LED is currently ON, so turn it OFF.
         */
        status_led_off();
    }
    else
    {
        /*
         * LED is currently OFF, so turn it ON.
         */
        status_led_on();
    }
}


bool status_led_is_on(void)
{
    /*
     * Because the LED is active-low, the LED is logically ON
     * when the PC13 output latch contains zero.
     */
    return (GPIOC_ODR & STATUS_LED_PIN_MASK) == 0UL;
}