#include "motor_outputs.h"

#define RCC_BASE_ADDRESS       0x40021000UL
#define GPIOA_BASE_ADDRESS     0x40010800UL
#define GPIOB_BASE_ADDRESS     0x40010C00UL

#define RCC_APB2ENR_OFFSET     0x18UL
#define GPIO_CRL_OFFSET        0x00UL
#define GPIO_ODR_OFFSET        0x0CUL
#define GPIO_BSRR_OFFSET       0x10UL

/*
 * Convert a peripheral register address into a volatile 32-bit
 * memory-mapped register.
 */
#define REG32(address)         (*(volatile uint32_t *)(address))

#define RCC_APB2ENR            \
    REG32(RCC_BASE_ADDRESS + RCC_APB2ENR_OFFSET)

#define GPIOA_CRL              \
    REG32(GPIOA_BASE_ADDRESS + GPIO_CRL_OFFSET)

#define GPIOA_ODR              \
    REG32(GPIOA_BASE_ADDRESS + GPIO_ODR_OFFSET)

#define GPIOA_BSRR             \
    REG32(GPIOA_BASE_ADDRESS + GPIO_BSRR_OFFSET)

#define GPIOB_CRL              \
    REG32(GPIOB_BASE_ADDRESS + GPIO_CRL_OFFSET)

#define GPIOB_ODR              \
    REG32(GPIOB_BASE_ADDRESS + GPIO_ODR_OFFSET)

#define GPIOB_BSRR             \
    REG32(GPIOB_BASE_ADDRESS + GPIO_BSRR_OFFSET)

/*
 * RCC_APB2ENR:
 *
 * Bit 2: IOPAEN — GPIOA clock enable
 * Bit 3: IOPBEN — GPIOB clock enable
 */
#define RCC_APB2ENR_IOPAEN     (1UL << 2)
#define RCC_APB2ENR_IOPBEN     (1UL << 3)

/*
 * Motor output pin assignments:
 *
 * Motor 1 → PA6
 * Motor 2 → PA7
 * Motor 3 → PB0
 * Motor 4 → PB1
 */
#define MOTOR_PA6_MASK         (1UL << 6)
#define MOTOR_PA7_MASK         (1UL << 7)
#define MOTOR_PB0_MASK         (1UL << 0)
#define MOTOR_PB1_MASK         (1UL << 1)

#define MOTOR_GPIOA_MASK       \
    (MOTOR_PA6_MASK | MOTOR_PA7_MASK)

#define MOTOR_GPIOB_MASK       \
    (MOTOR_PB0_MASK | MOTOR_PB1_MASK)

/*
 * Each pin in GPIOx_CRL has a four-bit configuration field:
 *
 * CNF[1:0]:MODE[1:0]
 *
 * 0b0010 means:
 *
 * CNF  = 00 → General-purpose push-pull output
 * MODE = 10 → Maximum output speed 2 MHz
 */
#define GPIO_OUTPUT_2MHZ_PUSH_PULL  0x2UL

/*
 * Produce the four-bit GPIO_CRL mask for a specified pin.
 */
#define GPIO_CRL_FIELD_MASK(pin)    \
    (0xFUL << ((pin) * 4UL))

/*
 * Produce the required 0b0010 configuration at the correct
 * position for a specified pin.
 */
#define GPIO_CRL_FIELD_VALUE(pin)   \
    (GPIO_OUTPUT_2MHZ_PUSH_PULL << ((pin) * 4UL))

/*
 * PA6 uses GPIOA_CRL bits 27:24.
 * PA7 uses GPIOA_CRL bits 31:28.
 */
#define GPIOA_MOTOR_CRL_MASK        \
    (GPIO_CRL_FIELD_MASK(6UL) |     \
     GPIO_CRL_FIELD_MASK(7UL))

#define GPIOA_MOTOR_CRL_VALUE       \
    (GPIO_CRL_FIELD_VALUE(6UL) |    \
     GPIO_CRL_FIELD_VALUE(7UL))

/*
 * PB0 uses GPIOB_CRL bits 3:0.
 * PB1 uses GPIOB_CRL bits 7:4.
 */
#define GPIOB_MOTOR_CRL_MASK        \
    (GPIO_CRL_FIELD_MASK(0UL) |     \
     GPIO_CRL_FIELD_MASK(1UL))

#define GPIOB_MOTOR_CRL_VALUE       \
    (GPIO_CRL_FIELD_VALUE(0UL) |    \
     GPIO_CRL_FIELD_VALUE(1UL))



volatile uint32_t g_motor_safe_state_requested;


/*
 * Records the result of the actual GPIO safety verification.
 *
 * 0: GPIO state is not confirmed safe
 * 1: GPIO state is confirmed safe
 */
volatile uint32_t g_motor_outputs_safe;


/*
 * Enable the GPIOA and GPIOB peripheral clocks.
 */
static void motor_gpio_clocks_enable(void)
{
    /*
     * Preserve all existing RCC_APB2ENR bits and set only the
     * GPIOA and GPIOB clock-enable bits.
     */
    RCC_APB2ENR |= RCC_APB2ENR_IOPAEN |
                   RCC_APB2ENR_IOPBEN;

    /*
     * Read the register back.
     *
     * This ensures the APB2 clock-enable write has completed
     * before accessing the GPIO registers.
     */
    (void)RCC_APB2ENR;
}


/*
 * Atomically clear the output latches for all four motor pins.
 */
static void motor_gpio_output_latches_clear(void)
{
    /*
     * GPIOx_BSRR bits 31:16 reset the corresponding GPIO
     * output-data-register bits.
     *
     * For example:
     *
     * Writing bit 22 clears GPIOA ODR bit 6.
     * Writing bit 23 clears GPIOA ODR bit 7.
     *
     * The output latches are cleared before changing the pin
     * modes. This prevents a short HIGH glitch when the pins
     * become outputs.
     */
    GPIOA_BSRR = MOTOR_GPIOA_MASK << 16;
    GPIOB_BSRR = MOTOR_GPIOB_MASK << 16;
}


/*
 * Configure PA6, PA7, PB0 and PB1 as ordinary push-pull outputs.
 *
 * TIM3 is not enabled during Phase 2.1.6.
 */
static void motor_gpio_configure_outputs(void)
{
    uint32_t gpioa_crl;
    uint32_t gpiob_crl;

    /*
     * Read the complete GPIO configuration registers so the
     * configurations of unrelated pins are preserved.
     */
    gpioa_crl = GPIOA_CRL;
    gpiob_crl = GPIOB_CRL;

    /*
     * Clear only the PA6 and PA7 configuration fields.
     */
    gpioa_crl &= (uint32_t)(~GPIOA_MOTOR_CRL_MASK);

    /*
     * Configure PA6 and PA7 as 2 MHz push-pull outputs.
     */
    gpioa_crl |= GPIOA_MOTOR_CRL_VALUE;
    GPIOA_CRL = gpioa_crl;

    /*
     * Clear only the PB0 and PB1 configuration fields.
     */
    gpiob_crl &= (uint32_t)(~GPIOB_MOTOR_CRL_MASK);

    /*
     * Configure PB0 and PB1 as 2 MHz push-pull outputs.
     */
    gpiob_crl |= GPIOB_MOTOR_CRL_VALUE;
    GPIOB_CRL = gpiob_crl;
}

/* ================================================================
 * PHASE 2.1.6 — NEW CODE END
 * ================================================================ */


/* ================================================================
 * PHASE 2.1.6 — UPDATED FUNCTION START
 *
 * This function existed in Phase 2.1.5, but its implementation
 * is replaced with the real GPIO safety operation.
 * ================================================================ */

void motor_outputs_force_safe(void)
{
    /*
     * Record the fact that the application or a fault handler
     * requested the motor-safe state.
     */
    g_motor_safe_state_requested = 1UL;

    /*
     * The hardware has not yet been verified, so begin with the
     * verification result cleared.
     */
    g_motor_outputs_safe = 0UL;

    /*
     * GPIO registers cannot be used until the corresponding
     * APB2 peripheral clocks are enabled.
     */
    motor_gpio_clocks_enable();

    /*
     * Clear the output latches before changing the pins from
     * their reset-state input configuration into output mode.
     */
    motor_gpio_output_latches_clear();

    /*
     * Configure PA6, PA7, PB0 and PB1 as 2 MHz general-purpose
     * push-pull GPIO outputs.
     */
    motor_gpio_configure_outputs();

    /*
     * Clear the output latches again after configuring the pins.
     * This guarantees that all four outputs remain LOW.
     */
    motor_gpio_output_latches_clear();

    /*
     * Data Synchronization Barrier.
     *
     * It ensures that all explicit memory accesses above have
     * completed before the safety result is checked.
     */
    __asm volatile ("dsb" ::: "memory");

    /*
     * Verify both:
     *
     * 1. GPIO configuration fields
     * 2. GPIO output-latch levels
     */
    g_motor_outputs_safe =
        motor_outputs_are_safe() ? 1UL : 0UL;
}
bool motor_outputs_are_safe(void)
{
    /*
     * Read only the GPIO configuration fields belonging to the
     * four motor-output pins.
     */
    const uint32_t gpioa_config =
        GPIOA_CRL & GPIOA_MOTOR_CRL_MASK;

    const uint32_t gpiob_config =
        GPIOB_CRL & GPIOB_MOTOR_CRL_MASK;

    /*
     * Read only the output-latch bits belonging to the four
     * motor-output pins.
     */
    const uint32_t gpioa_outputs =
        GPIOA_ODR & MOTOR_GPIOA_MASK;

    const uint32_t gpiob_outputs =
        GPIOB_ODR & MOTOR_GPIOB_MASK;

    /*
     * The safe state is accepted only when:
     *
     * 1. PA6 and PA7 have the required GPIO configuration.
     * 2. PB0 and PB1 have the required GPIO configuration.
     * 3. The PA6 and PA7 output latches are LOW.
     * 4. The PB0 and PB1 output latches are LOW.
     */
    return (gpioa_config == GPIOA_MOTOR_CRL_VALUE) &&
           (gpiob_config == GPIOB_MOTOR_CRL_VALUE) &&
           (gpioa_outputs == 0UL) &&
           (gpiob_outputs == 0UL);
}





bool motor_outputs_safe_state_was_requested(void)
{
    return (g_motor_safe_state_requested != 0UL);
}