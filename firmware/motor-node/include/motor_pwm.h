#ifndef MOTOR_PWM_H
#define MOTOR_PWM_H

#include <stdbool.h>
#include <stdint.h>


/*
 * ============================================================
 * PHASE 6.3 — TIM3 ESC PWM CONFIGURATION
 * ============================================================
 *
 * Locked motor/pin mapping:
 *
 *     M1 -> TIM3_CH1 -> PA6
 *     M2 -> TIM3_CH2 -> PA7
 *     M3 -> TIM3_CH3 -> PB0
 *     M4 -> TIM3_CH4 -> PB1
 *
 * The initial bench configuration uses a 1 MHz timer counter so one
 * timer count equals one microsecond. The initial SimonK bench range
 * is 1000..2000 us at 400 Hz. Those numerical values remain bench
 * configuration only until the installed ESCs are physically verified.
 *
 * Phase 6.3 deliberately defines protocol command 0 as the same
 * electrical pulse used by DISARMED/FAILSAFE. Therefore
 * MOTOR_ESC_SAFE_US and MOTOR_ESC_MIN_US are required to be equal.
 * A future measured "minimum reliably spinning motor" threshold is a
 * different concept and must not be implemented by raising this
 * electrical minimum, because doing that would make the zero-command
 * ARM guard spin the motors.
 */

#ifndef MOTOR_PWM_ENABLE
#define MOTOR_PWM_ENABLE 0
#endif

#if (MOTOR_PWM_ENABLE != 0) && \
    (MOTOR_PWM_ENABLE != 1)
#error "MOTOR_PWM_ENABLE must be 0 or 1"
#endif


#ifndef MOTOR_ESC_PWM_HZ
#define MOTOR_ESC_PWM_HZ 400UL
#endif

#ifndef MOTOR_ESC_SAFE_US
#define MOTOR_ESC_SAFE_US 1000UL
#endif

#ifndef MOTOR_ESC_MIN_US
#define MOTOR_ESC_MIN_US 1000UL
#endif

#ifndef MOTOR_ESC_MAX_US
#define MOTOR_ESC_MAX_US 2000UL
#endif

#define MOTOR_PWM_COUNTER_HZ 1000000UL


#if (MOTOR_ESC_PWM_HZ < 1UL) || \
    (MOTOR_ESC_PWM_HZ > 1000UL)
#error "MOTOR_ESC_PWM_HZ must be from 1 to 1000"
#endif

#if ((MOTOR_PWM_COUNTER_HZ % MOTOR_ESC_PWM_HZ) != 0UL)
#error "MOTOR_ESC_PWM_HZ must divide 1 MHz exactly"
#endif

#define MOTOR_PWM_PERIOD_US \
    (MOTOR_PWM_COUNTER_HZ / MOTOR_ESC_PWM_HZ)

#if (MOTOR_ESC_MIN_US < 1UL) || \
    (MOTOR_ESC_MAX_US <= MOTOR_ESC_MIN_US)
#error "Invalid MOTOR_ESC_MIN_US / MOTOR_ESC_MAX_US range"
#endif

#if (MOTOR_ESC_SAFE_US != MOTOR_ESC_MIN_US)
#error "Phase 6.3 requires MOTOR_ESC_SAFE_US == MOTOR_ESC_MIN_US"
#endif

#if (MOTOR_ESC_MAX_US >= MOTOR_PWM_PERIOD_US)
#error "MOTOR_ESC_MAX_US must be smaller than the PWM period"
#endif

#if (MOTOR_PWM_PERIOD_US > 65536UL)
#error "PWM period does not fit the 16-bit TIM3 auto-reload register"
#endif


/*
 * ============================================================
 * ISOLATED RAW-PWM BENCH TEST
 * ============================================================
 *
 * When enabled, motor-node main initializes TIM3 and writes exactly
 * these four pulse widths. UART, watchdog and the normal motor state
 * machine do not own the outputs in this mode.
 */

#ifndef MOTOR_PWM_TEST_MODE
#define MOTOR_PWM_TEST_MODE 0
#endif

#if (MOTOR_PWM_TEST_MODE != 0) && \
    (MOTOR_PWM_TEST_MODE != 1)
#error "MOTOR_PWM_TEST_MODE must be 0 or 1"
#endif

#ifndef MOTOR_PWM_TEST_M1_US
#define MOTOR_PWM_TEST_M1_US MOTOR_ESC_SAFE_US
#endif

#ifndef MOTOR_PWM_TEST_M2_US
#define MOTOR_PWM_TEST_M2_US MOTOR_ESC_SAFE_US
#endif

#ifndef MOTOR_PWM_TEST_M3_US
#define MOTOR_PWM_TEST_M3_US MOTOR_ESC_SAFE_US
#endif

#ifndef MOTOR_PWM_TEST_M4_US
#define MOTOR_PWM_TEST_M4_US MOTOR_ESC_SAFE_US
#endif

#if MOTOR_PWM_TEST_MODE

#if (MOTOR_PWM_TEST_M1_US < MOTOR_ESC_MIN_US) || \
    (MOTOR_PWM_TEST_M1_US > MOTOR_ESC_MAX_US) || \
    (MOTOR_PWM_TEST_M2_US < MOTOR_ESC_MIN_US) || \
    (MOTOR_PWM_TEST_M2_US > MOTOR_ESC_MAX_US) || \
    (MOTOR_PWM_TEST_M3_US < MOTOR_ESC_MIN_US) || \
    (MOTOR_PWM_TEST_M3_US > MOTOR_ESC_MAX_US) || \
    (MOTOR_PWM_TEST_M4_US < MOTOR_ESC_MIN_US) || \
    (MOTOR_PWM_TEST_M4_US > MOTOR_ESC_MAX_US)
#error "Raw PWM test pulse must be inside MOTOR_ESC_MIN_US..MOTOR_ESC_MAX_US"
#endif

#endif


typedef enum
{
    MOTOR_PWM_STATUS_NOT_INITIALIZED = 0,

    MOTOR_PWM_STATUS_OK,

    MOTOR_PWM_STATUS_INVALID_ARGUMENT,

    MOTOR_PWM_STATUS_TIMER_CLOCK_INVALID,

    MOTOR_PWM_STATUS_CONFIGURATION_INVALID,

    MOTOR_PWM_STATUS_PULSE_OUT_OF_RANGE,

    MOTOR_PWM_STATUS_HARDWARE_VERIFY_FAILED

} motor_pwm_status_t;


typedef struct
{
    uint32_t init_count;

    uint32_t set_count;

    uint32_t safe_set_count;

    uint32_t hard_disable_count;

    uint32_t invalid_request_count;

    uint32_t verify_failure_count;

    uint32_t timer_clock_hz;

    uint32_t counter_clock_hz;

    uint32_t prescaler;

    uint32_t auto_reload;

    uint32_t pwm_hz;

    uint16_t last_m1_us;
    uint16_t last_m2_us;
    uint16_t last_m3_us;
    uint16_t last_m4_us;

    uint32_t initialized;

    uint32_t outputs_enabled;

    uint32_t last_status;

} motor_pwm_diag_t;


extern volatile motor_pwm_status_t
    g_motor_pwm_status;

extern volatile motor_pwm_diag_t
    g_motor_pwm_diag;


/*
 * STM32F1 timer-clock rule from the RCC clock tree:
 *
 *     APB prescaler = 1  -> TIMxCLK = PCLKx
 *     APB prescaler > 1  -> TIMxCLK = 2 * PCLKx
 *
 * hclk_hz and pclk1_hz describe the already-configured clock tree.
 */
uint32_t
motor_pwm_timer_clock_from_apb1(
    uint32_t hclk_hz,
    uint32_t pclk1_hz);


/*
 * Calculate PSC/ARR for the fixed 1 MHz counter resolution.
 * Pure helper used by host tests and by motor_pwm_init().
 */
bool
motor_pwm_calculate_timer_config(
    uint32_t timer_clock_hz,
    uint32_t pwm_hz,
    uint32_t *prescaler_out,
    uint32_t *auto_reload_out);


bool
motor_pwm_pulse_is_valid(
    uint16_t pulse_us);


/*
 * Initialize TIM3 in edge-aligned PWM mode 1.
 *
 * Startup ordering is fail-closed:
 *
 *     hard GPIO LOW
 *     TIM3 disabled / channels disabled
 *     PSC/ARR/PWM/preload configured
 *     all CCRx loaded with safe pulse
 *     UG generated
 *     GPIO pins switched to AF push-pull
 *     channels enabled
 *     counter started
 */
bool
motor_pwm_init(
    uint32_t timer_clock_hz);


/*
 * Write four CCR preload values. Update events are temporarily
 * disabled during the four writes so an overflow cannot publish a
 * partial motor set. The complete set becomes active together at a
 * later normal timer update event.
 */
bool
motor_pwm_set_us(
    uint16_t m1_us,
    uint16_t m2_us,
    uint16_t m3_us,
    uint16_t m4_us);


bool
motor_pwm_set_safe(void);


/*
 * Strong emergency/fatal shutdown:
 *
 *     stop TIM3 / disable channels
 *     return PA6, PA7, PB0, PB1 to verified GPIO LOW
 */
void
motor_pwm_hard_disable(void);


bool
motor_pwm_is_initialized(void);


bool
motor_pwm_configuration_is_valid(void);


#endif /* MOTOR_PWM_H */
