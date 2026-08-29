#include "motor_pwm.h"

#include "motor_outputs.h"

#include <stdbool.h>
#include <stdint.h>


/*
 * ============================================================
 * STM32F103C8 PERIPHERAL ADDRESSES
 * ============================================================
 */

#define RCC_BASE_ADDRESS       0x40021000UL
#define AFIO_BASE_ADDRESS      0x40010000UL
#define GPIOA_BASE_ADDRESS     0x40010800UL
#define GPIOB_BASE_ADDRESS     0x40010C00UL
#define TIM3_BASE_ADDRESS      0x40000400UL


#define REG32(address) \
    (*(volatile uint32_t *)(uintptr_t)(address))


/* RCC */
#define RCC_APB1RSTR           REG32(RCC_BASE_ADDRESS + 0x10UL)
#define RCC_APB2ENR            REG32(RCC_BASE_ADDRESS + 0x18UL)
#define RCC_APB1ENR            REG32(RCC_BASE_ADDRESS + 0x1CUL)

#define RCC_APB1RSTR_TIM3RST   (1UL << 1)

#define RCC_APB1ENR_TIM3EN     (1UL << 1)

#define RCC_APB2ENR_AFIOEN     (1UL << 0)
#define RCC_APB2ENR_IOPAEN     (1UL << 2)
#define RCC_APB2ENR_IOPBEN     (1UL << 3)


/* AFIO */
#define AFIO_MAPR              REG32(AFIO_BASE_ADDRESS + 0x04UL)
#define AFIO_MAPR_TIM3_REMAP_MASK \
    (3UL << 10)


/* GPIO */
#define GPIOA_CRL              REG32(GPIOA_BASE_ADDRESS + 0x00UL)
#define GPIOB_CRL              REG32(GPIOB_BASE_ADDRESS + 0x00UL)

#define GPIO_CRL_FIELD_MASK(pin) \
    (0xFUL << ((pin) * 4UL))

/*
 * STM32F1 GPIO configuration field CNF[1:0]:MODE[1:0]
 *
 *     CNF  = 10 -> alternate-function push-pull
 *     MODE = 10 -> output mode, maximum 2 MHz
 *
 * Field value = 0b1010 = 0xA.
 */
#define GPIO_AF_PP_2MHZ       0xAUL

#define GPIO_CRL_FIELD_AF_VALUE(pin) \
    (GPIO_AF_PP_2MHZ << ((pin) * 4UL))

#define GPIOA_MOTOR_CRL_MASK \
    (GPIO_CRL_FIELD_MASK(6UL) | \
     GPIO_CRL_FIELD_MASK(7UL))

#define GPIOA_MOTOR_CRL_AF_VALUE \
    (GPIO_CRL_FIELD_AF_VALUE(6UL) | \
     GPIO_CRL_FIELD_AF_VALUE(7UL))

#define GPIOB_MOTOR_CRL_MASK \
    (GPIO_CRL_FIELD_MASK(0UL) | \
     GPIO_CRL_FIELD_MASK(1UL))

#define GPIOB_MOTOR_CRL_AF_VALUE \
    (GPIO_CRL_FIELD_AF_VALUE(0UL) | \
     GPIO_CRL_FIELD_AF_VALUE(1UL))


/* TIM3 register map — general-purpose timer TIM2..TIM5. */
#define TIM3_CR1               REG32(TIM3_BASE_ADDRESS + 0x00UL)
#define TIM3_CR2               REG32(TIM3_BASE_ADDRESS + 0x04UL)
#define TIM3_SMCR              REG32(TIM3_BASE_ADDRESS + 0x08UL)
#define TIM3_DIER              REG32(TIM3_BASE_ADDRESS + 0x0CUL)
#define TIM3_SR                REG32(TIM3_BASE_ADDRESS + 0x10UL)
#define TIM3_EGR               REG32(TIM3_BASE_ADDRESS + 0x14UL)
#define TIM3_CCMR1             REG32(TIM3_BASE_ADDRESS + 0x18UL)
#define TIM3_CCMR2             REG32(TIM3_BASE_ADDRESS + 0x1CUL)
#define TIM3_CCER              REG32(TIM3_BASE_ADDRESS + 0x20UL)
#define TIM3_CNT               REG32(TIM3_BASE_ADDRESS + 0x24UL)
#define TIM3_PSC               REG32(TIM3_BASE_ADDRESS + 0x28UL)
#define TIM3_ARR               REG32(TIM3_BASE_ADDRESS + 0x2CUL)
#define TIM3_CCR1              REG32(TIM3_BASE_ADDRESS + 0x34UL)
#define TIM3_CCR2              REG32(TIM3_BASE_ADDRESS + 0x38UL)
#define TIM3_CCR3              REG32(TIM3_BASE_ADDRESS + 0x3CUL)
#define TIM3_CCR4              REG32(TIM3_BASE_ADDRESS + 0x40UL)


/* TIM3_CR1 */
#define TIM_CR1_CEN            (1UL << 0)
#define TIM_CR1_UDIS           (1UL << 1)
#define TIM_CR1_DIR            (1UL << 4)
#define TIM_CR1_CMS_MASK       (3UL << 5)
#define TIM_CR1_ARPE           (1UL << 7)


/* TIM3_EGR */
#define TIM_EGR_UG             (1UL << 0)


/*
 * CCMR output configuration.
 *
 * PWM mode 1 = OCxM 110
 * preload    = OCxPE 1
 * CCxS       = 00 output
 */
#define TIM_CCMR_OC1PE         (1UL << 3)
#define TIM_CCMR_OC1M_PWM1     (6UL << 4)
#define TIM_CCMR_CC1S_MASK     (3UL << 0)
#define TIM_CCMR_OC1M_MASK     (7UL << 4)

#define TIM_CCMR_OC2PE         (1UL << 11)
#define TIM_CCMR_OC2M_PWM1     (6UL << 12)
#define TIM_CCMR_CC2S_MASK     (3UL << 8)
#define TIM_CCMR_OC2M_MASK     (7UL << 12)

#define TIM_CCMR_CH1_PWM_MASK \
    (TIM_CCMR_CC1S_MASK | \
     TIM_CCMR_OC1PE | \
     TIM_CCMR_OC1M_MASK)

#define TIM_CCMR_CH1_PWM_VALUE \
    (TIM_CCMR_OC1PE | \
     TIM_CCMR_OC1M_PWM1)

#define TIM_CCMR_CH2_PWM_MASK \
    (TIM_CCMR_CC2S_MASK | \
     TIM_CCMR_OC2PE | \
     TIM_CCMR_OC2M_MASK)

#define TIM_CCMR_CH2_PWM_VALUE \
    (TIM_CCMR_OC2PE | \
     TIM_CCMR_OC2M_PWM1)


/* CCMR2 has the same field locations for CH3/CH4. */
#define TIM_CCMR_CH3_PWM_MASK \
    TIM_CCMR_CH1_PWM_MASK

#define TIM_CCMR_CH3_PWM_VALUE \
    TIM_CCMR_CH1_PWM_VALUE

#define TIM_CCMR_CH4_PWM_MASK \
    TIM_CCMR_CH2_PWM_MASK

#define TIM_CCMR_CH4_PWM_VALUE \
    TIM_CCMR_CH2_PWM_VALUE


/* TIM3_CCER */
#define TIM_CCER_CC1E          (1UL << 0)
#define TIM_CCER_CC1P          (1UL << 1)
#define TIM_CCER_CC2E          (1UL << 4)
#define TIM_CCER_CC2P          (1UL << 5)
#define TIM_CCER_CC3E          (1UL << 8)
#define TIM_CCER_CC3P          (1UL << 9)
#define TIM_CCER_CC4E          (1UL << 12)
#define TIM_CCER_CC4P          (1UL << 13)

#define TIM_CCER_MOTOR_ENABLE_MASK \
    (TIM_CCER_CC1E | \
     TIM_CCER_CC2E | \
     TIM_CCER_CC3E | \
     TIM_CCER_CC4E)

#define TIM_CCER_MOTOR_POLARITY_MASK \
    (TIM_CCER_CC1P | \
     TIM_CCER_CC2P | \
     TIM_CCER_CC3P | \
     TIM_CCER_CC4P)


volatile motor_pwm_status_t
    g_motor_pwm_status =
        MOTOR_PWM_STATUS_NOT_INITIALIZED;


volatile motor_pwm_diag_t
    g_motor_pwm_diag;


static bool
    motor_pwm_initialized;


static uint32_t
    expected_prescaler;


static uint32_t
    expected_auto_reload;


static motor_pwm_status_t
record_status(
    motor_pwm_status_t status)
{
    g_motor_pwm_status =
        status;


    g_motor_pwm_diag
        .last_status =
        (uint32_t)
        status;


    return status;
}


static uint32_t
minimum_physical_pulse_us(void)
{
    if (MOTOR_ESC_SAFE_US <
        MOTOR_ESC_MIN_US)
    {
        return
            MOTOR_ESC_SAFE_US;
    }


    return
        MOTOR_ESC_MIN_US;
}


uint32_t
motor_pwm_timer_clock_from_apb1(
    uint32_t hclk_hz,
    uint32_t pclk1_hz)
{
    if ((hclk_hz == 0UL) ||
        (pclk1_hz == 0UL) ||
        (pclk1_hz > hclk_hz) ||
        ((hclk_hz % pclk1_hz) != 0UL))
    {
        return 0UL;
    }


    if (hclk_hz ==
        pclk1_hz)
    {
        return
            pclk1_hz;
    }


    if (pclk1_hz >
        (UINT32_MAX / 2UL))
    {
        return 0UL;
    }


    return
        pclk1_hz * 2UL;
}


bool
motor_pwm_calculate_timer_config(
    uint32_t timer_clock_hz,
    uint32_t pwm_hz,
    uint32_t *prescaler_out,
    uint32_t *auto_reload_out)
{
    uint32_t
        timer_divider;

    uint32_t
        period_counts;


    if ((timer_clock_hz == 0UL) ||
        (pwm_hz == 0UL) ||
        (prescaler_out ==
         (uint32_t *)0) ||
        (auto_reload_out ==
         (uint32_t *)0))
    {
        return false;
    }


    if ((timer_clock_hz %
         MOTOR_PWM_COUNTER_HZ) !=
        0UL)
    {
        return false;
    }


    timer_divider =
        timer_clock_hz /
        MOTOR_PWM_COUNTER_HZ;


    if ((timer_divider == 0UL) ||
        (timer_divider > 65536UL))
    {
        return false;
    }


    if ((MOTOR_PWM_COUNTER_HZ %
         pwm_hz) !=
        0UL)
    {
        return false;
    }


    period_counts =
        MOTOR_PWM_COUNTER_HZ /
        pwm_hz;


    if ((period_counts < 2UL) ||
        (period_counts > 65536UL))
    {
        return false;
    }


    *prescaler_out =
        timer_divider -
        1UL;


    *auto_reload_out =
        period_counts -
        1UL;


    return true;
}


bool
motor_pwm_pulse_is_valid(
    uint16_t pulse_us)
{
    uint32_t
        pulse;


    pulse =
        (uint32_t)
        pulse_us;


    return
        (pulse >=
         minimum_physical_pulse_us()) &&
        (pulse <=
         MOTOR_ESC_MAX_US) &&
        (pulse <
         MOTOR_PWM_PERIOD_US);
}


static void
motor_pwm_clocks_enable(void)
{
    RCC_APB2ENR |=
        RCC_APB2ENR_AFIOEN |
        RCC_APB2ENR_IOPAEN |
        RCC_APB2ENR_IOPBEN;


    RCC_APB1ENR |=
        RCC_APB1ENR_TIM3EN;


    (void)RCC_APB2ENR;
    (void)RCC_APB1ENR;
}


static void
motor_pwm_timer_reset(void)
{
    RCC_APB1RSTR |=
        RCC_APB1RSTR_TIM3RST;


    (void)RCC_APB1RSTR;


    RCC_APB1RSTR &=
        (uint32_t)(
            ~RCC_APB1RSTR_TIM3RST);


    (void)RCC_APB1RSTR;
}


static void
motor_pwm_select_default_tim3_mapping(void)
{
    AFIO_MAPR &=
        (uint32_t)(
            ~AFIO_MAPR_TIM3_REMAP_MASK);
}


static void
motor_pwm_gpio_configure_af(void)
{
    uint32_t
        gpioa_crl;

    uint32_t
        gpiob_crl;


    gpioa_crl =
        GPIOA_CRL;


    gpioa_crl &=
        (uint32_t)(
            ~GPIOA_MOTOR_CRL_MASK);


    gpioa_crl |=
        GPIOA_MOTOR_CRL_AF_VALUE;


    GPIOA_CRL =
        gpioa_crl;


    gpiob_crl =
        GPIOB_CRL;


    gpiob_crl &=
        (uint32_t)(
            ~GPIOB_MOTOR_CRL_MASK);


    gpiob_crl |=
        GPIOB_MOTOR_CRL_AF_VALUE;


    GPIOB_CRL =
        gpiob_crl;
}


bool
motor_pwm_configuration_is_valid(void)
{
    uint32_t
        cr1_mask;

    uint32_t
        cr1_expected;


    if (!motor_pwm_initialized)
    {
        return false;
    }


    if ((RCC_APB1ENR &
         RCC_APB1ENR_TIM3EN) ==
        0UL)
    {
        return false;
    }


    if ((RCC_APB2ENR &
         (RCC_APB2ENR_AFIOEN |
          RCC_APB2ENR_IOPAEN |
          RCC_APB2ENR_IOPBEN)) !=
        (RCC_APB2ENR_AFIOEN |
         RCC_APB2ENR_IOPAEN |
         RCC_APB2ENR_IOPBEN))
    {
        return false;
    }


    if ((AFIO_MAPR &
         AFIO_MAPR_TIM3_REMAP_MASK) !=
        0UL)
    {
        return false;
    }


    if ((GPIOA_CRL &
         GPIOA_MOTOR_CRL_MASK) !=
        GPIOA_MOTOR_CRL_AF_VALUE)
    {
        return false;
    }


    if ((GPIOB_CRL &
         GPIOB_MOTOR_CRL_MASK) !=
        GPIOB_MOTOR_CRL_AF_VALUE)
    {
        return false;
    }


    if ((TIM3_PSC & 0xFFFFUL) !=
        expected_prescaler)
    {
        return false;
    }


    if ((TIM3_ARR & 0xFFFFUL) !=
        expected_auto_reload)
    {
        return false;
    }


    if ((TIM3_CCMR1 &
         TIM_CCMR_CH1_PWM_MASK) !=
        TIM_CCMR_CH1_PWM_VALUE)
    {
        return false;
    }


    if ((TIM3_CCMR1 &
         TIM_CCMR_CH2_PWM_MASK) !=
        TIM_CCMR_CH2_PWM_VALUE)
    {
        return false;
    }


    if ((TIM3_CCMR2 &
         TIM_CCMR_CH3_PWM_MASK) !=
        TIM_CCMR_CH3_PWM_VALUE)
    {
        return false;
    }


    if ((TIM3_CCMR2 &
         TIM_CCMR_CH4_PWM_MASK) !=
        TIM_CCMR_CH4_PWM_VALUE)
    {
        return false;
    }


    if ((TIM3_CCER &
         TIM_CCER_MOTOR_ENABLE_MASK) !=
        TIM_CCER_MOTOR_ENABLE_MASK)
    {
        return false;
    }


    if ((TIM3_CCER &
         TIM_CCER_MOTOR_POLARITY_MASK) !=
        0UL)
    {
        return false;
    }


    cr1_mask =
        TIM_CR1_CEN |
        TIM_CR1_UDIS |
        TIM_CR1_DIR |
        TIM_CR1_CMS_MASK |
        TIM_CR1_ARPE;


    cr1_expected =
        TIM_CR1_CEN |
        TIM_CR1_ARPE;


    if ((TIM3_CR1 &
         cr1_mask) !=
        cr1_expected)
    {
        return false;
    }


    return true;
}


bool
motor_pwm_init(
    uint32_t timer_clock_hz)
{
    uint32_t
        prescaler;

    uint32_t
        auto_reload;


    motor_pwm_initialized =
        false;


    expected_prescaler =
        0UL;


    expected_auto_reload =
        0UL;


    g_motor_pwm_diag =
        (motor_pwm_diag_t){0};


    g_motor_pwm_status =
        MOTOR_PWM_STATUS_NOT_INITIALIZED;


    /*
     * Begin from the already-established strongest physical state.
     */
    motor_outputs_force_safe();


    if (!motor_pwm_calculate_timer_config(
            timer_clock_hz,
            MOTOR_ESC_PWM_HZ,
            &prescaler,
            &auto_reload))
    {
        g_motor_pwm_diag
            .invalid_request_count++;


        (void)record_status(
            MOTOR_PWM_STATUS_TIMER_CLOCK_INVALID);


        return false;
    }


    if (!motor_pwm_pulse_is_valid(
            (uint16_t)
            MOTOR_ESC_SAFE_US))
    {
        g_motor_pwm_diag
            .invalid_request_count++;


        (void)record_status(
            MOTOR_PWM_STATUS_CONFIGURATION_INVALID);


        return false;
    }


    expected_prescaler =
        prescaler;


    expected_auto_reload =
        auto_reload;


    motor_pwm_clocks_enable();


    motor_pwm_timer_reset();


    /*
     * Keep both the counter and all four output channels disabled
     * until every timing/output register contains a safe value.
     */
    TIM3_CR1 =
        0UL;

    TIM3_CCER =
        0UL;

    TIM3_DIER =
        0UL;

    TIM3_SMCR =
        0UL;

    TIM3_CR2 =
        0UL;


    motor_pwm_select_default_tim3_mapping();


    TIM3_PSC =
        prescaler;


    TIM3_ARR =
        auto_reload;


    TIM3_CCMR1 =
        TIM_CCMR_CH1_PWM_VALUE |
        TIM_CCMR_CH2_PWM_VALUE;


    TIM3_CCMR2 =
        TIM_CCMR_CH3_PWM_VALUE |
        TIM_CCMR_CH4_PWM_VALUE;


    TIM3_CCR1 =
        MOTOR_ESC_SAFE_US;

    TIM3_CCR2 =
        MOTOR_ESC_SAFE_US;

    TIM3_CCR3 =
        MOTOR_ESC_SAFE_US;

    TIM3_CCR4 =
        MOTOR_ESC_SAFE_US;


    TIM3_CNT =
        0UL;


    TIM3_CR1 =
        TIM_CR1_ARPE;


    /*
     * Load PSC/ARR/CCR preload values before the counter starts.
     */
    TIM3_EGR =
        TIM_EGR_UG;


    TIM3_SR =
        0UL;


    /*
     * Only now connect the physical pins to the timer outputs.
     */
    motor_pwm_gpio_configure_af();


    TIM3_CCER =
        TIM_CCER_MOTOR_ENABLE_MASK;


    TIM3_CR1 =
        TIM_CR1_ARPE |
        TIM_CR1_CEN;


    motor_pwm_initialized =
        true;


    g_motor_pwm_diag
        .init_count =
        1UL;


    g_motor_pwm_diag
        .timer_clock_hz =
        timer_clock_hz;


    g_motor_pwm_diag
        .counter_clock_hz =
        MOTOR_PWM_COUNTER_HZ;


    g_motor_pwm_diag
        .prescaler =
        prescaler;


    g_motor_pwm_diag
        .auto_reload =
        auto_reload;


    g_motor_pwm_diag
        .pwm_hz =
        MOTOR_ESC_PWM_HZ;


    g_motor_pwm_diag
        .last_m1_us =
        (uint16_t)
        MOTOR_ESC_SAFE_US;

    g_motor_pwm_diag
        .last_m2_us =
        (uint16_t)
        MOTOR_ESC_SAFE_US;

    g_motor_pwm_diag
        .last_m3_us =
        (uint16_t)
        MOTOR_ESC_SAFE_US;

    g_motor_pwm_diag
        .last_m4_us =
        (uint16_t)
        MOTOR_ESC_SAFE_US;


    g_motor_pwm_diag
        .initialized =
        1UL;


    g_motor_pwm_diag
        .outputs_enabled =
        1UL;


    if (!motor_pwm_configuration_is_valid())
    {
        g_motor_pwm_diag
            .verify_failure_count++;


        (void)record_status(
            MOTOR_PWM_STATUS_HARDWARE_VERIFY_FAILED);


        motor_pwm_hard_disable();


        return false;
    }


    (void)record_status(
        MOTOR_PWM_STATUS_OK);


    return true;
}


bool
motor_pwm_set_us(
    uint16_t m1_us,
    uint16_t m2_us,
    uint16_t m3_us,
    uint16_t m4_us)
{
    if (!motor_pwm_initialized)
    {
        g_motor_pwm_diag
            .invalid_request_count++;


        (void)record_status(
            MOTOR_PWM_STATUS_NOT_INITIALIZED);


        return false;
    }


    if ((!motor_pwm_pulse_is_valid(
             m1_us)) ||
        (!motor_pwm_pulse_is_valid(
             m2_us)) ||
        (!motor_pwm_pulse_is_valid(
             m3_us)) ||
        (!motor_pwm_pulse_is_valid(
             m4_us)))
    {
        g_motor_pwm_diag
            .invalid_request_count++;


        (void)record_status(
            MOTOR_PWM_STATUS_PULSE_OUT_OF_RANGE);


        return false;
    }


    if (!motor_pwm_configuration_is_valid())
    {
        g_motor_pwm_diag
            .verify_failure_count++;


        (void)record_status(
            MOTOR_PWM_STATUS_HARDWARE_VERIFY_FAILED);


        return false;
    }


    /*
     * Prevent a natural update event from transferring only a subset
     * of the four preload writes. After UDIS is cleared, the next
     * normal timer update publishes the complete four-channel set.
     */
    TIM3_CR1 |=
        TIM_CR1_UDIS;


    TIM3_CCR1 =
        (uint32_t)m1_us;

    TIM3_CCR2 =
        (uint32_t)m2_us;

    TIM3_CCR3 =
        (uint32_t)m3_us;

    TIM3_CCR4 =
        (uint32_t)m4_us;


    TIM3_CR1 &=
        (uint32_t)(
            ~TIM_CR1_UDIS);


    g_motor_pwm_diag
        .set_count++;


    g_motor_pwm_diag
        .last_m1_us =
        m1_us;

    g_motor_pwm_diag
        .last_m2_us =
        m2_us;

    g_motor_pwm_diag
        .last_m3_us =
        m3_us;

    g_motor_pwm_diag
        .last_m4_us =
        m4_us;


    (void)record_status(
        MOTOR_PWM_STATUS_OK);


    return true;
}


bool
motor_pwm_set_safe(void)
{
    if (!motor_pwm_set_us(
            (uint16_t)MOTOR_ESC_SAFE_US,
            (uint16_t)MOTOR_ESC_SAFE_US,
            (uint16_t)MOTOR_ESC_SAFE_US,
            (uint16_t)MOTOR_ESC_SAFE_US))
    {
        return false;
    }


    g_motor_pwm_diag
        .safe_set_count++;


    return true;
}


void
motor_pwm_hard_disable(void)
{
    if ((RCC_APB1ENR &
         RCC_APB1ENR_TIM3EN) !=
        0UL)
    {
        TIM3_CCER =
            0UL;


        TIM3_CR1 &=
            (uint32_t)(
                ~TIM_CR1_CEN);
    }


    motor_outputs_force_safe();


    motor_pwm_initialized =
        false;


    g_motor_pwm_diag
        .initialized =
        0UL;


    g_motor_pwm_diag
        .outputs_enabled =
        0UL;


    g_motor_pwm_diag
        .hard_disable_count++;
}


bool
motor_pwm_is_initialized(void)
{
    return
        motor_pwm_initialized;
}
