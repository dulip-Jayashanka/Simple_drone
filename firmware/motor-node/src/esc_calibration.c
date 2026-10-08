#include "motor_pwm.h"
#include "system_time.h"
#include "uart_diag.h"
#include <stdbool.h>
#include <stdint.h>

/* Standalone, deliberately isolated ESC endpoint calibration console.
 * USART1: PA9 TX / PA10 RX, 115200 8N1.
 * The shared diagnostic UART initializes TX; this module adds RX.
 */
#define RCC_APB2ENR (*(volatile uint32_t *)0x40021018UL)
#define GPIOA_CRH    (*(volatile uint32_t *)0x40010804UL)
#define USART1_SR    (*(volatile uint32_t *)0x40013800UL)
#define USART1_DR    (*(volatile uint32_t *)0x40013804UL)
#define USART1_CR1   (*(volatile uint32_t *)0x4001380CUL)
#define RXNE         (1UL << 5)
#define ORE          (1UL << 3)
#define FE           (1UL << 1)
#define NE           (1UL << 2)
#define RE           (1UL << 2)
#define HIGH_LIMIT_MS 30000UL

static bool high_active;
static bool shutdown_latched;
static uint32_t high_start_ms;

static bool set_all(uint16_t us)
{
    return motor_pwm_set_us(us, us, us, us);
}

void esc_calibration_init(void)
{
    uint32_t crh = GPIOA_CRH;
    /* PA10 floating input: CNF=01 MODE=00 => 0x4 */
    crh &= ~(0xFUL << 8);
    crh |= (0x4UL << 8);
    GPIOA_CRH = crh;
    USART1_CR1 |= RE;
    high_active = false;
    shutdown_latched = false;
    if (!set_all((uint16_t)MOTOR_ESC_MIN_US))
    {
        shutdown_latched = true;
        motor_pwm_hard_disable();
    }
    (void)uart_diag_write_line("[ESC CAL] LOW 1000us at boot. Remove propellers.");
    (void)uart_diag_write_line("[ESC CAL] H=HIGH; L=LOW; X=HARD STOP (reset required).");
    (void)uart_diag_write_line("[ESC CAL] Connect ESC battery ONLY after selecting H.");
}

void esc_calibration_process(void)
{
    uint32_t sr;
    char command;

    if (shutdown_latched)
        return;

    if (high_active && (uint32_t)(millis() - high_start_ms) >= HIGH_LIMIT_MS)
    {
        if (!set_all((uint16_t)MOTOR_ESC_MIN_US))
        {
            motor_pwm_hard_disable();
            shutdown_latched = true;
            return;
        }
        high_active = false;
        (void)uart_diag_write_line("[ESC CAL] HIGH timeout; returned to LOW.");
    }

    sr = USART1_SR;
    if ((sr & (RXNE | ORE | FE | NE)) == 0UL)
        return;

    /* SR followed by DR clears error conditions as well as RXNE. */
    command = (char)(uint8_t)USART1_DR;
    if ((sr & (ORE | FE | NE)) != 0UL)
        return;

    if (command == 'x' || command == 'X')
    {
        motor_pwm_hard_disable();
        shutdown_latched = true;
        high_active = false;
        (void)uart_diag_write_line("[ESC CAL] HARD STOP. Reset board to restart.");
    }
    else if (command == 'l' || command == 'L')
    {
        if (!set_all((uint16_t)MOTOR_ESC_MIN_US))
        {
            motor_pwm_hard_disable();
            shutdown_latched = true;
            return;
        }
        high_active = false;
        (void)uart_diag_write_line("[ESC CAL] LOW endpoint selected.");
    }
    else if (command == 'h' || command == 'H')
    {
        if (!set_all((uint16_t)MOTOR_ESC_MAX_US))
        {
            motor_pwm_hard_disable();
            shutdown_latched = true;
            return;
        }
        high_start_ms = millis();
        high_active = true;
        (void)uart_diag_write_line("[ESC CAL] HIGH endpoint selected. Await beep, then L.");
    }
}
