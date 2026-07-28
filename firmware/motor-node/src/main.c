#include "system_clock.h"

#include <stdint.h>

/*
 * These volatile variables are intentionally visible to a debugger.
 * Inspect them after main() runs to verify Phase 2.1.4 in GDB or Renode.
 */
volatile system_clock_status_t g_clock_status;
volatile uint32_t g_sysclk_hz;
volatile uint32_t g_hclk_hz;
volatile uint32_t g_pclk1_hz;
volatile uint32_t g_pclk2_hz;
volatile uint32_t g_adcclk_hz;

int main(void)
{
    g_clock_status = system_clock_init();

    /*
     * Read through the public interface rather than duplicating clock
     * constants inside the application.
     */
    g_sysclk_hz = system_clock_get_sysclk_hz();
    g_hclk_hz = system_clock_get_hclk_hz();
    g_pclk1_hz = system_clock_get_pclk1_hz();
    g_pclk2_hz = system_clock_get_pclk2_hz();
    g_adcclk_hz = system_clock_get_adcclk_hz();

    if (g_clock_status != SYSTEM_CLOCK_OK)
    {
        /*
         * The status value identifies the failure. The CPU remains usable
         * on HSI at 8 MHz, which allows a debugger to inspect the state.
         */
        while (1)
        {
            __asm volatile ("nop");
        }
    }

    /*
     * Later phases will initialize GPIO, UART, timers, I2C and CAN here.
     */
    while (1)
    {
        __asm volatile ("nop");
    }
}