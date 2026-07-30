#include "fault_test.h"

#include <stdint.h>

#ifndef FAULT_TEST_MODE
#define FAULT_TEST_MODE (0)
#endif

#if (FAULT_TEST_MODE < 0) || (FAULT_TEST_MODE > 3)
#error "FAULT_TEST_MODE must be 0, 1, 2 or 3"
#endif

#define NVIC_ISER0_ADDRESS       (0xE000E100UL)
#define NVIC_ISPR0_ADDRESS       (0xE000E200UL)
#define SCB_ICSR_ADDRESS         (0xE000ED04UL)

#define TIM2_IRQ_NUMBER          (28UL)
#define SCB_ICSR_NMIPENDSET      (1UL << 31)

#define REG32(address) (*(volatile uint32_t *)(address))

void fault_test_run(void)
{
#if FAULT_TEST_MODE == 0

    /*
     * Normal build: generate no fault.
     */
    return;

#elif FAULT_TEST_MODE == 1

    /*
     * Enable TIM2 interrupt in the NVIC.
     */
    REG32(NVIC_ISER0_ADDRESS) =
        (1UL << TIM2_IRQ_NUMBER);

    /*
     * Pend the TIM2 interrupt without configuring TIM2.
     *
     * The weak TIM2_IRQHandler calls Default_Handler.
     */
    REG32(NVIC_ISPR0_ADDRESS) =
        (1UL << TIM2_IRQ_NUMBER);

    __asm volatile ("dsb" ::: "memory");
    __asm volatile ("isb" ::: "memory");

#elif FAULT_TEST_MODE == 2

    /*
     * Pend an NMI using the Cortex-M3 ICSR register.
     */
    REG32(SCB_ICSR_ADDRESS) =
        SCB_ICSR_NMIPENDSET;

    __asm volatile ("dsb" ::: "memory");
    __asm volatile ("isb" ::: "memory");

#elif FAULT_TEST_MODE == 3

    /*
     * Execute an undefined instruction.
     *
     * UsageFault is disabled after reset, so the UsageFault
     * escalates to HardFault.
     */
    __asm volatile ("udf #0");

#endif

    /*
     * A fault-test build should not continue beyond this point.
     */
    for (;;)
    {
        __asm volatile ("nop");
    }
}