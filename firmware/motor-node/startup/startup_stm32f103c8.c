#include <stdint.h>

/*
 * Linker symbols defined in stm32f103c8.ld.
 *
 * These are addresses created by the linker.
 * They are not normal C variables.
 */
extern uint32_t _estack;

extern uint32_t _sidata;
extern uint32_t _sdata;
extern uint32_t _edata;

extern uint32_t _sbss;
extern uint32_t _ebss;

/* Application entry point. */
extern int main(void);

/* Core startup handlers. */
void Reset_Handler(void);
void Default_Handler(void);

/*
 * Create a replaceable handler that initially points to
 * Default_Handler.
 */
#define WEAK_DEFAULT_HANDLER(handler_name)                     \
    void handler_name(void)                                   \
        __attribute__((weak, alias("Default_Handler")))

/* Cortex-M3 exception handlers. */
WEAK_DEFAULT_HANDLER(NMI_Handler);
WEAK_DEFAULT_HANDLER(HardFault_Handler);
WEAK_DEFAULT_HANDLER(MemManage_Handler);
WEAK_DEFAULT_HANDLER(BusFault_Handler);
WEAK_DEFAULT_HANDLER(UsageFault_Handler);
WEAK_DEFAULT_HANDLER(SVC_Handler);
WEAK_DEFAULT_HANDLER(DebugMon_Handler);
WEAK_DEFAULT_HANDLER(PendSV_Handler);
WEAK_DEFAULT_HANDLER(SysTick_Handler);

/* STM32F103C8T6 peripheral interrupt handlers. */
WEAK_DEFAULT_HANDLER(WWDG_IRQHandler);
WEAK_DEFAULT_HANDLER(PVD_IRQHandler);
WEAK_DEFAULT_HANDLER(TAMPER_IRQHandler);
WEAK_DEFAULT_HANDLER(RTC_IRQHandler);
WEAK_DEFAULT_HANDLER(FLASH_IRQHandler);
WEAK_DEFAULT_HANDLER(RCC_IRQHandler);

WEAK_DEFAULT_HANDLER(EXTI0_IRQHandler);
WEAK_DEFAULT_HANDLER(EXTI1_IRQHandler);
WEAK_DEFAULT_HANDLER(EXTI2_IRQHandler);
WEAK_DEFAULT_HANDLER(EXTI3_IRQHandler);
WEAK_DEFAULT_HANDLER(EXTI4_IRQHandler);

WEAK_DEFAULT_HANDLER(DMA1_Channel1_IRQHandler);
WEAK_DEFAULT_HANDLER(DMA1_Channel2_IRQHandler);
WEAK_DEFAULT_HANDLER(DMA1_Channel3_IRQHandler);
WEAK_DEFAULT_HANDLER(DMA1_Channel4_IRQHandler);
WEAK_DEFAULT_HANDLER(DMA1_Channel5_IRQHandler);
WEAK_DEFAULT_HANDLER(DMA1_Channel6_IRQHandler);
WEAK_DEFAULT_HANDLER(DMA1_Channel7_IRQHandler);

WEAK_DEFAULT_HANDLER(ADC1_2_IRQHandler);

WEAK_DEFAULT_HANDLER(USB_HP_CAN1_TX_IRQHandler);
WEAK_DEFAULT_HANDLER(USB_LP_CAN1_RX0_IRQHandler);
WEAK_DEFAULT_HANDLER(CAN1_RX1_IRQHandler);
WEAK_DEFAULT_HANDLER(CAN1_SCE_IRQHandler);

WEAK_DEFAULT_HANDLER(EXTI9_5_IRQHandler);

WEAK_DEFAULT_HANDLER(TIM1_BRK_IRQHandler);
WEAK_DEFAULT_HANDLER(TIM1_UP_IRQHandler);
WEAK_DEFAULT_HANDLER(TIM1_TRG_COM_IRQHandler);
WEAK_DEFAULT_HANDLER(TIM1_CC_IRQHandler);

WEAK_DEFAULT_HANDLER(TIM2_IRQHandler);
WEAK_DEFAULT_HANDLER(TIM3_IRQHandler);
WEAK_DEFAULT_HANDLER(TIM4_IRQHandler);

WEAK_DEFAULT_HANDLER(I2C1_EV_IRQHandler);
WEAK_DEFAULT_HANDLER(I2C1_ER_IRQHandler);
WEAK_DEFAULT_HANDLER(I2C2_EV_IRQHandler);
WEAK_DEFAULT_HANDLER(I2C2_ER_IRQHandler);

WEAK_DEFAULT_HANDLER(SPI1_IRQHandler);
WEAK_DEFAULT_HANDLER(SPI2_IRQHandler);

WEAK_DEFAULT_HANDLER(USART1_IRQHandler);
WEAK_DEFAULT_HANDLER(USART2_IRQHandler);
WEAK_DEFAULT_HANDLER(USART3_IRQHandler);

WEAK_DEFAULT_HANDLER(EXTI15_10_IRQHandler);
WEAK_DEFAULT_HANDLER(RTC_Alarm_IRQHandler);
WEAK_DEFAULT_HANDLER(USBWakeUp_IRQHandler);

/*
 * Interrupt vector table.
 *
 * The linker script places this section at:
 *
 *     0x08000000
 *
 * Every element is one 32-bit vector-table entry.
 */
__attribute__((section(".isr_vector"), used))
const uintptr_t vector_table[] =
{
    /* Cortex-M3 processor vectors */

    (uintptr_t)&_estack,                 /* 0x000: Initial MSP */
    (uintptr_t)Reset_Handler,            /* 0x004: Reset */
    (uintptr_t)NMI_Handler,              /* 0x008: NMI */
    (uintptr_t)HardFault_Handler,        /* 0x00C: HardFault */
    (uintptr_t)MemManage_Handler,        /* 0x010: MemManage */
    (uintptr_t)BusFault_Handler,         /* 0x014: BusFault */
    (uintptr_t)UsageFault_Handler,       /* 0x018: UsageFault */

    (uintptr_t)0,                        /* 0x01C: Reserved */
    (uintptr_t)0,                        /* 0x020: Reserved */
    (uintptr_t)0,                        /* 0x024: Reserved */
    (uintptr_t)0,                        /* 0x028: Reserved */

    (uintptr_t)SVC_Handler,              /* 0x02C: SVCall */
    (uintptr_t)DebugMon_Handler,         /* 0x030: Debug monitor */

    (uintptr_t)0,                        /* 0x034: Reserved */

    (uintptr_t)PendSV_Handler,           /* 0x038: PendSV */
    (uintptr_t)SysTick_Handler,          /* 0x03C: SysTick */

    /* STM32F103 external interrupt vectors */

    (uintptr_t)WWDG_IRQHandler,          /* 0x040: IRQ 0 */
    (uintptr_t)PVD_IRQHandler,           /* 0x044: IRQ 1 */
    (uintptr_t)TAMPER_IRQHandler,        /* 0x048: IRQ 2 */
    (uintptr_t)RTC_IRQHandler,           /* 0x04C: IRQ 3 */
    (uintptr_t)FLASH_IRQHandler,         /* 0x050: IRQ 4 */
    (uintptr_t)RCC_IRQHandler,           /* 0x054: IRQ 5 */

    (uintptr_t)EXTI0_IRQHandler,         /* 0x058: IRQ 6 */
    (uintptr_t)EXTI1_IRQHandler,         /* 0x05C: IRQ 7 */
    (uintptr_t)EXTI2_IRQHandler,         /* 0x060: IRQ 8 */
    (uintptr_t)EXTI3_IRQHandler,         /* 0x064: IRQ 9 */
    (uintptr_t)EXTI4_IRQHandler,         /* 0x068: IRQ 10 */

    (uintptr_t)DMA1_Channel1_IRQHandler, /* 0x06C: IRQ 11 */
    (uintptr_t)DMA1_Channel2_IRQHandler, /* 0x070: IRQ 12 */
    (uintptr_t)DMA1_Channel3_IRQHandler, /* 0x074: IRQ 13 */
    (uintptr_t)DMA1_Channel4_IRQHandler, /* 0x078: IRQ 14 */
    (uintptr_t)DMA1_Channel5_IRQHandler, /* 0x07C: IRQ 15 */
    (uintptr_t)DMA1_Channel6_IRQHandler, /* 0x080: IRQ 16 */
    (uintptr_t)DMA1_Channel7_IRQHandler, /* 0x084: IRQ 17 */

    (uintptr_t)ADC1_2_IRQHandler,        /* 0x088: IRQ 18 */

    (uintptr_t)USB_HP_CAN1_TX_IRQHandler,  /* 0x08C: IRQ 19 */
    (uintptr_t)USB_LP_CAN1_RX0_IRQHandler, /* 0x090: IRQ 20 */
    (uintptr_t)CAN1_RX1_IRQHandler,        /* 0x094: IRQ 21 */
    (uintptr_t)CAN1_SCE_IRQHandler,        /* 0x098: IRQ 22 */

    (uintptr_t)EXTI9_5_IRQHandler,       /* 0x09C: IRQ 23 */

    (uintptr_t)TIM1_BRK_IRQHandler,      /* 0x0A0: IRQ 24 */
    (uintptr_t)TIM1_UP_IRQHandler,       /* 0x0A4: IRQ 25 */
    (uintptr_t)TIM1_TRG_COM_IRQHandler,  /* 0x0A8: IRQ 26 */
    (uintptr_t)TIM1_CC_IRQHandler,       /* 0x0AC: IRQ 27 */

    (uintptr_t)TIM2_IRQHandler,          /* 0x0B0: IRQ 28 */
    (uintptr_t)TIM3_IRQHandler,          /* 0x0B4: IRQ 29 */
    (uintptr_t)TIM4_IRQHandler,          /* 0x0B8: IRQ 30 */

    (uintptr_t)I2C1_EV_IRQHandler,       /* 0x0BC: IRQ 31 */
    (uintptr_t)I2C1_ER_IRQHandler,       /* 0x0C0: IRQ 32 */
    (uintptr_t)I2C2_EV_IRQHandler,       /* 0x0C4: IRQ 33 */
    (uintptr_t)I2C2_ER_IRQHandler,       /* 0x0C8: IRQ 34 */

    (uintptr_t)SPI1_IRQHandler,          /* 0x0CC: IRQ 35 */
    (uintptr_t)SPI2_IRQHandler,          /* 0x0D0: IRQ 36 */

    (uintptr_t)USART1_IRQHandler,        /* 0x0D4: IRQ 37 */
    (uintptr_t)USART2_IRQHandler,        /* 0x0D8: IRQ 38 */
    (uintptr_t)USART3_IRQHandler,        /* 0x0DC: IRQ 39 */

    (uintptr_t)EXTI15_10_IRQHandler,     /* 0x0E0: IRQ 40 */
    (uintptr_t)RTC_Alarm_IRQHandler,     /* 0x0E4: IRQ 41 */
    (uintptr_t)USBWakeUp_IRQHandler      /* 0x0E8: IRQ 42 */
};

/*
 * Runs when an interrupt has no real handler yet.
 */
void Default_Handler(void)
{
    while (1)
    {
        /*
         * Stay here so an attached debugger can identify
         * the unexpected interrupt.
         */
    }
}

/*
 * First C function executed after reset.
 */
void Reset_Handler(void)
{
    uint32_t *flash_source;
    uint32_t *ram_destination;

    /*
     * Copy initialized variables from their Flash load
     * addresses to their RAM runtime addresses.
     *
     * Flash source:
     *     &_sidata
     *
     * RAM destination:
     *     &_sdata up to, but not including, &_edata
     */
    flash_source = &_sidata;
    ram_destination = &_sdata;

    while (ram_destination < &_edata)
    {
        *ram_destination = *flash_source;

        ram_destination++;
        flash_source++;
    }

    /*
     * Clear the .bss section.
     *
     * All uninitialized and zero-initialized global/static
     * variables must contain zero before main() begins.
     */
    ram_destination = &_sbss;

    while (ram_destination < &_ebss)
    {
        *ram_destination = 0U;
        ram_destination++;
    }

    /*
     * RAM initialization is complete.
     * Enter the main application.
     */
    (void)main();

    /*
     * main() should never return in embedded firmware.
     */
    while (1)
    {
    }
}