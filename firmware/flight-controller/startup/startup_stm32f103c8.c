#include "fault_handlers.h"

#include <stdint.h>

extern uint32_t _estack;
extern uint32_t _sidata;
extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t _sbss;
extern uint32_t _ebss;

extern int main(void);

void Reset_Handler(void);

#define WEAK_DEFAULT_HANDLER(handler_name)       \
    void handler_name(void) __attribute__((weak)); \
    void handler_name(void)                     \
    {                                           \
        Default_Handler();                      \
    }

/* Cortex-M3 configurable exceptions. */
WEAK_DEFAULT_HANDLER(MemManage_Handler)
WEAK_DEFAULT_HANDLER(BusFault_Handler)
WEAK_DEFAULT_HANDLER(UsageFault_Handler)
WEAK_DEFAULT_HANDLER(SVC_Handler)
WEAK_DEFAULT_HANDLER(DebugMon_Handler)
WEAK_DEFAULT_HANDLER(PendSV_Handler)
WEAK_DEFAULT_HANDLER(SysTick_Handler)

/* STM32F103C8 peripheral interrupts: IRQ 0-42. */
WEAK_DEFAULT_HANDLER(WWDG_IRQHandler)
WEAK_DEFAULT_HANDLER(PVD_IRQHandler)
WEAK_DEFAULT_HANDLER(TAMPER_IRQHandler)
WEAK_DEFAULT_HANDLER(RTC_IRQHandler)
WEAK_DEFAULT_HANDLER(FLASH_IRQHandler)
WEAK_DEFAULT_HANDLER(RCC_IRQHandler)

WEAK_DEFAULT_HANDLER(EXTI0_IRQHandler)
WEAK_DEFAULT_HANDLER(EXTI1_IRQHandler)
WEAK_DEFAULT_HANDLER(EXTI2_IRQHandler)
WEAK_DEFAULT_HANDLER(EXTI3_IRQHandler)
WEAK_DEFAULT_HANDLER(EXTI4_IRQHandler)

WEAK_DEFAULT_HANDLER(DMA1_Channel1_IRQHandler)
WEAK_DEFAULT_HANDLER(DMA1_Channel2_IRQHandler)
WEAK_DEFAULT_HANDLER(DMA1_Channel3_IRQHandler)
WEAK_DEFAULT_HANDLER(DMA1_Channel4_IRQHandler)
WEAK_DEFAULT_HANDLER(DMA1_Channel5_IRQHandler)
WEAK_DEFAULT_HANDLER(DMA1_Channel6_IRQHandler)
WEAK_DEFAULT_HANDLER(DMA1_Channel7_IRQHandler)

WEAK_DEFAULT_HANDLER(ADC1_2_IRQHandler)

WEAK_DEFAULT_HANDLER(USB_HP_CAN1_TX_IRQHandler)
WEAK_DEFAULT_HANDLER(USB_LP_CAN1_RX0_IRQHandler)
WEAK_DEFAULT_HANDLER(CAN1_RX1_IRQHandler)
WEAK_DEFAULT_HANDLER(CAN1_SCE_IRQHandler)

WEAK_DEFAULT_HANDLER(EXTI9_5_IRQHandler)

WEAK_DEFAULT_HANDLER(TIM1_BRK_IRQHandler)
WEAK_DEFAULT_HANDLER(TIM1_UP_IRQHandler)
WEAK_DEFAULT_HANDLER(TIM1_TRG_COM_IRQHandler)
WEAK_DEFAULT_HANDLER(TIM1_CC_IRQHandler)

WEAK_DEFAULT_HANDLER(TIM2_IRQHandler)
WEAK_DEFAULT_HANDLER(TIM3_IRQHandler)
WEAK_DEFAULT_HANDLER(TIM4_IRQHandler)

WEAK_DEFAULT_HANDLER(I2C1_EV_IRQHandler)
WEAK_DEFAULT_HANDLER(I2C1_ER_IRQHandler)
WEAK_DEFAULT_HANDLER(I2C2_EV_IRQHandler)
WEAK_DEFAULT_HANDLER(I2C2_ER_IRQHandler)

WEAK_DEFAULT_HANDLER(SPI1_IRQHandler)
WEAK_DEFAULT_HANDLER(SPI2_IRQHandler)

WEAK_DEFAULT_HANDLER(USART1_IRQHandler)
WEAK_DEFAULT_HANDLER(USART2_IRQHandler)
WEAK_DEFAULT_HANDLER(USART3_IRQHandler)

WEAK_DEFAULT_HANDLER(EXTI15_10_IRQHandler)
WEAK_DEFAULT_HANDLER(RTC_Alarm_IRQHandler)
WEAK_DEFAULT_HANDLER(USBWakeUp_IRQHandler)

/*
 * 16 Cortex-M3 vectors + 43 STM32 IRQ vectors
 * = 59 entries.
 */
__attribute__((section(".isr_vector"), used))
const uintptr_t vector_table[] =
{
    (uintptr_t)&_estack,
    (uintptr_t)Reset_Handler,
    (uintptr_t)NMI_Handler,
    (uintptr_t)HardFault_Handler,
    (uintptr_t)MemManage_Handler,
    (uintptr_t)BusFault_Handler,
    (uintptr_t)UsageFault_Handler,
    (uintptr_t)0,
    (uintptr_t)0,
    (uintptr_t)0,
    (uintptr_t)0,
    (uintptr_t)SVC_Handler,
    (uintptr_t)DebugMon_Handler,
    (uintptr_t)0,
    (uintptr_t)PendSV_Handler,
    (uintptr_t)SysTick_Handler,

    (uintptr_t)WWDG_IRQHandler,
    (uintptr_t)PVD_IRQHandler,
    (uintptr_t)TAMPER_IRQHandler,
    (uintptr_t)RTC_IRQHandler,
    (uintptr_t)FLASH_IRQHandler,
    (uintptr_t)RCC_IRQHandler,

    /* IRQ 6: MPU9250 DATA_RDY through PA0. */
    (uintptr_t)EXTI0_IRQHandler,

    (uintptr_t)EXTI1_IRQHandler,
    (uintptr_t)EXTI2_IRQHandler,
    (uintptr_t)EXTI3_IRQHandler,
    (uintptr_t)EXTI4_IRQHandler,

    (uintptr_t)DMA1_Channel1_IRQHandler,
    (uintptr_t)DMA1_Channel2_IRQHandler,
    (uintptr_t)DMA1_Channel3_IRQHandler,
    (uintptr_t)DMA1_Channel4_IRQHandler,
    (uintptr_t)DMA1_Channel5_IRQHandler,
    (uintptr_t)DMA1_Channel6_IRQHandler,
    (uintptr_t)DMA1_Channel7_IRQHandler,

    (uintptr_t)ADC1_2_IRQHandler,

    (uintptr_t)USB_HP_CAN1_TX_IRQHandler,
    (uintptr_t)USB_LP_CAN1_RX0_IRQHandler,
    (uintptr_t)CAN1_RX1_IRQHandler,
    (uintptr_t)CAN1_SCE_IRQHandler,

    (uintptr_t)EXTI9_5_IRQHandler,

    (uintptr_t)TIM1_BRK_IRQHandler,
    (uintptr_t)TIM1_UP_IRQHandler,
    (uintptr_t)TIM1_TRG_COM_IRQHandler,
    (uintptr_t)TIM1_CC_IRQHandler,

    (uintptr_t)TIM2_IRQHandler,
    (uintptr_t)TIM3_IRQHandler,
    (uintptr_t)TIM4_IRQHandler,

    (uintptr_t)I2C1_EV_IRQHandler,
    (uintptr_t)I2C1_ER_IRQHandler,
    (uintptr_t)I2C2_EV_IRQHandler,
    (uintptr_t)I2C2_ER_IRQHandler,

    (uintptr_t)SPI1_IRQHandler,
    (uintptr_t)SPI2_IRQHandler,

    (uintptr_t)USART1_IRQHandler,
    (uintptr_t)USART2_IRQHandler,
    (uintptr_t)USART3_IRQHandler,

    (uintptr_t)EXTI15_10_IRQHandler,
    (uintptr_t)RTC_Alarm_IRQHandler,
    (uintptr_t)USBWakeUp_IRQHandler
};

void Reset_Handler(void)
{
    uint32_t *flash_source;
    uint32_t *ram_destination;

    /*
     * Copy initialized .data values from Flash to RAM.
     */
    flash_source = &_sidata;
    ram_destination = &_sdata;

    while (ram_destination < &_edata)
    {
        *ram_destination = *flash_source;

        flash_source++;
        ram_destination++;
    }

    /*
     * Clear zero-initialized .bss memory.
     */
    ram_destination = &_sbss;

    while (ram_destination < &_ebss)
    {
        *ram_destination = 0UL;
        ram_destination++;
    }

    (void)main();

    /*
     * Returning from main is a fatal condition.
     */
    Default_Handler();
}