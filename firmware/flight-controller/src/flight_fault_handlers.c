#include "fault_handlers.h"

#include <stdint.h>

volatile fault_record_t g_fault_record;

static __attribute__((noreturn)) void stop(void)
{
    __asm volatile ("cpsid i" ::: "memory");

    for (;;)
    {
        __asm volatile ("nop");
    }
}

void fault_record_clear(void)
{
    volatile uint32_t *words =
        (volatile uint32_t *)&g_fault_record;

    uint32_t count =
        (uint32_t)(sizeof(g_fault_record) /
                   sizeof(uint32_t));

    uint32_t index;

    for (index = 0UL; index < count; index++)
    {
        words[index] = 0UL;
    }
}

void Default_Handler(void)
{
    g_fault_record.type =
        FAULT_TYPE_DEFAULT_HANDLER;

    g_fault_record.magic =
        FAULT_RECORD_MAGIC;

    stop();
}

void NMI_Handler(void)
{
    g_fault_record.type =
        FAULT_TYPE_NMI;

    g_fault_record.magic =
        FAULT_RECORD_MAGIC;

    stop();
}

void HardFault_Handler(void)
{
    g_fault_record.type =
        FAULT_TYPE_HARDFAULT;

    g_fault_record.magic =
        FAULT_RECORD_MAGIC;

    stop();
}