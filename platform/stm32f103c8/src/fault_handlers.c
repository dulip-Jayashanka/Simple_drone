#include "fault_handlers.h"

#include "motor_outputs.h"
#include  <stddef.h>
#include <stdint.h>

/*
 * RCC clock interrupt register.
 *
 * RCC base = 0x40021000
 * RCC_CIR  = RCC base + 0x08
 */
#define RCC_CIR_ADDRESS       (0x40021008UL)

#define RCC_CIR_CSSF          (1UL << 7)
#define RCC_CIR_CSSC          (1UL << 23)

/*
 * Cortex-M3 System Control Block fault registers.
 */
#define SCB_SHCSR_ADDRESS     (0xE000ED24UL)
#define SCB_CFSR_ADDRESS      (0xE000ED28UL)
#define SCB_HFSR_ADDRESS      (0xE000ED2CUL)
#define SCB_DFSR_ADDRESS      (0xE000ED30UL)
#define SCB_MMFAR_ADDRESS     (0xE000ED34UL)
#define SCB_BFAR_ADDRESS      (0xE000ED38UL)
#define SCB_AFSR_ADDRESS      (0xE000ED3CUL)

#define REG32(address) (*(volatile uint32_t *)(address))

volatile fault_record_t g_fault_record;

static void disable_maskable_interrupts(void)
{
    /*
     * Prevent another normal interrupt from interrupting fault handling.
     * NMI and HardFault remain possible because PRIMASK cannot disable them.
     */
    __asm volatile ("cpsid i" ::: "memory");
}

static uint32_t read_active_exception(void)
{
    uint32_t ipsr;

    /*
     * IPSR contains the currently active exception number.
     */
    __asm volatile (
        "mrs %0, ipsr"
        : "=r" (ipsr)
    );

    return (ipsr & 0x1FFUL);
}

static void clear_saved_stack_frame(void)
{
    g_fault_record.stack.r0 = 0UL;
    g_fault_record.stack.r1 = 0UL;
    g_fault_record.stack.r2 = 0UL;
    g_fault_record.stack.r3 = 0UL;
    g_fault_record.stack.r12 = 0UL;
    g_fault_record.stack.lr = 0UL;
    g_fault_record.stack.pc = 0UL;
    g_fault_record.stack.xpsr = 0UL;
}

static void capture_system_fault_registers(void)
{
    g_fault_record.hfsr = REG32(SCB_HFSR_ADDRESS);
    g_fault_record.cfsr = REG32(SCB_CFSR_ADDRESS);
    g_fault_record.shcsr = REG32(SCB_SHCSR_ADDRESS);
    g_fault_record.dfsr = REG32(SCB_DFSR_ADDRESS);
    g_fault_record.mmfar = REG32(SCB_MMFAR_ADDRESS);
    g_fault_record.bfar = REG32(SCB_BFAR_ADDRESS);
    g_fault_record.afsr = REG32(SCB_AFSR_ADDRESS);
}

static void begin_fault_record(fault_type_t type)
{
    /*
     * Zero means the record is currently incomplete.
     */
    g_fault_record.magic = 0UL;

    g_fault_record.type = (uint32_t)type;
    g_fault_record.active_exception = read_active_exception();
    g_fault_record.exc_return = 0UL;

    capture_system_fault_registers();
    clear_saved_stack_frame();
}

static void finish_fault_record(void)
{
    /*
     * Finish all previous memory writes.
     */
    __asm volatile ("dmb" ::: "memory");

    /*
     * Mark the record as complete.
     */
    g_fault_record.magic = FAULT_RECORD_MAGIC;

    __asm volatile ("dsb" ::: "memory");
    __asm volatile ("isb" ::: "memory");
}

static __attribute__((noreturn))
void stop_after_fault(void)
{
    /*
     * Never return to possibly corrupted firmware.
     */
    for (;;)
    {
        __asm volatile ("nop");
    }
}

void fault_record_clear(void)
{
    volatile uint32_t *record_words;
    uint32_t index;
    uint32_t word_count;

    record_words = (volatile uint32_t *)&g_fault_record;

    word_count =
        (uint32_t)(sizeof(g_fault_record) / sizeof(uint32_t));

    for (index = 0UL; index < word_count; index++)
    {
        record_words[index] = 0UL;
    }

    __asm volatile ("dmb" ::: "memory");
}

void Default_Handler(void)
{
    disable_maskable_interrupts();

    /*
     * Safety action must happen before debugging work.
     */
    motor_outputs_force_safe();

    begin_fault_record(FAULT_TYPE_DEFAULT_HANDLER);
    finish_fault_record();

    stop_after_fault();
}

void NMI_Handler(void)
{
    disable_maskable_interrupts();

    motor_outputs_force_safe();

    /*
     * The STM32 Clock Security System generates an NMI if HSE fails.
     *
     * CSSF = Clock Security System interrupt flag
     * CSSC = Write one to clear CSSF
     */
    if ((REG32(RCC_CIR_ADDRESS) & RCC_CIR_CSSF) != 0UL)
    {
        REG32(RCC_CIR_ADDRESS) |= RCC_CIR_CSSC;
    }

    begin_fault_record(FAULT_TYPE_NMI);
    finish_fault_record();

    stop_after_fault();
}

/*
 * HardFault exception entry may use either MSP or PSP.
 *
 * EXC_RETURN bit 2:
 *
 * 0 = stack frame uses MSP
 * 1 = stack frame uses PSP
 *
 * The assembly wrapper passes:
 *
 * r0 = address of exception stack frame
 * r1 = original EXC_RETURN value from LR
 */
__attribute__((naked, noreturn))
void HardFault_Handler(void)
{
    __asm volatile (
        "tst lr, #4          \n"
        "ite eq              \n"
        "mrseq r0, msp       \n"
        "mrsne r0, psp       \n"
        "mov r1, lr          \n"
        "b hard_fault_capture\n"
    );
}

/*
 * The 'used' attribute prevents linker garbage collection because the
 * reference from HardFault_Handler exists inside assembly code.
 */
__attribute__((used, noreturn))
void hard_fault_capture(
    const fault_stack_frame_t *stack_frame,
    uint32_t exc_return)
{
    disable_maskable_interrupts();

    motor_outputs_force_safe();

    begin_fault_record(FAULT_TYPE_HARDFAULT);

    g_fault_record.exc_return = exc_return;

    if (stack_frame != (const fault_stack_frame_t *)0)
    {
        g_fault_record.stack.r0 = stack_frame->r0;
        g_fault_record.stack.r1 = stack_frame->r1;
        g_fault_record.stack.r2 = stack_frame->r2;
        g_fault_record.stack.r3 = stack_frame->r3;
        g_fault_record.stack.r12 = stack_frame->r12;
        g_fault_record.stack.lr = stack_frame->lr;
        g_fault_record.stack.pc = stack_frame->pc;
        g_fault_record.stack.xpsr = stack_frame->xpsr;
    }

    finish_fault_record();

    stop_after_fault();
}