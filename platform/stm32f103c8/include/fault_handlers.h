#ifndef FAULT_HANDLERS_H
#define FAULT_HANDLERS_H

#include <stdint.h>

/*
 * ASCII value "FAUL".
 *
 * If g_fault_record.magic contains this value, the fault record
 * was completely written.
 */
#define FAULT_RECORD_MAGIC (0x4641554CUL)

typedef enum
{
    FAULT_TYPE_NONE = 0,
    FAULT_TYPE_DEFAULT_HANDLER = 1,
    FAULT_TYPE_NMI = 2,
    FAULT_TYPE_HARDFAULT = 3
} fault_type_t;

/*
 * Registers automatically pushed onto the stack by Cortex-M3
 * when an exception occurs.
 */
typedef struct
{
    uint32_t r0;
    uint32_t r1;
    uint32_t r2;
    uint32_t r3;
    uint32_t r12;
    uint32_t lr;
    uint32_t pc;
    uint32_t xpsr;
} fault_stack_frame_t;

/*
 * Complete record saved when a fault occurs.
 */
typedef struct
{
    uint32_t magic;
    uint32_t type;
    uint32_t active_exception;
    uint32_t exc_return;

    uint32_t hfsr;
    uint32_t cfsr;
    uint32_t shcsr;
    uint32_t dfsr;
    uint32_t mmfar;
    uint32_t bfar;
    uint32_t afsr;

    fault_stack_frame_t stack;
} fault_record_t;

/*
 * Global record visible through GDB and Renode.
 */
extern volatile fault_record_t g_fault_record;

void fault_record_clear(void);

void Default_Handler(void) __attribute__((noreturn));
void NMI_Handler(void) __attribute__((noreturn));
void HardFault_Handler(void) __attribute__((noreturn));

#endif /* FAULT_HANDLERS_H */