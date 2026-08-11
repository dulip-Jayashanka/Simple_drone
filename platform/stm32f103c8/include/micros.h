#ifndef MICROS_H
#define MICROS_H

#include <stdint.h>

typedef enum
{
    MICROS_OK = 0,
    MICROS_INVALID_CLOCK,
    MICROS_PRESCALER_OUT_OF_RANGE
} micros_status_t;

/* Configure TIM2 as a free-running 32-bit, 1 MHz timestamp counter. */
micros_status_t micros_init(uint32_t tim2_clock_hz);
uint32_t micros(void);
void delay_ms(uint32_t milliseconds);

#endif /* MICROS_H */