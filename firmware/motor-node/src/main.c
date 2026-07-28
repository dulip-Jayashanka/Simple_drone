#include <stdint.h>

int main(void)
{
    volatile uint32_t counter = 0U;
    volatile uint32_t hansi = 0U;

    while (1) {
        counter++;
        volatile int32_t dulip = 102;
        dulip ++;
        hansi = hansi + counter;
        hansi=hansi +1;
    }

    return 0;
}