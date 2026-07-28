#include <stdint.h>

int main(void)
{
    volatile uint32_t counter = 0U;

    

    static volatile uint32_t dulip = 78;

    while (1) {
        counter++;
        dulip++;
        
        
    }

    return 0;
}