#include <stddef.h>

void *memset(void *destination, int value, size_t count)
{
    unsigned char *current = (unsigned char *)destination;
    const unsigned char byte_value = (unsigned char)value;

    while (count > 0U)
    {
        *current = byte_value;
        current++;
        count--;
    }

    return destination;
}