#include <stddef.h>

void *memcpy(void *destination, const void *source, size_t count)
{
    unsigned char *destination_byte = (unsigned char *)destination;
    const unsigned char *source_byte =
        (const unsigned char *)source;

    while (count > 0U)
    {
        *destination_byte = *source_byte;
        destination_byte++;
        source_byte++;
        count--;
    }

    return destination;
}

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
