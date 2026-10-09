#include <stddef.h>

/*
 * Test 7.5 links as a freestanding Cortex-M7 image with -nostdlib.
 * validated_stream_buffer.c uses memset(), so provide the tiny C runtime
 * primitive needed by this firmware-only Renode test.
 */
void *memset(void *destination, int value, size_t length)
{
    unsigned char *bytes = (unsigned char *)destination;
    const unsigned char fill = (unsigned char)value;

    for (size_t i = 0U; i < length; ++i)
    {
        bytes[i] = fill;
    }

    return destination;
}

void *memcpy(void *destination, const void *source, size_t length)
{
    unsigned char *dst = (unsigned char *)destination;
    const unsigned char *src = (const unsigned char *)source;

    for (size_t i = 0; i < length; ++i)
        dst[i] = src[i];

    return destination;
}
