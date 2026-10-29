#ifndef P05_MOCK_FT_PROTOCOL_H
#define P05_MOCK_FT_PROTOCOL_H
/* TEST-ONLY wire format, no claim of compatibility with any commercial F/T
 * sensor. Mocks retain their own explicit sequence and CRC checks. */
#include "../ft_serial_transport.h"
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#define PACKET_LEN 34U

typedef struct { uint8_t bytes[PACKET_LEN]; size_t used; } MockDecoder;
static MockDecoder mock;
static uint32_t load_u32(const uint8_t *b)
{
    return (uint32_t)b[0] | ((uint32_t)b[1] << 8U) |
           ((uint32_t)b[2] << 16U) | ((uint32_t)b[3] << 24U);
}
static void store_u32(uint8_t *b, uint32_t x)
{
    for (unsigned k = 0; k < 4U; ++k) b[k] = (uint8_t)(x >> (8U * k));
}
static uint32_t crc32(const uint8_t *p, size_t length)
{
    uint32_t crc = UINT32_MAX;
    for (size_t i = 0; i < length; ++i) {
        crc ^= p[i];
        for (unsigned bit = 0; bit < 8U; ++bit)
            crc = (crc >> 1U) ^ (UINT32_C(0xEDB88320) &
                   (uint32_t)-(int32_t)(crc & 1U));
    }
    return crc ^ UINT32_MAX;
}
static void mock_reset(void *ctx)
{
    MockDecoder *d = ctx;
    d->used = 0U;
}
static bool mock_partial(const void *ctx)
{
    return ((const MockDecoder *)ctx)->used > 0U;
}
static FtSerialDecodeResult mock_feed(void *ctx, uint8_t byte,
                                      FtSerialDecoded *out)
{
    MockDecoder *d = ctx;
    if (d->used == 0U && byte != 0xA5U) return FT_SERIAL_MORE;
    if (d->used == 1U && byte != 0x5AU) {
        d->used = byte == 0xA5U ? 1U : 0U;
        return FT_SERIAL_MORE;
    }
    d->bytes[d->used++] = byte;
    if (d->used != PACKET_LEN) return FT_SERIAL_MORE;
    const uint32_t actual = load_u32(&d->bytes[30]);
    const uint32_t expected = crc32(d->bytes, 30U);
    d->used = 0U;
    if (actual != expected) return FT_SERIAL_CORRUPT;
    out->sequence = load_u32(&d->bytes[2]);
    for (unsigned i = 0; i < 3U; ++i) {
        out->force_N[i] = (double)(int32_t)load_u32(&d->bytes[6U + 4U * i]) / 1000.0;
        out->torque_Nm[i] = (double)(int32_t)load_u32(&d->bytes[18U + 4U * i]) / 1000.0;
    }
    return FT_SERIAL_FRAME;
}
static FtSerialProtocol mock_protocol(void)
{
    FtSerialProtocol p = {.reset=mock_reset,.feed=mock_feed,
                           .partial=mock_partial,.ctx=&mock,.commissioned=true};
    return p;
}
static void packet(uint8_t *data, uint32_t sequence, int32_t force_mN)
{
    memset(data, 0, PACKET_LEN);
    data[0] = 0xA5U; data[1] = 0x5AU;
    store_u32(&data[2], sequence);
    store_u32(&data[6], (uint32_t)force_mN);
    store_u32(&data[30], crc32(data, 30U));
}
#endif
