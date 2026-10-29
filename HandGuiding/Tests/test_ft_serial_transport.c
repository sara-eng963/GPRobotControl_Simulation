/* P05 host-only serial decoder fixture. NO real vendor packet format is
 * asserted. Test framing: A5 5A + u32 LE sequence + 6 x i32 LE milliscale
 * wrench + CRC32(IEEE reflected) LE. NOT FOR PHYSICAL FIRMWARE. */
#include "../ft_serial_transport.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "p05_mock_ft_protocol.h"

static void push(FtSerialPort *p, const uint8_t *data, size_t len, uint32_t at)
{
    /* Test driver obeys the same <=64-byte ISR callback bound. */
    assert(len <= FT_SERIAL_ISR_MAX_BYTES);
    assert(ft_serial_isr_receive(p, data, len, at));
}
static FtSensorSample read_sample(FtSerialPort *p, uint32_t now)
{
    FtSensorSample s = {0};
    assert(ft_serial_latest(p, now, &s));
    assert(s.valid);
    return s;
}
int main(void)
{
    FtSerialPort port;
    uint8_t pkt[PACKET_LEN];
    FtSensorSample sample = {0};
    assert(!ft_serial_init(&port, NULL, 0U, 1000U, 3000U));
    assert(ft_serial_init(&port, NULL, 5000U, 1500U, 3000U));
    packet(pkt, 1U, 5000);
    push(&port, pkt, PACKET_LEN, 1000U);
    assert(ft_serial_poll(&port, 1100U, 128U) == PACKET_LEN);
    assert(!ft_serial_latest(&port, 1100U, &sample));
    assert(port.status == FT_SERIAL_STATUS_UNCOMMISSIONED);
    puts("P05-SERIAL-01 uncommissioned vendor protocol denied");

    FtSerialProtocol parser = mock_protocol();
    assert(ft_serial_init(&port, &parser, 5000U, 1500U, 3000U));
    packet(pkt, 7U, 5000);
    push(&port, pkt, PACKET_LEN, 2000U);
    assert(ft_serial_poll(&port, 2100U, 4U) == 4U);
    assert(!ft_serial_latest(&port, 2100U, &sample));
    assert(ft_serial_poll(&port, 2100U, 100U) == PACKET_LEN-4U);
    sample = read_sample(&port, 2100U);
    assert(sample.force_N[0] == 5.0 && sample.received_time_us == 2000U);
    assert(port.diagnostics.frames_accepted == 1U);
    puts("P05-SERIAL-02 bounded backlog, monotonic sample, SI representation");

    push(&port, pkt, PACKET_LEN, 2500U); /* duplicate */
    (void)ft_serial_poll(&port, 2510U, 128U);
    assert(!ft_serial_latest(&port, 2510U, &sample));
    assert(port.diagnostics.duplicate_or_old_sequences == 1U);
    packet(pkt, 6U, 1234); /* old */
    push(&port, pkt, PACKET_LEN, 2600U);
    (void)ft_serial_poll(&port, 2610U, 128U);
    assert(port.diagnostics.duplicate_or_old_sequences == 2U);
    packet(pkt, 8U, -1000); /* new */
    push(&port, pkt, PACKET_LEN, 3000U);
    (void)ft_serial_poll(&port, 3100U, 128U);
    assert(read_sample(&port, 3100U).force_N[0] == -1.0);
    puts("P05-SERIAL-03 duplicate/replay denied; recovery requires newer sequence");

    packet(pkt, 9U, 1000);
    pkt[8] ^= 0x80U; /* CRC fault */
    push(&port, pkt, PACKET_LEN, 3500U);
    (void)ft_serial_poll(&port, 3600U, 128U);
    assert(!ft_serial_latest(&port, 3600U, &sample));
    assert(port.diagnostics.malformed_frames == 1U);
    /* Byte deletion, truncation, then idle timeout and clean next header. */
    packet(pkt, 10U, 2000);
    push(&port, pkt, 13U, 3900U);
    (void)ft_serial_poll(&port, 3950U, 128U);
    (void)ft_serial_poll(&port, 5600U, 128U);
    assert(port.diagnostics.partial_timeouts >= 1U);
    packet(pkt, 11U, 2000);
    push(&port, pkt, PACKET_LEN, 5800U);
    (void)ft_serial_poll(&port, 5850U, 128U);
    assert(read_sample(&port, 5850U).force_N[0] == 2.0);
    puts("P05-SERIAL-04 CRC, truncation, resynchronization");

    const uint32_t accepted_before_stale = port.diagnostics.frames_accepted;
    packet(pkt, 12U, 2000);
    push(&port, pkt, PACKET_LEN, 6000U);
    (void)ft_serial_poll(&port, 9501U, 128U); /* old ISR time */
    assert(!ft_serial_latest(&port, 9501U, &sample));
    assert(port.diagnostics.stale_frames > 0U);
    assert(port.diagnostics.frames_accepted == accepted_before_stale);
    packet(pkt, 13U, 2000);
    push(&port, pkt, PACKET_LEN, 9600U);
    (void)ft_serial_poll(&port, 9700U, 128U);
    assert(read_sample(&port, 9700U).valid);
    (void)ft_serial_poll(&port, 12701U, 128U);
    assert(!ft_serial_latest(&port, 12701U, &sample));
    (void)ft_serial_poll(&port, 16000U, 128U);
    assert(port.status == FT_SERIAL_STATUS_DISCONNECTED);
    puts("P05-SERIAL-05 stale queued packets, sample aging, disconnect");

    uint8_t burst[FT_SERIAL_ISR_MAX_BYTES] = {0};
    /* Overflow: queue contents rejected before parsing. */
    for (unsigned k=0; k<FT_SERIAL_RING_SIZE/FT_SERIAL_ISR_MAX_BYTES; ++k)
        push(&port, burst, sizeof(burst), 17000U);
    assert(!ft_serial_isr_receive(&port, burst, 1U, 17000U));
    assert(ft_serial_poll(&port, 17100U, 128U) == 0U);
    assert(!ft_serial_latest(&port, 17100U, &sample));
    assert(port.diagnostics.dma_overflow_events == 1U);
    packet(pkt, 14U, 3000);
    push(&port, pkt, PACKET_LEN, 17500U);
    (void)ft_serial_poll(&port, 17600U, 128U);
    assert(read_sample(&port, 17600U).force_N[0] == 3.0);
    ft_serial_isr_uart_error(&port);
    (void)ft_serial_poll(&port, 17700U, 128U);
    assert(!ft_serial_latest(&port, 17700U, &sample));
    assert(port.diagnostics.uart_error_events == 1U);
    packet(pkt, 15U, 4000);
    push(&port, pkt, PACKET_LEN, 18000U);
    (void)ft_serial_poll(&port, 18100U, 128U);
    assert(read_sample(&port, 18100U).force_N[0] == 4.0);
    assert(port.diagnostics.incident_latched);
    puts("P05-SERIAL-06 DMA overflow, UART fault, bounded fault recovery");
    /* Deliberately delete a byte in a frame. The following packet loses its
     * start byte while completing the corrupted frame; a later full header
     * must recover, rather than accepting shifted payload bytes. */
    FtSerialPort dropped;
    assert(ft_serial_init(&dropped, &parser, 5000U, 1500U, 3000U));
    packet(pkt, 1U, 1500);
    push(&dropped, pkt, 5U, 1000U);
    push(&dropped, pkt + 6U, PACKET_LEN - 6U, 1000U);
    packet(pkt, 2U, 1500);
    push(&dropped, pkt, PACKET_LEN, 1200U);
    (void)ft_serial_poll(&dropped, 1250U, 128U);
    assert(!ft_serial_latest(&dropped, 1250U, &sample));
    assert(dropped.diagnostics.malformed_frames >= 1U);
    packet(pkt, 3U, 2500);
    push(&dropped, pkt, PACKET_LEN, 1400U);
    (void)ft_serial_poll(&dropped, 1450U, 128U);
    assert(read_sample(&dropped, 1450U).force_N[0] == 2.5);
    uint8_t too_large[FT_SERIAL_ISR_MAX_BYTES + 1U] = {0};
    assert(!ft_serial_isr_receive(&dropped, too_large,
                                  sizeof(too_large), 1500U));
    (void)ft_serial_poll(&dropped, 1550U, 128U);
    assert(!ft_serial_latest(&dropped, 1550U, &sample));
    puts("P05-SERIAL-07 deleted byte and oversized ISR callback containment");
    /* Sequence and local timestamp wrap-around are evaluated mod 2^32.
     * Only forward differences smaller than half-range may be accepted. */
    FtSerialPort wrapping;
    assert(ft_serial_init(&wrapping, &parser, 5000U, 1500U, 3000U));
    packet(pkt, UINT32_MAX - 1U, 1000);
    push(&wrapping, pkt, PACKET_LEN, UINT32_MAX - 100U);
    (void)ft_serial_poll(&wrapping, UINT32_MAX - 50U, 128U);
    assert(read_sample(&wrapping, UINT32_MAX - 50U).valid);
    packet(pkt, 0U, 1200);
    push(&wrapping, pkt, PACKET_LEN, 50U);
    (void)ft_serial_poll(&wrapping, 100U, 128U);
    assert(read_sample(&wrapping, 100U).force_N[0] == 1.2);
    packet(pkt, UINT32_MAX, 1200); /* backward replay after wrap */
    push(&wrapping, pkt, PACKET_LEN, 200U);
    (void)ft_serial_poll(&wrapping, 250U, 128U);
    assert(!ft_serial_latest(&wrapping, 250U, &sample));
    puts("P05-SERIAL-08 local timestamp and sequence wrap; replay denied");
    /* 1 Mbaud/8N1 worst-case 2ms occupancy is 200 octets.
     * Drain 204 octets (six valid 34-byte frames) in one 256-byte pass.
     * The producer respects <=64 bytes per callback. */
    FtSerialPort sustained;
    assert(ft_serial_init(&sustained, &parser, 6000U, 3500U, 3500U));
    uint8_t stream[PACKET_LEN * 6U];
    for (uint32_t n = 0; n < 6U; ++n)
        packet(stream + n*PACKET_LEN, n+1U, 1000);
    for (size_t start = 0U; start < sizeof(stream); start += 64U) {
        size_t len = sizeof(stream)-start;
        if (len > 64U) len = 64U;
        push(&sustained, stream+start, len, 20000U);
    }
    assert(ft_serial_poll(&sustained, 20050U, 256U) == sizeof(stream));
    assert(sustained.diagnostics.frames_accepted == 6U);
    assert(read_sample(&sustained, 20050U).valid);
    puts("P05-SERIAL-09 204 bytes per cycle drained with 256-byte bounded service");
    puts("PASS: nine P05 generic serial boundary scenario groups (host-only fixture)");
    return 0;
}
