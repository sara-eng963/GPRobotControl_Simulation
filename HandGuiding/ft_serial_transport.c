#include "ft_serial_transport.h"
#include <limits.h>
#include <math.h>
#include <string.h>

_Static_assert(ATOMIC_INT_LOCK_FREE == 2,
               "ISR ring requires lock-free machine-word atomics");
_Static_assert(FT_SERIAL_RING_SIZE > FT_SERIAL_ISR_MAX_BYTES,
               "ISR batch must fit into ring");
_Static_assert(FT_SERIAL_RING_SIZE <= UINT32_MAX / 2U,
               "cursor arithmetic requires a bounded ring");

static bool valid_age(uint32_t age) { return age > 0U && age < INT32_MAX; }
static bool elapsed_exceeds(uint32_t now, uint32_t then, uint32_t limit)
{
    return (uint32_t)(now - then) > limit;
}
static uint32_t sat_inc(uint32_t x) { return x == UINT32_MAX ? x : x + 1U; }
static uint32_t sat_add(uint32_t x, uint32_t d)
{
    return d > UINT32_MAX - x ? UINT32_MAX : x + d;
}

bool ft_serial_init(FtSerialPort *p, const FtSerialProtocol *protocol,
                    uint32_t link_timeout, uint32_t fragment_timeout,
                    uint32_t sample_max_age)
{
    if (p == NULL || !valid_age(link_timeout) ||
        !valid_age(fragment_timeout) || !valid_age(sample_max_age)) return false;
    memset(p, 0, sizeof(*p));
    atomic_init(&p->write_cursor, 0U);
    atomic_init(&p->read_cursor, 0U);
    atomic_init(&p->overflow_events, 0U);
    atomic_init(&p->uart_errors, 0U);
    p->link_timeout_us = link_timeout;
    p->fragment_timeout_us = fragment_timeout;
    p->max_sample_age_us = sample_max_age;
    if (protocol != NULL) p->protocol = *protocol;
    /* A claimed commission without a real parser cannot authorize samples. */
    p->protocol.commissioned = p->protocol.commissioned &&
        p->protocol.reset != NULL && p->protocol.feed != NULL &&
        p->protocol.partial != NULL;
    p->status = p->protocol.commissioned ? FT_SERIAL_STATUS_DISCONNECTED
                                         : FT_SERIAL_STATUS_UNCOMMISSIONED;
    if (p->protocol.reset != NULL) p->protocol.reset(p->protocol.ctx);
    return true;
}

bool ft_serial_isr_receive(FtSerialPort *p, const uint8_t *data,
                           size_t len, uint32_t isr_us)
{
    if (p == NULL || (data == NULL && len != 0U)) return false;
    if (len == 0U) return true;
    if (len > FT_SERIAL_ISR_MAX_BYTES) {
        (void)atomic_fetch_add_explicit(&p->overflow_events, 1U,
                                        memory_order_relaxed);
        return false;
    }
    const uint32_t head = atomic_load_explicit(&p->write_cursor,
                                                memory_order_relaxed);
    const uint32_t tail = atomic_load_explicit(&p->read_cursor,
                                                memory_order_acquire);
    if ((uint32_t)(head - tail) > FT_SERIAL_RING_SIZE ||
        len > FT_SERIAL_RING_SIZE - (uint32_t)(head - tail)) {
        (void)atomic_fetch_add_explicit(&p->overflow_events, 1U,
                                        memory_order_relaxed);
        return false;
    }
    for (size_t i = 0U; i < len; ++i) {
        FtSerialRxByte *slot = &p->ring[(head + (uint32_t)i) % FT_SERIAL_RING_SIZE];
        slot->octet = data[i];
        slot->observed_us = isr_us;
    }
    atomic_store_explicit(&p->write_cursor, head + (uint32_t)len,
                          memory_order_release);
    return true;
}

void ft_serial_isr_uart_error(FtSerialPort *p)
{
    if (p != NULL)
        (void)atomic_fetch_add_explicit(&p->uart_errors, 1U,
                                        memory_order_relaxed);
}

static void invalidate(FtSerialPort *p)
{
    p->last_sample.valid = false;
    p->partial_started = false;
    if (p->protocol.reset != NULL) p->protocol.reset(p->protocol.ctx);
}

static bool consume_errors(FtSerialPort *p)
{
    const uint32_t o = atomic_load_explicit(&p->overflow_events,
                                            memory_order_acquire);
    const uint32_t e = atomic_load_explicit(&p->uart_errors,
                                            memory_order_acquire);
    if (o == p->overflow_seen && e == p->uart_seen) return false;
    const uint32_t delta_o = o - p->overflow_seen;
    const uint32_t delta_e = e - p->uart_seen;
    p->overflow_seen = o;
    p->uart_seen = e;
    p->diagnostics.dma_overflow_events =
        sat_add(p->diagnostics.dma_overflow_events, delta_o);
    p->diagnostics.uart_error_events =
        sat_add(p->diagnostics.uart_error_events, delta_e);
    p->diagnostics.incident_latched = true;
    /* Discard all pre-fault bytes. Post-fault bytes may arrive concurrently
     * after this load and will remain queued for resynchronization. */
    atomic_store_explicit(&p->read_cursor,
        atomic_load_explicit(&p->write_cursor, memory_order_acquire),
        memory_order_release);
    invalidate(p);
    p->status = p->protocol.commissioned ? FT_SERIAL_STATUS_FAULT
                                          : FT_SERIAL_STATUS_UNCOMMISSIONED;
    return true;
}

static void accept_decoded(FtSerialPort *p, const FtSerialDecoded *decoded,
                           uint32_t observed_us, uint32_t now_us)
{
    if (elapsed_exceeds(now_us, observed_us, p->max_sample_age_us)) {
        p->diagnostics.stale_frames = sat_inc(p->diagnostics.stale_frames);
        p->last_sample.valid = false;
        p->status = FT_SERIAL_STATUS_FAULT;
        p->diagnostics.incident_latched = true;
        return;
    }
    for (size_t i = 0U; i < 3U; ++i)
        if (!isfinite(decoded->force_N[i]) || !isfinite(decoded->torque_Nm[i])) {
            p->diagnostics.malformed_frames = sat_inc(p->diagnostics.malformed_frames);
            p->diagnostics.incident_latched = true;
            p->last_sample.valid = false;
            p->status = FT_SERIAL_STATUS_FAULT;
            return;
        }
    if (p->have_sequence) {
        const uint32_t delta = decoded->sequence - p->last_sequence;
        if (delta == 0U || delta >= UINT32_C(0x80000000)) {
            p->diagnostics.duplicate_or_old_sequences =
                sat_inc(p->diagnostics.duplicate_or_old_sequences);
            p->diagnostics.incident_latched = true;
            p->last_sample.valid = false;
            p->status = FT_SERIAL_STATUS_FAULT;
            return;
        }
    }
    p->have_sequence = true;
    p->last_sequence = decoded->sequence;
    for (size_t i = 0U; i < 3U; ++i) {
        p->last_sample.force_N[i] = decoded->force_N[i];
        p->last_sample.torque_Nm[i] = decoded->torque_Nm[i];
    }
    p->last_sample.received_time_us = observed_us;
    p->last_sample.valid = true;
    p->diagnostics.last_sequence = decoded->sequence;
    p->diagnostics.last_rx_us = observed_us;
    p->diagnostics.frames_accepted = sat_inc(p->diagnostics.frames_accepted);
    p->status = FT_SERIAL_STATUS_STREAMING;
}

size_t ft_serial_poll(FtSerialPort *p, uint32_t now_us, size_t budget)
{
    if (p == NULL) return 0U;
    if (budget > FT_SERIAL_POLL_MAX_BYTES) budget = FT_SERIAL_POLL_MAX_BYTES;
    if (consume_errors(p)) return 0U;
    size_t used = 0U;
    while (used < budget) {
        const uint32_t tail = atomic_load_explicit(&p->read_cursor,
                                                   memory_order_relaxed);
        const uint32_t head = atomic_load_explicit(&p->write_cursor,
                                                   memory_order_acquire);
        if (tail == head) break;
        const uint32_t depth = head - tail;
        if (depth > p->diagnostics.max_ring_depth)
            p->diagnostics.max_ring_depth = depth;
        const FtSerialRxByte slot = p->ring[tail % FT_SERIAL_RING_SIZE];
        atomic_store_explicit(&p->read_cursor, tail + 1U, memory_order_release);
        ++used;
        p->diagnostics.bytes_processed = sat_inc(p->diagnostics.bytes_processed);
        /* An old physical packet must not become fresh just because the task
         * drained it in this cycle. ISR observation timestamps are retained. */
        if (p->seen_byte &&
            elapsed_exceeds(slot.observed_us, p->last_byte_us,
                            p->fragment_timeout_us) &&
            p->protocol.commissioned && p->protocol.partial(p->protocol.ctx)) {
            p->diagnostics.partial_timeouts = sat_inc(p->diagnostics.partial_timeouts);
            p->diagnostics.incident_latched = true;
            invalidate(p);
        }
        p->last_byte_us = slot.observed_us;
        p->seen_byte = true;
        p->diagnostics.ever_received = true;
        if (!p->protocol.commissioned) continue;
        FtSerialDecoded decoded = {0};
        const FtSerialDecodeResult result =
            p->protocol.feed(p->protocol.ctx, slot.octet, &decoded);
        if (result == FT_SERIAL_FRAME) {
            accept_decoded(p, &decoded, slot.observed_us, now_us);
        } else if (result == FT_SERIAL_CORRUPT || result != FT_SERIAL_MORE) {
            p->diagnostics.malformed_frames = sat_inc(p->diagnostics.malformed_frames);
            p->diagnostics.incident_latched = true;
            p->last_sample.valid = false;
            p->status = FT_SERIAL_STATUS_FAULT;
            p->protocol.reset(p->protocol.ctx);
        }
    }
    if (p->protocol.commissioned) {
        if (p->seen_byte && elapsed_exceeds(now_us, p->last_byte_us,
                                             p->link_timeout_us)) {
            if (!p->link_timed_out)
                p->diagnostics.link_timeouts = sat_inc(p->diagnostics.link_timeouts);
            p->link_timed_out = true;
            invalidate(p);
            p->status = FT_SERIAL_STATUS_DISCONNECTED;
        } else if (p->seen_byte) {
            p->link_timed_out = false;
            if (p->protocol.partial(p->protocol.ctx) &&
                elapsed_exceeds(now_us, p->last_byte_us, p->fragment_timeout_us)) {
                p->diagnostics.partial_timeouts = sat_inc(p->diagnostics.partial_timeouts);
                p->diagnostics.incident_latched = true;
                invalidate(p);
                p->status = FT_SERIAL_STATUS_FAULT;
            } else if (p->status == FT_SERIAL_STATUS_DISCONNECTED) {
                p->status = FT_SERIAL_STATUS_WAITING;
            }
        }
        if (p->last_sample.valid &&
            elapsed_exceeds(now_us, p->last_sample.received_time_us,
                            p->max_sample_age_us)) {
            p->last_sample.valid = false;
            p->status = FT_SERIAL_STATUS_FAULT;
            p->diagnostics.stale_frames = sat_inc(p->diagnostics.stale_frames);
            p->diagnostics.incident_latched = true;
        }
    }
    /* ISR could report an error while this task was processing bytes. */
    (void)consume_errors(p);
    return used;
}

bool ft_serial_latest(FtSerialPort *p, uint32_t now_us, FtSensorSample *out)
{
    if (out == NULL) return false;
    memset(out, 0, sizeof(*out));
    if (p == NULL || !p->protocol.commissioned) return false;
    (void)consume_errors(p);
    if (!p->last_sample.valid ||
        elapsed_exceeds(now_us, p->last_sample.received_time_us,
                        p->max_sample_age_us)) {
        if (p->last_sample.valid) {
            p->last_sample.valid = false;
            p->diagnostics.stale_frames = sat_inc(p->diagnostics.stale_frames);
            p->diagnostics.incident_latched = true;
            p->status = FT_SERIAL_STATUS_FAULT;
        }
        return false;
    }
    *out = p->last_sample;
    return true;
}

const FtSerialDiagnostics *ft_serial_diagnostics(const FtSerialPort *port)
{
    return port == NULL ? NULL : &port->diagnostics;
}

void ft_serial_sampler_init(FtSerialDryRunSampler *sampler, FtSerialPort *port)
{
    if (sampler != NULL) {
        sampler->port = port;
        sampler->now_us = 0U;
        sampler->updated = false;
    }
}

size_t ft_serial_sampler_update(FtSerialDryRunSampler *sampler,
                                uint32_t now_us, size_t byte_budget)
{
    if (sampler == NULL || sampler->port == NULL) return 0U;
    sampler->updated = true;
    sampler->now_us = now_us;
    return ft_serial_poll(sampler->port, now_us, byte_budget);
}

bool ft_serial_sampler_read(void *context, FtSensorSample *out)
{
    FtSerialDryRunSampler *sampler = context;
    if (sampler == NULL || !sampler->updated || sampler->port == NULL)
        return false;
    return ft_serial_latest(sampler->port, sampler->now_us, out);
}
