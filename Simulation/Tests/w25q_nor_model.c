#include "w25q_nor_model.h"
#include <assert.h>
#include <string.h>
const char *const stage_names[STAGE_COUNT] = {
    "discovery", "header erase", "erase readback", "payload erase", "payload program",
    "payload verify", "header program", "header verify", "marker program",
    "marker verify", "final payload verify"
};
static uint8_t *memory(Model *m) { return m->external_bytes ? m->external_bytes : m->bytes; }
static size_t capacity(const Model *m) { return m->external_bytes ? m->memory_size : IMAGE_BYTES; }
static void persist(Model *m, uint8_t op, uint32_t at, const uint8_t *tx, size_t n)
{
    if (op == 0x21U) {
        assert(at % SECTOR == 0 && n <= SECTOR && at + SECTOR <= capacity(m));
        memset(memory(m)+at,0xFF,n);
    } else {
        assert(op == 0x12U && n <= PAGE);
        for (size_t i=0; i<n; ++i) {
            uint32_t location=(at & ~(PAGE-1U)) | ((at+(uint32_t)i) & (PAGE-1U));
            assert(location < capacity(m));
            memory(m)[location] &= tx[i];
        }
    }
}

static void finish_pending(Model *m)
{
    if (m->pending_op && !m->stuck_wip && m->now >= m->busy_until) {
        persist(m,m->pending_op,m->pending_at,m->pending_data,m->pending_size);
        m->pending_op=0;
        if (!m->keep_wel) m->wel=false;
    }
}

static Stage classify(Model *m, uint8_t op, uint32_t at)
{
    const bool header=(at % SLOT_BYTES) < SECTOR;
    if (op == 0x21U) return header ? HEADER_ERASE : PAYLOAD_ERASE;
    if (op == 0x12U) {
        if (!header) return PAYLOAD_PROGRAM;
        if (at % SLOT_BYTES == m->marker_offset)
            return MARKER_PROGRAM;
        return HEADER_PROGRAM;
    }
    if (!m->updating) return DISCOVERY;
    if (header) {
        if (m->marker_started) return MARKER_VERIFY;
        return m->header_started ? HEADER_VERIFY : ERASE_READBACK;
    }
    return m->marker_started ? FINAL_PAYLOAD_VERIFY : PAYLOAD_VERIFY;
}

bool w25_model_command(void *ctx, uint8_t op, uint32_t at,
                    const uint8_t *tx, size_t txsz, uint8_t *rx, size_t rxsz)
{
    Model *m=ctx;
    ++m->op_calls[op];
    if (!m->powered || m->now < m->read_allowed_us || m->now < m->reset_until_us)
        return false;
    if (op == 0x05U) m->now+=m->status_latency_us;
    finish_pending(m);
    if (op == 0x66U) { m->reset_enabled=true; return true; }
    if (op == 0x99U) {
        if (m->reset_enabled) {
            if (m->pending_op) {
                size_t n=m->reset_partial < m->pending_size ? m->reset_partial : m->pending_size;
                persist(m,m->pending_op,m->pending_at,m->pending_data,n);
            }
            m->pending_op=0; m->wel=false; m->stuck_wip=false;
            m->reset_until_us=m->now+30;
        }
        m->reset_enabled=false;
        return true;
    }
    m->reset_enabled=false;
    if (op == 0x05U) {
        assert(rx && rxsz == 1 && txsz == 0);
        *rx=((m->pending_op || m->stuck_wip) ? 1U : 0U) | (m->wel ? 2U : 0U);
        return true;
    }
    if (m->pending_op || m->stuck_wip) { ++m->busy_rejections; return false; }
    if (op == 0x9FU) {
        assert(rx && rxsz == 3 && txsz == 0);
        rx[0]=(uint8_t)(m->jedec_id >> 16);
        rx[1]=(uint8_t)(m->jedec_id >> 8);
        rx[2]=(uint8_t)m->jedec_id;
        return true;
    }
    if (op == 0x06U) {
        if (!m->reject_wren && m->now >= m->write_allowed_us) m->wel=true;
        return true;
    }
    if (op == 0x04U) { m->wel=false; return true; }
    if (op != 0x13U && op != 0x12U && op != 0x21U) {
        ++m->unknown_commands; return false;
    }
    size_t n=op == 0x21U ? SECTOR : (txsz ? txsz : rxsz);
    if (at >= capacity(m) || (op == 0x13U && (uint64_t)at+n > capacity(m))) return false;
    if (op == 0x21U) at &= ~(SECTOR-1U);
    Stage stage=classify(m,op,at);
    assert(m->event_count < MAX_EVENTS);
    unsigned number=++m->event_count;
    m->events[number-1]=(Event){op,at,n,stage};
    bool inject=m->fault != NO_FAULT && number == m->fail_event;
    if (inject) m->fired=true;
    if (inject && (m->fault == CUT_BEFORE || m->fault == READ_ERROR)) {
        if (m->fault == CUT_BEFORE) m->powered=false;
        return false;
    }
    if (op == 0x13U) {
        assert(rx && rxsz && !txsz);
        size_t amount=inject && m->fault == CUT_DURING ? m->partial : rxsz;
        if (amount > rxsz) amount=rxsz;
        memcpy(rx,memory(m)+at,amount);
        if (inject && (m->fault == CUT_DURING || m->fault == CUT_AFTER)) {
            m->powered=false;
            return false;
        }
        return true;
    }
    if (!m->wel || m->now < m->write_allowed_us) return true;
    /* Every mutation must target the inactive slot, regardless of reload use. */
    if (m->enforce_target)
        assert(at / SLOT_BYTES == m->target_base / SLOT_BYTES);
    if (op == 0x21U) { ++m->erase_count; m->updating=true; }
    if (stage == HEADER_PROGRAM) m->header_started=true;
    if (stage == MARKER_PROGRAM) m->marker_started=true;
    bool protected_range=m->protect_end > m->protect_begin &&
        at < m->protect_end && (uint64_t)at+n > m->protect_begin;
    if ((inject && m->fault == IGNORE_WRITE) || protected_range) {
        if (m->ignored_clears_wel) m->wel=false;
        return true;
    }
    if (op == 0x12U) { assert(tx && txsz && txsz <= PAGE && !rxsz); }
    if (inject && (m->fault == CUT_DURING || m->fault == CUT_AFTER)) {
        size_t amount=m->fault == CUT_AFTER ? n : m->partial;
        if (amount > n) amount=n;
        persist(m,op,at,tx,amount);
        m->powered=false;
        /* The command was accepted, but power is lost during internal WIP.
         * Subsequent status polls fail; reboot cancels volatile pending state. */
        return true;
    }
    if (inject && m->fault == TRUNCATE_PROGRAM) {
        assert(op == 0x12U);
        n=m->partial < n ? m->partial : n;
    }
    if (inject && m->fault == SILENT_CORRUPTION) {
        persist(m,op,at,tx,n);
        memory(m)[at] ^= 0x01U;
        m->wel=false;
        return true;
    }
    if (inject && m->fault == CORRUPT_PAYLOAD_AT_MARKER) {
        assert(stage == MARKER_PROGRAM);
        memory(m)[m->target_base+SECTOR+64] ^= 0x80U;
    }
    if (inject && m->fault == CORRUPT_HEADER_AT_MARKER) {
        assert(stage == MARKER_PROGRAM);
        memory(m)[m->target_base+m->late_corrupt_header_offset] ^= 0x80U;
    }
    m->pending_op=op;
    m->pending_at=at;
    m->pending_size=n;
    if (op == 0x12U) memcpy(m->pending_data,tx,n);
    m->busy_until=m->now+(op == 0x21U ? m->erase_latency_us : m->program_latency_us);
    return true;
}


uint64_t w25_model_time(void *ctx) {
    Model *m=ctx;
    uint64_t now=m->now;
    m->now+=m->clock_step_us;
    return now;
}
void w25_model_idle(void *ctx) { ((Model *)ctx)->now+=100; }
void w25_model_power_cycle(Model *m) {
    m->powered=true; m->wel=false; m->pending_op=0;
    m->now=m->busy_until=m->reset_until_us=0; m->reset_enabled=false;
    m->stuck_wip=false;
    m->read_allowed_us=20; m->write_allowed_us=5000;
    m->event_count=m->erase_count=0;
    m->updating=m->header_started=m->marker_started=false;
    m->fault=NO_FAULT; m->fired=false;
    memset(m->op_calls,0,sizeof(m->op_calls)); m->unknown_commands=0;
}
void w25_model_init(Model *m, uint8_t *external_bytes, size_t memory_size) {
    memset(m,0,sizeof(*m));
    m->external_bytes=external_bytes; m->memory_size=memory_size;
    m->marker_offset=UINT32_MAX; m->jedec_id=0xEF4020;
    m->program_latency_us=700; m->erase_latency_us=50000;
    memset(memory(m),0xFF,capacity(m));
    w25_model_power_cycle(m);
}
