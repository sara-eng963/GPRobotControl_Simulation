/* Deterministic host fault injection for the existing version-2 A/B format.
 * Literal opcodes and NOR page wrapping are independent of driver constants.
 * Cuts model before/partial/complete persistence, not physical brownout timing.
 */
#include "../Renode/Storage/qspi_nor_validated_storage.h"
#include "../Renode/Storage/w25_artifact_crc.h"
#include <assert.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#define CAPACITY 512U
#define COUNT 300U
#define SECTOR 4096U
#define PAGE 256U
#define SLOT_BYTES 16384U
#define IMAGE_BYTES (2U * SLOT_BYTES)
#define MAX_EVENTS 512U
#define MARKER UINT32_C(0x434F4D4D)

/* Independent fixture description of the unchanged persistent format. */
typedef struct {
    uint32_t magic, version, header_size, sample_size;
    uint32_t capacity_samples, sample_count, generation;
    ValidatedTrajectory metadata;
    uint32_t header_crc, commit_marker;
} HeaderFixture;

typedef enum {
    DISCOVERY, HEADER_ERASE, ERASE_READBACK, PAYLOAD_ERASE, PAYLOAD_PROGRAM,
    PAYLOAD_VERIFY, HEADER_PROGRAM, HEADER_VERIFY, MARKER_PROGRAM,
    MARKER_VERIFY, FINAL_PAYLOAD_VERIFY, STAGE_COUNT
} Stage;
static const char *const stage_names[] = {
    "discovery", "header erase", "erase readback", "payload erase", "payload program",
    "payload verify", "header program", "header verify", "marker program",
    "marker verify", "final payload verify"
};
typedef enum {
    NO_FAULT, CUT_BEFORE, CUT_DURING, CUT_AFTER, IGNORE_WRITE,
    TRUNCATE_PROGRAM, SILENT_CORRUPTION, READ_ERROR,
    CORRUPT_PAYLOAD_AT_MARKER, CORRUPT_HEADER_AT_MARKER
} Fault;
typedef struct { uint8_t op; uint32_t at; size_t size; Stage stage; } Event;
typedef struct {
    uint8_t bytes[IMAGE_BYTES];
    bool powered, wel, updating, header_started, marker_started;
    uint64_t now, busy_until;
    uint8_t pending_op, pending_data[PAGE];
    uint32_t pending_at, target_base;
    size_t pending_size;
    Event events[MAX_EVENTS];
    unsigned event_count, fail_event, erase_count;
    Fault fault;
    size_t partial;
    bool fired;
} Model;
static Model model;
static unsigned power_cases, integrity_cases;
static unsigned stage_cases[STAGE_COUNT];

static uint32_t crc32(uint32_t crc, const void *data, size_t n)
{
    const uint8_t *bytes = data;
    for (size_t i=0; i<n; ++i) {
        crc ^= bytes[i];
        for (unsigned bit=0; bit<8; ++bit)
            crc=(crc >> 1)^((crc & 1U) ? UINT32_C(0xEDB88320) : 0U);
    }
    return crc;
}

static void persist(uint8_t op, uint32_t at, const uint8_t *tx, size_t n)
{
    if (op == 0x21U) {
        assert(at % SECTOR == 0 && n <= SECTOR && at + SECTOR <= IMAGE_BYTES);
        memset(model.bytes+at,0xFF,n);
    } else {
        assert(op == 0x12U && n <= PAGE);
        for (size_t i=0; i<n; ++i) {
            uint32_t location=(at & ~(PAGE-1U)) | ((at+(uint32_t)i) & (PAGE-1U));
            assert(location < IMAGE_BYTES);
            model.bytes[location] &= tx[i];
        }
    }
}

static void finish_pending(void)
{
    if (model.pending_op && model.now >= model.busy_until) {
        persist(model.pending_op,model.pending_at,model.pending_data,model.pending_size);
        model.pending_op=0;
        model.wel=false;
    }
}

static Stage classify(uint8_t op, uint32_t at)
{
    const bool header=(at % SLOT_BYTES) < SECTOR;
    if (op == 0x21U) return header ? HEADER_ERASE : PAYLOAD_ERASE;
    if (op == 0x12U) {
        if (!header) return PAYLOAD_PROGRAM;
        if (at % SLOT_BYTES == offsetof(HeaderFixture,commit_marker))
            return MARKER_PROGRAM;
        return HEADER_PROGRAM;
    }
    if (!model.updating) return DISCOVERY;
    if (header) {
        if (model.marker_started) return MARKER_VERIFY;
        return model.header_started ? HEADER_VERIFY : ERASE_READBACK;
    }
    return model.marker_started ? FINAL_PAYLOAD_VERIFY : PAYLOAD_VERIFY;
}

static bool command(void *ctx, uint8_t op, uint32_t at,
                    const uint8_t *tx, size_t txsz, uint8_t *rx, size_t rxsz)
{
    (void)ctx;
    if (!model.powered) return false;
    finish_pending();
    if (op == 0x05U) {
        assert(rx && rxsz == 1 && txsz == 0);
        *rx=(model.pending_op ? 1U : 0U) | (model.wel ? 2U : 0U);
        return true;
    }
    if (model.pending_op) return false;
    if (op == 0x9FU) {
        assert(rx && rxsz == 3 && txsz == 0);
        rx[0]=0xEF; rx[1]=0x40; rx[2]=0x20;
        return true;
    }
    if (op == 0x06U) { model.wel=true; return true; }
    assert(op == 0x13U || op == 0x12U || op == 0x21U);
    size_t n=op == 0x21U ? SECTOR : (txsz ? txsz : rxsz);
    assert(at < IMAGE_BYTES && (uint64_t)at+n <= IMAGE_BYTES);
    Stage stage=classify(op,at);
    assert(model.event_count < MAX_EVENTS);
    unsigned number=++model.event_count;
    model.events[number-1]=(Event){op,at,n,stage};
    bool inject=model.fault != NO_FAULT && number == model.fail_event;
    if (inject) model.fired=true;
    if (inject && (model.fault == CUT_BEFORE || model.fault == READ_ERROR)) {
        if (model.fault == CUT_BEFORE) model.powered=false;
        return false;
    }
    if (op == 0x13U) {
        assert(rx && rxsz && !txsz);
        size_t amount=inject && model.fault == CUT_DURING ? model.partial : rxsz;
        if (amount > rxsz) amount=rxsz;
        memcpy(rx,model.bytes+at,amount);
        if (inject && (model.fault == CUT_DURING || model.fault == CUT_AFTER)) {
            model.powered=false;
            return false;
        }
        return true;
    }
    assert(model.wel);
    /* Every mutation must target the inactive slot, regardless of reload use. */
    assert(at / SLOT_BYTES == model.target_base / SLOT_BYTES);
    if (op == 0x21U) { ++model.erase_count; model.updating=true; }
    if (stage == HEADER_PROGRAM) model.header_started=true;
    if (stage == MARKER_PROGRAM) model.marker_started=true;
    if (inject && model.fault == IGNORE_WRITE) return true;
    if (op == 0x12U) { assert(tx && txsz && txsz <= PAGE && !rxsz); }
    if (inject && (model.fault == CUT_DURING || model.fault == CUT_AFTER)) {
        size_t amount=model.fault == CUT_AFTER ? n : model.partial;
        if (amount > n) amount=n;
        persist(op,at,tx,amount);
        model.powered=false;
        /* The command was accepted, but power is lost during internal WIP.
         * Subsequent status polls fail; reboot cancels volatile pending state. */
        return true;
    }
    if (inject && model.fault == TRUNCATE_PROGRAM) {
        assert(op == 0x12U);
        n=model.partial < n ? model.partial : n;
    }
    if (inject && model.fault == SILENT_CORRUPTION) {
        persist(op,at,tx,n);
        model.bytes[at] ^= 0x01U;
        model.wel=false;
        return true;
    }
    if (inject && model.fault == CORRUPT_PAYLOAD_AT_MARKER) {
        assert(stage == MARKER_PROGRAM);
        model.bytes[model.target_base+SECTOR+64] ^= 0x80U;
    }
    if (inject && model.fault == CORRUPT_HEADER_AT_MARKER) {
        assert(stage == MARKER_PROGRAM);
        model.bytes[model.target_base+offsetof(HeaderFixture,metadata)] ^= 0x80U;
    }
    model.pending_op=op;
    model.pending_at=at;
    model.pending_size=n;
    if (op == 0x12U) memcpy(model.pending_data,tx,n);
    model.busy_until=model.now+(op == 0x21U ? 50000U : 700U);
    return true;
}

static uint64_t clock_us(void *ctx) { (void)ctx; return model.now; }
static void idle(void *ctx) { (void)ctx; model.now+=100; }
static void boot(QspiNorValidatedStorage *s)
{
    /* Retain only device bytes across reboot; reset MCU/flash volatile state. */
    model.powered=true;
    model.wel=false;
    model.pending_op=0;
    model.now=model.busy_until=0;
    model.event_count=model.erase_count=0;
    model.updating=model.header_started=model.marker_started=false;
    model.fault=NO_FAULT;
    model.fired=false;
    memset(s,0xA5,sizeof(*s));
    W25Q512JVBus bus={command,clock_us,idle,NULL};
    assert(qspi_nor_validated_storage_set_bus(&bus));
    assert(qspi_nor_validated_storage_init(s,0,CAPACITY));
    assert(s->slot_bytes == SLOT_BYTES);
}

static PvExecutionSample sample(uint32_t index, uint32_t program)
{
    PvExecutionSample s;
    for (unsigned j=0; j<6; ++j) {
        int32_t magnitude=(int32_t)(program*100000U+index*137U+j*31U);
        s.target_position_units[j]=j % 2U ? -magnitude : magnitude;
    }
    return s;
}

static bool publish(QspiNorValidatedStorage *s, uint32_t program)
{
    if (!qspi_nor_validated_storage_begin(s)) return false;
    uint32_t crc=UINT32_MAX;
    for (uint32_t i=0; i<COUNT; ++i) {
        PvExecutionSample v=sample(i,program);
        crc=crc32(crc,&v,sizeof(v));
        if (!qspi_nor_validated_storage_write_sample(i,&v,s)) return false;
    }
    ValidatedTrajectory m={0};
    m.program_id=program; m.source_revision=program;
    m.sample_count=COUNT; m.sample_period_us=2000;
    m.sample_data_crc=~crc; m.duration_s=(COUNT-1U)*0.002;
    m.segment_count=1; m.segments[0].sample_count=COUNT;
    m.artifact_crc=w25_artifact_crc(&m);
    return qspi_nor_validated_storage_commit(&m,s);
}

static void inspect(QspiNorValidatedStorage *s, uint32_t program)
{
    assert(qspi_nor_validated_storage_load_committed(s));
    assert(s->committed && s->readback_verified && !s->writing);
    assert(s->metadata.program_id == program && s->sample_count == COUNT);
    for (uint32_t i=0; i<COUNT; ++i) {
        PvExecutionSample actual, expected=sample(i,program);
        assert(qspi_nor_validated_storage_read_sample(i,&actual,s));
        for (unsigned j=0; j<6; ++j)
            assert(actual.target_position_units[j] == expected.target_position_units[j]);
    }
    model.event_count=0; /* inspection is not part of update injection */
}

static uint32_t fixture(uint8_t *snapshot, unsigned active, bool both)
{
    QspiNorValidatedStorage s;
    memset(&model,0,sizeof(model));
    memset(model.bytes,0xFF,sizeof(model.bytes));
    boot(&s);
    model.target_base=0;
    assert(publish(&s,1));
    uint32_t program=1;
    if (active || both) {
        model.event_count=0; model.target_base=SLOT_BYTES;
        assert(publish(&s,2)); program=2;
    }
    if (!active && both) {
        model.event_count=0; model.target_base=0;
        assert(publish(&s,3)); program=3;
    }
    if (active && !both) memset(model.bytes,0xFF,SLOT_BYTES);
    assert(s.active_slot == (int)active);
    memcpy(snapshot,model.bytes,IMAGE_BYTES);
    return program;
}

static void restore(QspiNorValidatedStorage *s, const uint8_t *snapshot, unsigned active)
{
    memcpy(model.bytes,snapshot,IMAGE_BYTES);
    boot(s);
    model.target_base=(1U-active)*SLOT_BYTES;
    /* Deliberately do NOT load_committed: begin must discover after reboot. */
    assert(s->active_slot == -1);
}

static void test_power_cuts(void)
{
    uint8_t snapshot[IMAGE_BYTES];
    Event trace[MAX_EVENTS];
    for (unsigned active=0; active<2; ++active) {
        for (unsigned both=0; both<2; ++both) {
            uint32_t old=fixture(snapshot,active,both != 0);
            QspiNorValidatedStorage s;
            restore(&s,snapshot,active);
            assert(publish(&s,4));
            unsigned events=model.event_count;
            memcpy(trace,model.events,events*sizeof(*trace));
            for (unsigned event=0; event<events; ++event) {
                /* Every addressed read/program/erase boundary, plus partial
                 * persistence at first/middle/last bytes of each mutation. */
                Fault faults[]={CUT_BEFORE,CUT_DURING,CUT_AFTER};
                for (unsigned f=0; f<3; ++f) {
                    size_t amounts[]={1,trace[event].size/2,trace[event].size-1};
                    unsigned repeats=faults[f] == CUT_DURING && trace[event].op != 0x13U ? 3 : 1;
                    for (unsigned p=0; p<repeats; ++p) {
                        restore(&s,snapshot,active);
                        model.fault=faults[f]; model.fail_event=event+1;
                        model.partial=amounts[p];
                        assert(!publish(&s,4));
                        assert(model.fired && !model.powered);
                        assert(s.metadata.program_id != 4); /* no live publication */
                        PvExecutionSample v;
                        if (s.writing) assert(!qspi_nor_validated_storage_read_sample(0,&v,&s));
                        /* Active bytes are never modified, regardless of cut. */
                        assert(memcmp(model.bytes+active*SLOT_BYTES,
                                      snapshot+active*SLOT_BYTES,SLOT_BYTES) == 0);
                        bool complete_marker=(trace[event].stage == MARKER_PROGRAM && faults[f] == CUT_AFTER)
                            || trace[event].stage == MARKER_VERIFY
                            || trace[event].stage == FINAL_PAYLOAD_VERIFY;
                        boot(&s);
                        inspect(&s,complete_marker ? 4 : old);
                        ++power_cases; ++stage_cases[trace[event].stage];
                    }
                }
            }
            /* Power loss immediately after the API's successful publication. */
            restore(&s,snapshot,active);
            assert(publish(&s,4));
            boot(&s);
            inspect(&s,4);
            ++power_cases;
        }
    }
    printf("[PASS] %u power-cut/reboot cases, sole/both valid slots, A/B alternation\n",power_cases);
    for (unsigned stage=0; stage<STAGE_COUNT; ++stage) {
        assert(stage_cases[stage] > 0);
        printf("  %s: %u injected cuts\n",stage_names[stage],stage_cases[stage]);
    }
}

static void test_integrity_faults(void)
{
    uint8_t snapshot[IMAGE_BYTES];
    Event trace[MAX_EVENTS];
    for (unsigned active=0; active<2; ++active) {
        uint32_t old=fixture(snapshot,active,true);
        QspiNorValidatedStorage s;
        restore(&s,snapshot,active);
        assert(publish(&s,4));
        unsigned events=model.event_count;
        memcpy(trace,model.events,events*sizeof(*trace));
        for (unsigned event=0; event<events; ++event) {
            Fault faults[]={IGNORE_WRITE,TRUNCATE_PROGRAM,SILENT_CORRUPTION,READ_ERROR};
            for (unsigned f=0; f<4; ++f) {
                uint8_t op=trace[event].op;
                if (op == 0x13U && faults[f] != READ_ERROR) continue;
                if (op != 0x13U && faults[f] == READ_ERROR) continue;
                if (op == 0x21U && (faults[f] == TRUNCATE_PROGRAM || faults[f] == SILENT_CORRUPTION)) continue;
                restore(&s,snapshot,active);
                model.fault=faults[f]; model.fail_event=event+1;
                model.partial=trace[event].size/2;
                assert(!publish(&s,4));
                assert(model.fired);
                assert(s.metadata.program_id != 4);
                assert(memcmp(model.bytes+active*SLOT_BYTES,
                              snapshot+active*SLOT_BYTES,SLOT_BYTES) == 0);
                /* Readback failure after the full marker is an ambiguous
                 * acknowledgement, but reboot must recover fully verified data. */
                bool new_valid=trace[event].stage == MARKER_VERIFY ||
                               trace[event].stage == FINAL_PAYLOAD_VERIFY;
                boot(&s); inspect(&s,new_valid ? 4 : old);
                ++integrity_cases;
            }
            if (trace[event].stage == MARKER_PROGRAM) {
                Fault late_faults[]={CORRUPT_PAYLOAD_AT_MARKER,CORRUPT_HEADER_AT_MARKER};
                for (unsigned f=0; f<2; ++f) {
                    restore(&s,snapshot,active);
                    model.fault=late_faults[f]; model.fail_event=event+1;
                    assert(!publish(&s,4)); assert(model.fired);
                    assert(s.metadata.program_id != 4);
                    boot(&s); inspect(&s,old);
                    ++integrity_cases;
                }
            }
        }
    }
    printf("[PASS] %u ignored/truncated/corrupted write and read-error cases\n",integrity_cases);
}

static void test_discovery_and_ambiguous_commit(void)
{
    uint8_t snapshot[IMAGE_BYTES];
    QspiNorValidatedStorage s;
    (void)fixture(snapshot,0,false);
    restore(&s,snapshot,0);
    model.fault=READ_ERROR; model.fail_event=1;
    assert(!qspi_nor_validated_storage_begin(&s));
    assert(model.fired && model.erase_count == 0);
    assert(memcmp(model.bytes,snapshot,IMAGE_BYTES) == 0);
    model.fault=NO_FAULT; model.event_count=0;
    assert(publish(&s,4)); /* same object retries discovery without reboot */
    assert(s.active_slot == 1);
    puts("[PASS] Discovery I/O failure blocks all erases; retry discovers sole valid slot");

    (void)fixture(snapshot,0,true);
    restore(&s,snapshot,0);
    assert(publish(&s,4));
    unsigned marker_read=0;
    for (unsigned i=0; i<model.event_count; ++i)
        if (model.events[i].stage == MARKER_VERIFY) { marker_read=i+1; break; }
    assert(marker_read);
    restore(&s,snapshot,0);
    model.fault=READ_ERROR; model.fail_event=marker_read;
    assert(!publish(&s,4)); assert(model.fired);
    qspi_nor_validated_storage_abort(&s);
    model.fault=NO_FAULT; model.event_count=0;
    model.updating=model.header_started=model.marker_started=false;
    model.target_base=0; /* rediscovery must preserve the fully committed B */
    assert(publish(&s,5));
    assert(s.active_slot == 0 && s.next_generation == 6);
    boot(&s); inspect(&s,5);
    model.bytes[SECTOR+64] ^= 0x40U;
    inspect(&s,4);
    model.bytes[SLOT_BYTES+SECTOR+64] ^= 0x40U;
    assert(!qspi_nor_validated_storage_load_committed(&s));
    PvExecutionSample v;
    assert(!qspi_nor_validated_storage_read_sample(0,&v,&s));
    puts("[PASS] Ambiguous commit/abort rediscovery, consecutive update and corruption fallback");
}

int main(void)
{
    assert(sizeof(HeaderFixture) <= SECTOR);
    assert(~crc32(UINT32_MAX,"123456789",9) == UINT32_C(0xCBF43926));
    test_power_cuts();
    test_integrity_faults();
    test_discovery_and_ambiguous_commit();
    puts("ALL A/B STORAGE POWER-LOSS TESTS PASS");
    return 0;
}
