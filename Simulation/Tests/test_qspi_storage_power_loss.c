/* Deterministic host fault injection for the existing version-2 A/B format.
 * Literal opcodes and NOR page wrapping are independent of driver constants.
 * Cuts model before/partial/complete persistence, not physical brownout timing.
 */
#include "../Renode/Storage/qspi_nor_validated_storage.h"
#include "../Renode/Storage/w25_artifact_crc.h"
#include "w25q_nor_model.h"
#include <assert.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#define CAPACITY 512U
#define COUNT 300U
#define MARKER UINT32_C(0x434F4D4D)

/* Independent fixture description of the unchanged persistent format. */
typedef struct {
    uint32_t magic, version, header_size, sample_size;
    uint32_t capacity_samples, sample_count, generation;
    ValidatedTrajectory metadata;
    uint32_t header_crc, commit_marker;
} HeaderFixture;

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

static void boot(QspiNorValidatedStorage *s)
{
    /* Retain only device bytes across reboot; reset MCU/flash volatile state. */
    w25_model_power_cycle(&model);
    memset(s,0xA5,sizeof(*s));
    model.enforce_target=true;
    model.marker_offset=offsetof(HeaderFixture,commit_marker);
    model.late_corrupt_header_offset=offsetof(HeaderFixture,metadata);
    model.jedec_id=0xEF4020;
    model.program_latency_us=700; model.erase_latency_us=50000;
    W25Q512JVBus bus={w25_model_command,w25_model_time,w25_model_idle,&model};
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
    /* Restore fixture configuration as well as bytes. Ordinary boot preserves
     * configured protection, matching nonvolatile protection across resets. */
    model.protect_begin=model.protect_end=0;
    model.ignored_clears_wel=model.keep_wel=model.reject_wren=false;
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

static void test_degraded_recovery_reporting(void)
{
    uint8_t snapshot[IMAGE_BYTES];
    QspiNorValidatedStorage s;
    (void)fixture(snapshot,1,true); /* A=program 1, B=program 2 (newer) */
    for (unsigned failed_slot=0; failed_slot<2; ++failed_slot) {
        for (unsigned payload=0; payload<2; ++payload) {
            restore(&s,snapshot,1);
            assert(qspi_nor_validated_storage_load_committed(&s));
            unsigned failure=0;
            uint32_t address=failed_slot*SLOT_BYTES+(payload ? SECTOR : 0U);
            for (unsigned i=0; i<model.event_count; ++i)
                if (model.events[i].op == 0x13 && model.events[i].at == address) {
                    failure=i+1; break;
                }
            assert(failure);
            model.event_count=0; model.fault=READ_ERROR; model.fail_event=failure;
            assert(qspi_nor_validated_storage_load_committed(&s));
            assert(s.committed && s.readback_verified && model.fired);
            assert(s.scan_io_error_mask == (1U << failed_slot));
            assert(s.metadata.program_id == (failed_slot ? 1U : 2U));
            PvExecutionSample v, expected=sample(0,failed_slot ? 1U : 2U);
            assert(qspi_nor_validated_storage_read_sample(0,&v,&s));
            assert(memcmp(&v,&expected,sizeof(v)) == 0);
            /* The same failure must block updating even after degraded load. */
            model.event_count=0; model.fired=false;
            assert(!qspi_nor_validated_storage_begin(&s));
            assert(model.fired && model.erase_count == 0);
            assert(memcmp(model.bytes,snapshot,IMAGE_BYTES) == 0);
            model.fault=NO_FAULT; model.event_count=0;
            assert(qspi_nor_validated_storage_load_committed(&s));
            assert(s.scan_io_error_mask == 0 && s.metadata.program_id == 2);
        }
    }
    puts("[PASS] Degraded A/B header/payload I/O recovery reports mask; updates remain blocked");
}

static void test_protected_storage_commits(void)
{
    uint8_t snapshot[IMAGE_BYTES];
    QspiNorValidatedStorage s;
    uint32_t old=fixture(snapshot,0,true);
    Event trace[MAX_EVENTS];
    restore(&s,snapshot,0);
    assert(publish(&s,4));
    unsigned events=model.event_count;
    memcpy(trace,model.events,events*sizeof(*trace));
    for (unsigned clears=0; clears<2; ++clears) {
        for (unsigned region=0; region<2; ++region) {
            restore(&s,snapshot,0);
            model.protect_begin=SLOT_BYTES+(region ? SECTOR : 0U);
            model.protect_end=model.protect_begin+SECTOR;
            model.ignored_clears_wel=clears != 0;
            assert(!publish(&s,4));
            assert(s.metadata.program_id != 4);
            assert(memcmp(model.bytes,snapshot,SLOT_BYTES) == 0);
            assert(model.protect_end > model.protect_begin);
            assert(model.op_calls[0x98] == 0 && model.unknown_commands == 0);
            boot(&s); inspect(&s,old);
        }
        /* Activate protection specifically at marker write (after erase and
         * header verification). No driver change may clear this protection. */
        restore(&s,snapshot,0);
        model.ignored_clears_wel=clears != 0;
        for (unsigned i=0; i<events; ++i) {
            if (trace[i].stage != MARKER_PROGRAM) continue;
            model.fault=IGNORE_WRITE; model.fail_event=i+1;
            assert(!publish(&s,4) && model.fired);
            assert(s.metadata.program_id != 4 && model.op_calls[0x98] == 0);
            boot(&s); inspect(&s,old);
            break;
        }
    }
    puts("[PASS] Protected header/payload and ignored marker cannot publish a committed slot");
}

int main(void)
{
    assert(sizeof(HeaderFixture) <= SECTOR);
    assert(~crc32(UINT32_MAX,"123456789",9) == UINT32_C(0xCBF43926));
    test_power_cuts();
    test_integrity_faults();
    test_discovery_and_ambiguous_commit();
    test_degraded_recovery_reporting();
    test_protected_storage_commits();
    puts("ALL A/B STORAGE POWER-LOSS TESTS PASS");
    return 0;
}
