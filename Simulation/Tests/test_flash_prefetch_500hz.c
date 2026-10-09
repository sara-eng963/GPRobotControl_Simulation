/*
 * Batch 5 exploratory host timing/integration test.
 * Real QspiNorValidatedStorage + real TrajectoryPrefetch, driven by the
 * existing independent W25 NOR protocol model. The consumer is wall-clock
 * paced at 2ms, but Linux/WSL scheduling is NOT an STM32 timing guarantee.
 *
 * Compile (from repository root):
 *  gcc -std=c11 -O2 -Wall -Wextra -Werror -pthread \
 *    Simulation/Tests/test_flash_prefetch_500hz.c \
 *    Simulation/Storage/trajectory_prefetch.c \
 *    Simulation/Renode/Storage/qspi_nor_validated_storage.c \
 *    Simulation/Renode/Storage/w25q512jv_flash.c \
 *    Simulation/Tests/w25q_nor_model.c -o /tmp/flash_prefetch_500hz
 */
#define _POSIX_C_SOURCE 200809L
#include "../Storage/trajectory_prefetch.h"
#include "../Renode/Storage/qspi_nor_validated_storage.h"
#include "../Renode/Storage/w25_artifact_crc.h"
#include "w25q_nor_model.h"
#include <errno.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <inttypes.h>

#define SAMPLES 1536U
#define PERIOD_US 2000U
#define FLASH_IMAGE_BYTES (256U * 1024U)
#define NO_FAILURE UINT32_MAX

static Model flash_model;
static uint8_t image[FLASH_IMAGE_BYTES];
static QspiNorValidatedStorage storage_writer, storage_reader;
static TrajectoryPrefetch prefetch;
/* Host-only injection. All writes/reads during execution are on the producer
 * thread. Configuration is set before creating and after joining that thread. */
static uint32_t burst_delay_us;
static uint32_t burst_read_calls;
static bool fail_next_transport_read;


typedef struct {
    const char *name;
    uint32_t refill_delay_us;
    uint32_t one_stall_us;
    uint32_t fail_at;
    bool stall_fired;
    uint32_t read_calls;
    uint32_t failed_reads;
    uint64_t read_wall_max_us;
    uint64_t read_wall_sum_us;
    uint32_t read_wall_samples;
} ReadSource;

typedef struct { bool stop; } Worker;

static uint64_t wall_us(void)
{
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) abort();
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

static void sleep_us(uint64_t delay)
{
    struct timespec t = {(time_t)(delay / 1000000ULL), (long)((delay % 1000000ULL) * 1000ULL)};
    while (nanosleep(&t, &t) < 0 && errno == EINTR) { }
}

static void sleep_until(uint64_t target)
{
    struct timespec t = {(time_t)(target / 1000000ULL), (long)((target % 1000000ULL) * 1000ULL)};
    while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL) == EINTR) { }
}

/* The generic W25 model uses a 512-event fault-injection log. For a long
 * fault-free stream this adapter discards only the MODEL's instrumentation
 * history, not flash bytes or any driver's state. Reads are divided into
 * 256-byte physical-style commands instead of a single 6KiB command. */
static bool flash_command(void *ctx, uint8_t op, uint32_t at,
                          const uint8_t *tx, size_t nt, uint8_t *rx, size_t nr)
{
    Model *m = ctx;
    if (op == 0x13U && nr > 256U) {
        for (size_t off = 0; off < nr; off += 256U) {
            size_t n = nr - off < 256U ? nr - off : 256U;
            if (!flash_command(ctx, op, at + (uint32_t)off, NULL, 0, rx + off, n)) return false;
        }
        return true;
    }
    if (op == 0x13U) {
        ++burst_read_calls;
        if (burst_delay_us != 0U) sleep_us(burst_delay_us);
        if (fail_next_transport_read) {
            fail_next_transport_read = false;
            return false; /* Failing actual bus transport, not skipping callback. */
        }
    }
    if (m->event_count >= MAX_EVENTS - 2U) m->event_count = 0U;
    return w25_model_command(ctx, op, at, tx, nt, rx, nr);
}

static PvExecutionSample make_sample(uint32_t i)
{
    PvExecutionSample p;
    for (uint32_t j = 0; j < 6; ++j) {
        int32_t mag = (int32_t)(100000U + i * 37U + j * 101U);
        p.target_position_units[j] = (j & 1U) ? -mag : mag;
    }
    return p;
}

static bool publish_and_load(void)
{
    w25_model_init(&flash_model, image, FLASH_IMAGE_BYTES);
    W25Q512JVBus bus = {flash_command, w25_model_time, w25_model_idle, &flash_model};
    if (!qspi_nor_validated_storage_set_bus(&bus) ||
        !qspi_nor_validated_storage_init(&storage_writer, 0U, SAMPLES) ||
        !qspi_nor_validated_storage_begin(&storage_writer)) return false;
    uint32_t crc = UINT32_MAX;
    for (uint32_t i = 0; i < SAMPLES; ++i) {
        PvExecutionSample p = make_sample(i);
        crc = w25q512jv_crc32(crc, &p, sizeof(p));
        if (!qspi_nor_validated_storage_write_sample(i, &p, &storage_writer)) return false;
    }
    ValidatedTrajectory m = {0};
    m.program_id = 500U;
    m.source_revision = 1U;
    m.sample_count = SAMPLES;
    m.sample_data_crc = ~crc;
    m.sample_period_us = PERIOD_US;
    m.duration_s = (real_t)((SAMPLES - 1U) * 0.002);
    m.segment_count = 1U;
    m.segments[0].first_sample = 0U;
    m.segments[0].sample_count = SAMPLES;
    m.segments[0].segment_type = (uint8_t)TEACH_SEGMENT_LINE;
    m.artifact_crc = w25_artifact_crc(&m);
    if (!qspi_nor_validated_storage_commit(&m, &storage_writer) ||
        !qspi_nor_validated_storage_init(&storage_reader, 0U, SAMPLES) ||
        !qspi_nor_validated_storage_load_committed(&storage_reader)) return false;
    return storage_reader.metadata.program_id == 500U && storage_reader.sample_count == SAMPLES;
}

/* Runs exclusively on producer thread once execution has begun. */
static bool flash_read_batch(uint32_t first, PvExecutionSample *out,
                             uint32_t count, void *context)
{
    ReadSource *src = context;
    const uint64_t begin = wall_us();
    ++src->read_calls;
    if (src->refill_delay_us != 0U) sleep_us(src->refill_delay_us);
    if (!src->stall_fired && src->one_stall_us != 0U && first >= 512U) {
        src->stall_fired = true;
        sleep_us(src->one_stall_us);
    }
    if (first >= src->fail_at) fail_next_transport_read = true;
    bool ok = qspi_nor_validated_storage_read_samples(first, out, count, &storage_reader);
    if (fail_next_transport_read) {
        fprintf(stderr, "[FAIL] transport injection was not consumed\n");
        abort();
    }
    if (!ok) ++src->failed_reads;
    uint64_t elapsed = wall_us() - begin;
    src->read_wall_sum_us += elapsed;
    ++src->read_wall_samples;
    if (elapsed > src->read_wall_max_us) src->read_wall_max_us = elapsed;
    return ok;
}

static void *worker_main(void *context)
{
    Worker *w = context;
    while (!__atomic_load_n(&w->stop, __ATOMIC_ACQUIRE)) {
        (void)trajectory_prefetch_worker_step(&prefetch);
        sleep_us(250U); /* background worker: not an RTOS task scheduler */
    }
    return NULL;
}

static int compare_u64(const void *a, const void *b)
{
    uint64_t x = *(const uint64_t *)a, y = *(const uint64_t *)b;
    return (x > y) - (x < y);
}

static bool run(const char *name, uint32_t delay, uint32_t one_stall, uint32_t fail_at,
                uint32_t per_burst_us, bool expect_success)
{
    ReadSource src = {0};
    src.name = name;
    src.refill_delay_us = delay;
    src.one_stall_us = one_stall;
    src.fail_at = fail_at;
    burst_delay_us = 0U; /* Do not throttle the startup prefill. */
    fail_next_transport_read = false;
    if (!trajectory_prefetch_init(&prefetch) ||
        !trajectory_prefetch_prepare(&prefetch, flash_read_batch, &src, SAMPLES) ||
        !trajectory_prefetch_refill(&prefetch) ||
        !trajectory_prefetch_refill(&prefetch) ||
        !trajectory_prefetch_resume_worker(&prefetch) ||
        !trajectory_prefetch_begin_execution(&prefetch, 0U, SAMPLES)) {
        fprintf(stderr, "[FAIL] %s: preflight or flash-backed 512-sample prefill\n", name);
        return false;
    }
    const uint32_t bursts_after_prefill = burst_read_calls;
    burst_delay_us = per_burst_us;
    Worker worker = {0};
    pthread_t tid;
    if (pthread_create(&tid, NULL, worker_main, &worker) != 0) abort();
    uint64_t delays[SAMPLES];
    uint32_t processed = 0U, misses = 0U, mismatch = 0U, lowest_before_end = 512U;
    uint64_t worst_lateness = 0U;
    bool read_fault = false, starved = false;
    uint64_t start = wall_us() + 5000U;
    for (uint32_t i = 0U; i < SAMPLES; ++i) {
        uint64_t due = start + (uint64_t)i * PERIOD_US;
        sleep_until(due);
        uint64_t now = wall_us();
        uint64_t late = now > due ? now - due : 0U;
        delays[i] = late;
        if (late > worst_lateness) worst_lateness = late;
        if (late > PERIOD_US) ++misses;
        PvExecutionSample actual = {0};
        if (!trajectory_prefetch_take(&prefetch, i, &actual)) {
            read_fault = __atomic_load_n(&prefetch.read_failed, __ATOMIC_ACQUIRE);
            starved = !read_fault && trajectory_prefetch_buffered(&prefetch) == 0U;
            break;
        }
        PvExecutionSample expected = make_sample(i);
        for (unsigned j = 0; j < 6; ++j)
            if (actual.target_position_units[j] != expected.target_position_units[j]) ++mismatch;
        ++processed;
        uint32_t buffered = trajectory_prefetch_buffered(&prefetch);
        /* Zero after the FINAL sample is normal EOF, not a starvation signal. */
        if (i + 1U < SAMPLES && buffered < lowest_before_end)
            lowest_before_end = buffered;
        if (mismatch != 0U) break;
    }
    __atomic_store_n(&worker.stop, true, __ATOMIC_RELEASE);
    pthread_join(tid, NULL);
    burst_delay_us = 0U;
    /* Consumer quiescent. No concurrent worker reset may occur now. */
    trajectory_prefetch_end_execution(&prefetch);
    qsort(delays, processed, sizeof(delays[0]), compare_u64);
    uint64_t p99 = processed != 0U ? delays[(processed - 1U) * 99U / 100U] : 0U;
    const uint32_t execution_bursts = burst_read_calls - bursts_after_prefill;
    const bool success = processed == SAMPLES && mismatch == 0U && !read_fault && !starved;
    const bool expected_failure = !expect_success && !success && processed < SAMPLES &&
                                  mismatch == 0U && (read_fault || starved);
    printf("[SCENARIO] %-12s result=%s samples=%u/%u mismatches=%u "
           "read_fault=%u starvation=%u host_late_p99_us=%" PRIu64
           " host_late_max_us=%" PRIu64 " host_missed_2ms=%u "
           "min_buffer_before_end=%u read_calls=%u read_failures=%u "
           "read_bursts=%u burst_delay_us=%u read_wall_max_us=%" PRIu64 "\n",
           name, success ? "complete" : "stopped", processed, SAMPLES,
           mismatch, read_fault ? 1U : 0U, starved ? 1U : 0U,
           p99, worst_lateness, misses, lowest_before_end, src.read_calls,
           src.failed_reads, execution_bursts, per_burst_us, src.read_wall_max_us);
    if ((expect_success && !success) || (!expect_success && !expected_failure)) {
        fprintf(stderr, "[FAIL] %s: unexpected outcome; see scenario metrics\n", name);
        return false;
    }
    printf("[PASS] %s: %s; no six-joint corruption "
           "(host deadline observations are NOT a hard-real-time pass)\n", name,
           expect_success ? "full 2ms-paced host playback" : "failure detected without continuing samples");
    return true;
}

int main(int argc, char **argv)
{
    const char *which = argc > 1 ? argv[1] : "all";
    if (argc > 2) { fprintf(stderr, "usage: %s [all|baseline|delay20|stall650|fault|burst1ms|burst30ms]\n", argv[0]); return 2; }
    if (strcmp(which, "all") && strcmp(which, "baseline") && strcmp(which, "delay20") &&
        strcmp(which, "stall650") && strcmp(which, "fault") &&
        strcmp(which, "burst1ms") && strcmp(which, "burst30ms")) {
        fprintf(stderr, "unknown scenario: %s\n", which); return 2;
    }
    const uint64_t began = wall_us();
    if (!publish_and_load()) { fputs("[FAIL] flash storage write/commit/reload\n", stderr); return 1; }
    printf("[FLASH] real validated storage: committed/reloaded %u six-axis samples; "
           "model bus, NOT hardware QSPI\n", SAMPLES);
    bool ok = true;
    if (!strcmp(which, "all") || !strcmp(which, "baseline"))
        ok &= run("baseline", 0U, 0U, NO_FAILURE, 0U, true);
    if (!strcmp(which, "all") || !strcmp(which, "delay20"))
        ok &= run("delay20", 20000U, 0U, NO_FAILURE, 0U, true);
    if (!strcmp(which, "all") || !strcmp(which, "stall650"))
        ok &= run("stall650", 0U, 650000U, NO_FAILURE, 0U, false);
    if (!strcmp(which, "all") || !strcmp(which, "fault"))
        ok &= run("fault", 0U, 0U, 512U, 0U, false);
    if (!strcmp(which, "all") || !strcmp(which, "burst1ms"))
        ok &= run("burst1ms", 0U, 0U, NO_FAILURE, 1000U, true);
    if (!strcmp(which, "all") || !strcmp(which, "burst30ms"))
        ok &= run("burst30ms", 0U, 0U, NO_FAILURE, 30000U, false);
    printf("[SUMMARY] %s elapsed_wall_ms=%" PRIu64 "\n", ok ? "PASS" : "FAIL",
           (uint64_t)((wall_us() - began) / 1000U));
    return ok ? 0 : 1;
}
