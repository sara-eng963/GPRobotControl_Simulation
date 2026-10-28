#define _POSIX_C_SOURCE 200809L
#include "host_timing_trace.h"

#include <inttypes.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define TIMING_RING_CAPACITY 2048U
#define TIMING_RING_MASK (TIMING_RING_CAPACITY - 1U)

typedef struct {
    uint64_t sequence;
    uint64_t start_us;
    uint64_t finish_us;
    uint64_t duration_us;
    uint64_t interval_us;
    uint64_t value;
    bool has_value;
    /* Measured body overrun only; actual RTOS release timestamps unavailable. */
    uint32_t dropped_total;
    uint8_t execution_budget_overrun;
    uint8_t success;
} TimingSample;

typedef struct {
    TimingSample samples[TIMING_RING_CAPACITY];
    uint32_t head;         /* producer only writes */
    uint32_t tail;         /* logger only writes */
    uint32_t dropped;      /* producer only writes */
    uint64_t count;        /* producer only writes */
    uint64_t last_start;   /* producer only writes */
    uint64_t began_us;     /* producer only writes */
    bool in_progress;      /* producer only writes */
} TimingRing;

typedef struct {
    const char *name;
    const char *kind;
    uint32_t period_us; /* Zero for event-driven / best-effort tasks. */
} TimingDefinition;

static const TimingDefinition defs[TIMING_SOURCE_COUNT] = {
    {"supervisor",    "periodic", 1000U},
    {"sim_inputs",    "best_effort", 0U},
    {"hmi",           "periodic", 10000U},
    {"validation",    "activity", 0U},
    {"flash_storage", "activity", 0U},
    {"prefetch",      "best_effort", 0U},
    {"can_targets",   "event", 0U},
    {"path_feedback_wait", "event", 0U},
    {"path_feedback_late", "event", 0U},
    {"path_period_wait",   "event", 0U},
    {"path_commands",      "event", 0U},
    {"path_samples_skipped", "counter", 0U},
    {"path_feedback_age_axis_1", "gauge", 0U},
    {"path_feedback_age_axis_2", "gauge", 0U},
    {"path_feedback_age_axis_3", "gauge", 0U},
    {"path_feedback_age_axis_4", "gauge", 0U},
    {"path_feedback_age_axis_5", "gauge", 0U},
    {"path_feedback_age_axis_6", "gauge", 0U},
    {"supervisor_2ms", "periodic", 2000U},
    {"guidance_observer", "activity", 0U},
    {"guidance_dispatch", "event", 0U},
};
static TimingRing rings[TIMING_SOURCE_COUNT];
static FILE *output;
static pthread_t writer_thread;
static uint64_t epoch_us;
static bool enabled;

static uint64_t monotonic_us(void)
{
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0)
        return 0U;
    return (uint64_t)ts.tv_sec * UINT64_C(1000000) +
           (uint64_t)(ts.tv_nsec / 1000L);
}

/* NEVER called by a periodic task; only background logger touches stdio. */
static void write_sample(unsigned source, const TimingSample *s)
{
    const TimingDefinition *def = &defs[source];
    fprintf(output,
        "3,%s,%s,%" PRIu64 ",%u,%" PRIu64 ",%" PRIu64 ","
        "%" PRIu64 ",%" PRIu64 ",",
        def->name, def->kind, s->sequence, def->period_us,
        s->start_us, s->finish_us, s->duration_us, s->interval_us);
    if (def->period_us != 0U)
        fprintf(output, "%u,", (unsigned)s->execution_budget_overrun);
    else
        fputs(",", output); /* No periodic execution budget. */
    fprintf(output, "%u,%u,", (unsigned)s->success, s->dropped_total);
    if (s->has_value)
        fprintf(output, "%" PRIu64 "\n", s->value);
    else
        fputc('\n', output);
}

static void *writer_entry(void *unused)
{
    (void)unused;
    for (;;) {
        bool wrote = false;
        for (unsigned source = 0; source < TIMING_SOURCE_COUNT; ++source) {
            TimingRing *ring = &rings[source];
            uint32_t tail = __atomic_load_n(&ring->tail, __ATOMIC_RELAXED);
            const uint32_t head =
                __atomic_load_n(&ring->head, __ATOMIC_ACQUIRE);
            while (tail != head) {
                const TimingSample sample =
                    ring->samples[tail & TIMING_RING_MASK];
                write_sample(source, &sample);
                ++tail;
                __atomic_store_n(&ring->tail, tail, __ATOMIC_RELEASE);
                wrote = true;
            }
        }
        if (wrote) (void)fflush(output);
        struct timespec sleep_time = {.tv_sec = 0, .tv_nsec = 5000000L};
        nanosleep(&sleep_time, NULL);
    }
    return NULL;
}

bool timing_init(const char *csv_path)
{
    if (csv_path == NULL || csv_path[0] == '\0') return true;
    if (enabled) return false;
    epoch_us = monotonic_us();
    if (epoch_us == 0U) return false;
    output = fopen(csv_path, "w");
    if (output == NULL) {
        perror("[TIMING] Cannot open CSV");
        return false;
    }
    fputs("schema_version,source,kind,sequence,period_us,start_us,finish_us,"
          "duration_us,interval_us,execution_budget_overrun,"
          "success,dropped_total,value\n", output);
    if (fflush(output) != 0) {
        fclose(output);
        output = NULL;
        return false;
    }
    /* The rings and epoch are ready before the producer tasks are started. */
    __atomic_store_n(&enabled, true, __ATOMIC_RELEASE);
    if (pthread_create(&writer_thread, NULL, writer_entry, NULL) != 0) {
        __atomic_store_n(&enabled, false, __ATOMIC_RELEASE);
        fclose(output);
        output = NULL;
        return false;
    }
    (void)pthread_detach(writer_thread);
    fprintf(stderr, "[TIMING] Nonblocking host timing CSV: %s\n", csv_path);
    return true;
}

void timing_begin(TimingSource source)
{
    if ((unsigned)source >= TIMING_SOURCE_COUNT ||
        !__atomic_load_n(&enabled, __ATOMIC_ACQUIRE)) return;
    TimingRing *ring = &rings[source];
    ring->began_us = monotonic_us() - epoch_us;
    ring->in_progress = true;
}

void timing_end(TimingSource source, bool success)
{
    if ((unsigned)source >= TIMING_SOURCE_COUNT ||
        !__atomic_load_n(&enabled, __ATOMIC_ACQUIRE)) return;
    TimingRing *ring = &rings[source];
    if (!ring->in_progress) return;
    const uint64_t finished_us = monotonic_us() - epoch_us;
    const uint64_t started_us = ring->began_us;
    ring->in_progress = false;
    const uint64_t sequence = ring->count++;
    const uint64_t last_start = ring->last_start;
    ring->last_start = started_us;

    TimingSample sample = {0};
    sample.sequence = sequence;
    sample.start_us = started_us;
    sample.finish_us = finished_us;
    sample.duration_us = finished_us >= started_us
        ? finished_us - started_us : 0U;
    sample.interval_us = sequence > 0U && started_us >= last_start
        ? started_us - last_start : 0U;
    const uint32_t period = defs[source].period_us;
    if (period != 0U)
        sample.execution_budget_overrun =
            (uint8_t)(sample.duration_us > period);
    sample.success = (uint8_t)(success ? 1U : 0U);
    const uint32_t head =
        __atomic_load_n(&ring->head, __ATOMIC_RELAXED);
    const uint32_t tail =
        __atomic_load_n(&ring->tail, __ATOMIC_ACQUIRE);
    if ((uint32_t)(head - tail) >= TIMING_RING_CAPACITY) {
        ++ring->dropped; /* Never block control tasks to log. */
        return;
    }
    sample.dropped_total = ring->dropped;
    ring->samples[head & TIMING_RING_MASK] = sample;
    __atomic_store_n(&ring->head, head + 1U, __ATOMIC_RELEASE);
}

/* Numeric diagnostics reuse single-producer rings and their async writer. */
void timing_value(TimingSource source, uint64_t value)
{
    if ((unsigned)source >= TIMING_SOURCE_COUNT ||
        !__atomic_load_n(&enabled, __ATOMIC_ACQUIRE)) return;
    TimingRing *ring = &rings[source];
    const uint64_t now = monotonic_us() - epoch_us;
    const uint64_t sequence = ring->count++;
    const uint64_t previous = ring->last_start;
    ring->last_start = now;
    TimingSample sample = {0};
    sample.sequence = sequence;
    sample.start_us = now;
    sample.finish_us = now;
    sample.interval_us = sequence > 0U && now >= previous ? now - previous : 0U;
    sample.success = 1U;
    sample.value = value;
    sample.has_value = true;
    const uint32_t head = __atomic_load_n(&ring->head, __ATOMIC_RELAXED);
    const uint32_t tail = __atomic_load_n(&ring->tail, __ATOMIC_ACQUIRE);
    if ((uint32_t)(head - tail) >= TIMING_RING_CAPACITY) {
        ++ring->dropped;
        return;
    }
    sample.dropped_total = ring->dropped;
    ring->samples[head & TIMING_RING_MASK] = sample;
    __atomic_store_n(&ring->head, head + 1U, __ATOMIC_RELEASE);
}
