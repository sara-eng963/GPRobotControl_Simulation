#define _POSIX_C_SOURCE 200809L
#include "host_timing_trace.h"

#include <inttypes.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define GP_TIMING_RING_CAPACITY 2048U
#define GP_TIMING_RING_MASK (GP_TIMING_RING_CAPACITY - 1U)

typedef struct {
    uint64_t sequence;
    uint64_t start_us;
    uint64_t finish_us;
    uint64_t duration_us;
    uint64_t interval_us;
    uint64_t scheduled_us;
    int64_t start_late_us;
    int64_t finish_late_us;
    uint32_t dropped_total;
    uint8_t missed_deadline;
    uint8_t success;
} GpTimingSample;

typedef struct {
    GpTimingSample samples[GP_TIMING_RING_CAPACITY];
    uint32_t head;         /* producer only writes */
    uint32_t tail;         /* logger only writes */
    uint32_t dropped;      /* producer only writes */
    uint64_t count;        /* producer only writes */
    uint64_t first_start;  /* producer only writes */
    uint64_t last_start;   /* producer only writes */
    uint64_t began_us;     /* producer only writes */
    bool in_progress;      /* producer only writes */
} GpTimingRing;

typedef struct {
    const char *name;
    const char *kind;
    uint32_t period_us; /* Zero for event-driven / best-effort tasks. */
} GpTimingDefinition;

static const GpTimingDefinition defs[GP_TIMING_SOURCE_COUNT] = {
    {"supervisor",    "periodic", 1000U},
    {"sim_inputs",    "best_effort", 0U},
    {"hmi",           "periodic", 10000U},
    {"validation",    "activity", 0U},
    {"flash_storage", "activity", 0U},
    {"prefetch",      "best_effort", 0U},
    {"can_targets",   "event", 0U},
};
static GpTimingRing rings[GP_TIMING_SOURCE_COUNT];
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
static void write_sample(unsigned source, const GpTimingSample *s)
{
    const GpTimingDefinition *def = &defs[source];
    fprintf(output,
        "1,%s,%s,%" PRIu64 ",%u,%" PRIu64 ",%" PRIu64 ","
        "%" PRIu64 ",%" PRIu64 ",",
        def->name, def->kind, s->sequence, def->period_us,
        s->start_us, s->finish_us, s->duration_us, s->interval_us);
    if (def->period_us != 0U)
        fprintf(output, "%" PRIu64 ",%" PRId64 ",%" PRId64 ",%u,",
                s->scheduled_us, s->start_late_us, s->finish_late_us,
                (unsigned)s->missed_deadline);
    else
        fputs(",,,,", output); /* scheduled, start_late, finish_late, missed */
    fprintf(output, "%u,%u\n", (unsigned)s->success, s->dropped_total);
}

static void *writer_entry(void *unused)
{
    (void)unused;
    for (;;) {
        bool wrote = false;
        for (unsigned source = 0; source < GP_TIMING_SOURCE_COUNT; ++source) {
            GpTimingRing *ring = &rings[source];
            uint32_t tail = __atomic_load_n(&ring->tail, __ATOMIC_RELAXED);
            const uint32_t head =
                __atomic_load_n(&ring->head, __ATOMIC_ACQUIRE);
            while (tail != head) {
                const GpTimingSample sample =
                    ring->samples[tail & GP_TIMING_RING_MASK];
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

bool gp_timing_init(const char *csv_path)
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
          "duration_us,interval_us,scheduled_us,start_lateness_us,"
          "finish_lateness_us,missed_deadline,success,dropped_total\n", output);
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

void gp_timing_begin(GpTimingSource source)
{
    if ((unsigned)source >= GP_TIMING_SOURCE_COUNT ||
        !__atomic_load_n(&enabled, __ATOMIC_ACQUIRE)) return;
    GpTimingRing *ring = &rings[source];
    ring->began_us = monotonic_us() - epoch_us;
    ring->in_progress = true;
}

void gp_timing_end(GpTimingSource source, bool success)
{
    if ((unsigned)source >= GP_TIMING_SOURCE_COUNT ||
        !__atomic_load_n(&enabled, __ATOMIC_ACQUIRE)) return;
    GpTimingRing *ring = &rings[source];
    if (!ring->in_progress) return;
    const uint64_t finished_us = monotonic_us() - epoch_us;
    const uint64_t started_us = ring->began_us;
    ring->in_progress = false;
    const uint64_t sequence = ring->count++;
    if (sequence == 0U) ring->first_start = started_us;
    const uint64_t last_start = ring->last_start;
    ring->last_start = started_us;

    GpTimingSample sample = {0};
    sample.sequence = sequence;
    sample.start_us = started_us;
    sample.finish_us = finished_us;
    sample.duration_us = finished_us >= started_us
        ? finished_us - started_us : 0U;
    sample.interval_us = sequence > 0U && started_us >= last_start
        ? started_us - last_start : 0U;
    const uint32_t period = defs[source].period_us;
    if (period != 0U) {
        sample.scheduled_us = ring->first_start +
            sequence * (uint64_t)period;
        sample.start_late_us =
            (int64_t)started_us - (int64_t)sample.scheduled_us;
        sample.finish_late_us =
            (int64_t)finished_us -
            (int64_t)(sample.scheduled_us + period);
        sample.missed_deadline = (uint8_t)(sample.finish_late_us > 0);
    }
    sample.success = (uint8_t)(success ? 1U : 0U);
    const uint32_t head =
        __atomic_load_n(&ring->head, __ATOMIC_RELAXED);
    const uint32_t tail =
        __atomic_load_n(&ring->tail, __ATOMIC_ACQUIRE);
    if ((uint32_t)(head - tail) >= GP_TIMING_RING_CAPACITY) {
        ++ring->dropped; /* Never block control tasks to log. */
        return;
    }
    sample.dropped_total = ring->dropped;
    ring->samples[head & GP_TIMING_RING_MASK] = sample;
    __atomic_store_n(&ring->head, head + 1U, __ATOMIC_RELEASE);
}
