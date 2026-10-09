#include "trajectory_prefetch.h"
#include <string.h>

/* GCC/Clang __atomic builtins work on Cortex-M7 aligned 32-bit counters.
 * The producer and consumer may run as two RTOS tasks on ONE core. Do not
 * reconfigure the stream while either task is accessing it. */
#define ACQUIRE __ATOMIC_ACQUIRE
#define RELEASE __ATOMIC_RELEASE
#define RELAXED __ATOMIC_RELAXED
_Static_assert(__atomic_always_lock_free(sizeof(uint32_t), 0),
               "prefetch requires lock-free 32-bit atomics");
_Static_assert(__atomic_always_lock_free(sizeof(bool), 0),
               "prefetch requires lock-free flag atomics");

static uint32_t lifecycle(const TrajectoryPrefetch *s)
{
    return __atomic_load_n(&s->lifecycle, ACQUIRE);
}

static void reset_counters(TrajectoryPrefetch *s)
{
    s->next_flash_index = 0U;
    s->refill_batches = 0U; /* producer-owned; inspect only when quiescent */
    s->underruns = 0U;     /* consumer-owned; reset only after its release */
    __atomic_store_n(&s->read_failed, false, RELEASE);
    __atomic_store_n(&s->written, 0U, RELEASE);
    __atomic_store_n(&s->consumed, 0U, RELEASE);
}

bool trajectory_prefetch_init(TrajectoryPrefetch *s)
{
    if (s == NULL) return false;
    memset(s, 0, sizeof(*s));
    return sizeof(PvExecutionSample) == 24U;
}

bool trajectory_prefetch_prepare(TrajectoryPrefetch *s,
                                TrajectoryPrefetchReadBatch read_batch,
                                void *context, uint32_t sample_count)
{
    if (s == NULL || read_batch == NULL || sample_count == 0U ||
        !trajectory_prefetch_stopped(s)) return false;
    /* PRECONDITION: worker is quiescent and consumer not running. */
    __atomic_store_n(&s->armed, false, RELEASE);
    s->read_batch = read_batch;
    s->read_context = context;
    s->total_samples = sample_count;
    reset_counters(s);
    __atomic_store_n(&s->armed, true, RELEASE);
    return true;
}

void trajectory_prefetch_disarm(TrajectoryPrefetch *s)
{
    if (s == NULL || !trajectory_prefetch_stopped(s)) return;
    /* PRECONDITION: worker and consumer have been stopped. */
    __atomic_store_n(&s->armed, false, RELEASE);
}

bool trajectory_prefetch_stopped(const TrajectoryPrefetch *s)
{
    return s != NULL && lifecycle(s) == TRAJECTORY_PREFETCH_STOPPED;
}

bool trajectory_prefetch_request_stop(TrajectoryPrefetch *s)
{
    if (s == NULL) return false;
    uint32_t mode = lifecycle(s);
    for (;;) {
        if (mode == TRAJECTORY_PREFETCH_EXECUTING) return false;
        if (mode == TRAJECTORY_PREFETCH_STOPPED ||
            mode == TRAJECTORY_PREFETCH_STOP_PENDING) return true;
        if (__atomic_compare_exchange_n(&s->lifecycle, &mode,
                TRAJECTORY_PREFETCH_STOP_PENDING, false, __ATOMIC_ACQ_REL, ACQUIRE))
            return true;
    }
}

bool trajectory_prefetch_resume_worker(TrajectoryPrefetch *s)
{
    if (s == NULL || !__atomic_load_n(&s->armed, ACQUIRE) ||
        __atomic_load_n(&s->read_failed, ACQUIRE)) return false;
    uint32_t mode = TRAJECTORY_PREFETCH_STOPPED;
    return __atomic_compare_exchange_n(&s->lifecycle, &mode,
            TRAJECTORY_PREFETCH_REFILLING, false, RELEASE, RELAXED);
}

bool trajectory_prefetch_begin_execution(TrajectoryPrefetch *s,
                                        uint32_t next_index,
                                        uint32_t sample_count)
{
    if (s == NULL) return false;
    uint32_t mode = lifecycle(s);
    if (mode != TRAJECTORY_PREFETCH_REFILLING &&
        mode != TRAJECTORY_PREFETCH_EXECUTING) return false;
    if (sample_count != s->total_samples || next_index > sample_count ||
        __atomic_load_n(&s->consumed, ACQUIRE) != next_index) return false;
    const uint32_t minimum = mode == TRAJECTORY_PREFETCH_REFILLING
        ? (sample_count < TRAJECTORY_PREFETCH_CAPACITY
            ? sample_count : TRAJECTORY_PREFETCH_CAPACITY)
        : (next_index < sample_count ? 1U : 0U);
    if (!trajectory_prefetch_ready(s, minimum)) return false;
    if (mode == TRAJECTORY_PREFETCH_EXECUTING) return true;
    if (next_index != 0U) return false;
    return __atomic_compare_exchange_n(&s->lifecycle, &mode,
            TRAJECTORY_PREFETCH_EXECUTING, false, __ATOMIC_ACQ_REL, ACQUIRE);
}

void trajectory_prefetch_end_execution(TrajectoryPrefetch *s)
{
    if (s == NULL) return;
    uint32_t mode = lifecycle(s);
    /* The control owner has exited execution (or failed its preflight).
     * Its release orders the last take before the worker's counter reset. */
    while (mode == TRAJECTORY_PREFETCH_EXECUTING ||
           mode == TRAJECTORY_PREFETCH_REFILLING) {
        if (__atomic_compare_exchange_n(&s->lifecycle, &mode,
                TRAJECTORY_PREFETCH_REPLAY_PENDING, false, __ATOMIC_ACQ_REL, ACQUIRE))
            return;
    }
}

bool trajectory_prefetch_worker_step(TrajectoryPrefetch *s)
{
    if (s == NULL) return false;
    uint32_t mode = lifecycle(s);
    if (mode == TRAJECTORY_PREFETCH_STOP_PENDING) {
        /* Only the worker acknowledges, after its previous read/publication
         * has returned. No source or configuration accesses while STOPPED. */
        __atomic_store_n(&s->lifecycle, TRAJECTORY_PREFETCH_STOPPED, RELEASE);
        return true;
    }
    if (mode == TRAJECTORY_PREFETCH_STOPPED) return true;
    if (mode == TRAJECTORY_PREFETCH_REPLAY_PENDING) {
        reset_counters(s);
        /* A stop request may arrive during reset; never overwrite it. */
        if (!__atomic_compare_exchange_n(&s->lifecycle, &mode,
                TRAJECTORY_PREFETCH_REFILLING, false, RELEASE, RELAXED))
            return true;
    }
    if (trajectory_prefetch_buffered(s) > TRAJECTORY_PREFETCH_BATCH) return true;
    return trajectory_prefetch_refill(s);
}

uint32_t trajectory_prefetch_buffered(const TrajectoryPrefetch *s)
{
    if (s == NULL) return 0U;
    const uint32_t w = __atomic_load_n(&s->written, ACQUIRE);
    const uint32_t r = __atomic_load_n(&s->consumed, ACQUIRE);
    return (w >= r && w-r <= TRAJECTORY_PREFETCH_CAPACITY) ? w-r : 0U;
}

bool trajectory_prefetch_ready(const TrajectoryPrefetch *s,
                               uint32_t minimum_buffered)
{
    if (s == NULL) return false;
    const uint32_t mode = lifecycle(s);
    if ((mode != TRAJECTORY_PREFETCH_REFILLING &&
         mode != TRAJECTORY_PREFETCH_EXECUTING) ||
        !__atomic_load_n(&s->armed, ACQUIRE) || __atomic_load_n(&s->read_failed, ACQUIRE) ||
        minimum_buffered > TRAJECTORY_PREFETCH_CAPACITY) return false;
    return trajectory_prefetch_buffered(s) >= minimum_buffered;
}

bool trajectory_prefetch_refill(TrajectoryPrefetch *s)
{
    if (s == NULL) return false;
    const uint32_t mode = lifecycle(s);
    if ((mode != TRAJECTORY_PREFETCH_STOPPED &&
         mode != TRAJECTORY_PREFETCH_REFILLING &&
         mode != TRAJECTORY_PREFETCH_EXECUTING) ||
        !__atomic_load_n(&s->armed, ACQUIRE) ||
        __atomic_load_n(&s->read_failed, ACQUIRE) || s->read_batch == NULL)
        return false;
    if (s->next_flash_index >= s->total_samples) return true;

    const uint32_t w = __atomic_load_n(&s->written, RELAXED);
    const uint32_t r = __atomic_load_n(&s->consumed, ACQUIRE);
    if (w < r || w-r > TRAJECTORY_PREFETCH_CAPACITY) {
        __atomic_store_n(&s->read_failed, true, RELEASE);
        return false;
    }
    uint32_t free_slots = TRAJECTORY_PREFETCH_CAPACITY - (w-r);
    if (free_slots == 0U) return true;

    uint32_t batch = s->total_samples - s->next_flash_index;
    if (batch > TRAJECTORY_PREFETCH_BATCH) batch = TRAJECTORY_PREFETCH_BATCH;
    if (batch > free_slots) batch = free_slots;

    /* Only this function reads storage; staging is not visible to the consumer. */
    if (!s->read_batch(s->next_flash_index, s->staging, batch,
                       s->read_context)) {
        __atomic_store_n(&s->read_failed, true, RELEASE);
        return false;
    }
    for (uint32_t i = 0; i < batch; ++i)
        s->samples[(w+i) % TRAJECTORY_PREFETCH_CAPACITY] = s->staging[i];

    /* Publish ONLY after every byte of the batch is written to SRAM. */
    __atomic_store_n(&s->written, w+batch, RELEASE);
    s->next_flash_index += batch;
    s->refill_batches++;
    return true;
}

bool trajectory_prefetch_take(TrajectoryPrefetch *s,
                              uint32_t expected_index,
                              PvExecutionSample *out)
{
    if (s == NULL || out == NULL || lifecycle(s) != TRAJECTORY_PREFETCH_EXECUTING ||
        !__atomic_load_n(&s->armed, ACQUIRE) || __atomic_load_n(&s->read_failed, ACQUIRE))
        return false;
    const uint32_t r = __atomic_load_n(&s->consumed, RELAXED);
    const uint32_t w = __atomic_load_n(&s->written, ACQUIRE);
    /* Nonsequential execution is rejected, never silently skipped. */
    if (expected_index != r || r >= s->total_samples) return false;
    if (w == r) {
        s->underruns++;
        return false;
    }
    *out = s->samples[r % TRAJECTORY_PREFETCH_CAPACITY];
    __atomic_store_n(&s->consumed, r+1U, RELEASE);
    return true;
}
