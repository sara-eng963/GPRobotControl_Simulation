#include "trajectory_prefetch.h"
#include <string.h>

/* GCC/Clang __atomic builtins work on Cortex-M7 aligned 32-bit counters.
 * The producer and consumer may run as two RTOS tasks on ONE core. Do not
 * reconfigure the stream while either task is accessing it. */
#define ACQUIRE __ATOMIC_ACQUIRE
#define RELEASE __ATOMIC_RELEASE
#define RELAXED __ATOMIC_RELAXED

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
    if (s == NULL || read_batch == NULL || sample_count == 0U) return false;
    /* PRECONDITION: worker is quiescent and consumer not running. */
    s->armed = false;
    s->read_batch = read_batch;
    s->read_context = context;
    s->total_samples = sample_count;
    s->next_flash_index = 0U;
    s->refill_batches = 0U;
    s->underruns = 0U;
    __atomic_store_n(&s->read_failed, false, RELEASE);
    __atomic_store_n(&s->written, 0U, RELEASE);
    __atomic_store_n(&s->consumed, 0U, RELEASE);
    s->armed = true;
    return true;
}

void trajectory_prefetch_disarm(TrajectoryPrefetch *s)
{
    if (s == NULL) return;
    /* PRECONDITION: worker and consumer have been stopped. */
    s->armed = false;
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
    if (s == NULL || !s->armed || __atomic_load_n(&s->read_failed, ACQUIRE) ||
        minimum_buffered > TRAJECTORY_PREFETCH_CAPACITY) return false;
    return trajectory_prefetch_buffered(s) >= minimum_buffered;
}

bool trajectory_prefetch_refill(TrajectoryPrefetch *s)
{
    if (s == NULL || !s->armed || __atomic_load_n(&s->read_failed, ACQUIRE) || s->read_batch == NULL)
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
    if (s == NULL || out == NULL || !s->armed || s->read_failed)
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
