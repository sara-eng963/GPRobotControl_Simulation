#ifndef TRAJECTORY_PREFETCH_H
#define TRAJECTORY_PREFETCH_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

/* Small, ABI-compatible sample stub for the standalone host tests ONLY. */
#if defined(TRAJECTORY_PREFETCH_HOST_TEST)
typedef struct { int32_t target_position_units[6]; } PvExecutionSample;
#else
#include "../../StateMachine/States/state_path_validation.h"
#endif

#define TRAJECTORY_PREFETCH_CAPACITY 512U
#define TRAJECTORY_PREFETCH_BATCH 256U

/* Called only by the lower-priority refill worker. Must not be called in
 * PATH_EXECUTION, an ISR, or another time-critical controller path. */
typedef bool (*TrajectoryPrefetchReadBatch)(uint32_t first_sample,
                                             PvExecutionSample *out,
                                             uint32_t count,
                                             void *context);

typedef struct {
    PvExecutionSample samples[TRAJECTORY_PREFETCH_CAPACITY];
    PvExecutionSample staging[TRAJECTORY_PREFETCH_BATCH];
    TrajectoryPrefetchReadBatch read_batch;
    void *read_context;
    uint32_t total_samples;
    uint32_t next_flash_index; /* producer owns */
    uint32_t written;          /* producer publishes with release ordering */
    uint32_t consumed;         /* consumer publishes with release ordering */
    uint32_t refill_batches;
    uint32_t underruns;
    bool read_failed;
    bool armed;
} TrajectoryPrefetch;

/* Initialize once, before any tasks are started. */
bool trajectory_prefetch_init(TrajectoryPrefetch *stream);

/* A reset of an active stream is NOT concurrent-safe. Stop/join the refill
 * worker and ensure PATH_EXECUTION is inactive before prepare/disarm. */
bool trajectory_prefetch_prepare(TrajectoryPrefetch *stream,
                                TrajectoryPrefetchReadBatch read_batch,
                                void *read_context, uint32_t sample_count);
void trajectory_prefetch_disarm(TrajectoryPrefetch *stream);

/* Worker-only. Executes at most ONE batch-sized storage read. A blocking
 * read here is permissible only at lower priority than the CAN/supervisor
 * control task. It must be bounded by the driver timeout. */
bool trajectory_prefetch_refill(TrajectoryPrefetch *stream);

/* Consumer-only. Pure SRAM read. No callbacks, QSPI, mutex or task waits. */
bool trajectory_prefetch_take(TrajectoryPrefetch *stream,
                              uint32_t expected_index,
                              PvExecutionSample *out);

/* Readiness check for the start-motion preflight (must be used before
 * enabling production welding / issuing the first trajectory sample). */
bool trajectory_prefetch_ready(const TrajectoryPrefetch *stream,
                               uint32_t minimum_buffered);
uint32_t trajectory_prefetch_buffered(const TrajectoryPrefetch *stream);

#endif
