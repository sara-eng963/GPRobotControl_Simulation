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

typedef enum {
    TRAJECTORY_PREFETCH_STOPPED = 0,
    TRAJECTORY_PREFETCH_REFILLING,
    TRAJECTORY_PREFETCH_EXECUTING,
    TRAJECTORY_PREFETCH_REPLAY_PENDING,
    TRAJECTORY_PREFETCH_STOP_PENDING
} TrajectoryPrefetchLifecycle;

typedef struct {
    PvExecutionSample samples[TRAJECTORY_PREFETCH_CAPACITY];
    PvExecutionSample staging[TRAJECTORY_PREFETCH_BATCH];
    TrajectoryPrefetchReadBatch read_batch;
    void *read_context;
    uint32_t total_samples;    /* immutable until acknowledged STOPPED */
    uint32_t next_flash_index; /* producer owns */
    uint32_t written;          /* producer publishes with release ordering */
    uint32_t consumed;         /* consumer publishes with release ordering */
    uint32_t refill_batches;   /* producer owns; inspect only when STOPPED */
    uint32_t underruns;        /* consumer owns; worker resets after release */
    bool read_failed;          /* atomic fault publication */
    bool armed;                /* atomic; configuration still needs quiescence */
    uint32_t lifecycle; /* atomic; never access shared fields directly */
} TrajectoryPrefetch;

/* Initialize once, before any tasks are started. */
bool trajectory_prefetch_init(TrajectoryPrefetch *stream);

/* One control owner (the consumer's task) manages the lifecycle. It must stop
 * calling take before end_execution/request_stop. PAUSED retains ownership.
 * prepare/disarm require a worker STOPPED acknowledgement. Initial prefill may
 * call refill while STOPPED; resume_worker publishes configuration afterwards.
 * The worker must use worker_step, not refill directly, during task operation.
 */
bool trajectory_prefetch_prepare(TrajectoryPrefetch *stream,
                                TrajectoryPrefetchReadBatch read_batch,
                                void *read_context, uint32_t sample_count);
void trajectory_prefetch_disarm(TrajectoryPrefetch *stream);
bool trajectory_prefetch_request_stop(TrajectoryPrefetch *stream);
bool trajectory_prefetch_stopped(const TrajectoryPrefetch *stream);
bool trajectory_prefetch_resume_worker(TrajectoryPrefetch *stream);
bool trajectory_prefetch_worker_step(TrajectoryPrefetch *stream);

/* SRAM-only, nonblocking execution preflight. Fresh execution needs
 * min(sample_count,512) samples at index zero. Rechecks/resume retain ownership
 * and require the next sample (if any), without resetting the stream. */
bool trajectory_prefetch_begin_execution(TrajectoryPrefetch *stream,
                                        uint32_t next_index,
                                        uint32_t sample_count);
/* Explicit consumer quiescence, including external fault/abort transitions.
 * Requests automatic replay preparation by the worker; it performs no reads. */
void trajectory_prefetch_end_execution(TrajectoryPrefetch *stream);

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
