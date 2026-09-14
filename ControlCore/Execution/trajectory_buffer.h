#ifndef TRAJECTORY_BUFFER_H
#define TRAJECTORY_BUFFER_H

#include "../Math/control_types.h"

#include <stdbool.h>
#include <stddef.h>

/*
 * Fixed-size FIFO ring buffer for 1 ms joint-reference samples.
 *
 * Important:
 * - No dynamic allocation.
 * - No FreeRTOS dependency.
 * - One JointVector = one future EtherCAT CSP cycle.
 * - Total trajectory duration is NOT limited by this buffer size.
 *
 * Concurrency:
 * This module is intentionally RTOS-agnostic. When a planner task produces
 * samples while the EtherCAT task consumes them, protect push/pop operations
 * with the RTOS synchronization strategy used by the final controller.
 */
typedef struct
{
    JointVector *storage;
    size_t capacity;

    size_t readIndex;
    size_t writeIndex;
    size_t count;

    size_t highWaterMark;
    size_t totalPushed;
    size_t totalPopped;

} TrajectoryBuffer;

bool trajectory_buffer_init(
    TrajectoryBuffer *buffer,
    JointVector *storage,
    size_t capacity
);

void trajectory_buffer_reset(
    TrajectoryBuffer *buffer
);

bool trajectory_buffer_push(
    TrajectoryBuffer *buffer,
    const JointVector *sample
);

bool trajectory_buffer_pop(
    TrajectoryBuffer *buffer,
    JointVector *sample
);

bool trajectory_buffer_peek(
    const TrajectoryBuffer *buffer,
    JointVector *sample
);

size_t trajectory_buffer_count(
    const TrajectoryBuffer *buffer
);

size_t trajectory_buffer_free(
    const TrajectoryBuffer *buffer
);

bool trajectory_buffer_is_empty(
    const TrajectoryBuffer *buffer
);

bool trajectory_buffer_is_full(
    const TrajectoryBuffer *buffer
);

#endif /* TRAJECTORY_BUFFER_H */
