#include "trajectory_buffer.h"

#include <string.h>

bool trajectory_buffer_init(
    TrajectoryBuffer *buffer,
    JointVector *storage,
    size_t capacity
)
{
    if (
        buffer == NULL ||
        storage == NULL ||
        capacity == 0
    )
    {
        return false;
    }

    memset(buffer, 0, sizeof(*buffer));

    buffer->storage = storage;
    buffer->capacity = capacity;

    return true;
}

void trajectory_buffer_reset(
    TrajectoryBuffer *buffer
)
{
    if (buffer == NULL)
    {
        return;
    }

    buffer->readIndex = 0;
    buffer->writeIndex = 0;
    buffer->count = 0;

    buffer->highWaterMark = 0;
    buffer->totalPushed = 0;
    buffer->totalPopped = 0;
}

bool trajectory_buffer_push(
    TrajectoryBuffer *buffer,
    const JointVector *sample
)
{
    if (
        buffer == NULL ||
        sample == NULL ||
        buffer->storage == NULL ||
        buffer->capacity == 0 ||
        buffer->count >= buffer->capacity
    )
    {
        return false;
    }

    buffer->storage[buffer->writeIndex] =
        *sample;

    buffer->writeIndex =
        (buffer->writeIndex + 1U) %
        buffer->capacity;

    buffer->count++;
    buffer->totalPushed++;

    if (buffer->count > buffer->highWaterMark)
    {
        buffer->highWaterMark =
            buffer->count;
    }

    return true;
}

bool trajectory_buffer_pop(
    TrajectoryBuffer *buffer,
    JointVector *sample
)
{
    if (
        buffer == NULL ||
        sample == NULL ||
        buffer->storage == NULL ||
        buffer->count == 0
    )
    {
        return false;
    }

    *sample =
        buffer->storage[buffer->readIndex];

    buffer->readIndex =
        (buffer->readIndex + 1U) %
        buffer->capacity;

    buffer->count--;
    buffer->totalPopped++;

    return true;
}

bool trajectory_buffer_peek(
    const TrajectoryBuffer *buffer,
    JointVector *sample
)
{
    if (
        buffer == NULL ||
        sample == NULL ||
        buffer->storage == NULL ||
        buffer->count == 0
    )
    {
        return false;
    }

    *sample =
        buffer->storage[buffer->readIndex];

    return true;
}

size_t trajectory_buffer_count(
    const TrajectoryBuffer *buffer
)
{
    if (buffer == NULL)
    {
        return 0;
    }

    return buffer->count;
}

size_t trajectory_buffer_free(
    const TrajectoryBuffer *buffer
)
{
    if (
        buffer == NULL ||
        buffer->capacity < buffer->count
    )
    {
        return 0;
    }

    return
        buffer->capacity -
        buffer->count;
}

bool trajectory_buffer_is_empty(
    const TrajectoryBuffer *buffer
)
{
    return
        buffer == NULL ||
        buffer->count == 0;
}

bool trajectory_buffer_is_full(
    const TrajectoryBuffer *buffer
)
{
    return
        buffer != NULL &&
        buffer->capacity > 0 &&
        buffer->count >= buffer->capacity;
}
