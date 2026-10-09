#include "validated_stream_buffer.h"

#include <string.h>


bool validated_stream_buffer_init(
    ValidatedStreamBuffer *buffer,
    PvExecutionSample *storage,
    size_t capacity
)
{
    if (
        buffer == NULL ||
        storage == NULL ||
        capacity == 0U
    )
    {
        return false;
    }

    memset(
        buffer,
        0,
        sizeof(*buffer)
    );

    buffer->storage =
        storage;

    buffer->capacity =
        capacity;

    return true;
}


void validated_stream_buffer_reset(
    ValidatedStreamBuffer *buffer
)
{
    if (buffer == NULL)
    {
        return;
    }

    buffer->read_index =
        0U;

    buffer->write_index =
        0U;

    buffer->count =
        0U;

    buffer->high_water_mark =
        0U;

    buffer->total_pushed =
        0U;

    buffer->total_popped =
        0U;

    buffer->underrun_count =
        0U;
}


bool validated_stream_buffer_push(
    ValidatedStreamBuffer *buffer,
    const PvExecutionSample *sample
)
{
    if (
        buffer == NULL ||
        sample == NULL ||
        buffer->storage == NULL ||
        buffer->capacity == 0U ||
        buffer->count >= buffer->capacity
    )
    {
        return false;
    }

    buffer->storage[
        buffer->write_index
    ] = *sample;

    buffer->write_index =
        (buffer->write_index + 1U) %
        buffer->capacity;

    buffer->count++;
    buffer->total_pushed++;

    if (
        buffer->count >
        buffer->high_water_mark
    )
    {
        buffer->high_water_mark =
            buffer->count;
    }

    return true;
}


bool validated_stream_buffer_pop(
    ValidatedStreamBuffer *buffer,
    PvExecutionSample *sample
)
{
    if (
        buffer == NULL ||
        sample == NULL ||
        buffer->storage == NULL ||
        buffer->capacity == 0U
    )
    {
        return false;
    }

    if (buffer->count == 0U)
    {
        buffer->underrun_count++;
        return false;
    }

    *sample =
        buffer->storage[
            buffer->read_index
        ];

    buffer->read_index =
        (buffer->read_index + 1U) %
        buffer->capacity;

    buffer->count--;
    buffer->total_popped++;

    return true;
}


size_t validated_stream_buffer_count(
    const ValidatedStreamBuffer *buffer
)
{
    if (buffer == NULL)
    {
        return 0U;
    }

    return
        buffer->count;
}


size_t validated_stream_buffer_free(
    const ValidatedStreamBuffer *buffer
)
{
    if (
        buffer == NULL ||
        buffer->capacity < buffer->count
    )
    {
        return 0U;
    }

    return
        buffer->capacity -
        buffer->count;
}


bool validated_stream_buffer_is_empty(
    const ValidatedStreamBuffer *buffer
)
{
    return
        buffer == NULL ||
        buffer->count == 0U;
}


bool validated_stream_buffer_is_full(
    const ValidatedStreamBuffer *buffer
)
{
    return
        buffer != NULL &&
        buffer->capacity > 0U &&
        buffer->count >=
            buffer->capacity;
}