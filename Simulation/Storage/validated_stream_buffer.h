#ifndef VALIDATED_STREAM_BUFFER_H
#define VALIDATED_STREAM_BUFFER_H

#include "../../StateMachine/States/state_path_validation.h"

#include <stdbool.h>
#include <stddef.h>


typedef struct
{
    PvExecutionSample *storage;

    size_t capacity;
    size_t read_index;
    size_t write_index;
    size_t count;

    size_t high_water_mark;

    size_t total_pushed;
    size_t total_popped;

    size_t underrun_count;

} ValidatedStreamBuffer;


/*
 * Initialize a fixed-size RAM ring buffer.
 *
 * The caller owns the actual sample array.
 * No dynamic allocation is used.
 */
bool validated_stream_buffer_init(
    ValidatedStreamBuffer *buffer,
    PvExecutionSample *storage,
    size_t capacity
);


/*
 * Empty the buffer and reset statistics.
 */
void validated_stream_buffer_reset(
    ValidatedStreamBuffer *buffer
);


/*
 * Add one trajectory sample.
 */
bool validated_stream_buffer_push(
    ValidatedStreamBuffer *buffer,
    const PvExecutionSample *sample
);


/*
 * Remove the oldest trajectory sample.
 *
 * If the buffer is empty, the call fails and
 * underrun_count is incremented.
 */
bool validated_stream_buffer_pop(
    ValidatedStreamBuffer *buffer,
    PvExecutionSample *sample
);


size_t validated_stream_buffer_count(
    const ValidatedStreamBuffer *buffer
);


size_t validated_stream_buffer_free(
    const ValidatedStreamBuffer *buffer
);


bool validated_stream_buffer_is_empty(
    const ValidatedStreamBuffer *buffer
);


bool validated_stream_buffer_is_full(
    const ValidatedStreamBuffer *buffer
);


#endif