/* PathExecutionServices-compatible M7 callback regression.
 * Runs independently from the legacy single-core prefetch implementation.
 */
#include "../trajectory_stream_path_port.h"
#include <assert.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

static bool source_read(uint32_t first, TrajectorySample *out,
                        uint32_t count, void *unused)
{
    (void)unused;
    for (uint32_t i = 0U; i < count; ++i)
        for (unsigned j = 0U; j < 6U; ++j)
            out[i].target_position_units[j] =
                (int32_t)((first + i) * 7U + j);
    return true;
}

static void verify_sample(const PvExecutionSample *sample, uint32_t index)
{
    for (unsigned j = 0U; j < 6U; ++j)
        assert(sample->target_position_units[j] == (int32_t)(index * 7U + j));
}

typedef struct {
    unsigned relay_calls;
    unsigned stop_calls;
    unsigned retract_calls;
} PhysicalCallbacks;

static bool physical_wire(bool enabled, void *context)
{
    PhysicalCallbacks *p = context;
    p->relay_calls += enabled ? 2U : 1U;
    return true;
}
static bool physical_prepare(void *context)
{
    PhysicalCallbacks *p = context;
    ++p->retract_calls;
    return true;
}
static StateStepResult physical_step(void *context)
{
    PhysicalCallbacks *p = context;
    ++p->retract_calls;
    return STATE_STEP_COMPLETE;
}
static bool physical_clearance(void *context)
{
    return context != NULL;
}
static bool physical_stop(bool *stopped, void *context)
{
    PhysicalCallbacks *p = context;
    ++p->stop_calls;
    *stopped = true;
    return true;
}

static void test_production_binding(void)
{
    static TrajectoryShared shared;
    static TrajectoryProducer producer;
    static TrajectoryExecutionBinding binding;
    TrajectoryIdentity id = {
        .program_id = 42U, .source_revision = 8U,
        .artifact_crc = UINT32_C(0x99887766),
        .sample_count = 513U,
        .sample_period_us = TRAJECTORY_SAMPLE_PERIOD_US
    };
    ValidatedTrajectory artifact = {0};
    artifact.program_id = id.program_id;
    artifact.source_revision = id.source_revision;
    artifact.artifact_crc = id.artifact_crc;
    artifact.sample_count = id.sample_count;
    artifact.sample_period_us = id.sample_period_us;
    PhysicalCallbacks physical = {0};
    PathExecutionServices board = {
        .set_wire_feed_enabled = physical_wire,
        .prepare_retraction = physical_prepare,
        .step_retraction = physical_step,
        .clearance_verified = physical_clearance,
        .controlled_stop = physical_stop,
        .prefill_timeout_ms = 1000U,
        .context = &physical
    };
    trajectory_shared_initialize(&shared);
    trajectory_m4_init(&producer, &shared);
    board.read_sample = trajectory_path_read_sample; /* Reject competing source. */
    assert(!trajectory_execution_bind(&binding, &shared, &artifact, &board));
    board.read_sample = NULL;
    assert(trajectory_execution_bind(&binding, &shared, &artifact, &board));
    PathExecutionServices *s = &binding.services;
    assert(s->context == &binding);
    assert(!s->stream_ready(0U, artifact.sample_count, s->context));
    assert(s->set_wire_feed_enabled(false, s->context));
    assert(s->prepare_retraction(s->context));
    assert(s->step_retraction(s->context) == STATE_STEP_COMPLETE);
    assert(s->clearance_verified(s->context));
    bool stopped = false;
    assert(s->controlled_stop(&stopped, s->context) && stopped);
    assert(physical.relay_calls == 1U && physical.stop_calls == 1U &&
           physical.retract_calls == 2U);
    /* Prepared but short of the required 512-sample prefill. */
    assert(trajectory_m4_prepare(&producer, source_read, NULL, &id));
    assert(trajectory_m4_step(&producer));
    assert(!s->stream_ready(0U, artifact.sample_count, s->context));
    assert(trajectory_m4_step(&producer));
    assert(s->stream_ready(0U, artifact.sample_count, s->context));
    PvExecutionSample sample = {0};
    assert(s->read_sample(0U, &sample, s->context));
    verify_sample(&sample, 0U);
    assert(s->stream_ready(1U, artifact.sample_count, s->context));
    /* Metadata changed while playback is active: fail, never use old ring. */
    artifact.artifact_crc ^= 1U;
    assert(!s->stream_ready(1U, artifact.sample_count, s->context));
    assert(!s->read_sample(1U, &sample, s->context));
    assert(!s->skip_samples(1U, 1U, s->context));
    artifact.artifact_crc ^= 1U;
    assert(s->skip_samples(1U, 2U, s->context));
    assert(s->read_sample(3U, &sample, s->context));
    verify_sample(&sample, 3U);
    s->end_stream(s->context);
    assert(!binding.consumer.active);
    assert(trajectory_m4_step(&producer));
    assert(trajectory_m4_release_acknowledged(&shared, binding.consumer.epoch));
    /* A second playback must publish a fresh epoch. */
    assert(!s->stream_ready(0U, artifact.sample_count, s->context));
    assert(trajectory_m4_prepare(&producer, source_read, NULL, &id));
    assert(trajectory_m4_step(&producer));
    assert(trajectory_m4_step(&producer));
    s->end_stream(s->context); /* Abort before consumer entered the new epoch. */
    assert(trajectory_m4_step(&producer));
    assert(trajectory_current_epoch(&shared) == 2U);
    assert(trajectory_m4_release_acknowledged(&shared, trajectory_current_epoch(&shared)));
    /* B4: attachment is explicit. An offline M4 never causes the
     * canonical M7 PathExecution callback to fabricate READY. */
    ValidationShared offline;
    validation_shared_initialize(&offline);
    TrajectoryExecutionBinding gated = {0};
    assert(trajectory_execution_bind(&gated, &shared, &artifact, &board));
    assert(trajectory_execution_attach_playback(&gated, &offline));
    assert(!trajectory_execution_attach_playback(&gated, &offline));
    assert(!gated.services.stream_ready(0U, artifact.sample_count,
                                        gated.services.context));
    assert(!gated.playback_requested);
    puts("PASS: B4 M7 playback attachment rejects absent M4 worker");
    puts("PASS: composed M7 stream identity, board callback context and release");
}

int main(void)
{
    static TrajectoryShared shared;
    static TrajectoryProducer m4;
    static TrajectoryConsumer m7;
    trajectory_shared_initialize(&shared);
    trajectory_m4_init(&m4, &shared);
    trajectory_m7_init(&m7, &shared);

    const TrajectoryIdentity id = {
        .program_id = 101U, .source_revision = 4U,
        .artifact_crc = UINT32_C(0x12345678),
        .sample_count = 513U,
        .sample_period_us = TRAJECTORY_SAMPLE_PERIOD_US
    };
    TrajectoryPathPort port = { .consumer = &m7, .identity = id };
    assert(!trajectory_path_stream_ready(0U, id.sample_count, &port));
    assert(!trajectory_path_stream_ready(0U, 512U, &port));
    assert(trajectory_m4_prepare(&m4, source_read, NULL, &id));
    assert(trajectory_m4_step(&m4));
    assert(!trajectory_path_stream_ready(0U, id.sample_count, &port));
    assert(trajectory_m4_step(&m4));
    assert(trajectory_path_stream_ready(0U, id.sample_count, &port));

    PvExecutionSample sample = {0};
    assert(!trajectory_path_read_sample(1U, &sample, &port));
    for (uint32_t i = 0U; i < id.sample_count; ++i) {
        if (trajectory_buffered(&shared) <= TRAJECTORY_BATCH)
            assert(trajectory_m4_step(&m4));
        assert(trajectory_path_read_sample(i, &sample, &port));
        verify_sample(&sample, i);
        if (i < id.sample_count - 1U)
            assert(trajectory_path_stream_ready(i + 1U, id.sample_count, &port));
    }
    assert(!trajectory_path_read_sample(id.sample_count, &sample, &port));
    trajectory_path_end_stream(&port);
    assert(!m7.active);
    assert(trajectory_m4_step(&m4));
    assert(trajectory_m4_release_acknowledged(&shared, m7.epoch));
    assert(!trajectory_path_stream_ready(0U, id.sample_count, &port));

    /* Same geometry/repeated playback uses a NEW stream epoch and prefill. */
    assert(trajectory_m4_prepare(&m4, source_read, NULL, &id));
    assert(trajectory_m4_step(&m4));
    assert(trajectory_m4_step(&m4));
    assert(trajectory_path_stream_ready(0U, id.sample_count, &port));
    assert(trajectory_path_read_sample(0U, &sample, &port));
    verify_sample(&sample, 0U);
    /* Constant-time skip preserves ordering and rejects invalid offsets. */
    assert(!trajectory_path_skip_samples(2U, 1U, &port));
    assert(!trajectory_path_skip_samples(1U, 512U, &port));
    assert(trajectory_path_skip_samples(1U, 2U, &port));
    assert(!trajectory_path_read_sample(2U, &sample, &port));
    assert(trajectory_path_read_sample(3U, &sample, &port));
    verify_sample(&sample, 3U);
    trajectory_path_end_stream(&port);
    assert(trajectory_m4_step(&m4));
    test_production_binding();
    puts("PASS: M7 PathExecution callbacks preflight, read, pause, release, replay");
    return 0;
}
