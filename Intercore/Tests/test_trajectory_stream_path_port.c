/* PathExecutionServices-compatible M7 callback regression.
 * Runs independently from the legacy single-core prefetch implementation.
 */
#include "../trajectory_stream_path_port.h"
#include <assert.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

static bool source_read(uint32_t first, GpTrajectorySample *out,
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
    static GpTrajectoryShared shared;
    static GpTrajectoryProducer producer;
    static GpTrajectoryExecutionBinding binding;
    GpTrajectoryIdentity id = {
        .program_id = 42U, .source_revision = 8U,
        .artifact_crc = UINT32_C(0x99887766),
        .sample_count = 513U,
        .sample_period_us = GP_TRAJ_SAMPLE_PERIOD_US
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
        .context = &physical
    };
    gp_traj_shared_initialize(&shared);
    gp_traj_m4_init(&producer, &shared);
    board.read_sample = gp_traj_path_read_sample; /* Reject competing source. */
    assert(!gp_traj_execution_bind(&binding, &shared, &artifact, &board));
    board.read_sample = NULL;
    assert(gp_traj_execution_bind(&binding, &shared, &artifact, &board));
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
    assert(gp_traj_m4_prepare(&producer, source_read, NULL, &id));
    assert(gp_traj_m4_step(&producer));
    assert(!s->stream_ready(0U, artifact.sample_count, s->context));
    assert(gp_traj_m4_step(&producer));
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
    assert(gp_traj_m4_step(&producer));
    assert(gp_traj_m4_release_acknowledged(&shared, binding.consumer.epoch));
    /* A second playback must publish a fresh epoch. */
    assert(!s->stream_ready(0U, artifact.sample_count, s->context));
    assert(gp_traj_m4_prepare(&producer, source_read, NULL, &id));
    assert(gp_traj_m4_step(&producer));
    assert(gp_traj_m4_step(&producer));
    s->end_stream(s->context); /* Abort before consumer entered the new epoch. */
    assert(gp_traj_m4_step(&producer));
    assert(gp_traj_m4_release_acknowledged(&shared, id.sample_count ? 2U : 0U));
    puts("PASS: composed M7 stream identity, board callback context and release");
}

int main(void)
{
    static GpTrajectoryShared shared;
    static GpTrajectoryProducer m4;
    static GpTrajectoryConsumer m7;
    gp_traj_shared_initialize(&shared);
    gp_traj_m4_init(&m4, &shared);
    gp_traj_m7_init(&m7, &shared);

    const GpTrajectoryIdentity id = {
        .program_id = 101U, .source_revision = 4U,
        .artifact_crc = UINT32_C(0x12345678),
        .sample_count = 513U,
        .sample_period_us = GP_TRAJ_SAMPLE_PERIOD_US
    };
    GpTrajectoryPathPort port = { .consumer = &m7, .identity = id };
    assert(!gp_traj_path_stream_ready(0U, id.sample_count, &port));
    assert(!gp_traj_path_stream_ready(0U, 512U, &port));
    assert(gp_traj_m4_prepare(&m4, source_read, NULL, &id));
    assert(gp_traj_m4_step(&m4));
    assert(!gp_traj_path_stream_ready(0U, id.sample_count, &port));
    assert(gp_traj_m4_step(&m4));
    assert(gp_traj_path_stream_ready(0U, id.sample_count, &port));

    PvExecutionSample sample = {0};
    assert(!gp_traj_path_read_sample(1U, &sample, &port));
    for (uint32_t i = 0U; i < id.sample_count; ++i) {
        if (gp_traj_buffered(&shared) <= GP_TRAJ_BATCH)
            assert(gp_traj_m4_step(&m4));
        assert(gp_traj_path_read_sample(i, &sample, &port));
        verify_sample(&sample, i);
        if (i < id.sample_count - 1U)
            assert(gp_traj_path_stream_ready(i + 1U, id.sample_count, &port));
    }
    assert(!gp_traj_path_read_sample(id.sample_count, &sample, &port));
    gp_traj_path_end_stream(&port);
    assert(!m7.active);
    assert(gp_traj_m4_step(&m4));
    assert(gp_traj_m4_release_acknowledged(&shared, m7.epoch));
    assert(!gp_traj_path_stream_ready(0U, id.sample_count, &port));

    /* Same geometry/repeated playback uses a NEW stream epoch and prefill. */
    assert(gp_traj_m4_prepare(&m4, source_read, NULL, &id));
    assert(gp_traj_m4_step(&m4));
    assert(gp_traj_m4_step(&m4));
    assert(gp_traj_path_stream_ready(0U, id.sample_count, &port));
    assert(gp_traj_path_read_sample(0U, &sample, &port));
    verify_sample(&sample, 0U);
    /* Constant-time skip preserves ordering and rejects invalid offsets. */
    assert(!gp_traj_path_skip_samples(2U, 1U, &port));
    assert(!gp_traj_path_skip_samples(1U, 512U, &port));
    assert(gp_traj_path_skip_samples(1U, 2U, &port));
    assert(!gp_traj_path_read_sample(2U, &sample, &port));
    assert(gp_traj_path_read_sample(3U, &sample, &port));
    verify_sample(&sample, 3U);
    gp_traj_path_end_stream(&port);
    assert(gp_traj_m4_step(&m4));
    test_production_binding();
    puts("PASS: M7 PathExecution callbacks preflight, read, pause, release, replay");
    return 0;
}
