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
    gp_traj_path_end_stream(&port);
    assert(gp_traj_m4_step(&m4));
    puts("PASS: M7 PathExecution callbacks preflight, read, pause, release, replay");
    return 0;
}
