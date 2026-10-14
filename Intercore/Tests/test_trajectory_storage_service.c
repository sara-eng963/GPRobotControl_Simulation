/* B4 host-only end-to-end tests. A RAM fake is used ONLY inside this test.
 * Firmware board contracts never install or advertise this fake storage. */
#include "../trajectory_storage_service.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#define SAMPLE_COUNT 513U
#define CAPACITY 600U

typedef struct {
    GpTrajectorySample samples[CAPACITY];
    ValidatedTrajectory committed;
    uint32_t writes, reads, commits, aborts;
    uint32_t fail_read_at;
    bool open, durable, cancel_on_read;
    GpValidationShared *shared;
} FakeNor;

static FakeNor nor;
static GpValidationShared v;
static GpTrajectoryShared t;
static GpValidationM4Worker validation;
static GpValidationM7Client client;
static GpPlaybackM4Worker playback;
static GpTrajectoryConsumer m7;

static uint32_t crc(uint32_t state, const void *data, size_t n)
{
    const uint8_t *bytes = data;
    for (size_t i = 0; i < n; ++i) {
        state ^= bytes[i];
        for (unsigned j = 0; j < 8U; ++j)
            state = (state >> 1U) ^ (UINT32_C(0xEDB88320) &
                                      (uint32_t)-(int32_t)(state & 1U));
    }
    return state;
}
static PvExecutionSample sample(uint32_t i)
{
    PvExecutionSample s = {0};
    for (unsigned j = 0; j < 6U; ++j)
        s.target_position_units[j] = (int32_t)(i * 11U + j);
    return s;
}
static bool begin_write(void *c)
{
    FakeNor *n = c;
    if (n->open) return false;
    n->open = true;
    n->durable = false;
    n->writes = 0U;
    return true;
}
static bool write_one(uint32_t i, const PvExecutionSample *s, void *c)
{
    FakeNor *n = c;
    if (!n->open || i != n->writes || i >= CAPACITY) return false;
    memcpy(&n->samples[i], s, sizeof(*s));
    ++n->writes;
    return true;
}
static bool commit_write(const ValidatedTrajectory *meta, void *c)
{
    FakeNor *n = c;
    if (!n->open || meta->sample_count != n->writes) return false;
    n->committed = *meta;
    n->open = false;
    n->durable = true;
    ++n->commits;
    return true;
}
static void abort_write(void *c)
{
    FakeNor *n = c;
    n->open = false;
    n->durable = false;
    ++n->aborts;
}
static bool read_metadata(ValidatedTrajectory *meta, void *c)
{
    const FakeNor *n = c;
    if (!n->durable) return false;
    *meta = n->committed;
    return true;
}
static bool read_samples(uint32_t i, GpTrajectorySample *out,
                         uint32_t count, void *c)
{
    FakeNor *n = c;
    if (!n->durable || count == 0U || count > GP_TRAJ_BATCH ||
        i >= n->fail_read_at || i + count > n->committed.sample_count)
        return false;
    memcpy(out, &n->samples[i], (size_t)count * sizeof(*out));
    ++n->reads;
    if (n->cancel_on_read) {
        n->cancel_on_read = false;
        gp_playback_m7_cancel(n->shared);
    }
    return true;
}
static void init(void)
{
    memset(&nor, 0, sizeof(nor));
    nor.fail_read_at = UINT32_MAX;
    nor.shared = &v;
    gp_validation_shared_initialize(&v);
    gp_traj_shared_initialize(&t);
    gp_validation_m7_init(&client, &v);
    gp_traj_m7_init(&m7, &t);
    PathValidationStorage backend = {
        .begin = begin_write, .write_sample = write_one,
        .commit = commit_write, .abort = abort_write,
        .capacity_samples = CAPACITY, .context = &nor
    };
    GpCommittedTrajectoryReader reader = {
        .read_metadata = read_metadata, .read_samples = read_samples,
        .capacity_samples = CAPACITY, .context = &nor
    };
    assert(gp_playback_m4_init(&playback, &v, &t, &reader));
    assert(gp_validation_m4_init(&validation, &v, &backend));
}
static GpTrajectoryIdentity identity(void)
{
    return (GpTrajectoryIdentity) {
        .program_id = 17U, .source_revision = 2U,
        .artifact_crc = UINT32_C(0xDEADBEEF),
        .sample_count = SAMPLE_COUNT,
        .sample_period_us = GP_TRAJ_SAMPLE_PERIOD_US
    };
}
static void make_durable_artifact(void)
{
    PathValidationStorage proxy = gp_validation_m7_storage(&client, CAPACITY);
    assert(proxy.begin(proxy.context));
    uint32_t checksum = UINT32_MAX;
    for (uint32_t i = 0U; i < SAMPLE_COUNT; ++i) {
        PvExecutionSample s = sample(i);
        checksum = crc(checksum, &s, sizeof(s));
        PvStorageTryResult result = proxy.try_write_sample(i, &s, proxy.context);
        for (uint32_t spin = 0; result == PV_STORAGE_TRY_BUSY &&
                                spin < 100U; ++spin) {
            assert(gp_validation_m4_step(&validation));
            result = proxy.try_write_sample(i, &s, proxy.context);
        }
        assert(result == PV_STORAGE_TRY_ACCEPTED);
    }
    GpTrajectoryIdentity id = identity();
    ValidatedTrajectory meta = {0};
    meta.program_id = id.program_id;
    meta.source_revision = id.source_revision;
    meta.artifact_crc = id.artifact_crc;
    meta.sample_count = id.sample_count;
    meta.sample_period_us = id.sample_period_us;
    meta.sample_data_crc = ~checksum;
    PvStorageTryResult result = proxy.try_commit(&meta, proxy.context);
    for (uint32_t spin = 0; result == PV_STORAGE_TRY_BUSY &&
                            spin < 1000U; ++spin) {
        assert(gp_validation_m4_step(&validation));
        result = proxy.try_commit(&meta, proxy.context);
    }
    assert(result == PV_STORAGE_TRY_ACCEPTED);
    assert(gp_validation_m4_step(&validation)); /* release committed epoch */
    assert(gp_validation_m4_idle(&v));
    assert(nor.durable && nor.commits == 1U && nor.writes == SAMPLE_COUNT);
}

static void scan_and_prefill(void)
{
    for (uint32_t i = 0U; i < 30U &&
        __atomic_load_n(&v.playback_status, __ATOMIC_ACQUIRE) !=
            GP_PLAYBACK_STREAMING; ++i)
        assert(gp_playback_m4_step(&playback));
    assert(v.playback_status == GP_PLAYBACK_STREAMING);
    assert(gp_playback_m4_step(&playback));
    assert(gp_playback_m4_step(&playback));
    assert(gp_traj_buffered(&t) == GP_TRAJ_CAPACITY);
}

static void end_stream(void)
{
    assert(gp_traj_m7_finish(&m7));
    gp_playback_m7_cancel(&v);
    assert(gp_playback_m4_step(&playback));
    assert(v.playback_status == GP_PLAYBACK_IDLE);
    assert(gp_traj_m4_release_acknowledged(&t, m7.epoch));
}

static void test_committed_readback_and_replay(void)
{
    init();
    assert(!gp_playback_m7_request(&v, &t, &(GpTrajectoryIdentity){0}));
    assert(!gp_traj_m7_begin(&m7, &(GpTrajectoryIdentity){0}));
    make_durable_artifact();
    GpTrajectoryIdentity id = identity();
    assert(gp_playback_m7_request(&v, &t, &id));
    assert(!gp_playback_m7_request(&v, &t, &id)); /* single outstanding */
    assert(!gp_traj_m7_begin(&m7, &id));
    scan_and_prefill();
    assert(gp_traj_m7_begin(&m7, &id));
    GpTrajectorySample received;
    for (uint32_t i = 0U; i < SAMPLE_COUNT; ++i) {
        if (i == GP_TRAJ_BATCH) assert(gp_playback_m4_step(&playback));
        assert(gp_traj_m7_take(&m7, i, &received) == GP_TRAJ_READ_OK);
        PvExecutionSample expected = sample(i);
        assert(memcmp(&received, &expected, sizeof(received)) == 0);
    }
    assert(playback.producer.refills == 3U);
    end_stream();
    uint32_t first_epoch = m7.epoch;
    assert(gp_playback_m7_request(&v, &t, &id));
    scan_and_prefill();
    assert(gp_traj_m7_begin(&m7, &id) && m7.epoch != first_epoch);
    end_stream();
    puts("[PASS] M7 durable write -> M4 CRC scan -> ring prefill/refill -> replay");
}

static void test_corruption_and_read_failure(void)
{
    init();
    make_durable_artifact();
    GpTrajectoryIdentity id = identity();
    nor.samples[100].target_position_units[0] ^= 1234;
    assert(gp_playback_m7_request(&v, &t, &id));
    assert(gp_playback_m4_step(&playback));
    while (v.playback_status == GP_PLAYBACK_VERIFYING)
        (void)gp_playback_m4_step(&playback);
    assert(v.playback_status == GP_PLAYBACK_FAILED);
    assert(t.ready == 0U && !gp_traj_m7_begin(&m7, &id));
    gp_playback_m7_cancel(&v);
    assert(gp_playback_m4_step(&playback));
    assert(v.playback_status == GP_PLAYBACK_IDLE);
    nor.samples[100].target_position_units[0] ^= 1234;
    nor.fail_read_at = 256U;
    assert(gp_playback_m7_request(&v, &t, &id));
    assert(gp_playback_m4_step(&playback));
    assert(gp_playback_m4_step(&playback));
    assert(!gp_playback_m4_step(&playback));
    assert(v.playback_status == GP_PLAYBACK_FAILED && t.ready == 0U);
    gp_playback_m7_cancel(&v);
    assert(gp_playback_m4_step(&playback));
    puts("[PASS] CRC mismatch and NOR read errors never publish a stream");
}

static void test_cancel_during_readback(void)
{
    init();
    make_durable_artifact();
    GpTrajectoryIdentity id = identity();
    assert(gp_playback_m7_request(&v, &t, &id));
    assert(gp_playback_m4_step(&playback));
    nor.cancel_on_read = true;
    assert(gp_playback_m4_step(&playback));
    assert(gp_playback_m4_step(&playback));
    assert(v.playback_status == GP_PLAYBACK_IDLE);
    assert(t.ready == 0U);
    assert(!gp_traj_m7_begin(&m7, &id));
    puts("[PASS] cancellation during blocking read prevents stale publication");
}

static void test_metadata_mismatch_and_uncommissioned(void)
{
    init();
    assert(!gp_playback_m7_request(&v, &t, &(GpTrajectoryIdentity){0}));
    make_durable_artifact();
    GpTrajectoryIdentity id = identity();
    id.artifact_crc ^= 1U;
    assert(gp_playback_m7_request(&v, &t, &id));
    assert(!gp_playback_m4_step(&playback));
    assert(v.playback_status == GP_PLAYBACK_FAILED && t.ready == 0U);
    gp_playback_m7_cancel(&v);
    assert(gp_playback_m4_step(&playback));
    gp_validation_m4_shutdown(&validation);
    id = identity();
    assert(!gp_playback_m7_request(&v, &t, &id));
    puts("[PASS] identity mismatch and missing backend fail closed");
}

int main(void)
{
    test_committed_readback_and_replay();
    test_corruption_and_read_failure();
    test_cancel_during_readback();
    test_metadata_mismatch_and_uncommissioned();
    puts("PASS: B4 M4 storage replay host-only regression");
    return 0;
}
