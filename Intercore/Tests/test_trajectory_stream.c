/* Concurrent M4 producer / M7 consumer test for non-cacheable shared-SRAM
 * protocol. Host timing and caches do not represent STM32 hardware.
 */
#include "../trajectory_stream.h"
#include <assert.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>

typedef struct {
    uint32_t fail_at;
    uint32_t calls;
    uint32_t max_batch;
    pthread_mutex_t mutex;
    pthread_cond_t condition;
    uint32_t block_at;
    bool entered, released;
} Source;

typedef struct {
    GpTrajectoryShared shared;
    GpTrajectoryProducer m4;
    GpTrajectoryConsumer m7;
    Source source;
} Test;

static int32_t expected(uint32_t index, unsigned joint)
{
    const int32_t v = (int32_t)(1000U + index * 97U + joint * 13U);
    return (joint % 2U) ? -v : v;
}

static bool read_flash(uint32_t first, GpTrajectorySample *out,
                       uint32_t count, void *context)
{
    Source *source = context;
    assert(pthread_mutex_lock(&source->mutex) == 0);
    ++source->calls;
    if (count > source->max_batch) source->max_batch = count;
    if (first == source->block_at) {
        source->entered = true;
        assert(pthread_cond_broadcast(&source->condition) == 0);
        while (!source->released)
            assert(pthread_cond_wait(&source->condition,
                                     &source->mutex) == 0);
    }
    assert(pthread_mutex_unlock(&source->mutex) == 0);
    if (first >= source->fail_at) return false;
    for (uint32_t i = 0; i < count; ++i)
        for (unsigned j = 0; j < 6U; ++j)
            out[i].target_position_units[j] = expected(first + i, j);
    return true;
}

static GpTrajectoryIdentity identity(uint32_t id, uint32_t count)
{
    const GpTrajectoryIdentity result = {
        .program_id = id, .source_revision = 3U,
        .artifact_crc = UINT32_C(0x98765432),
        .sample_count = count,
        .sample_period_us = GP_TRAJ_SAMPLE_PERIOD_US
    };
    return result;
}

static void init(Test *t)
{
    memset(t, 0, sizeof(*t));
    assert(pthread_mutex_init(&t->source.mutex, NULL) == 0);
    assert(pthread_cond_init(&t->source.condition, NULL) == 0);
    t->source.fail_at = UINT32_MAX;
    t->source.block_at = UINT32_MAX;
    gp_traj_shared_initialize(&t->shared);
    gp_traj_m4_init(&t->m4, &t->shared);
    gp_traj_m7_init(&t->m7, &t->shared);
    assert(gp_traj_shared_valid(&t->shared));
}

static void deinit(Test *t)
{
    assert(pthread_cond_destroy(&t->source.condition) == 0);
    assert(pthread_mutex_destroy(&t->source.mutex) == 0);
}

static void prefill(Test *t, const GpTrajectoryIdentity *id)
{
    assert(gp_traj_m4_prepare(&t->m4, read_flash, &t->source, id));
    assert(!gp_traj_m7_begin(&t->m7, id));
    assert(gp_traj_m4_step(&t->m4));
    if (id->sample_count > 256U) assert(gp_traj_m4_step(&t->m4));
    assert(gp_traj_buffered(&t->shared) ==
           (id->sample_count < 512U ? id->sample_count : 512U));
    assert(gp_traj_m7_begin(&t->m7, id));
}

static void verify(const GpTrajectorySample *s, uint32_t index)
{
    for (unsigned j = 0; j < 6U; ++j)
        assert(s->target_position_units[j] == expected(index, j));
}

static void test_basic_and_replay(void)
{
    static Test t;
    init(&t);
    GpTrajectoryIdentity id = identity(11U, 1536U);
    prefill(&t, &id);
    const uint32_t first_epoch = t.m7.epoch;
    assert(!gp_traj_m4_prepare(&t.m4, read_flash, &t.source, &id));
    GpTrajectorySample value;
    assert(gp_traj_m7_take(&t.m7, 1U, &value) == GP_TRAJ_READ_INVALID_INDEX);
    for (uint32_t index = 0U; index < id.sample_count; ++index) {
        if (gp_traj_buffered(&t.shared) <= GP_TRAJ_BATCH)
            assert(gp_traj_m4_step(&t.m4));
        assert(gp_traj_m7_take(&t.m7, index, &value) == GP_TRAJ_READ_OK);
        verify(&value, index);
    }
    assert(t.m4.refills == 6U);
    assert(t.m7.underruns == 0U);
    assert(t.source.max_batch <= GP_TRAJ_BATCH);
    assert(gp_traj_buffered(&t.shared) == 0U);
    assert(gp_traj_m7_finish(&t.m7));
    assert(!gp_traj_m4_prepare(&t.m4, read_flash, &t.source, &id));
    assert(!gp_traj_m4_release_acknowledged(&t.shared, first_epoch));
    assert(gp_traj_m4_step(&t.m4));
    assert(gp_traj_m4_release_acknowledged(&t.shared, first_epoch));
    id = identity(12U, 3U);
    prefill(&t, &id);
    assert(t.m7.epoch != first_epoch);
    for (uint32_t i = 0U; i < 3U; ++i) {
        assert(gp_traj_m7_take(&t.m7, i, &value) == GP_TRAJ_READ_OK);
        verify(&value, i);
    }
    assert(gp_traj_m7_take(&t.m7, 3U, &value) == GP_TRAJ_READ_INVALID_INDEX);
    assert(gp_traj_m7_finish(&t.m7));
    assert(gp_traj_m4_step(&t.m4));
    deinit(&t);
    puts("[PASS] prefill 512, six refills, ordered six-joint values, replay new epoch");
}

static void test_starvation_and_flash_error(void)
{
    static Test t;
    init(&t);
    GpTrajectoryIdentity id = identity(21U, 1024U);
    prefill(&t, &id);
    GpTrajectorySample value;
    for (uint32_t i = 0U; i < 512U; ++i)
        assert(gp_traj_m7_take(&t.m7, i, &value) == GP_TRAJ_READ_OK);
    assert(gp_traj_m7_take(&t.m7, 512U, &value) == GP_TRAJ_READ_STARVED);
    assert(t.m7.underruns == 1U && t.m7.next_index == 512U);
    assert(gp_traj_m7_finish(&t.m7));
    assert(gp_traj_m4_step(&t.m4));
    t.source.fail_at = 512U;
    prefill(&t, &id);
    for (uint32_t i = 0U; i < 256U; ++i)
        assert(gp_traj_m7_take(&t.m7, i, &value) == GP_TRAJ_READ_OK);
    assert(!gp_traj_m4_step(&t.m4));
    assert(t.shared.read_error != 0U);
    assert(gp_traj_m7_take(&t.m7, 256U, &value) ==
           GP_TRAJ_READ_STORAGE_ERROR);
    assert(t.m7.next_index == 256U);
    assert(gp_traj_m7_finish(&t.m7));
    assert(gp_traj_m4_step(&t.m4));
    deinit(&t);
    puts("[PASS] starvation and injected flash failure fail closed without advancing");
}

typedef struct { GpTrajectoryProducer *producer; bool ok; } Worker;
static void *worker_once(void *argument)
{
    Worker *worker = argument;
    worker->ok = gp_traj_m4_step(worker->producer);
    return NULL;
}

static void test_cancel_in_flight(void)
{
    static Test t;
    init(&t);
    GpTrajectoryIdentity id = identity(31U, 1024U);
    prefill(&t, &id);
    for (uint32_t i = 0; i < 256U; ++i) {
        GpTrajectorySample value;
        assert(gp_traj_m7_take(&t.m7, i, &value) == GP_TRAJ_READ_OK);
    }
    assert(pthread_mutex_lock(&t.source.mutex) == 0);
    t.source.block_at = 512U;
    assert(pthread_mutex_unlock(&t.source.mutex) == 0);
    Worker worker = { .producer = &t.m4 };
    pthread_t thread;
    assert(pthread_create(&thread, NULL, worker_once, &worker) == 0);
    assert(pthread_mutex_lock(&t.source.mutex) == 0);
    while (!t.source.entered)
        assert(pthread_cond_wait(&t.source.condition, &t.source.mutex) == 0);
    assert(pthread_mutex_unlock(&t.source.mutex) == 0);
    const uint32_t epoch = t.m7.epoch;
    assert(gp_traj_m7_finish(&t.m7));
    assert(!gp_traj_m4_release_acknowledged(&t.shared, epoch));
    assert(!gp_traj_m4_prepare(&t.m4, read_flash, &t.source, &id));
    assert(pthread_mutex_lock(&t.source.mutex) == 0);
    t.source.released = true;
    assert(pthread_cond_broadcast(&t.source.condition) == 0);
    assert(pthread_mutex_unlock(&t.source.mutex) == 0);
    assert(pthread_join(thread, NULL) == 0);
    assert(worker.ok);
    assert(gp_traj_m4_release_acknowledged(&t.shared, epoch));
    assert(t.m4.next_index == 512U); /* abandoned read not published */
    assert(gp_traj_buffered(&t.shared) == 256U);
    t.source.block_at = UINT32_MAX;
    id = identity(32U, 6U);
    prefill(&t, &id);
    GpTrajectorySample value;
    assert(gp_traj_m7_take(&t.m7, 0U, &value) == GP_TRAJ_READ_OK);
    verify(&value, 0U);
    assert(gp_traj_m7_finish(&t.m7));
    assert(gp_traj_m4_step(&t.m4));
    deinit(&t);
    puts("[PASS] in-flight flash read cannot ack release or publish stale batch");
}

static void test_discard_and_identity(void)
{
    static Test t;
    init(&t);
    GpTrajectoryIdentity id = identity(41U, 600U);
    assert(gp_traj_m4_prepare(&t.m4, read_flash, &t.source, &id));
    assert(gp_traj_m4_step(&t.m4));
    assert(gp_traj_m7_discard(&t.m7));
    assert(gp_traj_m4_step(&t.m4));
    assert(gp_traj_m4_release_acknowledged(&t.shared, 1U));
    assert(!gp_traj_m7_begin(&t.m7, &id));
    id = identity(42U, 513U);
    assert(gp_traj_m4_prepare(&t.m4, read_flash, &t.source, &id));
    assert(gp_traj_m4_step(&t.m4));
    GpTrajectoryIdentity wrong = id;
    wrong.artifact_crc++;
    assert(!gp_traj_m7_begin(&t.m7, &wrong));
    assert(!gp_traj_m7_begin(&t.m7, &id)); /* only half prefetched */
    assert(gp_traj_m4_step(&t.m4));
    assert(gp_traj_m7_begin(&t.m7, &id));
    assert(gp_traj_m7_finish(&t.m7));
    assert(gp_traj_m4_step(&t.m4));
    deinit(&t);
    puts("[PASS] discard handshake, full preflight, mismatched artifact rejected");
}

int main(void)
{
    test_basic_and_replay();
    test_starvation_and_flash_error();
    test_cancel_in_flight();
    test_discard_and_identity();
    puts("PASS: dual-core trajectory stream host regression");
    return 0;
}
