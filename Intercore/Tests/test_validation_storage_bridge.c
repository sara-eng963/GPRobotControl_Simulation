/* M7 compute task -> M4 flash task, no shared FreeRTOS mutex.
 * Protocol test on POSIX threads only; no STM32 MPU/flash deadline claim.
 */
#include "../validation_storage_bridge.h"
#include <assert.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>

typedef struct {
    uint32_t next;
    uint32_t fail_at;
    uint32_t aborts;
    uint32_t begins;
    uint32_t commits;
    bool open;
    bool durable;
    pthread_mutex_t mutex;
    pthread_cond_t changed;
    bool hold_begin;
    bool begin_entered;
    bool begin_released;
} FakeFlash;

static bool begin(void *ctx)
{
    FakeFlash *f = ctx;
    assert(pthread_mutex_lock(&f->mutex) == 0);
    if (f->hold_begin) {
        f->begin_entered = true;
        assert(pthread_cond_broadcast(&f->changed) == 0);
        while (!f->begin_released)
            assert(pthread_cond_wait(&f->changed, &f->mutex) == 0);
    }
    assert(pthread_mutex_unlock(&f->mutex) == 0);
    f->open = true;
    f->durable = false;
    f->next = 0U;
    f->begins++;
    return true;
}

static bool write_sample(uint32_t index, const PvExecutionSample *sample,
                         void *ctx)
{
    FakeFlash *f = ctx;
    if (!f->open || index != f->next || index >= f->fail_at)
        return false;
    for (uint32_t axis = 0; axis < 6U; ++axis)
        assert(sample->target_position_units[axis] ==
               (int32_t)(index * 11U + axis));
    ++f->next;
    return true;
}

static bool commit(const ValidatedTrajectory *metadata, void *ctx)
{
    FakeFlash *f = ctx;
    if (!f->open || metadata->sample_count != f->next ||
        metadata->sample_period_us != PATH_VALIDATION_SAMPLE_PERIOD_US)
        return false;
    f->open = false;
    f->durable = true;
    ++f->commits;
    return true;
}

static void abort_flash(void *ctx)
{
    FakeFlash *f = ctx;
    f->open = false;
    f->durable = false;
    ++f->aborts;
}

static GpValidationShared shared;
static GpValidationM7Client m7;
static GpValidationM4Worker m4;
static FakeFlash flash;
static PathValidationStorage proxy;

static void reset(void)
{
    memset(&flash, 0, sizeof(flash));
    assert(pthread_mutex_init(&flash.mutex, NULL) == 0);
    assert(pthread_cond_init(&flash.changed, NULL) == 0);
    flash.fail_at = UINT32_MAX;
    gp_validation_shared_initialize(&shared);
    gp_validation_m7_init(&m7, &shared);
    PathValidationStorage backend = {0};
    backend.begin = begin;
    backend.write_sample = write_sample;
    backend.commit = commit;
    backend.abort = abort_flash;
    backend.capacity_samples = 10000U;
    backend.context = &flash;
    assert(gp_validation_m4_init(&m4, &shared, &backend));
    proxy = gp_validation_m7_storage(&m7, 10000U);
}

static void cleanup(void)
{
    assert(pthread_cond_destroy(&flash.changed) == 0);
    assert(pthread_mutex_destroy(&flash.mutex) == 0);
}

static PvExecutionSample sample_at(uint32_t index)
{
    PvExecutionSample s = {0};
    for (uint32_t j = 0U; j < 6U; ++j)
        s.target_position_units[j] = (int32_t)(index * 11U + j);
    return s;
}

static void test_queue_backpressure_commit(void)
{
    reset();
    assert(gp_validation_m4_idle(&shared));
    assert(proxy.begin(proxy.context));
    assert(!gp_validation_m4_idle(&shared));
    for (uint32_t index = 0U;
         index < GP_VALIDATION_QUEUE_CAPACITY - 1U; ++index) {
        const PvExecutionSample sample = sample_at(index);
        assert(proxy.try_write_sample(index, &sample, proxy.context) ==
               PV_STORAGE_TRY_ACCEPTED);
    }
    const uint32_t retry_index = GP_VALIDATION_QUEUE_CAPACITY - 1U;
    const PvExecutionSample retry = sample_at(retry_index);
    assert(proxy.try_write_sample(retry_index, &retry, proxy.context) ==
           PV_STORAGE_TRY_BUSY);
    assert(m7.next_index == retry_index);
    assert(gp_validation_m4_step(&m4)); /* backend BEGIN */
    assert(proxy.try_write_sample(retry_index, &retry, proxy.context) ==
           PV_STORAGE_TRY_ACCEPTED);
    for (uint32_t i = 0; i < GP_VALIDATION_QUEUE_CAPACITY; ++i)
        assert(gp_validation_m4_step(&m4));
    assert(flash.next == GP_VALIDATION_QUEUE_CAPACITY);
    ValidatedTrajectory metadata = {0};
    metadata.program_id = 99U;
    metadata.sample_count = GP_VALIDATION_QUEUE_CAPACITY;
    metadata.sample_period_us = PATH_VALIDATION_SAMPLE_PERIOD_US;
    assert(proxy.try_commit(&metadata, proxy.context) == PV_STORAGE_TRY_BUSY);
    assert(!flash.durable && flash.commits == 0U);
    assert(!gp_validation_m4_idle(&shared));
    assert(gp_validation_m4_step(&m4)); /* backend durable COMMIT */
    assert(flash.durable && flash.commits == 1U);
    assert(proxy.try_commit(&metadata, proxy.context) ==
           PV_STORAGE_TRY_ACCEPTED);
    assert(!gp_validation_m4_idle(&shared));
    assert(gp_validation_m4_step(&m4)); /* release ACK */
    assert(gp_validation_m4_idle(&shared));
    assert(proxy.begin(proxy.context)); /* new generation permitted */
    proxy.abort(proxy.context);
    assert(!gp_validation_m4_idle(&shared));
    assert(gp_validation_m4_step(&m4)); /* cancel queued begin */
    assert(gp_validation_m4_idle(&shared));
    cleanup();
    puts("[PASS] queue FULL yields BUSY, no dropped samples, VALID after durable commit");
}

static void test_storage_failure(void)
{
    reset();
    flash.fail_at = 3U;
    assert(proxy.begin(proxy.context));
    assert(gp_validation_m4_step(&m4));
    for (uint32_t index = 0U; index < 6U; ++index) {
        const PvExecutionSample sample = sample_at(index);
        assert(proxy.try_write_sample(index, &sample, proxy.context) ==
               PV_STORAGE_TRY_ACCEPTED);
    }
    for (unsigned n = 0; n < 3U; ++n)
        assert(gp_validation_m4_step(&m4));
    assert(!gp_validation_m4_step(&m4));
    assert(flash.aborts == 1U);
    PvExecutionSample sample = sample_at(6U);
    assert(proxy.try_write_sample(6U, &sample, proxy.context) ==
           PV_STORAGE_TRY_FAILED);
    proxy.abort(proxy.context);
    assert(!gp_validation_m4_idle(&shared));
    assert(gp_validation_m4_step(&m4));
    assert(gp_validation_m4_idle(&shared));
    assert(!flash.durable);
    cleanup();
    puts("[PASS] M4 flash error propagates to M7, abort drains stale queue");
}

static bool in_flight_step_ok;
static void *in_flight_begin(void *unused)
{
    (void)unused;
    in_flight_step_ok = gp_validation_m4_step(&m4);
    return NULL;
}

static void test_cancel_in_flight(void)
{
    reset();
    flash.hold_begin = true;
    assert(proxy.begin(proxy.context));
    pthread_t thread;
    assert(pthread_create(&thread, NULL, in_flight_begin, NULL) == 0);
    assert(pthread_mutex_lock(&flash.mutex) == 0);
    while (!flash.begin_entered)
        assert(pthread_cond_wait(&flash.changed, &flash.mutex) == 0);
    assert(pthread_mutex_unlock(&flash.mutex) == 0);
    proxy.abort(proxy.context); /* nonblocking M7 cancellation */
    assert(!gp_validation_m4_idle(&shared));
    assert(!proxy.begin(proxy.context));
    assert(pthread_mutex_lock(&flash.mutex) == 0);
    flash.begin_released = true;
    assert(pthread_cond_broadcast(&flash.changed) == 0);
    assert(pthread_mutex_unlock(&flash.mutex) == 0);
    assert(pthread_join(thread, NULL) == 0);
    assert(in_flight_step_ok);
    assert(!gp_validation_m4_idle(&shared)); /* no premature acknowledgement */
    assert(gp_validation_m4_step(&m4)); /* abort after flash callback returned */
    assert(gp_validation_m4_idle(&shared));
    assert(flash.aborts == 1U && !flash.open);
    cleanup();
    puts("[PASS] cancel does not reset an in-flight flash transaction");
}

static void test_offline_m4_gate(void)
{
    static GpValidationShared offline;
    GpValidationM7Client client;
    gp_validation_shared_initialize(&offline);
    gp_validation_m7_init(&client, &offline);
    PathValidationStorage proxy_offline = gp_validation_m7_storage(&client, 100U);
    assert(gp_validation_shared_valid(&offline));
    assert(!gp_validation_m4_idle(&offline));
    assert(!proxy_offline.begin(proxy_offline.context));
    assert(!client.active && offline.head == offline.tail);
    reset();
    GpValidationM4Worker duplicate;
    PathValidationStorage backend = m4.backend;
    assert(!gp_validation_m4_init(&duplicate, &shared, &backend));
    assert(proxy.begin(proxy.context));
    assert(gp_validation_m4_step(&m4));
    gp_validation_m4_shutdown(&m4);
    assert(!gp_validation_m4_idle(&shared));
    const PvExecutionSample s = sample_at(0U);
    assert(proxy.try_write_sample(0U, &s, proxy.context) ==
           PV_STORAGE_TRY_FAILED);
    assert(!gp_validation_m4_step(&m4));
    cleanup();
    puts("[PASS] absent or shut-down M4 backend cannot authorize validation");
}

int main(void)
{
    test_offline_m4_gate();
    test_queue_backpressure_commit();
    test_storage_failure();
    test_cancel_in_flight();
    puts("PASS: M7/M4 asynchronous validation storage bridge");
    return 0;
}
