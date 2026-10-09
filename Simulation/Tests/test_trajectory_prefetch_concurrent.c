/* Host concurrency evidence, not an STM32 scheduling/timing guarantee.
 * Condition variables schedule real producer/consumer interleavings without
 * sleeps. Storage callbacks can be held in flight until the consumer releases.
 */
#include "../Storage/trajectory_prefetch.h"
#include <assert.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <string.h>
#include <limits.h>

typedef struct {
    pthread_mutex_t mutex;
    pthread_cond_t changed;
    uint32_t block_at, fail_at, bias, calls;
    bool entered, released;
} Source;

static int32_t joint_value(uint32_t index, unsigned joint, uint32_t bias)
{
    int32_t v = (int32_t)(index * 97U + joint * 13U + bias);
    return joint % 2U ? -v : v;
}

static bool read_batch(uint32_t first, PvExecutionSample *out,
                       uint32_t count, void *context)
{
    Source *src = context;
    assert(pthread_mutex_lock(&src->mutex) == 0);
    ++src->calls;
    if (first == src->block_at) {
        src->entered = true;
        assert(pthread_cond_broadcast(&src->changed) == 0);
        while (!src->released)
            assert(pthread_cond_wait(&src->changed, &src->mutex) == 0);
    }
    for (uint32_t i = 0; i < count; ++i)
        for (unsigned j = 0; j < 6; ++j)
            out[i].target_position_units[j] = joint_value(first + i, j, src->bias);
    bool ok = first < src->fail_at; /* failure can leave staging partially changed */
    assert(pthread_mutex_unlock(&src->mutex) == 0);
    return ok;
}

static void source_init(Source *src)
{
    memset(src, 0, sizeof(*src));
    assert(pthread_mutex_init(&src->mutex, NULL) == 0);
    assert(pthread_cond_init(&src->changed, NULL) == 0);
    src->block_at = src->fail_at = UINT32_MAX;
}

static void source_destroy(Source *src)
{
    assert(pthread_cond_destroy(&src->changed) == 0);
    assert(pthread_mutex_destroy(&src->mutex) == 0);
}

static void take(TrajectoryPrefetch *s, uint32_t index, uint32_t bias)
{
    PvExecutionSample sample;
    assert(trajectory_prefetch_take(s, index, &sample));
    for (unsigned j = 0; j < 6; ++j)
        assert(sample.target_position_units[j] == joint_value(index, j, bias));
    assert(__atomic_load_n(&s->consumed, __ATOMIC_ACQUIRE) == index + 1U);
}

typedef struct {
    TrajectoryPrefetch *stream;
    pthread_mutex_t mutex;
    pthread_cond_t changed;
    pthread_t thread;
    unsigned requested, completed;
    bool quit;
} Worker;

static void *scheduled_worker(void *context)
{
    Worker *w = context;
    assert(pthread_mutex_lock(&w->mutex) == 0);
    for (;;) {
        while (!w->quit && w->requested == w->completed)
            assert(pthread_cond_wait(&w->changed, &w->mutex) == 0);
        if (w->quit) break;
        assert(pthread_mutex_unlock(&w->mutex) == 0);
        (void)trajectory_prefetch_worker_step(w->stream);
        assert(pthread_mutex_lock(&w->mutex) == 0);
        ++w->completed;
        assert(pthread_cond_broadcast(&w->changed) == 0);
    }
    assert(pthread_mutex_unlock(&w->mutex) == 0);
    return NULL;
}

static void worker_init(Worker *w, TrajectoryPrefetch *s)
{
    memset(w, 0, sizeof(*w));
    w->stream = s;
    assert(pthread_mutex_init(&w->mutex, NULL) == 0);
    assert(pthread_cond_init(&w->changed, NULL) == 0);
    assert(pthread_create(&w->thread, NULL, scheduled_worker, w) == 0);
}

static void request_step(Worker *w)
{
    assert(pthread_mutex_lock(&w->mutex) == 0);
    ++w->requested;
    assert(pthread_cond_broadcast(&w->changed) == 0);
    assert(pthread_mutex_unlock(&w->mutex) == 0);
}

static void await_step(Worker *w)
{
    assert(pthread_mutex_lock(&w->mutex) == 0);
    while (w->completed != w->requested)
        assert(pthread_cond_wait(&w->changed, &w->mutex) == 0);
    assert(pthread_mutex_unlock(&w->mutex) == 0);
}

static void step(Worker *w) { request_step(w); await_step(w); }

static void worker_destroy(Worker *w)
{
    assert(pthread_mutex_lock(&w->mutex) == 0);
    w->quit = true;
    assert(pthread_cond_broadcast(&w->changed) == 0);
    assert(pthread_mutex_unlock(&w->mutex) == 0);
    assert(pthread_join(w->thread, NULL) == 0);
    assert(pthread_cond_destroy(&w->changed) == 0);
    assert(pthread_mutex_destroy(&w->mutex) == 0);
}

static void prepare(TrajectoryPrefetch *s, Source *src, uint32_t count)
{
    assert(trajectory_prefetch_init(s));
    assert(trajectory_prefetch_prepare(s, read_batch, src, count));
    assert(trajectory_prefetch_resume_worker(s));
}

static void test_short_and_final_sample(void)
{
    const uint32_t counts[] = {1, 3, 256, 257, 511, 512, 513};
    for (unsigned k = 0; k < sizeof(counts) / sizeof(counts[0]); ++k) {
        TrajectoryPrefetch s;
        Source src;
        Worker w;
        const uint32_t n = counts[k];
        source_init(&src);
        prepare(&s, &src, n);
        worker_init(&w, &s);
        assert(!trajectory_prefetch_begin_execution(&s, 0, n));
        step(&w);
        if (n > 256U) {
            assert(!trajectory_prefetch_begin_execution(&s, 0, n));
            step(&w);
        }
        assert(!trajectory_prefetch_begin_execution(&s, 0, n + 1U));
        assert(trajectory_prefetch_begin_execution(&s, 0, n));
        assert(!trajectory_prefetch_request_stop(&s));
        assert(!trajectory_prefetch_prepare(&s, read_batch, &src, n));
        trajectory_prefetch_disarm(&s); /* active ownership cannot be invalidated */
        for (uint32_t i = 0; i < n; ++i) {
            PvExecutionSample untouched = {{11, 22, 33, 44, 55, 66}}, before = untouched;
            assert(!trajectory_prefetch_take(&s, i + 1U, &untouched));
            assert(memcmp(&untouched, &before, sizeof(before)) == 0);
            take(&s, i, 0);
            if (trajectory_prefetch_buffered(&s) <= 256U) step(&w);
        }
        /* Worker can run after the last take, while final feedback/retraction
         * still owns the consumer. It must not reset any stream counters. */
        for (unsigned i = 0; i < 4; ++i) step(&w);
        assert(__atomic_load_n(&s.consumed, __ATOMIC_ACQUIRE) == n);
        assert(trajectory_prefetch_begin_execution(&s, n, n));
        assert(!trajectory_prefetch_begin_execution(&s, 0, n));
        trajectory_prefetch_end_execution(&s);
        assert(!trajectory_prefetch_begin_execution(&s, 0, n));
        step(&w);
        if (n > 256U) step(&w);
        assert(trajectory_prefetch_begin_execution(&s, 0, n));
        take(&s, 0, 0);
        worker_destroy(&w);
        source_destroy(&src);
    }
    puts("[PASS] Short/partial batches, ordering, final-sample ownership and automatic replay");
}

static void test_inflight_stop_and_restart(void)
{
    TrajectoryPrefetch s;
    Source src;
    Worker w;
    source_init(&src);
    prepare(&s, &src, 1024);
    worker_init(&w, &s);
    step(&w); step(&w);
    assert(trajectory_prefetch_begin_execution(&s, 0, 1024));
    for (uint32_t i = 0; i < 256; ++i) take(&s, i, 0);
    assert(pthread_mutex_lock(&src.mutex) == 0);
    src.block_at = 512;
    assert(pthread_mutex_unlock(&src.mutex) == 0);
    request_step(&w);
    assert(pthread_mutex_lock(&src.mutex) == 0);
    while (!src.entered) assert(pthread_cond_wait(&src.changed, &src.mutex) == 0);
    assert(pthread_mutex_unlock(&src.mutex) == 0);
    trajectory_prefetch_end_execution(&s); /* abort with a storage read in flight */
    assert(trajectory_prefetch_request_stop(&s));
    assert(!trajectory_prefetch_stopped(&s));
    assert(!trajectory_prefetch_prepare(&s, read_batch, &src, 3));
    assert(pthread_mutex_lock(&src.mutex) == 0);
    src.released = true;
    assert(pthread_cond_broadcast(&src.changed) == 0);
    assert(pthread_mutex_unlock(&src.mutex) == 0);
    await_step(&w);
    assert(!trajectory_prefetch_stopped(&s));
    step(&w); /* acknowledgement must come from the worker after read return */
    assert(trajectory_prefetch_stopped(&s));
    assert(pthread_mutex_lock(&src.mutex) == 0);
    src.bias = 12345;
    assert(pthread_mutex_unlock(&src.mutex) == 0);
    assert(trajectory_prefetch_prepare(&s, read_batch, &src, 3));
    assert(trajectory_prefetch_resume_worker(&s));
    step(&w);
    assert(trajectory_prefetch_begin_execution(&s, 0, 3));
    for (uint32_t i = 0; i < 3; ++i) take(&s, i, 12345);
    worker_destroy(&w);
    source_destroy(&src);
    puts("[PASS] In-flight read blocks stop acknowledgement; restart has no old samples");
}

static void test_failures_and_starvation(void)
{
    TrajectoryPrefetch s;
    Source src;
    Worker w;
    source_init(&src);
    src.fail_at = 0;
    prepare(&s, &src, 1024);
    worker_init(&w, &s);
    step(&w);
    assert(!trajectory_prefetch_begin_execution(&s, 0, 1024));
    assert(trajectory_prefetch_buffered(&s) == 0);
    assert(pthread_mutex_lock(&src.mutex) == 0);
    src.fail_at = UINT32_MAX;
    assert(pthread_mutex_unlock(&src.mutex) == 0);
    trajectory_prefetch_end_execution(&s); /* failed preflight can be retried */
    step(&w); step(&w);
    assert(trajectory_prefetch_begin_execution(&s, 0, 1024));
    for (uint32_t i = 0; i < 256; ++i) take(&s, i, 0);
    assert(pthread_mutex_lock(&src.mutex) == 0);
    src.fail_at = 512;
    assert(pthread_mutex_unlock(&src.mutex) == 0);
    step(&w);
    assert(trajectory_prefetch_buffered(&s) == 256); /* failed batch not published */
    PvExecutionSample sample;
    assert(!trajectory_prefetch_take(&s, 256, &sample));
    assert(!trajectory_prefetch_begin_execution(&s, 256, 1024));
    assert(__atomic_load_n(&s.consumed, __ATOMIC_ACQUIRE) == 256);
    trajectory_prefetch_end_execution(&s);
    assert(pthread_mutex_lock(&src.mutex) == 0);
    src.fail_at = UINT32_MAX;
    assert(pthread_mutex_unlock(&src.mutex) == 0);
    step(&w); step(&w);
    assert(trajectory_prefetch_begin_execution(&s, 0, 1024));
    for (uint32_t i = 0; i < 512; ++i) take(&s, i, 0);
    assert(!trajectory_prefetch_take(&s, 512, &sample));
    assert(!trajectory_prefetch_begin_execution(&s, 512, 1024));
    assert(s.underruns == 1);
    assert(__atomic_load_n(&s.consumed, __ATOMIC_ACQUIRE) == 512);
    worker_destroy(&w);
    source_destroy(&src);
    puts("[PASS] Failed initial/refill reads, unpublished staging, retry and starvation");
}

static void test_fault_publication_during_consumption(void)
{
    TrajectoryPrefetch s;
    Source src;
    Worker w;
    source_init(&src);
    src.block_at = src.fail_at = 512;
    prepare(&s, &src, 1024);
    worker_init(&w, &s);
    step(&w); step(&w);
    assert(trajectory_prefetch_begin_execution(&s, 0, 1024));
    for (uint32_t i = 0; i < 256; ++i) take(&s, i, 0);
    request_step(&w);
    assert(pthread_mutex_lock(&src.mutex) == 0);
    while (!src.entered) assert(pthread_cond_wait(&src.changed, &src.mutex) == 0);
    /* Consumer continues from SRAM while the failing read is held in flight. */
    take(&s, 256, 0);
    src.released = true;
    assert(pthread_cond_broadcast(&src.changed) == 0);
    assert(pthread_mutex_unlock(&src.mutex) == 0);
    uint32_t index = 257;
    PvExecutionSample sample;
    /* No completion handshake here: real takes race the producer's fault
     * publication. A take ordered before publication may still succeed. */
    while (index < 512 && trajectory_prefetch_take(&s, index, &sample)) {
        for (unsigned j = 0; j < 6; ++j)
            assert(sample.target_position_units[j] == joint_value(index, j, 0));
        ++index;
        sched_yield();
    }
    await_step(&w);
    assert(__atomic_load_n(&s.read_failed, __ATOMIC_ACQUIRE));
    assert(!trajectory_prefetch_take(&s, index, &sample));
    assert(__atomic_load_n(&s.consumed, __ATOMIC_ACQUIRE) == index);
    worker_destroy(&w);
    source_destroy(&src);
    puts("[PASS] Fault publication races SRAM consumption; no progress after observed failure");
}

typedef struct { TrajectoryPrefetch *stream; bool quit; } StressWorker;
static void *stress_worker(void *context)
{
    StressWorker *w = context;
    while (!__atomic_load_n(&w->quit, __ATOMIC_ACQUIRE)) {
        (void)trajectory_prefetch_worker_step(w->stream);
        sched_yield();
    }
    return NULL;
}

static void test_concurrent_wraps_and_replay(void)
{
    TrajectoryPrefetch s;
    Source src;
    StressWorker w = {&s, false};
    pthread_t producer;
    source_init(&src);
    prepare(&s, &src, 32001);
    assert(pthread_create(&producer, NULL, stress_worker, &w) == 0);
    for (unsigned run = 0; run < 12; ++run) {
        while (!trajectory_prefetch_begin_execution(&s, 0, 32001)) sched_yield();
        for (uint32_t i = 0; i < 32001; ++i) {
            while (trajectory_prefetch_buffered(&s) == 0) sched_yield();
            take(&s, i, 0);
            if (i == 1234) {
                /* PAUSED does not release/reset ownership; resume checks the
                 * current index even while the producer continues refilling. */
                while (!trajectory_prefetch_begin_execution(&s, i + 1U, 32001))
                    sched_yield();
            }
        }
        trajectory_prefetch_end_execution(&s);
    }
    assert(trajectory_prefetch_request_stop(&s));
    while (!trajectory_prefetch_stopped(&s)) sched_yield();
    /* Repeated configuration handoffs, with the producer task still running.
     * Stop can race replay reset, but prepare is legal only after its ack. */
    for (unsigned run = 0; run < 32; ++run) {
        uint32_t count = 1U + run * 17U, bias = 1000U + run;
        assert(pthread_mutex_lock(&src.mutex) == 0);
        src.bias = bias;
        assert(pthread_mutex_unlock(&src.mutex) == 0);
        assert(trajectory_prefetch_prepare(&s, read_batch, &src, count));
        assert(trajectory_prefetch_resume_worker(&s));
        while (!trajectory_prefetch_begin_execution(&s, 0, count)) sched_yield();
        for (uint32_t i = 0; i < count; ++i) {
            while (trajectory_prefetch_buffered(&s) == 0) sched_yield();
            take(&s, i, bias);
        }
        trajectory_prefetch_end_execution(&s);
        assert(trajectory_prefetch_request_stop(&s));
        while (!trajectory_prefetch_stopped(&s)) sched_yield();
    }
    __atomic_store_n(&w.quit, true, __ATOMIC_RELEASE);
    assert(pthread_join(producer, NULL) == 0);
    source_destroy(&src);
    puts("[PASS] 392476 concurrent samples, all six joints/indices, 12 replays and 32 stop/restarts");
}

int main(void)
{
    test_short_and_final_sample();
    test_inflight_stop_and_restart();
    test_failures_and_starvation();
    test_fault_publication_during_consumption();
    test_concurrent_wraps_and_replay();
    puts("ALL CONCURRENT TRAJECTORY PREFETCH TESTS PASS");
    return 0;
}
