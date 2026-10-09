#include "../Storage/trajectory_prefetch.h"
#include <assert.h>
#include <stdio.h>

struct Source { uint32_t calls; uint32_t fail_after; uint32_t max_batch; };
static bool read_samples(uint32_t first, PvExecutionSample *out,
                         uint32_t n, void *p)
{
    struct Source *s = (struct Source *)p;
    s->calls++;
    if (first >= s->fail_after) return false;
    if (n > s->max_batch) s->max_batch = n;
    for (uint32_t i = 0; i < n; ++i)
        for (unsigned j = 0; j < 6; ++j)
            out[i].target_position_units[j] = (int32_t)((first+i)*10U+j);
    return true;
}
static void verify(const PvExecutionSample *s, uint32_t i)
{
    for (unsigned j = 0; j < 6; ++j)
        assert(s->target_position_units[j] == (int32_t)(i*10U+j));
}
static void prepare(TrajectoryPrefetch *s, struct Source *src, uint32_t count)
{
    trajectory_prefetch_end_execution(s);
    assert(trajectory_prefetch_request_stop(s));
    assert(trajectory_prefetch_worker_step(s));
    assert(trajectory_prefetch_stopped(s));
    assert(trajectory_prefetch_prepare(s, read_samples, src, count));
}
int main(void)
{
    TrajectoryPrefetch s;
    struct Source src = {0, 100000U, 0};
    PvExecutionSample v;
    assert(trajectory_prefetch_init(&s));
    prepare(&s, &src, 1536);
    assert(trajectory_prefetch_refill(&s));
    assert(trajectory_prefetch_refill(&s));
    assert(trajectory_prefetch_resume_worker(&s));
    assert(trajectory_prefetch_ready(&s, 512));
    assert(trajectory_prefetch_begin_execution(&s, 0, 1536));
    assert(trajectory_prefetch_buffered(&s) == 512);
    assert(!trajectory_prefetch_take(&s, 1, &v));
    assert(trajectory_prefetch_buffered(&s) == 512);
    puts("[PASS] Full 512-sample prefill; wrong execution index rejected");

    for (uint32_t i = 0; i < 1536; ++i) {
        if (trajectory_prefetch_buffered(&s) <= 256U &&
            s.next_flash_index < s.total_samples)
            assert(trajectory_prefetch_refill(&s));
        uint32_t before = src.calls;
        assert(trajectory_prefetch_take(&s, i, &v));
        assert(src.calls == before); /* no flash read in consumer callback */
        verify(&v, i);
    }
    assert(s.refill_batches == 6);
    assert(s.underruns == 0);
    assert(trajectory_prefetch_buffered(&s) == 0);
    printf("[PASS] 1536 samples, 6 bulk refills, 0 underruns; execution read never calls storage\n");

    /* Intentional starvation: the producer cannot fill after 512 prefill. */
    prepare(&s, &src, 1024);
    assert(trajectory_prefetch_refill(&s));
    assert(trajectory_prefetch_refill(&s));
    assert(trajectory_prefetch_resume_worker(&s));
    assert(trajectory_prefetch_begin_execution(&s, 0, 1024));
    for (uint32_t i = 0; i < 512; ++i)
        assert(trajectory_prefetch_take(&s, i, &v));
    assert(!trajectory_prefetch_take(&s, 512, &v));
    assert(s.underruns == 1);
    puts("[PASS] Starved buffer reliably fails instead of reading storage in execution");

    src.calls = 0; src.fail_after = 512;
    prepare(&s, &src, 1024);
    assert(trajectory_prefetch_refill(&s));
    assert(trajectory_prefetch_refill(&s));
    assert(trajectory_prefetch_refill(&s)); /* Full buffer: no source access. */
    assert(src.calls == 2);
    assert(trajectory_prefetch_resume_worker(&s));
    assert(trajectory_prefetch_begin_execution(&s, 0, 1024));
    for (uint32_t i = 0; i < 256; ++i)
        assert(trajectory_prefetch_take(&s, i, &v));
    assert(!trajectory_prefetch_refill(&s)); /* Injected flash read failure. */
    assert(!trajectory_prefetch_take(&s, 256, &v));
    puts("[PASS] Injected storage fault propagates to nonblocking consumer");

    src.fail_after = 100000U;
    prepare(&s, &src, 3);
    assert(trajectory_prefetch_refill(&s));
    assert(trajectory_prefetch_resume_worker(&s));
    assert(trajectory_prefetch_ready(&s, 3));
    assert(trajectory_prefetch_begin_execution(&s, 0, 3));
    for (uint32_t i = 0; i < 3; ++i) {
        assert(trajectory_prefetch_take(&s, i, &v));
        verify(&v, i);
    }
    assert(!trajectory_prefetch_take(&s, 3, &v));
    puts("[PASS] Rearm with shorter trajectory prevents stale data reuse");
    puts("ALL STANDALONE TRAJECTORY PREFETCH TESTS PASS");
    return 0;
}
