#include "trajectory_stream.h"
#include <string.h>

#define GP_ACQUIRE __ATOMIC_ACQUIRE
#define GP_RELEASE __ATOMIC_RELEASE
#define GP_RELAXED __ATOMIC_RELAXED

_Static_assert(sizeof(GpTrajectorySample) == 24U, "Six drive targets must occupy 24 bytes");
_Static_assert(offsetof(GpTrajectoryShared, samples) % 32U == 0U,
               "Ring must begin on a 32-byte boundary");
_Static_assert(_Alignof(GpTrajectoryShared) >= 32U,
               "Shared ring requires 32-byte alignment");
_Static_assert(__atomic_always_lock_free(sizeof(uint32_t), 0),
               "32-bit lock-free atomic counters required");

static bool identity_equal(const GpTrajectoryIdentity *a,
                           const GpTrajectoryIdentity *b)
{
    return a->program_id == b->program_id &&
           a->source_revision == b->source_revision &&
           a->artifact_crc == b->artifact_crc &&
           a->sample_count == b->sample_count &&
           a->sample_period_us == b->sample_period_us;
}

static bool identity_valid(const GpTrajectoryIdentity *identity)
{
    return identity != NULL && identity->sample_count > 0U &&
           identity->sample_period_us == GP_TRAJ_SAMPLE_PERIOD_US;
}

void gp_traj_shared_initialize(GpTrajectoryShared *shared)
{
    if (shared == NULL) return;
    /* Both CPUs must be quiescent; a live ring can NEVER be memset. */
    memset(shared, 0, sizeof(*shared));
    shared->abi_version = GP_TRAJ_ABI_VERSION;
    __atomic_store_n(&shared->magic, GP_TRAJ_MAGIC, GP_RELEASE);
}

bool gp_traj_shared_valid(const GpTrajectoryShared *shared)
{
    return shared != NULL &&
           __atomic_load_n(&shared->magic, GP_ACQUIRE) == GP_TRAJ_MAGIC &&
           shared->abi_version == GP_TRAJ_ABI_VERSION;
}

void gp_traj_m4_init(GpTrajectoryProducer *producer, GpTrajectoryShared *shared)
{
    if (producer == NULL) return;
    memset(producer, 0, sizeof(*producer));
    producer->shared = shared;
}

void gp_traj_m7_init(GpTrajectoryConsumer *consumer, GpTrajectoryShared *shared)
{
    if (consumer == NULL) return;
    memset(consumer, 0, sizeof(*consumer));
    consumer->shared = shared;
}

uint32_t gp_traj_current_epoch(const GpTrajectoryShared *shared)
{
    return gp_traj_shared_valid(shared) ?
        __atomic_load_n(&shared->epoch, GP_ACQUIRE) : 0U;
}

uint32_t gp_traj_buffered(const GpTrajectoryShared *shared)
{
    if (!gp_traj_shared_valid(shared)) return 0U;
    const uint32_t written = __atomic_load_n(&shared->written, GP_ACQUIRE);
    const uint32_t consumed = __atomic_load_n(&shared->consumed, GP_ACQUIRE);
    const uint32_t available = (uint32_t)(written - consumed);
    return available <= GP_TRAJ_CAPACITY ? available : 0U;
}

bool gp_traj_m4_release_acknowledged(const GpTrajectoryShared *shared,
                                     uint32_t epoch)
{
    return gp_traj_shared_valid(shared) && epoch != 0U &&
           __atomic_load_n(&shared->release_ack_epoch, GP_ACQUIRE) == epoch;
}

bool gp_traj_m4_prepare(GpTrajectoryProducer *producer,
                        GpTrajectoryReadBatch read_batch, void *context,
                        const GpTrajectoryIdentity *identity)
{
    if (producer == NULL || !gp_traj_shared_valid(producer->shared) ||
        producer->active || read_batch == NULL || !identity_valid(identity))
        return false;
    GpTrajectoryShared *shared = producer->shared;
    const uint32_t previous_epoch =
        __atomic_load_n(&shared->epoch, GP_ACQUIRE);
    if (previous_epoch != 0U &&
        !gp_traj_m4_release_acknowledged(shared, previous_epoch))
        return false;
    /* Release/ack has made M7 quiescent; preserve the M7-owned tail.
     * M4 writes only the head, so counters need not be reset by both cores. */
    const uint32_t consumed = __atomic_load_n(&shared->consumed, GP_ACQUIRE);
    __atomic_store_n(&shared->ready, 0U, GP_RELEASE);
    shared->identity = *identity;
    __atomic_store_n(&shared->written, consumed, GP_RELAXED);
    __atomic_store_n(&shared->read_error, 0U, GP_RELAXED);
    uint32_t new_epoch = previous_epoch + 1U;
    if (new_epoch == 0U) new_epoch = 1U;
    producer->read_batch = read_batch;
    producer->read_context = context;
    producer->next_index = 0U;
    producer->refills = 0U;
    producer->epoch = new_epoch;
    producer->active = true;
    __atomic_store_n(&shared->epoch, new_epoch, GP_RELEASE);
    __atomic_store_n(&shared->ready, 1U, GP_RELEASE);
    return true;
}

bool gp_traj_m4_step(GpTrajectoryProducer *producer)
{
    if (producer == NULL || !producer->active ||
        !gp_traj_shared_valid(producer->shared)) return false;
    GpTrajectoryShared *shared = producer->shared;
    if (__atomic_load_n(&shared->release_epoch, GP_ACQUIRE) ==
        producer->epoch) {
        /* Complete pending callback/publication BEFORE acknowledging release. */
        producer->active = false;
        __atomic_store_n(&shared->ready, 0U, GP_RELEASE);
        __atomic_store_n(&shared->release_ack_epoch, producer->epoch, GP_RELEASE);
        return true;
    }
    if (__atomic_load_n(&shared->read_error, GP_ACQUIRE) != 0U) return false;
    if (producer->next_index >= shared->identity.sample_count) return true;
    const uint32_t written = __atomic_load_n(&shared->written, GP_RELAXED);
    const uint32_t consumed = __atomic_load_n(&shared->consumed, GP_ACQUIRE);
    const uint32_t buffered = written - consumed;
    if (buffered > GP_TRAJ_CAPACITY) {
        __atomic_store_n(&shared->read_error, 2U, GP_RELEASE);
        return false;
    }
    if (buffered > GP_TRAJ_BATCH) return true;
    uint32_t batch = shared->identity.sample_count - producer->next_index;
    if (batch > GP_TRAJ_BATCH) batch = GP_TRAJ_BATCH;
    if (batch > GP_TRAJ_CAPACITY - buffered)
        batch = GP_TRAJ_CAPACITY - buffered;
    if (batch == 0U) return true;

    /* Blocking storage callback touches PRIVATE M4 staging, never shared SRAM. */
    if (!producer->read_batch(producer->next_index, producer->staging,
                              batch, producer->read_context)) {
        __atomic_store_n(&shared->read_error, 1U, GP_RELEASE);
        return false;
    }
    if (__atomic_load_n(&shared->release_epoch, GP_ACQUIRE) ==
        producer->epoch) {
        /* M7 already stopped: discard an in-flight flash read. */
        producer->active = false;
        __atomic_store_n(&shared->ready, 0U, GP_RELEASE);
        __atomic_store_n(&shared->release_ack_epoch, producer->epoch, GP_RELEASE);
        return true;
    }
    for (uint32_t i = 0U; i < batch; ++i)
        shared->samples[(written + i) % GP_TRAJ_CAPACITY] = producer->staging[i];
    /* Release publishes the ENTIRE batch after all six joints are copied. */
    __atomic_store_n(&shared->written, written + batch, GP_RELEASE);
    producer->next_index += batch;
    producer->refills++;
    return true;
}

bool gp_traj_m7_begin(GpTrajectoryConsumer *consumer,
                      const GpTrajectoryIdentity *expected)
{
    if (consumer == NULL || consumer->active ||
        !gp_traj_shared_valid(consumer->shared) || !identity_valid(expected))
        return false;
    GpTrajectoryShared *shared = consumer->shared;
    if (__atomic_load_n(&shared->ready, GP_ACQUIRE) != 1U ||
        __atomic_load_n(&shared->read_error, GP_ACQUIRE) != 0U)
        return false;
    const uint32_t epoch = __atomic_load_n(&shared->epoch, GP_ACQUIRE);
    /* A stream already released/discarded by M7 must not be reopened
     * while M4 still considers it READY (before release acknowledgement).
     * Otherwise M4 could reuse the ring underneath a new reader. */
    if (epoch == 0U || epoch == consumer->epoch ||
        __atomic_load_n(&shared->release_epoch, GP_ACQUIRE) == epoch ||
        !identity_equal(expected, &shared->identity))
        return false;
    const uint32_t minimum =
        expected->sample_count < GP_TRAJ_CAPACITY ?
        expected->sample_count : GP_TRAJ_CAPACITY;
    if (gp_traj_buffered(shared) < minimum) return false;
    /* The publisher can only reset metadata after M7 releases this epoch. */
    consumer->epoch = epoch;
    consumer->expected = *expected;
    consumer->next_index = 0U;
    consumer->underruns = 0U;
    consumer->active = true;
    return true;
}

GpTrajectoryReadResult gp_traj_m7_take(GpTrajectoryConsumer *consumer,
                                      uint32_t expected_index,
                                      GpTrajectorySample *sample)
{
    if (consumer == NULL || sample == NULL || !consumer->active ||
        !gp_traj_shared_valid(consumer->shared))
        return GP_TRAJ_READ_UNAVAILABLE;
    GpTrajectoryShared *shared = consumer->shared;
    if (__atomic_load_n(&shared->epoch, GP_ACQUIRE) != consumer->epoch ||
        __atomic_load_n(&shared->ready, GP_ACQUIRE) != 1U)
        return GP_TRAJ_READ_UNAVAILABLE;
    if (__atomic_load_n(&shared->read_error, GP_ACQUIRE) != 0U)
        return GP_TRAJ_READ_STORAGE_ERROR;
    if (expected_index != consumer->next_index ||
        expected_index >= consumer->expected.sample_count)
        return GP_TRAJ_READ_INVALID_INDEX;

    const uint32_t consumed = __atomic_load_n(&shared->consumed, GP_RELAXED);
    const uint32_t written = __atomic_load_n(&shared->written, GP_ACQUIRE);
    const uint32_t buffered = written - consumed;
    if (buffered == 0U) {
        consumer->underruns++;
        return GP_TRAJ_READ_STARVED;
    }
    if (buffered > GP_TRAJ_CAPACITY) return GP_TRAJ_READ_STORAGE_ERROR;
    *sample = shared->samples[consumed % GP_TRAJ_CAPACITY];
    /* Release makes this slot reusable ONLY after the entire sample was read. */
    __atomic_store_n(&shared->consumed, consumed + 1U, GP_RELEASE);
    consumer->next_index++;
    return GP_TRAJ_READ_OK;
}

GpTrajectoryReadResult gp_traj_m7_skip(GpTrajectoryConsumer *consumer,
                                      uint32_t expected_index,
                                      uint32_t count)
{
    if (consumer == NULL || !consumer->active ||
        !gp_traj_shared_valid(consumer->shared))
        return GP_TRAJ_READ_UNAVAILABLE;
    GpTrajectoryShared *shared = consumer->shared;
    if (__atomic_load_n(&shared->epoch, GP_ACQUIRE) != consumer->epoch ||
        __atomic_load_n(&shared->ready, GP_ACQUIRE) != 1U)
        return GP_TRAJ_READ_UNAVAILABLE;
    if (__atomic_load_n(&shared->read_error, GP_ACQUIRE) != 0U)
        return GP_TRAJ_READ_STORAGE_ERROR;
    if (expected_index != consumer->next_index ||
        count > consumer->expected.sample_count - consumer->next_index)
        return GP_TRAJ_READ_INVALID_INDEX;
    const uint32_t consumed = __atomic_load_n(&shared->consumed, GP_RELAXED);
    const uint32_t written = __atomic_load_n(&shared->written, GP_ACQUIRE);
    const uint32_t buffered = written - consumed;
    if (buffered > GP_TRAJ_CAPACITY) return GP_TRAJ_READ_STORAGE_ERROR;
    if (count > buffered) {
        consumer->underruns++;
        return GP_TRAJ_READ_STARVED;
    }
    __atomic_store_n(&shared->consumed, consumed + count, GP_RELEASE);
    consumer->next_index += count;
    return GP_TRAJ_READ_OK;
}

bool gp_traj_m7_finish(GpTrajectoryConsumer *consumer)
{
    if (consumer == NULL || !consumer->active ||
        !gp_traj_shared_valid(consumer->shared)) return false;
    consumer->active = false;
    __atomic_store_n(&consumer->shared->release_epoch, consumer->epoch, GP_RELEASE);
    return true;
}

bool gp_traj_m7_discard(GpTrajectoryConsumer *consumer)
{
    if (consumer == NULL || consumer->active ||
        !gp_traj_shared_valid(consumer->shared)) return false;
    GpTrajectoryShared *shared = consumer->shared;
    const uint32_t epoch = __atomic_load_n(&shared->epoch, GP_ACQUIRE);
    if (epoch == 0U ||
        __atomic_load_n(&shared->ready, GP_ACQUIRE) == 0U ||
        __atomic_load_n(&shared->release_epoch, GP_ACQUIRE) == epoch)
        return false;
    consumer->epoch = epoch;
    __atomic_store_n(&shared->release_epoch, epoch, GP_RELEASE);
    return true;
}
