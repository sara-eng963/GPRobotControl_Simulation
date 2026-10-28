#include "trajectory_stream.h"
#include <string.h>

#define ATOMIC_ACQUIRE __ATOMIC_ACQUIRE
#define ATOMIC_RELEASE __ATOMIC_RELEASE
#define ATOMIC_RELAXED __ATOMIC_RELAXED

_Static_assert(sizeof(TrajectorySample) == 24U, "Six drive targets must occupy 24 bytes");
_Static_assert(offsetof(TrajectoryShared, samples) % 32U == 0U,
               "Ring must begin on a 32-byte boundary");
_Static_assert(_Alignof(TrajectoryShared) >= 32U,
               "Shared ring requires 32-byte alignment");
_Static_assert(__atomic_always_lock_free(sizeof(uint32_t), 0),
               "32-bit lock-free atomic counters required");

static bool identity_equal(const TrajectoryIdentity *a,
                           const TrajectoryIdentity *b)
{
    return a->program_id == b->program_id &&
           a->source_revision == b->source_revision &&
           a->artifact_crc == b->artifact_crc &&
           a->sample_count == b->sample_count &&
           a->sample_period_us == b->sample_period_us;
}

static bool identity_valid(const TrajectoryIdentity *identity)
{
    return identity != NULL && identity->sample_count > 0U &&
           identity->sample_period_us == TRAJECTORY_SAMPLE_PERIOD_US;
}

void trajectory_shared_initialize(TrajectoryShared *shared)
{
    if (shared == NULL) return;
    /* Both CPUs must be quiescent; a live ring can NEVER be memset. */
    memset(shared, 0, sizeof(*shared));
    shared->abi_version = TRAJECTORY_ABI_VERSION;
    __atomic_store_n(&shared->magic, TRAJECTORY_MAGIC, ATOMIC_RELEASE);
}

bool trajectory_shared_valid(const TrajectoryShared *shared)
{
    return shared != NULL &&
           __atomic_load_n(&shared->magic, ATOMIC_ACQUIRE) == TRAJECTORY_MAGIC &&
           shared->abi_version == TRAJECTORY_ABI_VERSION;
}

void trajectory_m4_init(TrajectoryProducer *producer, TrajectoryShared *shared)
{
    if (producer == NULL) return;
    memset(producer, 0, sizeof(*producer));
    producer->shared = shared;
}

void trajectory_m7_init(TrajectoryConsumer *consumer, TrajectoryShared *shared)
{
    if (consumer == NULL) return;
    memset(consumer, 0, sizeof(*consumer));
    consumer->shared = shared;
}

uint32_t trajectory_current_epoch(const TrajectoryShared *shared)
{
    return trajectory_shared_valid(shared) ?
        __atomic_load_n(&shared->epoch, ATOMIC_ACQUIRE) : 0U;
}

uint32_t trajectory_buffered(const TrajectoryShared *shared)
{
    if (!trajectory_shared_valid(shared)) return 0U;
    const uint32_t written = __atomic_load_n(&shared->written, ATOMIC_ACQUIRE);
    const uint32_t consumed = __atomic_load_n(&shared->consumed, ATOMIC_ACQUIRE);
    const uint32_t available = (uint32_t)(written - consumed);
    return available <= TRAJECTORY_CAPACITY ? available : 0U;
}

bool trajectory_m4_release_acknowledged(const TrajectoryShared *shared,
                                     uint32_t epoch)
{
    return trajectory_shared_valid(shared) && epoch != 0U &&
           __atomic_load_n(&shared->release_ack_epoch, ATOMIC_ACQUIRE) == epoch;
}

bool trajectory_m4_prepare(TrajectoryProducer *producer,
                        TrajectoryReadBatch read_batch, void *context,
                        const TrajectoryIdentity *identity)
{
    if (producer == NULL || !trajectory_shared_valid(producer->shared) ||
        producer->active || read_batch == NULL || !identity_valid(identity))
        return false;
    TrajectoryShared *shared = producer->shared;
    const uint32_t previous_epoch =
        __atomic_load_n(&shared->epoch, ATOMIC_ACQUIRE);
    if (previous_epoch != 0U &&
        !trajectory_m4_release_acknowledged(shared, previous_epoch))
        return false;
    /* Release/ack has made M7 quiescent; preserve the M7-owned tail.
     * M4 writes only the head, so counters need not be reset by both cores. */
    const uint32_t consumed = __atomic_load_n(&shared->consumed, ATOMIC_ACQUIRE);
    __atomic_store_n(&shared->ready, 0U, ATOMIC_RELEASE);
    shared->identity = *identity;
    __atomic_store_n(&shared->written, consumed, ATOMIC_RELAXED);
    __atomic_store_n(&shared->read_error, 0U, ATOMIC_RELAXED);
    uint32_t new_epoch = previous_epoch + 1U;
    if (new_epoch == 0U) new_epoch = 1U;
    producer->read_batch = read_batch;
    producer->read_context = context;
    producer->next_index = 0U;
    producer->refills = 0U;
    producer->epoch = new_epoch;
    producer->active = true;
    __atomic_store_n(&shared->epoch, new_epoch, ATOMIC_RELEASE);
    __atomic_store_n(&shared->ready, 1U, ATOMIC_RELEASE);
    return true;
}

bool trajectory_m4_step(TrajectoryProducer *producer)
{
    if (producer == NULL || !producer->active ||
        !trajectory_shared_valid(producer->shared)) return false;
    TrajectoryShared *shared = producer->shared;
    if (__atomic_load_n(&shared->release_epoch, ATOMIC_ACQUIRE) ==
        producer->epoch) {
        /* Complete pending callback/publication BEFORE acknowledging release. */
        producer->active = false;
        __atomic_store_n(&shared->ready, 0U, ATOMIC_RELEASE);
        __atomic_store_n(&shared->release_ack_epoch, producer->epoch, ATOMIC_RELEASE);
        return true;
    }
    if (__atomic_load_n(&shared->read_error, ATOMIC_ACQUIRE) != 0U) return false;
    if (producer->next_index >= shared->identity.sample_count) return true;
    const uint32_t written = __atomic_load_n(&shared->written, ATOMIC_RELAXED);
    const uint32_t consumed = __atomic_load_n(&shared->consumed, ATOMIC_ACQUIRE);
    const uint32_t buffered = written - consumed;
    if (buffered > TRAJECTORY_CAPACITY) {
        __atomic_store_n(&shared->read_error, 2U, ATOMIC_RELEASE);
        return false;
    }
    if (buffered > TRAJECTORY_BATCH) return true;
    uint32_t batch = shared->identity.sample_count - producer->next_index;
    if (batch > TRAJECTORY_BATCH) batch = TRAJECTORY_BATCH;
    if (batch > TRAJECTORY_CAPACITY - buffered)
        batch = TRAJECTORY_CAPACITY - buffered;
    if (batch == 0U) return true;

    /* Blocking storage callback touches PRIVATE M4 staging, never shared SRAM. */
    if (!producer->read_batch(producer->next_index, producer->staging,
                              batch, producer->read_context)) {
        __atomic_store_n(&shared->read_error, 1U, ATOMIC_RELEASE);
        return false;
    }
    if (__atomic_load_n(&shared->release_epoch, ATOMIC_ACQUIRE) ==
        producer->epoch) {
        /* M7 already stopped: discard an in-flight flash read. */
        producer->active = false;
        __atomic_store_n(&shared->ready, 0U, ATOMIC_RELEASE);
        __atomic_store_n(&shared->release_ack_epoch, producer->epoch, ATOMIC_RELEASE);
        return true;
    }
    for (uint32_t i = 0U; i < batch; ++i)
        shared->samples[(written + i) % TRAJECTORY_CAPACITY] = producer->staging[i];
    /* Release publishes the ENTIRE batch after all six joints are copied. */
    __atomic_store_n(&shared->written, written + batch, ATOMIC_RELEASE);
    producer->next_index += batch;
    producer->refills++;
    return true;
}

bool trajectory_m7_begin(TrajectoryConsumer *consumer,
                      const TrajectoryIdentity *expected)
{
    if (consumer == NULL || consumer->active ||
        !trajectory_shared_valid(consumer->shared) || !identity_valid(expected))
        return false;
    TrajectoryShared *shared = consumer->shared;
    if (__atomic_load_n(&shared->ready, ATOMIC_ACQUIRE) != 1U ||
        __atomic_load_n(&shared->read_error, ATOMIC_ACQUIRE) != 0U)
        return false;
    const uint32_t epoch = __atomic_load_n(&shared->epoch, ATOMIC_ACQUIRE);
    /* A stream already released/discarded by M7 must not be reopened
     * while M4 still considers it READY (before release acknowledgement).
     * Otherwise M4 could reuse the ring underneath a new reader. */
    if (epoch == 0U || epoch == consumer->epoch ||
        __atomic_load_n(&shared->release_epoch, ATOMIC_ACQUIRE) == epoch ||
        !identity_equal(expected, &shared->identity))
        return false;
    const uint32_t minimum =
        expected->sample_count < TRAJECTORY_CAPACITY ?
        expected->sample_count : TRAJECTORY_CAPACITY;
    if (trajectory_buffered(shared) < minimum) return false;
    /* The publisher can only reset metadata after M7 releases this epoch. */
    consumer->epoch = epoch;
    consumer->expected = *expected;
    consumer->next_index = 0U;
    consumer->underruns = 0U;
    consumer->active = true;
    return true;
}

TrajectoryReadResult trajectory_m7_take(TrajectoryConsumer *consumer,
                                      uint32_t expected_index,
                                      TrajectorySample *sample)
{
    if (consumer == NULL || sample == NULL || !consumer->active ||
        !trajectory_shared_valid(consumer->shared))
        return TRAJECTORY_READ_UNAVAILABLE;
    TrajectoryShared *shared = consumer->shared;
    if (__atomic_load_n(&shared->epoch, ATOMIC_ACQUIRE) != consumer->epoch ||
        __atomic_load_n(&shared->ready, ATOMIC_ACQUIRE) != 1U)
        return TRAJECTORY_READ_UNAVAILABLE;
    if (__atomic_load_n(&shared->read_error, ATOMIC_ACQUIRE) != 0U)
        return TRAJECTORY_READ_STORAGE_ERROR;
    if (expected_index != consumer->next_index ||
        expected_index >= consumer->expected.sample_count)
        return TRAJECTORY_READ_INVALID_INDEX;

    const uint32_t consumed = __atomic_load_n(&shared->consumed, ATOMIC_RELAXED);
    const uint32_t written = __atomic_load_n(&shared->written, ATOMIC_ACQUIRE);
    const uint32_t buffered = written - consumed;
    if (buffered == 0U) {
        consumer->underruns++;
        return TRAJECTORY_READ_STARVED;
    }
    if (buffered > TRAJECTORY_CAPACITY) return TRAJECTORY_READ_STORAGE_ERROR;
    *sample = shared->samples[consumed % TRAJECTORY_CAPACITY];
    /* Release makes this slot reusable ONLY after the entire sample was read. */
    __atomic_store_n(&shared->consumed, consumed + 1U, ATOMIC_RELEASE);
    consumer->next_index++;
    return TRAJECTORY_READ_OK;
}

TrajectoryReadResult trajectory_m7_skip(TrajectoryConsumer *consumer,
                                      uint32_t expected_index,
                                      uint32_t count)
{
    if (consumer == NULL || !consumer->active ||
        !trajectory_shared_valid(consumer->shared))
        return TRAJECTORY_READ_UNAVAILABLE;
    TrajectoryShared *shared = consumer->shared;
    if (__atomic_load_n(&shared->epoch, ATOMIC_ACQUIRE) != consumer->epoch ||
        __atomic_load_n(&shared->ready, ATOMIC_ACQUIRE) != 1U)
        return TRAJECTORY_READ_UNAVAILABLE;
    if (__atomic_load_n(&shared->read_error, ATOMIC_ACQUIRE) != 0U)
        return TRAJECTORY_READ_STORAGE_ERROR;
    if (expected_index != consumer->next_index ||
        count > consumer->expected.sample_count - consumer->next_index)
        return TRAJECTORY_READ_INVALID_INDEX;
    const uint32_t consumed = __atomic_load_n(&shared->consumed, ATOMIC_RELAXED);
    const uint32_t written = __atomic_load_n(&shared->written, ATOMIC_ACQUIRE);
    const uint32_t buffered = written - consumed;
    if (buffered > TRAJECTORY_CAPACITY) return TRAJECTORY_READ_STORAGE_ERROR;
    if (count > buffered) {
        consumer->underruns++;
        return TRAJECTORY_READ_STARVED;
    }
    __atomic_store_n(&shared->consumed, consumed + count, ATOMIC_RELEASE);
    consumer->next_index += count;
    return TRAJECTORY_READ_OK;
}

bool trajectory_m7_finish(TrajectoryConsumer *consumer)
{
    if (consumer == NULL || !consumer->active ||
        !trajectory_shared_valid(consumer->shared)) return false;
    consumer->active = false;
    __atomic_store_n(&consumer->shared->release_epoch, consumer->epoch, ATOMIC_RELEASE);
    return true;
}

bool trajectory_m7_discard(TrajectoryConsumer *consumer)
{
    if (consumer == NULL || consumer->active ||
        !trajectory_shared_valid(consumer->shared)) return false;
    TrajectoryShared *shared = consumer->shared;
    const uint32_t epoch = __atomic_load_n(&shared->epoch, ATOMIC_ACQUIRE);
    if (epoch == 0U ||
        __atomic_load_n(&shared->ready, ATOMIC_ACQUIRE) == 0U ||
        __atomic_load_n(&shared->release_epoch, ATOMIC_ACQUIRE) == epoch)
        return false;
    consumer->epoch = epoch;
    __atomic_store_n(&shared->release_epoch, epoch, ATOMIC_RELEASE);
    return true;
}
