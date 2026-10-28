#include "trajectory_storage_service.h"
#include <string.h>

#define ACQUIRE __ATOMIC_ACQUIRE
#define RELEASE __ATOMIC_RELEASE
#define RELAXED __ATOMIC_RELAXED
#define CRC32_POLYNOMIAL UINT32_C(0xEDB88320)

_Static_assert(sizeof(TrajectorySample) == sizeof(PvExecutionSample),
               "NOR samples and execution samples must share their ABI");

static bool identity_valid(const TrajectoryIdentity *id)
{
    return id != NULL && id->sample_count > 0U &&
           id->sample_period_us == TRAJECTORY_SAMPLE_PERIOD_US;
}

static bool metadata_matches(const ValidatedTrajectory *metadata,
                             const TrajectoryIdentity *id)
{
    return metadata->program_id == id->program_id &&
           metadata->source_revision == id->source_revision &&
           metadata->artifact_crc == id->artifact_crc &&
           metadata->sample_count == id->sample_count &&
           metadata->sample_period_us == id->sample_period_us;
}

static uint32_t crc32_step(uint32_t crc, const void *buffer, size_t length)
{
    const uint8_t *p = buffer;
    for (size_t i = 0U; i < length; ++i) {
        crc ^= p[i];
        for (unsigned bit = 0U; bit < 8U; ++bit)
            crc = (crc >> 1U) ^ (CRC32_POLYNOMIAL &
                                 (uint32_t)-(int32_t)(crc & 1U));
    }
    return crc;
}

bool playback_m7_request(ValidationShared *v, TrajectoryShared *t,
                            const TrajectoryIdentity *identity)
{
    if (!validation_m4_idle(v) || !trajectory_shared_valid(t) ||
        !identity_valid(identity) ||
        __atomic_load_n(&t->ready, ACQUIRE) != 0U)
        return false;
    const uint32_t previous = __atomic_load_n(&v->playback_request_seq, ACQUIRE);
    if (previous != __atomic_load_n(&v->playback_ack_seq, ACQUIRE))
        return false;
    uint32_t next = previous + 1U;
    if (next == 0U) next = 1U;
    v->playback_identity = *identity;
    /* M4 may read the non-atomic identity only after this release. */
    __atomic_store_n(&v->playback_request_seq, next, RELEASE);
    return true;
}

void playback_m7_cancel(ValidationShared *v)
{
    if (!validation_shared_valid(v)) return;
    const uint32_t seq = __atomic_load_n(&v->playback_request_seq, ACQUIRE);
    if (seq != 0U)
        __atomic_store_n(&v->playback_cancel_seq, seq, RELEASE);
}

bool playback_m4_init(PlaybackM4Worker *w,
                         ValidationShared *v,
                         TrajectoryShared *t,
                         const CommittedTrajectoryReader *reader)
{
    if (w == NULL || !validation_shared_valid(v) ||
        !trajectory_shared_valid(t) || reader == NULL ||
        reader->read_metadata == NULL || reader->read_samples == NULL ||
        reader->capacity_samples == 0U ||
        __atomic_load_n(&v->playback_request_seq, ACQUIRE) != 0U ||
        __atomic_load_n(&v->playback_ack_seq, ACQUIRE) != 0U)
        return false;
    memset(w, 0, sizeof(*w));
    w->validation = v;
    w->reader = *reader;
    trajectory_m4_init(&w->producer, t);
    w->initialized = true;
    return true;
}

static bool storage_read(uint32_t first, TrajectorySample *out,
                         uint32_t count, void *context)
{
    PlaybackM4Worker *w = context;
    if (!w->initialized || count == 0U || count > TRAJECTORY_BATCH ||
        first > w->metadata.sample_count ||
        count > w->metadata.sample_count - first)
        return false;
    return w->reader.read_samples(first, out, count, w->reader.context);
}

static void fail(PlaybackM4Worker *w)
{
    __atomic_store_n(&w->validation->playback_status, PLAYBACK_FAILED, RELEASE);
}

bool playback_m4_step(PlaybackM4Worker *w)
{
    if (w == NULL || !w->initialized ||
        !validation_shared_valid(w->validation) ||
        !trajectory_shared_valid(w->producer.shared))
        return false;
    ValidationShared *v = w->validation;
    const uint32_t requested = __atomic_load_n(&v->playback_request_seq, ACQUIRE);
    const uint32_t cancel = __atomic_load_n(&v->playback_cancel_seq, ACQUIRE);
    const uint32_t status = __atomic_load_n(&v->playback_status, ACQUIRE);
    if (status == PLAYBACK_IDLE) {
        if (requested == __atomic_load_n(&v->playback_ack_seq, ACQUIRE))
            return true;
        /* Serialized with validation writes by the same M4 task. */
        if (__atomic_load_n(&v->worker_ready, ACQUIRE) != 1U ||
            __atomic_load_n(&v->flash_status, ACQUIRE) != VALIDATION_FLASH_IDLE ||
            __atomic_load_n(&v->head, ACQUIRE) !=
                __atomic_load_n(&v->tail, ACQUIRE)) {
            fail(w);
            __atomic_store_n(&v->playback_ack_seq, requested, RELEASE);
            return false;
        }
        w->request_seq = requested;
        w->requested = v->playback_identity;
        if (cancel == requested) {
            __atomic_store_n(&v->playback_ack_seq, requested, RELEASE);
            return true;
        }
        /* Block any new validation BEGIN before acknowledging the request:
         * no window may exist where storage appears idle during NOR read. */
        __atomic_store_n(&v->playback_status, PLAYBACK_VERIFYING, RELEASE);
        __atomic_store_n(&v->playback_ack_seq, requested, RELEASE);
        if (!identity_valid(&w->requested) ||
            w->requested.sample_count > w->reader.capacity_samples ||
            !w->reader.read_metadata(&w->metadata, w->reader.context) ||
            !metadata_matches(&w->metadata, &w->requested)) {
            fail(w);
            return false;
        }
        w->verify_crc = UINT32_MAX;
        w->verify_index = 0U;
        __atomic_store_n(&v->playback_status, PLAYBACK_VERIFYING, RELEASE);
        return true;
    }

    if (status == PLAYBACK_VERIFYING) {
        if (cancel == w->request_seq) {
            __atomic_store_n(&v->playback_status, PLAYBACK_IDLE, RELEASE);
            return true;
        }
        if (w->verify_index < w->requested.sample_count) {
            uint32_t count = w->requested.sample_count - w->verify_index;
            if (count > TRAJECTORY_BATCH) count = TRAJECTORY_BATCH;
            if (!storage_read(w->verify_index, w->verify_buffer, count, w)) {
                fail(w);
                return false;
            }
            w->verify_crc = crc32_step(w->verify_crc, w->verify_buffer,
                                      (size_t)count * sizeof(TrajectorySample));
            w->verify_index += count;
            return true;
        }
        /* A COMPLETE readback/CRC precedes publication of even sample zero. */
        if ((~w->verify_crc) != w->metadata.sample_data_crc ||
            !trajectory_m4_prepare(&w->producer, storage_read, w, &w->requested)) {
            fail(w);
            return false;
        }
        __atomic_store_n(&v->playback_status, PLAYBACK_STREAMING, RELEASE);
        return true;
    }

    if (status == PLAYBACK_STREAMING || status == PLAYBACK_FAILED) {
        /* M7 publishes cancellation only AFTER the execution state has
         * stopped using the ring. M4 can now abandon a prepared-but-unused
         * epoch even when M7 never called trajectory_m7_begin(). */
        if (cancel == w->request_seq) {
            if (w->producer.active) {
                TrajectoryShared *t = w->producer.shared;
                __atomic_store_n(&t->ready, 0U, RELEASE);
                w->producer.active = false;
                __atomic_store_n(&t->release_ack_epoch, w->producer.epoch, RELEASE);
            }
            __atomic_store_n(&v->playback_status, PLAYBACK_IDLE, RELEASE);
            return true;
        }
        /* Release ack must be processed after any in-flight flash callback.
         * A failed producer also processes release before returning failure. */
        if (w->producer.active) {
            if (!trajectory_m4_step(&w->producer)) {
                fail(w);
                return false;
            }
            if (w->producer.active) return true;
        }
        if (status == PLAYBACK_STREAMING &&
            __atomic_load_n(&w->producer.shared->read_error, ACQUIRE) != 0U) {
            fail(w);
            return false;
        }
        if (cancel == w->request_seq || status == PLAYBACK_STREAMING) {
            __atomic_store_n(&v->playback_status, PLAYBACK_IDLE, RELEASE);
            return true;
        }
        return false; /* A failure persists until M7 cancels. */
    }
    fail(w);
    return false;
}
