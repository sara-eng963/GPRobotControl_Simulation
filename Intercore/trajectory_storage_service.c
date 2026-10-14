#include "trajectory_storage_service.h"
#include <string.h>

#define ACQUIRE __ATOMIC_ACQUIRE
#define RELEASE __ATOMIC_RELEASE
#define RELAXED __ATOMIC_RELAXED
#define GP_CRC32_POLYNOMIAL UINT32_C(0xEDB88320)

_Static_assert(sizeof(GpTrajectorySample) == sizeof(PvExecutionSample),
               "NOR samples and execution samples must share their ABI");

static bool identity_valid(const GpTrajectoryIdentity *id)
{
    return id != NULL && id->sample_count > 0U &&
           id->sample_period_us == GP_TRAJ_SAMPLE_PERIOD_US;
}

static bool metadata_matches(const ValidatedTrajectory *metadata,
                             const GpTrajectoryIdentity *id)
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
            crc = (crc >> 1U) ^ (GP_CRC32_POLYNOMIAL &
                                 (uint32_t)-(int32_t)(crc & 1U));
    }
    return crc;
}

bool gp_playback_m7_request(GpValidationShared *v, GpTrajectoryShared *t,
                            const GpTrajectoryIdentity *identity)
{
    if (!gp_validation_m4_idle(v) || !gp_traj_shared_valid(t) ||
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

void gp_playback_m7_cancel(GpValidationShared *v)
{
    if (!gp_validation_shared_valid(v)) return;
    const uint32_t seq = __atomic_load_n(&v->playback_request_seq, ACQUIRE);
    if (seq != 0U)
        __atomic_store_n(&v->playback_cancel_seq, seq, RELEASE);
}

bool gp_playback_m4_init(GpPlaybackM4Worker *w,
                         GpValidationShared *v,
                         GpTrajectoryShared *t,
                         const GpCommittedTrajectoryReader *reader)
{
    if (w == NULL || !gp_validation_shared_valid(v) ||
        !gp_traj_shared_valid(t) || reader == NULL ||
        reader->read_metadata == NULL || reader->read_samples == NULL ||
        reader->capacity_samples == 0U ||
        __atomic_load_n(&v->playback_request_seq, ACQUIRE) != 0U ||
        __atomic_load_n(&v->playback_ack_seq, ACQUIRE) != 0U)
        return false;
    memset(w, 0, sizeof(*w));
    w->validation = v;
    w->reader = *reader;
    gp_traj_m4_init(&w->producer, t);
    w->initialized = true;
    return true;
}

static bool storage_read(uint32_t first, GpTrajectorySample *out,
                         uint32_t count, void *context)
{
    GpPlaybackM4Worker *w = context;
    if (!w->initialized || count == 0U || count > GP_TRAJ_BATCH ||
        first > w->metadata.sample_count ||
        count > w->metadata.sample_count - first)
        return false;
    return w->reader.read_samples(first, out, count, w->reader.context);
}

static void fail(GpPlaybackM4Worker *w)
{
    __atomic_store_n(&w->validation->playback_status, GP_PLAYBACK_FAILED, RELEASE);
}

bool gp_playback_m4_step(GpPlaybackM4Worker *w)
{
    if (w == NULL || !w->initialized ||
        !gp_validation_shared_valid(w->validation) ||
        !gp_traj_shared_valid(w->producer.shared))
        return false;
    GpValidationShared *v = w->validation;
    const uint32_t requested = __atomic_load_n(&v->playback_request_seq, ACQUIRE);
    const uint32_t cancel = __atomic_load_n(&v->playback_cancel_seq, ACQUIRE);
    const uint32_t status = __atomic_load_n(&v->playback_status, ACQUIRE);
    if (status == GP_PLAYBACK_IDLE) {
        if (requested == __atomic_load_n(&v->playback_ack_seq, ACQUIRE))
            return true;
        /* Serialized with validation writes by the same M4 task. */
        if (__atomic_load_n(&v->worker_ready, ACQUIRE) != 1U ||
            __atomic_load_n(&v->flash_status, ACQUIRE) != GP_VALIDATION_FLASH_IDLE ||
            __atomic_load_n(&v->head, ACQUIRE) !=
                __atomic_load_n(&v->tail, ACQUIRE)) {
            fail(w);
            __atomic_store_n(&v->playback_ack_seq, requested, RELEASE);
            return false;
        }
        w->request_seq = requested;
        w->requested = v->playback_identity;
        __atomic_store_n(&v->playback_ack_seq, requested, RELEASE);
        if (cancel == requested) return true; /* handled below on next step */
        if (!identity_valid(&w->requested) ||
            w->requested.sample_count > w->reader.capacity_samples ||
            !w->reader.read_metadata(&w->metadata, w->reader.context) ||
            !metadata_matches(&w->metadata, &w->requested)) {
            fail(w);
            return false;
        }
        w->verify_crc = UINT32_MAX;
        w->verify_index = 0U;
        __atomic_store_n(&v->playback_status, GP_PLAYBACK_VERIFYING, RELEASE);
        return true;
    }

    if (status == GP_PLAYBACK_VERIFYING) {
        if (cancel == w->request_seq) {
            __atomic_store_n(&v->playback_status, GP_PLAYBACK_IDLE, RELEASE);
            return true;
        }
        if (w->verify_index < w->requested.sample_count) {
            uint32_t count = w->requested.sample_count - w->verify_index;
            if (count > GP_TRAJ_BATCH) count = GP_TRAJ_BATCH;
            if (!storage_read(w->verify_index, w->verify_buffer, count, w)) {
                fail(w);
                return false;
            }
            w->verify_crc = crc32_step(w->verify_crc, w->verify_buffer,
                                      (size_t)count * sizeof(GpTrajectorySample));
            w->verify_index += count;
            return true;
        }
        /* A COMPLETE readback/CRC precedes publication of even sample zero. */
        if ((~w->verify_crc) != w->metadata.sample_data_crc ||
            !gp_traj_m4_prepare(&w->producer, storage_read, w, &w->requested)) {
            fail(w);
            return false;
        }
        __atomic_store_n(&v->playback_status, GP_PLAYBACK_STREAMING, RELEASE);
        return true;
    }

    if (status == GP_PLAYBACK_STREAMING || status == GP_PLAYBACK_FAILED) {
        /* M7 publishes cancellation only AFTER the execution state has
         * stopped using the ring. M4 can now abandon a prepared-but-unused
         * epoch even when M7 never called gp_traj_m7_begin(). */
        if (cancel == w->request_seq) {
            if (w->producer.active) {
                GpTrajectoryShared *t = w->producer.shared;
                __atomic_store_n(&t->ready, 0U, RELEASE);
                w->producer.active = false;
                __atomic_store_n(&t->release_ack_epoch, w->producer.epoch, RELEASE);
            }
            __atomic_store_n(&v->playback_status, GP_PLAYBACK_IDLE, RELEASE);
            return true;
        }
        /* Release ack must be processed after any in-flight flash callback.
         * A failed producer also processes release before returning failure. */
        if (w->producer.active) {
            if (!gp_traj_m4_step(&w->producer)) {
                fail(w);
                return false;
            }
            if (w->producer.active) return true;
        }
        if (status == GP_PLAYBACK_STREAMING &&
            __atomic_load_n(&w->producer.shared->read_error, ACQUIRE) != 0U) {
            fail(w);
            return false;
        }
        if (cancel == w->request_seq || status == GP_PLAYBACK_STREAMING) {
            __atomic_store_n(&v->playback_status, GP_PLAYBACK_IDLE, RELEASE);
            return true;
        }
        return false; /* A failure persists until M7 cancels. */
    }
    fail(w);
    return false;
}
