#ifndef TRAJECTORY_STORAGE_SERVICE_H
#define TRAJECTORY_STORAGE_SERVICE_H

#include "trajectory_stream.h"
#include "validation_storage_bridge.h"

/* Board-owned immutable committed-artifact reader. Readback must use the
 * SAME NOR artifact committed by the M4 validation backend. Neither callback
 * may fabricate a successful read; both execute only on the M4 worker task.
 * Metadata is validated against the M7 identity and the entire sample set
 * is CRC32-verified BEFORE any trajectory epoch is published. */
typedef struct {
    bool (*read_metadata)(ValidatedTrajectory *metadata, void *context);
    bool (*read_samples)(uint32_t first, TrajectorySample *samples,
                         uint32_t count, void *context);
    uint32_t capacity_samples;
    void *context;
} CommittedTrajectoryReader;

/* Private M4 state. Includes 2 x 256-sample staging arrays, no heap.
 * One call performs at most ONE blocking storage read, or ring bookkeeping. */
typedef struct {
    ValidationShared *validation;
    CommittedTrajectoryReader reader;
    TrajectoryProducer producer;
    TrajectorySample verify_buffer[TRAJECTORY_BATCH];
    ValidatedTrajectory metadata;
    TrajectoryIdentity requested;
    uint32_t request_seq;
    uint32_t verify_index;
    uint32_t verify_crc;
    bool initialized;
} PlaybackM4Worker;

bool playback_m4_init(PlaybackM4Worker *worker,
                         ValidationShared *validation,
                         TrajectoryShared *trajectory,
                         const CommittedTrajectoryReader *reader);
bool playback_m4_step(PlaybackM4Worker *worker);

/* M7: O(1) SRAM-only request/cancel, never call NOR or wait on a mutex.
 * The first preflight requests a fresh epoch; subsequent polls inspect
 * the published trajectory via trajectory_m7_begin(). */
bool playback_m7_request(ValidationShared *validation,
                            TrajectoryShared *trajectory,
                            const TrajectoryIdentity *identity);
void playback_m7_cancel(ValidationShared *validation);

#endif
