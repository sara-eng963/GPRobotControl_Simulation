#ifndef GP_TRAJECTORY_STORAGE_SERVICE_H
#define GP_TRAJECTORY_STORAGE_SERVICE_H

#include "trajectory_stream.h"
#include "validation_storage_bridge.h"

/* Board-owned immutable committed-artifact reader. Readback must use the
 * SAME NOR artifact committed by the M4 validation backend. Neither callback
 * may fabricate a successful read; both execute only on the M4 worker task.
 * Metadata is validated against the M7 identity and the entire sample set
 * is CRC32-verified BEFORE any trajectory epoch is published. */
typedef struct {
    bool (*read_metadata)(ValidatedTrajectory *metadata, void *context);
    bool (*read_samples)(uint32_t first, GpTrajectorySample *samples,
                         uint32_t count, void *context);
    uint32_t capacity_samples;
    void *context;
} GpCommittedTrajectoryReader;

/* Private M4 state. Includes 2 x 256-sample staging arrays, no heap.
 * One call performs at most ONE blocking storage read, or ring bookkeeping. */
typedef struct {
    GpValidationShared *validation;
    GpCommittedTrajectoryReader reader;
    GpTrajectoryProducer producer;
    GpTrajectorySample verify_buffer[GP_TRAJ_BATCH];
    ValidatedTrajectory metadata;
    GpTrajectoryIdentity requested;
    uint32_t request_seq;
    uint32_t verify_index;
    uint32_t verify_crc;
    bool initialized;
} GpPlaybackM4Worker;

bool gp_playback_m4_init(GpPlaybackM4Worker *worker,
                         GpValidationShared *validation,
                         GpTrajectoryShared *trajectory,
                         const GpCommittedTrajectoryReader *reader);
bool gp_playback_m4_step(GpPlaybackM4Worker *worker);

/* M7: O(1) SRAM-only request/cancel, never call NOR or wait on a mutex.
 * The first preflight requests a fresh epoch; subsequent polls inspect
 * the published trajectory via gp_traj_m7_begin(). */
bool gp_playback_m7_request(GpValidationShared *validation,
                            GpTrajectoryShared *trajectory,
                            const GpTrajectoryIdentity *identity);
void gp_playback_m7_cancel(GpValidationShared *validation);

#endif
