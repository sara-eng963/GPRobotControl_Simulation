#ifndef GP_TRAJECTORY_STREAM_H
#define GP_TRAJECTORY_STREAM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Hardware contract: 512 x 24-byte samples reside in non-cacheable SRAM
 * visible to both STM32H745 cores. M4's 256-sample flash staging buffer and
 * flash callback stay PRIVATE to M4; no executable pointers cross cores.
 *
 * The two cores must complete the startup/MPU handshake before use. These
 * routines do not configure the MPU, cache, linker or STM32 HSEM.
 */
#define GP_TRAJ_MAGIC UINT32_C(0x47505453)
#define GP_TRAJ_ABI_VERSION UINT32_C(1)
#define GP_TRAJ_CAPACITY 512U
#define GP_TRAJ_BATCH 256U
#define GP_TRAJ_SAMPLE_PERIOD_US UINT32_C(2000)

typedef struct {
    int32_t target_position_units[6];
} GpTrajectorySample;

typedef struct {
    uint32_t program_id;
    uint32_t source_revision;
    uint32_t artifact_crc;
    uint32_t sample_count;
    uint32_t sample_period_us;
} GpTrajectoryIdentity;

/* Fields have exactly one writer:
 * M4: epoch, identity, written, read_error, ready, release_ack_epoch, samples.
 * M7: consumed and release_epoch.
 * The ring is SPSC; never run two producers or two consumers simultaneously.
 */
typedef struct {
    uint32_t magic;
    uint32_t abi_version;
    uint32_t epoch;
    GpTrajectoryIdentity identity;
    uint32_t written;
    uint32_t consumed;
    uint32_t read_error;
    uint32_t ready;
    uint32_t release_epoch;
    uint32_t release_ack_epoch;
    _Alignas(32) GpTrajectorySample samples[GP_TRAJ_CAPACITY];
} GpTrajectoryShared;

typedef bool (*GpTrajectoryReadBatch)(
    uint32_t first, GpTrajectorySample *out, uint32_t count, void *context);

/* M4-private descriptor; keep outside shared SRAM (e.g., M4 SRAM). */
typedef struct {
    GpTrajectoryShared *shared;
    GpTrajectoryReadBatch read_batch;
    void *read_context;
    GpTrajectorySample staging[GP_TRAJ_BATCH];
    uint32_t epoch;
    uint32_t next_index;
    uint32_t refills;
    bool active;
} GpTrajectoryProducer;

/* M7-private descriptor; never store in inter-core shared SRAM. */
typedef struct {
    GpTrajectoryShared *shared;
    GpTrajectoryIdentity expected;
    uint32_t epoch;
    uint32_t next_index;
    uint32_t underruns;
    bool active;
} GpTrajectoryConsumer;

typedef enum {
    GP_TRAJ_READ_OK = 0,
    GP_TRAJ_READ_UNAVAILABLE,
    GP_TRAJ_READ_INVALID_INDEX,
    GP_TRAJ_READ_STORAGE_ERROR,
    GP_TRAJ_READ_STARVED
} GpTrajectoryReadResult;

/* Initialization requires both cores quiescent. Do NOT reset a live stream. */
void gp_traj_shared_initialize(GpTrajectoryShared *shared);
bool gp_traj_shared_valid(const GpTrajectoryShared *shared);
void gp_traj_m4_init(GpTrajectoryProducer *producer, GpTrajectoryShared *shared);
void gp_traj_m7_init(GpTrajectoryConsumer *consumer, GpTrajectoryShared *shared);

/* M4 only. Requires STOPPED acknowledgement from the previous epoch.
 * read_batch must access storage from M4 only; may block on flash there. */
bool gp_traj_m4_prepare(GpTrajectoryProducer *producer,
                        GpTrajectoryReadBatch read_batch, void *context,
                        const GpTrajectoryIdentity *identity);

/* M4 only. At most one 256-sample storage callback per call.
 * A release request is acknowledged only AFTER an in-flight read returns. */
bool gp_traj_m4_step(GpTrajectoryProducer *producer);

/* M7 only. Fresh starts require min(count,512) buffered samples.
 * Returns immediately if startup data is not yet ready. */
bool gp_traj_m7_begin(GpTrajectoryConsumer *consumer,
                      const GpTrajectoryIdentity *expected);
GpTrajectoryReadResult gp_traj_m7_take(GpTrajectoryConsumer *consumer,
                                      uint32_t expected_index,
                                      GpTrajectorySample *sample);
/* M7-only O(1) discard, bounded by published samples and current epoch. */
GpTrajectoryReadResult gp_traj_m7_skip(GpTrajectoryConsumer *consumer,
                                      uint32_t expected_index,
                                      uint32_t count);

/* M7 only: after motion has fully stopped reading, publish release.
 * M4 must acknowledge before reusing/resetting the ring. */
bool gp_traj_m7_finish(GpTrajectoryConsumer *consumer);
/* Release a prepared stream that never entered execution (validation gate). */
bool gp_traj_m7_discard(GpTrajectoryConsumer *consumer);
bool gp_traj_m4_release_acknowledged(const GpTrajectoryShared *shared,
                                     uint32_t epoch);

uint32_t gp_traj_buffered(const GpTrajectoryShared *shared);
uint32_t gp_traj_current_epoch(const GpTrajectoryShared *shared);

#endif
