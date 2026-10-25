#ifndef TRAJECTORY_STREAM_H
#define TRAJECTORY_STREAM_H

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
#define TRAJECTORY_MAGIC UINT32_C(0x47505453)
#define TRAJECTORY_ABI_VERSION UINT32_C(1)
#define TRAJECTORY_CAPACITY 512U
#define TRAJECTORY_BATCH 256U
#define TRAJECTORY_SAMPLE_PERIOD_US UINT32_C(2000)

typedef struct {
    int32_t target_position_units[6];
} TrajectorySample;

typedef struct {
    uint32_t program_id;
    uint32_t source_revision;
    uint32_t artifact_crc;
    uint32_t sample_count;
    uint32_t sample_period_us;
} TrajectoryIdentity;

/* Fields have exactly one writer:
 * M4: epoch, identity, written, read_error, ready, release_ack_epoch, samples.
 * M7: consumed and release_epoch.
 * The ring is SPSC; never run two producers or two consumers simultaneously.
 */
typedef struct {
    uint32_t magic;
    uint32_t abi_version;
    uint32_t epoch;
    TrajectoryIdentity identity;
    uint32_t written;
    uint32_t consumed;
    uint32_t read_error;
    uint32_t ready;
    uint32_t release_epoch;
    uint32_t release_ack_epoch;
    _Alignas(32) TrajectorySample samples[TRAJECTORY_CAPACITY];
} TrajectoryShared;

typedef bool (*TrajectoryReadBatch)(
    uint32_t first, TrajectorySample *out, uint32_t count, void *context);

/* M4-private descriptor; keep outside shared SRAM (e.g., M4 SRAM). */
typedef struct {
    TrajectoryShared *shared;
    TrajectoryReadBatch read_batch;
    void *read_context;
    TrajectorySample staging[TRAJECTORY_BATCH];
    uint32_t epoch;
    uint32_t next_index;
    uint32_t refills;
    bool active;
} TrajectoryProducer;

/* M7-private descriptor; never store in inter-core shared SRAM. */
typedef struct {
    TrajectoryShared *shared;
    TrajectoryIdentity expected;
    uint32_t epoch;
    uint32_t next_index;
    uint32_t underruns;
    bool active;
} TrajectoryConsumer;

typedef enum {
    TRAJECTORY_READ_OK = 0,
    TRAJECTORY_READ_UNAVAILABLE,
    TRAJECTORY_READ_INVALID_INDEX,
    TRAJECTORY_READ_STORAGE_ERROR,
    TRAJECTORY_READ_STARVED
} TrajectoryReadResult;

/* Initialization requires both cores quiescent. Do NOT reset a live stream. */
void trajectory_shared_initialize(TrajectoryShared *shared);
bool trajectory_shared_valid(const TrajectoryShared *shared);
void trajectory_m4_init(TrajectoryProducer *producer, TrajectoryShared *shared);
void trajectory_m7_init(TrajectoryConsumer *consumer, TrajectoryShared *shared);

/* M4 only. Requires STOPPED acknowledgement from the previous epoch.
 * read_batch must access storage from M4 only; may block on flash there. */
bool trajectory_m4_prepare(TrajectoryProducer *producer,
                        TrajectoryReadBatch read_batch, void *context,
                        const TrajectoryIdentity *identity);

/* M4 only. At most one 256-sample storage callback per call.
 * A release request is acknowledged only AFTER an in-flight read returns. */
bool trajectory_m4_step(TrajectoryProducer *producer);

/* M7 only. Fresh starts require min(count,512) buffered samples.
 * Returns immediately if startup data is not yet ready. */
bool trajectory_m7_begin(TrajectoryConsumer *consumer,
                      const TrajectoryIdentity *expected);
TrajectoryReadResult trajectory_m7_take(TrajectoryConsumer *consumer,
                                      uint32_t expected_index,
                                      TrajectorySample *sample);
/* M7-only O(1) discard, bounded by published samples and current epoch. */
TrajectoryReadResult trajectory_m7_skip(TrajectoryConsumer *consumer,
                                      uint32_t expected_index,
                                      uint32_t count);

/* M7 only: after motion has fully stopped reading, publish release.
 * M4 must acknowledge before reusing/resetting the ring. */
bool trajectory_m7_finish(TrajectoryConsumer *consumer);
/* Release a prepared stream that never entered execution (validation gate). */
bool trajectory_m7_discard(TrajectoryConsumer *consumer);
bool trajectory_m4_release_acknowledged(const TrajectoryShared *shared,
                                     uint32_t epoch);

uint32_t trajectory_buffered(const TrajectoryShared *shared);
uint32_t trajectory_current_epoch(const TrajectoryShared *shared);

#endif
