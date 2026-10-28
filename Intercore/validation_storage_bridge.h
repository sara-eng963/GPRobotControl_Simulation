#ifndef VALIDATION_STORAGE_BRIDGE_H
#define VALIDATION_STORAGE_BRIDGE_H

#include "../StateMachine/States/state_path_validation.h"
#include "validation_storage_wire.h"
#include "trajectory_stream.h"
#include <stdbool.h>
#include <stdint.h>

/* M7 computes geometry/IK. M4 alone calls the underlying flash storage.
 * Shared region is fixed-capacity, SPSC, with no pointers or mutex handles.
 * Place ValidationShared in MPU NON-CACHEABLE memory visible to both cores.
 * Initialize while both cores are quiescent; hardware boot/HSEM is separate.
 */
typedef enum {
    VALIDATION_FLASH_IDLE = 0,
    VALIDATION_FLASH_WRITING = 1,
    VALIDATION_FLASH_COMMITTED = 2,
    VALIDATION_FLASH_FAILED = 3
} ValidationFlashStatus;

typedef enum {
    VALIDATION_OP_BEGIN = 1,
    VALIDATION_OP_SAMPLE = 2,
    VALIDATION_OP_COMMIT = 3
} ValidationOperation;

typedef struct {
    uint32_t kind;
    uint32_t index;
    PvExecutionSample sample;
} ValidationStorageMessage;

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t head;            /* M7 writes */
    uint32_t tail;            /* M4 writes */
    uint32_t flash_status;    /* M4 writes */
    uint32_t cancel_request;  /* M7 writes, M4 acknowledges via status/flag */
    uint32_t release_request; /* M7 writes after observed durable commit */
    uint32_t committed_samples; /* M4 writes, diagnostic only */
    uint32_t worker_ready;     /* M4 publishes only after backend init */
    uint32_t reserved_alignment; /* 8-byte align metadata in wire header */
    ValidatedTrajectory commit_metadata; /* M7 publishes before COMMIT slot */
    ValidationStorageMessage slots[VALIDATION_QUEUE_CAPACITY];
    /* B4: M7 single-writer playback request; M4 single-writer response.
     * These fields are entirely within the existing 12 KiB reservation.
     * M7 never reads NOR or writes the trajectory ring. */
    TrajectoryIdentity playback_identity;
    uint32_t playback_request_seq;
    uint32_t playback_cancel_seq;
    uint32_t playback_ack_seq;
    uint32_t playback_status;
} ValidationShared;

/* These descriptors are PRIVATE to the respective core. */
typedef struct {
    ValidationShared *shared;
    uint32_t next_index;
    bool active;
    bool commit_queued;
} ValidationM7Client;

typedef struct {
    ValidationShared *shared;
    PathValidationStorage backend; /* M4's actual QSPI/NOR callbacks only */
    uint32_t next_index;
    bool open;
} ValidationM4Worker;

void validation_shared_initialize(ValidationShared *shared);
bool validation_shared_valid(const ValidationShared *shared);
void validation_m7_init(ValidationM7Client *client, ValidationShared *shared);
void validation_m4_shutdown(ValidationM4Worker *worker);
bool validation_m4_init(ValidationM4Worker *worker, ValidationShared *shared,
                           const PathValidationStorage *backend);

/* M7: nonblocking adapter for state_path_validation's storage callbacks. */
PathValidationStorage validation_m7_storage(ValidationM7Client *client,
                                               uint32_t capacity_samples);
bool validation_m4_idle(const ValidationShared *shared);

/* M4: run one queued flash operation (or abort) per call.
 * May block inside the backend, NEVER invoke from M7's control task. */
bool validation_m4_step(ValidationM4Worker *worker);

#endif
