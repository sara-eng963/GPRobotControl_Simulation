#ifndef GP_VALIDATION_STORAGE_BRIDGE_H
#define GP_VALIDATION_STORAGE_BRIDGE_H

#include "../StateMachine/States/state_path_validation.h"
#include "validation_storage_wire.h"
#include <stdbool.h>
#include <stdint.h>

/* M7 computes geometry/IK. M4 alone calls the underlying flash storage.
 * Shared region is fixed-capacity, SPSC, with no pointers or mutex handles.
 * Place GpValidationShared in MPU NON-CACHEABLE memory visible to both cores.
 * Initialize while both cores are quiescent; hardware boot/HSEM is separate.
 */
typedef enum {
    GP_VALIDATION_FLASH_IDLE = 0,
    GP_VALIDATION_FLASH_WRITING = 1,
    GP_VALIDATION_FLASH_COMMITTED = 2,
    GP_VALIDATION_FLASH_FAILED = 3
} GpValidationFlashStatus;

typedef enum {
    GP_VALIDATION_OP_BEGIN = 1,
    GP_VALIDATION_OP_SAMPLE = 2,
    GP_VALIDATION_OP_COMMIT = 3
} GpValidationOperation;

typedef struct {
    uint32_t kind;
    uint32_t index;
    PvExecutionSample sample;
} GpValidationStorageMessage;

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
    GpValidationStorageMessage slots[GP_VALIDATION_QUEUE_CAPACITY];
} GpValidationShared;

/* These descriptors are PRIVATE to the respective core. */
typedef struct {
    GpValidationShared *shared;
    uint32_t next_index;
    bool active;
    bool commit_queued;
} GpValidationM7Client;

typedef struct {
    GpValidationShared *shared;
    PathValidationStorage backend; /* M4's actual QSPI/NOR callbacks only */
    uint32_t next_index;
    bool open;
} GpValidationM4Worker;

void gp_validation_shared_initialize(GpValidationShared *shared);
bool gp_validation_shared_valid(const GpValidationShared *shared);
void gp_validation_m7_init(GpValidationM7Client *client, GpValidationShared *shared);
void gp_validation_m4_shutdown(GpValidationM4Worker *worker);
bool gp_validation_m4_init(GpValidationM4Worker *worker, GpValidationShared *shared,
                           const PathValidationStorage *backend);

/* M7: nonblocking adapter for state_path_validation's storage callbacks. */
PathValidationStorage gp_validation_m7_storage(GpValidationM7Client *client,
                                               uint32_t capacity_samples);
bool gp_validation_m4_idle(const GpValidationShared *shared);

/* M4: run one queued flash operation (or abort) per call.
 * May block inside the backend, NEVER invoke from M7's control task. */
bool gp_validation_m4_step(GpValidationM4Worker *worker);

#endif
