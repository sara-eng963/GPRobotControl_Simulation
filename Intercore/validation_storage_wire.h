#ifndef VALIDATION_STORAGE_WIRE_H
#define VALIDATION_STORAGE_WIRE_H
#include <stdint.h>
#define VALIDATION_MAGIC UINT32_C(0x47505653)
#define VALIDATION_VERSION UINT32_C(3)
#define VALIDATION_QUEUE_CAPACITY 64U
/* M4 writes state; M7 publishes an immutable identity then request_seq. */
#define PLAYBACK_IDLE UINT32_C(0)
#define PLAYBACK_VERIFYING UINT32_C(1)
#define PLAYBACK_STREAMING UINT32_C(2)
#define PLAYBACK_FAILED UINT32_C(3)
/* Header/prefix readable by M4 startup without ControlCore headers. */
typedef struct {
    uint32_t magic, version, head, tail, flash_status;
    uint32_t cancel_request, release_request, committed_samples;
    uint32_t worker_ready;
    uint32_t reserved_alignment;
} ValidationWireHeader;
#endif
