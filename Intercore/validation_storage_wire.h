#ifndef GP_VALIDATION_STORAGE_WIRE_H
#define GP_VALIDATION_STORAGE_WIRE_H
#include <stdint.h>
#define GP_VALIDATION_MAGIC UINT32_C(0x47505653)
#define GP_VALIDATION_VERSION UINT32_C(2)
#define GP_VALIDATION_QUEUE_CAPACITY 64U
/* Header/prefix readable by M4 startup without ControlCore headers. */
typedef struct {
    uint32_t magic, version, head, tail, flash_status;
    uint32_t cancel_request, release_request, committed_samples;
    uint32_t worker_ready;
    uint32_t reserved_alignment;
} GpValidationWireHeader;
#endif
