#ifndef INTERCORE_SHARED_MEMORY_H
#define INTERCORE_SHARED_MEMORY_H

#include <stdbool.h>

/*
 * Cortex-M7 only. Initialize the shared FreeRTOS Message Buffers and
 * trajectory storage once, before the ST HSEM-0 release wakes Cortex-M4.
 * Do not call when the other core is already accessing shared SRAM4.
 */
bool intercore_shared_memory_initialize(void);

#endif /* INTERCORE_SHARED_MEMORY_H */
