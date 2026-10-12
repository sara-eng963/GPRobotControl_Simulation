#ifndef INTERCORE_STARTUP_VALIDATION_H
#define INTERCORE_STARTUP_VALIDATION_H

#include <stdbool.h>

/*
 * Cortex-M4: validate the existing Cortex-M7-owned shared FreeRTOS
 * Message Buffers and trajectory storage after the HSEM-0 wakeup.
 * The M4 never initializes or clears either shared object.
 */
bool intercore_startup_validate(void);

#endif /* INTERCORE_STARTUP_VALIDATION_H */
