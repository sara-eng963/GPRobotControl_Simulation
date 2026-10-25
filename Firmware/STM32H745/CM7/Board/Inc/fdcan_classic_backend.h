#ifndef STM32_FDCAN_CLASSIC_BACKEND_H
#define STM32_FDCAN_CLASSIC_BACKEND_H

/*
 * STM32CubeH7 HAL1 transport for the existing CanBackend abstraction.
 * This header is intentionally unavailable until STM32CubeMX generates
 * FDCAN for CM7. Never enable HAL_FDCAN manually in a vendor header.
 */
#include "main.h"
#ifndef HAL_FDCAN_MODULE_ENABLED
#error "Enable CM7 FDCAN via STM32CubeMX before using this board adapter"
#endif

#include "can_backend.h"
#include <stdbool.h>

typedef struct
{
    FDCAN_HandleTypeDef *handle;
    bool started;
} FdcanClassicBackend;

/* The FDCAN instance, kernel clock, timing, GPIO pins and message RAM are
 * exclusively configured by CubeMX and HAL_FDCAN_Init().
 * Requires normal-mode 11-bit Classic CAN and hardware FIFOs >= 16 frames.
 * Configures ST's global filters, then starts the HAL peripheral.
 * No CANopen/servo initialization or motor motion is performed here.
 *
 * Caller keeps both state and interface objects alive; use from the
 * single CM7 CANopen supervisor owner task (not ISR).
 */
bool fdcan_classic_backend_open(FdcanClassicBackend *state,
                                   CanBackend *interface,
                                   FDCAN_HandleTypeDef *fdcan);

#endif /* STM32_FDCAN_CLASSIC_BACKEND_H */
