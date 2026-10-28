#ifndef IPC_STM32H745_HSEM_H
#define IPC_STM32H745_HSEM_H

/* This optional STM32 HAL port is compiled separately into EACH core image.
 * Enable exactly one: IPC_CORE_M7 or IPC_CORE_M4. The caller owns
 * HAL_HSEM_FreeCallback and forwards its SemMask into on_free(). */
#include "ipc_mailbox.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>

/* Reserve HSEM IDs 24..27 in BOTH firmware images for IPC. STM32CubeH7
 * startup templates commonly use ID 0 to release CPU2; do not reuse it. */
#define IPC_HSEM_BASE_ID 24U
#define IPC_HSEM_ID(event) (IPC_HSEM_BASE_ID + (uint32_t)(event))

void ipc_stm32_hsem_setup(TaskHandle_t local_service_task);
void ipc_stm32_hsem_on_free(uint32_t sem_mask);
uint32_t ipc_stm32_hsem_signal_failures(void);

#endif
