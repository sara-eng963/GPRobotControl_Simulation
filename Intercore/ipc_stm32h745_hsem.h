#ifndef GP_IPC_STM32H745_HSEM_H
#define GP_IPC_STM32H745_HSEM_H

/* This optional STM32 HAL port is compiled separately into EACH core image.
 * Enable exactly one: GP_IPC_CORE_M7 or GP_IPC_CORE_M4. The caller owns
 * HAL_HSEM_FreeCallback and forwards its SemMask into on_free(). */
#include "ipc_mailbox.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>

/* Reserve HSEM IDs 24..27 in BOTH firmware images for IPC. STM32CubeH7
 * startup templates commonly use ID 0 to release CPU2; do not reuse it. */
#define GP_IPC_HSEM_BASE_ID 24U
#define GP_IPC_HSEM_ID(event) (GP_IPC_HSEM_BASE_ID + (uint32_t)(event))

void gp_ipc_stm32_hsem_setup(TaskHandle_t local_service_task);
void gp_ipc_stm32_hsem_on_free(uint32_t sem_mask);
uint32_t gp_ipc_stm32_hsem_signal_failures(void);

#endif
