#ifndef GP_IPC_STM32H745_HSEM_H
#define GP_IPC_STM32H745_HSEM_H

/* This optional STM32 HAL port is compiled separately into EACH core image.
 * Enable exactly one: GP_IPC_CORE_M7 or GP_IPC_CORE_M4. The caller owns
 * HAL_HSEM_FreeCallback and forwards its SemMask into on_free(). */
#include "ipc_mailbox.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>

void gp_ipc_stm32_hsem_setup(TaskHandle_t local_service_task);
void gp_ipc_stm32_hsem_on_free(uint32_t sem_mask);
uint32_t gp_ipc_stm32_hsem_signal_failures(void);

#endif
