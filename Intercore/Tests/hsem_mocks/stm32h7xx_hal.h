#ifndef MOCK_HAL_H
#define MOCK_HAL_H
#include <stdint.h>
typedef enum { HAL_OK = 0, HAL_ERROR = 1 } HAL_StatusTypeDef;
HAL_StatusTypeDef HAL_HSEM_FastTake(uint32_t id);
void HAL_HSEM_Release(uint32_t id, uint32_t proc_id);
void HAL_HSEM_ActivateNotification(uint32_t mask);
void mock_clock_enable(void);
void mock_dmb(void);
#define __HAL_RCC_HSEM_CLK_ENABLE() mock_clock_enable()
#define __DMB() mock_dmb()
#endif
