#ifndef MOCK_TASK_H
#define MOCK_TASK_H
#include "FreeRTOS.h"
typedef void *TaskHandle_t;
#define eSetBits 2
BaseType_t xTaskNotifyFromISR(TaskHandle_t task, uint32_t bits, int action, BaseType_t *awoken);
void mock_yield(BaseType_t should_yield);
#define portYIELD_FROM_ISR(x) mock_yield(x)
#endif
