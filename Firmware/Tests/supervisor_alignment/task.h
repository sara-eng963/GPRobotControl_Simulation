#ifndef ALIGNMENT_TASK_H
#define ALIGNMENT_TASK_H
#include "FreeRTOS.h"
TickType_t xTaskGetTickCount(void);
TaskHandle_t xTaskGetCurrentTaskHandle(void);
void vTaskDelay(TickType_t ticks);
void vTaskDelayUntil(TickType_t *previous, TickType_t ticks);
TaskHandle_t xTaskCreateStatic(void (*entry)(void *), const char *name,
    uint32_t stack, void *arg, UBaseType_t priority, StackType_t *buffer,
    StaticTask_t *control);
#endif
