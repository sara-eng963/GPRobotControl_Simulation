#ifndef P03_FAKE_TASK_H
#define P03_FAKE_TASK_H
#include "FreeRTOS.h"
TickType_t xTaskGetTickCount(void);
void vTaskDelayUntil(TickType_t *last_wake,TickType_t period);
void vTaskDelay(TickType_t periods);
TaskHandle_t xTaskCreateStatic(void (*entry)(void *),const char *name,
 unsigned stack,void *arg,UBaseType_t priority,StackType_t *buf,StaticTask_t *ctl);
#endif
