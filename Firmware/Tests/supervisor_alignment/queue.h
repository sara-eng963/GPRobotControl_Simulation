#ifndef ALIGNMENT_QUEUE_H
#define ALIGNMENT_QUEUE_H
#include "FreeRTOS.h"
typedef void *QueueHandle_t;
QueueHandle_t xQueueCreateStatic(UBaseType_t count, UBaseType_t size,
    uint8_t *storage, StaticQueue_t *control);
BaseType_t xQueueSend(QueueHandle_t q, const void *item, TickType_t wait);
BaseType_t xQueueSendFromISR(QueueHandle_t q, const void *item, BaseType_t *woken);
BaseType_t xQueueReceive(QueueHandle_t q, void *item, TickType_t wait);
#endif
