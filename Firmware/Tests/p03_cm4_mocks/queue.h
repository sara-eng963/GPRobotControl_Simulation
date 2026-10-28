#ifndef P03_FAKE_QUEUE_H
#define P03_FAKE_QUEUE_H
#include "FreeRTOS.h"
typedef void * QueueHandle_t;
QueueHandle_t xQueueCreateStatic(UBaseType_t n,UBaseType_t size,uint8_t *storage,StaticQueue_t *ctl);
BaseType_t xQueuePeek(QueueHandle_t q,void *out,TickType_t wait);
BaseType_t xQueueOverwrite(QueueHandle_t q,const void *data);
BaseType_t xQueueSend(QueueHandle_t q,const void *data,TickType_t wait);
BaseType_t xQueueReceive(QueueHandle_t q,void *out,TickType_t wait);
BaseType_t xQueueReset(QueueHandle_t q);
#endif
