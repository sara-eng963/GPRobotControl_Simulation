#include "FreeRTOS.h"
TaskHandle_t xTaskCreateStatic(void (*task)(void *), const char *name,
    uint32_t stack, void *arg, uint32_t priority, StackType_t *buffer,
    StaticTask_t *control);
int xTaskCreate(void (*task)(void *), const char *name, uint32_t stack,
    void *arg, uint32_t priority, TaskHandle_t *handle);
void vTaskDelay(uint32_t ticks);
