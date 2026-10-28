#ifndef ALIGNMENT_FREERTOS_H
#define ALIGNMENT_FREERTOS_H
/* Deterministic scheduling seam only. No target RTOS/layout/timing claim. */
#include <stdint.h>
#include <stddef.h>
#include <assert.h>
#define configSUPPORT_STATIC_ALLOCATION 1
#define configTICK_RATE_HZ 1000U
#define pdPASS 1
#define pdMS_TO_TICKS(ms) (ms)
#define configASSERT(x) assert(x)
typedef uint32_t TickType_t;
typedef uint32_t StackType_t;
typedef unsigned UBaseType_t;
typedef int BaseType_t;
typedef void *TaskHandle_t;
typedef struct { unsigned unused; } StaticTask_t;
typedef struct { unsigned unused; } StaticQueue_t;
void alignment_enter_critical(void);
void alignment_exit_critical(void);
#define taskENTER_CRITICAL() alignment_enter_critical()
#define taskEXIT_CRITICAL() alignment_exit_critical()
#endif
