#ifndef P03_FAKE_RTOS_H
#define P03_FAKE_RTOS_H
#include <stdint.h>
#include <stddef.h>
#include <assert.h>
#define configSUPPORT_STATIC_ALLOCATION 1
#define configTICK_RATE_HZ 1000U
#define portMAX_DELAY UINT32_MAX
#define pdPASS 1
#define pdFAIL 0
#define pdMS_TO_TICKS(ms) (ms)
#define configASSERT(x) assert(x)
typedef uint32_t TickType_t;
typedef uint32_t StackType_t;
typedef unsigned UBaseType_t;
typedef int BaseType_t;
typedef void * TaskHandle_t;
typedef struct { int unused; } StaticTask_t;
typedef struct { int unused; } StaticQueue_t;
void p03_enter_critical(void);
void p03_exit_critical(void);
#define taskENTER_CRITICAL() p03_enter_critical()
#define taskEXIT_CRITICAL() p03_exit_critical()
#endif
