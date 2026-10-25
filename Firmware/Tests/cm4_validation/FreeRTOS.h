#ifndef TEST_FREERTOS_H
#define TEST_FREERTOS_H
#include <stdint.h>
#ifndef configSUPPORT_STATIC_ALLOCATION
#define configSUPPORT_STATIC_ALLOCATION 1
#endif
#ifndef TEST_TICK_HZ
#define TEST_TICK_HZ 1000U
#endif
#define pdMS_TO_TICKS(ms) ((ms) * TEST_TICK_HZ / 1000U)
#define tskIDLE_PRIORITY 0U
#define pdPASS 1
typedef uint32_t StackType_t;
typedef struct { uint32_t unused; } StaticTask_t;
typedef void *TaskHandle_t;
#endif
