#ifndef MOCK_FREERTOS_H
#define MOCK_FREERTOS_H
#include <stdint.h>
#include <stdbool.h>
#define configSUPPORT_STATIC_ALLOCATION 1
#define configUSE_STREAM_BUFFERS 1
typedef int BaseType_t;
#define pdFALSE 0
#define pdTRUE 1
#endif
