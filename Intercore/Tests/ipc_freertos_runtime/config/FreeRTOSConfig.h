#ifndef IPC_RUNTIME_TEST_FREERTOS_CONFIG_H
#define IPC_RUNTIME_TEST_FREERTOS_CONFIG_H

/* Inherit the existing project host configuration, but build an isolated
 * FreeRTOS kernel instance with static allocation enabled for MessageBuffers.
 * Never use this configuration in the production Cortex-M7/M4 firmware. */
#include "../../../../FreeRTOSConfig.h"
#undef configSUPPORT_STATIC_ALLOCATION
#define configSUPPORT_STATIC_ALLOCATION 1
#define configUSE_STREAM_BUFFERS 1

/* The test supplies vApplicationGetIdleTaskMemory() explicitly. */
#ifdef configKERNEL_PROVIDED_STATIC_MEMORY
#undef configKERNEL_PROVIDED_STATIC_MEMORY
#endif
#define configKERNEL_PROVIDED_STATIC_MEMORY 0

#endif
