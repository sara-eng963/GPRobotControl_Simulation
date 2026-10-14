/* M4 owns all physical validation-storage callbacks. Never call from M7.
 * Official STM32Cube HAL NOR adapter is supplied by the board-specific
 * storage provider in Batch B4. Its absence leaves worker_ready = 0. */
#include "application_validation_service.h"
#include "validation_storage_bridge.h"
#include "sram4_memory_map.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define GP_M4_VALIDATION_STACK_WORDS 768U /* provisional; Stage C watermark */
#define GP_M4_VALIDATION_PERIOD_TICKS pdMS_TO_TICKS(1U)
#define GP_M4_VALIDATION_PRIORITY (tskIDLE_PRIORITY + 1U)

/* The board vendor port MUST supply real HAL-backed durable operations.
 * No default fake, RAM backend, NOR register driver, or node IDs. */
extern bool gp_board_m4_supply_validation_backend(
    PathValidationStorage *out) __attribute__((weak));

static GpValidationM4Worker g_worker;
static TaskHandle_t g_worker_handle;
#if (configSUPPORT_STATIC_ALLOCATION == 1)
static StaticTask_t g_worker_control;
static StackType_t g_worker_stack[GP_M4_VALIDATION_STACK_WORDS];
#endif

static void validation_task(void *arg)
{
    (void)arg;
    for (;;) {
        /* One flash callback per call; this task is lower priority than
         * time-sensitive M7 work and separate from M4 HMI IPC handling. */
        (void)gp_validation_m4_step(&g_worker);
        vTaskDelay(GP_M4_VALIDATION_PERIOD_TICKS);
    }
}

bool application_validation_service_start(void)
{
    if (g_worker_handle != NULL ||
        GP_M4_VALIDATION_PERIOD_TICKS == 0U ||
        gp_board_m4_supply_validation_backend == NULL)
        return false;

    GpValidationShared *const shared =
        (GpValidationShared *)(uintptr_t)GP_SHARED_VALIDATION_BASE;
    if (!gp_validation_shared_valid(shared))
        return false;

    PathValidationStorage backend = {0};
    if (!gp_board_m4_supply_validation_backend(&backend) ||
        backend.begin == NULL || backend.write_sample == NULL ||
        backend.commit == NULL || backend.abort == NULL ||
        backend.capacity_samples == 0U)
        return false;

    if (!gp_validation_m4_init(&g_worker, shared, &backend))
        return false;

#if (configSUPPORT_STATIC_ALLOCATION == 1)
    g_worker_handle = xTaskCreateStatic(
        validation_task, "NorValidate", GP_M4_VALIDATION_STACK_WORDS,
        NULL, GP_M4_VALIDATION_PRIORITY,
        g_worker_stack, &g_worker_control);
#else
    if (xTaskCreate(validation_task, "NorValidate",
                    GP_M4_VALIDATION_STACK_WORDS, NULL,
                    GP_M4_VALIDATION_PRIORITY,
                    &g_worker_handle) != pdPASS)
        g_worker_handle = NULL;
#endif
    if (g_worker_handle == NULL) {
        gp_validation_m4_shutdown(&g_worker);
        return false;
    }
    return true;
}
