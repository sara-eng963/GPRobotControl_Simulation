/* STM32CubeH7 HAL/HSEM port for the nonblocking FreeRTOS IPC backend.
 * No direct peripheral-register programming. HAL calls covered by host mocks. */
#include "ipc_stm32h745_hsem.h"
#include "stm32h7xx_hal.h"

#if !defined(GP_IPC_FREERTOS_BACKEND)
#error "HSEM port requires GP_IPC_FREERTOS_BACKEND"
#endif
#if (defined(GP_IPC_CORE_M7) && defined(GP_IPC_CORE_M4)) || \
    (!defined(GP_IPC_CORE_M7) && !defined(GP_IPC_CORE_M4))
#error "Define exactly one GP_IPC_CORE_M7 or GP_IPC_CORE_M4"
#endif

_Static_assert(GP_IPC_HSEM_BASE_ID + GP_IPC_NOTIFY_M4_TO_M7_SPACE < 32U,
               "IPC HSEM IDs must fit in 32-bit mask");
#define GP_IPC_HSEM_BIT(id) (UINT32_C(1) << (uint32_t)(id))
#if defined(GP_IPC_CORE_M7)
#define GP_IPC_INCOMING_MASK \
    (GP_IPC_HSEM_BIT(GP_IPC_HSEM_ID(GP_IPC_NOTIFY_M4_TO_M7_DATA)) | \
     GP_IPC_HSEM_BIT(GP_IPC_HSEM_ID(GP_IPC_NOTIFY_M7_TO_M4_SPACE)))
#else
#define GP_IPC_INCOMING_MASK \
    (GP_IPC_HSEM_BIT(GP_IPC_HSEM_ID(GP_IPC_NOTIFY_M7_TO_M4_DATA)) | \
     GP_IPC_HSEM_BIT(GP_IPC_HSEM_ID(GP_IPC_NOTIFY_M4_TO_M7_SPACE)))
#endif

static TaskHandle_t local_service_task;
static uint32_t signal_failures;

static void stm32_hsem_notify(GpIpcNotifyEvent event, void *unused)
{
    (void)unused;
    /* Only unique, reserved HSEM IDs are used. No wait/retry on M7 path. */
    if (HAL_HSEM_FastTake(GP_IPC_HSEM_ID(event)) == HAL_OK) {
        __DMB();
        HAL_HSEM_Release(GP_IPC_HSEM_ID(event), 0U);
    } else {
        __atomic_add_fetch(&signal_failures, 1U, __ATOMIC_RELAXED);
        /* Message remains queued; consumer must also poll/recheck. */
    }
}

void gp_ipc_stm32_hsem_setup(TaskHandle_t task)
{
    local_service_task = task; /* core-local address; never put in shared SRAM */
    __HAL_RCC_HSEM_CLK_ENABLE();
    gp_ipc_set_notifier(stm32_hsem_notify, NULL);
    HAL_HSEM_ActivateNotification(GP_IPC_INCOMING_MASK);
    /* Caller must enable NVIC HSEM IRQ at a FreeRTOS-API-safe priority. */
}

void gp_ipc_stm32_hsem_on_free(uint32_t sem_mask)
{
    const uint32_t matched = sem_mask & GP_IPC_INCOMING_MASK;
    if (matched == 0U) return;
    __DMB();
    /* The HAL clears its HSEM pending flags before invoking FreeCallback. */
    HAL_HSEM_ActivateNotification(matched);
    if (local_service_task != NULL) {
        BaseType_t awakened = pdFALSE;
        xTaskNotifyFromISR(local_service_task, matched, eSetBits, &awakened);
        portYIELD_FROM_ISR(awakened);
    }
}

uint32_t gp_ipc_stm32_hsem_signal_failures(void)
{
    return __atomic_load_n(&signal_failures, __ATOMIC_RELAXED);
}
