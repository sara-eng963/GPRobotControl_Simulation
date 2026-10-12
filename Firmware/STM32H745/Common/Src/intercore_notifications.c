/*
 * STM32H745 dual-FreeRTOS IPC: ST HAL EXTI software-interrupt notifications.
 *
 * Based on the notification principle in ST's STM32CubeH7
 * STM32H745I-DISCO/FreeRTOS_AMP_Dual_RTOS example, adapted to the HAL API
 * shipped with STM32Cube FW_H7 V1.13.0.
 *
 * One shared FreeRTOS Message Buffer per direction:
 *     EXTI0: M7 publishes to M4; only CM4 EXTI0_IRQn is enabled.
 *     EXTI1: M4 publishes to M7; only CM7 EXTI1_IRQn is enabled.
 *
 * We reuse the backend's successful nonblocking-send notifier. No custom
 * FreeRTOS queue implementation or scheduler modifications are required.
 * The ISR uses the vendor xMessageBufferSendCompletedFromISR helper.
 *
 * CRITICAL: ipc_mailbox_freertos.c uses timeout=0 on every operation.
 * Do not introduce blocking calls on shared buffers: the default native
 * FreeRTOS send-completion hook cannot signal a task on the other kernel.
 * This adapter delivers interrupts and a kernel completion opportunity,
 * but application task wait registration is not implemented here.
 */
#define GP_IPC_FREERTOS_BACKEND 1
#include "intercore_notifications.h"

#include <stdint.h>
#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "message_buffer.h"
#include "task.h"
#include "ipc_mailbox.h"
#include "sram4_memory_map.h"

#if !defined(DUAL_CORE)
#error "Intercore EXTI adapter requires the dual-core STM32H745 target"
#endif
#if (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY > 6)
#error "EXTI priority 6 is too urgent for FreeRTOS FromISR calls"
#endif
#if defined(CORE_CM7)
#define IPC_RX_LINE            EXTI_LINE_1
#define IPC_TX_LINE            EXTI_LINE_0
#define IPC_RX_IRQn            EXTI1_IRQn
#define IPC_TX_EVENT           GP_IPC_NOTIFY_M7_TO_M4_DATA
#elif defined(CORE_CM4)
#define IPC_RX_LINE            EXTI_LINE_0
#define IPC_TX_LINE            EXTI_LINE_1
#define IPC_RX_IRQn            EXTI0_IRQn
#define IPC_TX_EVENT           GP_IPC_NOTIFY_M4_TO_M7_DATA
#else
#error "Compile intercore_notifications.c separately for CORE_CM7 or CORE_CM4"
#endif

static EXTI_HandleTypeDef exti_rx;
static EXTI_HandleTypeDef exti_tx;
static MessageBufferHandle_t received_buffer;
static bool notifications_ready;

static void notify_remote_after_send(GpIpcNotifyEvent event, void *context)
{
    (void)context;
    if (notifications_ready && event == IPC_TX_EVENT)
    {
        HAL_EXTI_GenerateSWI(&exti_tx);
    }
}

static void on_peer_data(void)
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    if (notifications_ready && received_buffer != NULL)
    {
        /* Matches ST's EXTI -> FreeRTOS Message Buffer completion pattern.
         * Currently all application receive APIs are nonblocking. */
        (void)xMessageBufferSendCompletedFromISR(
            received_buffer, &higher_priority_task_woken);
    }
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

bool intercore_notifications_initialize(void)
{
    const GpIpcShared *const ipc =
        (const GpIpcShared *)(uintptr_t)GP_SHARED_IPC_BASE;

    if (!gp_ipc_valid(ipc))
        return false;

#if defined(CORE_CM7)
    /* The M7 owns EXTI configuration. Configure BOTH directions before
     * releasing CM4 with ST's existing HSEM-0 boot sequence. */
    EXTI_ConfigTypeDef cfg = {0};

    cfg.Line = EXTI_LINE_0;
    cfg.Mode = EXTI_MODE_CORE2_INTERRUPT;
    cfg.Trigger = EXTI_TRIGGER_RISING;
    cfg.GPIOSel = EXTI_GPIOA;
    if (HAL_EXTI_SetConfigLine(&exti_tx, &cfg) != HAL_OK)
        return false;

    cfg.Line = EXTI_LINE_1;
    cfg.Mode = EXTI_MODE_CORE1_INTERRUPT;
    if (HAL_EXTI_SetConfigLine(&exti_rx, &cfg) != HAL_OK)
        return false;
    received_buffer = ipc->m4_to_m7_handle;
#else
    /* M7 has already configured both EXTI lines in the global EXTI
     * registers. M4 must not reconfigure the shared routing registers. */
    if (HAL_EXTI_GetHandle(&exti_rx, IPC_RX_LINE) != HAL_OK ||
        HAL_EXTI_GetHandle(&exti_tx, IPC_TX_LINE) != HAL_OK)
        return false;
    received_buffer = ipc->m7_to_m4_handle;
#endif

    if (HAL_EXTI_RegisterCallback(&exti_rx, HAL_EXTI_COMMON_CB_ID,
                                  on_peer_data) != HAL_OK)
        return false;

    /* HAL clears only THIS core's pending flag. */
    HAL_EXTI_ClearPending(&exti_rx, EXTI_TRIGGER_RISING);
    HAL_NVIC_SetPriority(IPC_RX_IRQn, 6U, 0U);
    notifications_ready = true;
    gp_ipc_set_notifier(notify_remote_after_send, NULL);
    HAL_NVIC_EnableIRQ(IPC_RX_IRQn);
    return true;
}

void intercore_notifications_handle_irq(void)
{
    if (notifications_ready)
    {
        HAL_EXTI_IRQHandler(&exti_rx);
    }
}
