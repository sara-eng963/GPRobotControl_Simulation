/* STM32H745 Cortex-M4 application endpoint for Stage 2 IPC.
 * Runs in the existing CMSIS/FreeRTOS defaultTask; uses native FreeRTOS
 * static queues only on the M4 kernel and the established zero-timeout
 * shared FreeRTOS Message Buffers. No new transport or cross-core lock.
 *
 * HMI transport and M7 Supervisor startup are not yet integrated.
 * This module does not itself permit motion or implement a safety channel.
 */
#define GP_IPC_FREERTOS_BACKEND 1
#include "application_ipc_service.h"
#include "ipc_mailbox.h"
#include "sram4_memory_map.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include <stddef.h>

#define APPLICATION_IPC_COMMAND_QUEUE_LENGTH 16U
#define APPLICATION_IPC_MAX_RECEIVES_PER_CYCLE 8U
#define APPLICATION_IPC_PERIOD_TICKS pdMS_TO_TICKS(1U)
#define APPLICATION_IPC_STATUS_TIMEOUT_MS 100U

typedef struct {
    ApplicationControllerStatus status;
    TickType_t local_receipt_tick;
} StatusSnapshot;

static StaticQueue_t command_queue_control;
static uint8_t command_queue_storage[
    APPLICATION_IPC_COMMAND_QUEUE_LENGTH * sizeof(SupervisorHmiCommand)];
static QueueHandle_t command_queue;

static StaticQueue_t status_queue_control;
static uint8_t status_queue_storage[sizeof(StatusSnapshot)];
static QueueHandle_t status_queue;

static GpIpcShared *shared_ipc;
static uint32_t last_status_sequence;
static uint32_t next_command_sequence;

static TickType_t age_ticks(uint32_t max_age_ms)
{
    /* Round UP so a requested age never becomes a zero-tick allowance. */
    if (max_age_ms == 0U) return 0U;
    const uint64_t ticks =
        ((uint64_t)max_age_ms * (uint64_t)configTICK_RATE_HZ + 999ULL) / 1000ULL;
    return (TickType_t)((ticks > (uint64_t)portMAX_DELAY)
                            ? (uint64_t)portMAX_DELAY : ticks);
}

bool application_ipc_service_initialize(void)
{
    if (shared_ipc != NULL || command_queue != NULL || status_queue != NULL)
        return false;

    GpIpcShared *const ipc = (GpIpcShared *)(uintptr_t)GP_SHARED_IPC_BASE;
    if (!gp_ipc_valid(ipc) || APPLICATION_IPC_PERIOD_TICKS == 0U)
        return false;

    QueueHandle_t commands = xQueueCreateStatic(
        APPLICATION_IPC_COMMAND_QUEUE_LENGTH, sizeof(SupervisorHmiCommand),
        command_queue_storage, &command_queue_control);
    QueueHandle_t status = xQueueCreateStatic(
        1U, sizeof(StatusSnapshot),
        status_queue_storage, &status_queue_control);
    if (commands == NULL || status == NULL)
        return false;

    command_queue = commands;
    status_queue = status;
    shared_ipc = ipc;
    return true;
}

bool application_ipc_service_read_status(
    ApplicationControllerStatus *out, uint32_t max_age_ms)
{
    if (out == NULL || status_queue == NULL) return false;
    StatusSnapshot snapshot;
    if (xQueuePeek(status_queue, &snapshot, 0U) != pdPASS) return false;
    if (max_age_ms != 0U &&
        (TickType_t)(xTaskGetTickCount() - snapshot.local_receipt_tick)
            > age_ticks(max_age_ms))
        return false;
    *out = snapshot.status;
    return true;
}

bool application_ipc_service_submit_hmi(SupervisorHmiCommand command)
{
    if (command < SUP_HMI_LINE || command >= SUP_HMI_COMMAND_COUNT ||
        command_queue == NULL)
        return false;

    ApplicationControllerStatus status;
    if (!application_ipc_service_read_status(
            &status, APPLICATION_IPC_STATUS_TIMEOUT_MS))
        return false;

    /* Local queue is multi-writer safe. Only the service task ever sends
     * to the shared M4->M7 Message Buffer. Full queue rejects the request. */
    return xQueueSend(command_queue, &command, 0U) == pdPASS;
}

static void receive_controller_status(void)
{
    GpIpcMessage message;
    for (uint32_t i = 0; i < APPLICATION_IPC_MAX_RECEIVES_PER_CYCLE; ++i) {
        if (!gp_ipc_m4_receive(shared_ipc, &message)) break;
        if (message.type != GP_IPC_CONTROLLER_STATUS ||
            message.sequence == 0U ||
            (int32_t)(message.sequence - last_status_sequence) <= 0 ||
            message.data[1] > 1U || message.data[4] > 1U)
            continue;

        last_status_sequence = message.sequence;
        StatusSnapshot snapshot = {0};
        snapshot.local_receipt_tick = xTaskGetTickCount();
        snapshot.status.ipc_sequence = message.sequence;
        snapshot.status.robot_state = message.data[0];
        snapshot.status.motion_permitted = message.data[1] != 0U;
        snapshot.status.active_fault = message.data[2];
        snapshot.status.transition_count = message.data[3];
        snapshot.status.estop_active = message.data[4] != 0U;
        snapshot.status.last_hmi_received_sequence = message.data[5];
        snapshot.status.rejected_event_count = message.data[6];

        /* Latest status replaces old telemetry, never an HMI command. */
        (void)xQueueOverwrite(status_queue, &snapshot);
    }
}

void application_ipc_service_run(void)
{
    configASSERT(shared_ipc != NULL && command_queue != NULL &&
                 status_queue != NULL);

    SupervisorHmiCommand pending = SUP_HMI_COMMAND_COUNT;
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        /* Polling is mandatory even though the Stage 2 EXTI notification
         * exists: interrupts may coalesce and no remote-kernel task waiter
         * is registered with the native shared Message Buffer. */
        receive_controller_status();

        ApplicationControllerStatus status;
        if (!application_ipc_service_read_status(
                &status, APPLICATION_IPC_STATUS_TIMEOUT_MS)) {
            /* Never deliver HMI commands buffered across controller loss.
             * Queue reset affects only this CM4-local FreeRTOS queue. */
            pending = SUP_HMI_COMMAND_COUNT;
            (void)xQueueReset(command_queue);
        } else {
            if (pending == SUP_HMI_COMMAND_COUNT)
                (void)xQueueReceive(command_queue, &pending, 0U);

            if (pending != SUP_HMI_COMMAND_COUNT) {
                const uint32_t sequence = next_command_sequence + 1U;
                GpIpcMessage command = {0};
                command.type = GP_IPC_HMI_COMMAND;
                command.sequence = (sequence == 0U) ? 1U : sequence;
                command.timestamp_ms = (uint32_t)(
                    ((uint64_t)xTaskGetTickCount() * 1000ULL) /
                    (uint64_t)configTICK_RATE_HZ);
                command.data[0] = (uint32_t)pending;
                if (gp_ipc_m4_send(shared_ipc, &command)) {
                    next_command_sequence = command.sequence;
                    pending = SUP_HMI_COMMAND_COUNT;
                }
                /* Full shared buffer: retain only this pending command;
                 * retry next cycle while status remains fresh. */
            }
        }
        vTaskDelayUntil(&last_wake, APPLICATION_IPC_PERIOD_TICKS);
    }
}
