/* Runs the unmodified production HSEM adapter against HAL/FreeRTOS doubles.
 * Tests control flow only. The real HAL binary and interrupts are NOT tested. */
#include "ipc_stm32h745_hsem.h"
#include "stm32h7xx_hal.h"
#include <assert.h>
#include <stdio.h>
#include <stdint.h>

static GpIpcNotifyFn callback;
static void *callback_ctx;
static uint32_t mask_log[8], mask_n, clock_calls, barrier_calls;
static uint32_t take_count, release_count, last_id, last_proc;
static uint32_t fail_take, notify_count, notified_bits, yielded;
static TaskHandle_t notified_task;
static eNotifyAction notified_action;
static UBaseType_t notified_index;

void gp_ipc_set_notifier(GpIpcNotifyFn cb, void *context)
{ callback = cb; callback_ctx = context; }
void mock_clock_enable(void) { ++clock_calls; }
void mock_dmb(void) { ++barrier_calls; }
HAL_StatusTypeDef HAL_HSEM_FastTake(uint32_t id)
{ ++take_count; last_id = id; return fail_take ? HAL_ERROR : HAL_OK; }
void HAL_HSEM_Release(uint32_t id, uint32_t proc_id)
{ ++release_count; last_id = id; last_proc = proc_id; }
void HAL_HSEM_ActivateNotification(uint32_t mask)
{ assert(mask_n < 8U); mask_log[mask_n++] = mask; }
BaseType_t xTaskGenericNotifyFromISR(TaskHandle_t task,
    UBaseType_t index, uint32_t bits, eNotifyAction action,
    uint32_t *previous, BaseType_t *awoken)
{
    ++notify_count;
    assert(previous == NULL);
    notified_index = index;
    notified_task = task;
    notified_bits = bits;
    notified_action = action;
    *awoken = pdTRUE;
    return pdTRUE;
}
void vPortYield(void) { ++yielded; }

int main(void)
{
#if defined(GP_IPC_CORE_M7)
    const uint32_t incoming = (1U << GP_IPC_HSEM_ID(GP_IPC_NOTIFY_M4_TO_M7_DATA) |
                              (1U << GP_IPC_HSEM_ID(GP_IPC_NOTIFY_M7_TO_M4_SPACE);
    const uint32_t irrelevant = 1U << GP_IPC_HSEM_ID(GP_IPC_NOTIFY_M7_TO_M4_DATA;
#else
    const uint32_t incoming = (1U << GP_IPC_HSEM_ID(GP_IPC_NOTIFY_M7_TO_M4_DATA) |
                              (1U << GP_IPC_HSEM_ID(GP_IPC_NOTIFY_M4_TO_M7_SPACE);
    const uint32_t irrelevant = 1U << GP_IPC_HSEM_ID(GP_IPC_NOTIFY_M4_TO_M7_DATA;
#endif
    assert(GP_IPC_HSEM_BASE_ID > 3U);
    assert(((incoming | irrelevant) & 1U) == 0U);
    const TaskHandle_t task = (TaskHandle_t)(uintptr_t)0x1234U;
    gp_ipc_stm32_hsem_setup(task);
    assert(clock_calls == 1U && callback != NULL &&
           mask_n == 1U && mask_log[0] == incoming);
    assert(gp_ipc_stm32_hsem_signal_failures() == 0U);

    for (unsigned id = 0; id < 4U; ++id) {
        callback((GpIpcNotifyEvent)id, callback_ctx);
        assert(take_count == id + 1U && release_count == id + 1U &&
               last_id == GP_IPC_HSEM_ID(id) && last_proc == 0U);
    }
    assert(barrier_calls == 4U);
    fail_take = 1U;
    callback(GP_IPC_NOTIFY_M7_TO_M4_DATA, callback_ctx);
    assert(take_count == 5U && release_count == 4U &&
           gp_ipc_stm32_hsem_signal_failures() == 1U &&
           barrier_calls == 4U);
    fail_take = 0U;

    gp_ipc_stm32_hsem_on_free(irrelevant | 1U);
    assert(mask_n == 1U && notify_count == 0U && yielded == 0U);
    gp_ipc_stm32_hsem_on_free(incoming | irrelevant);
    assert(mask_n == 2U && mask_log[1] == incoming &&
           notify_count == 1U && notified_task == task &&
           notified_bits == incoming && notified_action == eSetBits &&
           notified_index == 0U &&
           yielded == 1U && barrier_calls == 5U);

    gp_ipc_stm32_hsem_setup(NULL);
    assert(mask_n == 3U && mask_log[2] == incoming && clock_calls == 2U);
    gp_ipc_stm32_hsem_on_free(incoming);
    assert(mask_n == 4U && mask_log[3] == incoming &&
           notify_count == 1U && yielded == 1U);
    puts("PASS: HSEM adapter mock tests (mask, HAL, fallback, ISR, both core configurations)");
    return 0;
}
