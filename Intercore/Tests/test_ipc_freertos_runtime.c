/* Functional test of the REAL FreeRTOS message buffer transport.
 * This binary runs on the FreeRTOS GCC_POSIX port: tasks are scheduled by
 * ONE simulated kernel, not concurrently executing independent STM32 cores.
 * No STM32 HSEM, cache, reset, IRQ priority or shared-SRAM claim is made. */
#include "../../Intercore/ipc_mailbox.h"
#include "FreeRTOS.h"
#include "task.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define GP_STRESS_MESSAGES 2000U
#define GP_WORKER_STACK_WORDS 1024U
#define GP_MAX_WAIT_TICKS pdMS_TO_TICKS(20000U)

static GpIpcShared ipc_shared;
static uint32_t notices[4];
static volatile uint32_t failed;
static volatile uint32_t finished;
static volatile uint32_t done_producers;
static volatile uint32_t done_consumers;

typedef struct {
    uint32_t direction;
    uint32_t producer;
} WorkerArgument;
static WorkerArgument args[4] = {
    {0U, 1U}, {0U, 0U}, {1U, 1U}, {1U, 0U}
};

static void require(int valid, const char *label)
{
    if (!valid) {
        ++failed;
        fprintf(stderr, "FAIL: %s\n", label);
    }
}

static uint32_t payload(uint32_t index, uint32_t word, uint32_t direction)
{
    return (index * UINT32_C(0x45d9f3b) + word * 109U) ^
           (direction ? UINT32_C(0xa5c3e2d1) : UINT32_C(0x27bd512c));
}

static GpIpcMessage make_message(uint32_t i, uint32_t direction)
{
    GpIpcMessage m = {0};
    m.type = direction ? GP_IPC_CONTROLLER_STATUS : GP_IPC_HMI_COMMAND;
    m.sequence = i;
    m.timestamp_ms = 2U * i;
    for (unsigned j = 0; j < GP_IPC_WORDS; ++j)
        m.data[j] = payload(i, j, direction);
    return m;
}

static int valid_message(const GpIpcMessage *m, uint32_t i, uint32_t direction)
{
    if (m->type != (direction ? GP_IPC_CONTROLLER_STATUS : GP_IPC_HMI_COMMAND) ||
        m->sequence != i || m->timestamp_ms != 2U * i)
        return 0;
    for (uint32_t j = 0; j < GP_IPC_WORDS; ++j)
        if (m->data[j] != payload(i, j, direction)) return 0;
    return 1;
}

static bool send_one(uint32_t direction, const GpIpcMessage *m)
{
    return direction ? gp_ipc_m7_send(&ipc_shared, m) :
                       gp_ipc_m4_send(&ipc_shared, m);
}

static bool recv_one(uint32_t direction, GpIpcMessage *m)
{
    return direction ? gp_ipc_m4_receive(&ipc_shared, m) :
                       gp_ipc_m7_receive(&ipc_shared, m);
}

static uint32_t pending(uint32_t direction)
{
    return direction ? gp_ipc_m7_to_m4_pending(&ipc_shared) :
                       gp_ipc_m4_to_m7_pending(&ipc_shared);
}

static void on_notice(GpIpcNotifyEvent event, void *context)
{
    uint32_t *counter = (uint32_t *)context;
    if ((unsigned)event < 4U) ++counter[(unsigned)event];
    else ++failed;
}

static void unit_checks(void)
{
    GpIpcMessage rx = {0};
    GpIpcMessage tx = make_message(42U, 0U);
    require(!gp_ipc_valid(&ipc_shared), "not valid before bootstrap");
    require(!gp_ipc_m4_send(&ipc_shared, &tx), "send rejected before init");
    gp_ipc_initialize(&ipc_shared);
    require(gp_ipc_valid(&ipc_shared), "static message buffer initialization");

    gp_ipc_set_notifier(on_notice, notices);
    require(!gp_ipc_m7_receive(&ipc_shared, &rx), "initial empty M4->M7");
    require(!gp_ipc_m4_receive(&ipc_shared, &rx), "initial empty M7->M4");
    require(!gp_ipc_m4_send(&ipc_shared, NULL), "reject NULL payload");

    tx.type = 99U;
    require(!gp_ipc_m4_send(&ipc_shared, &tx), "reject invalid payload type");

    /* Fill both channels to exact logical depth, then reject an extra item.
     * Exercise real FreeRTOS framing, not the project's old ring. */
    for (uint32_t direction = 0U; direction < 2U; ++direction) {
        for (uint32_t i = 1U; i <= GP_IPC_RING_CAPACITY; ++i) {
            tx = make_message(i, direction);
            require(send_one(direction, &tx), "full-buffer fill");
        }
        require(pending(direction) == GP_IPC_RING_CAPACITY,
                "accurate full-buffer message count");
        tx = make_message(100U, direction);
        require(!send_one(direction, &tx), "full-buffer refuses overwrite");
        for (uint32_t i = 1U; i <= GP_IPC_RING_CAPACITY; ++i) {
            require(recv_one(direction, &rx), "drain full buffer");
            require(valid_message(&rx, i, direction), "payload preserved / FIFO");
        }
        require(pending(direction) == 0U, "buffer drained");
        require(!recv_one(direction, &rx), "empty read fails");
    }

    require(notices[GP_IPC_NOTIFY_M4_TO_M7_DATA] == GP_IPC_RING_CAPACITY,
            "M4 data notification only after successful send");
    require(notices[GP_IPC_NOTIFY_M4_TO_M7_SPACE] == GP_IPC_RING_CAPACITY,
            "M7 free-space notification only after successful receive");
    require(notices[GP_IPC_NOTIFY_M7_TO_M4_DATA] == GP_IPC_RING_CAPACITY,
            "M7 data notification only after successful send");
    require(notices[GP_IPC_NOTIFY_M7_TO_M4_SPACE] == GP_IPC_RING_CAPACITY,
            "M4 free-space notification only after successful receive");

    /* Duplicate sequences are transported, not falsely suppressed: sequence
     * rejection belongs to the Supervisor's application protocol. */
    tx = make_message(7U, 0U);
    require(gp_ipc_m4_send(&ipc_shared, &tx), "first sequence");
    require(gp_ipc_m4_send(&ipc_shared, &tx), "duplicate sequence");
    require(gp_ipc_m7_receive(&ipc_shared, &rx) && rx.sequence == 7U,
            "first duplicate read");
    require(gp_ipc_m7_receive(&ipc_shared, &rx) && rx.sequence == 7U,
            "second duplicate read");
    require(pending(0U) == 0U, "duplicate test drained");
}

static void worker_task(void *parameter)
{
    const WorkerArgument a = *(const WorkerArgument *)parameter;
    const TickType_t started = xTaskGetTickCount();

    for (uint32_t i = 1U; i <= GP_STRESS_MESSAGES && !failed; ++i) {
        if (a.producer) {
            const GpIpcMessage tx = make_message(i, a.direction);
            while (!send_one(a.direction, &tx)) {
                if (xTaskGetTickCount() - started > GP_MAX_WAIT_TICKS) {
                    require(0, "producer timeout");
                    goto finished_task;
                }
                vTaskDelay(1U);
            }
        } else {
            GpIpcMessage rx = {0};
            while (!recv_one(a.direction, &rx)) {
                if (xTaskGetTickCount() - started > GP_MAX_WAIT_TICKS) {
                    require(0, "consumer timeout");
                    goto finished_task;
                }
                vTaskDelay(1U);
            }
            if (!valid_message(&rx, i, a.direction)) {
                require(0, "worker FIFO / 12-word payload corruption");
                goto finished_task;
            }
        }
        /* Force interleaving even under scheduler implementations that
         * do not time-slice equal-priority tasks. */
        if ((i & 7U) == 0U) vTaskDelay(1U);
    }

finished_task:
    if (a.producer) ++done_producers;
    else ++done_consumers;
    vTaskDelete(NULL);
}

static void controller_task(void *parameter)
{
    (void)parameter;
    unit_checks();
    if (!failed) {
        const TickType_t start = xTaskGetTickCount();
        for (unsigned i = 0; i < 4U; ++i) {
            const BaseType_t made = xTaskCreate(worker_task, "ipc-worker",
                GP_WORKER_STACK_WORDS, &args[i], tskIDLE_PRIORITY + 2U, NULL);
            require(made == pdPASS, "create worker task");
        }
        if (!failed) {
            while ((done_producers != 2U || done_consumers != 2U) &&
                   !failed &&
                   xTaskGetTickCount() - start < GP_MAX_WAIT_TICKS)
                vTaskDelay(1U);
            require(done_producers == 2U && done_consumers == 2U,
                    "all bidirectional workers completed");
            require(pending(0U) == 0U && pending(1U) == 0U,
                    "no messages left after parallel task test");
            require(notices[GP_IPC_NOTIFY_M4_TO_M7_DATA] ==
                        notices[GP_IPC_NOTIFY_M4_TO_M7_SPACE] &&
                    notices[GP_IPC_NOTIFY_M7_TO_M4_DATA] ==
                        notices[GP_IPC_NOTIFY_M7_TO_M4_SPACE],
                    "notification count balanced");
        }
    }

    if (failed == 0U) {
        puts("PASS: real FreeRTOS Message Buffers, 16-slot backpressure, "
             "notifications, two directions, 2000 messages/direction");
    } else {
        fprintf(stderr, "FAIL: %u runtime IPC assertions\n", (unsigned)failed);
    }
    fflush(stdout);
    fflush(stderr);
    finished = 1U;
    vTaskEndScheduler();
    vTaskDelete(NULL);
}

/* Required by this isolated static-enabled FreeRTOS POSIX kernel build. */
void vApplicationGetIdleTaskMemory(StaticTask_t **tcb, StackType_t **stack,
                                   StackType_t *words)
{
    static StaticTask_t idle_tcb;
    static StackType_t idle_stack[configMINIMAL_STACK_SIZE];
    *tcb = &idle_tcb;
    *stack = idle_stack;
    *words = configMINIMAL_STACK_SIZE;
}

int main(void)
{
    if (xTaskCreate(controller_task, "ipc-test", GP_WORKER_STACK_WORDS,
                    NULL, tskIDLE_PRIORITY + 3U, NULL) != pdPASS) {
        fprintf(stderr, "FAIL: create IPC controller task\n");
        return EXIT_FAILURE;
    }
    (void)vTaskStartScheduler();
    if (!finished || failed) return EXIT_FAILURE;
    return EXIT_SUCCESS;
}
