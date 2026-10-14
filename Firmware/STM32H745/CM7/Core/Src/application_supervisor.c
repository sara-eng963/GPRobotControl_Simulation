/* STM32H745 CM7 Supervisor composition.
 *
 * CubeMX owns FDCAN1 clocks, pins, Message RAM and HAL_FDCAN_Init.
 * Existing FreeRTOS IPC and application StateMachine remain the source of
 * truth. This file only joins a REAL board-provided configuration to the
 * CANopen master through the official ST HAL1 Classic CAN adapter.
 *
 * No physical board provider is supplied by default: fail closed.
 */
#define GP_IPC_FREERTOS_BACKEND 1
#include "application_supervisor.h"
#include "board_commissioning.h"
#include "fdcan_classic_backend.h"
#include "ipc_mailbox.h"
#include "sram4_memory_map.h"
#include <stdint.h>
#include <string.h>

/* CubeMX-generated in CM7/Core/Src/main.c. Never reinitialize it here. */
extern FDCAN_HandleTypeDef hfdcan1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

/* TIM2 1 MHz 32-bit overflow extension; only the ISR increments the high
 * word. Supervisor task reads it under FreeRTOS task critical section. */
static volatile uint32_t g_tim2_wraps;
static volatile bool g_timing_running;

static uint64_t gp_time_now_us(void *unused)
{
    (void)unused;
    taskENTER_CRITICAL();  /* Masks the configured TIM2 priority 5. */
    uint32_t high = g_tim2_wraps;
    uint32_t low = __HAL_TIM_GET_COUNTER(&htim2);
    if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE) != RESET) {
        ++high; /* Overflow IRQ pending, but currently masked. */
        low = __HAL_TIM_GET_COUNTER(&htim2);
    }
    taskEXIT_CRITICAL();
    return ((uint64_t)high << 32U) | (uint64_t)low;
}

static bool gp_timer_start(void *unused)
{
    (void)unused;
    if (g_timing_running || htim2.Instance != TIM2 ||
        htim3.Instance != TIM3)
        return false;
    RCC_ClkInitTypeDef clock = {0};
    uint32_t flash_latency = 0U;
    HAL_RCC_GetClockConfig(&clock, &flash_latency);
    /* Fail closed if the measured system clock configuration no longer
     * matches the CubeMX-generated 1 MHz counter / 2 ms cycle settings. */
    if (HAL_RCC_GetPCLK1Freq() != 64000000U ||
        clock.APB1CLKDivider != RCC_APB1_DIV1 ||
        htim2.Init.Prescaler != 63U || htim2.Init.Period != UINT32_MAX ||
        htim3.Init.Prescaler != 63U || htim3.Init.Period != 1999U)
        return false;

    g_tim2_wraps = 0U;
    __HAL_TIM_SET_COUNTER(&htim2, 0U);
    __HAL_TIM_SET_COUNTER(&htim3, 0U);
    /* HAL TIM init emits UG/UIF; clear stale update before Start_IT. */
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
    if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
        return false;
    g_timing_running = true;
    if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK) {
        g_timing_running = false;
        (void)HAL_TIM_Base_Stop_IT(&htim2);
        return false;
    }
    return true;
}

static uint32_t gp_timer_wait(void *unused, TickType_t timeout_ticks)
{
    (void)unused;
    return (uint32_t)ulTaskNotifyTake(pdTRUE, timeout_ticks);
}

/* Delegate only TIM2/TIM3; never invoke motor, CANopen or path logic in ISR. */
void application_supervisor_timer_period_elapsed_isr(
    TIM_HandleTypeDef *timer)
{
    if (!g_timing_running || timer == NULL) return;
    if (timer->Instance == TIM2) {
        ++g_tim2_wraps;
    } else if (timer->Instance == TIM3) {
        TaskHandle_t owner = supervisor_task_handle();
        if (owner != NULL) {
            BaseType_t woken = pdFALSE;
            vTaskNotifyGiveFromISR(owner, &woken);
            portYIELD_FROM_ISR(woken);
        }
    }
}

/* An UNDEFINED weak function reference is NULL when no real board module
 * is linked. The header declaration intentionally remains strong so the
 * eventual board implementation is a normal strong definition. */
extern bool gp_board_supply_commissioned_config(
    GpBoardCommissioning *out) __attribute__((weak));

/* Persistent storage: do not place this large CANopen/robot config on the
 * 512-byte CubeMX defaultTask stack. Exactly one CM7 CANopen owner exists. */
static GpBoardCommissioning g_board_candidate;
static GpFdcanClassicBackend g_fdcan_state;
static CanBackend g_can_backend;
static CanopenMaster g_canopen_master;
static bool g_board_prepared;

/* This hook remains weak for compatibility with a future fully physical
 * board provider, but it implements an opt-in safe composition path.
 *
 * Prior to supplying a real board module, it returns false without starting
 * FDCAN, creating tasks or issuing CAN frames. */
__attribute__((weak))
bool application_supervisor_board_configure(SupervisorTaskConfig *config)
{
    if (config == NULL || g_board_prepared ||
        gp_board_supply_commissioned_config == NULL)
        return false;

    memset(&g_board_candidate, 0, sizeof(g_board_candidate));
    if (!gp_board_supply_commissioned_config(&g_board_candidate))
        return false;

    const GpBoardCommissioning *const board = &g_board_candidate;
    const SupervisorIoServices *const io = &board->supervisor.io;

    /* All of these dependencies must be implemented by the physical
     * board. The two software checks below are NOT certified safety. */
    if (board->hardware_interlocks_verified == NULL ||
        !board->hardware_interlocks_verified(
            board->hardware_interlock_context) ||
        io->read_inputs == NULL || io->write_outputs == NULL ||
        board->supervisor.canopen_master != NULL ||
        board->node_count != JOINT_DRIVE_AXES ||
        board->heartbeat_timeout_ms == 0U ||
        board->sdo_timeout_ms == 0U)
        return false;

    /* Require an actual coherent physical snapshot before even starting
     * FDCAN. A released E-stop alone is NOT permission for robot motion. */
    SupervisorInputSnapshot inputs = {0};
    if (!io->read_inputs(io->context, &inputs) || !inputs.valid ||
        inputs.asserted[SUP_IO_ESTOP] ||
        inputs.asserted[SUP_IO_PROTECTIVE_STOP])
        return false;

    /* No made-up motor node IDs: the coordinator checks uniqueness and
     * allowed CANopen IDs using the supplied physical drive configuration. */
    memset(&g_fdcan_state, 0, sizeof(g_fdcan_state));
    memset(&g_can_backend, 0, sizeof(g_can_backend));
    if (!gp_fdcan_classic_backend_open(
            &g_fdcan_state, &g_can_backend, &hfdcan1))
        return false;

    CanopenMasterConfig master_cfg = {0};
    master_cfg.backend = &g_can_backend;
    master_cfg.node_count = board->node_count;
    memcpy(master_cfg.node_ids, board->node_ids,
           sizeof(master_cfg.node_ids));
    master_cfg.heartbeat_timeout_ms = board->heartbeat_timeout_ms;
    master_cfg.sdo_timeout_ms = board->sdo_timeout_ms;

    if (!canopen_master_init(&g_canopen_master, &master_cfg))
    {
        can_backend_close(&g_can_backend);
        return false;
    }

    *config = board->supervisor;
    config->canopen_master = &g_canopen_master;
    g_board_prepared = true;
    return true;
}

/* Roll back transport ownership when initial Supervisor composition fails.
 * No CANopen runtime task is bound until Supervisor starts. Do not leave the
 * peripheral operating after an invalid configuration/IPC failure. */
static void revoke_unstarted_board(const SupervisorTaskConfig *board)
{
    if (!g_board_prepared || board == NULL ||
        board->canopen_master != &g_canopen_master)
        return;

    canopen_master_close(&g_canopen_master);
    memset(&g_can_backend, 0, sizeof(g_can_backend));
    g_board_prepared = false;
}

bool application_supervisor_start(const SupervisorTaskConfig *board)
{
    if (board == NULL || board->canopen_master == NULL ||
        !board->canopen_master->initialized ||
        !can_backend_valid(board->canopen_master->backend) ||
        board->io.read_inputs == NULL || board->io.write_outputs == NULL ||
        board->period_ticks != 2U)
    {
        revoke_unstarted_board(board);
        return false;
    }

    GpIpcShared *const shared = (GpIpcShared *)(uintptr_t)GP_SHARED_IPC_BASE;
    if (!gp_ipc_valid(shared) ||
        (board->ipc != NULL && board->ipc != shared))
    {
        revoke_unstarted_board(board);
        return false;
    }

    /* Avoid another large stack-local configuration on CubeMX's
     * provisional 512-byte defaultTask stack. Runtime stack watermark
     * verification remains mandatory before any board operation. */
    static SupervisorTaskConfig config;
    config = *board;
    config.ipc = shared;
    /* The actual board provider remains mandatory: timing cannot spoof
     * safety conditions, a CAN transport or physical drive feedback. */
    config.timing.monotonic_us = gp_time_now_us;
    config.timing.start = gp_timer_start;
    config.timing.wait_next = gp_timer_wait;
    config.timing.context = NULL;
    if (!supervisor_task_init(&config) || !supervisor_task_start(4U))
    {
        revoke_unstarted_board(board);
        return false;
    }
    return true;
}
