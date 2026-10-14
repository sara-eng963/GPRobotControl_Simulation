/* Cortex-M7 production application composition: use the existing Supervisor.
 * No simulated CAN, fake healthy safety inputs, or substitute state machine.
 * Stage 2 IPC has exactly one M7 producer and one M7 consumer: Supervisor.
 */
#define GP_IPC_FREERTOS_BACKEND 1
#include "application_supervisor.h"
#include "ipc_mailbox.h"
#include "sram4_memory_map.h"
#include <stdint.h>

/* A later physical board adapter overrides this via the conventional GCC
 * weak/strong linkage mechanism. Do not run control without real inputs. */
__attribute__((weak))
bool application_supervisor_board_configure(SupervisorTaskConfig *config)
{
    (void)config;
    return false;
}

bool application_supervisor_start(const SupervisorTaskConfig *board)
{
    if (board == NULL || board->canopen_master == NULL ||
        !board->canopen_master->initialized ||
        !can_backend_valid(board->canopen_master->backend) ||
        board->io.read_inputs == NULL || board->io.write_outputs == NULL ||
        board->period_ticks == 0U)
        return false;

    GpIpcShared *const shared = (GpIpcShared *)(uintptr_t)GP_SHARED_IPC_BASE;
    if (!gp_ipc_valid(shared) ||
        (board->ipc != NULL && board->ipc != shared))
        return false;

    SupervisorTaskConfig config = *board;
    config.ipc = shared;
    return supervisor_task_init(&config) && supervisor_task_start(4U);
}
