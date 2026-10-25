/*
 * STM32H745 Cortex-M4: validate M7-created objects before starting
 * application tasks. The startup order follows ST's STM32CubeH7
 * FreeRTOS_AMP_Dual_RTOS dual-kernel example.
 *
 * The M7 owns the physical object definitions and must initialize both
 * BEFORE waking M4 via HSEM-0. M4 only references the fixed SRAM4
 * addresses, reusing existing FreeRTOS IPC and trajectory ABI checks.
 * A failure prevents M4's application scheduler from starting.
 */
#define IPC_FREERTOS_BACKEND 1
#include "intercore_startup_validation.h"
#include "ipc_mailbox.h"
#include "trajectory_stream.h"
#include "validation_storage_wire.h"
#include "../../../Common/Inc/sram4_memory_map.h"

_Static_assert(sizeof(IpcShared) <= SRAM4_IPC_BYTES,
               "M4 IPC ABI does not fit shared SRAM4 partition");
_Static_assert(sizeof(TrajectoryShared) <= SRAM4_TRAJECTORY_BYTES,
               "M4 trajectory ABI does not fit shared SRAM4 partition");
_Static_assert(sizeof(uintptr_t) == sizeof(uint32_t),
               "STM32H745 M4 requires 32-bit physical addresses");

bool intercore_startup_validate(void)
{
    const IpcShared *const ipc =
        (const IpcShared *)(uintptr_t)SRAM4_IPC_BASE;
    const TrajectoryShared *const trajectory =
        (const TrajectoryShared *)(uintptr_t)SRAM4_TRAJECTORY_BASE;

    const ValidationWireHeader *const validation =
        (const ValidationWireHeader *)(uintptr_t)SRAM4_VALIDATION_BASE;
    return ipc_valid(ipc) && trajectory_shared_valid(trajectory) &&
           __atomic_load_n(&validation->magic, __ATOMIC_ACQUIRE) ==
               VALIDATION_MAGIC &&
           validation->version == VALIDATION_VERSION &&
           __atomic_load_n(&validation->worker_ready, __ATOMIC_ACQUIRE) == 0U;
}
