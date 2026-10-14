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
#define GP_IPC_FREERTOS_BACKEND 1
#include "intercore_startup_validation.h"
#include "ipc_mailbox.h"
#include "trajectory_stream.h"
#include "validation_storage_wire.h"
#include "../../../Common/Inc/sram4_memory_map.h"

_Static_assert(sizeof(GpIpcShared) <= GP_SHARED_IPC_BYTES,
               "M4 IPC ABI does not fit shared SRAM4 partition");
_Static_assert(sizeof(GpTrajectoryShared) <= GP_SHARED_TRAJECTORY_BYTES,
               "M4 trajectory ABI does not fit shared SRAM4 partition");
_Static_assert(sizeof(uintptr_t) == sizeof(uint32_t),
               "STM32H745 M4 requires 32-bit physical addresses");

bool intercore_startup_validate(void)
{
    const GpIpcShared *const ipc =
        (const GpIpcShared *)(uintptr_t)GP_SHARED_IPC_BASE;
    const GpTrajectoryShared *const trajectory =
        (const GpTrajectoryShared *)(uintptr_t)GP_SHARED_TRAJECTORY_BASE;

    const GpValidationWireHeader *const validation =
        (const GpValidationWireHeader *)(uintptr_t)GP_SHARED_VALIDATION_BASE;
    return gp_ipc_valid(ipc) && gp_traj_shared_valid(trajectory) &&
           __atomic_load_n(&validation->magic, __ATOMIC_ACQUIRE) ==
               GP_VALIDATION_MAGIC &&
           validation->version == GP_VALIDATION_VERSION &&
           __atomic_load_n(&validation->worker_ready, __ATOMIC_ACQUIRE) == 0U;
}
