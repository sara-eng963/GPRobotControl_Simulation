/*
 * STM32H745 Cortex-M7: physical ownership of the two shared SRAM4 objects.
 *
 * Memory placement uses the existing linker sections. The objects have NO
 * initializers: startup must not independently clear memory the M4 may use.
 * ST's dual-FreeRTOS AMP example initializes shared message buffers on M7
 * before the HSEM-0 wakeup of M4. CM7 main.c invokes the initializer here
 * from its CubeMX boot user-code section before that wakeup.
 *
 * There is exactly ONE definition of each shared object, in the M7 image.
 * The M4 image must reference the same physical SRAM4 addresses, not
 * define second objects or reinitialize the contents.
 */
#define IPC_FREERTOS_BACKEND 1
#include "ipc_mailbox.h"
#include "trajectory_stream.h"
#include "validation_storage_bridge.h"
#include "intercore_shared_memory.h"
#include <string.h>
#include "../../../Common/Inc/sram4_memory_map.h"

_Static_assert(sizeof(IpcShared) <= SRAM4_IPC_BYTES,
               "Shared IPC does not fit its 4 KiB SRAM4 slot");
_Static_assert(sizeof(TrajectoryShared) <= SRAM4_TRAJECTORY_BYTES,
               "Shared trajectory does not fit its 16 KiB SRAM4 slot");
_Static_assert(sizeof(ValidationShared) <= SRAM4_VALIDATION_BYTES,
               "Validation ABI exceeds 12 KiB SRAM4 reservation");

#if !defined(__GNUC__)
#error "STM32H745 shared-object placement expects the CubeIDE GCC toolchain"
#endif

/* KEEP in the CM7 linker script prevents section garbage collection. */
IpcShared intercore_ipc_shared
    __attribute__((used, aligned(32), section(".shared_ipc")));

TrajectoryShared intercore_trajectory_shared
    __attribute__((used, aligned(32), section(".shared_trajectory")));

/* Preserve B2 ELF size/address while activating the typed protocol at
 * offset zero. The remaining bytes stay reserved and non-cacheable. */
typedef union {
    ValidationShared shared;
    uint8_t reserved[SRAM4_VALIDATION_BYTES];
} ValidationReservation;
_Static_assert(sizeof(ValidationReservation) == SRAM4_VALIDATION_BYTES,
               "SRAM4 reservation size drift");
ValidationReservation intercore_validation_storage
    __attribute__((used, aligned(32), section(".shared_validation")));

/* ST's FreeRTOS_AMP_Dual_RTOS initializes static message buffers before
 * releasing CM4 from STOP via HSEM-0. No separate boot protocol is needed
 * for the initial cold-start sequencing.
 *
 * The target's ipc_mailbox_freertos.c uses xMessageBufferCreateStatic().
 * Both objects must be initialized with M4 still quiescent. A failure
 * prevents M7 from releasing M4, avoiding partially initialized IPC.
 */
bool intercore_shared_memory_initialize(void)
{
    ipc_initialize(&intercore_ipc_shared);
    if (!ipc_valid(&intercore_ipc_shared))
        return false;

    trajectory_shared_initialize(&intercore_trajectory_shared);
    if (!trajectory_shared_valid(&intercore_trajectory_shared)) return false;
    memset(&intercore_validation_storage, 0, sizeof(intercore_validation_storage));
    validation_shared_initialize(&intercore_validation_storage.shared);
    return validation_shared_valid(&intercore_validation_storage.shared);
}
