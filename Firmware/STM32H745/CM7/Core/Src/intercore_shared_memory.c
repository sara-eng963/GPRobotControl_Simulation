/*
 * STM32H745 Cortex-M7: physical ownership of the two shared SRAM4 objects.
 *
 * Memory placement uses the existing linker sections. The objects have NO
 * initializers: startup must not independently clear memory the M4 may use.
 * ST's dual-FreeRTOS AMP example initializes shared message buffers on M7
 * before the HSEM-0 wakeup of M4; that boot integration is a separate step.
 *
 * There is exactly ONE definition of each shared object, in the M7 image.
 * The M4 image must reference the same physical SRAM4 addresses, not
 * define second objects or reinitialize the contents.
 */
#define GP_IPC_FREERTOS_BACKEND 1
#include "ipc_mailbox.h"
#include "trajectory_stream.h"
#include "../../../Common/Inc/sram4_memory_map.h"

_Static_assert(sizeof(GpIpcShared) <= GP_SHARED_IPC_BYTES,
               "Shared IPC does not fit its 4 KiB SRAM4 slot");
_Static_assert(sizeof(GpTrajectoryShared) <= GP_SHARED_TRAJECTORY_BYTES,
               "Shared trajectory does not fit its 16 KiB SRAM4 slot");

#if !defined(__GNUC__)
#error "STM32H745 shared-object placement expects the CubeIDE GCC toolchain"
#endif

/* KEEP in the CM7 linker script prevents section garbage collection. */
GpIpcShared intercore_ipc_shared
    __attribute__((used, aligned(32), section(".gp_shared_ipc")));

GpTrajectoryShared intercore_trajectory_shared
    __attribute__((used, aligned(32), section(".gp_shared_trajectory")));
