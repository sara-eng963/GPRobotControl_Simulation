/* Stage 2C: synthetic SRAM4 linker proof, never part of a production image.
 * Each firmware core builds this file with its own FreeRTOS 10.6.2 config.
 * The two structs are deliberately uninitialized: NOLOAD means neither ELF
 * may ship an initial value for live cross-core SRAM4 data.
 */
#include <stdint.h>
#include "ipc_mailbox.h"
#include "trajectory_stream.h"
#include "gp_shared_sram4_layout.h"

_Static_assert(sizeof(GpIpcShared) <= GP_SHARED_IPC_BYTES,
               "IPC object exceeds linker IPC reservation");
_Static_assert(sizeof(GpTrajectoryShared) <= GP_SHARED_TRAJECTORY_BYTES,
               "Trajectory object exceeds linker trajectory reservation");

#define GP_LINKER_PROBE __attribute__((used, aligned(32)))

GpIpcShared gp_stage2_linker_ipc
    GP_LINKER_PROBE __attribute__((section(".gp_shared_ipc")));

GpTrajectoryShared gp_stage2_linker_trajectory
    GP_LINKER_PROBE __attribute__((section(".gp_shared_trajectory")));

/* Real app uses CubeMX startup; this standalone probe uses an inert entry. */
void Reset_Handler(void) { for (;;) { __asm volatile("nop"); } }
