/* Stage 2C: synthetic SRAM4 linker proof, never part of a production image.
 * Each firmware core builds this file with its own FreeRTOS 10.6.2 config.
 * The two structs are deliberately uninitialized: NOLOAD means neither ELF
 * may ship an initial value for live cross-core SRAM4 data.
 */
#include <stdint.h>
#define IPC_FREERTOS_BACKEND 1
#include "ipc_mailbox.h"
#include "trajectory_stream.h"
#include "sram4_memory_map.h"

_Static_assert(sizeof(IpcShared) <= SRAM4_IPC_BYTES,
               "IPC object exceeds linker IPC reservation");
_Static_assert(sizeof(TrajectoryShared) <= SRAM4_TRAJECTORY_BYTES,
               "Trajectory object exceeds linker trajectory reservation");

#define LINKER_PROBE_USED __attribute__((used, aligned(32)))

IpcShared linker_probe_ipc
    LINKER_PROBE_USED __attribute__((section(".shared_ipc")));

TrajectoryShared linker_probe_trajectory
    LINKER_PROBE_USED __attribute__((section(".shared_trajectory")));

/* Synthetic validation allocation, never linked into production. */
uint8_t linker_probe_validation[SRAM4_VALIDATION_BYTES]
    LINKER_PROBE_USED __attribute__((section(".shared_validation")));

/* Real app uses CubeMX startup; this standalone probe uses an inert entry. */
void Reset_Handler(void) { for (;;) { __asm volatile("nop"); } }
