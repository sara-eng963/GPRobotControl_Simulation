/* Stage 2A: compile-only ABI/sizeof/alignment probe. NEVER link into firmware.
 * Uses generated per-core FreeRTOSConfig.h and the vendored kernel 10.6.2.
 * The exported object symbols allow readelf to report native ARM layouts.
 */
#include "stm32h7xx.h"
#include <stdint.h>
#include <stddef.h>

#define IPC_FREERTOS_BACKEND 1
#include "ipc_mailbox.h"
#include "trajectory_stream.h"
#include "sram4_memory_map.h"
#include "validation_storage_bridge.h"
_Static_assert(sizeof(ValidationShared) <= SRAM4_VALIDATION_BYTES, "Validation partition overflow");

_Static_assert(sizeof(void *) == 4U, "Expected ARM 32-bit pointer layout");
_Static_assert(sizeof(size_t) == 4U, "Expected ARM 32-bit size_t layout");
_Static_assert(sizeof(IpcMessage) == 60U, "IPC payload ABI drift");
_Static_assert(sizeof(TrajectorySample) == 24U, "Trajectory sample ABI drift");
_Static_assert(TRAJECTORY_CAPACITY == 512U, "Unexpected trajectory capacity");
_Static_assert(_Alignof(TrajectoryShared) >= 32U,
               "Trajectory sample alignment requires 32 bytes");
_Static_assert(sizeof(IpcShared) <= SRAM4_IPC_BYTES,
               "IPC storage exceeds its 4 KiB reservation");
_Static_assert(sizeof(TrajectoryShared) <= SRAM4_TRAJECTORY_BYTES,
               "Trajectory storage exceeds its 16 KiB reservation");
_Static_assert(SRAM4_VALIDATION_BASE == UINT32_C(0x38005000) &&
               SRAM4_VALIDATION_BYTES == 12288U,
               "Validation slot address/size mismatch");
_Static_assert((sizeof(IpcShared) + sizeof(TrajectoryShared)) <= SRAM4_SHARED_BYTES,
               "Objects exceed shared MPU window");

#define GP_PROBE __attribute__((used))
/* The following are BSS probe objects, not actual firmware allocations. */
GP_PROBE IpcShared gp_stage2_shared_ipc;
GP_PROBE TrajectoryShared gp_stage2_shared_trajectory;
GP_PROBE IpcMessage gp_stage2_ipc_message;
GP_PROBE TrajectorySample gp_stage2_trajectory_sample;
GP_PROBE TrajectoryProducer gp_stage2_private_m4_producer;
GP_PROBE TrajectoryConsumer gp_stage2_private_m7_consumer;
/* Array sizes encode constants for readelf's per-symbol Size field. */
GP_PROBE uint8_t gp_stage2_ipc_align[_Alignof(IpcShared)];
GP_PROBE uint8_t gp_stage2_trajectory_align[_Alignof(TrajectoryShared)];
GP_PROBE uint8_t gp_stage2_ipc_m4_storage_offset[offsetof(IpcShared, m4_to_m7_storage) + 1U];
GP_PROBE uint8_t gp_stage2_ipc_m7_storage_offset[offsetof(IpcShared, m7_to_m4_storage) + 1U];
GP_PROBE uint8_t gp_stage2_trajectory_samples_offset[offsetof(TrajectoryShared, samples) + 1U];
GP_PROBE uint8_t gp_stage2_ipc_length_prefix[sizeof(size_t)];
