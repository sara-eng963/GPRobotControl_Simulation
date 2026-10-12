/* Stage 2A: compile-only ABI/sizeof/alignment probe. NEVER link into firmware.
 * Uses generated per-core FreeRTOSConfig.h and the vendored kernel 10.6.2.
 * The exported object symbols allow readelf to report native ARM layouts.
 */
#include "stm32h7xx.h"
#include <stdint.h>
#include <stddef.h>

#include "ipc_mailbox.h"
#include "trajectory_stream.h"
#include "sram4_memory_map.h"

_Static_assert(sizeof(void *) == 4U, "Expected ARM 32-bit pointer layout");
_Static_assert(sizeof(size_t) == 4U, "Expected ARM 32-bit size_t layout");
_Static_assert(sizeof(GpIpcMessage) == 60U, "IPC payload ABI drift");
_Static_assert(sizeof(GpTrajectorySample) == 24U, "Trajectory sample ABI drift");
_Static_assert(GP_TRAJ_CAPACITY == 512U, "Unexpected trajectory capacity");
_Static_assert(_Alignof(GpTrajectoryShared) >= 32U,
               "Trajectory sample alignment requires 32 bytes");
_Static_assert(sizeof(GpIpcShared) <= GP_SHARED_IPC_BYTES,
               "IPC storage exceeds its 4 KiB reservation");
_Static_assert(sizeof(GpTrajectoryShared) <= GP_SHARED_TRAJECTORY_BYTES,
               "Trajectory storage exceeds its 16 KiB reservation");
_Static_assert((sizeof(GpIpcShared) + sizeof(GpTrajectoryShared)) <= GP_SHARED_BYTES,
               "Objects exceed shared MPU window");

#define GP_PROBE __attribute__((used))
/* The following are BSS probe objects, not actual firmware allocations. */
GP_PROBE GpIpcShared gp_stage2_shared_ipc;
GP_PROBE GpTrajectoryShared gp_stage2_shared_trajectory;
GP_PROBE GpIpcMessage gp_stage2_ipc_message;
GP_PROBE GpTrajectorySample gp_stage2_trajectory_sample;
GP_PROBE GpTrajectoryProducer gp_stage2_private_m4_producer;
GP_PROBE GpTrajectoryConsumer gp_stage2_private_m7_consumer;
/* Array sizes encode constants for readelf's per-symbol Size field. */
GP_PROBE uint8_t gp_stage2_ipc_align[_Alignof(GpIpcShared)];
GP_PROBE uint8_t gp_stage2_trajectory_align[_Alignof(GpTrajectoryShared)];
GP_PROBE uint8_t gp_stage2_ipc_m4_storage_offset[offsetof(GpIpcShared, m4_to_m7_storage) + 1U];
GP_PROBE uint8_t gp_stage2_ipc_m7_storage_offset[offsetof(GpIpcShared, m7_to_m4_storage) + 1U];
GP_PROBE uint8_t gp_stage2_trajectory_samples_offset[offsetof(GpTrajectoryShared, samples) + 1U];
GP_PROBE uint8_t gp_stage2_ipc_length_prefix[sizeof(size_t)];
