#ifndef SRAM4_MEMORY_MAP_H
#define SRAM4_MEMORY_MAP_H
#include <stdint.h>
/* Stage 2B design contract. Does NOT yet configure linker, MPU or startup.
 * Both H745 cores must agree on these fixed 32-byte-aligned physical addresses.
 */
#define GP_SRAM4_BASE                  UINT32_C(0x38000000)
#define GP_SRAM4_BYTES                 UINT32_C(0x10000)
#define GP_SHARED_BASE                 GP_SRAM4_BASE
#define GP_SHARED_BYTES                UINT32_C(0x8000)
#define GP_SHARED_IPC_BASE             GP_SHARED_BASE
#define GP_SHARED_IPC_BYTES            UINT32_C(0x1000)
#define GP_SHARED_TRAJECTORY_BASE      UINT32_C(0x38001000)
#define GP_SHARED_TRAJECTORY_BYTES     UINT32_C(0x4000)
#define GP_SHARED_RESERVE_BASE         UINT32_C(0x38005000)
#define GP_SHARED_RESERVE_BYTES        UINT32_C(0x3000)
#define GP_SHARED_VALIDATION_BASE      GP_SHARED_RESERVE_BASE
#define GP_SHARED_VALIDATION_BYTES     GP_SHARED_RESERVE_BYTES
#define GP_SRAM4_UNALLOCATED_BASE      UINT32_C(0x38008000)
#define GP_SRAM4_UNALLOCATED_BYTES     UINT32_C(0x8000)
_Static_assert((GP_SHARED_BASE & (GP_SHARED_BYTES - 1U)) == 0U,
               "MPU base must align to 32 KiB");
_Static_assert(GP_SHARED_IPC_BASE + GP_SHARED_IPC_BYTES == GP_SHARED_TRAJECTORY_BASE,
               "IPC and trajectory slots must meet");
_Static_assert(GP_SHARED_TRAJECTORY_BASE + GP_SHARED_TRAJECTORY_BYTES == GP_SHARED_RESERVE_BASE,
               "Trajectory and spare slots must meet");
_Static_assert(GP_SHARED_RESERVE_BASE + GP_SHARED_RESERVE_BYTES == GP_SRAM4_UNALLOCATED_BASE,
               "Shared window must total 32 KiB");
_Static_assert(GP_SRAM4_UNALLOCATED_BASE + GP_SRAM4_UNALLOCATED_BYTES == GP_SRAM4_BASE + GP_SRAM4_BYTES,
               "Whole SRAM4 must total 64 KiB");
_Static_assert((GP_SHARED_IPC_BASE % 32U) == 0U && (GP_SHARED_TRAJECTORY_BASE % 32U) == 0U,
               "Shared object bases must be aligned");
#endif
