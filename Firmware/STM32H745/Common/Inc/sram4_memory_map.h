#ifndef SRAM4_MEMORY_MAP_H
#define SRAM4_MEMORY_MAP_H
#include <stdint.h>
/* Stage 2B design contract. Does NOT yet configure linker, MPU or startup.
 * Both H745 cores must agree on these fixed 32-byte-aligned physical addresses.
 */
#define SRAM4_BASE                  UINT32_C(0x38000000)
#define SRAM4_BYTES                 UINT32_C(0x10000)
#define SRAM4_SHARED_BASE                 SRAM4_BASE
#define SRAM4_SHARED_BYTES                UINT32_C(0x8000)
#define SRAM4_IPC_BASE             SRAM4_SHARED_BASE
#define SRAM4_IPC_BYTES            UINT32_C(0x1000)
#define SRAM4_TRAJECTORY_BASE      UINT32_C(0x38001000)
#define SRAM4_TRAJECTORY_BYTES     UINT32_C(0x4000)
#define SRAM4_RESERVED_BASE         UINT32_C(0x38005000)
#define SRAM4_RESERVED_BYTES        UINT32_C(0x3000)
#define SRAM4_VALIDATION_BASE      SRAM4_RESERVED_BASE
#define SRAM4_VALIDATION_BYTES     SRAM4_RESERVED_BYTES
#define SRAM4_UNALLOCATED_BASE      UINT32_C(0x38008000)
#define SRAM4_UNALLOCATED_BYTES     UINT32_C(0x8000)
_Static_assert((SRAM4_SHARED_BASE & (SRAM4_SHARED_BYTES - 1U)) == 0U,
               "MPU base must align to 32 KiB");
_Static_assert(SRAM4_IPC_BASE + SRAM4_IPC_BYTES == SRAM4_TRAJECTORY_BASE,
               "IPC and trajectory slots must meet");
_Static_assert(SRAM4_TRAJECTORY_BASE + SRAM4_TRAJECTORY_BYTES == SRAM4_RESERVED_BASE,
               "Trajectory and spare slots must meet");
_Static_assert(SRAM4_RESERVED_BASE + SRAM4_RESERVED_BYTES == SRAM4_UNALLOCATED_BASE,
               "Shared window must total 32 KiB");
_Static_assert(SRAM4_UNALLOCATED_BASE + SRAM4_UNALLOCATED_BYTES == SRAM4_BASE + SRAM4_BYTES,
               "Whole SRAM4 must total 64 KiB");
_Static_assert((SRAM4_IPC_BASE % 32U) == 0U && (SRAM4_TRAJECTORY_BASE % 32U) == 0U,
               "Shared object bases must be aligned");
#endif
