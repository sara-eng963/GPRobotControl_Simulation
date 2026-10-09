#ifndef GP_IPC_MAILBOX_H
#define GP_IPC_MAILBOX_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* One producer and one consumer per ring. On STM32H745 the entire
 * GpIpcShared must be in explicitly NON-CACHEABLE shared SRAM.
 * Initialize ONCE before releasing the second core.
 * This is not a certified safety channel. */
#define GP_IPC_ABI_VERSION UINT32_C(1)
#define GP_IPC_MAGIC UINT32_C(0x47504943)
#define GP_IPC_RING_CAPACITY 16U
#define GP_IPC_WORDS 12U

/* No pointers, size_t, task handles, or FreeRTOS objects cross cores.
 * Fixed-size little-endian 32-bit wire ABI. */
typedef enum {
    GP_IPC_HMI_COMMAND = 1,
    GP_IPC_CONTROLLER_STATUS = 2,
    GP_IPC_STORAGE_REQUEST = 3,
    GP_IPC_STORAGE_RESULT = 4,
    GP_IPC_DIAGNOSTICS = 5
} GpIpcType;

typedef struct {
    uint32_t type;
    uint32_t sequence;
    uint32_t timestamp_ms;
    uint32_t data[GP_IPC_WORDS];
} GpIpcMessage;

typedef struct {
    uint32_t head; /* producer-owned */
    uint32_t tail; /* consumer-owned */
    GpIpcMessage messages[GP_IPC_RING_CAPACITY];
} GpIpcRing;

typedef struct {
    uint32_t magic;
    uint32_t abi_version;
    GpIpcRing m4_to_m7;
    GpIpcRing m7_to_m4;
} GpIpcShared;

/* Bootstrap only; resetting a live mailbox is forbidden. */
void gp_ipc_initialize(GpIpcShared *shared);
bool gp_ipc_valid(const GpIpcShared *shared);

/* Nonblocking SPSC API; false for empty/full/invalid, no overwrite. */
bool gp_ipc_m4_send(GpIpcShared *shared, const GpIpcMessage *message);
bool gp_ipc_m7_receive(GpIpcShared *shared, GpIpcMessage *message);
bool gp_ipc_m7_send(GpIpcShared *shared, const GpIpcMessage *message);
bool gp_ipc_m4_receive(GpIpcShared *shared, GpIpcMessage *message);
uint32_t gp_ipc_m4_to_m7_pending(const GpIpcShared *shared);
uint32_t gp_ipc_m7_to_m4_pending(const GpIpcShared *shared);

#endif
