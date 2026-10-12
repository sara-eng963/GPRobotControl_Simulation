#ifndef GP_IPC_MAILBOX_H
#define GP_IPC_MAILBOX_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* One producer and one consumer per ring. On STM32H745 the entire
 * GpIpcShared must be in explicitly NON-CACHEABLE shared SRAM.
 * Initialize ONCE before releasing the second core.
 * This is not a certified safety channel. */
#if defined(GP_IPC_FREERTOS_BACKEND)
#define GP_IPC_ABI_VERSION UINT32_C(2)
#else
#define GP_IPC_ABI_VERSION UINT32_C(1)
#endif
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

/*
 * Target-only backend. Both STM32 images MUST use the same pinned FreeRTOS
 * kernel, configuration and ABI. A message-buffer descriptor is a FreeRTOS
 * object, not a portable wire structure; only the 60-byte application message
 * is portable. No blocking API is allowed across independently scheduled cores.
 *
 * Host POSIX builds deliberately retain the existing self-contained SPSC
 * adapter until a real two-image firmware build exists.
 */
#if defined(GP_IPC_FREERTOS_BACKEND)
#include "FreeRTOS.h"
#include "message_buffer.h"
#if (configSUPPORT_STATIC_ALLOCATION != 1)
#error "The STM32 inter-core message-buffer backend requires FreeRTOS static allocation"
#endif
/* FreeRTOS 10.6.2 message_buffer.h includes stream_buffer.h and has no
 * configUSE_STREAM_BUFFERS feature toggle. Compilation and linking of the
 * vendor stream_buffer.c is verified by the firmware build instead. */

/* 16 whole 60-byte messages plus the FreeRTOS size_t length prefix.
 * One extra byte compensates for the underlying stream-buffer sentinel. */
#define GP_IPC_MESSAGE_STORAGE_BYTES \
    ((GP_IPC_RING_CAPACITY * (sizeof(GpIpcMessage) + sizeof(size_t))) + 1U)
typedef struct {
    uint32_t magic;
    uint32_t abi_version;
    MessageBufferHandle_t m4_to_m7_handle;
    MessageBufferHandle_t m7_to_m4_handle;
    StaticMessageBuffer_t m4_to_m7_control;
    StaticMessageBuffer_t m7_to_m4_control;
    _Alignas(32) uint8_t m4_to_m7_storage[GP_IPC_MESSAGE_STORAGE_BYTES];
    _Alignas(32) uint8_t m7_to_m4_storage[GP_IPC_MESSAGE_STORAGE_BYTES];
} GpIpcShared;

/* Local-core callback, NOT stored in GpIpcShared or shared with other core.
 * Used for nonblocking STM32 HAL HSEM notifications after successful IO. */
typedef enum {
    GP_IPC_NOTIFY_M7_TO_M4_DATA = 0,
    GP_IPC_NOTIFY_M7_TO_M4_SPACE = 1,
    GP_IPC_NOTIFY_M4_TO_M7_DATA = 2,
    GP_IPC_NOTIFY_M4_TO_M7_SPACE = 3
} GpIpcNotifyEvent;
typedef void (*GpIpcNotifyFn)(GpIpcNotifyEvent event, void *context);
void gp_ipc_set_notifier(GpIpcNotifyFn notifier, void *context);
#else
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
#endif

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
