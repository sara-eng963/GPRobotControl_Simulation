#ifndef IPC_MAILBOX_H
#define IPC_MAILBOX_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* One producer and one consumer per ring. On STM32H745 the entire
 * IpcShared must be in explicitly NON-CACHEABLE shared SRAM.
 * Initialize ONCE before releasing the second core.
 * This is not a certified safety channel. */
#if defined(IPC_FREERTOS_BACKEND)
#define IPC_ABI_VERSION UINT32_C(2)
#else
#define IPC_ABI_VERSION UINT32_C(1)
#endif
#define IPC_MAGIC UINT32_C(0x47504943)
#define IPC_RING_CAPACITY 16U
#define IPC_WORDS 12U

/* Application messages never contain pointers, size_t or FreeRTOS task
 * handles. The 60-byte payload is a fixed 32-bit little-endian wire ABI.
 * In the STM32 backend, shared mailbox metadata separately contains
 * native FreeRTOS buffer control blocks and handles to shared memory;
 * those are not portable application-message fields. */
typedef enum {
    IPC_HMI_COMMAND = 1,
    IPC_CONTROLLER_STATUS = 2,
    IPC_STORAGE_REQUEST = 3,
    IPC_STORAGE_RESULT = 4,
    IPC_DIAGNOSTICS = 5,
    /* P03 protocol v2. Fixed 60-byte application ABI is retained. */
    IPC_CM4_PROGRESS = 6,
    IPC_CM4_HELLO = 7
} IpcType;

typedef struct {
    uint32_t type;
    uint32_t sequence;
    uint32_t timestamp_ms;
    uint32_t data[IPC_WORDS];
} IpcMessage;

/*
 * Target-only backend. Both STM32 images MUST use the same pinned FreeRTOS
 * kernel, configuration and ABI. A message-buffer descriptor is a FreeRTOS
 * object, not a portable wire structure; only the 60-byte application message
 * is portable. No blocking API is allowed across independently scheduled cores.
 *
 * Host POSIX builds deliberately retain the existing self-contained SPSC
 * adapter until a real two-image firmware build exists.
 */
#if defined(IPC_FREERTOS_BACKEND)
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
#define IPC_MESSAGE_STORAGE_BYTES \
    ((IPC_RING_CAPACITY * (sizeof(IpcMessage) + sizeof(size_t))) + 1U)
typedef struct {
    uint32_t magic;
    uint32_t abi_version;
    MessageBufferHandle_t m4_to_m7_handle;
    MessageBufferHandle_t m7_to_m4_handle;
    StaticMessageBuffer_t m4_to_m7_control;
    StaticMessageBuffer_t m7_to_m4_control;
    _Alignas(32) uint8_t m4_to_m7_storage[IPC_MESSAGE_STORAGE_BYTES];
    _Alignas(32) uint8_t m7_to_m4_storage[IPC_MESSAGE_STORAGE_BYTES];
} IpcShared;

/* Local-core callback, NOT stored in IpcShared or shared with other core.
 * Used for nonblocking STM32 HAL HSEM notifications after successful IO. */
typedef enum {
    IPC_NOTIFY_M7_TO_M4_DATA = 0,
    IPC_NOTIFY_M7_TO_M4_SPACE = 1,
    IPC_NOTIFY_M4_TO_M7_DATA = 2,
    IPC_NOTIFY_M4_TO_M7_SPACE = 3
} IpcNotifyEvent;
typedef void (*IpcNotifyFn)(IpcNotifyEvent event, void *context);
void ipc_set_notifier(IpcNotifyFn notifier, void *context);
#else
typedef struct {
    uint32_t head; /* producer-owned */
    uint32_t tail; /* consumer-owned */
    IpcMessage messages[IPC_RING_CAPACITY];
} IpcRing;

typedef struct {
    uint32_t magic;
    uint32_t abi_version;
    IpcRing m4_to_m7;
    IpcRing m7_to_m4;
} IpcShared;
#endif

/* Bootstrap only; resetting a live mailbox is forbidden. */
void ipc_initialize(IpcShared *shared);
bool ipc_valid(const IpcShared *shared);

/* Nonblocking SPSC API; false for empty/full/invalid, no overwrite. */
bool ipc_m4_send(IpcShared *shared, const IpcMessage *message);
bool ipc_m7_receive(IpcShared *shared, IpcMessage *message);
bool ipc_m7_send(IpcShared *shared, const IpcMessage *message);
bool ipc_m4_receive(IpcShared *shared, IpcMessage *message);
uint32_t ipc_m4_to_m7_pending(const IpcShared *shared);
uint32_t ipc_m7_to_m4_pending(const IpcShared *shared);

#endif
