/* Target-only implementation: compile INSTEAD OF ipc_mailbox.c.
 * The FreeRTOS kernel supplies actual message storage, framing and state.
 * Each channel has exactly one sender and one receiver, both nonblocking.
 * This is not a safety or emergency-stop path. */
/* This compilation unit selects the FreeRTOS implementation by design.
 * It is never compiled alongside the host-only ipc_mailbox.c backend.
 * Setting GP_IPC_FREERTOS_BACKEND=0 explicitly is a configuration error. */
#ifndef GP_IPC_FREERTOS_BACKEND
#define GP_IPC_FREERTOS_BACKEND 1
#elif (GP_IPC_FREERTOS_BACKEND != 1)
#error "ipc_mailbox_freertos.c requires GP_IPC_FREERTOS_BACKEND=1"
#endif
#include "ipc_mailbox.h"
#include <string.h>

_Static_assert(sizeof(uint32_t) == 4U, "Requires 32-bit words");
/* Only the POSIX runtime test uses host-sized pointers and size_t.
 * Both actual STM32 binaries still enforce their 32-bit ABI at compile time.
 * The host build uses the real FreeRTOS message-buffer implementation. */
#if !defined(GP_IPC_POSIX_TEST)
_Static_assert(sizeof(size_t) == 4U, "Identical 32-bit FreeRTOS ABI required");
_Static_assert(sizeof(void *) == 4U, "STM32H745 is a 32-bit target");
#endif
_Static_assert(sizeof(GpIpcMessage) == 60U, "IPC application wire ABI changed");
_Static_assert(__atomic_always_lock_free(sizeof(uint32_t), 0),
               "Requires lock-free publication of ready flag");

static GpIpcNotifyFn local_notifier;
static void *local_notify_context;

void gp_ipc_set_notifier(GpIpcNotifyFn notifier, void *context)
{
    /* Must be called on each core independently, before concurrent operation. */
    local_notify_context = context;
    local_notifier = notifier;
}

static void notify_peer(GpIpcNotifyEvent event)
{
    /* No callbacks or target function pointers are stored in shared SRAM. */
    if (local_notifier != NULL)
        local_notifier(event, local_notify_context);
}

void gp_ipc_initialize(GpIpcShared *shared)
{
    if (shared == NULL) return;
    /* M7 only; M4 must remain quiescent until initialization is complete. */
    memset(shared, 0, sizeof(*shared));
    shared->m4_to_m7_handle = xMessageBufferCreateStatic(
        sizeof(shared->m4_to_m7_storage), shared->m4_to_m7_storage,
        &shared->m4_to_m7_control);
    shared->m7_to_m4_handle = xMessageBufferCreateStatic(
        sizeof(shared->m7_to_m4_storage), shared->m7_to_m4_storage,
        &shared->m7_to_m4_control);
    if (shared->m4_to_m7_handle == NULL || shared->m7_to_m4_handle == NULL)
        return; /* magic remains invalid: fail closed */
    shared->abi_version = GP_IPC_ABI_VERSION;
    __atomic_store_n(&shared->magic, GP_IPC_MAGIC, __ATOMIC_RELEASE);
}

bool gp_ipc_valid(const GpIpcShared *shared)
{
    return shared != NULL &&
           __atomic_load_n(&shared->magic, __ATOMIC_ACQUIRE) == GP_IPC_MAGIC &&
           shared->abi_version == GP_IPC_ABI_VERSION &&
           shared->m4_to_m7_handle != NULL &&
           shared->m7_to_m4_handle != NULL;
}

static bool valid_message(const GpIpcMessage *msg)
{
    return msg != NULL && msg->type >= GP_IPC_HMI_COMMAND &&
           msg->type <= GP_IPC_DIAGNOSTICS;
}

static bool send_message(const GpIpcShared *shared, MessageBufferHandle_t handle,
                         const GpIpcMessage *msg, GpIpcNotifyEvent event)
{
    if (!gp_ipc_valid(shared) || !valid_message(msg)) return false;
    /* IMPORTANT: 0 timeout. Never install a task waiter in shared FreeRTOS
     * control blocks; the peer kernel must not directly notify local tasks. */
    if (xMessageBufferSend(handle, msg, sizeof(*msg), 0) != sizeof(*msg))
        return false;
    notify_peer(event);
    return true;
}

static bool receive_message(const GpIpcShared *shared, MessageBufferHandle_t handle,
                            GpIpcMessage *msg, GpIpcNotifyEvent event)
{
    if (!gp_ipc_valid(shared) || msg == NULL) return false;
    if (xMessageBufferReceive(handle, msg, sizeof(*msg), 0) != sizeof(*msg))
        return false;
    notify_peer(event);
    return true;
}

bool gp_ipc_m4_send(GpIpcShared *shared, const GpIpcMessage *msg)
{
    return shared != NULL && send_message(shared, shared->m4_to_m7_handle, msg,
                                          GP_IPC_NOTIFY_M4_TO_M7_DATA);
}
bool gp_ipc_m7_receive(GpIpcShared *shared, GpIpcMessage *msg)
{
    return shared != NULL && receive_message(shared, shared->m4_to_m7_handle, msg,
                                             GP_IPC_NOTIFY_M4_TO_M7_SPACE);
}
bool gp_ipc_m7_send(GpIpcShared *shared, const GpIpcMessage *msg)
{
    return shared != NULL && send_message(shared, shared->m7_to_m4_handle, msg,
                                          GP_IPC_NOTIFY_M7_TO_M4_DATA);
}
bool gp_ipc_m4_receive(GpIpcShared *shared, GpIpcMessage *msg)
{
    return shared != NULL && receive_message(shared, shared->m7_to_m4_handle, msg,
                                             GP_IPC_NOTIFY_M7_TO_M4_SPACE);
}

static uint32_t pending_messages(const GpIpcShared *shared,
                                 MessageBufferHandle_t handle)
{
    if (!gp_ipc_valid(shared)) return 0U;
    /* Diagnostic snapshot only. No decisions or concurrency proof rely on it.
     * One message is 60 payload bytes plus four framing bytes on STM32. */
    const size_t bytes = xStreamBufferBytesAvailable(handle);
    const size_t total = sizeof(GpIpcMessage) + sizeof(size_t);
    const size_t count = bytes / total;
    return (uint32_t)(count > GP_IPC_RING_CAPACITY ?
                      GP_IPC_RING_CAPACITY : count);
}

uint32_t gp_ipc_m4_to_m7_pending(const GpIpcShared *shared)
{
    return shared == NULL ? 0U :
           pending_messages(shared, shared->m4_to_m7_handle);
}
uint32_t gp_ipc_m7_to_m4_pending(const GpIpcShared *shared)
{
    return shared == NULL ? 0U :
           pending_messages(shared, shared->m7_to_m4_handle);
}
