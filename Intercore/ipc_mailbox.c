#include "ipc_mailbox.h"
#include <stdatomic.h>
#include <string.h>

_Static_assert(sizeof(uint32_t) == 4U, "IPC requires 32-bit words");
_Static_assert(sizeof(IpcMessage) == 60U, "IPC wire ABI changed");
_Static_assert(_Alignof(IpcRing) >= 4U, "IPC counters must align");
_Static_assert(ATOMIC_INT_LOCK_FREE == 2, "IPC needs lock-free 32-bit atomics");

void ipc_initialize(IpcShared *shared)
{
    if (shared == NULL) return;
    /* Other core must be stopped or waiting during initialisation. */
    memset(shared, 0, sizeof(*shared));
    shared->abi_version = IPC_ABI_VERSION;
    __atomic_store_n(&shared->magic, IPC_MAGIC, __ATOMIC_RELEASE);
}

bool ipc_valid(const IpcShared *shared)
{
    return shared != NULL &&
           __atomic_load_n(&shared->magic, __ATOMIC_ACQUIRE) == IPC_MAGIC &&
           shared->abi_version == IPC_ABI_VERSION;
}

static bool message_valid(const IpcMessage *message)
{
    return message != NULL && message->type >= IPC_HMI_COMMAND &&
           message->type <= IPC_CM4_HELLO;
}

static bool ring_send(IpcRing *ring, const IpcMessage *message)
{
    const uint32_t head = __atomic_load_n(&ring->head, __ATOMIC_RELAXED);
    const uint32_t tail = __atomic_load_n(&ring->tail, __ATOMIC_ACQUIRE);
    if ((uint32_t)(head - tail) >= IPC_RING_CAPACITY) return false;
    ring->messages[head % IPC_RING_CAPACITY] = *message;
    __atomic_store_n(&ring->head, head + 1U, __ATOMIC_RELEASE);
    return true;
}

static bool ring_receive(IpcRing *ring, IpcMessage *message)
{
    const uint32_t tail = __atomic_load_n(&ring->tail, __ATOMIC_RELAXED);
    const uint32_t head = __atomic_load_n(&ring->head, __ATOMIC_ACQUIRE);
    if (head == tail) return false;
    *message = ring->messages[tail % IPC_RING_CAPACITY];
    __atomic_store_n(&ring->tail, tail + 1U, __ATOMIC_RELEASE);
    return true;
}

static uint32_t ring_pending(const IpcRing *ring)
{
    const uint32_t head = __atomic_load_n(&ring->head, __ATOMIC_ACQUIRE);
    const uint32_t tail = __atomic_load_n(&ring->tail, __ATOMIC_ACQUIRE);
    const uint32_t distance = (uint32_t)(head - tail);
    return distance <= IPC_RING_CAPACITY ? distance : IPC_RING_CAPACITY;
}

bool ipc_m4_send(IpcShared *shared, const IpcMessage *message)
{
    return ipc_valid(shared) && message_valid(message) &&
           ring_send(&shared->m4_to_m7, message);
}

bool ipc_m7_receive(IpcShared *shared, IpcMessage *message)
{
    return ipc_valid(shared) && message != NULL &&
           ring_receive(&shared->m4_to_m7, message);
}

bool ipc_m7_send(IpcShared *shared, const IpcMessage *message)
{
    return ipc_valid(shared) && message_valid(message) &&
           ring_send(&shared->m7_to_m4, message);
}

bool ipc_m4_receive(IpcShared *shared, IpcMessage *message)
{
    return ipc_valid(shared) && message != NULL &&
           ring_receive(&shared->m7_to_m4, message);
}

uint32_t ipc_m4_to_m7_pending(const IpcShared *shared)
{
    return ipc_valid(shared) ? ring_pending(&shared->m4_to_m7) : 0U;
}

uint32_t ipc_m7_to_m4_pending(const IpcShared *shared)
{
    return ipc_valid(shared) ? ring_pending(&shared->m7_to_m4) : 0U;
}
