#include "ipc_mailbox.h"
#include <stdatomic.h>
#include <string.h>

_Static_assert(sizeof(uint32_t) == 4U, "IPC requires 32-bit words");
_Static_assert(sizeof(GpIpcMessage) == 60U, "IPC wire ABI changed");
_Static_assert(_Alignof(GpIpcRing) >= 4U, "IPC counters must align");
_Static_assert(ATOMIC_INT_LOCK_FREE == 2, "IPC needs lock-free 32-bit atomics");

void gp_ipc_initialize(GpIpcShared *shared)
{
    if (shared == NULL) return;
    /* Other core must be stopped or waiting during initialisation. */
    memset(shared, 0, sizeof(*shared));
    shared->abi_version = GP_IPC_ABI_VERSION;
    __atomic_store_n(&shared->magic, GP_IPC_MAGIC, __ATOMIC_RELEASE);
}

bool gp_ipc_valid(const GpIpcShared *shared)
{
    return shared != NULL &&
           __atomic_load_n(&shared->magic, __ATOMIC_ACQUIRE) == GP_IPC_MAGIC &&
           shared->abi_version == GP_IPC_ABI_VERSION;
}

static bool message_valid(const GpIpcMessage *message)
{
    return message != NULL && message->type >= GP_IPC_HMI_COMMAND &&
           message->type <= GP_IPC_DIAGNOSTICS;
}

static bool ring_send(GpIpcRing *ring, const GpIpcMessage *message)
{
    const uint32_t head = __atomic_load_n(&ring->head, __ATOMIC_RELAXED);
    const uint32_t tail = __atomic_load_n(&ring->tail, __ATOMIC_ACQUIRE);
    if ((uint32_t)(head - tail) >= GP_IPC_RING_CAPACITY) return false;
    ring->messages[head % GP_IPC_RING_CAPACITY] = *message;
    __atomic_store_n(&ring->head, head + 1U, __ATOMIC_RELEASE);
    return true;
}

static bool ring_receive(GpIpcRing *ring, GpIpcMessage *message)
{
    const uint32_t tail = __atomic_load_n(&ring->tail, __ATOMIC_RELAXED);
    const uint32_t head = __atomic_load_n(&ring->head, __ATOMIC_ACQUIRE);
    if (head == tail) return false;
    *message = ring->messages[tail % GP_IPC_RING_CAPACITY];
    __atomic_store_n(&ring->tail, tail + 1U, __ATOMIC_RELEASE);
    return true;
}

static uint32_t ring_pending(const GpIpcRing *ring)
{
    const uint32_t head = __atomic_load_n(&ring->head, __ATOMIC_ACQUIRE);
    const uint32_t tail = __atomic_load_n(&ring->tail, __ATOMIC_ACQUIRE);
    const uint32_t distance = (uint32_t)(head - tail);
    return distance <= GP_IPC_RING_CAPACITY ? distance : GP_IPC_RING_CAPACITY;
}

bool gp_ipc_m4_send(GpIpcShared *shared, const GpIpcMessage *message)
{
    return gp_ipc_valid(shared) && message_valid(message) &&
           ring_send(&shared->m4_to_m7, message);
}

bool gp_ipc_m7_receive(GpIpcShared *shared, GpIpcMessage *message)
{
    return gp_ipc_valid(shared) && message != NULL &&
           ring_receive(&shared->m4_to_m7, message);
}

bool gp_ipc_m7_send(GpIpcShared *shared, const GpIpcMessage *message)
{
    return gp_ipc_valid(shared) && message_valid(message) &&
           ring_send(&shared->m7_to_m4, message);
}

bool gp_ipc_m4_receive(GpIpcShared *shared, GpIpcMessage *message)
{
    return gp_ipc_valid(shared) && message != NULL &&
           ring_receive(&shared->m7_to_m4, message);
}

uint32_t gp_ipc_m4_to_m7_pending(const GpIpcShared *shared)
{
    return gp_ipc_valid(shared) ? ring_pending(&shared->m4_to_m7) : 0U;
}

uint32_t gp_ipc_m7_to_m4_pending(const GpIpcShared *shared)
{
    return gp_ipc_valid(shared) ? ring_pending(&shared->m7_to_m4) : 0U;
}
