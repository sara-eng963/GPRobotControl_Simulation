#include "validation_storage_bridge.h"
#include <string.h>

#define ATOMIC_ACQUIRE __ATOMIC_ACQUIRE
#define ATOMIC_RELEASE __ATOMIC_RELEASE
#define ATOMIC_RELAXED __ATOMIC_RELAXED

_Static_assert(offsetof(ValidationShared, worker_ready) ==
               offsetof(ValidationWireHeader, worker_ready),
               "Validation worker-ready wire offset drift");
_Static_assert(offsetof(ValidationShared, commit_metadata) ==
               sizeof(ValidationWireHeader),
               "Validation wire prefix drift");
_Static_assert(sizeof(PvExecutionSample) == 24U,
               "Validation samples must contain six int32 joint targets");
_Static_assert(sizeof(ValidationStorageMessage) == 32U,
               "Validation ring ABI must remain fixed");
_Static_assert(__atomic_always_lock_free(sizeof(uint32_t), 0),
               "M7/M4 ring requires lock-free 32-bit atomics");

void validation_shared_initialize(ValidationShared *shared)
{
    if (shared == NULL) return;
    /* No other core/task may read or write while this runs. */
    memset(shared, 0, sizeof(*shared));
    shared->version = VALIDATION_VERSION;
    __atomic_store_n(&shared->magic, VALIDATION_MAGIC, ATOMIC_RELEASE);
}

bool validation_shared_valid(const ValidationShared *shared)
{
    return shared != NULL &&
           __atomic_load_n(&shared->magic, ATOMIC_ACQUIRE) == VALIDATION_MAGIC &&
           shared->version == VALIDATION_VERSION;
}

void validation_m7_init(ValidationM7Client *client, ValidationShared *shared)
{
    if (client == NULL) return;
    memset(client, 0, sizeof(*client));
    client->shared = shared;
}

bool validation_m4_init(ValidationM4Worker *worker, ValidationShared *shared,
                           const PathValidationStorage *backend)
{
    if (worker == NULL || !validation_shared_valid(shared) ||
        backend == NULL || backend->begin == NULL ||
        backend->write_sample == NULL || backend->commit == NULL ||
        backend->abort == NULL || backend->capacity_samples == 0U ||
        __atomic_load_n(&shared->worker_ready, ATOMIC_ACQUIRE) != 0U ||
        __atomic_load_n(&shared->flash_status, ATOMIC_ACQUIRE) != VALIDATION_FLASH_IDLE ||
        __atomic_load_n(&shared->head, ATOMIC_ACQUIRE) !=
            __atomic_load_n(&shared->tail, ATOMIC_ACQUIRE) ||
        __atomic_load_n(&shared->cancel_request, ATOMIC_ACQUIRE) != 0U ||
        __atomic_load_n(&shared->release_request, ATOMIC_ACQUIRE) != 0U)
        return false;
    memset(worker, 0, sizeof(*worker));
    worker->shared = shared;
    worker->backend = *backend;
    __atomic_store_n(&shared->worker_ready, 1U, ATOMIC_RELEASE);
    return true;
}

void validation_m4_shutdown(ValidationM4Worker *worker)
{
    if (worker == NULL || !validation_shared_valid(worker->shared)) return;
    ValidationShared *shared = worker->shared;
    /* M4 sole owner calls only after in-flight storage IO has returned. */
    __atomic_store_n(&shared->worker_ready, 0U, ATOMIC_RELEASE);
    if (worker->open) {
        worker->backend.abort(worker->backend.context);
        worker->open = false;
    }
    __atomic_store_n(&shared->flash_status, VALIDATION_FLASH_FAILED, ATOMIC_RELEASE);
}

bool validation_m4_idle(const ValidationShared *shared)
{
    if (!validation_shared_valid(shared)) return false;
    return __atomic_load_n(&shared->worker_ready, ATOMIC_ACQUIRE) == 1U &&
           __atomic_load_n(&shared->playback_status, ATOMIC_ACQUIRE) == PLAYBACK_IDLE &&
           __atomic_load_n(&shared->playback_request_seq, ATOMIC_ACQUIRE) ==
               __atomic_load_n(&shared->playback_ack_seq, ATOMIC_ACQUIRE) &&
           __atomic_load_n(&shared->flash_status, ATOMIC_ACQUIRE) ==
               VALIDATION_FLASH_IDLE &&
           __atomic_load_n(&shared->cancel_request, ATOMIC_ACQUIRE) == 0U &&
           __atomic_load_n(&shared->release_request, ATOMIC_ACQUIRE) == 0U &&
           __atomic_load_n(&shared->head, ATOMIC_ACQUIRE) ==
               __atomic_load_n(&shared->tail, ATOMIC_ACQUIRE);
}

static bool enqueue(ValidationShared *shared,
                    const ValidationStorageMessage *message)
{
    const uint32_t head = __atomic_load_n(&shared->head, ATOMIC_RELAXED);
    const uint32_t tail = __atomic_load_n(&shared->tail, ATOMIC_ACQUIRE);
    if ((uint32_t)(head - tail) >= VALIDATION_QUEUE_CAPACITY)
        return false;
    shared->slots[head % VALIDATION_QUEUE_CAPACITY] = *message;
    __atomic_store_n(&shared->head, head + 1U, ATOMIC_RELEASE);
    return true;
}

static bool client_begin(void *ctx)
{
    ValidationM7Client *client = ctx;
    if (client == NULL || client->active || !validation_m4_idle(client->shared))
        return false;
    ValidationStorageMessage command = {0};
    command.kind = VALIDATION_OP_BEGIN;
    if (!enqueue(client->shared, &command)) return false;
    client->active = true;
    client->commit_queued = false;
    client->next_index = 0U;
    return true;
}

static PvStorageTryResult client_write(uint32_t sample_index,
                                        const PvExecutionSample *sample, void *ctx)
{
    ValidationM7Client *client = ctx;
    if (client == NULL || !client->active || client->commit_queued ||
        sample == NULL || sample_index != client->next_index)
        return PV_STORAGE_TRY_FAILED;
    ValidationShared *shared = client->shared;
    if (!validation_shared_valid(shared) ||
        __atomic_load_n(&shared->cancel_request, ATOMIC_ACQUIRE) != 0U ||
        __atomic_load_n(&shared->worker_ready, ATOMIC_ACQUIRE) != 1U ||
        __atomic_load_n(&shared->flash_status, ATOMIC_ACQUIRE) ==
             VALIDATION_FLASH_FAILED)
        return PV_STORAGE_TRY_FAILED;
    ValidationStorageMessage command = {0};
    command.kind = VALIDATION_OP_SAMPLE;
    command.index = sample_index;
    command.sample = *sample;
    if (!enqueue(shared, &command)) return PV_STORAGE_TRY_BUSY;
    ++client->next_index;
    return PV_STORAGE_TRY_ACCEPTED;
}

static PvStorageTryResult client_commit(const ValidatedTrajectory *metadata,
                                         void *ctx)
{
    ValidationM7Client *client = ctx;
    if (client == NULL || !client->active || metadata == NULL ||
        metadata->sample_count != client->next_index)
        return PV_STORAGE_TRY_FAILED;
    ValidationShared *shared = client->shared;
    if (!validation_shared_valid(shared)) return PV_STORAGE_TRY_FAILED;
    const uint32_t status = __atomic_load_n(&shared->flash_status, ATOMIC_ACQUIRE);
    if (status == VALIDATION_FLASH_FAILED ||
        __atomic_load_n(&shared->worker_ready, ATOMIC_ACQUIRE) != 1U)
        return PV_STORAGE_TRY_FAILED;
    if (!client->commit_queued) {
        const uint32_t head = __atomic_load_n(&shared->head, ATOMIC_RELAXED);
        const uint32_t tail = __atomic_load_n(&shared->tail, ATOMIC_ACQUIRE);
        if ((uint32_t)(head - tail) >= VALIDATION_QUEUE_CAPACITY)
            return PV_STORAGE_TRY_BUSY;
        shared->commit_metadata = *metadata;
        ValidationStorageMessage command = {0};
        command.kind = VALIDATION_OP_COMMIT;
        /* Single M7 producer; no other code may enqueue concurrently. */
        if (!enqueue(shared, &command)) return PV_STORAGE_TRY_BUSY;
        client->commit_queued = true;
    }
    if (status != VALIDATION_FLASH_COMMITTED) return PV_STORAGE_TRY_BUSY;
    /* Durable commit acknowledged. Release the flash service for future work. */
    client->active = false;
    __atomic_store_n(&shared->release_request, 1U, ATOMIC_RELEASE);
    return PV_STORAGE_TRY_ACCEPTED;
}

static void client_abort(void *ctx)
{
    ValidationM7Client *client = ctx;
    if (client == NULL || !client->active ||
        !validation_shared_valid(client->shared)) return;
    /* An in-flight NOR operation must finish before M4 honors cancellation. */
    __atomic_store_n(&client->shared->cancel_request, 1U, ATOMIC_RELEASE);
    client->active = false;
}

static bool unsupported_blocking_write(uint32_t i, const PvExecutionSample *s,
                                        void *ctx)
{
    (void)i; (void)s; (void)ctx;
    return false; /* Fail closed if the validator's async API isn't used. */
}
static bool unsupported_blocking_commit(const ValidatedTrajectory *m, void *ctx)
{
    (void)m; (void)ctx;
    return false;
}

PathValidationStorage validation_m7_storage(ValidationM7Client *client,
                                               uint32_t capacity_samples)
{
    PathValidationStorage storage = {0};
    storage.begin = client_begin;
    storage.write_sample = unsupported_blocking_write;
    storage.commit = unsupported_blocking_commit;
    storage.try_write_sample = client_write;
    storage.try_commit = client_commit;
    storage.abort = client_abort;
    storage.capacity_samples = capacity_samples;
    storage.context = client;
    return storage;
}

static void fail_storage(ValidationM4Worker *worker)
{
    if (worker->open) {
        worker->backend.abort(worker->backend.context);
        worker->open = false;
    }
    __atomic_store_n(&worker->shared->flash_status, VALIDATION_FLASH_FAILED,
                     ATOMIC_RELEASE);
}

bool validation_m4_step(ValidationM4Worker *worker)
{
    if (worker == NULL || !validation_shared_valid(worker->shared) ||
        __atomic_load_n(&worker->shared->worker_ready, ATOMIC_ACQUIRE) != 1U)
        return false;
    ValidationShared *shared = worker->shared;
    if (__atomic_load_n(&shared->cancel_request, ATOMIC_ACQUIRE) != 0U) {
        /* Only M4 touches flash; cancellation never truncates an in-flight IO. */
        if (worker->open) worker->backend.abort(worker->backend.context);
        worker->open = false;
        worker->next_index = 0U;
        const uint32_t head = __atomic_load_n(&shared->head, ATOMIC_ACQUIRE);
        __atomic_store_n(&shared->tail, head, ATOMIC_RELEASE);
        __atomic_store_n(&shared->flash_status, VALIDATION_FLASH_IDLE,
                         ATOMIC_RELEASE);
        __atomic_store_n(&shared->cancel_request, 0U, ATOMIC_RELEASE);
        return true;
    }
    if (__atomic_load_n(&shared->release_request, ATOMIC_ACQUIRE) != 0U) {
        if (worker->open) return false;
        __atomic_store_n(&shared->flash_status, VALIDATION_FLASH_IDLE,
                         ATOMIC_RELEASE);
        __atomic_store_n(&shared->release_request, 0U, ATOMIC_RELEASE);
        return true;
    }
    if (__atomic_load_n(&shared->flash_status, ATOMIC_ACQUIRE) ==
        VALIDATION_FLASH_FAILED) return false;
    const uint32_t tail = __atomic_load_n(&shared->tail, ATOMIC_RELAXED);
    const uint32_t head = __atomic_load_n(&shared->head, ATOMIC_ACQUIRE);
    if (head == tail) return true;
    const ValidationStorageMessage message =
        shared->slots[tail % VALIDATION_QUEUE_CAPACITY];
    bool ok = false;
    switch (message.kind) {
    case VALIDATION_OP_BEGIN:
        if (!worker->open &&
            __atomic_load_n(&shared->flash_status, ATOMIC_ACQUIRE) ==
                VALIDATION_FLASH_IDLE) {
            ok = worker->backend.begin(worker->backend.context);
            if (ok) {
                worker->open = true;
                worker->next_index = 0U;
                __atomic_store_n(&shared->committed_samples, 0U, ATOMIC_RELEASE);
                __atomic_store_n(&shared->flash_status,
                    VALIDATION_FLASH_WRITING, ATOMIC_RELEASE);
            }
        }
        break;
    case VALIDATION_OP_SAMPLE:
        if (worker->open && message.index == worker->next_index &&
            message.index < worker->backend.capacity_samples) {
            ok = worker->backend.write_sample(message.index, &message.sample,
                                              worker->backend.context);
            if (ok) {
                ++worker->next_index;
                __atomic_store_n(&shared->committed_samples,
                                 worker->next_index, ATOMIC_RELEASE);
            }
        }
        break;
    case VALIDATION_OP_COMMIT:
        if (worker->open && worker->next_index ==
            shared->commit_metadata.sample_count) {
            ok = worker->backend.commit(&shared->commit_metadata,
                                        worker->backend.context);
            if (ok) {
                worker->open = false;
                __atomic_store_n(&shared->flash_status,
                    VALIDATION_FLASH_COMMITTED, ATOMIC_RELEASE);
            }
        }
        break;
    default:
        break;
    }
    /* Consumer releases a slot only after its callback has finished. */
    __atomic_store_n(&shared->tail, tail + 1U, ATOMIC_RELEASE);
    if (!ok) fail_storage(worker);
    return ok;
}
