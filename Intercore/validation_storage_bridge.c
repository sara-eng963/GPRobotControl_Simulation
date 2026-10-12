#include "validation_storage_bridge.h"
#include <string.h>

#define GP_ACQUIRE __ATOMIC_ACQUIRE
#define GP_RELEASE __ATOMIC_RELEASE
#define GP_RELAXED __ATOMIC_RELAXED

_Static_assert(sizeof(PvExecutionSample) == 24U,
               "Validation samples must contain six int32 joint targets");
_Static_assert(sizeof(GpValidationStorageMessage) == 32U,
               "Validation ring ABI must remain fixed");
_Static_assert(__atomic_always_lock_free(sizeof(uint32_t), 0),
               "M7/M4 ring requires lock-free 32-bit atomics");

void gp_validation_shared_initialize(GpValidationShared *shared)
{
    if (shared == NULL) return;
    /* No other core/task may read or write while this runs. */
    memset(shared, 0, sizeof(*shared));
    shared->version = GP_VALIDATION_VERSION;
    __atomic_store_n(&shared->magic, GP_VALIDATION_MAGIC, GP_RELEASE);
}

bool gp_validation_shared_valid(const GpValidationShared *shared)
{
    return shared != NULL &&
           __atomic_load_n(&shared->magic, GP_ACQUIRE) == GP_VALIDATION_MAGIC &&
           shared->version == GP_VALIDATION_VERSION;
}

void gp_validation_m7_init(GpValidationM7Client *client, GpValidationShared *shared)
{
    if (client == NULL) return;
    memset(client, 0, sizeof(*client));
    client->shared = shared;
}

bool gp_validation_m4_init(GpValidationM4Worker *worker, GpValidationShared *shared,
                           const PathValidationStorage *backend)
{
    if (worker == NULL || !gp_validation_shared_valid(shared) ||
        backend == NULL || backend->begin == NULL ||
        backend->write_sample == NULL || backend->commit == NULL ||
        backend->abort == NULL || backend->capacity_samples == 0U)
        return false;
    memset(worker, 0, sizeof(*worker));
    worker->shared = shared;
    worker->backend = *backend;
    return true;
}

bool gp_validation_m4_idle(const GpValidationShared *shared)
{
    if (!gp_validation_shared_valid(shared)) return false;
    return __atomic_load_n(&shared->flash_status, GP_ACQUIRE) ==
               GP_VALIDATION_FLASH_IDLE &&
           __atomic_load_n(&shared->cancel_request, GP_ACQUIRE) == 0U &&
           __atomic_load_n(&shared->release_request, GP_ACQUIRE) == 0U &&
           __atomic_load_n(&shared->head, GP_ACQUIRE) ==
               __atomic_load_n(&shared->tail, GP_ACQUIRE);
}

static bool enqueue(GpValidationShared *shared,
                    const GpValidationStorageMessage *message)
{
    const uint32_t head = __atomic_load_n(&shared->head, GP_RELAXED);
    const uint32_t tail = __atomic_load_n(&shared->tail, GP_ACQUIRE);
    if ((uint32_t)(head - tail) >= GP_VALIDATION_QUEUE_CAPACITY)
        return false;
    shared->slots[head % GP_VALIDATION_QUEUE_CAPACITY] = *message;
    __atomic_store_n(&shared->head, head + 1U, GP_RELEASE);
    return true;
}

static bool client_begin(void *ctx)
{
    GpValidationM7Client *client = ctx;
    if (client == NULL || client->active || !gp_validation_m4_idle(client->shared))
        return false;
    GpValidationStorageMessage command = {0};
    command.kind = GP_VALIDATION_OP_BEGIN;
    if (!enqueue(client->shared, &command)) return false;
    client->active = true;
    client->commit_queued = false;
    client->next_index = 0U;
    return true;
}

static PvStorageTryResult client_write(uint32_t sample_index,
                                        const PvExecutionSample *sample, void *ctx)
{
    GpValidationM7Client *client = ctx;
    if (client == NULL || !client->active || client->commit_queued ||
        sample == NULL || sample_index != client->next_index)
        return PV_STORAGE_TRY_FAILED;
    GpValidationShared *shared = client->shared;
    if (!gp_validation_shared_valid(shared) ||
        __atomic_load_n(&shared->cancel_request, GP_ACQUIRE) != 0U ||
        __atomic_load_n(&shared->flash_status, GP_ACQUIRE) ==
             GP_VALIDATION_FLASH_FAILED)
        return PV_STORAGE_TRY_FAILED;
    GpValidationStorageMessage command = {0};
    command.kind = GP_VALIDATION_OP_SAMPLE;
    command.index = sample_index;
    command.sample = *sample;
    if (!enqueue(shared, &command)) return PV_STORAGE_TRY_BUSY;
    ++client->next_index;
    return PV_STORAGE_TRY_ACCEPTED;
}

static PvStorageTryResult client_commit(const ValidatedTrajectory *metadata,
                                         void *ctx)
{
    GpValidationM7Client *client = ctx;
    if (client == NULL || !client->active || metadata == NULL ||
        metadata->sample_count != client->next_index)
        return PV_STORAGE_TRY_FAILED;
    GpValidationShared *shared = client->shared;
    if (!gp_validation_shared_valid(shared)) return PV_STORAGE_TRY_FAILED;
    const uint32_t status = __atomic_load_n(&shared->flash_status, GP_ACQUIRE);
    if (status == GP_VALIDATION_FLASH_FAILED)
        return PV_STORAGE_TRY_FAILED;
    if (!client->commit_queued) {
        const uint32_t head = __atomic_load_n(&shared->head, GP_RELAXED);
        const uint32_t tail = __atomic_load_n(&shared->tail, GP_ACQUIRE);
        if ((uint32_t)(head - tail) >= GP_VALIDATION_QUEUE_CAPACITY)
            return PV_STORAGE_TRY_BUSY;
        shared->commit_metadata = *metadata;
        GpValidationStorageMessage command = {0};
        command.kind = GP_VALIDATION_OP_COMMIT;
        /* Single M7 producer; no other code may enqueue concurrently. */
        if (!enqueue(shared, &command)) return PV_STORAGE_TRY_BUSY;
        client->commit_queued = true;
    }
    if (status != GP_VALIDATION_FLASH_COMMITTED) return PV_STORAGE_TRY_BUSY;
    /* Durable commit acknowledged. Release the flash service for future work. */
    client->active = false;
    __atomic_store_n(&shared->release_request, 1U, GP_RELEASE);
    return PV_STORAGE_TRY_ACCEPTED;
}

static void client_abort(void *ctx)
{
    GpValidationM7Client *client = ctx;
    if (client == NULL || !client->active ||
        !gp_validation_shared_valid(client->shared)) return;
    /* An in-flight NOR operation must finish before M4 honors cancellation. */
    __atomic_store_n(&client->shared->cancel_request, 1U, GP_RELEASE);
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

PathValidationStorage gp_validation_m7_storage(GpValidationM7Client *client,
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

static void fail_storage(GpValidationM4Worker *worker)
{
    if (worker->open) {
        worker->backend.abort(worker->backend.context);
        worker->open = false;
    }
    __atomic_store_n(&worker->shared->flash_status, GP_VALIDATION_FLASH_FAILED,
                     GP_RELEASE);
}

bool gp_validation_m4_step(GpValidationM4Worker *worker)
{
    if (worker == NULL || !gp_validation_shared_valid(worker->shared)) return false;
    GpValidationShared *shared = worker->shared;
    if (__atomic_load_n(&shared->cancel_request, GP_ACQUIRE) != 0U) {
        /* Only M4 touches flash; cancellation never truncates an in-flight IO. */
        if (worker->open) worker->backend.abort(worker->backend.context);
        worker->open = false;
        worker->next_index = 0U;
        const uint32_t head = __atomic_load_n(&shared->head, GP_ACQUIRE);
        __atomic_store_n(&shared->tail, head, GP_RELEASE);
        __atomic_store_n(&shared->flash_status, GP_VALIDATION_FLASH_IDLE,
                         GP_RELEASE);
        __atomic_store_n(&shared->cancel_request, 0U, GP_RELEASE);
        return true;
    }
    if (__atomic_load_n(&shared->release_request, GP_ACQUIRE) != 0U) {
        if (worker->open) return false;
        __atomic_store_n(&shared->flash_status, GP_VALIDATION_FLASH_IDLE,
                         GP_RELEASE);
        __atomic_store_n(&shared->release_request, 0U, GP_RELEASE);
        return true;
    }
    if (__atomic_load_n(&shared->flash_status, GP_ACQUIRE) ==
        GP_VALIDATION_FLASH_FAILED) return false;
    const uint32_t tail = __atomic_load_n(&shared->tail, GP_RELAXED);
    const uint32_t head = __atomic_load_n(&shared->head, GP_ACQUIRE);
    if (head == tail) return true;
    const GpValidationStorageMessage message =
        shared->slots[tail % GP_VALIDATION_QUEUE_CAPACITY];
    bool ok = false;
    switch (message.kind) {
    case GP_VALIDATION_OP_BEGIN:
        if (!worker->open &&
            __atomic_load_n(&shared->flash_status, GP_ACQUIRE) ==
                GP_VALIDATION_FLASH_IDLE) {
            ok = worker->backend.begin(worker->backend.context);
            if (ok) {
                worker->open = true;
                worker->next_index = 0U;
                __atomic_store_n(&shared->committed_samples, 0U, GP_RELEASE);
                __atomic_store_n(&shared->flash_status,
                    GP_VALIDATION_FLASH_WRITING, GP_RELEASE);
            }
        }
        break;
    case GP_VALIDATION_OP_SAMPLE:
        if (worker->open && message.index == worker->next_index &&
            message.index < worker->backend.capacity_samples) {
            ok = worker->backend.write_sample(message.index, &message.sample,
                                              worker->backend.context);
            if (ok) {
                ++worker->next_index;
                __atomic_store_n(&shared->committed_samples,
                                 worker->next_index, GP_RELEASE);
            }
        }
        break;
    case GP_VALIDATION_OP_COMMIT:
        if (worker->open && worker->next_index ==
            shared->commit_metadata.sample_count) {
            ok = worker->backend.commit(&shared->commit_metadata,
                                        worker->backend.context);
            if (ok) {
                worker->open = false;
                __atomic_store_n(&shared->flash_status,
                    GP_VALIDATION_FLASH_COMMITTED, GP_RELEASE);
            }
        }
        break;
    default:
        break;
    }
    /* Consumer releases a slot only after its callback has finished. */
    __atomic_store_n(&shared->tail, tail + 1U, GP_RELEASE);
    if (!ok) fail_storage(worker);
    return ok;
}
