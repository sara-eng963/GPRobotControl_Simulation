/* Native policy test only. Mock providers exist exclusively in this test.
 * Run the actual service and portable protocol with scheduler calls mocked. */
#include "application_validation_service.h"
#include "sram4_memory_map.h"
#include "FreeRTOS.h"
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <sys/mman.h>

static void (*worker_task)(void *);
static int allocation_failure, task_count;
static jmp_buf end_iteration;
TaskHandle_t xTaskCreateStatic(void (*task)(void *), const char *name,
    uint32_t stack, void *arg, uint32_t priority, StackType_t *buffer,
    StaticTask_t *control)
{
    (void)name; (void)arg; (void)buffer; (void)control;
    assert(stack >= 768U && priority == 1U);
    ++task_count;
    if (allocation_failure) return NULL;
    worker_task = task;
    return (void *)1;
}
int xTaskCreate(void (*task)(void *), const char *name, uint32_t stack,
    void *arg, uint32_t priority, TaskHandle_t *handle)
{
    *handle = xTaskCreateStatic(task, name, stack, arg, priority, NULL, NULL);
    return *handle != NULL ? pdPASS : 0;
}
void vTaskDelay(uint32_t ticks)
{
    assert(ticks == 1U);
    longjmp(end_iteration, 1);
}

#if !defined(TEST_NO_PROVIDERS) && TEST_TICK_HZ == 1000U
static int mode, storage_begin_count;
static bool begin(void *ctx) { (void)ctx; ++storage_begin_count; return true; }
static bool write_sample(uint32_t i, const PvExecutionSample *s, void *ctx)
{ (void)i; (void)s; (void)ctx; return false; }
static bool commit(const ValidatedTrajectory *m, void *ctx)
{ (void)m; (void)ctx; return false; }
static void abort_storage(void *ctx) { (void)ctx; }
static bool read_metadata(ValidatedTrajectory *m, void *ctx)
{ (void)m; (void)ctx; return false; }
static bool read_samples(uint32_t i, TrajectorySample *s, uint32_t n, void *ctx)
{ (void)i; (void)s; (void)n; (void)ctx; return false; }

bool board_m4_supply_validation_backend(PathValidationStorage *out)
{
    *out = (PathValidationStorage){.begin=begin, .write_sample=write_sample,
        .commit=commit, .abort=abort_storage, .capacity_samples=512U};
    if (mode == 1) out->commit = NULL;
    if (mode == 2) out->capacity_samples = 0U;
    return mode != 3;
}
bool board_m4_supply_trajectory_reader(CommittedTrajectoryReader *out)
{
    *out = (CommittedTrajectoryReader){.read_metadata=read_metadata,
        .read_samples=read_samples, .capacity_samples=512U};
    if (mode == 4) out->read_samples = NULL;
    if (mode == 5) out->capacity_samples = 256U;
    return mode != 6;
}
#endif

int main(void)
{
    void *mapped = mmap((void *)(uintptr_t)SRAM4_BASE, SRAM4_SHARED_BYTES,
        PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE,
        -1, 0);
    assert(mapped == (void *)(uintptr_t)SRAM4_BASE);
    ValidationShared *v = (void *)(uintptr_t)SRAM4_VALIDATION_BASE;
    TrajectoryShared *t = (void *)(uintptr_t)SRAM4_TRAJECTORY_BASE;
    validation_shared_initialize(v);
    trajectory_shared_initialize(t);
#if defined(TEST_NO_PROVIDERS) || TEST_TICK_HZ != 1000U
    assert(!application_validation_service_start());
    assert(v->worker_ready == 0U && task_count == 0);
    puts("PASS: missing providers/invalid task period leave validation disabled");
#else
    for (mode = 1; mode <= 6; ++mode) {
        assert(!application_validation_service_start());
        assert(v->worker_ready == 0U && task_count == 0);
    }
    mode = 0;
    v->magic = 0;
    assert(!application_validation_service_start());
    validation_shared_initialize(v);
    t->magic = 0;
    assert(!application_validation_service_start());
    assert(v->worker_ready == 0U && task_count == 0);
    trajectory_shared_initialize(t);
    allocation_failure = 1;
    assert(!application_validation_service_start());
    assert(v->worker_ready == 0U && task_count == 1);
    /* Quiescent test reset after simulated allocation failure. */
    validation_shared_initialize(v);
    allocation_failure = 0;
    assert(application_validation_service_start());
    assert(v->worker_ready == 1U && task_count == 2 && worker_task);
    assert(!application_validation_service_start()); /* no second owner */
    assert(task_count == 2 && v->worker_ready == 1U);
    ValidationM7Client client;
    validation_m7_init(&client, v);
    PathValidationStorage proxy = validation_m7_storage(&client, 512U);
    assert(proxy.begin(proxy.context));
    assert(storage_begin_count == 0); /* M7 only queues the operation */
    if (setjmp(end_iteration) == 0) worker_task(NULL);
    assert(storage_begin_count == 1 && v->tail == v->head);
    puts("PASS: invalid providers, allocation rollback, one owner, M4-only IO");
#endif
    assert(munmap(mapped, SRAM4_SHARED_BYTES) == 0);
    return 0;
}
