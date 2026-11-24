/* Diagnostic firmware only. Constant-space history, scanned with CPUs halted.
 * Instrumentation changes scheduling costs; sizes of these
 * images must never be substituted for the production ELF accounting. */
#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>

#define MAX_TASKS 32U
typedef struct {
    uint32_t handle, stack, bytes, allocation, switches, active;
    char name[16];
} TaskRecord;
typedef struct {
    uint32_t magic, version, count, overflow;
    uint32_t malloc_failures, creation_failures, malloc_calls, free_calls;
    TaskRecord tasks[MAX_TASKS];
} Trace;
_Static_assert(sizeof(TaskRecord) == 40, "Trace wire layout");
_Static_assert(sizeof(Trace) == 1312, "Trace wire size");
volatile Trace membench_trace = { .magic = 0x4D424E43, .version = 1 };
extern void membench_application_switch(void) __attribute__((weak));

void membench_create(void *tcb, void *stack, unsigned bytes, const char *name, unsigned allocation)
{
    /* Can be called before scheduling; use the port critical nesting policy. */
    taskENTER_CRITICAL();
    uint32_t n = membench_trace.count;
    if (n == MAX_TASKS) { ++membench_trace.overflow; taskEXIT_CRITICAL(); return; }
    volatile TaskRecord *r = &membench_trace.tasks[n];
    r->handle = (uint32_t)tcb;
    r->stack = (uint32_t)stack;
    r->bytes = bytes;
    r->allocation = allocation;
    r->active = 1;
    for (unsigned i = 0; i < 16; ++i) {
        r->name[i] = name[i];
        if (name[i] == 0) break;
    }
    membench_trace.count = n + 1;
    taskEXIT_CRITICAL();
}
void membench_switch(void *tcb)
{
    if (membench_application_switch) membench_application_switch();
    for (uint32_t i = 0; i < membench_trace.count; ++i)
        if (membench_trace.tasks[i].handle == (uint32_t)tcb && membench_trace.tasks[i].active)
            ++membench_trace.tasks[i].switches;
}
void membench_delete(void *tcb)
{
    for (uint32_t i = 0; i < membench_trace.count; ++i)
        if (membench_trace.tasks[i].handle == (uint32_t)tcb)
            membench_trace.tasks[i].active = 0;
}
void membench_malloc(void *p)
{
    ++membench_trace.malloc_calls;
    if (p == 0) ++membench_trace.malloc_failures;
}
void membench_free(void) { ++membench_trace.free_calls; }

BaseType_t __real_xTaskCreate(TaskFunction_t, const char *, const configSTACK_DEPTH_TYPE,
                             void *, UBaseType_t, TaskHandle_t *);
BaseType_t __wrap_xTaskCreate(TaskFunction_t entry, const char *name, const configSTACK_DEPTH_TYPE depth,
                             void *arg, UBaseType_t priority, TaskHandle_t *handle)
{
    BaseType_t result = __real_xTaskCreate(entry, name, depth, arg, priority, handle);
    if (result != pdPASS) ++membench_trace.creation_failures;
    return result;
}
TaskHandle_t __real_xTaskCreateStatic(TaskFunction_t, const char *, const uint32_t, void *,
                                    UBaseType_t, StackType_t *, StaticTask_t *);
TaskHandle_t __wrap_xTaskCreateStatic(TaskFunction_t entry, const char *name, const uint32_t depth,
                                    void *arg, UBaseType_t priority, StackType_t *stack, StaticTask_t *tcb)
{
    TaskHandle_t result = __real_xTaskCreateStatic(entry, name, depth, arg, priority, stack, tcb);
    if (result == 0) ++membench_trace.creation_failures;
    return result;
}

/* Only the explicit negative image calls this. Normal images never inject
 * allocation failures. Both operations must fail without starving tasks. */
void membench_negative(void)
{
    (void)pvPortMalloc(configTOTAL_HEAP_SIZE);
    (void)xTaskCreate((TaskFunction_t)membench_negative, "must-not-exist",
                      configTOTAL_HEAP_SIZE, 0, tskIDLE_PRIORITY, 0);
}
