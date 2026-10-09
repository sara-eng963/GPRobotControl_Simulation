#!/usr/bin/env python3
"""Safely wire the prefetch component into the CAN/SIL Kit simulator only.

This script explicitly DOES NOT integrate W25Q hardware or modify supervisor_task.
Refuses to edit if the expected source layout has changed.
"""
from pathlib import Path

root = Path(__file__).resolve().parent.parent
sim_file = root / 'Simulation/supervisor_silkit_main.c'
cmake_file = root / 'supervisor_silkit_sim.cmake'

sim = sim_file.read_text()
cmake = cmake_file.read_text()

if 'static TrajectoryPrefetch validated_prefetch;' in sim:
    raise SystemExit('Already integrated; no files changed')


def replace_once(src, old, new, label):
    n = src.count(old)
    if n != 1:
        raise SystemExit(f'Refusing changes: expected exactly 1 {label}, found {n}. No files changed.')
    return src.replace(old, new, 1)

sim = replace_once(sim, '#include "../ControlCore/Kinematics/control_fk.h"',
'''#include "../ControlCore/Kinematics/control_fk.h"
#include "Storage/trajectory_prefetch.h"''', 'include')
sim = replace_once(sim, 'static RamValidatedStorage validated_storage;',
'''static RamValidatedStorage validated_storage;
static TrajectoryPrefetch validated_prefetch;
static TaskHandle_t prefetch_task_handle;
static bool prefetch_worker_suspended;''', 'global storage')

helpers = '''/* Simulated RAM source: physical STM32 firmware will substitute the W25Q
 * bulk read API. Only the prefill/worker invokes this function. */
static bool ram_prefetch_read_batch(uint32_t first, PvExecutionSample *out,
                                    uint32_t count, void *context)
{
    RamValidatedStorage *s = (RamValidatedStorage *)context;
    if (!s || !out || !s->committed || first > s->metadata.sample_count ||
        count > s->metadata.sample_count - first ||
        count > s->sample_count - first) return false;
    memcpy(out, &s->samples[first], (size_t)count * sizeof(*out));
    return true;
}

/* The producer runs at priority 2, below Supervisor (4). Before resetting
 * stream state we suspend it, so prepare/disarm have exclusive ownership. */
static void suspend_prefetch_worker(void)
{
    if (prefetch_task_handle && !prefetch_worker_suspended) {
        vTaskSuspend(prefetch_task_handle);
        prefetch_worker_suspended = true;
    }
}
static void resume_prefetch_worker(void)
{
    if (prefetch_task_handle && prefetch_worker_suspended) {
        prefetch_worker_suspended = false;
        vTaskResume(prefetch_task_handle);
    }
}

static void prefetch_worker_task(void *arg)
{
    (void)arg;
    for (;;) {
        if (validated_prefetch.armed && !validated_prefetch.read_failed &&
            validated_prefetch.next_flash_index < validated_prefetch.total_samples &&
            trajectory_prefetch_buffered(&validated_prefetch) <= TRAJECTORY_PREFETCH_BATCH)
            (void)trajectory_prefetch_refill(&validated_prefetch);

        /* Replay support: after the last sample was consumed, the worker
         * prepares a fresh 512-sample prefill for a subsequent production run.
         * Never restart after a storage error. */
        if (validated_prefetch.armed && !validated_prefetch.read_failed &&
            validated_prefetch.total_samples > 0U &&
            __atomic_load_n(&validated_prefetch.consumed, __ATOMIC_ACQUIRE)
              == validated_prefetch.total_samples) {
            uint32_t n = validated_prefetch.total_samples;
            if (trajectory_prefetch_prepare(&validated_prefetch,
                    ram_prefetch_read_batch, &validated_storage, n))
                (void)trajectory_prefetch_refill(&validated_prefetch);
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

'''
sim = replace_once(sim, 'static bool ram_storage_begin(', helpers + 'static bool ram_storage_begin(', 'RAM begin helper')
sim = replace_once(sim, '''    storage->sample_count =
        0U;

    storage->writing =''',
'''    suspend_prefetch_worker();
    trajectory_prefetch_disarm(&validated_prefetch);

    storage->sample_count =
        0U;

    storage->writing =''', 'begin suspend')
sim = replace_once(sim, '''    storage->committed =
        true;

    return true;
}''',
'''    storage->committed =
        true;

    /* This initial prefill is from PC RAM during VALIDATION COMMIT, never
     * from QSPI and never during active motion. Worker remains suspended. */
    if (!trajectory_prefetch_prepare(&validated_prefetch,
            ram_prefetch_read_batch, storage, metadata->sample_count) ||
        !trajectory_prefetch_refill(&validated_prefetch) ||
        !trajectory_prefetch_refill(&validated_prefetch)) {
        storage->committed = false;
        trajectory_prefetch_disarm(&validated_prefetch);
        return false;
    }
    resume_prefetch_worker();
    return true;
}''', 'RAM commit')
sim = replace_once(sim, '''static void ram_storage_abort(
    void *context
)
{
    RamValidatedStorage *storage =
        (RamValidatedStorage *)context;

    if (storage == NULL)
    {
        return;
    }
''',
'''static void ram_storage_abort(
    void *context
)
{
    RamValidatedStorage *storage =
        (RamValidatedStorage *)context;

    if (storage == NULL)
    {
        return;
    }
    suspend_prefetch_worker();
    trajectory_prefetch_disarm(&validated_prefetch);
''', 'RAM abort')

sim = replace_once(sim, 'static void configure_robot(void)',
'''/* This callback executes inside PATH_EXECUTION. It accesses SRAM only;
 * all source reads are confined to prefill/worker code above. */
static bool execution_read_prefetched(uint32_t sample_index,
                                     PvExecutionSample *sample, void *ctx)
{
    (void)ctx;
    return trajectory_prefetch_take(&validated_prefetch, sample_index, sample);
}

static void configure_robot(void)''', 'execution read')
sim = replace_once(sim,
'''    configure_robot();configure_state_dependencies();''',
'''    if (!trajectory_prefetch_init(&validated_prefetch)) return false;
    configure_robot();configure_state_dependencies();''', 'start initialization')
sim = replace_once(sim,
'''execution_services=(PathExecutionServices){approach_read_validated_sample,relay,retract_prepare,retract_step,clearance,hold,&validated_storage};''',
'''execution_services=(PathExecutionServices){execution_read_prefetched,relay,retract_prepare,retract_step,clearance,hold,&validated_storage};''', 'execution binding')
sim = replace_once(sim,
'''        xTaskCreate(input_task,"SimInputs",1024,NULL,3,NULL)==pdPASS;''',
'''        xTaskCreate(input_task,"SimInputs",1024,NULL,3,NULL)==pdPASS &&
        xTaskCreate(prefetch_worker_task,"TrajPrefetch",1024,NULL,2,
                    &prefetch_task_handle)==pdPASS;''', 'worker task creation')

cmake = replace_once(cmake,
'''        ${CMAKE_SOURCE_DIR}/Simulation/supervisor_silkit_main.c''',
'''        ${CMAKE_SOURCE_DIR}/Simulation/supervisor_silkit_main.c
        ${CMAKE_SOURCE_DIR}/Simulation/Storage/trajectory_prefetch.c''', 'CMake source')

# Both patches validated: only now write the files.
sim_file.write_text(sim)
cmake_file.write_text(cmake)
print('[OK] Buffered execution reader + low-priority refill task integrated into PC SIL Kit adapter')
print('[OK] APPROACH kept on existing RAM callback; no supervisor/CAN state-machine code modified')
print('[NOTE] This is SIM RAM->RAM prefetch, not an STM32 QSPI hardware integration')
