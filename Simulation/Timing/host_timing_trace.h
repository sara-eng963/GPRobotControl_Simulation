#ifndef HOST_TIMING_TRACE_H
#define HOST_TIMING_TRACE_H

/* POSIX-only instrumentation. Never link this module into STM32 firmware.
 * A single RTOS task writes each named ring; a separate background host
 * pthread drains the rings to CSV. The measured code NEVER writes files,
 * blocks, locks a mutex, allocates memory, or waits for the logger.
 */
#include <stdbool.h>
#include <stdint.h>

typedef enum {
    TIMING_SUPERVISOR = 0,
    TIMING_SIM_INPUTS,
    TIMING_HMI,
    TIMING_VALIDATION,
    TIMING_FLASH,
    TIMING_PREFETCH,
    TIMING_CAN_TARGET,
    /* Events/callbacks issued only by PathExecution in the Supervisor task.
     * Each waiting event counts one Supervisor polling opportunity, not a
     * unique missed command or deadline. No periodic cadence assigned. */
    TIMING_PATH_FEEDBACK_WAIT,
    TIMING_PATH_FEEDBACK_LATE,
    TIMING_PATH_PERIOD_WAIT,
    TIMING_PATH_COMMAND,
    TIMING_PATH_SAMPLES_SKIPPED,
    TIMING_PATH_FEEDBACK_AGE_AXIS_1,
    TIMING_PATH_FEEDBACK_AGE_AXIS_2,
    TIMING_PATH_FEEDBACK_AGE_AXIS_3,
    TIMING_PATH_FEEDBACK_AGE_AXIS_4,
    TIMING_PATH_FEEDBACK_AGE_AXIS_5,
    TIMING_PATH_FEEDBACK_AGE_AXIS_6,
    /* F1: explicit 2 ms HOST Supervisor, not a physical deadline. */
    TIMING_SUPERVISOR_2MS,
    /* Source/adapter work. No nominal 2-ms release guarantee. */
    TIMING_GUIDANCE_OBSERVER,
    /* Gate + six RPDO/SYNC dispatch (M7 host owner only). */
    TIMING_GUIDANCE_DISPATCH,
    TIMING_SOURCE_COUNT
} TimingSource;

/* Calling with NULL or "" disables recording. Call before scheduler start.
 * Returns false on invalid/failed output path; caller should fail startup.
 */
bool timing_init(const char *csv_path);

/* Exactly one producer task per source. Instrumented calls are no-ops when
 * HOST_TIMING_CSV is unset. 'success' reports whether a measured operation
 * returned success, not whether the overall robot task succeeded.
 */
void timing_begin(TimingSource source);
void timing_end(TimingSource source, bool success);
/* Nonblocking numeric diagnostic; value is NOT an elapsed duration. */
void timing_value(TimingSource source, uint64_t value);

#endif
