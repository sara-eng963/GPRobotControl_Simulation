#ifndef GP_HOST_TIMING_TRACE_H
#define GP_HOST_TIMING_TRACE_H

/* POSIX-only instrumentation. Never link this module into STM32 firmware.
 * A single RTOS task writes each named ring; a separate background host
 * pthread drains the rings to CSV. The measured code NEVER writes files,
 * blocks, locks a mutex, allocates memory, or waits for the logger.
 */
#include <stdbool.h>
#include <stdint.h>

typedef enum {
    GP_TIMING_SUPERVISOR = 0,
    GP_TIMING_SIM_INPUTS,
    GP_TIMING_HMI,
    GP_TIMING_VALIDATION,
    GP_TIMING_FLASH,
    GP_TIMING_PREFETCH,
    GP_TIMING_CAN_TARGET,
    /* Events/callbacks issued only by PathExecution in the Supervisor task.
     * Each waiting event counts one Supervisor polling opportunity, not a
     * unique missed command or deadline. No periodic cadence assigned. */
    GP_TIMING_PATH_FEEDBACK_WAIT,
    GP_TIMING_PATH_FEEDBACK_LATE,
    GP_TIMING_PATH_PERIOD_WAIT,
    GP_TIMING_PATH_COMMAND,
    GP_TIMING_PATH_SAMPLES_SKIPPED,
    GP_TIMING_PATH_FEEDBACK_AGE_AXIS_1,
    GP_TIMING_PATH_FEEDBACK_AGE_AXIS_2,
    GP_TIMING_PATH_FEEDBACK_AGE_AXIS_3,
    GP_TIMING_PATH_FEEDBACK_AGE_AXIS_4,
    GP_TIMING_PATH_FEEDBACK_AGE_AXIS_5,
    GP_TIMING_PATH_FEEDBACK_AGE_AXIS_6,
    GP_TIMING_SOURCE_COUNT
} GpTimingSource;

/* Calling with NULL or "" disables recording. Call before scheduler start.
 * Returns false on invalid/failed output path; caller should fail startup.
 */
bool gp_timing_init(const char *csv_path);

/* Exactly one producer task per source. Instrumented calls are no-ops when
 * GP_TIMING_CSV is unset. 'success' reports whether a measured operation
 * returned success, not whether the overall robot task succeeded.
 */
void gp_timing_begin(GpTimingSource source);
void gp_timing_end(GpTimingSource source, bool success);
/* Nonblocking numeric diagnostic; value is NOT an elapsed duration. */
void gp_timing_value(GpTimingSource source, uint64_t value);

#endif
